// Copyright 2026 The libkrun Authors.
// SPDX-License-Identifier: Apache-2.0

use crate::api::AttachContext;
mod config;
mod pci_regs;
use crate::vmm::device_manager::pci::PciBarKind;
use config::{ConfigAccess, write_bars};
use devices::pci::{
    PCI_CONVENTIONAL_CONFIG_SPACE_SIZE, PCI_EXTENDED_CONFIG_SPACE_SIZE, PciAddress, PciBarAccess,
    PciFunction,
};
use devices::virtio::VmmExitObserver;
use kvm_bindings::*;
use kvm_ioctls::VmFd;
use pci_regs::*;
use std::io;
use std::ops::Range;
use std::os::fd::AsRawFd;
use std::path::Path;
use std::sync::{
    Arc,
    atomic::{AtomicI32, Ordering},
};
use std::{array, mem};
use utils::eventfd::EventFd;
use vfio_bindings::bindings::vfio::*;
use vfio_ioctls::{VfioContainer, VfioDevice, VfioDeviceFd};
use vm_memory::{Address, GuestAddress, GuestMemoryBackend, GuestMemoryMmap, GuestMemoryRegion};

const CONFIG_SIZE: usize = PCI_EXTENDED_CONFIG_SPACE_SIZE as usize;
const PAGE_SIZE: u64 = arch::PAGE_SIZE as u64;
const MIN_IO_BAR_SIZE: u64 = !PCI_BASE_ADDRESS_IO_MASK as u64 + 1;
const MIN_MEMORY_BAR_SIZE: u64 = !PCI_BASE_ADDRESS_MEM_MASK as u64 + 1;
const CAP_HEADER_SIZE: usize = PCI_CAP_LIST_NEXT + 1;
const PCI_INTERRUPT_PIN_INTA: u8 = 1;
const GUEST_WRITABLE_COMMAND: u16 = PCI_COMMAND_IO
    | PCI_COMMAND_MEMORY
    | PCI_COMMAND_MASTER
    | PCI_COMMAND_PARITY
    | PCI_COMMAND_SERR
    | PCI_COMMAND_INTX_DISABLE;

fn contains(base: u64, size: u64, address: u64, length: u64) -> bool {
    address >= base
        && address
            .checked_add(length)
            .zip(base.checked_add(size))
            .is_some_and(|(end, limit)| end <= limit)
}
fn overlaps(a: u64, a_size: u64, b: u64, b_size: u64) -> bool {
    a < b + b_size && b < a + a_size
}
fn get16(data: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes(data[offset..offset + 2].try_into().unwrap())
}
fn get32(data: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(data[offset..offset + 4].try_into().unwrap())
}
fn put16(data: &mut [u8], offset: usize, value: u16) {
    data[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}
fn put32(data: &mut [u8], offset: usize, value: u32) {
    data[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn transferred(count: isize, expected: usize, operation: &str) -> io::Result<()> {
    if count < 0 {
        Err(io::Error::last_os_error())
    } else if count as usize != expected {
        Err(io::Error::other(format!("short VFIO {operation}")))
    } else {
        Ok(())
    }
}
fn bar_register(index: usize) -> usize {
    PCI_BASE_ADDRESS_0 + index * size_of::<u32>()
}
fn bar_address_mask(register: u32) -> u32 {
    if register & PCI_BASE_ADDRESS_SPACE_IO != 0 {
        PCI_BASE_ADDRESS_IO_MASK
    } else {
        PCI_BASE_ADDRESS_MEM_MASK
    }
}

#[derive(Clone, Debug)]
struct Bar {
    index: usize,
    flags: u32,
    size: u64,
    address: u64,
    window: Range<u64>,
}
impl Bar {
    fn io(&self) -> bool {
        self.flags & PCI_BASE_ADDRESS_SPACE_IO != 0
    }
    fn wide(&self) -> bool {
        !self.io() && self.flags & PCI_BASE_ADDRESS_MEM_TYPE_MASK == PCI_BASE_ADDRESS_MEM_TYPE_64
    }
    fn decode_enable(&self) -> u16 {
        if self.io() {
            PCI_COMMAND_IO
        } else {
            PCI_COMMAND_MEMORY
        }
    }
    fn mask(&self) -> u64 {
        !(self.size - 1)
    }
    fn kind(&self) -> PciBarKind {
        if self.io() {
            PciBarKind::Io
        } else if self.wide() {
            PciBarKind::Memory64
        } else {
            PciBarKind::Memory32
        }
    }
    fn bounds(&self) -> bool {
        self.address.is_multiple_of(self.size)
            && contains(
                self.window.start,
                self.window.end - self.window.start,
                self.address,
                self.size,
            )
    }
    /// Applies a guest write to BAR register `index`, which holds this BAR's
    /// lower dword or, for a 64-bit BAR, its upper one.
    fn write_address(&mut self, index: usize, value: u32) {
        if index == self.index {
            self.address = (self.address & !u64::from(u32::MAX))
                | u64::from(value & self.mask() as u32 & bar_address_mask(self.flags));
        } else {
            self.address =
                (self.address & u64::from(u32::MAX)) | ((u64::from(value) << 32) & self.mask());
        }
    }
}

fn write_bar_register(config: &mut [u8], bar: &Bar) {
    put32(
        config,
        bar_register(bar.index),
        bar.address as u32 | bar.flags,
    );
    if bar.wide() {
        put32(
            config,
            bar_register(bar.index + 1),
            (bar.address >> 32) as u32,
        );
    }
}

/// Validates the host function's BARs, returning them without addresses.
fn discover(config: &[u8], sizes: &[u64; PCI_STD_NUM_BARS]) -> io::Result<Vec<Bar>> {
    let mut bars = Vec::new();
    let mut index = 0;
    while index < PCI_STD_NUM_BARS {
        let value = get32(config, bar_register(index));
        let size = sizes[index];
        if size == 0 {
            index += 1;
            continue;
        }
        let bar = Bar {
            index,
            flags: value & !bar_address_mask(value),
            size,
            address: 0,
            window: 0..0,
        };
        let min_size = if bar.io() {
            MIN_IO_BAR_SIZE
        } else {
            MIN_MEMORY_BAR_SIZE
        };
        if !size.is_power_of_two()
            || size < min_size
            || (bar.wide() && (index + 1 == PCI_STD_NUM_BARS || sizes[index + 1] != 0))
            || (!bar.io() && !bar.wide() && size > 1 << 32)
            || (!bar.io()
                && !matches!(
                    bar.flags & PCI_BASE_ADDRESS_MEM_TYPE_MASK,
                    PCI_BASE_ADDRESS_MEM_TYPE_32 | PCI_BASE_ADDRESS_MEM_TYPE_64
                ))
        {
            return Err(io::Error::other("invalid PCI BAR size/type"));
        }
        index += if bar.wide() { 2 } else { 1 };
        bars.push(bar);
    }
    Ok(bars)
}

/// A guest may relocate each BAR only within the span allocated to this
/// function for that BAR's kind, so it can never overlap another function's
/// BARs.
fn place(bars: &mut [Bar], addresses: &[u64]) {
    for (bar, address) in bars.iter_mut().zip(addresses) {
        bar.address = *address;
    }
    for kind in [PciBarKind::Io, PciBarKind::Memory32, PciBarKind::Memory64] {
        let span = bars.iter().filter(|b| b.kind() == kind);
        let start = span.clone().map(|b| b.address).min().unwrap_or(0);
        let end = span.map(|b| b.address + b.size).max().unwrap_or(0);
        for bar in bars.iter_mut().filter(|b| b.kind() == kind) {
            bar.window = start..end;
        }
    }
}

fn dma_regions(
    mem: &GuestMemoryMmap,
    excluded: &[(GuestAddress, usize)],
) -> io::Result<Vec<(u64, usize, *mut u8)>> {
    mem.iter()
        .filter(|region| {
            !excluded.iter().any(|(start, size)| {
                overlaps(
                    region.start_addr().raw_value(),
                    region.len(),
                    start.raw_value(),
                    *size as u64,
                )
            })
        })
        .map(|region| {
            let address = region.start_addr().raw_value();
            let size = region.len() as usize;
            let host = region.as_ptr();
            if !address.is_multiple_of(PAGE_SIZE)
                || !region.len().is_multiple_of(PAGE_SIZE)
                || !(host as u64).is_multiple_of(PAGE_SIZE)
            {
                return Err(io::Error::other("VFIO DMA memory must be page aligned"));
            }
            Ok((address, size, host))
        })
        .collect()
}

/// Validates the capability list and relinks it to expose only the
/// capabilities the guest may drive, which are power management and PCI
/// Express.
fn capabilities(config: &mut [u8; CONFIG_SIZE], access: &mut ConfigAccess) -> io::Result<()> {
    let mut next = if get16(config, PCI_STATUS) & PCI_STATUS_CAP_LIST != 0 {
        config[PCI_CAPABILITY_LIST] as usize
    } else {
        0
    };
    let mut seen = [false; PCI_CONVENTIONAL_CONFIG_SPACE_SIZE];
    let mut occupied = [false; PCI_CONVENTIONAL_CONFIG_SPACE_SIZE];
    let mut link = PCI_CAPABILITY_LIST;
    let mut hidden = Vec::new();
    while next != 0 {
        if !(PCI_STD_HEADER_SIZEOF..PCI_CONVENTIONAL_CONFIG_SPACE_SIZE).contains(&next)
            || !next.is_multiple_of(size_of::<u32>())
            || seen[next]
        {
            return Err(io::Error::other("invalid PCI capability chain"));
        }
        let cap = next;
        seen[cap] = true;
        next = config[cap + PCI_CAP_LIST_NEXT] as usize;
        let id = config[cap];
        let control = get16(config, cap + PCI_CAP_FLAGS);
        let size = match id {
            PCI_CAP_ID_PM => PCI_PM_SIZEOF,
            PCI_CAP_ID_EXP => match (
                control & PCI_EXP_FLAGS_VERS,
                (control & PCI_EXP_FLAGS_TYPE) >> 4,
            ) {
                (1, PCI_EXP_TYPE_ENDPOINT | PCI_EXP_TYPE_LEG_END) => PCI_CAP_EXP_ENDPOINT_SIZEOF_V1,
                (1, PCI_EXP_TYPE_RC_END) => PCI_CAP_EXP_RC_ENDPOINT_SIZEOF_V1,
                (2, PCI_EXP_TYPE_ENDPOINT | PCI_EXP_TYPE_LEG_END) => PCI_CAP_EXP_ENDPOINT_SIZEOF_V2,
                (2, PCI_EXP_TYPE_RC_END) => PCI_CAP_EXP_RC_ENDPOINT_SIZEOF_V2,
                _ => {
                    return Err(io::Error::other(
                        "unsupported PCI Express endpoint capability",
                    ));
                }
            },
            PCI_CAP_ID_MSI => match (
                control & PCI_MSI_FLAGS_64BIT != 0,
                control & PCI_MSI_FLAGS_MASKBIT != 0,
            ) {
                (false, false) => 10,
                (true, false) => 14,
                (false, true) => 20,
                (true, true) => 24,
            },
            PCI_CAP_ID_MSIX => 12,
            PCI_CAP_ID_VPD => 8,
            PCI_CAP_ID_AF => 6,
            PCI_CAP_ID_VNDR => {
                let length = config[cap + 2] as usize;
                if length < 3 {
                    return Err(io::Error::other("invalid vendor capability length"));
                }
                length
            }
            _ => CAP_HEADER_SIZE,
        };
        if cap + size > PCI_CONVENTIONAL_CONFIG_SPACE_SIZE
            || occupied[cap..cap + size].iter().any(|b| *b)
        {
            return Err(io::Error::other("truncated or overlapping PCI capability"));
        }
        occupied[cap..cap + size].fill(true);
        access.forwarded[cap..cap + size].fill(0);
        if !matches!(id, PCI_CAP_ID_PM | PCI_CAP_ID_EXP) {
            hidden.push((cap, size));
            continue;
        }
        access.protect(cap..cap + CAP_HEADER_SIZE);
        if id == PCI_CAP_ID_PM {
            if access.pm.replace(cap + PCI_PM_CTRL).is_some() {
                return Err(io::Error::other(
                    "duplicate PCI power-management capability",
                ));
            }
            put16(&mut access.forwarded, cap + PCI_PM_CTRL, u16::MAX);
        }
        config[link] = cap as u8;
        link = cap + PCI_CAP_LIST_NEXT;
        if id == PCI_CAP_ID_EXP {
            let value = get32(config, cap + PCI_EXP_DEVCAP) & !PCI_EXP_DEVCAP_FLR;
            put32(config, cap + PCI_EXP_DEVCAP, value);
            put32(
                &mut access.emulated,
                cap + PCI_EXP_DEVCAP,
                PCI_EXP_DEVCAP_FLR,
            );
            put16(
                &mut access.emulated,
                cap + PCI_EXP_DEVCTL,
                PCI_EXP_DEVCTL_BCR_FLR,
            );
            let devctl = get16(config, cap + PCI_EXP_DEVCTL) & !PCI_EXP_DEVCTL_BCR_FLR;
            put16(config, cap + PCI_EXP_DEVCTL, devctl);
            put16(
                &mut access.forwarded,
                cap + PCI_EXP_DEVCTL,
                !PCI_EXP_DEVCTL_BCR_FLR,
            );
            put16(&mut access.forwarded, cap + PCI_EXP_DEVSTA, u16::MAX);
            if control & PCI_EXP_FLAGS_VERS == 2 {
                access.forwarded[cap + PCI_EXP_DEVCTL2..cap + PCI_EXP_DEVSTA2 + 2].fill(0xff);
            }
        }
    }
    for (cap, size) in hidden {
        // Opaque capabilities reserve up to the next physical capability. Supporting
        // vendor registers in that gap requires a known capability length or a quirk.
        let end = if size == CAP_HEADER_SIZE {
            (cap + size..PCI_CONVENTIONAL_CONFIG_SPACE_SIZE)
                .find(|offset| seen[*offset])
                .unwrap_or(PCI_CONVENTIONAL_CONFIG_SPACE_SIZE)
        } else {
            cap + size
        };
        access.protect(cap..end);
        config[cap..end].fill(0);
    }
    config[link] = 0;
    if config[PCI_CAPABILITY_LIST] == 0 {
        let status = get16(config, PCI_STATUS) & !PCI_STATUS_CAP_LIST;
        put16(config, PCI_STATUS, status);
    }
    Ok(())
}

fn release_all<T>(
    values: &mut Vec<T>,
    mut release: impl FnMut(&mut T) -> io::Result<()>,
) -> io::Result<()> {
    let mut first = None;
    values.retain_mut(|value| match release(value) {
        Ok(()) => false,
        Err(e) => {
            first.get_or_insert(e);
            true
        }
    });
    first.map_or(Ok(()), Err)
}

struct Intx {
    event: EventFd,
    resample: EventFd,
    registered: bool,
}
pub(crate) struct VfioPci {
    vfio: Option<(VfioDevice, Arc<VfioContainer>)>,
    vm: Arc<VmFd>,
    mem: GuestMemoryMmap,
    config: [u8; CONFIG_SIZE],
    bars: Vec<Bar>,
    access: ConfigAccess,
    dma: Vec<(u64, usize)>,
    pub(crate) address: PciAddress,
    exit_evt: EventFd,
    exit_code: Arc<AtomicI32>,
    irq: u32,
    intx: Option<Intx>,
    mode: Option<u32>,
    failed: bool,
}
impl VfioPci {
    pub(crate) fn irq(&self) -> u32 {
        self.irq
    }
    fn device(&self) -> &VfioDevice {
        &self.vfio.as_ref().unwrap().0
    }
    fn container(&self) -> &VfioContainer {
        &self.vfio.as_ref().unwrap().1
    }
    fn region_offset(&self, region: u32, offset: u64, length: usize) -> io::Result<i64> {
        if !contains(
            0,
            self.device().get_region_size(region),
            offset,
            length as u64,
        ) {
            return Err(io::Error::other("VFIO access exceeds region"));
        }
        self.device()
            .get_region_offset(region)
            .checked_add(offset)
            .and_then(|offset| i64::try_from(offset).ok())
            .ok_or_else(|| io::Error::other("VFIO offset overflow"))
    }
    fn io_read(&self, region: u32, offset: u64, data: &mut [u8]) -> io::Result<()> {
        let offset = self.region_offset(region, offset, data.len())?;
        let count = unsafe {
            libc::pread(
                self.device().as_raw_fd(),
                data.as_mut_ptr().cast(),
                data.len(),
                offset,
            )
        };
        transferred(count, data.len(), "read")
    }
    fn io_write(&self, region: u32, offset: u64, data: &[u8]) -> io::Result<()> {
        let offset = self.region_offset(region, offset, data.len())?;
        let count = unsafe {
            libc::pwrite(
                self.device().as_raw_fd(),
                data.as_ptr().cast(),
                data.len(),
                offset,
            )
        };
        transferred(count, data.len(), "write")
    }
    pub(crate) fn new(host_bdf: &str, ctx: &mut AttachContext) -> io::Result<Self> {
        let (address, irq) = ctx.allocate_pci_function().map_err(io::Error::other)?;
        let vm = ctx.vm_fd();
        let mut create = kvm_create_device {
            type_: kvm_device_type_KVM_DEV_TYPE_VFIO,
            ..Default::default()
        };
        let kvm = vm.create_device(&mut create)?;
        // VfioContainer requires TYPE1v2 and VfioDevice checks group viability.
        // No unsafe no-IOMMU fallback is permitted.
        let container = Arc::new(
            VfioContainer::new(Some(Arc::new(VfioDeviceFd::new_from_kvm(kvm))))
                .map_err(io::Error::other)?,
        );
        let path = Path::new("/sys/bus/pci/devices").join(host_bdf);
        let device = VfioDevice::new(&path, container.clone()).map_err(io::Error::other)?;
        let mut device = Self {
            vfio: Some((device, container)),
            vm,
            mem: ctx.guest_memory().clone(),
            config: [0; CONFIG_SIZE],
            bars: Vec::new(),
            access: ConfigAccess::new(),
            dma: Vec::new(),
            address,
            irq,
            exit_evt: ctx.exit_evt()?,
            exit_code: ctx.exit_code().clone(),
            intx: None,
            mode: None,
            failed: false,
        };
        device.initialize(ctx)?;
        Ok(device)
    }

    fn initialize(&mut self, ctx: &mut AttachContext) -> io::Result<()> {
        let config_size = self.device().get_region_size(VFIO_PCI_CONFIG_REGION_INDEX) as usize;
        if !(PCI_CONVENTIONAL_CONFIG_SPACE_SIZE..=CONFIG_SIZE).contains(&config_size) {
            return Err(io::Error::other("invalid VFIO PCI configuration size"));
        }
        let mut config = [0u8; CONFIG_SIZE];
        self.io_read(VFIO_PCI_CONFIG_REGION_INDEX, 0, &mut config[..config_size])?;
        if get16(&config, PCI_VENDOR_ID) == u16::MAX
            || config[PCI_HEADER_TYPE] & PCI_HEADER_TYPE_MASK != PCI_HEADER_TYPE_NORMAL
        {
            return Err(io::Error::other("VFIO requires a PCI endpoint"));
        }
        self.io_write(
            VFIO_PCI_CONFIG_REGION_INDEX,
            PCI_COMMAND as u64,
            &0u16.to_le_bytes(),
        )?;
        let sizes = array::from_fn(|i| self.device().get_region_size(i as u32));
        self.bars = discover(&config, &sizes)?;
        let requests: Vec<_> = self.bars.iter().map(|b| (b.kind(), b.size)).collect();
        let addresses = ctx.allocate_pci_bars(&requests).map_err(io::Error::other)?;
        place(&mut self.bars, &addresses);
        self.config = config;
        put16(&mut self.config, PCI_COMMAND, 0);
        self.config[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_NORMAL;
        self.config[PCI_INTERRUPT_LINE] = self.irq as u8;
        self.config[PCI_INTERRUPT_PIN] = if config[PCI_INTERRUPT_PIN] == 0 {
            0
        } else {
            PCI_INTERRUPT_PIN_INTA
        };
        self.config[PCI_BASE_ADDRESS_0..bar_register(PCI_STD_NUM_BARS)].fill(0);
        self.config[PCI_ROM_ADDRESS..PCI_ROM_ADDRESS + size_of::<u32>()].fill(0);
        // The VM has no IOMMU, SR-IOV or resizable-BAR support, so hide extended capabilities.
        self.config[PCI_CONVENTIONAL_CONFIG_SPACE_SIZE..].fill(0);
        for bar in &self.bars {
            write_bar_register(&mut self.config, bar);
        }
        capabilities(&mut self.config, &mut self.access)?;
        let mut config = self.config;
        self.access.read_controls(&mut config, |start, data| {
            self.io_read(VFIO_PCI_CONFIG_REGION_INDEX, start as u64, data)
        })?;
        self.config = config;
        for (address, size, host) in dma_regions(&self.mem, &ctx.shm_regions())? {
            // SAFETY: the memory clone is retained until DMA has been unmapped.
            unsafe { self.container().vfio_dma_map(address, size, host) }.map_err(|e| {
                io::Error::other(format!("{e} (RLIMIT_MEMLOCK must cover guest RAM)"))
            })?;
            self.dma.push((address, size));
        }
        self.update_interrupts()
    }
    fn bars_valid(&self) -> bool {
        self.bars.iter().all(|bar| bar.bounds())
            && !self.bars.iter().enumerate().any(|(i, a)| {
                self.bars[i + 1..]
                    .iter()
                    .any(|b| a.io() == b.io() && overlaps(a.address, a.size, b.address, b.size))
            })
    }
    /// The VFIO interrupt index the guest has enabled, if any.
    fn interrupt_mode(&self) -> Option<u32> {
        let intx = get16(&self.config, PCI_COMMAND) & PCI_COMMAND_INTX_DISABLE == 0
            && self.config[PCI_INTERRUPT_PIN] != 0
            && self
                .device()
                .get_irq_info(VFIO_PCI_INTX_IRQ_INDEX)
                .is_some_and(|i| i.count != 0);
        intx.then_some(VFIO_PCI_INTX_IRQ_INDEX)
    }
    fn clear_interrupts(&mut self) -> io::Result<()> {
        let mut first = None;
        if let Some(mode) = self.mode {
            match self.device().disable_irq(mode) {
                Ok(()) => self.mode = None,
                Err(e) => first = Some(io::Error::other(e)),
            }
        }
        if let Some(intx) = &self.intx {
            let result = if intx.registered {
                self.vm.unregister_irqfd(&intx.event, self.irq)
            } else {
                Ok(())
            };
            match result {
                Ok(()) => self.intx = None,
                Err(e) => {
                    first.get_or_insert(e.into());
                }
            }
        }
        first.map_or(Ok(()), Err)
    }

    fn update_interrupts(&mut self) -> io::Result<()> {
        let mode = self.interrupt_mode();
        if self.mode != mode {
            self.clear_interrupts()?;
        }
        if mode.is_some() && self.mode.is_none() {
            let event = EventFd::new(libc::EFD_NONBLOCK | libc::EFD_CLOEXEC)?;
            let resample = EventFd::new(libc::EFD_NONBLOCK | libc::EFD_CLOEXEC)?;
            self.intx = Some(Intx {
                event,
                resample,
                registered: false,
            });
            let intx = self.intx.as_ref().unwrap();
            let device = &self.vfio.as_ref().unwrap().0;
            device
                .enable_irq(VFIO_PCI_INTX_IRQ_INDEX, vec![&intx.event])
                .map_err(io::Error::other)?;
            self.mode = Some(VFIO_PCI_INTX_IRQ_INDEX);
            device
                .set_irq_resample_fd(VFIO_PCI_INTX_IRQ_INDEX, vec![&intx.resample])
                .map_err(io::Error::other)?;
            self.vm
                .register_irqfd_with_resample(&intx.event, &intx.resample, self.irq)?;
            self.intx.as_mut().unwrap().registered = true;
            self.device()
                .unmask_irq(VFIO_PCI_INTX_IRQ_INDEX)
                .map_err(io::Error::other)?;
        }
        Ok(())
    }
    pub(crate) fn shutdown(&mut self) -> io::Result<()> {
        if self.vfio.is_none() {
            return Ok(());
        }
        let mut errors = Vec::new();
        if let Err(e) = self.io_write(
            VFIO_PCI_CONFIG_REGION_INDEX,
            PCI_COMMAND as u64,
            &0u16.to_le_bytes(),
        ) {
            errors.push(format!("disable DMA: {e}"));
        }
        if let Err(e) = self.clear_interrupts() {
            errors.push(format!("disable interrupts: {e}"));
        }
        let container = &self.vfio.as_ref().unwrap().1;
        if let Err(e) = release_all(&mut self.dma, |(address, size)| {
            container
                .vfio_dma_unmap(*address, *size)
                .map_err(io::Error::other)
        }) {
            errors.push(format!("unmap DMA: {e}"));
        }
        if errors.is_empty() {
            self.vfio.take();
            Ok(())
        } else {
            Err(io::Error::other(errors.join("; ")))
        }
    }
    fn emulate_config_write(&mut self, start: usize, data: &[u8]) -> io::Result<()> {
        self.access.write(start, data, |offset, bytes| {
            self.io_write(VFIO_PCI_CONFIG_REGION_INDEX, offset as u64, bytes)
        })?;
        let mut config = self.config;
        self.access.emulate(&mut config, start, data);
        write_bars(&mut config, &mut self.bars, start, data);
        if self.access.affects_decode(start, data.len()) {
            self.access.read_controls(&mut config, |offset, bytes| {
                self.io_read(VFIO_PCI_CONFIG_REGION_INDEX, offset as u64, bytes)
            })?;
        } else if config == self.config {
            return Ok(());
        }
        self.config = config;
        self.update_interrupts()
    }
    fn failure(&mut self, result: io::Result<()>) {
        if let Err(error) = result {
            self.failed = true;
            log::error!("VFIO device access: {error}");
            self.exit_code.store(1, Ordering::SeqCst);
            if let Err(error) = self.exit_evt.write(1) {
                log::error!("VFIO exit notification: {error}");
            }
        }
    }
    fn bar(&self, address: u64, length: usize) -> Option<(usize, u64)> {
        if self.failed || self.vfio.is_none() || !self.bars_valid() {
            return None;
        }
        self.bars
            .iter()
            .find(|bar| {
                contains(bar.address, bar.size, address, length as u64)
                    && self.access.decoding(&self.config, bar.decode_enable())
            })
            .map(|bar| (bar.index, address - bar.address))
    }
}
impl Drop for VfioPci {
    fn drop(&mut self) {
        if let Err(error) = self.shutdown() {
            log::error!("VFIO cleanup: {error}; retaining mappings until process exit");
            // DMA must not outlive its backing memory, so fail closed.
            mem::forget(self.mem.clone());
            mem::forget(self.vm.clone());
            mem::forget(self.vfio.take());
        }
    }
}
impl VmmExitObserver for VfioPci {
    fn on_vmm_exit(&mut self) {
        if let Err(error) = self.shutdown() {
            log::error!("VFIO cleanup: {error}");
        }
    }
}
impl PciFunction for VfioPci {
    fn read_config(&mut self, offset: u16, data: &mut [u8]) {
        let start = offset as usize;
        if self.failed || self.vfio.is_none() || start + data.len() > CONFIG_SIZE {
            data.fill(0xff);
            return;
        }
        let result = self
            .access
            .read(&self.config, start, data, |offset, bytes| {
                self.io_read(VFIO_PCI_CONFIG_REGION_INDEX, offset as u64, bytes)
            });
        if result.is_ok() {
            self.config[start..start + data.len()].copy_from_slice(data);
        } else {
            data.fill(0xff);
        }
        self.failure(result);
    }
    fn write_config(&mut self, offset: u16, data: &[u8]) {
        if self.failed || self.vfio.is_none() {
            return;
        }
        let start = offset as usize;
        if start + data.len() > PCI_CONVENTIONAL_CONFIG_SPACE_SIZE || data.is_empty() {
            return;
        }
        let result = self.emulate_config_write(start, data);
        self.failure(result);
    }
    fn read_bar(&mut self, address: u64, data: &mut [u8]) -> PciBarAccess {
        let Some((bar, offset)) = self.bar(address, data.len()) else {
            return PciBarAccess::Unhandled;
        };
        if let Err(error) = self.io_read(bar as u32, offset, data) {
            log::debug!("VFIO BAR {bar} read at {offset:#x}: {error}");
            data.fill(0xff);
        }
        PciBarAccess::Handled
    }
    fn write_bar(&mut self, address: u64, data: &[u8]) -> PciBarAccess {
        let Some((bar, offset)) = self.bar(address, data.len()) else {
            return PciBarAccess::Unhandled;
        };
        if let Err(error) = self.io_write(bar as u32, offset, data) {
            log::debug!("VFIO BAR {bar} write at {offset:#x}: {error}");
        }
        PciBarAccess::Handled
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vmm::device_manager::irq::IrqAllocator;
    use crate::vmm::device_manager::pci::PciHostManager;

    #[test]
    fn vfio_bar_probing_bounds_and_large_window() {
        let mut config = [0u8; 256];
        put32(&mut config, 0x10, 12);
        put32(&mut config, 0x18, 12);
        put32(&mut config, 0x20, 1);
        let sizes = [512 << 20, 0, 2 << 20, 0, 256, 1 << 20];
        let mut bars = discover(&config, &sizes).unwrap();
        let irqs = IrqAllocator::new(arch::IRQ_BASE, arch::IRQ_MAX);
        let mut pci = PciHostManager::new(irqs, Some(32 << 30..64 << 30));
        let requests: Vec<_> = bars.iter().map(|b| (b.kind(), b.size)).collect();
        place(&mut bars, &pci.allocate_bars(&requests).unwrap());
        assert_eq!(bars[0].mask() as u32 | bars[0].flags, 0xe000000c);
        assert_eq!(bars[0].address, 32 << 30);
        assert!(bars.iter().all(|b| b.bounds()));
        assert!(!contains(u64::MAX - 2, 8, u64::MAX - 1, 8));
        assert!(discover(&config, &[123, 0, 0, 0, 0, 0]).is_err());
        let large = discover(&config, &[0, 0, 0, 0, 0, 1 << 30]).unwrap();
        assert!(
            pci.allocate_bars(&[(large[0].kind(), large[0].size)])
                .is_err()
        );
    }
    #[test]
    fn vfio_capability_validation_rejects_cycles_and_overlap() {
        let mut config = [0; CONFIG_SIZE];
        put16(&mut config, PCI_STATUS, PCI_STATUS_CAP_LIST);
        config[PCI_CAPABILITY_LIST] = 0x40;
        config[0x40] = PCI_CAP_ID_PM;
        assert!(capabilities(&mut config.clone(), &mut ConfigAccess::new()).is_ok());
        for next in [0x40, 0x41, 0x20, 0x44] {
            config[0x41] = next;
            config[0x44] = PCI_CAP_ID_PM;
            assert!(capabilities(&mut config.clone(), &mut ConfigAccess::new()).is_err());
        }
    }
    #[test]
    fn vfio_capabilities_hide_msi_and_msix() {
        let mut config = [0; CONFIG_SIZE];
        put16(&mut config, PCI_STATUS, PCI_STATUS_CAP_LIST);
        config[PCI_CAPABILITY_LIST] = 0x40;
        config[0x40] = PCI_CAP_ID_PM;
        config[0x41] = 0x50;
        config[0x50] = PCI_CAP_ID_MSI;
        config[0x51] = 0x60;
        config[0x60] = PCI_CAP_ID_MSIX;
        capabilities(&mut config, &mut ConfigAccess::new()).unwrap();
        assert_eq!(config[PCI_CAPABILITY_LIST], 0x40);
        assert_eq!(config[0x41], 0);
        config[0x40] = PCI_CAP_ID_MSI;
        config[0x41] = 0x50;
        config[0x50] = PCI_CAP_ID_MSIX;
        config[0x51] = 0;
        capabilities(&mut config, &mut ConfigAccess::new()).unwrap();
        assert_eq!(config[PCI_CAPABILITY_LIST], 0);
        assert_eq!(get16(&config, 6) & 0x10, 0);
    }
    #[test]
    fn vfio_pcie_capability_bounds_follow_version_and_endpoint_type() {
        for (flags, size) in [(1, 0x14), (2, 0x34), (0x91, 0x0c), (0x92, 0x2c)] {
            let mut config = [0; CONFIG_SIZE];
            put16(&mut config, PCI_STATUS, PCI_STATUS_CAP_LIST);
            config[PCI_CAPABILITY_LIST] = 0x40;
            config[0x40] = PCI_CAP_ID_EXP;
            config[0x41] = 0x40 + size;
            put16(&mut config, 0x42, flags);
            config[0x40 + size as usize] = PCI_CAP_ID_PM;
            assert!(capabilities(&mut config.clone(), &mut ConfigAccess::new()).is_ok());
            config[0x41] -= 4;
            assert!(capabilities(&mut config, &mut ConfigAccess::new()).is_err());
        }
        for flags in [0, 3, 0x42, 0xf2] {
            let mut config = [0; CONFIG_SIZE];
            put16(&mut config, PCI_STATUS, PCI_STATUS_CAP_LIST);
            config[PCI_CAPABILITY_LIST] = 0x40;
            config[0x40] = PCI_CAP_ID_EXP;
            put16(&mut config, 0x42, flags);
            assert!(capabilities(&mut config, &mut ConfigAccess::new()).is_err());
        }
    }
    #[test]
    fn vfio_dma_includes_anonymous_kernel_and_file_backed_ram_but_excludes_dax() {
        use std::fs::File;
        use std::os::fd::FromRawFd;
        use vm_memory::FileOffset;
        let fd = unsafe {
            libc::memfd_create(
                c"vfio-test".as_ptr(),
                libc::MFD_CLOEXEC | libc::MFD_ALLOW_SEALING,
            )
        };
        assert!(fd >= 0);
        let file = unsafe { File::from_raw_fd(fd) };
        file.set_len(4096).unwrap();
        let mem = GuestMemoryMmap::from_ranges_with_files(&[
            (GuestAddress(0), 4096, None),
            (GuestAddress(0x20_0000), 4096, None),
            (
                GuestAddress(0x40_0000),
                4096,
                Some(FileOffset::new(file, 0)),
            ),
            (GuestAddress(1 << 32), 4096, None),
        ])
        .unwrap();
        let regions = dma_regions(&mem, &[(GuestAddress(1 << 32), 4096)]).unwrap();
        assert_eq!(
            regions.iter().map(|r| r.0).collect::<Vec<_>>(),
            [0, 0x20_0000, 0x40_0000]
        );
    }
    #[test]
    fn vfio_partial_cleanup_retries_only_failed_resources() {
        let mut resources = vec![1, 2, 3];
        let mut attempts = vec![];
        assert!(
            release_all(&mut resources, |resource| {
                attempts.push(*resource);
                if *resource == 2 {
                    Err(io::Error::other("injected unmap failure"))
                } else {
                    Ok(())
                }
            })
            .is_err()
        );
        assert_eq!(attempts, [1, 2, 3]);
        assert_eq!(resources, [2]);
        release_all(&mut resources, |resource| {
            attempts.push(*resource);
            Ok(())
        })
        .unwrap();
        release_all(&mut resources, |_| panic!("already released")).unwrap();
        assert_eq!(attempts, [1, 2, 3, 2]);
    }
}

// Copyright 2026 The libkrun Authors.
// SPDX-License-Identifier: Apache-2.0

use crate::api::AttachContext;
mod config;
mod irq;
mod mapping;
use irq::set_irq_eventfds;
use mapping::{Mapping, mapping_ranges, prepare_bar, set_memory, write_quiesced};
mod pci_regs;
use crate::vmm::device_manager::pci::PciBarKind;
use crate::vmm::gsi_routing::GsiRouting;
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
    Arc, Mutex,
    atomic::{AtomicI32, Ordering},
};
use std::{array, mem, ptr};
use utils::eventfd::EventFd;
use vfio_bindings::bindings::vfio::*;
use vfio_ioctls::{VfioContainer, VfioDevice, VfioDeviceFd, VfioRegionInfoCap};
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

/// Register offsets of an MSI capability, which shift with its 64-bit address
/// and per-vector masking control bits.
struct MsiLayout {
    wide: bool,
    maskable: bool,
    data: usize,
    mask: usize,
    pending: usize,
    end: usize,
}
impl MsiLayout {
    fn new(control: u16, cap: usize) -> Self {
        let wide = control & PCI_MSI_FLAGS_64BIT != 0;
        let maskable = control & PCI_MSI_FLAGS_MASKBIT != 0;
        let (data, mask, pending) = if wide {
            (PCI_MSI_DATA_64, PCI_MSI_MASK_64, PCI_MSI_PENDING_64)
        } else {
            (PCI_MSI_DATA_32, PCI_MSI_MASK_32, PCI_MSI_PENDING_32)
        };
        let end = if maskable {
            pending + size_of::<u32>()
        } else {
            data + size_of::<u16>()
        };
        Self {
            wide,
            maskable,
            data: cap + data,
            mask: cap + mask,
            pending: cap + pending,
            end: cap + end,
        }
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
    fn access(
        &self,
        address: u64,
        length: usize,
        config: &[u8],
        access: &ConfigAccess,
    ) -> Option<u64> {
        (contains(self.address, self.size, address, length as u64)
            && access.decoding(config, self.decode_enable()))
        .then(|| address - self.address)
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Message {
    address: u64,
    data: u32,
    masked: bool,
}
struct Interrupt {
    event: EventFd,
    gsi: u32,
    registered: bool,
}

struct Msix {
    cap: usize,
    bar: usize,
    offset: u64,
    table: Vec<u8>,
    pba_bar: usize,
    pba_offset: u64,
}
impl Msix {
    fn vectors(&self) -> usize {
        self.table.len() / PCI_MSIX_ENTRY_SIZE
    }
    fn pba_size(&self) -> u64 {
        (self.vectors() as u64).div_ceil(u64::BITS.into()) * size_of::<u64>() as u64
    }
    fn overlaps_pba(&self, bar: usize, offset: u64, size: usize) -> bool {
        self.pba_bar == bar && overlaps(self.pba_offset, self.pba_size(), offset, size as u64)
    }
    fn overlaps(&self, bar: usize, offset: u64, size: usize) -> bool {
        self.bar == bar && overlaps(self.offset, self.table.len() as u64, offset, size as u64)
    }
    fn access(&self, bar: usize, offset: u64, size: usize) -> Option<MsixAccess> {
        let index = |base: u64, length: u64| {
            contains(base, length, offset, size as u64).then(|| (offset - base) as usize)
        };
        if self.overlaps_pba(bar, offset, size) {
            Some(
                index(self.pba_offset, self.pba_size())
                    .map_or(MsixAccess::Straddling, MsixAccess::Pba),
            )
        } else if self.overlaps(bar, offset, size) {
            Some(
                index(self.offset, self.table.len() as u64)
                    .map_or(MsixAccess::Straddling, MsixAccess::Table),
            )
        } else {
            None
        }
    }
    fn messages(&self, config: &[u8], vectors: Range<usize>) -> Vec<Message> {
        let masked = get16(config, self.cap + PCI_MSIX_FLAGS) & PCI_MSIX_FLAGS_MASKALL != 0;
        self.table.as_chunks::<PCI_MSIX_ENTRY_SIZE>().0[vectors]
            .iter()
            .map(|e| Message {
                address: u64::from(get32(e, PCI_MSIX_ENTRY_LOWER_ADDR))
                    | (u64::from(get32(e, PCI_MSIX_ENTRY_UPPER_ADDR)) << 32),
                data: get32(e, PCI_MSIX_ENTRY_DATA),
                masked: masked
                    || get32(e, PCI_MSIX_ENTRY_VECTOR_CTRL) & PCI_MSIX_ENTRY_CTRL_MASKBIT != 0,
            })
            .collect()
    }
    fn write_table(&mut self, index: usize, data: &[u8]) -> Range<usize> {
        self.table[index..index + data.len()].copy_from_slice(data);
        let vectors =
            index / PCI_MSIX_ENTRY_SIZE..(index + data.len()).div_ceil(PCI_MSIX_ENTRY_SIZE);
        for entry in &mut self.table.as_chunks_mut::<PCI_MSIX_ENTRY_SIZE>().0[vectors.clone()] {
            let control = get32(entry, PCI_MSIX_ENTRY_VECTOR_CTRL) & PCI_MSIX_ENTRY_CTRL_MASKBIT;
            put32(entry, PCI_MSIX_ENTRY_VECTOR_CTRL, control);
        }
        vectors
    }
}
/// A BAR access that touches the MSI-X table or pending-bit array, with its
/// offset into the structure it lies within.
enum MsixAccess {
    Table(usize),
    Pba(usize),
    Straddling,
}

/// Validates the capability list and relinks it to expose only the
/// capabilities the guest may drive: power management, PCI Express, MSI and MSI-X.
fn capabilities(
    config: &mut [u8; CONFIG_SIZE],
    access: &mut ConfigAccess,
    msi_count: u32,
    bars: &[Bar],
    msix_count: u32,
) -> io::Result<(Option<usize>, Option<Msix>)> {
    let mut next = if get16(config, PCI_STATUS) & PCI_STATUS_CAP_LIST != 0 {
        config[PCI_CAPABILITY_LIST] as usize
    } else {
        0
    };
    let mut seen = [false; PCI_CONVENTIONAL_CONFIG_SPACE_SIZE];
    let mut occupied = [false; PCI_CONVENTIONAL_CONFIG_SPACE_SIZE];
    let mut link = PCI_CAPABILITY_LIST;
    let mut hidden = Vec::new();
    let mut msi = None;
    let mut msix = None;
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
            PCI_CAP_ID_MSI => MsiLayout::new(control, 0).end.next_multiple_of(4),
            PCI_CAP_ID_MSIX => PCI_CAP_MSIX_SIZEOF,
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
        if !matches!(
            id,
            PCI_CAP_ID_PM | PCI_CAP_ID_EXP | PCI_CAP_ID_MSI | PCI_CAP_ID_MSIX
        ) {
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
        } else if id == PCI_CAP_ID_MSI {
            if msi.is_some() || msi_count == 0 {
                return Err(io::Error::other("invalid MSI capability"));
            }
            msi = Some(cap);
            let layout = MsiLayout::new(control, cap);
            access.protect(cap..cap + size);
            put16(
                config,
                cap + PCI_MSI_FLAGS,
                control & !(PCI_MSI_FLAGS_ENABLE | PCI_MSI_FLAGS_QMASK | PCI_MSI_FLAGS_QSIZE),
            );
            put16(
                &mut access.writable,
                cap + PCI_MSI_FLAGS,
                PCI_MSI_FLAGS_ENABLE,
            );
            access.writable[cap + PCI_MSI_ADDRESS_LO..layout.data].fill(0xff);
            access.writable[cap + PCI_MSI_ADDRESS_LO] &= !3;
            put16(&mut access.writable, layout.data, u16::MAX);
            config[layout.data + 2..cap + size].fill(0);
            if layout.maskable {
                put32(&mut access.writable, layout.mask, 1);
            }
        } else if id == PCI_CAP_ID_MSIX {
            let count = (control & PCI_MSIX_FLAGS_QSIZE) as usize + 1;
            if msix.is_some() || msix_count < count as u32 {
                return Err(io::Error::other("invalid MSI-X capability"));
            }
            let location = get32(config, cap + PCI_MSIX_TABLE);
            let pba = get32(config, cap + PCI_MSIX_PBA);
            let mut table = vec![0u8; count * PCI_MSIX_ENTRY_SIZE];
            for entry in table.as_chunks_mut::<PCI_MSIX_ENTRY_SIZE>().0 {
                put32(
                    entry,
                    PCI_MSIX_ENTRY_VECTOR_CTRL,
                    PCI_MSIX_ENTRY_CTRL_MASKBIT,
                );
            }
            let table = Msix {
                cap,
                bar: (location & PCI_MSIX_TABLE_BIR) as usize,
                offset: u64::from(location & PCI_MSIX_TABLE_OFFSET),
                table,
                pba_bar: (pba & PCI_MSIX_PBA_BIR) as usize,
                pba_offset: u64::from(pba & PCI_MSIX_PBA_OFFSET),
            };
            let valid = |index, offset, size| {
                bars.iter()
                    .any(|b| b.index == index && !b.io() && contains(0, b.size, offset, size))
            };
            if !valid(table.bar, table.offset, table.table.len() as u64)
                || !valid(table.pba_bar, table.pba_offset, table.pba_size())
                || table.overlaps_pba(table.bar, table.offset, table.table.len())
            {
                return Err(io::Error::other("invalid MSI-X table or pending-bit array"));
            }
            msix = Some(table);
            put16(
                config,
                cap + PCI_MSIX_FLAGS,
                control & !(PCI_MSIX_FLAGS_ENABLE | PCI_MSIX_FLAGS_MASKALL),
            );
            access.protect(cap..cap + size);
            put16(
                &mut access.writable,
                cap + PCI_MSIX_FLAGS,
                PCI_MSIX_FLAGS_ENABLE | PCI_MSIX_FLAGS_MASKALL,
            );
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
    Ok((msi, msix))
}

/// The interrupt mode the guest enabled and, for MSI or MSI-X, the messages
/// of its vectors.
fn irq_messages(
    config: &[u8],
    msi: Option<usize>,
    msix: Option<&Msix>,
) -> (Option<u32>, Vec<Message>) {
    if let Some(table) =
        msix.filter(|t| get16(config, t.cap + PCI_MSIX_FLAGS) & PCI_MSIX_FLAGS_ENABLE != 0)
    {
        return (
            Some(VFIO_PCI_MSIX_IRQ_INDEX),
            table.messages(config, 0..table.vectors()),
        );
    }
    if let Some(cap) = msi.filter(|&c| get16(config, c + PCI_MSI_FLAGS) & PCI_MSI_FLAGS_ENABLE != 0)
    {
        let layout = MsiLayout::new(get16(config, cap + PCI_MSI_FLAGS), cap);
        let address = u64::from(get32(config, cap + PCI_MSI_ADDRESS_LO))
            | if layout.wide {
                u64::from(get32(config, cap + PCI_MSI_ADDRESS_HI)) << 32
            } else {
                0
            };
        let data = u32::from(get16(config, layout.data));
        let masked = layout.maskable && get32(config, layout.mask) & 1 != 0;
        return (
            Some(VFIO_PCI_MSI_IRQ_INDEX),
            vec![Message {
                address,
                data,
                masked,
            }],
        );
    }
    (
        if get16(config, PCI_COMMAND) & PCI_COMMAND_INTX_DISABLE == 0 {
            Some(VFIO_PCI_INTX_IRQ_INDEX)
        } else {
            None
        },
        Vec::new(),
    )
}

fn event_pending(event: &EventFd) -> io::Result<bool> {
    let mut fd = libc::pollfd {
        fd: event.as_raw_fd(),
        events: libc::POLLIN,
        revents: 0,
    };
    if unsafe { libc::poll(&mut fd, 1, 0) } < 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(fd.revents & libc::POLLIN != 0)
}

/// Clears an event that was pending when its irqfd was registered. KVM injects
/// such an event but leaves it counted, so it would still read as pending and
/// be injected again the next time the vector is unmasked. Events signalled
/// while the irqfd is registered are consumed by KVM itself.
fn consume_event(event: &EventFd) -> io::Result<()> {
    match event.read() {
        Err(e) if e.kind() != io::ErrorKind::WouldBlock => Err(e),
        _ => Ok(()),
    }
}

fn read_msix_pending(
    mode: Option<u32>,
    interrupts: &[Interrupt],
    offset: usize,
    data: &mut [u8],
) -> io::Result<()> {
    data.fill(0);
    if mode == Some(VFIO_PCI_MSIX_IRQ_INDEX) {
        for (byte, vectors) in data
            .iter_mut()
            .zip(interrupts.chunks(u8::BITS as usize).skip(offset))
        {
            for (bit, irq) in vectors.iter().enumerate() {
                if event_pending(&irq.event)? {
                    *byte |= 1 << bit;
                }
            }
        }
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
    mappings: Vec<Mapping>,
    dma: Vec<(u64, usize)>,
    pub(crate) address: PciAddress,
    exit_evt: EventFd,
    exit_code: Arc<AtomicI32>,
    irq: u32,
    intx: Option<Intx>,
    mode: Option<u32>,
    interrupts: Vec<Interrupt>,
    msi: Option<usize>,
    msix: Option<Msix>,
    routing: Arc<Mutex<GsiRouting>>,
    gsis: Range<u32>,
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
    fn region_offset(
        device: &VfioDevice,
        region: u32,
        offset: u64,
        length: usize,
    ) -> io::Result<i64> {
        if !contains(0, device.get_region_size(region), offset, length as u64) {
            return Err(io::Error::other("VFIO access exceeds region"));
        }
        device
            .get_region_offset(region)
            .checked_add(offset)
            .and_then(|offset| i64::try_from(offset).ok())
            .ok_or_else(|| io::Error::other("VFIO offset overflow"))
    }
    fn io_read(&self, region: u32, offset: u64, data: &mut [u8]) -> io::Result<()> {
        let offset = Self::region_offset(self.device(), region, offset, data.len())?;
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
    fn io_write(device: &VfioDevice, region: u32, offset: u64, data: &[u8]) -> io::Result<()> {
        let offset = Self::region_offset(device, region, offset, data.len())?;
        let count =
            unsafe { libc::pwrite(device.as_raw_fd(), data.as_ptr().cast(), data.len(), offset) };
        transferred(count, data.len(), "write")
    }
    pub(crate) fn new(host_bdf: &str, ctx: &mut AttachContext) -> io::Result<Self> {
        let (address, irq) = ctx.allocate_pci_function().map_err(io::Error::other)?;
        let routing = ctx
            .gsi_routing()
            .expect("validate_configuration rejects the split irqchip");
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
            mappings: Vec::new(),
            dma: Vec::new(),
            address,
            irq,
            exit_evt: ctx.exit_evt()?,
            exit_code: ctx.exit_code().clone(),
            intx: None,
            mode: None,
            interrupts: Vec::new(),
            msi: None,
            msix: None,
            routing,
            gsis: 0..0,
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
        Self::io_write(
            self.device(),
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
        let msi_count = self
            .device()
            .get_irq_info(VFIO_PCI_MSI_IRQ_INDEX)
            .map_or(0, |i| i.count);
        let msix_count = self
            .device()
            .get_irq_info(VFIO_PCI_MSIX_IRQ_INDEX)
            .map_or(0, |i| i.count);
        (self.msi, self.msix) = capabilities(
            &mut self.config,
            &mut self.access,
            msi_count,
            &self.bars,
            msix_count,
        )?;
        let vectors = self
            .msix
            .as_ref()
            .map_or(0, |msix| msix.vectors() as u32)
            .max(u32::from(self.msi.is_some()));
        self.gsis = self.routing.lock().unwrap().allocate(vectors)?;
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
        self.map_bars(ctx)?;
        self.update_interrupts()
    }
    fn map_bars(&mut self, ctx: &mut AttachContext) -> io::Result<()> {
        for bar in &self.bars {
            if bar.io()
                || self.device().get_region_flags(bar.index as u32) & VFIO_REGION_INFO_FLAG_MMAP
                    == 0
            {
                continue;
            }
            let caps = self.device().get_region_caps(bar.index as u32);
            let sparse = caps.iter().find_map(|cap| {
                if let VfioRegionInfoCap::SparseMmap(sparse) = cap {
                    Some(&sparse.areas)
                } else {
                    None
                }
            });
            let areas = sparse.map_or_else(
                || {
                    if bar.size < PAGE_SIZE {
                        vec![]
                    } else {
                        vec![(0, bar.size & !(PAGE_SIZE - 1))]
                    }
                },
                |areas| areas.iter().map(|a| (a.offset, a.size)).collect(),
            );
            let pieces = mapping_ranges(bar, &areas, self.msix.as_ref())?;
            let mappings = prepare_bar(
                bar.index,
                &pieces,
                |count| ctx.reserve_memory_slots(count).map_err(io::Error::other),
                |start, size| {
                    let offset =
                        Self::region_offset(self.device(), bar.index as u32, start, size as usize)?;
                    let address = unsafe {
                        libc::mmap(
                            ptr::null_mut(),
                            size as usize,
                            libc::PROT_READ | libc::PROT_WRITE,
                            libc::MAP_SHARED,
                            self.device().as_raw_fd(),
                            offset,
                        )
                    };
                    if address == libc::MAP_FAILED {
                        Err(io::Error::last_os_error())
                    } else {
                        Ok(address as usize)
                    }
                },
            );
            self.mappings.extend(mappings);
        }
        Ok(())
    }
    fn update_mappings(&mut self) -> io::Result<()> {
        let enabled = self.access.decoding(&self.config, PCI_COMMAND_MEMORY) && self.bars_valid();
        mapping::update(&mut self.mappings, &self.bars, enabled, |region| {
            set_memory(&self.vm, region)
        })
    }
    fn bars_valid(&self) -> bool {
        self.bars.iter().all(|bar| bar.bounds())
            && !self.bars.iter().enumerate().any(|(i, a)| {
                self.bars[i + 1..]
                    .iter()
                    .any(|b| a.io() == b.io() && overlaps(a.address, a.size, b.address, b.size))
            })
    }
    fn irq_messages(&self) -> (Option<u32>, Vec<Message>) {
        let (mode, messages) = irq_messages(&self.config, self.msi, self.msix.as_ref());
        let intx = self.config[PCI_INTERRUPT_PIN] != 0
            && self
                .device()
                .get_irq_info(VFIO_PCI_INTX_IRQ_INDEX)
                .is_some_and(|i| i.count != 0);
        (
            mode.filter(|mode| *mode != VFIO_PCI_INTX_IRQ_INDEX || intx),
            messages,
        )
    }
    fn clear_interrupts(&mut self) -> io::Result<()> {
        let mut first = None;
        if let Some(mode) = self.mode {
            match self.device().disable_irq(mode) {
                Ok(()) => self.mode = None,
                Err(e) => first = Some(io::Error::other(e)),
            }
        }
        if let Err(e) = release_all(&mut self.interrupts, |irq| {
            if irq.registered {
                self.vm.unregister_irqfd(&irq.event, irq.gsi)?;
            }
            Ok(())
        }) {
            first.get_or_insert(e);
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
        let (mode, messages) = self.irq_messages();
        if self.mode != mode {
            self.clear_interrupts()?;
        }
        match mode {
            Some(VFIO_PCI_INTX_IRQ_INDEX) if self.mode.is_none() => {
                let event = EventFd::new(libc::EFD_NONBLOCK | libc::EFD_CLOEXEC)?;
                let resample = EventFd::new(libc::EFD_NONBLOCK | libc::EFD_CLOEXEC)?;
                self.intx = Some(Intx {
                    event,
                    resample,
                    registered: false,
                });
                let intx = self.intx.as_ref().unwrap();
                let device = &self.vfio.as_ref().unwrap().0;
                set_irq_eventfds(device, VFIO_PCI_INTX_IRQ_INDEX, &[intx.event.as_raw_fd()])?;
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
            Some(index) if self.mode.is_none() => {
                for i in 0..messages.len() {
                    self.interrupts.push(Interrupt {
                        event: EventFd::new(libc::EFD_NONBLOCK | libc::EFD_CLOEXEC)?,
                        gsi: self.gsis.start + i as u32,
                        registered: false,
                    });
                }
                let device = &self.vfio.as_ref().unwrap().0;
                set_irq_eventfds(
                    device,
                    index,
                    &self
                        .interrupts
                        .iter()
                        .map(|i| i.event.as_raw_fd())
                        .collect::<Vec<_>>(),
                )?;
                self.mode = Some(index);
            }
            _ => (),
        }
        self.route_vectors(0..self.gsis.len(), &messages)
    }
    /// `messages` starts at the first of `vectors`. Vectors without an unmasked
    /// message lose their route.
    fn route_vectors(&mut self, vectors: Range<usize>, messages: &[Message]) -> io::Result<()> {
        let unmasked = |vector: usize| messages.get(vector - vectors.start).filter(|m| !m.masked);
        let active = vectors.start..vectors.end.min(self.interrupts.len());
        // Keep eventfds across masking and message changes so that masked
        // interrupts remain pending and KVM injects them when the irqfd is
        // registered again.
        for (vector, irq) in active.clone().zip(&mut self.interrupts[active.clone()]) {
            if irq.registered && unmasked(vector).is_none() {
                self.vm.unregister_irqfd(&irq.event, irq.gsi)?;
                irq.registered = false;
            }
        }
        let base = self.gsis.start;
        self.routing
            .lock()
            .unwrap()
            .update(vectors.clone().map(|vector| {
                let message = unmasked(vector).map(|m| (m.address, m.data));
                (base + vector as u32, message)
            }))?;
        for (vector, irq) in active.clone().zip(&mut self.interrupts[active]) {
            if !irq.registered && unmasked(vector).is_some() {
                self.vm.register_irqfd(&irq.event, irq.gsi)?;
                irq.registered = true;
                consume_event(&irq.event)?;
            }
        }
        Ok(())
    }
    pub(crate) fn shutdown(&mut self) -> io::Result<()> {
        if self.vfio.is_none() {
            return Ok(());
        }
        let mut errors = Vec::new();
        // The host can revoke BAR backing when command bits are cleared.
        if let Err(e) = release_all(&mut self.mappings, |mapping| mapping.unmap(&self.vm)) {
            errors.push(format!("unmap BAR: {e}"));
        }
        if let Err(e) = Self::io_write(
            self.device(),
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
        let device = &self.vfio.as_ref().unwrap().0;
        let decode = self.access.affects_decode(start, data.len());
        // Command and PM writes can be filtered by VFIO. Quiesce first and
        // restore slots only after a successful write and backend readback.
        write_quiesced(
            &mut self.mappings,
            decode,
            |region| set_memory(&self.vm, region),
            || {
                self.access.write(start, data, |offset, bytes| {
                    Self::io_write(device, VFIO_PCI_CONFIG_REGION_INDEX, offset as u64, bytes)
                })
            },
        )?;
        let mut config = self.config;
        self.access.emulate(&mut config, start, data);
        write_bars(&mut config, &mut self.bars, start, data);
        if decode {
            self.access.read_controls(&mut config, |offset, bytes| {
                self.io_read(VFIO_PCI_CONFIG_REGION_INDEX, offset as u64, bytes)
            })?;
        } else if config == self.config {
            return Ok(());
        }
        self.config = config;
        self.update_mappings()?;
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
        self.bars.iter().find_map(|bar| {
            bar.access(address, length, &self.config, &self.access)
                .map(|offset| (bar.index, offset))
        })
    }
}
impl Drop for VfioPci {
    fn drop(&mut self) {
        if let Err(error) = self.shutdown() {
            log::error!("VFIO cleanup: {error}; retaining mappings until process exit");
            // Neither DMA nor a KVM slot may outlive its backing, so fail closed.
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
        if let Some(pending) = self
            .msi
            .map(|cap| MsiLayout::new(get16(&self.config, cap + PCI_MSI_FLAGS), cap))
            .filter(|msi| {
                msi.maskable
                    && overlaps(
                        msi.pending as u64,
                        size_of::<u32>() as u64,
                        start as u64,
                        data.len() as u64,
                    )
            })
            .map(|msi| msi.pending)
        {
            let result = self
                .interrupts
                .first()
                .filter(|_| self.mode == Some(VFIO_PCI_MSI_IRQ_INDEX))
                .map_or(Ok(false), |irq| event_pending(&irq.event));
            match result {
                Ok(value) => put32(&mut self.config, pending, u32::from(value)),
                Err(e) => {
                    self.failure(Err(e));
                    data.fill(0xff);
                    return;
                }
            }
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
        let result = if let Some(msix) = &self.msix
            && let Some(access) = msix.access(bar, offset, data.len())
        {
            match access {
                MsixAccess::Table(index) => {
                    data.copy_from_slice(&msix.table[index..index + data.len()]);
                    Ok(())
                }
                MsixAccess::Pba(index) => {
                    read_msix_pending(self.mode, &self.interrupts, index, data)
                }
                MsixAccess::Straddling => {
                    data.fill(0xff);
                    Ok(())
                }
            }
        } else {
            if let Err(error) = self.io_read(bar as u32, offset, data) {
                log::debug!("VFIO BAR {bar} read at {offset:#x}: {error}");
                data.fill(0xff);
            }
            Ok(())
        };
        if result.is_err() {
            data.fill(0xff);
        }
        self.failure(result);
        PciBarAccess::Handled
    }
    fn write_bar(&mut self, address: u64, data: &[u8]) -> PciBarAccess {
        let Some((bar, offset)) = self.bar(address, data.len()) else {
            return PciBarAccess::Unhandled;
        };
        let result = if let Some(msix) = &mut self.msix
            && let Some(access) = msix.access(bar, offset, data.len())
        {
            if let MsixAccess::Table(index) = access {
                let vectors = msix.write_table(index, data);
                if self.mode == Some(VFIO_PCI_MSIX_IRQ_INDEX) {
                    let messages = msix.messages(&self.config, vectors.clone());
                    self.route_vectors(vectors, &messages)
                } else {
                    Ok(())
                }
            } else {
                Ok(())
            }
        } else {
            if let Err(error) = Self::io_write(self.device(), bar as u32, offset, data) {
                log::debug!("VFIO BAR {bar} write at {offset:#x}: {error}");
            }
            Ok(())
        };
        self.failure(result);
        PciBarAccess::Handled
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vmm::device_manager::irq::IrqAllocator;
    use crate::vmm::device_manager::pci::PciHostManager;

    pub(super) fn test_bar() -> Bar {
        Bar {
            index: 0,
            flags: 0,
            size: 0x8000,
            address: 0xe010_0000,
            window: 0xe010_0000..0xe011_0000,
        }
    }
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
        assert!(capabilities(&mut config.clone(), &mut ConfigAccess::new(), 0, &[], 0).is_ok());
        for next in [0x40, 0x41, 0x20, 0x44] {
            config[0x41] = next;
            config[0x44] = PCI_CAP_ID_PM;
            assert!(
                capabilities(&mut config.clone(), &mut ConfigAccess::new(), 0, &[], 0).is_err()
            );
        }
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
            assert!(capabilities(&mut config.clone(), &mut ConfigAccess::new(), 0, &[], 0).is_ok());
            config[0x41] -= 4;
            assert!(capabilities(&mut config, &mut ConfigAccess::new(), 0, &[], 0).is_err());
        }
        for flags in [0, 3, 0x42, 0xf2] {
            let mut config = [0; CONFIG_SIZE];
            put16(&mut config, PCI_STATUS, PCI_STATUS_CAP_LIST);
            config[PCI_CAPABILITY_LIST] = 0x40;
            config[0x40] = PCI_CAP_ID_EXP;
            put16(&mut config, 0x42, flags);
            assert!(capabilities(&mut config, &mut ConfigAccess::new(), 0, &[], 0).is_err());
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

    #[test]
    fn vfio_msi_capability_requires_a_host_vector_and_offers_one() {
        let mut config = [0; CONFIG_SIZE];
        put16(&mut config, PCI_STATUS, PCI_STATUS_CAP_LIST);
        config[PCI_CAPABILITY_LIST] = 0x40;
        config[0x40] = PCI_CAP_ID_MSI;
        put16(&mut config, 0x42, 0x18e);
        assert!(capabilities(&mut config.clone(), &mut ConfigAccess::new(), 0, &[], 0).is_err());
        let mut exposed = config;
        assert_eq!(
            capabilities(&mut exposed, &mut ConfigAccess::new(), 1, &[], 0)
                .unwrap()
                .0,
            Some(0x40)
        );
        assert_eq!(get16(&exposed, 0x42), 0x180);
        config[0x41] = 0x54;
        config[0x54] = PCI_CAP_ID_PM;
        assert!(capabilities(&mut config, &mut ConfigAccess::new(), 1, &[], 0).is_err());
    }
    #[test]
    fn vfio_msi_mask_selection_and_pending_event() {
        let mut config = [0; CONFIG_SIZE];
        put16(&mut config, 0x42, 0x181);
        put32(&mut config, 0x44, 0xfee0_0000);
        let (mode, messages) = irq_messages(&config, Some(0x40), None);
        assert_eq!(mode, Some(VFIO_PCI_MSI_IRQ_INDEX));
        assert!(!messages[0].masked);
        put32(&mut config, 0x50, 1);
        assert!(irq_messages(&config, Some(0x40), None).1[0].masked);
        put16(&mut config, 0x42, 0x180);
        assert_eq!(
            irq_messages(&config, Some(0x40), None),
            (Some(VFIO_PCI_INTX_IRQ_INDEX), vec![])
        );
        let event = EventFd::new(libc::EFD_NONBLOCK).unwrap();
        event.write(1).unwrap();
        assert!(event_pending(&event).unwrap());
        assert!(event_pending(&event).unwrap());
        assert_eq!(event.read().unwrap(), 1);
        assert!(!event_pending(&event).unwrap());
        event.write(1).unwrap();
        consume_event(&event).unwrap();
        assert!(!event_pending(&event).unwrap());
        consume_event(&event).unwrap();
    }
    #[test]
    fn vfio_msi_spanning_accesses_match_byte_word_and_dword_writes() {
        for flags in [
            0,
            PCI_MSI_FLAGS_64BIT,
            PCI_MSI_FLAGS_MASKBIT,
            PCI_MSI_FLAGS_64BIT | PCI_MSI_FLAGS_MASKBIT,
        ] {
            let layout = MsiLayout::new(flags, 0x40);
            for width in [1, 2, 4] {
                let mut config = [0; CONFIG_SIZE];
                put16(&mut config, PCI_STATUS, PCI_STATUS_CAP_LIST);
                config[PCI_CAPABILITY_LIST] = 0x40;
                config[0x40] = PCI_CAP_ID_MSI;
                config[0x41] = 0x60;
                config[0x60] = PCI_CAP_ID_PM;
                put16(&mut config, 0x42, flags | PCI_MSI_FLAGS_QMASK);
                let mut access = ConfigAccess::new();
                capabilities(&mut config, &mut access, 1, &[], 0).unwrap();
                let old = config;
                for (offset, bytes) in [(0x40, [0xff; 4]), (layout.data, [0x34, 0x12, 0xff, 0xff])]
                {
                    for (i, data) in bytes.chunks(width).enumerate() {
                        let start = offset + i * width;
                        access
                            .write(start, data, |_, _| panic!("MSI must stay emulated"))
                            .unwrap();
                        access.emulate(&mut config, start, data);
                    }
                }
                assert_eq!(&config[0x40..0x42], &old[0x40..0x42]);
                assert_eq!(get16(&config, 0x42), flags | PCI_MSI_FLAGS_ENABLE);
                assert_eq!(get16(&config, layout.data), 0x1234);
                assert_eq!(&config[layout.data + 2..layout.data + 4], &[0, 0]);
                if layout.maskable {
                    access.emulate(&mut config, layout.mask, &[0xff; 8]);
                    assert_eq!(get32(&config, layout.mask), 1);
                    assert_eq!(get32(&config, layout.pending), 0);
                }
                assert_eq!(&config[0x60..0x68], &old[0x60..0x68]);
                let mut data = vec![0; layout.end - 0x40];
                access
                    .read(&config, 0x40, &mut data, |_, _| {
                        panic!("MSI must stay emulated")
                    })
                    .unwrap();
                assert_eq!(data, config[0x40..layout.end]);
            }
        }
    }
    fn test_msix() -> Msix {
        Msix {
            cap: 0x60,
            bar: 0,
            offset: 0x1ff0,
            table: vec![0; 64],
            pba_bar: 0,
            pba_offset: 0x4000,
        }
    }
    #[test]
    fn vfio_msix_capability_rejects_bad_table_and_pending_bit_array() {
        let mut config = [0; CONFIG_SIZE];
        put16(&mut config, PCI_STATUS, PCI_STATUS_CAP_LIST);
        config[PCI_CAPABILITY_LIST] = 0x40;
        config[0x40] = PCI_CAP_ID_MSIX;
        put32(&mut config, 0x44, 0x1000);
        put32(&mut config, 0x48, 0x4000);
        put16(&mut config, 0x42, 3);
        let (_, msix) = capabilities(
            &mut config.clone(),
            &mut ConfigAccess::new(),
            0,
            &[test_bar()],
            4,
        )
        .unwrap();
        assert_eq!(msix.unwrap().table.len(), 64);
        assert!(
            capabilities(
                &mut config.clone(),
                &mut ConfigAccess::new(),
                0,
                &[test_bar()],
                3
            )
            .is_err()
        );
        for location in [1, 0x8000, 0x7ff8] {
            put32(&mut config, 0x44, location);
            assert!(
                capabilities(
                    &mut config.clone(),
                    &mut ConfigAccess::new(),
                    0,
                    &[test_bar()],
                    4
                )
                .is_err()
            );
        }
        put32(&mut config, 0x44, 0x4000);
        assert!(
            capabilities(
                &mut config.clone(),
                &mut ConfigAccess::new(),
                0,
                &[test_bar()],
                4
            )
            .is_err()
        );
        config[PCI_CAPABILITY_LIST] = 0xfc;
        config[0xfc] = PCI_CAP_ID_MSIX;
        assert!(capabilities(&mut config, &mut ConfigAccess::new(), 0, &[test_bar()], 4).is_err());
    }
    #[test]
    fn vfio_msix_partial_table_accesses_stay_trapped() {
        let table = Msix {
            cap: 0x40,
            bar: 5,
            offset: 0x1000,
            table: vec![0; 64],
            pba_bar: 5,
            pba_offset: 0x2000,
        };
        assert!(matches!(
            table.access(5, 0x1004, 4),
            Some(MsixAccess::Table(4))
        ));
        assert!(matches!(
            table.access(5, 0x2004, 4),
            Some(MsixAccess::Pba(4))
        ));
        for offset in [0xffc, 0x103c, 0x1ffc, 0x2004] {
            assert!(matches!(
                table.access(5, offset, 8),
                Some(MsixAccess::Straddling)
            ));
        }
        assert!(table.access(5, 0x1040, 8).is_none());
        assert!(table.access(5, 0x2008, 8).is_none());
        assert!(table.access(0, 0x1000, 8).is_none());
    }
    #[test]
    fn vfio_msix_mask_selection() {
        let mut config = [0; CONFIG_SIZE];
        put16(&mut config, 0x42, 0x181);
        let mut msix = test_msix();
        assert_eq!(
            irq_messages(&config, Some(0x40), Some(&msix)).0,
            Some(VFIO_PCI_MSI_IRQ_INDEX)
        );
        put16(&mut config, msix.cap + 2, 0x8000);
        put32(&mut msix.table, 12, 1);
        let (mode, messages) = irq_messages(&config, Some(0x40), Some(&msix));
        assert_eq!(mode, Some(VFIO_PCI_MSIX_IRQ_INDEX));
        assert!(messages[0].masked);
        assert!(!messages[1].masked);
        put16(&mut config, msix.cap + 2, 0xc000);
        assert!(
            irq_messages(&config, Some(0x40), Some(&msix))
                .1
                .iter()
                .all(|m| m.masked)
        );
    }
    #[test]
    fn vfio_msix_table_writes_reevaluate_only_touched_vectors() {
        let mut msix = test_msix();
        let data = PCI_MSIX_ENTRY_SIZE + PCI_MSIX_ENTRY_DATA;
        assert_eq!(msix.write_table(data, &0x45u32.to_le_bytes()), 1..2);
        assert_eq!(msix.write_table(PCI_MSIX_ENTRY_SIZE - 4, &[0; 8]), 0..2);
        assert_eq!(
            msix.messages(&[0; CONFIG_SIZE], 1..2),
            [Message {
                address: 0,
                data: 0x45,
                masked: false,
            }]
        );
    }
    #[test]
    fn vfio_msix_pending_reads_preserve_events_and_exclude_msi() {
        let irq = Interrupt {
            event: EventFd::new(libc::EFD_NONBLOCK).unwrap(),
            gsi: 24,
            registered: false,
        };
        irq.event.write(1).unwrap();
        let interrupts = [irq];
        let mut data = [0xff; 8];
        read_msix_pending(Some(VFIO_PCI_MSI_IRQ_INDEX), &interrupts, 0, &mut data).unwrap();
        assert_eq!(data, [0; 8]);
        for _ in 0..2 {
            read_msix_pending(Some(VFIO_PCI_MSIX_IRQ_INDEX), &interrupts, 0, &mut data).unwrap();
            assert_eq!(data, [1, 0, 0, 0, 0, 0, 0, 0]);
        }
        read_msix_pending(Some(VFIO_PCI_MSIX_IRQ_INDEX), &interrupts, 1, &mut data).unwrap();
        assert_eq!(data, [0; 8]);
        assert_eq!(interrupts[0].event.read().unwrap(), 1);
    }
    #[test]
    fn vfio_msix_capability_dword_writes_preserve_header_and_table_locations() {
        for width in [1, 2, 4] {
            let mut config = [0; CONFIG_SIZE];
            put16(&mut config, PCI_STATUS, PCI_STATUS_CAP_LIST);
            config[PCI_CAPABILITY_LIST] = 0x40;
            config[0x40] = PCI_CAP_ID_MSIX;
            config[0x41] = 0x50;
            config[0x50] = PCI_CAP_ID_PM;
            put16(&mut config, 0x42, 3);
            put32(&mut config, 0x44, 0x1000);
            put32(&mut config, 0x48, 0x4000);
            let mut access = ConfigAccess::new();
            capabilities(&mut config, &mut access, 0, &[test_bar()], 4).unwrap();
            let old = config;
            for (i, bytes) in [0xff; 12].chunks(width).enumerate() {
                let start = 0x40 + i * width;
                access
                    .write(start, bytes, |_, _| panic!("MSI-X is emulated"))
                    .unwrap();
                access.emulate(&mut config, start, bytes);
            }
            assert_eq!(&config[0x40..0x42], &old[0x40..0x42]);
            assert_eq!(
                get16(&config, 0x42),
                3 | PCI_MSIX_FLAGS_ENABLE | PCI_MSIX_FLAGS_MASKALL
            );
            assert_eq!(&config[0x44..0x58], &old[0x44..0x58]);
        }
    }

    #[test]
    fn vfio_sparse_mappings_trap_table_and_pending_pages() {
        let bar = test_bar();
        let msix = test_msix();
        assert_eq!(
            mapping_ranges(&bar, &[(0, 0x8000)], Some(&msix)).unwrap(),
            [(0, 0x1000), (0x3000, 0x1000), (0x5000, 0x3000)]
        );
        assert_eq!(mapping_ranges(&bar, &[], None).unwrap(), []);
        for areas in [
            vec![(1, 4096)],
            vec![(0, 0)],
            vec![(0, 8192), (4096, 4096)],
            vec![(0x7000, 8192)],
            vec![(u64::MAX - 4095, 4096)],
        ] {
            assert!(mapping_ranges(&bar, &areas, None).is_err());
        }
    }
}

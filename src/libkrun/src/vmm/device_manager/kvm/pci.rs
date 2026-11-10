// Copyright 2026 The libkrun Authors.
// SPDX-License-Identifier: Apache-2.0

//! PCI root and virtio-pci registration for the x86_64 KVM backend.

use std::fmt::{Display, Formatter};
use std::io;
use std::ops::Range;
use std::sync::{Arc, Mutex};

use arch::x86_64::layout::{
    PCI_BAR_END, PCI_BAR_START, PCI_ECAM_SIZE, PCI_ECAM_START, PCI_IO_END, PCI_IO_START,
};
use devices::Bus;
use devices::pci::{
    BarWindow, ConfigMechanism1, Ecam, PciAddress, PciFunction, PciIntxLine, PciResourceWindow,
    PciRootError,
};
use devices::virtio::{
    CreatePciTransportError, VIRTIO_PCI_BAR0_SIZE, VirtioDevice, VirtioPciTransport,
};
use kvm_ioctls::VmFd;
use vm_memory::GuestMemoryMmap;

use crate::vmm::device_manager::irq::IrqAllocator;
#[cfg(vfio)]
use arch::x86_64::layout::FIRST_ADDR_PAST_32BITS;

const PCI_BUS0: u8 = 0;

#[derive(Debug)]
pub enum Error {
    Bus(devices::BusError),
    CreateTransport(CreatePciTransportError),
    IrqsExhausted,
    PciRoot(PciRootError),
    Resource(io::Error),
}

impl Display for Error {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Bus(err) => write!(f, "failed to register PCI bus device: {err}"),
            Self::CreateTransport(err) => write!(f, "failed to create virtio-pci transport: {err}"),
            Self::IrqsExhausted => write!(f, "no more GSIs are available for PCI INTx"),
            Self::PciRoot(err) => write!(f, "failed to register PCI function: {err}"),
            Self::Resource(err) => write!(f, "failed to allocate PCI resource: {err}"),
        }
    }
}

type Result<T> = std::result::Result<T, Error>;

#[cfg(vfio)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum PciBarKind {
    Io,
    Memory32,
    Memory64,
}

#[cfg(vfio)]
pub(crate) fn high_window(guest_phys_bits: u32, guest_last_addr: u64) -> Result<Range<u64>> {
    let end = 1u64 << guest_phys_bits;
    guest_last_addr
        .checked_add(1)
        .and_then(|ram_end| {
            ram_end
                .max(FIRST_ADDR_PAST_32BITS)
                .checked_next_multiple_of(1 << 30)
        })
        .filter(|start| *start < end)
        .map(|start| start..end)
        .ok_or_else(|| {
            Error::Resource(io::Error::other(
                "no guest address space for a 64-bit PCI window",
            ))
        })
}

/// Owns the bus-0 PCI functions and their guest address windows.
pub struct PciHostManager {
    root: devices::pci::SharedPciRoot,
    next_device: u8,
    irqs: IrqAllocator,
    functions: Vec<arch::x86_64::PciFunctionInfo>,
    io: PciResourceWindow,
    mmio32: PciResourceWindow,
    mmio64: PciResourceWindow,
}

struct KvmPciIntxLine {
    vm: Arc<VmFd>,
    gsi: u32,
}

impl PciIntxLine for KvmPciIntxLine {
    fn set_level(&self, asserted: bool) -> std::io::Result<()> {
        self.vm
            .set_irq_line(self.gsi, asserted)
            .map_err(|err| std::io::Error::from_raw_os_error(err.errno()))
    }
}

impl PciHostManager {
    /// `high_window` is the guest-physical range for 64-bit BARs, if any.
    pub fn new(irqs: IrqAllocator, high_window: Option<Range<u64>>) -> Self {
        Self {
            root: devices::pci::PciRoot::shared(),
            next_device: 1,
            irqs,
            functions: Vec::new(),
            io: PciResourceWindow::new(PCI_IO_START..PCI_IO_END),
            mmio32: PciResourceWindow::new(PCI_BAR_START..PCI_BAR_END),
            mmio64: PciResourceWindow::new(high_window.unwrap_or(0..0)),
        }
    }

    fn insert_bar_window(&self, bus: &mut Bus, window: Range<u64>) -> Result<()> {
        bus.insert(
            Arc::new(Mutex::new(BarWindow::new(self.root.clone(), window.start))),
            window.start,
            window.end - window.start,
        )
        .map_err(Error::Bus)
    }

    pub fn register_config_io(&self, bus: &mut Bus) -> Result<()> {
        self.insert_bar_window(bus, self.io.range())?;
        bus.insert(
            Arc::new(Mutex::new(ConfigMechanism1::new(self.root.clone()))),
            devices::pci::CONFIG_MECHANISM_1_ADDRESS,
            devices::pci::CONFIG_MECHANISM_1_SIZE,
        )
        .map_err(Error::Bus)
    }

    pub fn register_mmio(&self, bus: &mut Bus) -> Result<()> {
        let high = self.mmio64.range();
        if !high.is_empty() {
            self.insert_bar_window(bus, high)?;
        }
        bus.insert(
            Arc::new(Mutex::new(Ecam::new(self.root.clone()))),
            PCI_ECAM_START,
            PCI_ECAM_SIZE,
        )
        .map_err(Error::Bus)?;
        self.insert_bar_window(bus, self.mmio32.range())
    }

    pub fn register_virtio_device(
        &mut self,
        vm: Arc<VmFd>,
        guest_memory: GuestMemoryMmap,
        device: Arc<Mutex<dyn VirtioDevice>>,
    ) -> Result<arch::x86_64::PciFunctionInfo> {
        let bar_base = self
            .mmio32
            .allocate(VIRTIO_PCI_BAR0_SIZE)
            .map_err(Error::Resource)?;
        let (address, irq) = self.allocate_function()?;
        let intx_line = Arc::new(KvmPciIntxLine { vm, gsi: irq });
        let transport =
            VirtioPciTransport::new(guest_memory, device, irq as u8, intx_line, bar_base as u32)
                .map_err(Error::CreateTransport)?;

        let function: Arc<Mutex<dyn PciFunction>> = Arc::new(Mutex::new(transport));
        self.register_function(address, irq, function)
    }

    pub(crate) fn allocate_function(&mut self) -> Result<(PciAddress, u32)> {
        if self.next_device > 31 {
            return Err(Error::IrqsExhausted);
        }
        let irq = self.irqs.alloc().map_err(|_| Error::IrqsExhausted)?;
        let address = PciAddress::new(PCI_BUS0, self.next_device, 0);
        self.next_device += 1;
        Ok((address, irq))
    }

    /// Allocates every requested BAR, or none of them.
    #[cfg(vfio)]
    pub(crate) fn allocate_bars(&mut self, bars: &[(PciBarKind, u64)]) -> Result<Vec<u64>> {
        let (mut staged_io, mut staged_mmio32, mut staged_mmio64) =
            (self.io.clone(), self.mmio32.clone(), self.mmio64.clone());
        let addresses = bars
            .iter()
            .map(|&(kind, size)| {
                match kind {
                    PciBarKind::Io => &mut staged_io,
                    PciBarKind::Memory32 => &mut staged_mmio32,
                    PciBarKind::Memory64 => &mut staged_mmio64,
                }
                .allocate(size)
            })
            .collect::<io::Result<_>>()
            .map_err(Error::Resource)?;
        (self.io, self.mmio32, self.mmio64) = (staged_io, staged_mmio32, staged_mmio64);
        Ok(addresses)
    }

    /// `address` and `irq` come from [`Self::allocate_function`].
    pub(crate) fn register_function(
        &mut self,
        address: PciAddress,
        irq: u32,
        function: Arc<Mutex<dyn PciFunction>>,
    ) -> Result<arch::x86_64::PciFunctionInfo> {
        self.root
            .lock()
            .expect("Poisoned PCI root lock")
            .insert(address, function)
            .map_err(Error::PciRoot)?;
        let info = arch::x86_64::PciFunctionInfo {
            device: address.device,
            function: address.function,
            gsi: irq,
        };
        self.functions.push(info);
        Ok(info)
    }

    pub fn acpi_info(&self) -> arch::x86_64::PciHostInfo {
        let high = self.mmio64.range();
        let mmio32 = self.mmio32.range();
        arch::x86_64::PciHostInfo {
            high_window: (!high.is_empty()).then_some(high),
            io_window: Some(self.io.range()),
            ecam_base: PCI_ECAM_START,
            bar_start: mmio32.start,
            bar_size: mmio32.end - mmio32.start,
            functions: self.functions.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::super::super::builder;
    use super::*;
    use crate::vmm::device_manager::mmio::MMIODeviceManager;
    use devices::legacy::{DummyIrqChip, KvmIoapic};
    use devices::virtio::{
        ActivateResult, DeviceQueue, InterruptTransport, MmioTransport, QueueConfig,
    };
    use vm_memory::GuestAddress;

    /// VIRTIO_F_VERSION_1 feature bit, required by the PCI transport.
    const VIRTIO_F_VERSION_1: u32 = 32;
    const QUEUE_CONFIG: &[QueueConfig] = &[QueueConfig::new(64)];

    struct DummyDevice;

    impl VirtioDevice for DummyDevice {
        fn avail_features(&self) -> u64 {
            1u64 << VIRTIO_F_VERSION_1
        }

        fn acked_features(&self) -> u64 {
            0
        }

        fn set_acked_features(&mut self, _: u64) {}

        fn device_type(&self) -> u32 {
            0
        }

        fn device_name(&self) -> &str {
            "dummy"
        }

        fn queue_config(&self) -> &[QueueConfig] {
            QUEUE_CONFIG
        }

        fn config_len(&self) -> Option<u32> {
            Some(0)
        }

        fn read_config(&self, _offset: u64, _data: &mut [u8]) {}

        fn write_config(&mut self, _offset: u64, _data: &[u8]) {}

        fn activate(
            &mut self,
            _mem: GuestMemoryMmap,
            _intc: InterruptTransport,
            _queues: Vec<DeviceQueue>,
        ) -> ActivateResult {
            Ok(())
        }

        fn is_activated(&self) -> bool {
            false
        }
    }

    fn dummy_device() -> Arc<Mutex<dyn VirtioDevice>> {
        Arc::new(Mutex::new(DummyDevice))
    }

    #[test]
    fn test_mmio_and_pci_devices_get_distinct_irqs() {
        let guest_mem = GuestMemoryMmap::from_ranges(&[
            (GuestAddress(0x0), 0x1000),
            (GuestAddress(0x1000), 0x1000),
        ])
        .unwrap();
        let mut arch_mem_info = arch::ArchMemoryInfo::default();
        let vm = builder::setup_vm(&guest_mem, &mut arch_mem_info, false).unwrap();
        let _kvmioapic = KvmIoapic::new(vm.fd()).unwrap();

        let irqs = IrqAllocator::new(arch::IRQ_BASE, arch::IRQ_MAX);
        let mut mmio = MMIODeviceManager::new(&mut 0xd000_0000, irqs.clone());
        let mut pci = PciHostManager::new(irqs, None);

        // Same order as build_microvm(): an MMIO device (the RTC) is attached
        // before the PCI devices.
        let transport = MmioTransport::new(
            guest_mem.clone(),
            DummyIrqChip::new().into(),
            dummy_device(),
        )
        .unwrap();
        let (_, mmio_irq) = mmio
            .register_mmio_device(vm.fd(), transport, 0, "rtc".to_string())
            .unwrap();
        let first = pci
            .register_virtio_device(vm.fd_shared(), guest_mem.clone(), dummy_device())
            .unwrap();
        let second = pci
            .register_virtio_device(vm.fd_shared(), guest_mem, dummy_device())
            .unwrap();

        assert_eq!(mmio_irq, arch::IRQ_BASE);
        assert_eq!(first.gsi, arch::IRQ_BASE + 1);
        assert_eq!(second.gsi, arch::IRQ_BASE + 2);
    }

    #[test]
    fn test_pci_acpi_info_describes_resource_windows() {
        let irqs = IrqAllocator::new(arch::IRQ_BASE, arch::IRQ_MAX);
        let info = PciHostManager::new(irqs.clone(), None).acpi_info();
        assert_eq!(info.high_window, None);
        assert_eq!(info.io_window, Some(PCI_IO_START..PCI_IO_END));
        assert_eq!(info.bar_start, PCI_BAR_START);
        let info = PciHostManager::new(irqs, Some(1 << 36..1 << 37)).acpi_info();
        assert_eq!(info.high_window, Some(1 << 36..1 << 37));
    }

    #[cfg(vfio)]
    #[test]
    fn test_high_window_starts_above_ram_on_a_gib_boundary() {
        assert_eq!(high_window(36, 0x7fff_ffff).unwrap(), 1 << 32..1 << 36);
        assert_eq!(
            high_window(40, (6 << 30) + 0xfff).unwrap(),
            7 << 30..1 << 40
        );
        assert!(high_window(32, 0x7fff_ffff).is_err());
    }

    #[cfg(vfio)]
    #[test]
    fn test_pci_bar_allocation_is_all_or_nothing() {
        let irqs = IrqAllocator::new(arch::IRQ_BASE, arch::IRQ_MAX);
        let mut pci = PciHostManager::new(irqs, None);
        let request = [
            (PciBarKind::Memory32, 0x1000),
            (PciBarKind::Memory64, 0x1000),
        ];
        assert!(pci.allocate_bars(&request).is_err());
        let request = [(PciBarKind::Memory32, 0x1000), (PciBarKind::Io, 0x100)];
        assert_eq!(
            pci.allocate_bars(&request).unwrap(),
            [PCI_BAR_START, PCI_IO_START]
        );
    }

    #[test]
    fn test_pci_reports_exhausted_shared_irqs() {
        let guest_mem = GuestMemoryMmap::from_ranges(&[(GuestAddress(0x0), 0x1000)]).unwrap();
        let mut arch_mem_info = arch::ArchMemoryInfo::default();
        let vm = builder::setup_vm(&guest_mem, &mut arch_mem_info, false).unwrap();

        let irqs = IrqAllocator::new(arch::IRQ_BASE, arch::IRQ_BASE);
        // Another manager already took the only IRQ.
        irqs.alloc().unwrap();
        let mut pci = PciHostManager::new(irqs, None);

        let err = pci
            .register_virtio_device(vm.fd_shared(), guest_mem, dummy_device())
            .unwrap_err();
        assert!(matches!(err, Error::IrqsExhausted));
    }
}

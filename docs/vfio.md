# VFIO PCI assignment

VFIO assignment gives a guest direct control of a host PCI function. Build with `make VFIO=1` (Cargo feature `vfio`). It is available on Linux/x86_64.

```rust
let mut devices = PciDeviceManager::new();
devices.add(ConsoleDevice::builder().build()?);
devices.add(VfioDevice::new("0000:03:00.0")?);
let vmm = VmmBuilder::new()
    .vcpus(2)?
    .ram_mib(1024)?
    .payload(payload)
    .acpi(true)?
    .devices(devices)
    .build()?;
vmm.run();
```

The C API uses `krun_vfio_device_new()` and `krun_pci_device_manager_add()`.

## Host setup

libkrun does not bind drivers, change permissions, or reset the device. Before building the VM:

- Bind the function to `vfio-pci` and grant access to `/dev/vfio/vfio` and its group's `/dev/vfio/<group>`. The group must be viable and use a type-1 IOMMU.
- Raise the locked-memory limit to cover guest RAM, which stays pinned for DMA.
- Give the VM exclusive access to the group. VFIO may reset the function when it is opened or closed. Recovering a device that fails to reset is up to the caller.

See the [kernel VFIO documentation](https://kernel.org/doc/html/latest/driver-api/vfio.html).

## Guest requirements

The guest kernel needs `CONFIG_ACPI`, `CONFIG_PCI`, `CONFIG_PCI_MMCONFIG`, `CONFIG_PCI_MSI`, and `CONFIG_VIRTIO_PCI`, plus the device's driver and firmware. The function appears at a guest address chosen by libkrun, not at its host address, and without its expansion ROM, function-level reset, or extended capabilities such as SR-IOV and resizable BARs.

## Assignment limits

This initial implementation supports one PCI endpoint with an IOMMU-backed VFIO group on Linux/x86_64. It requires ACPI and the in-kernel irqchip, and cannot be combined with devices that reclaim guest RAM, such as the balloon. Shared-memory windows whose backing can change are excluded from DMA.

Conventional configuration reads use live VFIO state. Guest BAR addresses, capability links, interrupt configuration, and hidden features are emulated or protected. Permitted writes, including vendor registers outside protected capabilities, go through VFIO. VFIO remains authoritative for read-only and write-one-to-clear semantics. The kernel can filter status writes, so libkrun does not promise unrestricted status clearing.

Hidden capabilities are non-writable. Known lengths bound their protected ranges. An opaque capability protects the space up to the next capability at a higher offset, or the end of conventional configuration space. Vendor registers in such a gap require additional capability knowledge or a device quirk.

BARs may move only within the span reserved for this function's BARs of the same kind. Overlapping or out-of-range BAR addresses disable BAR access. I/O and memory decode are independent, and both require the PM state read back from VFIO to be D0. Accessible BAR reads and writes trap to VFIO.

The compatibility contract covers standard PCI configuration and interrupt access paths. Devices whose drivers require configuration or interrupt-table aliases through BARs are outside this scope. PCIe capability version zero and device-specific MSI-X pending-bit-array errata are not corrected. Malformed capability and region metadata remain errors. This is not a claim that every GPU, NIC, or host-created virtual function works. Device reset/recovery and surprise physical removal require separate qualification and handling.

## Interrupts

INTx uses the in-kernel IOAPIC. An exposed MSI capability offers one vector and requires a reported host MSI vector and a guest GSI route at attachment. MSI address, data, enable, and mask writes are emulated even when a byte, word, or dword access also touches a capability header or reserved bytes. Masked events stay pending and are delivered when unmasked. Host activation must succeed before the mode becomes active or guest irqfds are registered. A later access failure stops the VM with a nonzero exit status.

MSI-X emulates the full table and pending-bit array (PBA), including vector and function masks. The table and PBA obey the same D0 and memory-decode gate as other memory BAR accesses. Attachment requires at least the table's reported host vector count and one guest GSI route per vector. Enabling MSI-X also requires actual allocation of the entire table on the host. A positive allocation result means insufficient vectors, not success. Allocation failure stops the VM with a nonzero exit status through the device error path. Dynamic vector allocation is deferred.

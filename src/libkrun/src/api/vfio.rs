// Copyright 2026 The libkrun Authors.
// SPDX-License-Identifier: Apache-2.0

#[cfg(vfio)]
use {
    super::{AttachContext, AttachDevice, DeviceRequirements, VmmError},
    crate::vmm::vfio::VfioPci,
    std::sync::{Arc, Mutex},
};

/// One host PCI function assigned through an IOMMU-protected VFIO group.
///
/// Driver binding, permissions, and reset policy belong to the caller.
#[cfg(vfio)]
pub struct VfioDevice {
    host_bdf: String,
}

#[cfg_attr(
    feature = "ffi",
    ffier::export(
        cfg = "all(feature = \"vfio\", target_os = \"linux\", target_arch = \"x86_64\", not(any(feature = \"tee\", feature = \"aws-nitro\")))"
    )
)]
impl VfioDevice {
    /// Validate a full `dddd:bb:dd.f` PCI address without opening the device.
    /// Hardware is opened only when the VM is built.
    pub fn new(host_bdf: &str) -> Result<Self, VmmError> {
        let bytes = host_bdf.as_bytes();
        if bytes.len() != 12
            || bytes[4] != b':'
            || bytes[7] != b':'
            || bytes[10] != b'.'
            || bytes
                .iter()
                .enumerate()
                .any(|(i, b)| !matches!(i, 4 | 7 | 10) && !b.is_ascii_hexdigit())
            || !matches!(bytes[8], b'0' | b'1')
            || bytes[11] > b'7'
        {
            return Err(VmmError::InvalidParam());
        }
        Ok(Self {
            host_bdf: host_bdf.to_ascii_lowercase(),
        })
    }
}

#[cfg_attr(
    feature = "ffi",
    ffier::export(
        cfg = "all(feature = \"vfio\", target_os = \"linux\", target_arch = \"x86_64\", not(any(feature = \"tee\", feature = \"aws-nitro\")))"
    )
)]
impl<'a> AttachDevice<'a> for VfioDevice {
    #[cfg_attr(feature = "ffi", ffier(skip))]
    fn requirements(&self) -> DeviceRequirements {
        DeviceRequirements {
            vfio_functions: 1,
            ..Default::default()
        }
    }

    #[cfg_attr(feature = "ffi", ffier(skip))]
    fn attach(self: Box<Self>, ctx: &mut AttachContext) -> Result<(), VmmError> {
        let vfio = VfioPci::new(&self.host_bdf, ctx)
            .map_err(|e| VmmError::Internal(format!("VFIO: {e}")))?;
        let address = vfio.address;
        let irq = vfio.irq();
        let vfio = Arc::new(Mutex::new(vfio));
        // Register cleanup before subsequent fallible attachment operations.
        ctx.push_exit_observer(vfio.clone());
        ctx.register_pci_function(address, irq, vfio)
    }
}

#[cfg(vfio)]
pub(crate) fn validate_configuration(
    requirements: &[DeviceRequirements],
    pci: bool,
    acpi: bool,
    split_irqchip: bool,
) -> Result<(), VmmError> {
    let functions: usize = requirements.iter().map(|r| r.vfio_functions).sum();
    if functions == 0 {
        return Ok(());
    }
    let unsupported = if functions > 1 {
        "VFIO supports one assigned function per VM"
    } else if !pci {
        "VFIO requires a PciDeviceManager"
    } else if !acpi {
        "VFIO requires ACPI for guest PCI discovery"
    } else if split_irqchip {
        "VFIO requires the in-kernel irqchip; split_irqchip is unsupported"
    } else if requirements.iter().any(|r| r.reclaims_guest_ram) {
        "VFIO pins guest RAM for DMA; devices that reclaim guest RAM are unsupported"
    } else {
        return Ok(());
    };
    Err(VmmError::MissingConfig(unsupported.into()))
}

#[cfg(all(test, vfio))]
mod tests {
    use super::*;
    #[test]
    fn vfio_requires_a_full_host_address() {
        for bdf in [
            "",
            "00:06.0",
            "0000:00:20.0",
            "0000:00:00.8",
            "0000:00:00.a",
            "../0000:00.0",
            "0000:00:00.0/",
            "0000:00:00.0\n",
            "0000:gg:00.0",
            "é000:00:0.0",
        ] {
            assert!(VfioDevice::new(bdf).is_err(), "{bdf}");
        }
        assert_eq!(
            VfioDevice::new("ABCD:FF:1F.7").unwrap().host_bdf,
            "abcd:ff:1f.7"
        );
    }
    #[test]
    fn vfio_rejects_second_device_and_incompatible_configuration_before_hardware() {
        let device = VfioDevice::new("0000:00:01.0").unwrap();
        let one = [device.requirements()];
        assert!(validate_configuration(&one, true, true, false).is_ok());
        assert!(
            validate_configuration(
                &[device.requirements(), device.requirements()],
                true,
                true,
                false
            )
            .is_err()
        );
        for (pci, acpi, split) in [
            (false, true, false),
            (true, false, false),
            (true, true, true),
        ] {
            assert!(validate_configuration(&one, pci, acpi, split).is_err());
        }
        assert!(
            validate_configuration(
                &[
                    device.requirements(),
                    DeviceRequirements {
                        reclaims_guest_ram: true,
                        ..Default::default()
                    }
                ],
                true,
                true,
                false
            )
            .is_err()
        );
    }
}

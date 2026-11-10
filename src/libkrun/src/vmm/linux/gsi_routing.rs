// Copyright 2026 The libkrun Authors.
// SPDX-License-Identifier: Apache-2.0

use std::collections::BTreeMap;
use std::io;
use std::ops::Range;
use std::sync::Arc;

use kvm_bindings::{
    KVM_IRQ_ROUTING_IRQCHIP, KVM_IRQ_ROUTING_MSI, KVM_IRQCHIP_IOAPIC, KVM_IRQCHIP_PIC_MASTER,
    KVM_IRQCHIP_PIC_SLAVE, KvmIrqRouting, kvm_irq_routing_entry,
    kvm_irq_routing_entry__bindgen_ty_1, kvm_irq_routing_irqchip, kvm_irq_routing_msi,
};
use kvm_ioctls::{Cap, VmFd};

const IOAPIC_PINS: u32 = 24;
const PIC_PINS: u32 = 16;
const DEFAULT_ROUTES: u32 = IOAPIC_PINS + PIC_PINS;

/// The address and data a device writes to raise a message-signaled interrupt.
pub(crate) type MsiMessage = (u64, u32);

#[derive(Clone, Default)]
struct MsiRoutes(BTreeMap<u32, MsiMessage>);

impl MsiRoutes {
    fn updated(
        &self,
        changes: impl IntoIterator<Item = (u32, Option<MsiMessage>)>,
    ) -> Option<Self> {
        let mut changes = changes
            .into_iter()
            .filter(|&(gsi, message)| self.0.get(&gsi).copied() != message)
            .peekable();
        changes.peek()?;
        let mut routes = self.clone();
        for (gsi, message) in changes {
            match message {
                Some(message) => routes.0.insert(gsi, message),
                None => routes.0.remove(&gsi),
            };
        }
        Some(routes)
    }

    /// KVM's default IOAPIC and PIC routes, followed by the MSI routes.
    fn entries(&self) -> Vec<kvm_irq_routing_entry> {
        let irqchip = |gsi, irqchip, pin| kvm_irq_routing_entry {
            gsi,
            type_: KVM_IRQ_ROUTING_IRQCHIP,
            u: kvm_irq_routing_entry__bindgen_ty_1 {
                irqchip: kvm_irq_routing_irqchip { irqchip, pin },
            },
            ..Default::default()
        };
        let mut entries = Vec::new();
        for gsi in 0..IOAPIC_PINS {
            entries.push(irqchip(gsi, KVM_IRQCHIP_IOAPIC, gsi));
            if gsi < PIC_PINS {
                let pic = if gsi < 8 {
                    KVM_IRQCHIP_PIC_MASTER
                } else {
                    KVM_IRQCHIP_PIC_SLAVE
                };
                entries.push(irqchip(gsi, pic, gsi % 8));
            }
        }
        for (&gsi, &(address, data)) in &self.0 {
            entries.push(kvm_irq_routing_entry {
                gsi,
                type_: KVM_IRQ_ROUTING_MSI,
                u: kvm_irq_routing_entry__bindgen_ty_1 {
                    msi: kvm_irq_routing_msi {
                        address_lo: address as u32,
                        address_hi: (address >> 32) as u32,
                        data,
                        ..Default::default()
                    },
                },
                ..Default::default()
            });
        }
        entries
    }
}

/// Owns the VM's GSI routing table.
///
/// KVM_SET_GSI_ROUTING replaces the whole table, so a device that set it
/// directly would erase the irqchip's default routes and every other
/// device's MSI routes. Devices instead allocate GSIs here and update only
/// their own routes, and each change resubmits the merged table.
pub(crate) struct GsiRouting {
    vm: Arc<VmFd>,
    routes: MsiRoutes,
    next_gsi: u32,
    max_entries: u32,
}

impl GsiRouting {
    pub(crate) fn new(vm: Arc<VmFd>) -> Self {
        let max_entries = vm.check_extension_int(Cap::IrqRouting) as u32;
        Self {
            vm,
            routes: MsiRoutes::default(),
            next_gsi: IOAPIC_PINS,
            max_entries,
        }
    }

    pub(crate) fn allocate(&mut self, count: u32) -> io::Result<Range<u32>> {
        let end = self
            .next_gsi
            .checked_add(count)
            .filter(|end| DEFAULT_ROUTES + (end - IOAPIC_PINS) <= self.max_entries)
            .ok_or_else(|| io::Error::other("KVM GSI routing table is full"))?;
        let gsis = self.next_gsi..end;
        self.next_gsi = end;
        Ok(gsis)
    }

    pub(crate) fn update(
        &mut self,
        changes: impl IntoIterator<Item = (u32, Option<MsiMessage>)>,
    ) -> io::Result<()> {
        let Some(routes) = self.routes.updated(changes) else {
            return Ok(());
        };
        let entries = routes.entries();
        let mut routing = KvmIrqRouting::new(entries.len()).map_err(io::Error::other)?;
        routing.as_mut_slice().copy_from_slice(&entries);
        self.vm.set_gsi_routing(&routing)?;
        self.routes = routes;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn msi_routes_preserve_all_default_routes() {
        let routes = MsiRoutes::default()
            .updated([(24, Some((0xfee0_0000, 0x45))), (25, None)])
            .unwrap();
        let entries = routes.entries();
        let irqchip: Vec<_> = entries
            .iter()
            .filter(|e| e.type_ == KVM_IRQ_ROUTING_IRQCHIP)
            .collect();
        assert_eq!(irqchip.len(), (IOAPIC_PINS + PIC_PINS) as usize);
        for gsi in 0..IOAPIC_PINS {
            assert!(irqchip.iter().any(|e| e.gsi == gsi));
        }
        assert_eq!(entries.last().unwrap().gsi, 24);
        assert_eq!(entries.last().unwrap().type_, KVM_IRQ_ROUTING_MSI);
    }

    #[test]
    fn unchanged_routes_are_not_resubmitted() {
        let routes = MsiRoutes::default()
            .updated([(24, Some((0xfee0_0000, 0x45)))])
            .unwrap();
        assert!(
            routes
                .updated([(24, Some((0xfee0_0000, 0x45))), (25, None)])
                .is_none()
        );
        assert!(routes.updated([(24, Some((0xfee0_1000, 0x45)))]).is_some());
        let routes = routes.updated([(24, None)]).unwrap();
        assert!(routes.updated([(24, None)]).is_none());
    }
}

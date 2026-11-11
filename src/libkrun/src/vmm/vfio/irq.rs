// Copyright 2026 The libkrun Authors.
// SPDX-License-Identifier: Apache-2.0

use std::io;
use std::os::fd::{AsRawFd, RawFd};
use utils::sized_vec::vec_with_array_field;
use vfio_bindings::bindings::vfio::*;
use vmm_sys_util::{ioctl::ioctl_with_ref, ioctl_io_nr};

ioctl_io_nr!(VFIO_DEVICE_SET_IRQS, VFIO_TYPE.into(), VFIO_BASE + 10);

pub(super) fn set_irq_eventfds(device: &impl AsRawFd, index: u32, fds: &[RawFd]) -> io::Result<()> {
    set_irq_eventfds_with(index, fds, |set| {
        // The aligned allocation includes the eventfd array described by argsz.
        unsafe { ioctl_with_ref(device, VFIO_DEVICE_SET_IRQS(), set) }
    })
}

fn set_irq_eventfds_with(
    index: u32,
    fds: &[RawFd],
    ioctl: impl FnOnce(&vfio_irq_set) -> i32,
) -> io::Result<()> {
    let mut allocation = vec_with_array_field::<vfio_irq_set, RawFd>(fds.len());
    let set = &mut allocation[0];
    set.argsz = (size_of::<vfio_irq_set>() + size_of_val(fds)) as u32;
    set.flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
    set.index = index;
    set.count = fds.len() as u32;
    // SAFETY: vec_with_array_field reserved this many trailing bytes.
    let data = unsafe { set.data.as_mut_slice(size_of_val(fds)) };
    for (bytes, fd) in data
        .as_chunks_mut::<{ size_of::<RawFd>() }>()
        .0
        .iter_mut()
        .zip(fds)
    {
        bytes.copy_from_slice(&fd.to_ne_bytes());
    }
    match ioctl(set) {
        0 => Ok(()),
        result if result < 0 => Err(io::Error::last_os_error()),
        result => Err(io::Error::other(format!(
            "VFIO IRQ {index}: insufficient host vector allocation (requested {}, available {result})",
            fds.len()
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vfio_irq_allocation_checks_results_and_eventfd_payload() {
        let fds: [RawFd; 4] = [7, 11, 19, 23];
        for result in [0, 2, -1] {
            let outcome = set_irq_eventfds_with(VFIO_PCI_MSIX_IRQ_INDEX, &fds, |set| {
                assert_eq!(
                    set.argsz as usize,
                    size_of::<vfio_irq_set>() + size_of_val(&fds)
                );
                assert_eq!(
                    set.flags,
                    VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER
                );
                assert_eq!(
                    (set.index, set.start, set.count),
                    (VFIO_PCI_MSIX_IRQ_INDEX, 0, 4)
                );
                assert_eq!(set as *const _ as usize % align_of::<vfio_irq_set>(), 0);
                // SAFETY: the header describes the allocation supplied to this ioctl.
                let bytes = unsafe { set.data.as_slice(size_of_val(&fds)) };
                for (bytes, fd) in bytes.as_chunks::<4>().0.iter().zip(&fds) {
                    assert_eq!(i32::from_ne_bytes(*bytes), *fd);
                }
                if result < 0 {
                    // An invalid descriptor sets errno without closing a live resource.
                    unsafe {
                        libc::close(-1);
                    }
                }
                result
            });
            match result {
                0 => outcome.unwrap(),
                -1 => assert_eq!(outcome.unwrap_err().raw_os_error(), Some(libc::EBADF)),
                _ => assert!(
                    outcome
                        .unwrap_err()
                        .to_string()
                        .contains("insufficient host vector allocation")
                ),
            }
        }
    }
}

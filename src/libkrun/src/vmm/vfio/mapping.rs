// Copyright 2026 The libkrun Authors.
// SPDX-License-Identifier: Apache-2.0

use super::*;

pub(super) fn mapping_ranges(
    bar: &Bar,
    areas: &[(u64, u64)],
    msix: Option<&Msix>,
) -> io::Result<Vec<(u64, u64)>> {
    let mut areas = areas.to_vec();
    areas.sort_unstable();
    let mut previous_end = 0;
    let mut result = Vec::new();
    for (offset, size) in areas {
        if size == 0
            || !contains(0, bar.size, offset, size)
            || !offset.is_multiple_of(PAGE_SIZE)
            || !size.is_multiple_of(PAGE_SIZE)
            || offset < previous_end
        {
            return Err(io::Error::other("invalid VFIO sparse mmap area"));
        }
        previous_end = offset + size;
        let mut pieces = vec![(offset, offset + size)];
        if let Some(msix) = msix {
            for (index, offset, length) in [
                (msix.bar, msix.offset, msix.table.len() as u64),
                (msix.pba_bar, msix.pba_offset, msix.pba_size()),
            ] {
                if index != bar.index {
                    continue;
                }
                let start = offset & !(PAGE_SIZE - 1);
                let end = (offset + length).next_multiple_of(PAGE_SIZE);
                pieces = pieces
                    .into_iter()
                    .flat_map(|(a, b)| [(a, b.min(start)), (a.max(end), b)])
                    .filter(|(a, b)| a < b)
                    .collect();
            }
        }
        result.extend(pieces.into_iter().map(|(a, b)| (a, b - a)));
    }
    Ok(result)
}

pub(super) struct Mapping {
    pub address: usize,
    pub size: u64,
    pub offset: u64,
    pub bar: usize,
    pub slot: u32,
    pub mapped_at: Option<u64>,
}
impl Mapping {
    pub fn set(
        &mut self,
        target: Option<u64>,
        set_region: &mut impl FnMut(kvm_userspace_memory_region) -> io::Result<()>,
    ) -> io::Result<()> {
        if self.mapped_at != target {
            set_region(kvm_userspace_memory_region {
                slot: self.slot,
                guest_phys_addr: target.unwrap_or(0),
                memory_size: target.map_or(0, |_| self.size),
                userspace_addr: self.address as u64,
                flags: 0,
            })?;
            self.mapped_at = target;
        }
        Ok(())
    }
    pub fn unmap(&mut self, vm: &VmFd) -> io::Result<()> {
        self.set(None, &mut |region| set_memory(vm, region))
    }
}
impl Drop for Mapping {
    fn drop(&mut self) {
        // A failed KVM unmap must not leave a slot pointing at released host memory.
        if self.mapped_at.is_none() {
            unsafe {
                libc::munmap(self.address as *mut _, self.size as usize);
            }
        }
    }
}

pub(super) fn set_memory(vm: &VmFd, region: kvm_userspace_memory_region) -> io::Result<()> {
    // Mapping retains the host mmap until the slot is successfully removed.
    unsafe { vm.set_user_memory_region(region) }.map_err(Into::into)
}

pub(super) fn prepare_bar(
    bar: usize,
    pieces: &[(u64, u64)],
    reserve: impl FnOnce(u32) -> io::Result<Range<u32>>,
    mut mmap: impl FnMut(u64, u64) -> io::Result<usize>,
) -> Vec<Mapping> {
    let result: io::Result<Vec<_>> = reserve(pieces.len() as u32).and_then(|slots| {
        pieces
            .iter()
            .zip(slots)
            .map(|(&(offset, size), slot)| {
                Ok(Mapping {
                    address: mmap(offset, size)?,
                    size,
                    offset,
                    bar,
                    slot,
                    mapped_at: None,
                })
            })
            .collect()
    });
    result.unwrap_or_else(|error| {
        log::warn!("VFIO BAR {bar} acceleration disabled: {error}; using trapped access");
        Vec::new()
    })
}

fn discard_bar(
    mappings: &mut Vec<Mapping>,
    bar: usize,
    set_region: &mut impl FnMut(kvm_userspace_memory_region) -> io::Result<()>,
) -> io::Result<()> {
    let mut retained: Vec<_> = mappings
        .extract_if(.., |mapping| mapping.bar == bar)
        .collect();
    let result = release_all(&mut retained, |mapping| mapping.set(None, set_region));
    mappings.extend(retained);
    result
}

pub(super) fn update(
    mappings: &mut Vec<Mapping>,
    bars: &[Bar],
    enabled: bool,
    mut set_region: impl FnMut(kvm_userspace_memory_region) -> io::Result<()>,
) -> io::Result<()> {
    let target = |mapping: &Mapping| {
        enabled.then(|| {
            bars.iter()
                .find(|bar| bar.index == mapping.bar)
                .unwrap()
                .address
                + mapping.offset
        })
    };
    // Remove every stale slot before installing any, including when BARs exchange addresses.
    for mapping in mappings.iter_mut() {
        if mapping.mapped_at != target(mapping) {
            mapping.set(None, &mut set_region)?;
        }
    }
    for bar in bars {
        let result = mappings
            .iter_mut()
            .filter(|mapping| mapping.bar == bar.index)
            .try_for_each(|mapping| mapping.set(target(mapping), &mut set_region));
        if let Err(error) = result {
            discard_bar(mappings, bar.index, &mut set_region).map_err(|rollback| {
                io::Error::other(format!(
                    "VFIO BAR {}: {error}; removing live KVM slots failed: {rollback}",
                    bar.index
                ))
            })?;
            log::warn!(
                "VFIO BAR {} acceleration disabled: {error}; using trapped access",
                bar.index
            );
        }
    }
    Ok(())
}

pub(super) fn write_quiesced<T>(
    mappings: &mut [Mapping],
    revoke: bool,
    mut set_region: impl FnMut(kvm_userspace_memory_region) -> io::Result<()>,
    write: impl FnOnce() -> io::Result<T>,
) -> io::Result<T> {
    if revoke {
        for mapping in mappings {
            mapping.set(None, &mut set_region)?;
        }
    }
    write()
}

#[cfg(test)]
mod tests {
    use super::super::tests::test_bar;
    use super::*;
    use std::cell::RefCell;

    fn host_page() -> usize {
        let address = unsafe {
            libc::mmap(
                ptr::null_mut(),
                PAGE_SIZE as usize,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_PRIVATE | libc::MAP_ANONYMOUS,
                -1,
                0,
            )
        };
        assert_ne!(address, libc::MAP_FAILED);
        address as usize
    }

    fn mappings() -> Vec<Mapping> {
        prepare_bar(
            0,
            &[(0, PAGE_SIZE), (PAGE_SIZE, PAGE_SIZE)],
            |n| Ok(10..10 + n),
            |_, _| Ok(host_page()),
        )
    }

    fn remove(mappings: &mut Vec<Mapping>) {
        release_all(mappings, |m| m.set(None, &mut |_| Ok(()))).unwrap();
    }

    #[test]
    fn vfio_mapping_preparation_handles_slot_and_partial_mmap_failures() {
        let pieces = [(0, PAGE_SIZE), (PAGE_SIZE, PAGE_SIZE)];
        assert!(
            prepare_bar(
                0,
                &pieces,
                |_| Err(io::Error::other("slots exhausted")),
                |_, _| panic!("no slots")
            )
            .is_empty()
        );
        for fail_at in [0, 1] {
            let mut count = 0;
            assert!(
                prepare_bar(
                    0,
                    &pieces,
                    |n| Ok(10..10 + n),
                    |_, _| {
                        let index = count;
                        count += 1;
                        if index == fail_at {
                            Err(io::Error::other("mmap"))
                        } else {
                            Ok(host_page())
                        }
                    }
                )
                .is_empty()
            );
            assert_eq!(count, fail_at + 1);
        }
        assert!(
            prepare_bar(
                0,
                &[],
                |n| {
                    assert_eq!(n, 0);
                    Ok(10..10)
                },
                |_, _| panic!("no areas")
            )
            .is_empty()
        );
    }

    #[test]
    fn vfio_mapping_install_failures_roll_back_only_that_bar_and_do_not_retry() {
        for failed_slot in [10, 11] {
            let bar = test_bar();
            let mut other = bar.clone();
            other.index = 2;
            other.address += bar.size;
            let mut mappings = mappings();
            mappings.extend(prepare_bar(
                2,
                &[(0, PAGE_SIZE)],
                |_| Ok(12..13),
                |_, _| Ok(host_page()),
            ));
            let mut operations = vec![];
            update(
                &mut mappings,
                &[bar.clone(), other.clone()],
                true,
                |region| {
                    operations.push((region.slot, region.memory_size));
                    if region.slot == failed_slot && region.memory_size != 0 {
                        Err(io::Error::other("install"))
                    } else {
                        Ok(())
                    }
                },
            )
            .unwrap();
            assert_eq!(mappings.len(), 1);
            assert_eq!(mappings[0].bar, 2);
            assert_eq!(mappings[0].mapped_at, Some(other.address));
            assert_eq!(operations.contains(&(10, 0)), failed_slot == 11);
            update(&mut mappings, &[bar, other], true, |_| {
                panic!("disabled BAR must not retry")
            })
            .unwrap();
            remove(&mut mappings);
        }
    }

    #[test]
    fn vfio_mapping_failed_rollback_retains_live_backing() {
        let mut mappings = mappings();
        let address = mappings[0].address;
        let error = update(&mut mappings, &[test_bar()], true, |region| {
            if region.slot == 11 || region.memory_size == 0 {
                Err(io::Error::other("injected"))
            } else {
                Ok(())
            }
        })
        .unwrap_err();
        assert!(error.to_string().contains("removing live KVM slots failed"));
        assert_eq!(mappings.len(), 1);
        assert_eq!(mappings[0].slot, 10);
        assert_eq!(mappings[0].mapped_at, Some(test_bar().address));
        drop(mappings);
        let mut residency = 0u8;
        assert_eq!(
            unsafe { libc::mincore(address as *mut _, PAGE_SIZE as usize, &mut residency) },
            0
        );
        // There is no real KVM slot in this injected-I/O test.
        assert_eq!(
            unsafe { libc::munmap(address as *mut _, PAGE_SIZE as usize) },
            0
        );
    }

    #[test]
    fn vfio_mapping_relocation_removes_all_stale_slots_before_installing() {
        let mut mappings = mappings();
        mappings[1].bar = 1;
        mappings[1].offset = 0;
        let mut bars = [test_bar(), test_bar()];
        bars[1].index = 1;
        bars[1].address += bars[0].size;
        update(&mut mappings, &bars, true, |_| Ok(())).unwrap();
        let old = bars[0].address;
        bars[0].address = bars[1].address;
        bars[1].address = old;
        let mut operations = vec![];
        update(&mut mappings, &bars, true, |region| {
            operations.push((region.slot, region.memory_size, region.guest_phys_addr));
            Ok(())
        })
        .unwrap();
        assert_eq!(
            operations,
            [
                (10, 0, 0),
                (11, 0, 0),
                (10, PAGE_SIZE, bars[0].address),
                (11, PAGE_SIZE, bars[1].address)
            ]
        );
        remove(&mut mappings);
    }

    #[test]
    fn vfio_mapping_decode_writes_quiesce_before_backend_io_and_fail_closed() {
        let access = ConfigAccess {
            pm: Some(0x44),
            ..ConfigAccess::new()
        };
        assert!(access.affects_decode(PCI_COMMAND, 4));
        assert!(access.affects_decode(0x42, 4));
        assert!(!access.affects_decode(0xe0, 4));
        for failure in ["none", "unmap", "write"] {
            let mut mappings = mappings();
            update(&mut mappings, &[test_bar()], true, |_| Ok(())).unwrap();
            let operations = RefCell::new(vec![]);
            let result = write_quiesced(
                &mut mappings,
                true,
                |region| {
                    assert_eq!(region.memory_size, 0);
                    operations.borrow_mut().push("unmap");
                    if failure == "unmap" {
                        Err(io::Error::other("unmap"))
                    } else {
                        Ok(())
                    }
                },
                || {
                    operations.borrow_mut().push("write");
                    if failure == "write" {
                        return Err(io::Error::other("write"));
                    }
                    Ok(())
                },
            );
            if failure == "unmap" {
                assert_eq!(*operations.borrow(), ["unmap"]);
                assert!(mappings.iter().all(|m| m.mapped_at.is_some()));
            } else {
                assert!(mappings.iter().all(|m| m.mapped_at.is_none()));
                assert_eq!(*operations.borrow(), ["unmap", "unmap", "write"]);
            }
            assert_eq!(result.is_ok(), failure == "none");
            remove(&mut mappings);
        }
    }
}

// SPDX-License-Identifier: Apache-2.0

use std::io;
use std::ops::Range;

/// Allocations are permanent for the lifetime of the PCI bus (no hotplug).
#[derive(Clone, Debug)]
pub struct PciResourceWindow {
    range: Range<u64>,
    next: u64,
}

impl PciResourceWindow {
    pub fn new(range: Range<u64>) -> Self {
        Self {
            next: range.start,
            range,
        }
    }

    pub fn range(&self) -> Range<u64> {
        self.range.clone()
    }

    pub fn allocate(&mut self, size: u64) -> io::Result<u64> {
        if !size.is_power_of_two() {
            return Err(io::Error::other("PCI resource size must be a power of two"));
        }
        let start = self.next.checked_add(size - 1).map(|v| v & !(size - 1));
        let end = start.and_then(|v| v.checked_add(size));
        match (start, end) {
            (Some(start), Some(end)) if end <= self.range.end => {
                self.next = end;
                Ok(start)
            }
            _ => Err(io::Error::other("PCI resource window exhausted")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounded_aligned_allocations_do_not_overlap() {
        let mut window = PciResourceWindow::new(0x1001..0x5000);
        assert_eq!(window.allocate(0x1000).unwrap(), 0x2000);
        assert_eq!(window.allocate(0x1000).unwrap(), 0x3000);
        assert!(window.allocate(0x2000).is_err());
        assert_eq!(window.allocate(0x1000).unwrap(), 0x4000);
        assert!(window.allocate(1).is_err());
    }

    #[test]
    fn invalid_sizes_and_overflow_leave_window_intact() {
        let mut window = PciResourceWindow::new(u64::MAX - 3..u64::MAX);
        for size in [0, 3, 8, 1 << 63] {
            assert!(window.allocate(size).is_err());
        }
        assert_eq!(window.allocate(2).unwrap(), u64::MAX - 3);
        assert!(
            PciResourceWindow::new(Range { start: 5, end: 4 })
                .allocate(1)
                .is_err()
        );
    }
}

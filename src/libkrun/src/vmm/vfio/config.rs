// Copyright 2026 The libkrun Authors.
// SPDX-License-Identifier: Apache-2.0

use super::*;

pub(super) struct ConfigAccess {
    pub emulated: [u8; CONFIG_SIZE],
    pub writable: [u8; CONFIG_SIZE],
    pub forwarded: [u8; PCI_CONVENTIONAL_CONFIG_SPACE_SIZE],
    pub pm: Option<usize>,
}

impl ConfigAccess {
    pub fn new() -> Self {
        let mut access = Self {
            emulated: [0; CONFIG_SIZE],
            writable: [0; CONFIG_SIZE],
            forwarded: [0xff; PCI_CONVENTIONAL_CONFIG_SPACE_SIZE],
            pm: None,
        };
        access.emulated[PCI_CONVENTIONAL_CONFIG_SPACE_SIZE..].fill(0xff);
        access.forwarded[..PCI_STD_HEADER_SIZEOF].fill(0);
        put16(&mut access.emulated, PCI_COMMAND, !GUEST_WRITABLE_COMMAND);
        put16(&mut access.forwarded, PCI_COMMAND, GUEST_WRITABLE_COMMAND);
        put16(&mut access.emulated, PCI_STATUS, PCI_STATUS_CAP_LIST);
        put16(&mut access.forwarded, PCI_STATUS, !PCI_STATUS_CAP_LIST);
        // Cache-line size and latency timer retain VFIO's write semantics.
        access.forwarded[PCI_CACHE_LINE_SIZE..PCI_HEADER_TYPE].fill(0xff);
        for range in [
            PCI_HEADER_TYPE..PCI_HEADER_TYPE + 1,
            PCI_BASE_ADDRESS_0..bar_register(PCI_STD_NUM_BARS),
            PCI_ROM_ADDRESS..PCI_ROM_ADDRESS + 4,
            PCI_CAPABILITY_LIST..PCI_CAPABILITY_LIST + 1,
            PCI_INTERRUPT_LINE..PCI_INTERRUPT_PIN + 1,
        ] {
            access.protect(range);
        }
        access
    }

    pub fn protect(&mut self, range: Range<usize>) {
        self.emulated[range.clone()].fill(0xff);
        self.forwarded[range].fill(0);
    }

    pub fn read(
        &self,
        shadow: &[u8],
        start: usize,
        data: &mut [u8],
        mut read: impl FnMut(usize, &mut [u8]) -> io::Result<()>,
    ) -> io::Result<()> {
        let live = data
            .len()
            .min(PCI_CONVENTIONAL_CONFIG_SPACE_SIZE.saturating_sub(start));
        data.fill(0);
        if self.emulated[start..start + live]
            .iter()
            .any(|mask| *mask != 0xff)
        {
            read(start, &mut data[..live])?;
        }
        for (i, byte) in data.iter_mut().enumerate() {
            let mask = self.emulated[start + i];
            *byte = (*byte & !mask) | (shadow[start + i] & mask);
        }
        Ok(())
    }

    pub fn write(
        &self,
        start: usize,
        data: &[u8],
        mut write: impl FnMut(usize, &[u8]) -> io::Result<()>,
    ) -> io::Result<()> {
        let mut offset = 0;
        while offset < data.len() {
            if self.forwarded[start + offset] == 0 {
                offset += 1;
                continue;
            }
            let from = offset;
            while offset < data.len() && self.forwarded[start + offset] != 0 {
                offset += 1;
            }
            let bytes: Vec<_> = data[from..offset]
                .iter()
                .enumerate()
                .map(|(i, byte)| byte & self.forwarded[start + from + i])
                .collect();
            // Only guest-supplied bits reach status registers; never read-modify-write W1C.
            write(start + from, &bytes)?;
        }
        Ok(())
    }

    pub fn emulate(&self, shadow: &mut [u8], start: usize, data: &[u8]) {
        for (i, byte) in data.iter().enumerate() {
            let mask = self.writable[start + i];
            shadow[start + i] = (shadow[start + i] & !mask) | (byte & mask);
        }
    }

    pub fn read_controls(
        &self,
        shadow: &mut [u8],
        mut read: impl FnMut(usize, &mut [u8]) -> io::Result<()>,
    ) -> io::Result<()> {
        for start in [Some(PCI_COMMAND), self.pm].into_iter().flatten() {
            let mut data = [0; 2];
            self.read(shadow, start, &mut data, &mut read)?;
            shadow[start..start + 2].copy_from_slice(&data);
        }
        Ok(())
    }

    pub fn affects_decode(&self, start: usize, length: usize) -> bool {
        [Some(PCI_COMMAND), self.pm]
            .into_iter()
            .flatten()
            .any(|offset| overlaps(offset as u64, 2, start as u64, length as u64))
    }

    pub fn decoding(&self, config: &[u8], enable: u16) -> bool {
        get16(config, PCI_COMMAND) & enable != 0
            && self
                .pm
                .is_none_or(|pm| get16(config, pm) & PCI_PM_CTRL_STATE_MASK == 0)
    }
}

pub(super) fn write_bars(config: &mut [u8], bars: &mut [Bar], start: usize, data: &[u8]) {
    for bar in bars {
        for half in 0..if bar.wide() { 2 } else { 1 } {
            let index = bar.index + half;
            let register = bar_register(index);
            let from = start.max(register);
            let to = (start + data.len()).min(register + 4);
            if from >= to {
                continue;
            }
            let mut bytes = get32(config, register).to_le_bytes();
            bytes[from - register..to - register].copy_from_slice(&data[from - start..to - start]);
            bar.write_address(index, u32::from_le_bytes(bytes));
            write_bar_register(config, bar);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn endpoint() -> ([u8; CONFIG_SIZE], ConfigAccess) {
        let mut config = [0; CONFIG_SIZE];
        put16(&mut config, PCI_STATUS, PCI_STATUS_CAP_LIST);
        config[PCI_CAPABILITY_LIST] = 0x40;
        config[0x40] = PCI_CAP_ID_PM;
        config[0x41] = 0x80;
        config[0x80] = PCI_CAP_ID_EXP;
        put16(&mut config, 0x82, 2);
        put32(&mut config, 0x84, PCI_EXP_DEVCAP_FLR);
        let mut access = ConfigAccess::new();
        capabilities(&mut config, &mut access).unwrap();
        (config, access)
    }

    #[test]
    fn vfio_live_reads_and_vendor_selector_data() {
        let (config, access) = endpoint();
        let mut selector = 0;
        for value in [3u32, 9] {
            access
                .write(0xe0, &value.to_le_bytes(), |offset, bytes| {
                    assert_eq!((offset, bytes.len()), (0xe0, 4));
                    selector = get32(bytes, 0);
                    Ok(())
                })
                .unwrap();
            let mut data = [0; 4];
            access
                .read(&config, 0xe4, &mut data, |offset, bytes| {
                    assert_eq!((offset, bytes.len()), (0xe4, 4));
                    bytes.copy_from_slice(&(selector * 2).to_le_bytes());
                    Ok(())
                })
                .unwrap();
            assert_eq!(get32(&data, 0), value * 2);
        }
        let mut status = [0; 2];
        for value in [0x8000u16, 0x4000] {
            access
                .read(&config, PCI_STATUS, &mut status, |_, bytes| {
                    bytes.copy_from_slice(&value.to_le_bytes());
                    Ok(())
                })
                .unwrap();
            assert_eq!(get16(&status, 0), value | PCI_STATUS_CAP_LIST);
        }
    }

    #[test]
    fn vfio_config_protects_owned_fields_and_hidden_capability_bounds() {
        let mut config = [0; CONFIG_SIZE];
        put16(&mut config, PCI_STATUS, PCI_STATUS_CAP_LIST);
        config[PCI_CAPABILITY_LIST] = 0x60;
        config[0x60] = PCI_CAP_ID_VNDR;
        config[0x61] = 0x40;
        config[0x62] = 8;
        config[0x40] = 0xfe;
        config[0x41] = 0x80;
        config[0x80] = PCI_CAP_ID_EXP;
        put16(&mut config, 0x82, 2);
        put32(&mut config, 0x84, PCI_EXP_DEVCAP_FLR);
        put16(&mut config, 0x88, PCI_EXP_DEVCTL_BCR_FLR);
        let mut access = ConfigAccess::new();
        capabilities(&mut config, &mut access).unwrap();
        assert_eq!(config[PCI_CAPABILITY_LIST], 0x80);
        assert!(access.forwarded[0x40..0x68].iter().all(|byte| *byte == 0));
        assert!(
            access.forwarded[0x68..0x80]
                .iter()
                .all(|byte| *byte == 0xff)
        );
        for (start, size) in [(0x10, 24), (0x30, 5), (0x3c, 2), (0x40, 40), (0x80, 4)] {
            access
                .write(start, &vec![0xff; size], |_, _| panic!("protected write"))
                .unwrap();
        }
        let mut extended = [0xff; 4];
        access
            .read(&config, 256, &mut extended, |_, _| panic!("extended read"))
            .unwrap();
        assert_eq!(extended, [0; 4]);
        let mut data = [0; 4];
        access
            .read(&config, 0x84, &mut data, |_, bytes| {
                bytes.fill(0xff);
                Ok(())
            })
            .unwrap();
        assert_eq!(get32(&data, 0), !PCI_EXP_DEVCAP_FLR);
        access
            .read(&config, 0x88, &mut data[..2], |_, bytes| {
                bytes.fill(0xff);
                Ok(())
            })
            .unwrap();
        assert_eq!(get16(&data, 0), !PCI_EXP_DEVCTL_BCR_FLR);
    }

    #[test]
    fn vfio_config_spanning_writes_preserve_width_and_status_input() {
        let (config, access) = endpoint();
        for (start, input, expected) in [
            (
                PCI_COMMAND,
                [0xff, 0xff, 0, 0x80],
                vec![(PCI_COMMAND, vec![0x47, 0x05, 0, 0x80])],
            ),
            (0x40, [0xff, 0xff, 0xff, 0xff], vec![]),
            (0x42, [0xff, 0xff, 3, 0x80], vec![(0x44, vec![3, 0x80])]),
            (
                0x88,
                [0xff, 0xff, 0x05, 0],
                vec![(0x88, vec![0xff, 0x7f, 0x05, 0])],
            ),
            (0xa8, [1, 2, 3, 4], vec![(0xa8, vec![1, 2, 3, 4])]),
        ] {
            let mut writes = vec![];
            access
                .write(start, &input, |offset, bytes| {
                    writes.push((offset, bytes.to_vec()));
                    Ok(())
                })
                .unwrap();
            assert_eq!(writes, expected);
        }
        // A backend may filter W1C writes. Reads must still report its status.
        access.write(PCI_STATUS, &[0, 0x80], |_, _| Ok(())).unwrap();
        let mut data = [0; 2];
        access
            .read(&config, PCI_STATUS, &mut data, |_, bytes| {
                bytes.copy_from_slice(&[0, 0x80]);
                Ok(())
            })
            .unwrap();
        assert_eq!(data, [PCI_STATUS_CAP_LIST as u8, 0x80]);
        access
            .write(PCI_COMMAND, &[7], |offset, bytes| {
                assert_eq!((offset, bytes), (PCI_COMMAND, &[7][..]));
                Ok(())
            })
            .unwrap();
    }

    #[test]
    fn vfio_config_io_failures_and_short_transfers_propagate() {
        let (config, access) = endpoint();
        for count in [-1, 3] {
            assert!(
                access
                    .read(&config, 0xe0, &mut [0; 4], |_, data| {
                        transferred(count, data.len(), "read")
                    })
                    .is_err()
            );
            assert!(
                access
                    .write(0xe0, &[0; 4], |_, data| {
                        transferred(count, data.len(), "write")
                    })
                    .is_err()
            );
        }
        transferred(4, 4, "read").unwrap();
        assert!(
            access
                .read_controls(&mut config.clone(), |_, _| Err(io::Error::other(
                    "readback"
                )))
                .is_err()
        );
    }

    #[test]
    fn vfio_power_gate_uses_backend_state_and_independent_decode_bits() {
        let (mut config, access) = endpoint();
        for state in 0u16..4 {
            for command in 0u16..4 {
                // The backend can reject a requested transition to D0.
                access.write(0x44, &[0, 0], |_, _| Ok(())).unwrap();
                access
                    .read_controls(&mut config, |offset, bytes| {
                        bytes.copy_from_slice(
                            &if offset == PCI_COMMAND {
                                command
                            } else {
                                state
                            }
                            .to_le_bytes(),
                        );
                        Ok(())
                    })
                    .unwrap();
                assert_eq!(
                    access.decoding(&config, PCI_COMMAND_IO),
                    state == 0 && command & 1 != 0
                );
                assert_eq!(
                    access.decoding(&config, PCI_COMMAND_MEMORY),
                    state == 0 && command & 2 != 0
                );
            }
        }
    }

    #[test]
    fn vfio_partial_bar_writes_match_dwords_and_preserve_flags() {
        for (size, address, probe) in [
            (0x1000, 0x19_1234_5000, 0xffff_ffff_ffff_f000),
            (8 << 30, 0x18_0000_0000, 0xffff_fffe_0000_0000),
        ] {
            for width in [1, 2, 4] {
                let mut config = [0; CONFIG_SIZE];
                let mut bars = [Bar {
                    index: 0,
                    flags: PCI_BASE_ADDRESS_MEM_TYPE_64,
                    size,
                    address: 0x10_0000_0000,
                    window: 0x10_0000_0000..0x20_0000_0000,
                }];
                write_bar_register(&mut config, &bars[0]);
                for (input, expected, valid) in [
                    (0x19_1234_5fffu64, address, true),
                    (u64::MAX, probe, false),
                    (0x20_0000_0000, 0x20_0000_0000, false),
                    (0, 0, false),
                    (0x10_0000_0000, 0x10_0000_0000, true),
                    (0x19_1234_5fff, address, true),
                ] {
                    let high = get32(&config, 0x14);
                    for (i, bytes) in input.to_le_bytes().chunks(width).enumerate() {
                        write_bars(&mut config, &mut bars, 0x10 + i * width, bytes);
                        if (i + 1) * width == 4 {
                            assert_eq!(get32(&config, 0x14), high);
                        }
                    }
                    assert_eq!(bars[0].address, expected);
                    assert_eq!(
                        get32(&config, 0x10),
                        expected as u32 | PCI_BASE_ADDRESS_MEM_TYPE_64
                    );
                    assert_eq!(get32(&config, 0x14), (expected >> 32) as u32);
                    assert_eq!(bars[0].bounds(), valid);
                }
                bars[0].address += 1;
                assert!(!bars[0].bounds());
            }
        }
    }
}

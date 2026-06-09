//! CRC helpers used by CRSF and direct commands.

/// Generic CRC-8 calculator.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct Crc8<const POLY: u8>(u8);

impl<const POLY: u8> Crc8<POLY> {
    /// Creates a zero-initialized CRC accumulator.
    pub const fn new() -> Self {
        Self(0)
    }

    /// Feeds one byte into the CRC accumulator.
    pub fn update(&mut self, byte: u8) {
        let mut crc = self.0 ^ byte;
        let mut i = 0;
        while i < 8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ POLY;
            } else {
                crc <<= 1;
            }
            i += 1;
        }
        self.0 = crc;
    }

    /// Computes a CRC over a full byte slice.
    pub fn compute(bytes: &[u8]) -> u8 {
        let mut crc = Self::new();
        let mut i = 0;
        while i < bytes.len() {
            crc.update(bytes[i]);
            i += 1;
        }
        crc.0
    }

    /// Returns the current CRC value.
    pub const fn value(self) -> u8 {
        self.0
    }
}

/// CRC used by CRSF frames.
pub type FrameCrc = Crc8<0xD5>;

/// CRC used by direct command payloads.
pub type CommandCrc = Crc8<0xBA>;

#[cfg(test)]
mod tests {
    use super::{CommandCrc, FrameCrc};

    #[test]
    fn frame_crc_matches_known_example() {
        let bytes = [0x16, 0xE0, 0x03];
        assert_eq!(FrameCrc::compute(&bytes), 0xB2);
    }

    #[test]
    fn command_crc_is_deterministic() {
        let bytes = [0x32, 0xEC, 0xEE, 0x10, 0x01];
        assert_eq!(CommandCrc::compute(&bytes), CommandCrc::compute(&bytes));
    }
}

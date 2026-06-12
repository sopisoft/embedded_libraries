use crate::Measurement;

/// Sliding parser for the TSD10 active 4-byte UART frame stream.
#[derive(Debug, Default)]
pub struct FrameParser {
    window: [u8; 4],
    len: usize,
}

impl FrameParser {
    /// Creates a new parser with no buffered bytes.
    pub const fn new() -> Self {
        Self {
            window: [0; 4],
            len: 0,
        }
    }

    /// Drops any partially collected bytes.
    pub fn reset(&mut self) {
        self.len = 0;
    }

    /// Pushes one byte from the UART stream.
    ///
    /// Returns `Some(Measurement)` only when the newest 4-byte window is a
    /// valid sensor frame.
    pub fn push(&mut self, byte: u8) -> Option<Measurement> {
        if self.len < self.window.len() {
            self.window[self.len] = byte;
            self.len += 1;
        } else {
            self.window.copy_within(1.., 0);
            self.window[self.window.len() - 1] = byte;
        }

        if self.len < self.window.len() {
            return None;
        }

        Measurement::from_frame(self.window).ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parser_resynchronizes_after_noise() {
        let mut parser = FrameParser::new();
        let bytes = [0x00, 0xFF, 0x5C, 0x02, 0x11, 0xEC];

        let mut measurement = None;
        for byte in bytes {
            if let Some(parsed) = parser.push(byte) {
                measurement = Some(parsed);
            }
        }

        let measurement = measurement.unwrap();
        assert_eq!(measurement.distance_mm, 4_354);
    }

    #[test]
    fn parser_continues_after_one_valid_frame() {
        let mut parser = FrameParser::new();
        let bytes = [0x5C, 0xD2, 0x04, 0x29, 0x5C, 0x34, 0x12, 0xB9];
        let mut hits = [0u16; 2];
        let mut count = 0usize;

        for byte in bytes {
            if let Some(parsed) = parser.push(byte) {
                hits[count] = parsed.distance_mm;
                count += 1;
            }
        }

        assert_eq!(count, 2);
        assert_eq!(hits, [1_234, 4_660]);
    }
}

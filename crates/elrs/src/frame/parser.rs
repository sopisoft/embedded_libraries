use crate::{DeviceAddress, crc::FrameCrc};

use super::{Frame, MAX_BODY_LEN, MAX_FRAME_SIZE, MAX_LENGTH_FIELD};

/// Errors returned while parsing a frame stream.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ParseError {
    /// The received length field is outside the valid CRSF range.
    InvalidLength(u8),
    /// The computed CRC did not match the frame CRC.
    CrcMismatch { expected: u8, actual: u8 },
    /// The parsed body exceeded the supported buffer length.
    BodyTooLong,
}

/// Incremental CRSF parser for UART byte streams.
#[derive(Debug)]
pub struct FrameParser {
    buffer: [u8; MAX_FRAME_SIZE],
    len: usize,
    expected_total: usize,
}

impl FrameParser {
    /// Creates a fresh parser.
    pub const fn new() -> Self {
        Self {
            buffer: [0; MAX_FRAME_SIZE],
            len: 0,
            expected_total: 0,
        }
    }

    /// Resets the parser state.
    pub fn reset(&mut self) {
        self.len = 0;
        self.expected_total = 0;
    }

    /// Pushes one byte into the parser.
    pub fn push(&mut self, byte: u8) -> Option<Result<Frame, ParseError>> {
        if self.len == 0 && !is_sync_byte(byte) {
            return None;
        }
        if self.len >= MAX_FRAME_SIZE {
            self.reset();
        }
        self.buffer[self.len] = byte;
        self.len += 1;

        if self.len == 2 {
            let length = self.buffer[1];
            if !(2..=MAX_LENGTH_FIELD as u8).contains(&length) {
                self.reset();
                return Some(Err(ParseError::InvalidLength(length)));
            }
            self.expected_total = length as usize + 2;
        }

        if self.expected_total != 0 && self.len == self.expected_total {
            let result = self.finish_frame();
            self.reset();
            return Some(result);
        }
        None
    }

    fn finish_frame(&self) -> Result<Frame, ParseError> {
        let body_len = self.buffer[1] as usize - 2;
        if body_len > MAX_BODY_LEN {
            return Err(ParseError::BodyTooLong);
        }

        let actual_crc = self.buffer[3 + body_len];
        let expected_crc = FrameCrc::compute(&self.buffer[2..3 + body_len]);
        if actual_crc != expected_crc {
            return Err(ParseError::CrcMismatch {
                expected: expected_crc,
                actual: actual_crc,
            });
        }

        Frame::from_body(
            DeviceAddress::new(self.buffer[0]),
            self.buffer[2],
            &self.buffer[3..3 + body_len],
        )
        .map_err(|_| ParseError::BodyTooLong)
    }
}

fn is_sync_byte(byte: u8) -> bool {
    matches!(
        byte,
        0x00 | 0x0E | 0x10 | 0x12 | 0x13 | 0x14 | 0x80 | 0x90
            ..=0x97 | 0xC0 | 0xC2 | 0xC4 | 0xC8 | 0xCC | 0xCE | 0xEA | 0xEB | 0xEC | 0xED | 0xEE
    )
}

impl Default for FrameParser {
    fn default() -> Self {
        Self::new()
    }
}

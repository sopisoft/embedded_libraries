/// TSD10 active measurement frame header.
pub const FRAME_HEADER: u8 = 0x5C;
/// TSD10 command / reply frame header.
pub const COMMAND_HEADER: u8 = 0x5A;
/// Value reported by the sensor when the target is out of range.
pub const OUT_OF_RANGE_MM: u16 = u16::MAX;
/// Factory-default UART speed described in the TSD10 manual.
pub const DEFAULT_BAUD_RATE: BaudRate = BaudRate::B460800;

/// UART baud rates supported by the sensor.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum BaudRate {
    B9600,
    B19200,
    B38400,
    B115200,
    B230400,
    B256000,
    B460800,
}

impl BaudRate {
    /// Returns the baud rate as an integer.
    pub const fn as_u32(self) -> u32 {
        match self {
            Self::B9600 => 9_600,
            Self::B19200 => 19_200,
            Self::B38400 => 38_400,
            Self::B115200 => 115_200,
            Self::B230400 => 230_400,
            Self::B256000 => 256_000,
            Self::B460800 => 460_800,
        }
    }

    pub(crate) const fn payload_bytes(self) -> [u8; 2] {
        match self {
            Self::B9600 => [0x60, 0x00],
            Self::B19200 => [0xC0, 0x00],
            Self::B38400 => [0x80, 0x01],
            Self::B115200 => [0x80, 0x04],
            Self::B230400 => [0x00, 0x09],
            Self::B256000 => [0x00, 0x0A],
            Self::B460800 => [0x00, 0x12],
        }
    }
}

/// One TSD10 distance sample.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Measurement {
    pub distance_mm: u16,
}

impl Measurement {
    /// Parses one exact 4-byte sensor frame.
    pub fn from_frame(frame: [u8; 4]) -> Result<Self, ParseError> {
        if frame[0] != FRAME_HEADER {
            return Err(ParseError::InvalidHeader(frame[0]));
        }

        let expected = checksum(&frame[1..3]);
        if frame[3] != expected {
            return Err(ParseError::InvalidChecksum {
                expected,
                actual: frame[3],
            });
        }

        Ok(Self {
            distance_mm: u16::from_le_bytes([frame[1], frame[2]]),
        })
    }

    /// Returns `true` when the sensor reports no valid target.
    pub const fn is_out_of_range(&self) -> bool {
        self.distance_mm == OUT_OF_RANGE_MM
    }

    /// Returns the measured distance in meters when it is in range.
    pub fn distance_m(&self) -> Option<f32> {
        if self.is_out_of_range() {
            None
        } else {
            Some(self.distance_mm as f32 / 1000.0)
        }
    }
}

/// Parsing error for one exact frame.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ParseError {
    InvalidHeader(u8),
    InvalidChecksum { expected: u8, actual: u8 },
}

/// Driver error.
#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    /// UART transport error.
    Io(E),
    /// End-of-stream before a complete frame arrived.
    UnexpectedEof,
    /// A valid reply frame was received, but it did not match the requested command.
    UnexpectedReply {
        expected_command: u8,
        actual_command: u8,
        payload: [u8; 2],
    },
}

pub(crate) fn checksum(bytes: &[u8]) -> u8 {
    let mut sum = 0u8;
    let mut index = 0usize;
    while index < bytes.len() {
        sum = sum.wrapping_add(bytes[index]);
        index += 1;
    }
    !sum
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn measurement_parses_manual_example_frame() {
        let measurement = Measurement::from_frame([FRAME_HEADER, 0x02, 0x11, 0xEC]).unwrap();
        assert_eq!(measurement.distance_mm, 4_354);
        assert_eq!(measurement.distance_m(), Some(4.354));
    }

    #[test]
    fn out_of_range_frame_is_detected() {
        let measurement = Measurement::from_frame([FRAME_HEADER, 0xFF, 0xFF, 0x01]).unwrap();
        assert!(measurement.is_out_of_range());
        assert_eq!(measurement.distance_m(), None);
    }
}

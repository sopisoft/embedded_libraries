/// Errors returned by telemetry helpers.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TelemetryError {
    /// The payload length was invalid for the telemetry type.
    InvalidLength,
    /// The string payload was invalid UTF-8.
    InvalidText,
    /// The text would not fit into the payload buffer.
    TextTooLong,
}

pub(crate) fn read_be_i16(bytes: &[u8]) -> i16 {
    i16::from_be_bytes([bytes[0], bytes[1]])
}

pub(crate) fn read_be_u16(bytes: &[u8]) -> u16 {
    u16::from_be_bytes([bytes[0], bytes[1]])
}

pub(crate) fn read_be_i32(bytes: &[u8]) -> i32 {
    i32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
}

pub(crate) fn write_be_u24(value: u32, out: &mut [u8]) {
    out[0] = ((value >> 16) & 0xFF) as u8;
    out[1] = ((value >> 8) & 0xFF) as u8;
    out[2] = (value & 0xFF) as u8;
}

pub(crate) fn read_be_u24(bytes: &[u8]) -> u32 {
    ((bytes[0] as u32) << 16) | ((bytes[1] as u32) << 8) | bytes[2] as u32
}

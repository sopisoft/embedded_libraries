use core::str;

use heapless::Vec;

use super::TelemetryError;

/// Flight mode view over a null-terminated payload.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct FlightMode<'a> {
    payload: &'a [u8],
}

impl<'a> FlightMode<'a> {
    /// Creates a borrowed view over a flight mode payload.
    pub const fn new(payload: &'a [u8]) -> Self {
        Self { payload }
    }

    /// Returns the raw payload bytes.
    pub const fn raw(&self) -> &'a [u8] {
        self.payload
    }

    /// Returns the decoded UTF-8 flight mode string.
    pub fn as_str(&self) -> Result<&'a str, TelemetryError> {
        let mut len = 0usize;
        while len < self.payload.len() && self.payload[len] != 0 {
            len += 1;
        }
        str::from_utf8(&self.payload[..len]).map_err(|_| TelemetryError::InvalidText)
    }
}

/// Encodes a null-terminated flight mode payload.
pub fn encode_flight_mode(mode: &str) -> Result<Vec<u8, 60>, TelemetryError> {
    let mut out = Vec::new();
    out.extend_from_slice(mode.as_bytes())
        .map_err(|_| TelemetryError::TextTooLong)?;
    out.push(0).map_err(|_| TelemetryError::TextTooLong)?;
    Ok(out)
}

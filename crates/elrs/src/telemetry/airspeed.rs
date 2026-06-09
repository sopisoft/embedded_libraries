use super::TelemetryError;
use super::shared::read_be_u16;

/// Airspeed telemetry payload.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Airspeed {
    /// Airspeed in `0.1 km/h`.
    pub speed_kmh_x10: u16,
}

impl Airspeed {
    /// Payload length in bytes.
    pub const LEN: usize = 2;

    /// Decodes an airspeed payload.
    pub fn decode(payload: &[u8]) -> Result<Self, TelemetryError> {
        if payload.len() != Self::LEN {
            return Err(TelemetryError::InvalidLength);
        }
        Ok(Self {
            speed_kmh_x10: read_be_u16(payload),
        })
    }

    /// Encodes the payload.
    pub fn encode_payload(&self) -> [u8; Self::LEN] {
        self.speed_kmh_x10.to_be_bytes()
    }
}

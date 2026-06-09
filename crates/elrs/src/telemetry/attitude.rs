use libm::roundf;

use super::TelemetryError;
use super::shared::read_be_i16;

/// Attitude telemetry payload with angles in radians.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Attitude {
    /// Pitch in radians.
    pub pitch_rad: f32,
    /// Roll in radians.
    pub roll_rad: f32,
    /// Yaw in radians.
    pub yaw_rad: f32,
}

impl Attitude {
    /// Payload length in bytes.
    pub const LEN: usize = 6;

    /// Decodes the payload.
    pub fn decode(payload: &[u8]) -> Result<Self, TelemetryError> {
        if payload.len() != Self::LEN {
            return Err(TelemetryError::InvalidLength);
        }
        Ok(Self {
            pitch_rad: read_be_i16(&payload[0..2]) as f32 / 10_000.0,
            roll_rad: read_be_i16(&payload[2..4]) as f32 / 10_000.0,
            yaw_rad: read_be_i16(&payload[4..6]) as f32 / 10_000.0,
        })
    }

    /// Encodes the payload.
    pub fn encode_payload(&self) -> [u8; Self::LEN] {
        let mut out = [0u8; Self::LEN];
        let pitch = roundf(self.pitch_rad * 10_000.0) as i16;
        let roll = roundf(self.roll_rad * 10_000.0) as i16;
        let yaw = roundf(self.yaw_rad * 10_000.0) as i16;
        out[0..2].copy_from_slice(&pitch.to_be_bytes());
        out[2..4].copy_from_slice(&roll.to_be_bytes());
        out[4..6].copy_from_slice(&yaw.to_be_bytes());
        out
    }
}

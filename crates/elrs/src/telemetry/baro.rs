use libm::{expf, fabsf, logf, roundf};

use super::TelemetryError;
use super::shared::read_be_u16;

/// Barometric altitude and vertical speed payload.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct BarometricAltitude {
    /// Packed CRSF altitude field.
    pub altitude_packed: u16,
    /// Packed CRSF vertical speed field.
    pub vertical_speed_packed: i8,
}

impl BarometricAltitude {
    /// Payload length in bytes.
    pub const LEN: usize = 3;

    /// Decodes a barometric payload.
    pub fn decode(payload: &[u8]) -> Result<Self, TelemetryError> {
        if payload.len() != Self::LEN {
            return Err(TelemetryError::InvalidLength);
        }
        Ok(Self {
            altitude_packed: read_be_u16(&payload[0..2]),
            vertical_speed_packed: payload[2] as i8,
        })
    }

    /// Encodes the payload.
    pub fn encode_payload(&self) -> [u8; Self::LEN] {
        let mut out = [0u8; Self::LEN];
        out[0..2].copy_from_slice(&self.altitude_packed.to_be_bytes());
        out[2] = self.vertical_speed_packed as u8;
        out
    }

    /// Returns altitude in decimeters.
    pub fn altitude_dm(&self) -> i32 {
        if (self.altitude_packed & 0x8000) != 0 {
            (self.altitude_packed & 0x7FFF) as i32 * 10
        } else {
            self.altitude_packed as i32 - 10_000
        }
    }

    /// Packs altitude in decimeters according to the CRSF format.
    pub fn pack_altitude_dm(altitude_dm: i32) -> u16 {
        const ALT_MIN_DM: i32 = 10_000;
        const ALT_THRESHOLD_DM: i32 = 0x8000 - ALT_MIN_DM;
        const ALT_MAX_DM: i32 = 0x7FFE * 10 - 5;

        if altitude_dm < -ALT_MIN_DM {
            0
        } else if altitude_dm > ALT_MAX_DM {
            0xFFFE
        } else if altitude_dm < ALT_THRESHOLD_DM {
            (altitude_dm + ALT_MIN_DM) as u16
        } else {
            (((altitude_dm + 5) / 10) as u16) | 0x8000
        }
    }

    /// Unpacks the logarithmic CRSF vertical speed field into centimeters per second.
    pub fn vertical_speed_cm_s(&self) -> i16 {
        Self::unpack_vertical_speed(self.vertical_speed_packed)
    }

    /// Packs vertical speed in centimeters per second.
    pub fn pack_vertical_speed(vertical_speed_cm_s: i16) -> i8 {
        const KL: f32 = 100.0;
        const KR: f32 = 0.026;

        if vertical_speed_cm_s == 0 {
            return 0;
        }
        let sign = if vertical_speed_cm_s < 0 { -1.0 } else { 1.0 };
        let magnitude = fabsf(vertical_speed_cm_s as f32);
        (roundf(logf(magnitude / KL + 1.0) / KR) * sign) as i8
    }

    /// Unpacks vertical speed in centimeters per second.
    pub fn unpack_vertical_speed(vertical_speed_packed: i8) -> i16 {
        const KL: f32 = 100.0;
        const KR: f32 = 0.026;

        if vertical_speed_packed == 0 {
            return 0;
        }
        let sign = if vertical_speed_packed < 0 { -1.0 } else { 1.0 };
        let magnitude = fabsf(vertical_speed_packed as f32);
        (roundf((expf(magnitude * KR) - 1.0) * KL) * sign) as i16
    }
}

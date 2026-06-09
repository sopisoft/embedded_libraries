use super::TelemetryError;
use super::shared::{read_be_i32, read_be_u16};

/// GPS telemetry payload.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Gps {
    /// Latitude in degrees times `1e7`.
    pub latitude_e7: i32,
    /// Longitude in degrees times `1e7`.
    pub longitude_e7: i32,
    /// Ground speed in `0.01 km/h`.
    pub groundspeed_kmh_x100: u16,
    /// Heading in `0.01 deg`.
    pub heading_deg_x100: u16,
    /// Altitude in meters with CRSF `+1000` offset applied.
    pub altitude_m_offset_1000: u16,
    /// Satellites in use.
    pub satellites: u8,
}

impl Gps {
    /// Payload length in bytes.
    pub const LEN: usize = 15;

    /// Decodes a GPS payload.
    pub fn decode(payload: &[u8]) -> Result<Self, TelemetryError> {
        if payload.len() != Self::LEN {
            return Err(TelemetryError::InvalidLength);
        }
        Ok(Self {
            latitude_e7: read_be_i32(&payload[0..4]),
            longitude_e7: read_be_i32(&payload[4..8]),
            groundspeed_kmh_x100: read_be_u16(&payload[8..10]),
            heading_deg_x100: read_be_u16(&payload[10..12]),
            altitude_m_offset_1000: read_be_u16(&payload[12..14]),
            satellites: payload[14],
        })
    }

    /// Encodes the payload.
    pub fn encode_payload(&self) -> [u8; Self::LEN] {
        let mut out = [0u8; Self::LEN];
        out[0..4].copy_from_slice(&self.latitude_e7.to_be_bytes());
        out[4..8].copy_from_slice(&self.longitude_e7.to_be_bytes());
        out[8..10].copy_from_slice(&self.groundspeed_kmh_x100.to_be_bytes());
        out[10..12].copy_from_slice(&self.heading_deg_x100.to_be_bytes());
        out[12..14].copy_from_slice(&self.altitude_m_offset_1000.to_be_bytes());
        out[14] = self.satellites;
        out
    }

    /// Returns altitude in meters relative to the CRSF zero level.
    pub fn altitude_m(&self) -> i32 {
        self.altitude_m_offset_1000 as i32 - 1000
    }
}

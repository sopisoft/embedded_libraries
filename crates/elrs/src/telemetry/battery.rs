use super::TelemetryError;
use super::shared::{read_be_u16, read_be_u24, write_be_u24};

/// Battery telemetry payload.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct BatterySensor {
    /// Voltage in volts times `100`.
    pub voltage_v_x100: u16,
    /// Current in amps times `100`.
    pub current_a_x100: u16,
    /// Used capacity in mAh.
    pub capacity_mah: u32,
    /// Remaining battery percentage.
    pub remaining_percent: u8,
}

impl BatterySensor {
    /// Payload length in bytes.
    pub const LEN: usize = 8;

    /// Decodes a battery payload.
    pub fn decode(payload: &[u8]) -> Result<Self, TelemetryError> {
        if payload.len() != Self::LEN {
            return Err(TelemetryError::InvalidLength);
        }
        Ok(Self {
            voltage_v_x100: read_be_u16(&payload[0..2]),
            current_a_x100: read_be_u16(&payload[2..4]),
            capacity_mah: read_be_u24(&payload[4..7]),
            remaining_percent: payload[7],
        })
    }

    /// Encodes the payload.
    pub fn encode_payload(&self) -> [u8; Self::LEN] {
        let mut out = [0u8; Self::LEN];
        out[0..2].copy_from_slice(&self.voltage_v_x100.to_be_bytes());
        out[2..4].copy_from_slice(&self.current_a_x100.to_be_bytes());
        write_be_u24(self.capacity_mah.min(0x00FF_FFFF), &mut out[4..7]);
        out[7] = self.remaining_percent;
        out
    }

    /// Returns voltage in volts.
    pub fn voltage_v(&self) -> f32 {
        self.voltage_v_x100 as f32 / 100.0
    }

    /// Returns current in amps.
    pub fn current_a(&self) -> f32 {
        self.current_a_x100 as f32 / 100.0
    }
}

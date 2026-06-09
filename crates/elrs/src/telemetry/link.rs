use super::TelemetryError;

/// Standard link statistics payload.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct LinkStatistics {
    pub up_rssi_ant1: u8,
    pub up_rssi_ant2: u8,
    pub up_link_quality: u8,
    pub up_snr: i8,
    pub active_antenna: u8,
    pub rf_profile: u8,
    pub up_rf_power: u8,
    pub down_rssi: u8,
    pub down_link_quality: u8,
    pub down_snr: i8,
}

impl LinkStatistics {
    /// Payload length in bytes.
    pub const LEN: usize = 10;

    /// Decodes the payload.
    pub fn decode(payload: &[u8]) -> Result<Self, TelemetryError> {
        if payload.len() != Self::LEN {
            return Err(TelemetryError::InvalidLength);
        }
        Ok(Self {
            up_rssi_ant1: payload[0],
            up_rssi_ant2: payload[1],
            up_link_quality: payload[2],
            up_snr: payload[3] as i8,
            active_antenna: payload[4],
            rf_profile: payload[5],
            up_rf_power: payload[6],
            down_rssi: payload[7],
            down_link_quality: payload[8],
            down_snr: payload[9] as i8,
        })
    }

    /// Encodes the payload.
    pub fn encode_payload(&self) -> [u8; Self::LEN] {
        [
            self.up_rssi_ant1,
            self.up_rssi_ant2,
            self.up_link_quality,
            self.up_snr as u8,
            self.active_antenna,
            self.rf_profile,
            self.up_rf_power,
            self.down_rssi,
            self.down_link_quality,
            self.down_snr as u8,
        ]
    }
}

/// Receiver-side link statistics payload.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct LinkStatisticsRx {
    pub rssi_db: u8,
    pub rssi_percent: u8,
    pub link_quality: u8,
    pub snr: i8,
    pub rf_power_db: u8,
}

impl LinkStatisticsRx {
    /// Payload length in bytes.
    pub const LEN: usize = 5;

    /// Decodes the payload.
    pub fn decode(payload: &[u8]) -> Result<Self, TelemetryError> {
        if payload.len() != Self::LEN {
            return Err(TelemetryError::InvalidLength);
        }
        Ok(Self {
            rssi_db: payload[0],
            rssi_percent: payload[1],
            link_quality: payload[2],
            snr: payload[3] as i8,
            rf_power_db: payload[4],
        })
    }

    /// Encodes the payload.
    pub fn encode_payload(&self) -> [u8; Self::LEN] {
        [
            self.rssi_db,
            self.rssi_percent,
            self.link_quality,
            self.snr as u8,
            self.rf_power_db,
        ]
    }
}

/// Transmitter-side link statistics payload.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct LinkStatisticsTx {
    pub rssi_db: u8,
    pub rssi_percent: u8,
    pub link_quality: u8,
    pub snr: i8,
    pub rf_power_db: u8,
    pub fps_div_10: u8,
}

impl LinkStatisticsTx {
    /// Payload length in bytes.
    pub const LEN: usize = 6;

    /// Decodes the payload.
    pub fn decode(payload: &[u8]) -> Result<Self, TelemetryError> {
        if payload.len() != Self::LEN {
            return Err(TelemetryError::InvalidLength);
        }
        Ok(Self {
            rssi_db: payload[0],
            rssi_percent: payload[1],
            link_quality: payload[2],
            snr: payload[3] as i8,
            rf_power_db: payload[4],
            fps_div_10: payload[5],
        })
    }

    /// Encodes the payload.
    pub fn encode_payload(&self) -> [u8; Self::LEN] {
        [
            self.rssi_db,
            self.rssi_percent,
            self.link_quality,
            self.snr as u8,
            self.rf_power_db,
            self.fps_div_10,
        ]
    }
}

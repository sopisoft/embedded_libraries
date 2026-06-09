use heapless::Vec;
use libm::roundf;

use crate::{
    DeviceAddress,
    frame::{FRAME_TYPE_SUBSET_RC_CHANNELS_PACKED, Frame},
};

use super::{CHANNEL_COUNT, RcError, micros_to_ticks};

/// Supported subset RC resolutions.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SubsetResolution {
    /// 10-bit subset payload.
    Bits10,
    /// 11-bit subset payload.
    Bits11,
    /// 12-bit subset payload.
    Bits12,
    /// 13-bit subset payload.
    Bits13,
}

impl SubsetResolution {
    /// Returns the wire encoding used in the header bitfield.
    pub const fn config_bits(self) -> u8 {
        match self {
            Self::Bits10 => 0,
            Self::Bits11 => 1,
            Self::Bits12 => 2,
            Self::Bits13 => 3,
        }
    }

    /// Returns the number of stored bits per channel.
    pub const fn bits_per_channel(self) -> usize {
        match self {
            Self::Bits10 => 10,
            Self::Bits11 => 11,
            Self::Bits12 => 12,
            Self::Bits13 => 13,
        }
    }

    /// Parses the header configuration bits.
    pub const fn from_config(value: u8) -> Option<Self> {
        match value & 0x03 {
            0 => Some(Self::Bits10),
            1 => Some(Self::Bits11),
            2 => Some(Self::Bits12),
            3 => Some(Self::Bits13),
            _ => None,
        }
    }

    fn raw_to_ticks(self, raw: u16) -> u16 {
        let scale = match self {
            Self::Bits10 => 1.0,
            Self::Bits11 => 0.5,
            Self::Bits12 => 0.25,
            Self::Bits13 => 0.125,
        };
        roundf(raw as f32 * scale + 988.0) as u16
    }

    fn ticks_to_raw(self, ticks: u16) -> u16 {
        let scale = match self {
            Self::Bits10 => 1.0,
            Self::Bits11 => 0.5,
            Self::Bits12 => 0.25,
            Self::Bits13 => 0.125,
        };
        let raw = roundf((ticks as f32 - 988.0) / scale);
        let max = (1u16 << self.bits_per_channel()) - 1;
        raw.clamp(0.0, max as f32) as u16
    }
}

/// Subset RC payload for frame type `0x17`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SubsetRcChannels {
    /// Index of the first channel in this subset.
    pub starting_channel: u8,
    /// Resolution of the packed channels.
    pub resolution: SubsetResolution,
    /// Whether the digital switch bit is asserted.
    pub digital_switch_flag: bool,
    /// Channel values expressed as standard CRSF ticks.
    pub values: Vec<u16, CHANNEL_COUNT>,
}

impl SubsetRcChannels {
    /// Creates an empty subset packet.
    pub fn new(
        starting_channel: u8,
        resolution: SubsetResolution,
        digital_switch_flag: bool,
    ) -> Result<Self, RcError> {
        if starting_channel >= CHANNEL_COUNT as u8 {
            return Err(RcError::InvalidConfiguration);
        }
        Ok(Self {
            starting_channel,
            resolution,
            digital_switch_flag,
            values: Vec::new(),
        })
    }

    /// Appends a CRSF tick value.
    pub fn push_ticks(&mut self, value: u16) -> Result<(), RcError> {
        self.values
            .push(value & 0x07FF)
            .map_err(|_| RcError::TooManyChannels)
    }

    /// Appends a microsecond value.
    pub fn push_micros(&mut self, value: u16) -> Result<(), RcError> {
        self.push_ticks(micros_to_ticks(value))
    }

    /// Encodes the subset payload.
    pub fn encode_payload(&self) -> Result<Vec<u8, 60>, RcError> {
        let mut out = Vec::new();
        let header = (self.starting_channel & 0x1F)
            | (self.resolution.config_bits() << 5)
            | ((self.digital_switch_flag as u8) << 7);
        out.push(header).map_err(|_| RcError::TooManyChannels)?;

        let bits_per_channel = self.resolution.bits_per_channel();
        let mut bit_index = 8usize;
        let mut channel_index = 0usize;
        while channel_index < self.values.len() {
            let raw = self.resolution.ticks_to_raw(self.values[channel_index]);
            let mut bit = 0usize;
            while bit < bits_per_channel {
                let overall_bit = bit_index + bit;
                let byte_index = overall_bit / 8;
                let bit_in_byte = overall_bit % 8;
                while out.len() <= byte_index {
                    out.push(0).map_err(|_| RcError::TooManyChannels)?;
                }
                if ((raw >> bit) & 1) != 0 {
                    out[byte_index] |= 1 << bit_in_byte;
                }
                bit += 1;
            }
            bit_index += bits_per_channel;
            channel_index += 1;
        }
        Ok(out)
    }

    /// Decodes a subset payload.
    pub fn decode(payload: &[u8]) -> Result<Self, RcError> {
        if payload.is_empty() {
            return Err(RcError::InvalidLength);
        }
        let header = payload[0];
        let starting_channel = header & 0x1F;
        let resolution = SubsetResolution::from_config((header >> 5) & 0x03)
            .ok_or(RcError::InvalidConfiguration)?;
        let digital_switch_flag = (header & 0x80) != 0;
        let bits_per_channel = resolution.bits_per_channel();
        let available_bits = (payload.len() - 1) * 8;
        let channel_count = available_bits / bits_per_channel;

        if channel_count == 0 || starting_channel as usize + channel_count > CHANNEL_COUNT {
            return Err(RcError::InvalidLength);
        }

        let mut values = Vec::new();
        let mut channel_index = 0usize;
        let mut bit_index = 8usize;
        while channel_index < channel_count {
            let mut raw = 0u16;
            let mut bit = 0usize;
            while bit < bits_per_channel {
                let overall_bit = bit_index + bit;
                let byte_index = overall_bit / 8;
                let bit_in_byte = overall_bit % 8;
                let bit_value = (payload[byte_index] >> bit_in_byte) & 1;
                raw |= (bit_value as u16) << bit;
                bit += 1;
            }
            values
                .push(resolution.raw_to_ticks(raw))
                .map_err(|_| RcError::TooManyChannels)?;
            bit_index += bits_per_channel;
            channel_index += 1;
        }

        Ok(Self {
            starting_channel,
            resolution,
            digital_switch_flag,
            values,
        })
    }

    /// Builds a CRSF frame containing the subset payload.
    pub fn encode_frame(&self, address: DeviceAddress) -> Result<Frame, RcError> {
        let payload = self.encode_payload()?;
        Ok(Frame::new(
            address,
            FRAME_TYPE_SUBSET_RC_CHANNELS_PACKED,
            payload.as_slice(),
        )?)
    }
}

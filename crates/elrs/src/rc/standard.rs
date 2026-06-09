use crate::{
    DeviceAddress,
    frame::{FRAME_TYPE_RC_CHANNELS_PACKED, Frame, FrameError},
};

use super::{CHANNEL_COUNT, RC_CHANNELS_PAYLOAD_LEN, micros_to_ticks, ticks_to_micros};

/// Standard CRSF 16-channel RC payload.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RcChannels {
    /// Raw 11-bit CRSF channel values.
    pub values: [u16; CHANNEL_COUNT],
}

impl RcChannels {
    /// Creates a new channel block from raw CRSF ticks.
    pub const fn new(values: [u16; CHANNEL_COUNT]) -> Self {
        Self { values }
    }

    /// Creates a new channel block from pulse widths in microseconds.
    pub fn from_micros(micros: [u16; CHANNEL_COUNT]) -> Self {
        let mut values = [0u16; CHANNEL_COUNT];
        let mut i = 0;
        while i < CHANNEL_COUNT {
            values[i] = micros_to_ticks(micros[i]);
            i += 1;
        }
        Self { values }
    }

    /// Returns one channel converted to microseconds.
    pub fn micros(&self, index: usize) -> Option<u16> {
        self.values.get(index).map(|value| ticks_to_micros(*value))
    }

    /// Sets a channel from CRSF ticks.
    pub fn set_ticks(&mut self, index: usize, value: u16) {
        if index < CHANNEL_COUNT {
            self.values[index] = value & 0x07FF;
        }
    }

    /// Sets a channel from microseconds.
    pub fn set_micros(&mut self, index: usize, value: u16) {
        self.set_ticks(index, micros_to_ticks(value));
    }

    /// Packs the channels into the CRSF 22-byte payload format.
    pub fn pack(self) -> [u8; RC_CHANNELS_PAYLOAD_LEN] {
        let mut out = [0u8; RC_CHANNELS_PAYLOAD_LEN];
        let mut bit_index = 0usize;
        let mut channel = 0usize;

        while channel < CHANNEL_COUNT {
            let value = self.values[channel] & 0x07FF;
            let mut bit = 0usize;
            while bit < 11 {
                if ((value >> bit) & 1) != 0 {
                    let byte_index = bit_index / 8;
                    let bit_in_byte = bit_index % 8;
                    out[byte_index] |= 1 << bit_in_byte;
                }
                bit_index += 1;
                bit += 1;
            }
            channel += 1;
        }
        out
    }

    /// Unpacks the CRSF 22-byte payload into channels.
    pub fn unpack(bytes: [u8; RC_CHANNELS_PAYLOAD_LEN]) -> Self {
        let mut values = [0u16; CHANNEL_COUNT];
        let mut bit_index = 0usize;
        let mut channel = 0usize;

        while channel < CHANNEL_COUNT {
            let mut value = 0u16;
            let mut bit = 0usize;
            while bit < 11 {
                let byte_index = bit_index / 8;
                let bit_in_byte = bit_index % 8;
                let bit_value = (bytes[byte_index] >> bit_in_byte) & 1;
                value |= (bit_value as u16) << bit;
                bit_index += 1;
                bit += 1;
            }
            values[channel] = value;
            channel += 1;
        }
        Self { values }
    }

    /// Builds a CRSF frame containing the packed channel payload.
    pub fn encode_frame(&self, address: DeviceAddress) -> Result<Frame, FrameError> {
        Frame::new(address, FRAME_TYPE_RC_CHANNELS_PACKED, &self.pack())
    }
}

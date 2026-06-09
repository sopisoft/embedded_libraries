use heapless::Vec;

use crate::{
    DeviceAddress,
    frame::{FRAME_TYPE_PARAMETER_ENTRY, Frame},
};

use super::{CommandStatus, ParameterError, ParameterType};

/// Parameter entry chunk returned by a device.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ParameterEntry<'a> {
    /// Parameter number.
    pub parameter: u8,
    /// Remaining chunk count.
    pub chunks_remaining: u8,
    /// Entry payload chunk bytes.
    pub data: &'a [u8],
}

impl<'a> ParameterEntry<'a> {
    /// Decodes the entry payload.
    pub fn decode(payload: &'a [u8]) -> Result<Self, ParameterError> {
        if payload.len() < 2 {
            return Err(ParameterError::InvalidLength);
        }
        Ok(Self {
            parameter: payload[0],
            chunks_remaining: payload[1],
            data: &payload[2..],
        })
    }

    /// Encodes the entry payload.
    pub fn encode_payload(&self) -> Result<Vec<u8, 58>, ParameterError> {
        let mut out = Vec::new();
        out.push(self.parameter)
            .map_err(|_| ParameterError::PayloadTooLong)?;
        out.push(self.chunks_remaining)
            .map_err(|_| ParameterError::PayloadTooLong)?;
        out.extend_from_slice(self.data)
            .map_err(|_| ParameterError::PayloadTooLong)?;
        Ok(out)
    }

    /// Returns the parent-folder index for typed parameter data.
    pub fn parent_folder(&self) -> Option<u8> {
        self.data.first().copied()
    }

    /// Returns the parameter type if the payload contains a typed entry.
    pub fn parameter_type(&self) -> Option<ParameterType> {
        self.data.get(1).copied().map(ParameterType::from_raw)
    }

    /// Returns the command status for a command parameter entry.
    pub fn command_status(&self) -> Option<CommandStatus> {
        if self.parameter_type() != Some(ParameterType::Command) {
            return None;
        }
        let mut nul_count = 0usize;
        let mut i = 2usize;
        while i < self.data.len() {
            if self.data[i] == 0 {
                nul_count += 1;
                if nul_count == 1 {
                    return self.data.get(i + 1).copied().map(CommandStatus::from_raw);
                }
            }
            i += 1;
        }
        None
    }

    /// Builds the extended CRSF frame.
    pub fn encode_frame(
        &self,
        address: DeviceAddress,
        destination: DeviceAddress,
        origin: DeviceAddress,
    ) -> Result<Frame, ParameterError> {
        let payload = self.encode_payload()?;
        Ok(Frame::new_extended(
            address,
            FRAME_TYPE_PARAMETER_ENTRY,
            destination,
            origin,
            payload.as_slice(),
        )?)
    }
}

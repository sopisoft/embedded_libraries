use heapless::Vec;

use crate::{
    DeviceAddress,
    frame::{
        FRAME_TYPE_PARAMETER_PING, FRAME_TYPE_PARAMETER_READ, FRAME_TYPE_PARAMETER_WRITE, Frame,
    },
};

use super::ParameterError;

/// Parameter read request.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ParameterRead {
    /// Parameter number.
    pub parameter: u8,
    /// Requested chunk number.
    pub chunk: u8,
}

impl ParameterRead {
    /// Encodes the payload.
    pub const fn encode_payload(&self) -> [u8; 2] {
        [self.parameter, self.chunk]
    }

    /// Decodes the payload.
    pub fn decode(payload: &[u8]) -> Result<Self, ParameterError> {
        if payload.len() != 2 {
            return Err(ParameterError::InvalidLength);
        }
        Ok(Self {
            parameter: payload[0],
            chunk: payload[1],
        })
    }

    /// Builds the extended CRSF frame.
    pub fn encode_frame(
        &self,
        address: DeviceAddress,
        destination: DeviceAddress,
        origin: DeviceAddress,
    ) -> Result<Frame, ParameterError> {
        Ok(Frame::new_extended(
            address,
            FRAME_TYPE_PARAMETER_READ,
            destination,
            origin,
            &self.encode_payload(),
        )?)
    }
}

/// Parameter value write request.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ParameterWrite<'a> {
    /// Parameter number.
    pub parameter: u8,
    /// Encoded parameter value bytes.
    pub data: &'a [u8],
}

impl<'a> ParameterWrite<'a> {
    /// Encodes the payload.
    pub fn encode_payload(&self) -> Result<Vec<u8, 58>, ParameterError> {
        let mut out = Vec::new();
        out.push(self.parameter)
            .map_err(|_| ParameterError::PayloadTooLong)?;
        out.extend_from_slice(self.data)
            .map_err(|_| ParameterError::PayloadTooLong)?;
        Ok(out)
    }

    /// Decodes the payload.
    pub fn decode(payload: &'a [u8]) -> Result<Self, ParameterError> {
        if payload.is_empty() {
            return Err(ParameterError::InvalidLength);
        }
        Ok(Self {
            parameter: payload[0],
            data: &payload[1..],
        })
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
            FRAME_TYPE_PARAMETER_WRITE,
            destination,
            origin,
            payload.as_slice(),
        )?)
    }
}

/// Builds an empty parameter ping frame.
pub fn encode_parameter_ping_frame(
    address: DeviceAddress,
    destination: DeviceAddress,
    origin: DeviceAddress,
) -> Result<Frame, ParameterError> {
    Ok(Frame::new_extended(
        address,
        FRAME_TYPE_PARAMETER_PING,
        destination,
        origin,
        &[],
    )?)
}

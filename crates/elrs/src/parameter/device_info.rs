use core::str;

use heapless::Vec;

use crate::{
    DeviceAddress,
    frame::{FRAME_TYPE_DEVICE_INFO, Frame},
};

use super::ParameterError;

/// Parameter device information payload.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct DeviceInfo<'a> {
    /// Null-terminated device name.
    pub name: &'a str,
    /// Serial number.
    pub serial_number: u32,
    /// Hardware identifier.
    pub hardware_id: u32,
    /// Firmware identifier.
    pub firmware_id: u32,
    /// Total number of parameters.
    pub parameters_total: u8,
    /// Parameter protocol version.
    pub parameter_version: u8,
}

impl<'a> DeviceInfo<'a> {
    /// Decodes the device information payload.
    pub fn decode(payload: &'a [u8]) -> Result<Self, ParameterError> {
        let mut nul = None;
        let mut i = 0usize;
        while i < payload.len() {
            if payload[i] == 0 {
                nul = Some(i);
                break;
            }
            i += 1;
        }
        let nul = nul.ok_or(ParameterError::InvalidLength)?;
        if payload.len() < nul + 1 + 14 {
            return Err(ParameterError::InvalidLength);
        }
        let base = nul + 1;
        Ok(Self {
            name: str::from_utf8(&payload[..nul]).map_err(|_| ParameterError::InvalidText)?,
            serial_number: u32::from_be_bytes(payload[base..base + 4].try_into().unwrap()),
            hardware_id: u32::from_be_bytes(payload[base + 4..base + 8].try_into().unwrap()),
            firmware_id: u32::from_be_bytes(payload[base + 8..base + 12].try_into().unwrap()),
            parameters_total: payload[base + 12],
            parameter_version: payload[base + 13],
        })
    }

    /// Encodes the device information payload.
    pub fn encode_payload(&self) -> Result<Vec<u8, 58>, ParameterError> {
        let mut out = Vec::new();
        out.extend_from_slice(self.name.as_bytes())
            .map_err(|_| ParameterError::PayloadTooLong)?;
        out.push(0).map_err(|_| ParameterError::PayloadTooLong)?;
        out.extend_from_slice(&self.serial_number.to_be_bytes())
            .map_err(|_| ParameterError::PayloadTooLong)?;
        out.extend_from_slice(&self.hardware_id.to_be_bytes())
            .map_err(|_| ParameterError::PayloadTooLong)?;
        out.extend_from_slice(&self.firmware_id.to_be_bytes())
            .map_err(|_| ParameterError::PayloadTooLong)?;
        out.push(self.parameters_total)
            .map_err(|_| ParameterError::PayloadTooLong)?;
        out.push(self.parameter_version)
            .map_err(|_| ParameterError::PayloadTooLong)?;
        Ok(out)
    }

    /// Builds the extended device-information frame.
    pub fn encode_frame(
        &self,
        address: DeviceAddress,
        destination: DeviceAddress,
        origin: DeviceAddress,
    ) -> Result<Frame, ParameterError> {
        let payload = self.encode_payload()?;
        Ok(Frame::new_extended(
            address,
            FRAME_TYPE_DEVICE_INFO,
            destination,
            origin,
            payload.as_slice(),
        )?)
    }
}

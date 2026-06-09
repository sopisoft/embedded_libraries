//! Direct command payloads.

use heapless::Vec;

use crate::{
    DeviceAddress,
    crc::CommandCrc,
    frame::{FRAME_TYPE_DIRECT_COMMAND, Frame, FrameError},
};

/// Errors returned by direct command helpers.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DirectCommandError {
    /// The command payload length was invalid.
    InvalidLength,
    /// The command CRC did not match the payload.
    CrcMismatch { expected: u8, actual: u8 },
    /// The payload was too long to fit into one CRSF frame.
    PayloadTooLong,
    /// The frame operation failed.
    Frame(FrameError),
}

impl From<FrameError> for DirectCommandError {
    fn from(value: FrameError) -> Self {
        Self::Frame(value)
    }
}

/// Direct command payload wrapped by frame type `0x32`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DirectCommand {
    /// Command destination.
    pub destination: DeviceAddress,
    /// Command origin.
    pub origin: DeviceAddress,
    /// Command identifier.
    pub command_id: u8,
    /// Command payload bytes.
    pub payload: Vec<u8, 56>,
}

impl DirectCommand {
    /// Creates a direct command.
    pub fn new(
        destination: DeviceAddress,
        origin: DeviceAddress,
        command_id: u8,
        payload: &[u8],
    ) -> Result<Self, DirectCommandError> {
        let mut owned = Vec::new();
        owned
            .extend_from_slice(payload)
            .map_err(|_| DirectCommandError::PayloadTooLong)?;
        Ok(Self {
            destination,
            origin,
            command_id,
            payload: owned,
        })
    }

    /// Computes the command CRC over the direct command payload.
    pub fn command_crc(&self) -> u8 {
        let mut crc = CommandCrc::new();
        crc.update(FRAME_TYPE_DIRECT_COMMAND);
        crc.update(self.destination.as_u8());
        crc.update(self.origin.as_u8());
        crc.update(self.command_id);
        let mut i = 0;
        while i < self.payload.len() {
            crc.update(self.payload[i]);
            i += 1;
        }
        crc.value()
    }

    /// Encodes the command payload bytes that live inside the CRSF frame body.
    pub fn encode_payload(&self) -> Result<Vec<u8, 58>, DirectCommandError> {
        let mut out = Vec::new();
        out.push(self.command_id)
            .map_err(|_| DirectCommandError::PayloadTooLong)?;
        out.extend_from_slice(self.payload.as_slice())
            .map_err(|_| DirectCommandError::PayloadTooLong)?;
        out.push(self.command_crc())
            .map_err(|_| DirectCommandError::PayloadTooLong)?;
        Ok(out)
    }

    /// Decodes a direct command from a CRSF frame.
    pub fn decode(frame: &Frame) -> Result<Self, DirectCommandError> {
        if frame.frame_type != FRAME_TYPE_DIRECT_COMMAND {
            return Err(DirectCommandError::InvalidLength);
        }
        let destination = frame
            .destination()
            .ok_or(DirectCommandError::InvalidLength)?;
        let origin = frame.origin().ok_or(DirectCommandError::InvalidLength)?;
        let payload = frame.payload();
        if payload.len() < 2 {
            return Err(DirectCommandError::InvalidLength);
        }
        let command_id = payload[0];
        let command_crc = payload[payload.len() - 1];
        let data = &payload[1..payload.len() - 1];
        let command = Self::new(destination, origin, command_id, data)?;
        let expected_crc = command.command_crc();
        if command_crc != expected_crc {
            return Err(DirectCommandError::CrcMismatch {
                expected: expected_crc,
                actual: command_crc,
            });
        }
        Ok(command)
    }

    /// Builds the CRSF direct-command frame.
    pub fn encode_frame(&self, address: DeviceAddress) -> Result<Frame, DirectCommandError> {
        let payload = self.encode_payload()?;
        Ok(Frame::new_extended(
            address,
            FRAME_TYPE_DIRECT_COMMAND,
            self.destination,
            self.origin,
            payload.as_slice(),
        )?)
    }
}

#[cfg(test)]
mod tests {
    use super::DirectCommand;
    use crate::DeviceAddress;

    #[test]
    fn command_roundtrip_works() {
        let command = DirectCommand::new(
            DeviceAddress::RECEIVER,
            DeviceAddress::TRANSMITTER,
            0x10,
            &[0x62, 0x6C],
        )
        .unwrap();
        let frame = command.encode_frame(DeviceAddress::TRANSMITTER).unwrap();
        let decoded = DirectCommand::decode(&frame).unwrap();
        assert_eq!(decoded, command);
    }
}

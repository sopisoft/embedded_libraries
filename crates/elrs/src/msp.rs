//! MSP-over-CRSF chunk helpers.

use heapless::Vec;

use crate::{
    DeviceAddress,
    frame::{FRAME_TYPE_MSP_REQUEST, FRAME_TYPE_MSP_RESPONSE, Frame, FrameError},
};

/// Errors returned by MSP helpers.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MspError {
    /// The CRSF frame payload is too short or too long.
    InvalidLength,
    /// The CRSF frame type is not an MSP wrapper.
    InvalidFrameType,
    /// The payload chunk exceeded the supported size.
    PayloadTooLong,
    /// The frame operation failed.
    Frame(FrameError),
}

impl From<FrameError> for MspError {
    fn from(value: FrameError) -> Self {
        Self::Frame(value)
    }
}

/// MSP chunk status byte.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct MspStatus {
    /// Lower 4-bit rolling chunk sequence number.
    pub sequence: u8,
    /// True when this chunk starts a new MSP frame.
    pub is_start: bool,
    /// MSP version encoded in bits 5..=6.
    pub version: u8,
    /// Response error flag stored in bit 7.
    pub error: bool,
}

impl MspStatus {
    /// Parses the MSP status byte.
    pub const fn from_byte(byte: u8) -> Self {
        Self {
            sequence: byte & 0x0F,
            is_start: (byte & 0x10) != 0,
            version: (byte >> 5) & 0x03,
            error: (byte & 0x80) != 0,
        }
    }

    /// Encodes the MSP status byte.
    pub const fn to_byte(self) -> u8 {
        (self.sequence & 0x0F)
            | ((self.is_start as u8) << 4)
            | ((self.version & 0x03) << 5)
            | ((self.error as u8) << 7)
    }
}

/// One MSP request or response chunk carried inside CRSF.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MspFrame {
    /// CRSF frame type, either request or response.
    pub frame_type: u8,
    /// Destination device.
    pub destination: DeviceAddress,
    /// Origin device.
    pub origin: DeviceAddress,
    /// Chunk status byte.
    pub status: MspStatus,
    /// Stripped MSP body bytes for this chunk.
    pub body: Vec<u8, 57>,
}

impl MspFrame {
    /// Creates an MSP request chunk.
    pub fn request(
        destination: DeviceAddress,
        origin: DeviceAddress,
        status: MspStatus,
        body: &[u8],
    ) -> Result<Self, MspError> {
        Self::new(FRAME_TYPE_MSP_REQUEST, destination, origin, status, body)
    }

    /// Creates an MSP response chunk.
    pub fn response(
        destination: DeviceAddress,
        origin: DeviceAddress,
        status: MspStatus,
        body: &[u8],
    ) -> Result<Self, MspError> {
        Self::new(FRAME_TYPE_MSP_RESPONSE, destination, origin, status, body)
    }

    fn new(
        frame_type: u8,
        destination: DeviceAddress,
        origin: DeviceAddress,
        status: MspStatus,
        body: &[u8],
    ) -> Result<Self, MspError> {
        if frame_type != FRAME_TYPE_MSP_REQUEST && frame_type != FRAME_TYPE_MSP_RESPONSE {
            return Err(MspError::InvalidFrameType);
        }
        let mut owned = Vec::new();
        owned
            .extend_from_slice(body)
            .map_err(|_| MspError::PayloadTooLong)?;
        Ok(Self {
            frame_type,
            destination,
            origin,
            status,
            body: owned,
        })
    }

    /// Encodes the MSP chunk payload inside the CRSF frame.
    pub fn encode_payload(&self) -> Result<Vec<u8, 58>, MspError> {
        let mut out = Vec::new();
        out.push(self.status.to_byte())
            .map_err(|_| MspError::PayloadTooLong)?;
        out.extend_from_slice(self.body.as_slice())
            .map_err(|_| MspError::PayloadTooLong)?;
        Ok(out)
    }

    /// Builds the CRSF frame.
    pub fn encode_frame(&self, address: DeviceAddress) -> Result<Frame, MspError> {
        let payload = self.encode_payload()?;
        Ok(Frame::new_extended(
            address,
            self.frame_type,
            self.destination,
            self.origin,
            payload.as_slice(),
        )?)
    }

    /// Decodes an MSP frame from a CRSF frame.
    pub fn decode(frame: &Frame) -> Result<Self, MspError> {
        if frame.frame_type != FRAME_TYPE_MSP_REQUEST && frame.frame_type != FRAME_TYPE_MSP_RESPONSE
        {
            return Err(MspError::InvalidFrameType);
        }
        let destination = frame.destination().ok_or(MspError::InvalidLength)?;
        let origin = frame.origin().ok_or(MspError::InvalidLength)?;
        let payload = frame.payload();
        if payload.is_empty() {
            return Err(MspError::InvalidLength);
        }
        Self::new(
            frame.frame_type,
            destination,
            origin,
            MspStatus::from_byte(payload[0]),
            &payload[1..],
        )
    }
}

#[cfg(test)]
mod tests {
    use super::{MspFrame, MspStatus};
    use crate::DeviceAddress;

    #[test]
    fn status_roundtrip_works() {
        let status = MspStatus {
            sequence: 7,
            is_start: true,
            version: 2,
            error: false,
        };
        assert_eq!(MspStatus::from_byte(status.to_byte()), status);
    }

    #[test]
    fn frame_roundtrip_works() {
        let status = MspStatus {
            sequence: 3,
            is_start: true,
            version: 2,
            error: false,
        };
        let frame = MspFrame::request(
            DeviceAddress::FLIGHT_CONTROLLER,
            DeviceAddress::TRANSMITTER,
            status,
            &[0x10, 0x00, 0x01, 0x02, 0x03],
        )
        .unwrap();
        let crsf = frame.encode_frame(DeviceAddress::TRANSMITTER).unwrap();
        let decoded = MspFrame::decode(&crsf).unwrap();
        assert_eq!(decoded, frame);
    }
}

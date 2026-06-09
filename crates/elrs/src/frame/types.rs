use heapless::Vec;

use crate::{address::DeviceAddress, crc::FrameCrc};

use super::FRAME_TYPE_ARDUPILOT_PASSTHROUGH;

/// Maximum CRSF frame size in bytes, including address and CRC.
pub const MAX_FRAME_SIZE: usize = 64;
/// Maximum value allowed in the CRSF length field.
pub const MAX_LENGTH_FIELD: usize = 62;
/// Maximum raw body size between frame type and CRC.
pub const MAX_BODY_LEN: usize = MAX_LENGTH_FIELD - 2;
/// Maximum extended-frame payload size after destination and origin.
pub const MAX_EXTENDED_PAYLOAD_LEN: usize = MAX_BODY_LEN - 2;

/// Errors returned while building or encoding frames.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// The body exceeds the CRSF maximum.
    BodyTooLong,
    /// The frame type does not match the selected constructor.
    UnexpectedFrameKind,
    /// The frame buffer is too small.
    BufferTooSmall,
}

/// An owned CRSF frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Frame {
    /// The address or sync byte.
    pub address: DeviceAddress,
    /// The CRSF frame type.
    pub frame_type: u8,
    body: Vec<u8, MAX_BODY_LEN>,
}

impl Frame {
    /// Creates a short CRSF frame.
    pub fn new(address: DeviceAddress, frame_type: u8, payload: &[u8]) -> Result<Self, FrameError> {
        if Self::is_extended_type(frame_type) {
            return Err(FrameError::UnexpectedFrameKind);
        }
        let mut body = Vec::new();
        body.extend_from_slice(payload)
            .map_err(|_| FrameError::BodyTooLong)?;
        Ok(Self {
            address,
            frame_type,
            body,
        })
    }

    /// Creates an extended CRSF frame with destination and origin routing bytes.
    pub fn new_extended(
        address: DeviceAddress,
        frame_type: u8,
        destination: DeviceAddress,
        origin: DeviceAddress,
        payload: &[u8],
    ) -> Result<Self, FrameError> {
        if !Self::is_extended_type(frame_type) {
            return Err(FrameError::UnexpectedFrameKind);
        }
        let mut body = Vec::new();
        body.push(destination.as_u8())
            .map_err(|_| FrameError::BodyTooLong)?;
        body.push(origin.as_u8())
            .map_err(|_| FrameError::BodyTooLong)?;
        body.extend_from_slice(payload)
            .map_err(|_| FrameError::BodyTooLong)?;
        Ok(Self {
            address,
            frame_type,
            body,
        })
    }

    /// Creates a frame from a raw body slice.
    pub fn from_body(
        address: DeviceAddress,
        frame_type: u8,
        body: &[u8],
    ) -> Result<Self, FrameError> {
        let mut out = Vec::new();
        out.extend_from_slice(body)
            .map_err(|_| FrameError::BodyTooLong)?;
        Ok(Self {
            address,
            frame_type,
            body: out,
        })
    }

    /// Returns whether the frame type uses an extended header.
    pub const fn is_extended_type(frame_type: u8) -> bool {
        frame_type >= 0x28 && frame_type != FRAME_TYPE_ARDUPILOT_PASSTHROUGH
    }

    /// Returns whether this frame uses an extended header.
    pub const fn is_extended(&self) -> bool {
        Self::is_extended_type(self.frame_type)
    }

    /// Returns the raw body, including routing bytes for extended frames.
    pub fn raw_body(&self) -> &[u8] {
        self.body.as_slice()
    }

    /// Returns the payload bytes without routing bytes.
    pub fn payload(&self) -> &[u8] {
        if self.is_extended() {
            &self.body.as_slice()[2..]
        } else {
            self.body.as_slice()
        }
    }

    /// Returns the destination address for an extended frame.
    pub fn destination(&self) -> Option<DeviceAddress> {
        if self.is_extended() && self.body.len() >= 2 {
            Some(DeviceAddress::new(self.body[0]))
        } else {
            None
        }
    }

    /// Returns the origin address for an extended frame.
    pub fn origin(&self) -> Option<DeviceAddress> {
        if self.is_extended() && self.body.len() >= 2 {
            Some(DeviceAddress::new(self.body[1]))
        } else {
            None
        }
    }

    /// Returns the CRSF length field value.
    pub fn frame_length(&self) -> usize {
        self.body.len() + 2
    }

    /// Computes the CRSF frame CRC.
    pub fn crc(&self) -> u8 {
        let mut crc = FrameCrc::new();
        crc.update(self.frame_type);
        let mut i = 0;
        while i < self.body.len() {
            crc.update(self.body[i]);
            i += 1;
        }
        crc.value()
    }

    /// Encodes the frame into a caller-provided buffer.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, FrameError> {
        let required = self.frame_length() + 2;
        if required > MAX_FRAME_SIZE {
            return Err(FrameError::BodyTooLong);
        }
        if out.len() < required {
            return Err(FrameError::BufferTooSmall);
        }
        out[0] = self.address.as_u8();
        out[1] = self.frame_length() as u8;
        out[2] = self.frame_type;
        let mut i = 0;
        while i < self.body.len() {
            out[3 + i] = self.body[i];
            i += 1;
        }
        out[required - 1] = self.crc();
        Ok(required)
    }

    /// Encodes the frame into an owned heapless buffer.
    pub fn to_bytes(&self) -> Result<Vec<u8, MAX_FRAME_SIZE>, FrameError> {
        let mut out = Vec::new();
        out.resize_default(self.frame_length() + 2)
            .map_err(|_| FrameError::BodyTooLong)?;
        self.encode(out.as_mut_slice())?;
        Ok(out)
    }
}

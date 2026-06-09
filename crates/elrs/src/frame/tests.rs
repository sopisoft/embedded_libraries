use super::super::{
    FRAME_TYPE_DIRECT_COMMAND, FRAME_TYPE_RC_CHANNELS_PACKED, Frame, FrameParser, ParseError,
};
use crate::DeviceAddress;

#[test]
fn short_frame_roundtrip_works() {
    let frame = Frame::new(
        DeviceAddress::FLIGHT_CONTROLLER,
        FRAME_TYPE_RC_CHANNELS_PACKED,
        &[1, 2, 3],
    )
    .unwrap();
    let bytes = frame.to_bytes().unwrap();
    let mut parser = FrameParser::new();
    let mut parsed = None;
    let mut i = 0;
    while i < bytes.len() {
        parsed = parser.push(bytes[i]);
        i += 1;
    }
    let parsed = parsed.unwrap().unwrap();
    assert_eq!(parsed.address, DeviceAddress::FLIGHT_CONTROLLER);
    assert_eq!(parsed.frame_type, FRAME_TYPE_RC_CHANNELS_PACKED);
    assert_eq!(parsed.payload(), &[1, 2, 3]);
}

#[test]
fn extended_frame_exposes_routing() {
    let frame = Frame::new_extended(
        DeviceAddress::TRANSMITTER,
        FRAME_TYPE_DIRECT_COMMAND,
        DeviceAddress::RECEIVER,
        DeviceAddress::TRANSMITTER,
        &[0x10, 0x20],
    )
    .unwrap();
    assert_eq!(frame.destination(), Some(DeviceAddress::RECEIVER));
    assert_eq!(frame.origin(), Some(DeviceAddress::TRANSMITTER));
    assert_eq!(frame.payload(), &[0x10, 0x20]);
}

#[test]
fn parser_rejects_bad_crc() {
    let mut parser = FrameParser::new();
    let bytes = [
        DeviceAddress::FLIGHT_CONTROLLER.as_u8(),
        4,
        FRAME_TYPE_RC_CHANNELS_PACKED,
        0xAA,
        0xBB,
        0x00,
    ];
    let mut result = None;
    let mut i = 0;
    while i < bytes.len() {
        result = parser.push(bytes[i]);
        i += 1;
    }
    assert!(matches!(
        result.unwrap(),
        Err(ParseError::CrcMismatch { .. })
    ));
}

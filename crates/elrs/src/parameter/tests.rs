use super::{
    CommandStatus, DeviceInfo, ParameterEntry, ParameterRead, ParameterType, ParameterWrite,
    encode_parameter_ping_frame,
};
use crate::DeviceAddress;

#[test]
fn device_info_roundtrip_works() {
    let info = DeviceInfo {
        name: "ELRS TX",
        serial_number: 1,
        hardware_id: 2,
        firmware_id: 3,
        parameters_total: 10,
        parameter_version: 1,
    };
    let payload = info.encode_payload().unwrap();
    let decoded = DeviceInfo::decode(payload.as_slice()).unwrap();
    assert_eq!(decoded, info);
}

#[test]
fn parameter_read_roundtrip_works() {
    let read = ParameterRead {
        parameter: 4,
        chunk: 2,
    };
    assert_eq!(ParameterRead::decode(&read.encode_payload()).unwrap(), read);
}

#[test]
fn parameter_write_roundtrip_works() {
    let write = ParameterWrite {
        parameter: 7,
        data: &[1, 2, 3, 4],
    };
    let payload = write.encode_payload().unwrap();
    let decoded = ParameterWrite::decode(payload.as_slice()).unwrap();
    assert_eq!(decoded, write);
}

#[test]
fn parameter_entry_detects_command_state() {
    let data = [0, 0x0D, b'B', b'i', b'n', b'd', 0, 2, 20, b'O', b'K', 0];
    let entry = ParameterEntry {
        parameter: 9,
        chunks_remaining: 0,
        data: &data,
    };
    assert_eq!(entry.parameter_type(), Some(ParameterType::Command));
    assert_eq!(entry.command_status(), Some(CommandStatus::Progress));
}

#[test]
fn ping_frame_is_extended() {
    let frame = encode_parameter_ping_frame(
        DeviceAddress::TRANSMITTER,
        DeviceAddress::RECEIVER,
        DeviceAddress::TRANSMITTER,
    )
    .unwrap();
    assert!(frame.is_extended());
}

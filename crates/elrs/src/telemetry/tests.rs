use libm::fabsf;

use super::super::{
    Attitude, BarometricAltitude, BatterySensor, FlightMode, Gps, encode_flight_mode,
};

#[test]
fn gps_roundtrip_works() {
    let gps = Gps {
        latitude_e7: 351_234_567,
        longitude_e7: 1_399_876_543,
        groundspeed_kmh_x100: 1234,
        heading_deg_x100: 9000,
        altitude_m_offset_1000: 1123,
        satellites: 14,
    };
    let decoded = Gps::decode(&gps.encode_payload()).unwrap();
    assert_eq!(gps, decoded);
    assert_eq!(decoded.altitude_m(), 123);
}

#[test]
fn battery_roundtrip_works() {
    let battery = BatterySensor {
        voltage_v_x100: 1680,
        current_a_x100: 345,
        capacity_mah: 1200,
        remaining_percent: 55,
    };
    let decoded = BatterySensor::decode(&battery.encode_payload()).unwrap();
    assert_eq!(decoded, battery);
}

#[test]
fn attitude_roundtrip_works() {
    let attitude = Attitude {
        pitch_rad: 0.1,
        roll_rad: -0.2,
        yaw_rad: 1.3,
    };
    let decoded = Attitude::decode(&attitude.encode_payload()).unwrap();
    assert!(fabsf(decoded.pitch_rad - 0.1) < 1.0e-4);
    assert!(fabsf(decoded.roll_rad + 0.2) < 1.0e-4);
    assert!(fabsf(decoded.yaw_rad - 1.3) < 1.0e-4);
}

#[test]
fn baro_helpers_are_reasonable() {
    let packed = BarometricAltitude::pack_altitude_dm(1234);
    let frame = BarometricAltitude {
        altitude_packed: packed,
        vertical_speed_packed: BarometricAltitude::pack_vertical_speed(250),
    };
    assert_eq!(frame.altitude_dm(), 1234);
    assert!(frame.vertical_speed_cm_s() > 0);
}

#[test]
fn flight_mode_string_roundtrip_works() {
    let payload = encode_flight_mode("CRUISE").unwrap();
    let mode = FlightMode::new(payload.as_slice());
    assert_eq!(mode.as_str().unwrap(), "CRUISE");
}

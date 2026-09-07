use super::*;
use elrs::{DeviceAddress, RcChannels, SubsetRcChannels, SubsetResolution};
use glam::Vec3;
use imu::MargSample;

fn level_sample() -> MargSample {
    MargSample::from_vectors(
        imu::AccelGyroSample::from_vectors_without_temperature(
            Vec3::new(0.0, 0.0, 9.80665),
            Vec3::ZERO,
        ),
        Vec3::X,
    )
}

fn send_channels(controller: &mut FlightController, channels: RcChannels, now_us: u32) {
    let bytes = channels
        .encode_frame(DeviceAddress::FLIGHT_CONTROLLER)
        .unwrap()
        .to_bytes()
        .unwrap();
    for byte in bytes {
        controller.push_crsf_byte(byte, now_us);
    }
}

fn neutral_channels() -> RcChannels {
    RcChannels::from_micros([
        1_500, 1_500, 1_000, 1_500, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000,
        1_000, 1_000, 1_000,
    ])
}

#[test]
fn startup_is_failsafe_with_idle_throttle_and_retracted_linear_servo() {
    let mut controller = FlightController::new(Config::default());
    let output = controller.update(level_sample(), 0, CONTROL_PERIOD);
    assert_eq!(output.mode, Mode::Failsafe);
    assert_eq!(output.gs1502_position, Gs1502Position::Retracted);
    assert_eq!(
        output.control.pulses[OutputChannel::Throttle.index()].as_micros(),
        1_000
    );
    assert_eq!(output.control.pulses[0].as_micros(), 1_500);
}

#[test]
fn rc_switch_moves_linear_servo_between_positions() {
    let mut controller = FlightController::new(Config::default());
    send_channels(&mut controller, neutral_channels(), 10);
    assert_eq!(
        controller
            .update(level_sample(), 10, CONTROL_PERIOD)
            .gs1502_position,
        Gs1502Position::Retracted
    );

    let mut channels = neutral_channels();
    channels.set_micros(GS1502_CHANNEL.index(), 2_000);
    send_channels(&mut controller, channels, 20);
    assert_eq!(
        controller
            .update(level_sample(), 20, CONTROL_PERIOD)
            .gs1502_position,
        Gs1502Position::Extended
    );
}

#[test]
fn gs1502_switch_threshold_is_configurable() {
    let mut controller = FlightController::new(Config {
        gs1502_switch_threshold_us: 2_000,
        ..Config::default()
    });
    let mut channels = neutral_channels();
    channels.set_micros(GS1502_CHANNEL.index(), 1_800);
    send_channels(&mut controller, channels, 10);
    assert_eq!(
        controller
            .update(level_sample(), 10, CONTROL_PERIOD)
            .gs1502_position,
        Gs1502Position::Retracted
    );
}

#[test]
fn valid_crsf_frame_enables_manual_output() {
    let mut controller = FlightController::new(Config::default());
    send_channels(
        &mut controller,
        RcChannels::from_micros([
            1_700, 1_500, 1_800, 1_500, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000,
            1_000, 1_000, 1_000, 1_000,
        ]),
        1_000,
    );
    let output = controller.update(level_sample(), 1_000, CONTROL_PERIOD);
    assert_eq!(output.mode, Mode::Manual);
    assert!(output.pilot.roll.get() > 0.0);
    assert!(output.control.pulses[OutputChannel::Throttle.index()].as_micros() > 1_700);
}

#[test]
fn stale_rc_frame_returns_to_failsafe() {
    let mut controller = FlightController::new(Config::default());
    let mut channels = neutral_channels();
    channels.set_micros(GS1502_CHANNEL.index(), 2_000);
    send_channels(&mut controller, channels, 10);
    assert_eq!(
        controller.update(level_sample(), 10, CONTROL_PERIOD).mode,
        Mode::Manual
    );
    let output = controller.update(
        level_sample(),
        10 + DEFAULT_FAILSAFE_TIMEOUT.as_micros() + 1,
        CONTROL_PERIOD,
    );
    assert_eq!(output.mode, Mode::Failsafe);
    assert_eq!(output.gs1502_position, Gs1502Position::Retracted);
    assert_eq!(
        output.control.pulses[OutputChannel::Throttle.index()].as_micros(),
        1_000
    );
}

#[test]
fn invalid_frame_does_not_arm_rc_link() {
    let mut controller = FlightController::new(Config::default());
    let bytes = neutral_channels()
        .encode_frame(DeviceAddress::FLIGHT_CONTROLLER)
        .unwrap()
        .to_bytes()
        .unwrap();
    for (index, byte) in bytes.iter().enumerate() {
        controller.push_crsf_byte(
            if index + 1 == bytes.len() {
                byte ^ 0xFF
            } else {
                *byte
            },
            0,
        );
    }
    assert!(!controller.rc_is_alive(0));
}

#[test]
fn subset_frame_updates_only_the_selected_channels() {
    let mut controller = FlightController::new(Config::default());
    let mut subset = SubsetRcChannels::new(1, SubsetResolution::Bits11, false).unwrap();
    subset.push_micros(1_700).unwrap();
    subset.push_micros(1_600).unwrap();
    let bytes = subset
        .encode_frame(DeviceAddress::FLIGHT_CONTROLLER)
        .unwrap()
        .to_bytes()
        .unwrap();
    for byte in bytes {
        controller.push_crsf_byte(byte, 100);
    }
    assert_eq!(controller.channels().micros(0).unwrap(), 1_500);
    assert_eq!(controller.channels().micros(1).unwrap(), 1_700);
    assert_eq!(controller.channels().micros(2).unwrap(), 1_600);
    assert!(controller.rc_is_alive(100));
}

#[test]
fn attitude_hold_output_stays_finite() {
    let mut controller = FlightController::new(Config::default());
    send_channels(
        &mut controller,
        RcChannels::from_micros([
            1_650, 1_400, 1_400, 1_550, 1_800, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000,
            1_000, 1_000, 1_000, 1_000,
        ]),
        20,
    );
    let output = controller.update(level_sample(), 20, CONTROL_PERIOD);
    assert_eq!(output.mode, Mode::AttitudeHold);
    assert!(
        output
            .control
            .pulses
            .iter()
            .all(|pulse| pulse.as_micros() > 0)
    );
    assert!(output.estimate.euler.is_finite());
}

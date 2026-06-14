use fugit::MicrosDurationU32;
use glam::Vec3;
use nalgebra::SMatrix;

use crate::{
    ControlEffectiveness, IndiAllocator, IndiAllocatorConfig, IndiAttitudeConfig,
    IndiAttitudeController, IndiAxis, IndiAxisConfig, IndiAxisInput, IndiRateController,
};

fn config() -> IndiAxisConfig {
    IndiAxisConfig {
        control_effectiveness: 20.0,
        rate_gain: 6.0,
        acceleration_feedforward_gain: 1.0,
        acceleration_limit_rad_s2: 30.0,
        actuator_min: -1.0,
        actuator_trim: 0.1,
        actuator_max: 1.0,
        actuator_slew_rate_per_s: 5.0,
        acceleration_filter_tau_s: 0.0,
        actuator_filter_tau_s: 0.0,
    }
}

#[test]
fn zero_rate_error_keeps_trim() {
    let mut axis = IndiAxis::new(config());
    let output =
        axis.update_rate_with_acceleration(0.0, 0.0, 0.0, MicrosDurationU32::from_millis(20));
    assert_eq!(output.actuator, 0.1);
    assert_eq!(output.actuator_delta, 0.0);
}

#[test]
fn positive_rate_error_increases_actuator() {
    let mut axis = IndiAxis::new(config());
    let output =
        axis.update_rate_with_acceleration(1.0, 0.0, 0.0, MicrosDurationU32::from_millis(20));
    assert!(output.actuator > 0.1);
    assert!(output.desired_angular_accel_rad_s2 > 0.0);
}

#[test]
fn actuator_feedback_becomes_reference() {
    let mut axis = IndiAxis::new(config());
    let output = axis.update(
        IndiAxisInput::rate(0.0, 0.0).with_actuator_feedback(0.4),
        MicrosDurationU32::from_millis(20),
    );
    assert_eq!(output.actuator_reference, 0.4);
}

#[test]
fn saturation_is_enforced() {
    let mut axis = IndiAxis::new(IndiAxisConfig {
        actuator_trim: 0.95,
        actuator_slew_rate_per_s: 100.0,
        ..config()
    });
    let output =
        axis.update_rate_with_acceleration(10.0, 0.0, 0.0, MicrosDurationU32::from_millis(100));
    assert_eq!(output.actuator, 1.0);
}

#[test]
fn internal_acceleration_estimate_uses_previous_rate() {
    let mut axis = IndiAxis::new(config());
    let dt = MicrosDurationU32::from_millis(100);

    let first = axis.update_rate(0.0, 0.0, dt);
    assert_eq!(first.measured_angular_accel_rad_s2, 0.0);

    let second = axis.update_rate(0.0, 0.2, dt);
    assert!((second.measured_angular_accel_rad_s2 - 2.0).abs() < 1.0e-4);
    assert!(second.actuator < first.actuator);
}

#[test]
fn three_axis_controller_updates_all_axes() {
    let axis = IndiAxis::new(config());
    let mut controller = IndiRateController::new(axis, axis, axis);
    let output = controller.update_rates_with_acceleration(
        Vec3::new(1.0, -1.0, 0.5),
        Vec3::ZERO,
        Vec3::ZERO,
        MicrosDurationU32::from_millis(20),
    );
    assert!(output.actuator.x > 0.0);
    assert!(output.actuator.y < 0.1);
    assert!(output.actuator.z > 0.1);
}

#[test]
fn attitude_controller_generates_rate_targets() {
    let axis = IndiAxis::new(config());
    let mut controller = IndiAttitudeController::new(
        IndiRateController::new(axis, axis, axis),
        IndiAttitudeConfig::fixed_wing_default(),
    );
    let output = controller.update_fixed_wing(
        0.2,
        -0.1,
        0.0,
        Vec3::ZERO,
        Vec3::ZERO,
        MicrosDurationU32::from_millis(20),
    );
    assert!(output.desired_rates_rad_s.x > 0.0);
    assert!(output.desired_rates_rad_s.y < 0.0);
}

#[test]
fn allocator_splits_roll_between_two_ailerons() {
    let effectiveness = ControlEffectiveness::new(
        SMatrix::<f32, 3, 2>::from_row_slice(&[10.0, -10.0, 0.0, 0.0, 0.0, 0.0]),
        0.1,
    );
    let mut allocator = IndiAllocator::new(IndiAllocatorConfig::normalized(effectiveness, 100.0));
    let output = allocator
        .update(
            Vec3::new(4.0, 0.0, 0.0),
            Vec3::ZERO,
            MicrosDurationU32::from_millis(20),
        )
        .unwrap();
    assert!(output.actuator[0] > 0.0);
    assert!(output.actuator[1] < 0.0);
}

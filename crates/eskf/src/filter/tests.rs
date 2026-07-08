use fugit::MicrosDurationU32;
use glam::{EulerRot, Quat, Vec3};

use super::super::Eskf;

#[test]
fn predict_keeps_stationary_state_stable() {
    let mut filter = Eskf::new();
    filter.predict(
        Vec3::ZERO,
        Vec3::new(0.0, 0.0, 9.80665),
        MicrosDurationU32::from_secs(1),
    );
    assert!(filter.velocity.x.abs() < 1.0e-6);
    assert!(filter.velocity.z.abs() < 1.0e-6);
}

#[test]
fn velocity_update_moves_state_toward_measurement() {
    let mut filter = Eskf::new();
    let before = filter.covariance[(0, 0)];
    filter.correct_velocity(Vec3::new(1.0, 0.0, 0.0), 0.1);
    assert!(filter.velocity.x > 0.0);
    assert!(filter.covariance[(0, 0)] < before);
}

#[test]
fn orientation_update_applies_small_rotation() {
    let mut filter = Eskf::new();
    let measured = Quat::from_scaled_axis(Vec3::new(0.1, 0.0, 0.0));
    filter.correct_orientation(measured, 0.1);
    assert!(filter.orientation.x > 0.0);
}

#[test]
fn heading_update_preserves_roll_and_pitch() {
    let mut filter = Eskf::new();
    filter.orientation = Quat::from_euler(
        EulerRot::XYZ,
        20.0f32.to_radians(),
        -15.0f32.to_radians(),
        30.0f32.to_radians(),
    );

    let (roll_before, pitch_before, _) = filter.orientation.to_euler(EulerRot::XYZ);
    filter.correct_heading(40.0f32.to_radians(), 0.1);
    let (roll_after, pitch_after, yaw_after) = filter.orientation.to_euler(EulerRot::XYZ);

    assert!((roll_after - roll_before).abs() < 0.05);
    assert!((pitch_after - pitch_before).abs() < 0.05);
    assert!(yaw_after > 30.0f32.to_radians());
}

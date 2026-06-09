use fugit::MicrosDurationU32;
use math::{Quat, Vec3};

use super::super::Eskf;

#[test]
fn predict_keeps_stationary_state_stable() {
    let mut filter = Eskf::new();
    filter.predict(
        Vec3::zero(),
        Vec3::new(0.0, 0.0, 9.80665),
        MicrosDurationU32::from_secs(1),
    );
    assert!(filter.position.x.abs() < 1.0e-6);
    assert!(filter.velocity.z.abs() < 1.0e-6);
}

#[test]
fn position_update_moves_state_toward_measurement() {
    let mut filter = Eskf::new();
    let before = filter.covariance.data[0][0];
    filter.correct_position(Vec3::new(1.0, 0.0, 0.0), 0.1);
    assert!(filter.position.x > 0.0);
    assert!(filter.covariance.data[0][0] < before);
}

#[test]
fn orientation_update_applies_small_rotation() {
    let mut filter = Eskf::new();
    let measured = Quat::from_small_angle(Vec3::new(0.1, 0.0, 0.0));
    filter.correct_orientation(measured, 0.1);
    assert!(filter.orientation.x > 0.0);
}

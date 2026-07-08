use core::f32::consts::PI;

use glam::{EulerRot, Quat};

use crate::{AccelGyroSample, Attitude, Quaternion, Vector3};

/// Converts raw STEMMA QT 9-DoF sensor axes into the estimator body frame.
///
/// Uses the sensor's native right-handed frame as the estimator body frame.
///
/// In other words, body-frame vectors are the raw sensor vectors after unit
/// conversion only. Any remaining mismatch should be fixed by aligning the
/// visualizer or by confirming the expected mechanical mounting, not by adding
/// ad-hoc sign flips here.
pub fn stemma_qt_9dof_body_vector(sensor_vector: Vector3) -> Vector3 {
    sensor_vector
}

/// Converts raw STEMMA QT accelerometer/gyroscope readings into one IMU sample.
pub fn stemma_qt_9dof_accel_gyro_sample(
    accel_g: Vector3,
    gyro_dps: Vector3,
    gravity_m_s2: f32,
) -> AccelGyroSample {
    AccelGyroSample::without_temperature(
        stemma_qt_9dof_body_vector(accel_g) * gravity_m_s2,
        stemma_qt_9dof_body_vector(gyro_dps).map(f32::to_radians),
    )
}

/// Converts the internal world-frame attitude into the user-facing display
/// convention while keeping the level pose unchanged.
pub fn display_orientation(orientation: Quaternion) -> Quaternion {
    let convention = Quat::from_rotation_y(PI);
    (convention * orientation * convention).normalize()
}

/// Converts the internal orientation into user-facing XYZ Euler angles.
pub fn display_attitude(orientation: Quaternion) -> Attitude {
    let orientation = display_orientation(orientation);
    let (roll_rad, pitch_rad, yaw_rad) = orientation.to_euler(EulerRot::XYZ);
    Attitude::new(roll_rad, pitch_rad, yaw_rad)
}

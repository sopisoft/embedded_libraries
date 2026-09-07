use core::f32::consts::PI;

use glam::{EulerRot, Quat};

use crate::{AccelGyroSample, Acceleration, AngularVelocity, Vec3};

/// Converts raw STEMMA QT 9-DoF sensor axes into the estimator body frame.
/// Uses the sensor's native right-handed frame as the estimator body frame.
pub fn stemma_qt_9dof_body_vector(sensor_vector: Vec3) -> Vec3 {
    sensor_vector
}

/// Converts raw STEMMA QT accelerometer/gyroscope readings into one IMU sample.
pub fn stemma_qt_9dof_accel_gyro_sample(
    accel_g: Vec3,
    gyro_dps: Vec3,
    gravity_m_s2: f32,
) -> AccelGyroSample {
    AccelGyroSample::without_temperature(
        Acceleration::new(stemma_qt_9dof_body_vector(accel_g) * gravity_m_s2),
        AngularVelocity::new(stemma_qt_9dof_body_vector(gyro_dps).map(f32::to_radians)),
    )
}

pub fn display_orientation(orientation: Quat) -> Quat {
    let convention = Quat::from_rotation_y(PI);
    (convention * orientation * convention).normalize()
}

/// Converts the internal orientation into user-facing XYZ Euler angles.
pub fn display_attitude(orientation: Quat) -> Vec3 {
    let orientation = display_orientation(orientation);
    let (roll_rad, pitch_rad, yaw_rad) = orientation.to_euler(EulerRot::XYZ);
    Vec3::new(roll_rad, pitch_rad, yaw_rad)
}

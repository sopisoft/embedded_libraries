//! Shared trait for attitude estimators.

use fugit::MicrosDurationU32;
use math::{Quat, Vec3};

/// Common interface for IMU-based attitude estimators.
pub trait AttitudeEstimator {
    /// Advances the estimator with gyroscope and accelerometer data.
    fn update_imu(&mut self, gyro_rad_s: Vec3, accel: Vec3, dt: MicrosDurationU32);

    /// Returns the current orientation estimate.
    fn orientation(&self) -> Quat;
}

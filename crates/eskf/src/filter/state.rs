use glam::{Quat, Vec3};

use super::Covariance;

/// A 12-state error-state Kalman filter.
///
/// State error ordering:
/// - `0..3`: velocity
/// - `3..6`: attitude
/// - `6..9`: accelerometer bias
/// - `9..12`: gyroscope bias
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Eskf {
    /// Velocity estimate in the world frame.
    pub velocity: Vec3,
    /// Orientation estimate.
    pub orientation: Quat,
    /// Accelerometer bias estimate.
    pub accel_bias: Vec3,
    /// Gyroscope bias estimate.
    pub gyro_bias: Vec3,
    /// State covariance.
    pub covariance: Covariance,
    /// Accelerometer white-noise density.
    pub accel_noise: f32,
    /// Gyroscope white-noise density.
    pub gyro_noise: f32,
    /// Accelerometer bias random walk.
    pub accel_bias_noise: f32,
    /// Gyroscope bias random walk.
    pub gyro_bias_noise: f32,
    /// Gravity vector in the world frame.
    pub gravity: Vec3,
}

impl Eskf {
    /// Creates a filter with conservative default noise values.
    pub fn new() -> Self {
        Self {
            velocity: Vec3::ZERO,
            orientation: Quat::IDENTITY,
            accel_bias: Vec3::ZERO,
            gyro_bias: Vec3::ZERO,
            covariance: Covariance::identity_scaled(1.0e-3),
            accel_noise: 0.5,
            gyro_noise: 0.05,
            accel_bias_noise: 0.01,
            gyro_bias_noise: 0.001,
            gravity: Vec3::new(0.0, 0.0, -9.80665),
        }
    }

    /// Updates the process-noise tuning values.
    pub fn set_noise(
        &mut self,
        accel_noise: f32,
        gyro_noise: f32,
        accel_bias_noise: f32,
        gyro_bias_noise: f32,
    ) {
        self.accel_noise = accel_noise;
        self.gyro_noise = gyro_noise;
        self.accel_bias_noise = accel_bias_noise;
        self.gyro_bias_noise = gyro_bias_noise;
    }
}

impl Default for Eskf {
    fn default() -> Self {
        Self::new()
    }
}

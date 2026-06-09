use math::{Matrix, Quat, Vec3};

/// A 15-state error-state Kalman filter.
///
/// State error ordering:
/// - `0..3`: position
/// - `3..6`: velocity
/// - `6..9`: attitude
/// - `9..12`: accelerometer bias
/// - `12..15`: gyroscope bias
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Eskf {
    /// Position estimate in the world frame.
    pub position: Vec3,
    /// Velocity estimate in the world frame.
    pub velocity: Vec3,
    /// Orientation estimate.
    pub orientation: Quat,
    /// Accelerometer bias estimate.
    pub accel_bias: Vec3,
    /// Gyroscope bias estimate.
    pub gyro_bias: Vec3,
    /// State covariance.
    pub covariance: Matrix<15, 15>,
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
            position: Vec3::zero(),
            velocity: Vec3::zero(),
            orientation: Quat::identity(),
            accel_bias: Vec3::zero(),
            gyro_bias: Vec3::zero(),
            covariance: Matrix::identity() * 1.0e-3,
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

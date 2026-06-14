//! Common IMU sample structures.

use crate::Vector3;

/// One accelerometer + gyroscope sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AccelGyroSample {
    /// Specific force in m/s^2 in the body frame.
    pub accel_m_s2: Vector3,
    /// Angular rate in rad/s in the body frame.
    pub gyro_rad_s: Vector3,
    /// Optional sensor temperature in degrees Celsius.
    pub temperature_c: Option<f32>,
}

impl AccelGyroSample {
    /// Creates a new sample.
    pub const fn new(accel_m_s2: Vector3, gyro_rad_s: Vector3, temperature_c: Option<f32>) -> Self {
        Self {
            accel_m_s2,
            gyro_rad_s,
            temperature_c,
        }
    }

    /// Creates a sample without temperature data.
    pub const fn without_temperature(accel_m_s2: Vector3, gyro_rad_s: Vector3) -> Self {
        Self::new(accel_m_s2, gyro_rad_s, None)
    }
}

/// One accelerometer + gyroscope + magnetometer sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MargSample {
    /// Accelerometer and gyroscope data.
    pub accel_gyro: AccelGyroSample,
    /// Magnetic field vector in the body frame.
    pub mag_body: Vector3,
}

impl MargSample {
    /// Creates a new MARG sample.
    pub const fn new(accel_gyro: AccelGyroSample, mag_body: Vector3) -> Self {
        Self {
            accel_gyro,
            mag_body,
        }
    }
}

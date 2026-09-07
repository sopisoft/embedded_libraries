//! Common IMU sample structures.

use crate::Vec3;

#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Acceleration(Vec3);

impl Acceleration {
    pub const fn new(value: Vec3) -> Self {
        Self(value)
    }

    pub const fn vector(self) -> Vec3 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AngularVelocity(Vec3);

impl AngularVelocity {
    pub const fn new(value: Vec3) -> Self {
        Self(value)
    }

    pub const fn vector(self) -> Vec3 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MagneticField(Vec3);

impl MagneticField {
    pub const fn new(value: Vec3) -> Self {
        Self(value)
    }

    pub const fn vector(self) -> Vec3 {
        self.0
    }
}

/// One accelerometer + gyroscope sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AccelGyroSample {
    /// Specific force in m/s^2 in the body frame.
    pub accel_m_s2: Acceleration,
    /// Angular rate in rad/s in the body frame.
    pub gyro_rad_s: AngularVelocity,
    /// Optional sensor temperature in degrees Celsius.
    pub temperature_c: Option<f32>,
}

impl AccelGyroSample {
    /// Creates a new sample.
    pub const fn new(
        accel_m_s2: Acceleration,
        gyro_rad_s: AngularVelocity,
        temperature_c: Option<f32>,
    ) -> Self {
        Self {
            accel_m_s2,
            gyro_rad_s,
            temperature_c,
        }
    }

    /// Creates a sample without temperature data.
    pub const fn without_temperature(
        accel_m_s2: Acceleration,
        gyro_rad_s: AngularVelocity,
    ) -> Self {
        Self::new(accel_m_s2, gyro_rad_s, None)
    }

    pub const fn from_vectors(
        accel_m_s2: Vec3,
        gyro_rad_s: Vec3,
        temperature_c: Option<f32>,
    ) -> Self {
        Self::new(
            Acceleration::new(accel_m_s2),
            AngularVelocity::new(gyro_rad_s),
            temperature_c,
        )
    }

    pub const fn from_vectors_without_temperature(accel_m_s2: Vec3, gyro_rad_s: Vec3) -> Self {
        Self::from_vectors(accel_m_s2, gyro_rad_s, None)
    }
}

/// One accelerometer + gyroscope + magnetometer sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MargSample {
    /// Accelerometer and gyroscope data.
    pub accel_gyro: AccelGyroSample,
    /// Magnetic field vector in the body frame.
    pub mag_body: MagneticField,
}

impl MargSample {
    /// Creates a new MARG sample.
    pub const fn new(accel_gyro: AccelGyroSample, mag_body: MagneticField) -> Self {
        Self {
            accel_gyro,
            mag_body,
        }
    }

    pub const fn from_vectors(accel_gyro: AccelGyroSample, mag_body: Vec3) -> Self {
        Self::new(accel_gyro, MagneticField::new(mag_body))
    }
}

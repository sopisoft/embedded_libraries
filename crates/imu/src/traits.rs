//! Sensor traits that keep estimation code independent from hardware drivers.

use crate::Vector3;
use crate::sample::{AccelGyroSample, MargSample};

/// Source for one accelerometer + gyroscope sample.
pub trait AccelGyroSource {
    type Error;

    fn read_accel_gyro(&mut self) -> Result<AccelGyroSample, Self::Error>;
}

/// Source for one magnetic-field sample.
pub trait MagnetometerSource {
    type Error;

    fn read_magnetometer(&mut self) -> Result<Vector3, Self::Error>;
}

/// Source for one full MARG sample.
pub trait MargSource {
    type Error;

    fn read_marg(&mut self) -> Result<MargSample, Self::Error>;
}

/// Error returned when combining two different sensor drivers.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MargReadError<AccelGyroError, MagnetometerError> {
    AccelGyro(AccelGyroError),
    Magnetometer(MagnetometerError),
}

/// Generic helper that combines a 6-DoF source with a magnetometer source.
#[derive(Debug)]
pub struct CombinedMargSource<AccelGyro, Mag> {
    accel_gyro: AccelGyro,
    magnetometer: Mag,
}

impl<AccelGyro, Mag> CombinedMargSource<AccelGyro, Mag> {
    /// Creates a combined source.
    pub const fn new(accel_gyro: AccelGyro, magnetometer: Mag) -> Self {
        Self {
            accel_gyro,
            magnetometer,
        }
    }

    /// Returns mutable access to the accelerometer/gyro source.
    pub fn accel_gyro_mut(&mut self) -> &mut AccelGyro {
        &mut self.accel_gyro
    }

    /// Returns mutable access to the magnetometer source.
    pub fn magnetometer_mut(&mut self) -> &mut Mag {
        &mut self.magnetometer
    }

    /// Releases the inner sources.
    pub fn into_inner(self) -> (AccelGyro, Mag) {
        (self.accel_gyro, self.magnetometer)
    }
}

impl<AccelGyro, Mag> MargSource for CombinedMargSource<AccelGyro, Mag>
where
    AccelGyro: AccelGyroSource,
    Mag: MagnetometerSource,
{
    type Error = MargReadError<AccelGyro::Error, Mag::Error>;

    fn read_marg(&mut self) -> Result<MargSample, Self::Error> {
        let accel_gyro = self
            .accel_gyro
            .read_accel_gyro()
            .map_err(MargReadError::AccelGyro)?;
        let mag_body = self
            .magnetometer
            .read_magnetometer()
            .map_err(MargReadError::Magnetometer)?;
        Ok(MargSample::new(accel_gyro, mag_body))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vector3;

    #[derive(Copy, Clone)]
    struct MockImu;

    impl AccelGyroSource for MockImu {
        type Error = ();

        fn read_accel_gyro(&mut self) -> Result<AccelGyroSample, Self::Error> {
            Ok(AccelGyroSample::without_temperature(
                Vector3::new(0.0, 0.0, 9.80665),
                Vector3::ZERO,
            ))
        }
    }

    #[derive(Copy, Clone)]
    struct MockMag;

    impl MagnetometerSource for MockMag {
        type Error = ();

        fn read_magnetometer(&mut self) -> Result<Vector3, Self::Error> {
            Ok(Vector3::X)
        }
    }

    #[test]
    fn combined_source_reads_both_sensors() {
        let mut source = CombinedMargSource::new(MockImu, MockMag);
        let sample = source.read_marg().unwrap();
        assert_eq!(sample.accel_gyro.gyro_rad_s, Vector3::ZERO);
        assert_eq!(sample.mag_body, Vector3::X);
    }
}

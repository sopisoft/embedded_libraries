#![no_std]

//! Generic IMU sample types, bus helpers, and lightweight estimation glue.

#[cfg(test)]
extern crate std;

pub mod bus;
pub mod calibration;
pub mod estimation;
pub mod frame;
pub mod sample;
pub mod traits;

pub use bus::{SharedI2c, find_i2c_address_by_id};
pub use calibration::{
    AllanDeviationPoint, AllanImuCalibration, AllanImuCalibrator, AllanNoiseSummary, ImuBiases,
    MagnetometerCalibrator, StationaryImuCalibrator,
};
pub use estimation::{
    EskfEstimator, EskfTuning, ImuEstimate, ImuEstimator, MargEstimator, NavigatorState,
    StationaryDetection,
};
pub use frame::{
    display_attitude, display_orientation, stemma_qt_9dof_accel_gyro_sample,
    stemma_qt_9dof_body_vector,
};
pub use sample::{AccelGyroSample, MargSample};
pub use traits::{
    AccelGyroSource, CombinedMargSource, MagnetometerSource, MargReadError, MargSource,
};
pub type Attitude = glam::Vec3;
pub type Quaternion = glam::Quat;
pub type Vector3 = glam::Vec3;

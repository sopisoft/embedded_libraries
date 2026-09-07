#![no_std]

//! Generic IMU sample types, bus helpers, and lightweight estimation glue.

#[cfg(test)]
extern crate std;

pub mod bus;
pub mod calibration;
pub mod estimation;
pub mod frame;
pub mod sample;

pub use bus::{SharedI2c, find_i2c_address_by_id};
pub use calibration::{
    AllanDeviationPoint, AllanImuCalibration, AllanImuCalibrator, AllanNoiseSummary, ImuBiases,
    MagnetometerCalibrator, StationaryImuCalibrator,
};
pub use estimation::{
    EskfEstimator, EskfTuning, ImuEstimate, MargEstimator, NavigatorState, StationaryDetection,
};
pub use frame::{
    display_attitude, display_orientation, stemma_qt_9dof_accel_gyro_sample,
    stemma_qt_9dof_body_vector,
};
pub use glam::{Quat, Vec3};
pub use sample::{AccelGyroSample, MargSample};
pub use sample::{Acceleration, AngularVelocity, MagneticField};

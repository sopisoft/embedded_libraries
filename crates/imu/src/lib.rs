#![no_std]

//! Generic IMU sample types, bus helpers, and lightweight estimation glue.

#[cfg(test)]
extern crate std;

pub mod bus;
pub mod estimation;
pub mod sample;
pub mod traits;

pub use bus::SharedI2c;
pub use estimation::{ImuEstimate, MargEstimator, NavigatorState, StationaryDetection};
pub use sample::{AccelGyroSample, MargSample};
pub use traits::{
    AccelGyroSource, CombinedMargSource, MagnetometerSource, MargReadError, MargSource,
};
pub type Attitude = glam::Vec3;
pub type Quaternion = glam::Quat;
pub type Vector3 = glam::Vec3;

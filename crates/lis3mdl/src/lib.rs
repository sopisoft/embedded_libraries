#![no_std]

//! `embedded-hal` 1.0 driver for the ST LIS3MDL 3-axis magnetometer.

#[cfg(test)]
extern crate std;

mod device;
mod registers;
#[cfg(test)]
mod tests;
mod types;

pub use device::Lis3mdl;
pub use types::{
    Address, Config, DEVICE_ID, DataRate, Error, FullScale, MagneticField, MeasurementMode,
    OperatingMode, RawMagneticField,
};

#![no_std]

//! `embedded-hal` 1.0 driver for the ST LPS25HB pressure sensor.
//!
//! This crate supports both I2C and 4-wire SPI access.
//!
//! The implementation is split into:
//! - a transport-independent core driver,
//! - `i2c` transport glue,
//! - `spi` transport glue,
//! - small conversion helpers for pressure, temperature, and altitude.

#[cfg(test)]
extern crate std;

mod device;
mod interface;
mod registers;
mod types;

pub mod i2c;
pub mod spi;

pub use device::{
    Lps25hb, altitude_to_pressure_hpa, one_point_calibration_rpds, pressure_error_to_rpds_counts,
    pressure_to_altitude_m, raw_pressure_to_hpa, raw_temperature_to_celsius,
    rpds_counts_to_pressure_hpa,
};
pub use types::{
    Address, Config, DEVICE_ID, Error, Measurement, OutputDataRate, PressureAverage,
    RawMeasurement, STANDARD_SEA_LEVEL_PRESSURE_HPA, Status, TemperatureAverage,
};

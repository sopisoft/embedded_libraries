#![no_std]

//! `no_std` analog pressure driver for the MIS-2500 series.

#[cfg(test)]
extern crate std;

mod adc;
mod driver;
mod error;
mod pressure;
#[cfg(test)]
mod tests;

pub use adc::AdcConfig;
pub use driver::{
    FULL_SCALE_PRESSURE_015G_PA, FULL_SCALE_PRESSURE_015V_PA, Mis2500, NOMINAL_SUPPLY_VOLTAGE_V,
    OUTPUT_SPAN_RATIO, TYPICAL_ZERO_OUTPUT_RATIO,
};
pub use error::Error;
pub use pressure::Pressure;

#![no_std]

//! Conventional fixed-wing flight-controller runtime.
//!
//! The board supplies one `imu::MargSample` per control tick. This crate owns
//! CRSF input, failsafe, attitude estimation, and actuator pulses.

#[cfg(test)]
extern crate std;

mod config;
mod runtime;
#[cfg(test)]
mod tests;

pub use config::{
    CONTROL_PERIOD, Config, DEFAULT_FAILSAFE_TIMEOUT, GS1502_CHANNEL, GS1502_SWITCH_THRESHOLD_US,
    OUTPUT_COUNT, OutputChannel, default_servos,
};
pub use runtime::{FlightController, Gs1502Position, Mode, Output};

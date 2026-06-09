#![no_std]

//! Incremental Nonlinear Dynamic Inversion primitives.
//!
//! This crate provides three layers:
//! - one-axis INDI rate control,
//! - three-axis body-rate and attitude wrappers,
//! - a small 3xN control-effectiveness allocator.

#[cfg(test)]
extern crate std;

mod allocation;
mod attitude;
mod axis;
mod filter;
mod rate;

#[cfg(test)]
mod tests;

pub use allocation::{
    ControlEffectiveness, IndiAllocator, IndiAllocatorConfig, IndiAllocatorOutput,
};
pub use attitude::{
    IndiAttitudeConfig, IndiAttitudeController, IndiAttitudeOutput, IndiFixedWingInput,
};
pub use axis::{IndiAxis, IndiAxisConfig, IndiAxisInput, IndiAxisOutput};
pub use filter::LowPassFilter;
pub use rate::{IndiOutputs, IndiRateController, IndiRateInput};

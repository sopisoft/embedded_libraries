#![no_std]

//! Cascaded stabilization helpers for aircraft and other embedded vehicles.
//!
//! The crate focuses on the common "attitude loop outside, rate loop inside"
//! architecture used by many flight controllers.

#[cfg(test)]
extern crate std;

pub mod cascade;

pub use cascade::{AxisErrorMode, CascadeAttitudeController, CascadeAxis, CascadeOutputs};

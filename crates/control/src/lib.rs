#![no_std]

//! Control primitives for embedded vehicles.
//!
//! The crate provides three layers that are commonly needed between estimation
//! and actuation:
//!
//! - pilot-input shaping such as deadband and expo
//! - time-based PID control using `fugit`
//! - fixed-wing output mixing for conventional, elevon, and V-tail airframes

#[cfg(test)]
extern crate std;

pub mod input;
pub mod mixing;
pub mod pid;

pub use input::{apply_deadband, apply_dual_rate, apply_expo, shape_rc_command};
pub use mixing::{
    ControlAxes, ConventionalTailMixer, ConventionalTailOutputs, ElevonMixer, ElevonOutputs,
    SurfaceChannel, ThrottleChannel, VTailMixer, VTailOutputs,
};
pub use pid::PidController;

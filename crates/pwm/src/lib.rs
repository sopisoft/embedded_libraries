#![no_std]

//! PWM helpers for hobby servos and ESCs.

#[cfg(test)]
extern crate std;

pub mod esc;
pub mod servo;

pub use esc::Esc;
pub use servo::{Servo, ServoBank, ServoOutput, ServoRange, ServoSet};

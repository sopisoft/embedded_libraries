#![no_std]

//! Error-state Kalman filter for inertial navigation.

#[cfg(test)]
extern crate std;

pub mod filter;

pub use filter::Eskf;

#![no_std]

//! Madgwick AHRS filter.

#[cfg(test)]
extern crate std;

pub mod filter;

pub use filter::Madgwick;

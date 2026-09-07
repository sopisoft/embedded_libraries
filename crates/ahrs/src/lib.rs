#![no_std]

//! Common attitude-estimation traits and filters.

#[cfg(test)]
extern crate std;

pub mod complementary;
pub mod madgwick;

pub use complementary::ComplementaryAttitudeFilter;
pub use madgwick::Madgwick;

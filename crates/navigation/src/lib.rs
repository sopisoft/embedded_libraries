#![no_std]

//! Navigation and dead-reckoning estimators.

#[cfg(test)]
extern crate std;

pub mod fixed_wing;
pub mod inertial;

pub use fixed_wing::FixedWingNavigator;
pub use inertial::InertialNavigator;

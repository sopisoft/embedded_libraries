#![no_std]

//! Total Energy Control System helpers for fixed-wing aircraft.

#[cfg(test)]
extern crate std;

mod fixed_wing;

pub use fixed_wing::{
    TecsConfig, TecsController, TecsOutput, TecsState, TecsTarget, specific_kinetic_energy,
    specific_potential_energy, specific_total_energy,
};

#![no_std]

//! Sensor-agnostic glue for fixed-wing RC aircraft control pipelines.

#[cfg(test)]
extern crate std;

pub mod fixed_wing;
pub mod input;

pub use fixed_wing::{
    AttitudeHoldLimits, AttitudeHoldOutput, DefaultAttitudeController, ElevonControlOutput,
    ElevonController, ElevonServoMap, FixedWingAttitudeBackend, FixedWingControlOutput,
    FixedWingController, ServoAssignment, ServoCommandMode, ServoMap, VTailControlOutput,
    VTailController, VTailServoMap,
};
pub use input::{
    AxisConfig, PilotCommand, RcChannelMap, RcInputConfig, SwitchConfig, apply_subset_channels,
};
pub type Attitude = glam::Vec3;
pub type Vector3 = glam::Vec3;

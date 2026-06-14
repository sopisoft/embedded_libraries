#![no_std]

//! State propagation and kinematic helpers.

#[cfg(test)]
extern crate std;

pub mod fixed_wing;
pub mod state;
pub mod types;

pub use fixed_wing::{FixedWingState, coordinated_turn_rate};
pub use state::{MotionState2, MotionState3, PlanarMotion, SpatialMotion};
pub use types::{Pose2, Pose3, Twist2, Twist3};

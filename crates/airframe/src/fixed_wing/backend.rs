use fugit::MicrosDurationU32;
#[cfg(feature = "indi")]
use indi::IndiAttitudeController;
#[cfg(feature = "cascade-pid")]
use stabilization::CascadeAttitudeController;

use crate::{Attitude, Vector3};

#[cfg(not(any(feature = "cascade-pid", feature = "indi")))]
compile_error!("airframe needs either the `cascade-pid` or `indi` feature enabled");

/// Default backend used when the controller type parameter is omitted.
#[cfg(feature = "cascade-pid")]
pub type DefaultAttitudeController = CascadeAttitudeController;

/// Default backend used when only the INDI feature is enabled.
#[cfg(all(not(feature = "cascade-pid"), feature = "indi"))]
pub type DefaultAttitudeController = IndiAttitudeController;

/// Backend-independent attitude controller output.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct AttitudeHoldOutput {
    pub actuator: Vector3,
    pub desired_rates_rad_s: Vector3,
}

/// Pilot-stick-to-attitude limits for attitude-hold mode.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AttitudeHoldLimits {
    pub max_roll_rad: f32,
    pub max_pitch_rad: f32,
    pub max_yaw_rate_rad_s: f32,
}

impl Default for AttitudeHoldLimits {
    fn default() -> Self {
        Self {
            max_roll_rad: 45.0f32.to_radians(),
            max_pitch_rad: 20.0f32.to_radians(),
            max_yaw_rate_rad_s: 90.0f32.to_radians(),
        }
    }
}

/// Common interface implemented by supported attitude-hold backends.
pub trait FixedWingAttitudeBackend {
    /// Clears controller state.
    fn reset(&mut self);

    /// Runs roll/pitch attitude hold and yaw-rate hold.
    fn update_fixed_wing(
        &mut self,
        target_roll_rad: f32,
        target_pitch_rad: f32,
        target_yaw_rate_rad_s: f32,
        measured_attitude: Attitude,
        measured_rates_rad_s: Vector3,
        dt: MicrosDurationU32,
    ) -> AttitudeHoldOutput;
}

#[cfg(feature = "cascade-pid")]
impl FixedWingAttitudeBackend for CascadeAttitudeController {
    fn reset(&mut self) {
        CascadeAttitudeController::reset(self);
    }

    fn update_fixed_wing(
        &mut self,
        target_roll_rad: f32,
        target_pitch_rad: f32,
        target_yaw_rate_rad_s: f32,
        measured_attitude: Attitude,
        measured_rates_rad_s: Vector3,
        dt: MicrosDurationU32,
    ) -> AttitudeHoldOutput {
        let output = CascadeAttitudeController::update_fixed_wing(
            self,
            target_roll_rad,
            target_pitch_rad,
            target_yaw_rate_rad_s,
            measured_attitude,
            measured_rates_rad_s,
            dt,
        );
        AttitudeHoldOutput {
            actuator: output.actuator,
            desired_rates_rad_s: output.desired_rates_rad_s,
        }
    }
}

#[cfg(feature = "indi")]
impl FixedWingAttitudeBackend for IndiAttitudeController {
    fn reset(&mut self) {
        IndiAttitudeController::reset(self);
    }

    fn update_fixed_wing(
        &mut self,
        target_roll_rad: f32,
        target_pitch_rad: f32,
        target_yaw_rate_rad_s: f32,
        measured_attitude: Attitude,
        measured_rates_rad_s: Vector3,
        dt: MicrosDurationU32,
    ) -> AttitudeHoldOutput {
        let output = IndiAttitudeController::update_fixed_wing(
            self,
            target_roll_rad,
            target_pitch_rad,
            target_yaw_rate_rad_s,
            measured_attitude,
            measured_rates_rad_s,
            dt,
        );
        AttitudeHoldOutput {
            actuator: output.actuator,
            desired_rates_rad_s: output.desired_rates_rad_s,
        }
    }
}

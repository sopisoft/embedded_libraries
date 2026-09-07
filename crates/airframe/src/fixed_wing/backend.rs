use control::{ControlAxes, SignedNormalized};
use fugit::MicrosDurationU32;
#[cfg(feature = "indi")]
use indi::IndiAttitudeController;
#[cfg(feature = "cascade-pid")]
use stabilization::CascadeAttitudeController;

use crate::{PilotCommand, Vec3};

#[cfg(not(any(feature = "cascade-pid", feature = "indi")))]
compile_error!("airframe needs either the `cascade-pid` or `indi` feature enabled");

/// Default backend used when the controller type parameter is omitted.
#[cfg(feature = "cascade-pid")]
pub type DefaultAttitudeController = CascadeAttitudeController;

/// Default backend used when only the INDI feature is enabled.
#[cfg(all(not(feature = "cascade-pid"), feature = "indi"))]
pub type DefaultAttitudeController = IndiAttitudeController;

/// Pilot-stick-to-attitude limits for attitude-hold mode.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AttitudeHoldLimits {
    max_roll_rad: f32,
    max_pitch_rad: f32,
    max_yaw_rate_rad_s: f32,
}

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct AttitudeControl {
    pub roll: SignedNormalized,
    pub pitch: SignedNormalized,
    pub yaw: SignedNormalized,
}

impl AttitudeHoldLimits {
    pub const fn new(max_roll_rad: f32, max_pitch_rad: f32, max_yaw_rate_rad_s: f32) -> Self {
        assert!(max_roll_rad.is_finite() && max_roll_rad >= 0.0);
        assert!(max_pitch_rad.is_finite() && max_pitch_rad >= 0.0);
        assert!(max_yaw_rate_rad_s.is_finite() && max_yaw_rate_rad_s >= 0.0);
        Self {
            max_roll_rad,
            max_pitch_rad,
            max_yaw_rate_rad_s,
        }
    }

    pub const fn max_roll_rad(self) -> f32 {
        self.max_roll_rad
    }

    pub const fn max_pitch_rad(self) -> f32 {
        self.max_pitch_rad
    }

    pub const fn max_yaw_rate_rad_s(self) -> f32 {
        self.max_yaw_rate_rad_s
    }
}

impl Default for AttitudeHoldLimits {
    fn default() -> Self {
        Self::new(
            45.0f32.to_radians(),
            20.0f32.to_radians(),
            90.0f32.to_radians(),
        )
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
        measured_attitude: Vec3,
        measured_rates_rad_s: Vec3,
        dt: MicrosDurationU32,
    ) -> AttitudeControl;
}

pub(crate) fn manual_axes(pilot: PilotCommand) -> ControlAxes {
    ControlAxes::new(
        pilot.roll,
        pilot.pitch,
        pilot.yaw,
        pilot.throttle,
        pilot.flaps,
    )
}

pub(crate) fn attitude_hold_axes<C: FixedWingAttitudeBackend>(
    controller: &mut C,
    limits: AttitudeHoldLimits,
    pilot: PilotCommand,
    measured_attitude: Vec3,
    measured_rates_rad_s: Vec3,
    dt: MicrosDurationU32,
) -> ControlAxes {
    let actuator = controller.update_fixed_wing(
        pilot.roll.get() * limits.max_roll_rad(),
        pilot.pitch.get() * limits.max_pitch_rad(),
        pilot.yaw.get() * limits.max_yaw_rate_rad_s(),
        measured_attitude,
        measured_rates_rad_s,
        dt,
    );
    ControlAxes::new(
        actuator.roll,
        actuator.pitch,
        actuator.yaw,
        pilot.throttle,
        pilot.flaps,
    )
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
        measured_attitude: Vec3,
        measured_rates_rad_s: Vec3,
        dt: MicrosDurationU32,
    ) -> AttitudeControl {
        let output = CascadeAttitudeController::update_fixed_wing(
            self,
            target_roll_rad,
            target_pitch_rad,
            target_yaw_rate_rad_s,
            measured_attitude,
            measured_rates_rad_s,
            dt,
        );
        AttitudeControl {
            roll: SignedNormalized::saturated(output.actuator.x),
            pitch: SignedNormalized::saturated(output.actuator.y),
            yaw: SignedNormalized::saturated(output.actuator.z),
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
        measured_attitude: Vec3,
        measured_rates_rad_s: Vec3,
        dt: MicrosDurationU32,
    ) -> AttitudeControl {
        let output = IndiAttitudeController::update_fixed_wing(
            self,
            target_roll_rad,
            target_pitch_rad,
            target_yaw_rate_rad_s,
            measured_attitude,
            measured_rates_rad_s,
            dt,
        );
        AttitudeControl {
            roll: SignedNormalized::saturated(output.actuator.x),
            pitch: SignedNormalized::saturated(output.actuator.y),
            yaw: SignedNormalized::saturated(output.actuator.z),
        }
    }
}

use control::{ControlAxes, VTailMixer, VTailOutputs};
use fugit::MicrosDurationU32;
use math::{EulerAngles, Vec3};
use pwm::{ServoBank, ServoSet};

use crate::PilotCommand;

use super::{
    AttitudeHoldLimits, DefaultAttitudeController, FixedWingAttitudeBackend,
    common::{ServoAssignment, apply_assignment, neutral_pulses},
};

/// Servo map for a V-tail aircraft.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct VTailServoMap {
    pub left_tail: ServoAssignment,
    pub right_tail: ServoAssignment,
    pub aileron: ServoAssignment,
    pub throttle: ServoAssignment,
}

impl VTailServoMap {
    pub const fn four_channel() -> Self {
        Self {
            left_tail: ServoAssignment::symmetric(0),
            right_tail: ServoAssignment::symmetric(1),
            aileron: ServoAssignment::symmetric(2),
            throttle: ServoAssignment::normalized(3),
        }
    }

    pub fn to_pulses<const N: usize>(
        &self,
        surfaces: VTailOutputs,
        servos: &ServoSet<N>,
    ) -> [MicrosDurationU32; N] {
        let mut pulses = neutral_pulses(servos);
        apply_assignment(&mut pulses, servos, self.left_tail, surfaces.left_tail);
        apply_assignment(&mut pulses, servos, self.right_tail, surfaces.right_tail);
        apply_assignment(&mut pulses, servos, self.aileron, surfaces.aileron);
        apply_assignment(&mut pulses, servos, self.throttle, surfaces.throttle);
        pulses
    }
}

/// Output block for a V-tail airframe.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VTailControlOutput<const N: usize> {
    pub axes: ControlAxes,
    pub surfaces: VTailOutputs,
    pub pulses: [MicrosDurationU32; N],
}

impl<const N: usize> VTailControlOutput<N> {
    pub fn apply_to_servo_bank<E>(&self, bank: &mut ServoBank<'_, E, N>) -> Result<(), E> {
        bank.set_pulse_widths(self.pulses)
    }
}

/// High-level controller for V-tail aircraft.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VTailController<const N: usize, C = DefaultAttitudeController> {
    pub attitude_hold: C,
    pub mixer: VTailMixer,
    pub servos: ServoSet<N>,
    pub servo_map: VTailServoMap,
    pub limits: AttitudeHoldLimits,
}

impl<const N: usize, C> VTailController<N, C> {
    pub const fn new(
        attitude_hold: C,
        mixer: VTailMixer,
        servos: ServoSet<N>,
        servo_map: VTailServoMap,
        limits: AttitudeHoldLimits,
    ) -> Self {
        Self {
            attitude_hold,
            mixer,
            servos,
            servo_map,
            limits,
        }
    }

    pub fn update_manual(&self, pilot: PilotCommand) -> VTailControlOutput<N> {
        let axes = ControlAxes::new(pilot.roll, pilot.pitch, pilot.yaw, pilot.throttle, 0.0);
        let surfaces = self.mixer.mix(axes);
        let pulses = self.servo_map.to_pulses(surfaces, &self.servos);
        VTailControlOutput {
            axes,
            surfaces,
            pulses,
        }
    }

    pub fn update_attitude_hold(
        &mut self,
        pilot: PilotCommand,
        measured_attitude: EulerAngles,
        measured_rates_rad_s: Vec3,
        dt: MicrosDurationU32,
    ) -> VTailControlOutput<N>
    where
        C: FixedWingAttitudeBackend,
    {
        let stabilized = self.attitude_hold.update_fixed_wing(
            pilot.roll * self.limits.max_roll_rad,
            pilot.pitch * self.limits.max_pitch_rad,
            pilot.yaw * self.limits.max_yaw_rate_rad_s,
            measured_attitude,
            measured_rates_rad_s,
            dt,
        );
        let axes = ControlAxes::new(
            stabilized.actuator.x,
            stabilized.actuator.y,
            stabilized.actuator.z,
            pilot.throttle,
            0.0,
        );
        let surfaces = self.mixer.mix(axes);
        let pulses = self.servo_map.to_pulses(surfaces, &self.servos);
        VTailControlOutput {
            axes,
            surfaces,
            pulses,
        }
    }

    pub fn update_selected(
        &mut self,
        pilot: PilotCommand,
        measured_attitude: EulerAngles,
        measured_rates_rad_s: Vec3,
        dt: MicrosDurationU32,
    ) -> VTailControlOutput<N>
    where
        C: FixedWingAttitudeBackend,
    {
        if pilot.attitude_hold_enabled {
            self.update_attitude_hold(pilot, measured_attitude, measured_rates_rad_s, dt)
        } else {
            self.update_manual(pilot)
        }
    }
}

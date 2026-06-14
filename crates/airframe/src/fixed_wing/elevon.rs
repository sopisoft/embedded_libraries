use control::{ControlAxes, ElevonMixer, ElevonOutputs};
use fugit::MicrosDurationU32;
use pwm::{ServoBank, ServoSet};

use crate::{Attitude, PilotCommand, Vector3};

use super::{
    AttitudeHoldLimits, DefaultAttitudeController, FixedWingAttitudeBackend,
    common::{ServoAssignment, apply_assignment, neutral_pulses},
};

/// Servo map for a two-elevon wing plus throttle.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ElevonServoMap {
    pub left_elevon: ServoAssignment,
    pub right_elevon: ServoAssignment,
    pub throttle: ServoAssignment,
}

impl ElevonServoMap {
    pub const fn three_channel() -> Self {
        Self {
            left_elevon: ServoAssignment::symmetric(0),
            right_elevon: ServoAssignment::symmetric(1),
            throttle: ServoAssignment::normalized(2),
        }
    }

    pub fn to_pulses<const N: usize>(
        &self,
        surfaces: ElevonOutputs,
        servos: &ServoSet<N>,
    ) -> [MicrosDurationU32; N] {
        let mut pulses = neutral_pulses(servos);
        apply_assignment(&mut pulses, servos, self.left_elevon, surfaces.left_elevon);
        apply_assignment(
            &mut pulses,
            servos,
            self.right_elevon,
            surfaces.right_elevon,
        );
        apply_assignment(&mut pulses, servos, self.throttle, surfaces.throttle);
        pulses
    }
}

/// Output block for an elevon-controlled airframe.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ElevonControlOutput<const N: usize> {
    pub axes: ControlAxes,
    pub surfaces: ElevonOutputs,
    pub pulses: [MicrosDurationU32; N],
}

impl<const N: usize> ElevonControlOutput<N> {
    pub fn apply_to_servo_bank<E>(&self, bank: &mut ServoBank<'_, E, N>) -> Result<(), E> {
        bank.set_pulse_widths(self.pulses)
    }
}

/// High-level controller for elevon aircraft.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ElevonController<const N: usize, C = DefaultAttitudeController> {
    pub attitude_hold: C,
    pub mixer: ElevonMixer,
    pub servos: ServoSet<N>,
    pub servo_map: ElevonServoMap,
    pub limits: AttitudeHoldLimits,
}

impl<const N: usize, C> ElevonController<N, C> {
    pub const fn new(
        attitude_hold: C,
        mixer: ElevonMixer,
        servos: ServoSet<N>,
        servo_map: ElevonServoMap,
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

    pub fn update_manual(&self, pilot: PilotCommand) -> ElevonControlOutput<N> {
        let axes = ControlAxes::new(pilot.roll, pilot.pitch, 0.0, pilot.throttle, 0.0);
        let surfaces = self.mixer.mix(axes);
        let pulses = self.servo_map.to_pulses(surfaces, &self.servos);
        ElevonControlOutput {
            axes,
            surfaces,
            pulses,
        }
    }

    pub fn update_attitude_hold(
        &mut self,
        pilot: PilotCommand,
        measured_attitude: Attitude,
        measured_rates_rad_s: Vector3,
        dt: MicrosDurationU32,
    ) -> ElevonControlOutput<N>
    where
        C: FixedWingAttitudeBackend,
    {
        let stabilized = self.attitude_hold.update_fixed_wing(
            pilot.roll * self.limits.max_roll_rad,
            pilot.pitch * self.limits.max_pitch_rad,
            0.0,
            measured_attitude,
            measured_rates_rad_s,
            dt,
        );
        let axes = ControlAxes::new(
            stabilized.actuator.x,
            stabilized.actuator.y,
            0.0,
            pilot.throttle,
            0.0,
        );
        let surfaces = self.mixer.mix(axes);
        let pulses = self.servo_map.to_pulses(surfaces, &self.servos);
        ElevonControlOutput {
            axes,
            surfaces,
            pulses,
        }
    }

    pub fn update_selected(
        &mut self,
        pilot: PilotCommand,
        measured_attitude: Attitude,
        measured_rates_rad_s: Vector3,
        dt: MicrosDurationU32,
    ) -> ElevonControlOutput<N>
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

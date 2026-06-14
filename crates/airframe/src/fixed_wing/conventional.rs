use control::{ControlAxes, ConventionalTailMixer, ConventionalTailOutputs};
use fugit::MicrosDurationU32;
use pwm::{ServoBank, ServoSet};

use crate::{Attitude, PilotCommand, Vector3};

use super::{
    AttitudeHoldLimits, DefaultAttitudeController, FixedWingAttitudeBackend,
    common::{ServoAssignment, apply_assignment, neutral_pulses},
};

/// Conventional fixed-wing actuator layout.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ServoMap {
    pub left_aileron: ServoAssignment,
    pub right_aileron: ServoAssignment,
    pub elevator: ServoAssignment,
    pub rudder: ServoAssignment,
    pub throttle: ServoAssignment,
    pub left_flap: Option<ServoAssignment>,
    pub right_flap: Option<ServoAssignment>,
}

impl ServoMap {
    pub const fn conventional_5ch() -> Self {
        Self {
            left_aileron: ServoAssignment::symmetric(0),
            right_aileron: ServoAssignment::symmetric(1),
            elevator: ServoAssignment::symmetric(2),
            rudder: ServoAssignment::symmetric(3),
            throttle: ServoAssignment::normalized(4),
            left_flap: None,
            right_flap: None,
        }
    }

    pub const fn conventional_7ch() -> Self {
        Self {
            left_aileron: ServoAssignment::symmetric(0),
            right_aileron: ServoAssignment::symmetric(1),
            elevator: ServoAssignment::symmetric(2),
            rudder: ServoAssignment::symmetric(3),
            throttle: ServoAssignment::normalized(4),
            left_flap: Some(ServoAssignment::normalized(5)),
            right_flap: Some(ServoAssignment::normalized(6)),
        }
    }

    pub fn to_pulses<const N: usize>(
        &self,
        surfaces: ConventionalTailOutputs,
        servos: &ServoSet<N>,
    ) -> [MicrosDurationU32; N] {
        let mut pulses = neutral_pulses(servos);
        apply_assignment(
            &mut pulses,
            servos,
            self.left_aileron,
            surfaces.left_aileron,
        );
        apply_assignment(
            &mut pulses,
            servos,
            self.right_aileron,
            surfaces.right_aileron,
        );
        apply_assignment(&mut pulses, servos, self.elevator, surfaces.elevator);
        apply_assignment(&mut pulses, servos, self.rudder, surfaces.rudder);
        apply_assignment(&mut pulses, servos, self.throttle, surfaces.throttle);
        if let Some(assignment) = self.left_flap {
            apply_assignment(&mut pulses, servos, assignment, surfaces.left_flap);
        }
        if let Some(assignment) = self.right_flap {
            apply_assignment(&mut pulses, servos, assignment, surfaces.right_flap);
        }
        pulses
    }
}

/// Full output of one control update.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FixedWingControlOutput<const N: usize> {
    pub axes: ControlAxes,
    pub surfaces: ConventionalTailOutputs,
    pub pulses: [MicrosDurationU32; N],
}

impl<const N: usize> FixedWingControlOutput<N> {
    pub fn apply_to_servo_bank<E>(&self, bank: &mut ServoBank<'_, E, N>) -> Result<(), E> {
        bank.set_pulse_widths(self.pulses)
    }
}

/// Sensor-agnostic fixed-wing controller.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FixedWingController<const N: usize, C = DefaultAttitudeController> {
    pub attitude_hold: C,
    pub mixer: ConventionalTailMixer,
    pub servos: ServoSet<N>,
    pub servo_map: ServoMap,
    pub limits: AttitudeHoldLimits,
}

impl<const N: usize, C> FixedWingController<N, C> {
    pub const fn new(
        attitude_hold: C,
        mixer: ConventionalTailMixer,
        servos: ServoSet<N>,
        servo_map: ServoMap,
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

    pub fn update_manual(&self, pilot: PilotCommand) -> FixedWingControlOutput<N> {
        let axes = ControlAxes::new(
            pilot.roll,
            pilot.pitch,
            pilot.yaw,
            pilot.throttle,
            pilot.flaps,
        );
        let surfaces = self.mixer.mix(axes);
        let pulses = self.servo_map.to_pulses(surfaces, &self.servos);
        FixedWingControlOutput {
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
    ) -> FixedWingControlOutput<N>
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
            pilot.flaps,
        );
        let surfaces = self.mixer.mix(axes);
        let pulses = self.servo_map.to_pulses(surfaces, &self.servos);
        FixedWingControlOutput {
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
    ) -> FixedWingControlOutput<N>
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

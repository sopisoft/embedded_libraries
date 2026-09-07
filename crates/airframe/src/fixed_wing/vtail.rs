use control::{ControlAxes, Normalized, VTailMixer, VTailOutputs};
use fugit::MicrosDurationU32;
use pwm::ServoSet;

use crate::{PilotCommand, Vec3};

use super::{
    AttitudeHoldLimits, DefaultAttitudeController, FixedWingAttitudeBackend,
    backend::{attitude_hold_axes, manual_axes},
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

    const fn is_valid<const N: usize>(&self) -> bool {
        self.left_tail.index < N
            && self.right_tail.index < N
            && self.aileron.index < N
            && self.throttle.index < N
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
        assert!(servo_map.is_valid::<N>());
        Self {
            attitude_hold,
            mixer,
            servos,
            servo_map,
            limits,
        }
    }

    pub fn update_manual(&self, pilot: PilotCommand) -> VTailControlOutput<N> {
        let mut axes = manual_axes(pilot);
        axes.flaps = Normalized::ZERO;
        self.output(axes)
    }

    fn output(&self, axes: ControlAxes) -> VTailControlOutput<N> {
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
        measured_attitude: Vec3,
        measured_rates_rad_s: Vec3,
        dt: MicrosDurationU32,
    ) -> VTailControlOutput<N>
    where
        C: FixedWingAttitudeBackend,
    {
        let mut pilot = pilot;
        pilot.flaps = Normalized::ZERO;
        let axes = attitude_hold_axes(
            &mut self.attitude_hold,
            self.limits,
            pilot,
            measured_attitude,
            measured_rates_rad_s,
            dt,
        );
        self.output(axes)
    }

    pub fn update_selected(
        &mut self,
        pilot: PilotCommand,
        measured_attitude: Vec3,
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

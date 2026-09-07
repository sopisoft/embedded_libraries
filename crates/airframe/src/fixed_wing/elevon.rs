use control::{ControlAxes, ElevonMixer, ElevonOutputs, Normalized, SignedNormalized};
use fugit::MicrosDurationU32;
use pwm::ServoSet;

use crate::{PilotCommand, Vec3};

use super::{
    AttitudeHoldLimits, DefaultAttitudeController, FixedWingAttitudeBackend,
    backend::{attitude_hold_axes, manual_axes},
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

    const fn is_valid<const N: usize>(&self) -> bool {
        self.left_elevon.index < N && self.right_elevon.index < N && self.throttle.index < N
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
        assert!(servo_map.is_valid::<N>());
        Self {
            attitude_hold,
            mixer,
            servos,
            servo_map,
            limits,
        }
    }

    pub fn update_manual(&self, pilot: PilotCommand) -> ElevonControlOutput<N> {
        let mut axes = manual_axes(pilot);
        axes.yaw = SignedNormalized::ZERO;
        axes.flaps = Normalized::ZERO;
        self.output(axes)
    }

    fn output(&self, axes: ControlAxes) -> ElevonControlOutput<N> {
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
        measured_attitude: Vec3,
        measured_rates_rad_s: Vec3,
        dt: MicrosDurationU32,
    ) -> ElevonControlOutput<N>
    where
        C: FixedWingAttitudeBackend,
    {
        let mut pilot = pilot;
        pilot.yaw = SignedNormalized::ZERO;
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

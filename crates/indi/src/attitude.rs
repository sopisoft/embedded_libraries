use fugit::MicrosDurationU32;
use glam::Vec3;

use crate::{IndiOutputs, IndiRateController, IndiRateInput};

fn wrap_pi(angle_rad: f32) -> f32 {
    let mut wrapped = angle_rad;
    while wrapped > core::f32::consts::PI {
        wrapped -= core::f32::consts::TAU;
    }
    while wrapped < -core::f32::consts::PI {
        wrapped += core::f32::consts::TAU;
    }
    wrapped
}

/// Outer-loop configuration for attitude-hold mode.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct IndiAttitudeConfig {
    pub attitude_gain_rad_s_per_rad: Vec3,
    pub rate_limits_rad_s: Vec3,
}

impl IndiAttitudeConfig {
    /// Creates a practical fixed-wing starting point.
    pub const fn fixed_wing_default() -> Self {
        Self {
            attitude_gain_rad_s_per_rad: Vec3::new(4.0, 4.0, 2.0),
            rate_limits_rad_s: Vec3::new(2.5, 2.0, 1.5),
        }
    }
}

impl Default for IndiAttitudeConfig {
    fn default() -> Self {
        Self::fixed_wing_default()
    }
}

/// Input for fixed-wing INDI attitude hold.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct IndiFixedWingInput {
    pub target_roll_rad: f32,
    pub target_pitch_rad: f32,
    pub target_yaw_rate_rad_s: f32,
    /// Euler attitude vector in roll, pitch, yaw order.
    pub measured_attitude_rad: Vec3,
    pub measured_rates_rad_s: Vec3,
    pub measured_angular_accel_rad_s2: Option<Vec3>,
}

/// Output of an INDI attitude-hold update.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct IndiAttitudeOutput {
    pub actuator: Vec3,
    pub desired_rates_rad_s: Vec3,
    pub rate: IndiOutputs,
}

/// Attitude-hold wrapper around an INDI body-rate controller.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct IndiAttitudeController {
    pub rate: IndiRateController,
    pub config: IndiAttitudeConfig,
}

impl IndiAttitudeController {
    /// Creates a new INDI attitude controller.
    pub const fn new(rate: IndiRateController, config: IndiAttitudeConfig) -> Self {
        Self { rate, config }
    }

    /// Resets all inner controller state.
    pub fn reset(&mut self) {
        self.rate.reset();
    }

    /// Runs roll, pitch, and yaw attitude hold.
    pub fn update_attitude(
        &mut self,
        target_attitude_rad: Vec3,
        measured_attitude_rad: Vec3,
        measured_rates_rad_s: Vec3,
        dt: MicrosDurationU32,
    ) -> IndiAttitudeOutput {
        let desired_rates_rad_s = Vec3::new(
            self.attitude_rate_target(target_attitude_rad.x, measured_attitude_rad.x, 0),
            self.attitude_rate_target(target_attitude_rad.y, measured_attitude_rad.y, 1),
            self.attitude_rate_target(target_attitude_rad.z, measured_attitude_rad.z, 2),
        );
        self.update_rates(desired_rates_rad_s, measured_rates_rad_s, dt)
    }

    /// Fixed-wing update: roll and pitch are attitude-controlled, yaw is rate-controlled.
    pub fn update_fixed_wing(
        &mut self,
        target_roll_rad: f32,
        target_pitch_rad: f32,
        target_yaw_rate_rad_s: f32,
        measured_attitude_rad: Vec3,
        measured_rates_rad_s: Vec3,
        dt: MicrosDurationU32,
    ) -> IndiAttitudeOutput {
        self.update_fixed_wing_input(
            IndiFixedWingInput {
                target_roll_rad,
                target_pitch_rad,
                target_yaw_rate_rad_s,
                measured_attitude_rad,
                measured_rates_rad_s,
                measured_angular_accel_rad_s2: None,
            },
            dt,
        )
    }

    /// Fixed-wing update using a compact input struct.
    pub fn update_fixed_wing_input(
        &mut self,
        input: IndiFixedWingInput,
        dt: MicrosDurationU32,
    ) -> IndiAttitudeOutput {
        let desired_rates_rad_s = Vec3::new(
            self.attitude_rate_target(input.target_roll_rad, input.measured_attitude_rad.x, 0),
            self.attitude_rate_target(input.target_pitch_rad, input.measured_attitude_rad.y, 1),
            input.target_yaw_rate_rad_s.clamp(
                -self.config.rate_limits_rad_s.z,
                self.config.rate_limits_rad_s.z,
            ),
        );
        let mut rate_input = IndiRateInput::rates(desired_rates_rad_s, input.measured_rates_rad_s);
        if let Some(measured_angular_accel_rad_s2) = input.measured_angular_accel_rad_s2 {
            rate_input = rate_input.with_measured_acceleration(measured_angular_accel_rad_s2);
        }
        let rate = self.rate.update(rate_input, dt);
        IndiAttitudeOutput {
            actuator: rate.actuator,
            desired_rates_rad_s,
            rate,
        }
    }

    fn update_rates(
        &mut self,
        desired_rates_rad_s: Vec3,
        measured_rates_rad_s: Vec3,
        dt: MicrosDurationU32,
    ) -> IndiAttitudeOutput {
        let rate = self
            .rate
            .update_rates(desired_rates_rad_s, measured_rates_rad_s, dt);
        IndiAttitudeOutput {
            actuator: rate.actuator,
            desired_rates_rad_s,
            rate,
        }
    }

    fn attitude_rate_target(
        &self,
        target_angle_rad: f32,
        measured_angle_rad: f32,
        axis: usize,
    ) -> f32 {
        let error = wrap_pi(target_angle_rad - measured_angle_rad);
        let (gain, limit) = match axis {
            0 => (
                self.config.attitude_gain_rad_s_per_rad.x,
                self.config.rate_limits_rad_s.x,
            ),
            1 => (
                self.config.attitude_gain_rad_s_per_rad.y,
                self.config.rate_limits_rad_s.y,
            ),
            _ => (
                self.config.attitude_gain_rad_s_per_rad.z,
                self.config.rate_limits_rad_s.z,
            ),
        };
        (error * gain).clamp(-limit, limit)
    }
}

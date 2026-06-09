use fugit::MicrosDurationU32;

use crate::LowPassFilter;

const EFFECTIVENESS_EPSILON: f32 = 1.0e-6;

/// Configuration for one INDI-controlled axis.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct IndiAxisConfig {
    pub control_effectiveness: f32,
    pub rate_gain: f32,
    pub acceleration_feedforward_gain: f32,
    pub acceleration_limit_rad_s2: f32,
    pub actuator_min: f32,
    pub actuator_trim: f32,
    pub actuator_max: f32,
    pub actuator_slew_rate_per_s: f32,
    pub acceleration_filter_tau_s: f32,
    pub actuator_filter_tau_s: f32,
}

impl IndiAxisConfig {
    /// Creates a symmetric normalized actuator configuration.
    pub const fn symmetric(
        control_effectiveness: f32,
        rate_gain: f32,
        acceleration_limit_rad_s2: f32,
        actuator_slew_rate_per_s: f32,
    ) -> Self {
        Self {
            control_effectiveness,
            rate_gain,
            acceleration_feedforward_gain: 1.0,
            acceleration_limit_rad_s2,
            actuator_min: -1.0,
            actuator_trim: 0.0,
            actuator_max: 1.0,
            actuator_slew_rate_per_s,
            acceleration_filter_tau_s: 0.02,
            actuator_filter_tau_s: 0.0,
        }
    }
}

/// Input for one INDI axis update.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct IndiAxisInput {
    pub target_rate_rad_s: f32,
    pub target_angular_accel_rad_s2: f32,
    pub measured_rate_rad_s: f32,
    pub measured_angular_accel_rad_s2: Option<f32>,
    pub actuator_feedback: Option<f32>,
}

impl IndiAxisInput {
    /// Builds a rate-only input. Angular acceleration is estimated internally.
    pub const fn rate(target_rate_rad_s: f32, measured_rate_rad_s: f32) -> Self {
        Self {
            target_rate_rad_s,
            target_angular_accel_rad_s2: 0.0,
            measured_rate_rad_s,
            measured_angular_accel_rad_s2: None,
            actuator_feedback: None,
        }
    }

    /// Supplies externally measured angular acceleration.
    pub const fn with_measured_acceleration(mut self, angular_accel_rad_s2: f32) -> Self {
        self.measured_angular_accel_rad_s2 = Some(angular_accel_rad_s2);
        self
    }

    /// Supplies actuator-position feedback or a delayed actuator estimate.
    pub const fn with_actuator_feedback(mut self, actuator_feedback: f32) -> Self {
        self.actuator_feedback = Some(actuator_feedback);
        self
    }
}

/// Output of one axis update.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct IndiAxisOutput {
    pub actuator: f32,
    pub actuator_delta: f32,
    pub actuator_reference: f32,
    pub rate_error_rad_s: f32,
    pub desired_angular_accel_rad_s2: f32,
    pub measured_angular_accel_rad_s2: f32,
    pub acceleration_error_rad_s2: f32,
}

/// One-axis INDI rate controller.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct IndiAxis {
    config: IndiAxisConfig,
    actuator: f32,
    previous_rate_rad_s: Option<f32>,
    acceleration_filter: LowPassFilter,
    actuator_filter: LowPassFilter,
}

impl IndiAxis {
    /// Creates a new controller with its actuator initialized at trim.
    pub const fn new(config: IndiAxisConfig) -> Self {
        Self {
            actuator: config.actuator_trim,
            previous_rate_rad_s: None,
            acceleration_filter: LowPassFilter::new(config.acceleration_filter_tau_s),
            actuator_filter: LowPassFilter::new(config.actuator_filter_tau_s),
            config,
        }
    }

    /// Returns the static configuration.
    pub const fn config(&self) -> IndiAxisConfig {
        self.config
    }

    /// Returns the current actuator command.
    pub const fn actuator(&self) -> f32 {
        self.actuator
    }

    /// Replaces the configuration and keeps the current actuator state.
    pub fn set_config(&mut self, config: IndiAxisConfig) {
        self.config = config;
        self.acceleration_filter
            .set_tau_s(config.acceleration_filter_tau_s);
        self.actuator_filter.set_tau_s(config.actuator_filter_tau_s);
        self.set_actuator(self.actuator);
    }

    /// Resets filter state and returns the actuator to trim.
    pub fn reset(&mut self) {
        self.actuator = self.config.actuator_trim;
        self.previous_rate_rad_s = None;
        self.acceleration_filter.reset();
        self.actuator_filter.reset();
    }

    /// Forces the actuator state to a specific value.
    pub fn set_actuator(&mut self, actuator: f32) {
        self.actuator = actuator.clamp(self.config.actuator_min, self.config.actuator_max);
    }

    /// Estimates angular acceleration from successive gyro-rate samples.
    pub fn estimate_angular_acceleration(
        &mut self,
        measured_rate_rad_s: f32,
        dt: MicrosDurationU32,
    ) -> f32 {
        let dt_s = dt.as_secs_f32();
        if dt_s <= 0.0 {
            return 0.0;
        }

        let accel = if let Some(previous_rate) = self.previous_rate_rad_s {
            (measured_rate_rad_s - previous_rate) / dt_s
        } else {
            0.0
        };
        self.previous_rate_rad_s = Some(measured_rate_rad_s);
        accel
    }

    /// Runs one INDI update.
    pub fn update(&mut self, input: IndiAxisInput, dt: MicrosDurationU32) -> IndiAxisOutput {
        let dt_s = dt.as_secs_f32();
        if dt_s <= 0.0 || self.config.control_effectiveness.abs() < EFFECTIVENESS_EPSILON {
            return IndiAxisOutput {
                actuator: self.actuator,
                actuator_reference: self.actuator,
                ..IndiAxisOutput::default()
            };
        }

        let raw_measured_accel = input
            .measured_angular_accel_rad_s2
            .unwrap_or_else(|| self.estimate_angular_acceleration(input.measured_rate_rad_s, dt));
        let measured_angular_accel_rad_s2 = self.acceleration_filter.update(raw_measured_accel, dt);

        let actuator_reference = input.actuator_feedback.unwrap_or(self.actuator);
        let actuator_reference = self.actuator_filter.update(actuator_reference, dt);

        let rate_error_rad_s = input.target_rate_rad_s - input.measured_rate_rad_s;
        let desired_angular_accel_rad_s2 = (self.config.rate_gain * rate_error_rad_s
            + self.config.acceleration_feedforward_gain * input.target_angular_accel_rad_s2)
            .clamp(
                -self.config.acceleration_limit_rad_s2,
                self.config.acceleration_limit_rad_s2,
            );
        let acceleration_error_rad_s2 =
            desired_angular_accel_rad_s2 - measured_angular_accel_rad_s2;

        let unconstrained =
            actuator_reference + acceleration_error_rad_s2 / self.config.control_effectiveness;
        let requested_delta = unconstrained - self.actuator;
        let max_delta = self.config.actuator_slew_rate_per_s.abs() * dt_s;
        let limited_delta = if max_delta > 0.0 {
            requested_delta.clamp(-max_delta, max_delta)
        } else {
            requested_delta
        };

        let previous_actuator = self.actuator;
        self.actuator = (self.actuator + limited_delta)
            .clamp(self.config.actuator_min, self.config.actuator_max);

        IndiAxisOutput {
            actuator: self.actuator,
            actuator_delta: self.actuator - previous_actuator,
            actuator_reference,
            rate_error_rad_s,
            desired_angular_accel_rad_s2,
            measured_angular_accel_rad_s2,
            acceleration_error_rad_s2,
        }
    }

    /// Runs a direct angular-acceleration INDI step.
    pub fn update_acceleration(
        &mut self,
        desired_angular_accel_rad_s2: f32,
        measured_angular_accel_rad_s2: f32,
        dt: MicrosDurationU32,
    ) -> IndiAxisOutput {
        self.update(
            IndiAxisInput {
                target_angular_accel_rad_s2: desired_angular_accel_rad_s2,
                measured_angular_accel_rad_s2: Some(measured_angular_accel_rad_s2),
                ..IndiAxisInput::default()
            },
            dt,
        )
    }

    /// Runs a rate-loop INDI step with a directly measured angular acceleration.
    pub fn update_rate_with_acceleration(
        &mut self,
        target_rate_rad_s: f32,
        measured_rate_rad_s: f32,
        measured_angular_accel_rad_s2: f32,
        dt: MicrosDurationU32,
    ) -> IndiAxisOutput {
        self.update(
            IndiAxisInput::rate(target_rate_rad_s, measured_rate_rad_s)
                .with_measured_acceleration(measured_angular_accel_rad_s2),
            dt,
        )
    }

    /// Runs a rate-loop INDI step using internally estimated angular acceleration.
    pub fn update_rate(
        &mut self,
        target_rate_rad_s: f32,
        measured_rate_rad_s: f32,
        dt: MicrosDurationU32,
    ) -> IndiAxisOutput {
        self.update(
            IndiAxisInput::rate(target_rate_rad_s, measured_rate_rad_s),
            dt,
        )
    }
}

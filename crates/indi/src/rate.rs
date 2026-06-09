use fugit::MicrosDurationU32;
use math::Vec3;

use crate::{IndiAxis, IndiAxisConfig, IndiAxisInput, IndiAxisOutput};

/// Input for a three-axis INDI rate update.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct IndiRateInput {
    pub target_rates_rad_s: Vec3,
    pub target_angular_accel_rad_s2: Vec3,
    pub measured_rates_rad_s: Vec3,
    pub measured_angular_accel_rad_s2: Option<Vec3>,
    pub actuator_feedback: Option<Vec3>,
}

impl IndiRateInput {
    /// Builds a rate-only input.
    pub const fn rates(target_rates_rad_s: Vec3, measured_rates_rad_s: Vec3) -> Self {
        Self {
            target_rates_rad_s,
            target_angular_accel_rad_s2: Vec3::zero(),
            measured_rates_rad_s,
            measured_angular_accel_rad_s2: None,
            actuator_feedback: None,
        }
    }

    /// Supplies externally measured angular acceleration.
    pub const fn with_measured_acceleration(mut self, angular_accel_rad_s2: Vec3) -> Self {
        self.measured_angular_accel_rad_s2 = Some(angular_accel_rad_s2);
        self
    }

    /// Supplies actuator feedback.
    pub const fn with_actuator_feedback(mut self, actuator_feedback: Vec3) -> Self {
        self.actuator_feedback = Some(actuator_feedback);
        self
    }
}

/// Three-axis INDI output bundle.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct IndiOutputs {
    pub actuator: Vec3,
    pub actuator_delta: Vec3,
    pub actuator_reference: Vec3,
    pub rate_error_rad_s: Vec3,
    pub desired_angular_accel_rad_s2: Vec3,
    pub measured_angular_accel_rad_s2: Vec3,
    pub acceleration_error_rad_s2: Vec3,
}

/// Three-axis INDI rate controller.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct IndiRateController {
    pub roll: IndiAxis,
    pub pitch: IndiAxis,
    pub yaw: IndiAxis,
}

impl IndiRateController {
    /// Creates a new controller for roll, pitch, and yaw.
    pub const fn new(roll: IndiAxis, pitch: IndiAxis, yaw: IndiAxis) -> Self {
        Self { roll, pitch, yaw }
    }

    /// Creates a controller from three axis configurations.
    pub const fn from_configs(
        roll: IndiAxisConfig,
        pitch: IndiAxisConfig,
        yaw: IndiAxisConfig,
    ) -> Self {
        Self::new(
            IndiAxis::new(roll),
            IndiAxis::new(pitch),
            IndiAxis::new(yaw),
        )
    }

    /// Resets all axis states.
    pub fn reset(&mut self) {
        self.roll.reset();
        self.pitch.reset();
        self.yaw.reset();
    }

    /// Returns the current actuator state of all axes.
    pub fn actuator(&self) -> Vec3 {
        Vec3::new(
            self.roll.actuator(),
            self.pitch.actuator(),
            self.yaw.actuator(),
        )
    }

    /// Forces the actuator states of all axes.
    pub fn set_actuator(&mut self, actuator: Vec3) {
        self.roll.set_actuator(actuator.x);
        self.pitch.set_actuator(actuator.y);
        self.yaw.set_actuator(actuator.z);
    }

    /// Runs one three-axis INDI update.
    pub fn update(&mut self, input: IndiRateInput, dt: MicrosDurationU32) -> IndiOutputs {
        let accel = input.measured_angular_accel_rad_s2;
        let actuator_feedback = input.actuator_feedback;

        let roll = self
            .roll
            .update(axis_input(input, Axis::Roll, accel, actuator_feedback), dt);
        let pitch = self
            .pitch
            .update(axis_input(input, Axis::Pitch, accel, actuator_feedback), dt);
        let yaw = self
            .yaw
            .update(axis_input(input, Axis::Yaw, accel, actuator_feedback), dt);

        combine_outputs(roll, pitch, yaw)
    }

    /// Rate-loop update with internally estimated angular accelerations.
    pub fn update_rates(
        &mut self,
        target_rates_rad_s: Vec3,
        measured_rates_rad_s: Vec3,
        dt: MicrosDurationU32,
    ) -> IndiOutputs {
        self.update(
            IndiRateInput::rates(target_rates_rad_s, measured_rates_rad_s),
            dt,
        )
    }

    /// Rate-loop update with externally measured angular accelerations.
    pub fn update_rates_with_acceleration(
        &mut self,
        target_rates_rad_s: Vec3,
        measured_rates_rad_s: Vec3,
        measured_angular_accel_rad_s2: Vec3,
        dt: MicrosDurationU32,
    ) -> IndiOutputs {
        self.update(
            IndiRateInput::rates(target_rates_rad_s, measured_rates_rad_s)
                .with_measured_acceleration(measured_angular_accel_rad_s2),
            dt,
        )
    }

    /// Direct angular-acceleration update.
    pub fn update_acceleration(
        &mut self,
        desired_angular_accel_rad_s2: Vec3,
        measured_angular_accel_rad_s2: Vec3,
        dt: MicrosDurationU32,
    ) -> IndiOutputs {
        self.update(
            IndiRateInput {
                target_angular_accel_rad_s2: desired_angular_accel_rad_s2,
                measured_angular_accel_rad_s2: Some(measured_angular_accel_rad_s2),
                ..IndiRateInput::default()
            },
            dt,
        )
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Axis {
    Roll,
    Pitch,
    Yaw,
}

fn axis_input(
    input: IndiRateInput,
    axis: Axis,
    accel: Option<Vec3>,
    actuator_feedback: Option<Vec3>,
) -> IndiAxisInput {
    let measured_accel = accel.map(|value| match axis {
        Axis::Roll => value.x,
        Axis::Pitch => value.y,
        Axis::Yaw => value.z,
    });
    let actuator_feedback = actuator_feedback.map(|value| match axis {
        Axis::Roll => value.x,
        Axis::Pitch => value.y,
        Axis::Yaw => value.z,
    });

    match axis {
        Axis::Roll => IndiAxisInput {
            target_rate_rad_s: input.target_rates_rad_s.x,
            target_angular_accel_rad_s2: input.target_angular_accel_rad_s2.x,
            measured_rate_rad_s: input.measured_rates_rad_s.x,
            measured_angular_accel_rad_s2: measured_accel,
            actuator_feedback,
        },
        Axis::Pitch => IndiAxisInput {
            target_rate_rad_s: input.target_rates_rad_s.y,
            target_angular_accel_rad_s2: input.target_angular_accel_rad_s2.y,
            measured_rate_rad_s: input.measured_rates_rad_s.y,
            measured_angular_accel_rad_s2: measured_accel,
            actuator_feedback,
        },
        Axis::Yaw => IndiAxisInput {
            target_rate_rad_s: input.target_rates_rad_s.z,
            target_angular_accel_rad_s2: input.target_angular_accel_rad_s2.z,
            measured_rate_rad_s: input.measured_rates_rad_s.z,
            measured_angular_accel_rad_s2: measured_accel,
            actuator_feedback,
        },
    }
}

fn combine_outputs(
    roll: IndiAxisOutput,
    pitch: IndiAxisOutput,
    yaw: IndiAxisOutput,
) -> IndiOutputs {
    IndiOutputs {
        actuator: Vec3::new(roll.actuator, pitch.actuator, yaw.actuator),
        actuator_delta: Vec3::new(
            roll.actuator_delta,
            pitch.actuator_delta,
            yaw.actuator_delta,
        ),
        actuator_reference: Vec3::new(
            roll.actuator_reference,
            pitch.actuator_reference,
            yaw.actuator_reference,
        ),
        rate_error_rad_s: Vec3::new(
            roll.rate_error_rad_s,
            pitch.rate_error_rad_s,
            yaw.rate_error_rad_s,
        ),
        desired_angular_accel_rad_s2: Vec3::new(
            roll.desired_angular_accel_rad_s2,
            pitch.desired_angular_accel_rad_s2,
            yaw.desired_angular_accel_rad_s2,
        ),
        measured_angular_accel_rad_s2: Vec3::new(
            roll.measured_angular_accel_rad_s2,
            pitch.measured_angular_accel_rad_s2,
            yaw.measured_angular_accel_rad_s2,
        ),
        acceleration_error_rad_s2: Vec3::new(
            roll.acceleration_error_rad_s2,
            pitch.acceleration_error_rad_s2,
            yaw.acceleration_error_rad_s2,
        ),
    }
}

//! Time-based PID controller.

use fugit::MicrosDurationU32;

/// Basic PID controller with derivative-on-measurement.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PidController {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    previous_measurement: Option<f32>,
    output_min: f32,
    output_max: f32,
    integral_min: f32,
    integral_max: f32,
}

impl PidController {
    /// Creates a PID controller with unconstrained integral and output terms.
    pub const fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            previous_measurement: None,
            output_min: -f32::INFINITY,
            output_max: f32::INFINITY,
            integral_min: -f32::INFINITY,
            integral_max: f32::INFINITY,
        }
    }

    /// Sets the controller gains.
    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    /// Clamps the output range.
    pub fn set_output_limits(&mut self, min: f32, max: f32) {
        self.output_min = min.min(max);
        self.output_max = min.max(max);
    }

    /// Clamps the integral state.
    pub fn set_integral_limits(&mut self, min: f32, max: f32) {
        self.integral_min = min.min(max);
        self.integral_max = min.max(max);
        self.integral = self.integral.clamp(self.integral_min, self.integral_max);
    }

    /// Clears the integral and derivative history.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_measurement = None;
    }

    /// Returns the current integral state.
    pub const fn integral(&self) -> f32 {
        self.integral
    }

    /// Updates the controller using a setpoint, a measurement, and a time step.
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: MicrosDurationU32) -> f32 {
        let dt_s = dt.as_secs_f32();
        if dt_s <= 0.0 {
            return 0.0;
        }

        let error = setpoint - measurement;
        self.integral = (self.integral + error * dt_s).clamp(self.integral_min, self.integral_max);

        let derivative = if let Some(previous) = self.previous_measurement {
            -(measurement - previous) / dt_s
        } else {
            0.0
        };
        self.previous_measurement = Some(measurement);

        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        output.clamp(self.output_min, self.output_max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pid_builds_positive_output_for_positive_error() {
        let mut pid = PidController::new(2.0, 0.0, 0.0);
        let output = pid.update(1.0, 0.0, MicrosDurationU32::from_millis(20));
        assert!(output > 0.0);
    }

    #[test]
    fn integral_is_limited() {
        let mut pid = PidController::new(0.0, 1.0, 0.0);
        pid.set_integral_limits(-0.5, 0.5);
        for _ in 0..100 {
            let _ = pid.update(1.0, 0.0, MicrosDurationU32::from_millis(20));
        }
        assert!(pid.integral() <= 0.5);
    }

    #[test]
    fn output_is_clamped() {
        let mut pid = PidController::new(10.0, 0.0, 0.0);
        pid.set_output_limits(-0.2, 0.2);
        let output = pid.update(1.0, 0.0, MicrosDurationU32::from_millis(20));
        assert_eq!(output, 0.2);
    }
}

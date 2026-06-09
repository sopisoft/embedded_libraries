use embedded_hal::pwm::SetDutyCycle;
use fugit::MicrosDurationU32;
use libm::roundf;

/// Servo timing and travel limits.
///
/// This type intentionally stays simple:
/// - frame period,
/// - minimum and maximum pulse widths,
/// - minimum and maximum angles.
///
/// Trim, reverse, and endpoint tuning are expected to be handled by the radio,
/// mixer, or the mechanical linkage.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ServoRange {
    /// Time from one pulse to the next.
    pub frame_period: MicrosDurationU32,
    /// Minimum pulse width.
    pub min_pulse: MicrosDurationU32,
    /// Maximum pulse width.
    pub max_pulse: MicrosDurationU32,
    /// Minimum mechanical angle in degrees.
    pub min_angle_deg: f32,
    /// Maximum mechanical angle in degrees.
    pub max_angle_deg: f32,
}

impl ServoRange {
    /// Creates a range from explicit timing and angle limits.
    pub const fn new(
        frame_period: MicrosDurationU32,
        min_pulse: MicrosDurationU32,
        max_pulse: MicrosDurationU32,
        min_angle_deg: f32,
        max_angle_deg: f32,
    ) -> Self {
        Self {
            frame_period,
            min_pulse,
            max_pulse,
            min_angle_deg,
            max_angle_deg,
        }
    }

    /// Converts a normalized value in `[0, 1]` to a pulse width.
    pub fn pulse_for_normalized(&self, position: f32) -> MicrosDurationU32 {
        let position = position.clamp(0.0, 1.0);
        let min_us = self.min_pulse.as_micros() as f32;
        let max_us = self.max_pulse.as_micros() as f32;
        let pulse_us = min_us + (max_us - min_us) * position;
        MicrosDurationU32::from_micros(roundf(pulse_us) as u32)
    }

    /// Converts a symmetric value in `[-1, 1]` to a pulse width.
    pub fn pulse_for_symmetric(&self, command: f32) -> MicrosDurationU32 {
        let command = command.clamp(-1.0, 1.0);
        let center_deg = 0.5 * (self.min_angle_deg + self.max_angle_deg);
        let half_span_deg = 0.5 * (self.max_angle_deg - self.min_angle_deg);
        self.pulse_for_angle_degrees(center_deg + half_span_deg * command)
    }

    /// Converts an angle in degrees to a pulse width.
    pub fn pulse_for_angle_degrees(&self, angle_deg: f32) -> MicrosDurationU32 {
        let span = self.max_angle_deg - self.min_angle_deg;
        let normalized = if span.abs() <= f32::EPSILON {
            0.0
        } else {
            (angle_deg - self.min_angle_deg) / span
        };
        self.pulse_for_normalized(normalized)
    }

    /// Converts an angle in radians to a pulse width.
    pub fn pulse_for_angle_radians(&self, angle_rad: f32) -> MicrosDurationU32 {
        self.pulse_for_angle_degrees(angle_rad * 180.0 / core::f32::consts::PI)
    }

    /// Converts a pulse width into a duty-cycle count for the supplied PWM period.
    pub fn duty_for_pulse<PWM: SetDutyCycle>(&self, pwm: &PWM, pulse: MicrosDurationU32) -> u16 {
        let duty = pulse.as_secs_f32() / self.frame_period.as_secs_f32();
        let duty = duty.clamp(0.0, 1.0);
        let max = pwm.max_duty_cycle() as f32;
        roundf(duty * max) as u16
    }
}

impl Default for ServoRange {
    fn default() -> Self {
        Self::new(
            MicrosDurationU32::from_micros(20_000),
            MicrosDurationU32::from_micros(1_000),
            MicrosDurationU32::from_micros(2_000),
            -90.0,
            90.0,
        )
    }
}

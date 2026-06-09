use embedded_hal::pwm::SetDutyCycle;
use fugit::MicrosDurationU32;

use super::{ServoOutput, ServoRange};

/// Servo PWM wrapper.
#[derive(Debug)]
pub struct Servo<PWM> {
    pwm: PWM,
    range: ServoRange,
}

impl<PWM> Servo<PWM> {
    /// Creates a servo from a shared range object.
    pub const fn from_range(pwm: PWM, range: ServoRange) -> Self {
        Self { pwm, range }
    }

    /// Releases the inner PWM peripheral.
    pub fn release(self) -> PWM {
        self.pwm
    }
}

impl<PWM: SetDutyCycle> Servo<PWM> {
    /// Creates a servo from explicit timing and angle limits.
    pub fn new(
        pwm: PWM,
        frame_period: MicrosDurationU32,
        min_pulse: MicrosDurationU32,
        max_pulse: MicrosDurationU32,
        min_angle_deg: f32,
        max_angle_deg: f32,
    ) -> Self {
        Self {
            pwm,
            range: ServoRange::new(
                frame_period,
                min_pulse,
                max_pulse,
                min_angle_deg,
                max_angle_deg,
            ),
        }
    }

    /// Sets the servo position as a normalized value in `[0, 1]`.
    pub fn set_normalized(&mut self, position: f32) -> Result<(), PWM::Error> {
        self.set_pulse_width(self.range.pulse_for_normalized(position))
    }

    /// Sets the servo position from a symmetric command in `[-1, 1]`.
    pub fn set_symmetric(&mut self, command: f32) -> Result<(), PWM::Error> {
        self.set_pulse_width(self.range.pulse_for_symmetric(command))
    }

    /// Sets the servo angle in degrees.
    pub fn set_angle_degrees(&mut self, angle_deg: f32) -> Result<(), PWM::Error> {
        self.set_pulse_width(self.range.pulse_for_angle_degrees(angle_deg))
    }

    /// Sets the servo angle in radians.
    pub fn set_angle_radians(&mut self, angle_rad: f32) -> Result<(), PWM::Error> {
        self.set_pulse_width(self.range.pulse_for_angle_radians(angle_rad))
    }

    /// Writes a raw pulse width.
    pub fn set_pulse_width(&mut self, pulse: MicrosDurationU32) -> Result<(), PWM::Error> {
        let value = self.range.duty_for_pulse(&self.pwm, pulse);
        self.pwm.set_duty_cycle(value)
    }
}

impl<PWM: SetDutyCycle> ServoOutput for Servo<PWM> {
    type Error = PWM::Error;

    fn set_normalized(&mut self, position: f32) -> Result<(), Self::Error> {
        Servo::set_normalized(self, position)
    }

    fn set_symmetric(&mut self, command: f32) -> Result<(), Self::Error> {
        Servo::set_symmetric(self, command)
    }

    fn set_angle_degrees(&mut self, angle_deg: f32) -> Result<(), Self::Error> {
        Servo::set_angle_degrees(self, angle_deg)
    }

    fn set_angle_radians(&mut self, angle_rad: f32) -> Result<(), Self::Error> {
        Servo::set_angle_radians(self, angle_rad)
    }

    fn set_pulse_width(&mut self, pulse: MicrosDurationU32) -> Result<(), Self::Error> {
        Servo::set_pulse_width(self, pulse)
    }
}

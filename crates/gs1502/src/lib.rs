#![no_std]

#[cfg(test)]
extern crate std;

use embedded_hal::pwm::SetDutyCycle;
use fugit::MicrosDurationU32;
use pwm::{Servo, ServoOutput, ServoRange};

/// GS-1502 PWM frame period.
pub const FRAME_PERIOD_US: u32 = 20_000;
/// Practical lower endpoint observed for GS-1502 units.
pub const MIN_PULSE_US: u32 = 700;
/// Practical upper endpoint observed for GS-1502 units.
pub const MAX_PULSE_US: u32 = 2_300;
/// Approximate total actuator stroke.
pub const STROKE_MM: f32 = 7.0;

/// Default GS-1502 timing and travel range.
pub const DEFAULT_RANGE: ServoRange = ServoRange::new(
    MicrosDurationU32::from_micros(FRAME_PERIOD_US),
    MicrosDurationU32::from_micros(MIN_PULSE_US),
    MicrosDurationU32::from_micros(MAX_PULSE_US),
    -90.0,
    90.0,
);

/// PWM-backed GS-1502 linear servo.
#[derive(Debug)]
pub struct Gs1502<PWM> {
    servo: Servo<PWM>,
}

impl<PWM> Gs1502<PWM> {
    /// Creates a GS-1502 using the default PWM range.
    pub fn new(pwm: PWM) -> Self
    where
        PWM: SetDutyCycle,
    {
        Self::from_range(pwm, DEFAULT_RANGE)
    }

    /// Creates a GS-1502 using a calibrated PWM range.
    pub const fn from_range(pwm: PWM, range: ServoRange) -> Self {
        Self {
            servo: Servo::from_range(pwm, range),
        }
    }

    /// Releases the underlying PWM peripheral.
    pub fn release(self) -> PWM {
        self.servo.release()
    }
}

impl<PWM: SetDutyCycle> Gs1502<PWM> {
    /// Sets the actuator position as a normalized value in `[0, 1]`.
    pub fn set_position(&mut self, position: f32) -> Result<(), PWM::Error> {
        self.servo.set_normalized(position)
    }

    /// Sets the actuator position in millimeters from one end of the stroke.
    pub fn set_stroke_mm(&mut self, position_mm: f32) -> Result<(), PWM::Error> {
        self.set_position(position_mm / STROKE_MM)
    }

    /// Sets a centered command in `[-1, 1]`.
    pub fn set_symmetric(&mut self, command: f32) -> Result<(), PWM::Error> {
        self.servo.set_symmetric(command)
    }

    /// Writes a raw pulse width.
    pub fn set_pulse_width(&mut self, pulse: MicrosDurationU32) -> Result<(), PWM::Error> {
        self.servo.set_pulse_width(pulse)
    }
}

impl<PWM: SetDutyCycle> ServoOutput for Gs1502<PWM> {
    type Error = PWM::Error;

    fn set_normalized(&mut self, position: f32) -> Result<(), Self::Error> {
        self.set_position(position)
    }

    fn set_symmetric(&mut self, command: f32) -> Result<(), Self::Error> {
        self.set_symmetric(command)
    }

    fn set_angle_degrees(&mut self, angle_deg: f32) -> Result<(), Self::Error> {
        self.servo.set_angle_degrees(angle_deg)
    }

    fn set_angle_radians(&mut self, angle_rad: f32) -> Result<(), Self::Error> {
        self.servo.set_angle_radians(angle_rad)
    }

    fn set_pulse_width(&mut self, pulse: MicrosDurationU32) -> Result<(), Self::Error> {
        self.set_pulse_width(pulse)
    }
}

#[cfg(test)]
mod tests {
    use core::convert::Infallible;

    use embedded_hal::pwm::{ErrorType, SetDutyCycle};

    use super::Gs1502;

    #[derive(Debug)]
    struct MockPwm {
        duty: u16,
    }

    impl ErrorType for MockPwm {
        type Error = Infallible;
    }

    impl SetDutyCycle for MockPwm {
        fn max_duty_cycle(&self) -> u16 {
            20_000
        }

        fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
            self.duty = duty;
            Ok(())
        }
    }

    #[test]
    fn stroke_maps_to_default_endpoints() {
        let mut servo = Gs1502::new(MockPwm { duty: 0 });
        servo.set_stroke_mm(3.5).unwrap();
        assert_eq!(servo.release().duty, 1_500);
    }
}

//! ESC wrapper that maps normalized throttle to pulse widths.

use embedded_hal::pwm::SetDutyCycle;
use fugit::MicrosDurationU32;
use libm::roundf;

/// ESC PWM wrapper.
#[derive(Debug)]
pub struct Esc<PWM> {
    pwm: PWM,
    frame_period: MicrosDurationU32,
    min_pulse: MicrosDurationU32,
    max_pulse: MicrosDurationU32,
}

impl<PWM: SetDutyCycle> Esc<PWM> {
    /// Creates an ESC wrapper from explicit timing values.
    pub fn new(
        pwm: PWM,
        frame_period: MicrosDurationU32,
        min_pulse: MicrosDurationU32,
        max_pulse: MicrosDurationU32,
    ) -> Self {
        Self {
            pwm,
            frame_period,
            min_pulse,
            max_pulse,
        }
    }

    /// Releases the inner PWM peripheral.
    pub fn release(self) -> PWM {
        self.pwm
    }

    /// Sets the throttle in `[0, 1]`.
    pub fn set_throttle(&mut self, throttle: f32) -> Result<(), PWM::Error> {
        let throttle = throttle.clamp(0.0, 1.0);
        let min_us = self.min_pulse.as_micros() as f32;
        let max_us = self.max_pulse.as_micros() as f32;
        let pulse_us = min_us + (max_us - min_us) * throttle;
        self.set_pulse_width(MicrosDurationU32::from_micros(roundf(pulse_us) as u32))
    }

    /// Writes a raw pulse width.
    pub fn set_pulse_width(&mut self, pulse: MicrosDurationU32) -> Result<(), PWM::Error> {
        let duty = pulse.as_secs_f32() / self.frame_period.as_secs_f32();
        let duty = duty.clamp(0.0, 1.0);
        let max = self.pwm.max_duty_cycle() as f32;
        let value = roundf(duty * max) as u16;
        self.pwm.set_duty_cycle(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;
    use embedded_hal::pwm::{ErrorType, SetDutyCycle};

    #[derive(Debug)]
    struct MockPwm {
        duty: u16,
        max: u16,
    }

    impl ErrorType for MockPwm {
        type Error = Infallible;
    }

    impl SetDutyCycle for MockPwm {
        fn max_duty_cycle(&self) -> u16 {
            self.max
        }

        fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
            self.duty = duty;
            Ok(())
        }
    }

    #[test]
    fn throttle_maps_to_duty() {
        let pwm = MockPwm { duty: 0, max: 1000 };
        let mut esc = Esc::new(
            pwm,
            MicrosDurationU32::from_micros(20_000),
            MicrosDurationU32::from_micros(1_000),
            MicrosDurationU32::from_micros(2_000),
        );
        esc.set_throttle(0.5).unwrap();
        let pwm = esc.release();
        assert_eq!(pwm.duty, 75);
    }
}

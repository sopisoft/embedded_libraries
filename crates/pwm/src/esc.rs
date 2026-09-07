//! ESC wrapper that maps normalized throttle to pulse widths.

use control::Normalized;
use embedded_hal::pwm::SetDutyCycle;
use fugit::MicrosDurationU32;
use libm::roundf;

#[derive(Debug)]
pub struct Esc<PWM> {
    pwm: PWM,
    frame_period: MicrosDurationU32,
    min_pulse: MicrosDurationU32,
    max_pulse: MicrosDurationU32,
}

impl<PWM: SetDutyCycle> Esc<PWM> {
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

    pub fn release(self) -> PWM {
        self.pwm
    }

    pub fn set_throttle(&mut self, throttle: Normalized) -> Result<(), PWM::Error> {
        let min_us = self.min_pulse.as_micros() as f32;
        let max_us = self.max_pulse.as_micros() as f32;
        let pulse_us = min_us + (max_us - min_us) * throttle.get();
        self.set_pulse_width(MicrosDurationU32::from_micros(roundf(pulse_us) as u32))
    }

    fn set_pulse_width(&mut self, pulse: MicrosDurationU32) -> Result<(), PWM::Error> {
        let duty = (pulse.as_secs_f32() / self.frame_period.as_secs_f32()).clamp(0.0, 1.0);
        self.pwm
            .set_duty_cycle(roundf(duty * self.pwm.max_duty_cycle() as f32) as u16)
    }
}

#[cfg(test)]
mod tests {
    use core::convert::Infallible;

    use embedded_hal::pwm::{ErrorType, SetDutyCycle};

    use super::*;

    struct MockPwm(u16);

    impl ErrorType for MockPwm {
        type Error = Infallible;
    }

    impl SetDutyCycle for MockPwm {
        fn max_duty_cycle(&self) -> u16 {
            20_000
        }

        fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
            self.0 = duty;
            Ok(())
        }
    }

    #[test]
    fn throttle_maps_to_pulse_range() {
        let mut esc = Esc::new(
            MockPwm(0),
            MicrosDurationU32::from_micros(20_000),
            MicrosDurationU32::from_micros(1_000),
            MicrosDurationU32::from_micros(2_000),
        );

        esc.set_throttle(Normalized::saturated(0.5)).unwrap();

        assert_eq!(esc.release().0, 1_500);
    }
}

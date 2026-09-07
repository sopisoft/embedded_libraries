use core::convert::Infallible;

use control::Normalized;
use embedded_hal::pwm::{ErrorType, SetDutyCycle};
use fugit::MicrosDurationU32;
use pwm::Esc;

#[derive(Debug)]
struct MockPwmChannel {
    compare: u16,
    top: u16,
}

impl ErrorType for MockPwmChannel {
    type Error = Infallible;
}

impl SetDutyCycle for MockPwmChannel {
    fn max_duty_cycle(&self) -> u16 {
        self.top
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.compare = duty;
        Ok(())
    }
}

fn main() {
    let pwm_channel = MockPwmChannel {
        compare: 0,
        top: 20_000,
    };

    let mut esc = Esc::new(
        pwm_channel,
        MicrosDurationU32::from_micros(20_000),
        MicrosDurationU32::from_micros(1_000),
        MicrosDurationU32::from_micros(2_000),
    );

    esc.set_throttle(Normalized::ZERO).unwrap();
    esc.set_throttle(Normalized::saturated(0.35)).unwrap();
    esc.set_throttle(Normalized::saturated(0.70)).unwrap();

    let pwm_channel = esc.release();
    println!(
        "Final ESC compare value: {} / {}",
        pwm_channel.compare, pwm_channel.top
    );
}

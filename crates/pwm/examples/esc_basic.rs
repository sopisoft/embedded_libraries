use core::convert::Infallible;

use embedded_hal::pwm::{ErrorType, SetDutyCycle};
use fugit::MicrosDurationU32;
use pwm::Esc;

// Replace this mock type with the PWM channel from your board support crate.
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
    // Hobby ESCs commonly use the same electrical pulse format as a servo:
    // 20 ms frame period and roughly 1000..2000 us pulse width.
    // The difference is at the application layer:
    // - a servo is usually commanded by angle,
    // - an ESC is usually commanded by throttle.
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

    // A real system usually performs its own arming or safety checks before
    // accepting throttle commands. This helper only maps throttle to pulse width.
    esc.set_throttle(0.0).unwrap();
    esc.set_throttle(0.35).unwrap();
    esc.set_throttle(0.70).unwrap();

    let pwm_channel = esc.release();
    println!(
        "Final ESC compare value: {} / {}",
        pwm_channel.compare, pwm_channel.top
    );
}

use core::convert::Infallible;

use control::Normalized;
use embedded_hal::pwm::{ErrorType, SetDutyCycle};
use fugit::MicrosDurationU32;
use pwm::Servo;

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

    let mut servo = Servo::new(
        pwm_channel,
        MicrosDurationU32::from_micros(20_000),
        MicrosDurationU32::from_micros(1_000),
        MicrosDurationU32::from_micros(2_000),
        -90.0,
        90.0,
    );

    servo.set_angle_degrees(-45.0).unwrap();
    servo.set_angle_degrees(0.0).unwrap();
    servo.set_angle_degrees(60.0).unwrap();

    servo.set_normalized(Normalized::saturated(0.25)).unwrap();

    let pwm_channel = servo.release();
    println!(
        "Final compare value sent to the PWM channel: {} / {}",
        pwm_channel.compare, pwm_channel.top
    );
}

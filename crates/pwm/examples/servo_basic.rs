use core::convert::Infallible;

use embedded_hal::pwm::{ErrorType, SetDutyCycle};
use fugit::MicrosDurationU32;
use pwm::Servo;

// This example is intentionally host-runnable.
// Replace this mock type with the PWM channel type from your HAL.
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
    // A standard hobby servo usually expects:
    // - one PWM period every 20 ms,
    // - a pulse width near 1000 us for one end,
    // - a pulse width near 2000 us for the other end.
    //
    // This wrapper lets you think in angles instead of duty-cycle counts.
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

    // Command the servo by angle.
    servo.set_angle_degrees(-45.0).unwrap();
    servo.set_angle_degrees(0.0).unwrap();
    servo.set_angle_degrees(60.0).unwrap();

    // If your controller already produces normalized values, you can use [0, 1].
    servo.set_normalized(0.25).unwrap();

    let pwm_channel = servo.release();
    println!(
        "Final compare value sent to the PWM channel: {} / {}",
        pwm_channel.compare, pwm_channel.top
    );
}

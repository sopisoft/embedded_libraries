use core::convert::Infallible;

use embedded_hal::pwm::{ErrorType, SetDutyCycle};
use fugit::MicrosDurationU32;

use super::super::{Servo, ServoBank, ServoOutput, ServoRange, ServoSet};

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
fn center_angle_maps_to_center_pulse() {
    let pwm = MockPwm {
        duty: 0,
        max: 1_000,
    };
    let mut servo = Servo::new(
        pwm,
        MicrosDurationU32::from_micros(20_000),
        MicrosDurationU32::from_micros(1_000),
        MicrosDurationU32::from_micros(2_000),
        -90.0,
        90.0,
    );
    servo.set_angle_degrees(0.0).unwrap();
    let pwm = servo.release();
    assert_eq!(pwm.duty, 75);
}

#[test]
fn symmetric_command_maps_centered_surfaces() {
    let pwm = MockPwm {
        duty: 0,
        max: 1_000,
    };
    let mut servo = Servo::new(
        pwm,
        MicrosDurationU32::from_micros(20_000),
        MicrosDurationU32::from_micros(1_000),
        MicrosDurationU32::from_micros(2_000),
        -90.0,
        90.0,
    );
    servo.set_symmetric(-1.0).unwrap();
    let pwm = servo.release();
    assert_eq!(pwm.duty, 50);
}

#[test]
fn servo_set_computes_multiple_pulses() {
    let range = ServoRange::default();
    let set = ServoSet::new([range, range, range]);
    let pulses = set.pulse_widths_from_symmetric([0.0, 0.5, -0.5]);
    assert_eq!(pulses[0].as_micros(), 1_500);
    assert!(pulses[1].as_micros() > 1_500);
    assert!(pulses[2].as_micros() < 1_500);
}

#[test]
fn servo_bank_updates_mixed_borrows() {
    let range = ServoRange::default();
    let mut left = Servo::from_range(
        MockPwm {
            duty: 0,
            max: 1_000,
        },
        range,
    );
    let mut right = Servo::from_range(
        MockPwm {
            duty: 0,
            max: 1_000,
        },
        range,
    );
    {
        let mut bank = ServoBank::new([
            &mut left as &mut dyn ServoOutput<Error = Infallible>,
            &mut right as &mut dyn ServoOutput<Error = Infallible>,
        ]);
        bank.set_symmetric([0.5, -0.5]).unwrap();
    }
    let left = left.release();
    let right = right.release();
    assert!(left.duty > right.duty);
}

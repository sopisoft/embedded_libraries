use core::convert::Infallible;

use control::{ControlAxes, ConventionalTailMixer};
use embedded_hal::pwm::{ErrorType, SetDutyCycle};
use fugit::MicrosDurationU32;
use pwm::{Servo, ServoBank, ServoOutput, ServoRange, ServoSet};

// This example demonstrates how to manage multiple servos with one shared set
// of travel limits. The servos still own their individual PWM channels, but a
// borrowed ServoBank lets us update them in one place.

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
    // Shared servo timing for three control surfaces:
    // left aileron, right aileron, elevator.
    let base = ServoRange::new(
        MicrosDurationU32::from_micros(20_000),
        MicrosDurationU32::from_micros(1_000),
        MicrosDurationU32::from_micros(2_000),
        -60.0,
        60.0,
    );

    let ranges = ServoSet::new([base, base, base]);

    let mut left_aileron = Servo::from_range(
        MockPwmChannel {
            compare: 0,
            top: 20_000,
        },
        ranges.range(0),
    );
    let mut right_aileron = Servo::from_range(
        MockPwmChannel {
            compare: 0,
            top: 20_000,
        },
        ranges.range(1),
    );
    let mut elevator = Servo::from_range(
        MockPwmChannel {
            compare: 0,
            top: 20_000,
        },
        ranges.range(2),
    );

    // A fixed-wing mixer usually sits one layer above the PWM library.
    let mixer = ConventionalTailMixer::new();
    let outputs = mixer.mix(ControlAxes::new(0.4, -0.2, 0.0, 0.6, 0.0));

    // Convert multiple logical surface commands into multiple pulse widths.
    let pulses = ranges.pulse_widths_from_symmetric([
        outputs.left_aileron,
        outputs.right_aileron,
        outputs.elevator,
    ]);

    // Apply the pulse widths to each servo channel.
    {
        let mut bank = ServoBank::new([
            &mut left_aileron as &mut dyn ServoOutput<Error = Infallible>,
            &mut right_aileron as &mut dyn ServoOutput<Error = Infallible>,
            &mut elevator as &mut dyn ServoOutput<Error = Infallible>,
        ]);
        bank.set_pulse_widths(pulses).unwrap();
    }

    let left_aileron = left_aileron.release();
    let right_aileron = right_aileron.release();
    let elevator = elevator.release();

    println!("Left aileron PWM compare:  {}", left_aileron.compare);
    println!("Right aileron PWM compare: {}", right_aileron.compare);
    println!("Elevator PWM compare:      {}", elevator.compare);
}

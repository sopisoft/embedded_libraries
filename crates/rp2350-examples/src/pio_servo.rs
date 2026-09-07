use core::convert::Infallible;

use embedded_hal::pwm::{ErrorType, SetDutyCycle};
use rp235x_hal::pio::{Running, StateMachine, Tx, ValidStateMachine};

pub const FRAME_PERIOD_US: u16 = 20_000;
const PROGRAM_OVERHEAD_CYCLES: u16 = 7;

pub fn servo_program() -> pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
    pio::pio_asm!(
        "pull block",
        "out x, 16",
        "out y, 16",
        "set pins, 1",
        "high:",
        "jmp x-- high",
        "set pins, 0",
        "low:",
        "jmp y-- low",
    )
    .program
}

pub struct PioServo<SM: ValidStateMachine> {
    state_machine: StateMachine<SM, Running>,
    tx: Tx<SM>,
}

impl<SM: ValidStateMachine> PioServo<SM> {
    pub const fn new(state_machine: StateMachine<SM, Running>, tx: Tx<SM>) -> Self {
        Self { state_machine, tx }
    }
}

impl<SM: ValidStateMachine> ErrorType for PioServo<SM> {
    type Error = Infallible;
}

impl<SM: ValidStateMachine> SetDutyCycle for PioServo<SM> {
    fn max_duty_cycle(&self) -> u16 {
        FRAME_PERIOD_US
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        let high = duty.min(FRAME_PERIOD_US);
        let low = FRAME_PERIOD_US
            .saturating_sub(PROGRAM_OVERHEAD_CYCLES)
            .saturating_sub(high);
        let frame = (u32::from(low) << 16) | u32::from(high);
        if self.tx.is_full() {
            self.state_machine.drain_tx_fifo();
        }
        let _ = self.tx.write(frame);
        Ok(())
    }
}

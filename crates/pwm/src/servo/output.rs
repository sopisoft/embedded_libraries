use fugit::MicrosDurationU32;

/// Object-safe PWM pulse output interface.
pub trait ServoOutput {
    /// Error type returned by the underlying PWM backend.
    type Error;

    /// Writes a raw pulse width.
    fn set_pulse_width(&mut self, pulse: MicrosDurationU32) -> Result<(), Self::Error>;
}

/// Borrowed view over several servo outputs, including mixed concrete types.
pub struct ServoBank<'a, E, const N: usize> {
    servos: [&'a mut dyn ServoOutput<Error = E>; N],
}

impl<'a, E, const N: usize> ServoBank<'a, E, N> {
    /// Creates a new borrowed servo bank.
    pub fn new(servos: [&'a mut dyn ServoOutput<Error = E>; N]) -> Self {
        Self { servos }
    }

    /// Returns the number of attached outputs.
    pub const fn len(&self) -> usize {
        N
    }

    /// Returns whether the bank is empty.
    pub const fn is_empty(&self) -> bool {
        N == 0
    }

    /// Applies explicit pulse widths to all servos.
    pub fn set_pulse_widths(&mut self, pulses: [MicrosDurationU32; N]) -> Result<(), E> {
        let mut i = 0;
        while i < N {
            self.servos[i].set_pulse_width(pulses[i])?;
            i += 1;
        }
        Ok(())
    }
}

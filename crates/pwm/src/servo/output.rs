use fugit::MicrosDurationU32;

/// Object-safe servo control interface.
///
/// This trait is useful when an application needs to update several servo
/// outputs that do not share the same concrete PWM type, such as RP2350 PWM
/// channels from different timer slices.
pub trait ServoOutput {
    /// Error type returned by the underlying PWM backend.
    type Error;

    /// Sets the servo position as a normalized value in `[0, 1]`.
    fn set_normalized(&mut self, position: f32) -> Result<(), Self::Error>;

    /// Sets the servo position from a symmetric command in `[-1, 1]`.
    fn set_symmetric(&mut self, command: f32) -> Result<(), Self::Error>;

    /// Sets the servo angle in degrees.
    fn set_angle_degrees(&mut self, angle_deg: f32) -> Result<(), Self::Error>;

    /// Sets the servo angle in radians.
    fn set_angle_radians(&mut self, angle_rad: f32) -> Result<(), Self::Error>;

    /// Writes a raw pulse width.
    fn set_pulse_width(&mut self, pulse: MicrosDurationU32) -> Result<(), Self::Error>;
}

/// Borrowed view over several servo outputs, including mixed concrete types.
///
/// This form is intended for hardware-specific code where channels often come
/// from different timer slices and therefore cannot be stored in a homogeneous
/// array.
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

    /// Applies normalized commands in `[0, 1]` to all servos.
    pub fn set_normalized(&mut self, commands: [f32; N]) -> Result<(), E> {
        let mut i = 0;
        while i < N {
            self.servos[i].set_normalized(commands[i])?;
            i += 1;
        }
        Ok(())
    }

    /// Applies symmetric commands in `[-1, 1]` to all servos.
    pub fn set_symmetric(&mut self, commands: [f32; N]) -> Result<(), E> {
        let mut i = 0;
        while i < N {
            self.servos[i].set_symmetric(commands[i])?;
            i += 1;
        }
        Ok(())
    }

    /// Applies angle commands in degrees to all servos.
    pub fn set_angles_degrees(&mut self, angles_deg: [f32; N]) -> Result<(), E> {
        let mut i = 0;
        while i < N {
            self.servos[i].set_angle_degrees(angles_deg[i])?;
            i += 1;
        }
        Ok(())
    }

    /// Applies angle commands in radians to all servos.
    pub fn set_angles_radians(&mut self, angles_rad: [f32; N]) -> Result<(), E> {
        let mut i = 0;
        while i < N {
            self.servos[i].set_angle_radians(angles_rad[i])?;
            i += 1;
        }
        Ok(())
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

//! Bus helpers for sharing one I2C peripheral across multiple sensors.

use core::cell::RefCell;

use embedded_hal::i2c::{ErrorType, I2c, Operation};

/// Shared mutable access to one I2C bus using `RefCell`.
///
/// This is useful when one board exposes multiple I2C devices, such as an
/// accelerometer/gyroscope plus a separate magnetometer.
#[derive(Copy, Clone)]
pub struct SharedI2c<'a, BUS> {
    bus: &'a RefCell<BUS>,
}

impl<'a, BUS> SharedI2c<'a, BUS> {
    /// Creates a new shared bus view.
    pub const fn new(bus: &'a RefCell<BUS>) -> Self {
        Self { bus }
    }
}

impl<BUS> ErrorType for SharedI2c<'_, BUS>
where
    BUS: ErrorType,
{
    type Error = BUS::Error;
}

impl<BUS> I2c for SharedI2c<'_, BUS>
where
    BUS: I2c,
{
    fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.borrow_mut().read(address, read)
    }

    fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.bus.borrow_mut().write(address, write)
    }

    fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.bus.borrow_mut().write_read(address, write, read)
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.bus.borrow_mut().transaction(address, operations)
    }
}

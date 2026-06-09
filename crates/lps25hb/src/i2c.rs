//! I2C transport for the LPS25HB driver.

use embedded_hal::i2c::I2c;

use crate::{Address, interface::RegisterInterface};

/// I2C register interface for `Lps25hb`.
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> I2cInterface<I2C> {
    /// Creates a new I2C transport wrapper.
    pub const fn new(i2c: I2C, address: Address) -> Self {
        Self {
            i2c,
            address: address.as_u8(),
        }
    }

    /// Returns the configured 7-bit I2C address.
    pub const fn address(&self) -> u8 {
        self.address
    }

    /// Releases the inner I2C peripheral.
    pub fn destroy(self) -> I2C {
        self.i2c
    }
}

impl<I2C> RegisterInterface for I2cInterface<I2C>
where
    I2C: I2c,
{
    type Error = I2C::Error;

    fn read_register(&mut self, register: u8) -> Result<u8, Self::Error> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.address, &[register], &mut buffer)?;
        Ok(buffer[0])
    }

    fn read_many(&mut self, start_register: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c
            .write_read(self.address, &[start_register | 0x80], buffer)
    }

    fn write_register(&mut self, register: u8, value: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.address, &[register, value])
    }

    fn write_many(&mut self, start_register: u8, values: &[u8]) -> Result<(), Self::Error> {
        match values {
            [] => Ok(()),
            [value] => self.write_register(start_register, *value),
            [first, second] => self
                .i2c
                .write(self.address, &[start_register | 0x80, *first, *second]),
            _ => {
                let mut index = 0usize;
                while index < values.len() {
                    self.write_register(start_register.wrapping_add(index as u8), values[index])?;
                    index += 1;
                }
                Ok(())
            }
        }
    }
}

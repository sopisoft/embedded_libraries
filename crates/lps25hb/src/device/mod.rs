use crate::{i2c::I2cInterface, spi::SpiInterface};

mod conversion;
mod driver;
#[cfg(test)]
mod tests;

pub use conversion::{
    altitude_to_pressure_hpa, one_point_calibration_rpds, pressure_error_to_rpds_counts,
    pressure_to_altitude_m, raw_pressure_to_hpa, raw_temperature_to_celsius,
    rpds_counts_to_pressure_hpa,
};
pub use driver::Lps25hb;

impl<I2C> Lps25hb<I2cInterface<I2C>> {
    /// Creates an I2C-backed driver.
    pub const fn new_i2c(i2c: I2C, address: crate::Address) -> Self {
        Self::new(I2cInterface::new(i2c, address))
    }

    /// Returns the configured I2C address.
    pub const fn address(&self) -> u8 {
        self.interface().address()
    }

    /// Releases the I2C peripheral.
    pub fn destroy(self) -> I2C {
        self.into_interface().destroy()
    }
}

impl<SPI, CS> Lps25hb<SpiInterface<SPI, CS>> {
    /// Creates a 4-wire SPI-backed driver.
    pub const fn new_spi(spi: SPI, cs: CS) -> Self {
        Self::new(SpiInterface::new(spi, cs))
    }

    /// Releases the SPI bus and chip-select pin.
    pub fn release(self) -> (SPI, CS) {
        self.into_interface().release()
    }
}

use embedded_hal::i2c::I2c;

use crate::registers::{
    CTRL_REG1, CTRL_REG2, CTRL_REG3, CTRL_REG4, CTRL_REG5, OUT_X_H, OUT_X_L, OUT_Y_H, OUT_Y_L,
    OUT_Z_H, OUT_Z_L, STATUS_REG, WHO_AM_I,
};
use crate::{
    Address, Config, DEVICE_ID, DataRate, Error, FullScale, MagneticField, MeasurementMode,
    OperatingMode, RawMagneticField,
};

/// LIS3MDL driver.
#[derive(Debug)]
pub struct Lis3mdl<I2C> {
    i2c: I2C,
    address: u8,
    full_scale: FullScale,
}

impl<I2C> Lis3mdl<I2C> {
    /// Creates a driver instance without touching the device yet.
    pub const fn new(i2c: I2C, address: Address) -> Self {
        Self {
            i2c,
            address: address.as_u8(),
            full_scale: FullScale::Gauss12,
        }
    }

    /// Returns the configured I2C address.
    pub const fn address(&self) -> u8 {
        self.address
    }

    /// Returns the current full-scale cache.
    pub const fn full_scale(&self) -> FullScale {
        self.full_scale
    }

    /// Releases the I2C peripheral.
    pub fn destroy(self) -> I2C {
        self.i2c
    }
}

impl<I2C> Lis3mdl<I2C>
where
    I2C: I2c,
{
    /// Verifies the chip ID and applies the supplied configuration.
    pub fn init(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        let device_id = self.who_am_i()?;
        if device_id != DEVICE_ID {
            return Err(Error::InvalidDeviceId(device_id));
        }
        self.apply_config(config)
    }

    /// Reads the chip ID.
    pub fn who_am_i(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(WHO_AM_I)
    }

    /// Applies a complete configuration block.
    pub fn apply_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        self.set_full_scale(config.full_scale)?;
        self.set_operating_mode(config.operating_mode)?;
        self.set_measurement_mode(config.measurement_mode)?;
        self.set_data_rate(config.data_rate)?;
        self.set_block_data_update(config.block_data_update)?;
        self.set_temperature_enable(config.temperature_enable)?;
        Ok(())
    }

    /// Sets the magnetic full-scale range.
    pub fn set_full_scale(&mut self, full_scale: FullScale) -> Result<(), Error<I2C::Error>> {
        self.update_register(CTRL_REG2, !(0b11 << 5), full_scale.register_bits())?;
        self.full_scale = full_scale;
        Ok(())
    }

    /// Sets the X/Y/Z operating mode.
    pub fn set_operating_mode(
        &mut self,
        operating_mode: OperatingMode,
    ) -> Result<(), Error<I2C::Error>> {
        self.update_register(CTRL_REG1, !((0b11 << 5) | 0b10), operating_mode.xy_bits())?;
        self.update_register(CTRL_REG4, !(0b11 << 2), operating_mode.z_bits())?;
        Ok(())
    }

    /// Sets the measurement mode.
    pub fn set_measurement_mode(
        &mut self,
        measurement_mode: MeasurementMode,
    ) -> Result<(), Error<I2C::Error>> {
        self.update_register(CTRL_REG3, !0b11, measurement_mode.register_bits())
    }

    /// Sets the output data rate.
    pub fn set_data_rate(&mut self, data_rate: DataRate) -> Result<(), Error<I2C::Error>> {
        self.update_register(CTRL_REG1, !((0b111 << 2) | 0b10), data_rate.register_bits())
    }

    /// Enables or disables block data update.
    pub fn set_block_data_update(&mut self, enabled: bool) -> Result<(), Error<I2C::Error>> {
        let bits = if enabled { 0b0100_0000 } else { 0 };
        self.update_register(CTRL_REG5, !0b0100_0000, bits)
    }

    /// Enables or disables the on-chip temperature sensor.
    pub fn set_temperature_enable(&mut self, enabled: bool) -> Result<(), Error<I2C::Error>> {
        let bits = if enabled { 0b1000_0000 } else { 0 };
        self.update_register(CTRL_REG1, !0b1000_0000, bits)
    }

    /// Returns `true` when a fresh XYZ magnetic sample is available.
    pub fn magnetic_data_ready(&mut self) -> Result<bool, Error<I2C::Error>> {
        Ok((self.read_register(STATUS_REG)? & 0b0000_1000) != 0)
    }

    /// Reads raw magnetic counts.
    pub fn read_raw_magnetic(&mut self) -> Result<RawMagneticField, Error<I2C::Error>> {
        Ok(RawMagneticField {
            x: self.read_i16(OUT_X_L, OUT_X_H)?,
            y: self.read_i16(OUT_Y_L, OUT_Y_H)?,
            z: self.read_i16(OUT_Z_L, OUT_Z_H)?,
        })
    }

    /// Reads magnetic field strength in milli-gauss.
    pub fn read_magnetic_mgauss(&mut self) -> Result<MagneticField, Error<I2C::Error>> {
        let raw = self.read_raw_magnetic()?;
        let sensitivity = self.full_scale.sensitivity_mgauss_per_lsb();
        Ok(MagneticField {
            x_mgauss: raw.x as f32 * sensitivity,
            y_mgauss: raw.y as f32 * sensitivity,
            z_mgauss: raw.z as f32 * sensitivity,
        })
    }

    fn read_i16(&mut self, low_reg: u8, high_reg: u8) -> Result<i16, Error<I2C::Error>> {
        let low = self.read_register(low_reg)?;
        let high = self.read_register(high_reg)?;
        Ok(i16::from_le_bytes([low, high]))
    }

    fn update_register(
        &mut self,
        register: u8,
        keep_mask: u8,
        set_bits: u8,
    ) -> Result<(), Error<I2C::Error>> {
        let current = self.read_register(register)?;
        self.write_register(register, (current & keep_mask) | set_bits)
    }

    fn read_register(&mut self, register: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.address, &[register], &mut buffer)
            .map_err(Error::Bus)?;
        Ok(buffer[0])
    }

    fn write_register(&mut self, register: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[register, value])
            .map_err(Error::Bus)
    }
}

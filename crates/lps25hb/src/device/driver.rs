use crate::{i2c::I2cInterface, spi::SpiInterface};
use crate::{
    interface::RegisterInterface,
    registers::{
        CTRL_REG1, CTRL_REG1_ACTIVE, CTRL_REG1_BDU, CTRL_REG1_DIFF_EN, CTRL_REG1_ODR_MASK,
        CTRL_REG2, CTRL_REG2_AUTOZERO, CTRL_REG2_BOOT, CTRL_REG2_ONE_SHOT, CTRL_REG2_SWRESET,
        PRESS_OUT_XL, RES_CONF, RPDS_L, STATUS_REG, WHO_AM_I,
    },
    types::{DEVICE_ID, Error, Measurement, OutputDataRate, RawMeasurement, Status},
};

use super::{one_point_calibration_rpds, raw_pressure_to_hpa, raw_temperature_to_celsius};

/// Transport-independent LPS25HB driver.
#[derive(Debug)]
pub struct Lps25hb<IF> {
    interface: IF,
}

impl<IF> Lps25hb<IF> {
    /// Creates a new driver from a prepared bus interface.
    pub const fn new(interface: IF) -> Self {
        Self { interface }
    }

    /// Releases the interface object.
    pub fn into_interface(self) -> IF {
        self.interface
    }

    /// Returns a shared reference to the interface.
    pub const fn interface(&self) -> &IF {
        &self.interface
    }

    /// Returns a mutable reference to the interface.
    pub fn interface_mut(&mut self) -> &mut IF {
        &mut self.interface
    }
}

impl<IF> Lps25hb<IF>
where
    IF: RegisterInterface,
{
    /// Verifies the chip ID and applies the supplied configuration.
    pub fn init(&mut self, config: crate::Config) -> Result<(), Error<IF::Error>> {
        let device_id = self.who_am_i()?;
        if device_id != DEVICE_ID {
            return Err(Error::InvalidDeviceId(device_id));
        }
        self.apply_config(config)
    }

    /// Reads the chip ID.
    pub fn who_am_i(&mut self) -> Result<u8, Error<IF::Error>> {
        self.read_register(WHO_AM_I)
    }

    /// Applies the full configuration block.
    pub fn apply_config(&mut self, config: crate::Config) -> Result<(), Error<IF::Error>> {
        let res_conf =
            config.temperature_average.register_bits() | config.pressure_average.register_bits();
        let mut ctrl_reg1 = CTRL_REG1_ACTIVE | config.output_data_rate.register_bits();
        if config.block_data_update {
            ctrl_reg1 |= CTRL_REG1_BDU;
        }
        if config.differential_output {
            ctrl_reg1 |= CTRL_REG1_DIFF_EN;
        }
        self.write_register(RES_CONF, res_conf)?;
        self.write_register(CTRL_REG1, ctrl_reg1)
    }

    /// Puts the device into power-down mode while preserving lower control bits.
    pub fn power_down(&mut self) -> Result<(), Error<IF::Error>> {
        self.update_register(CTRL_REG1, 0b0000_1111, 0)
    }

    /// Sets the continuous output data rate and enables active mode.
    pub fn set_output_data_rate(
        &mut self,
        output_data_rate: OutputDataRate,
    ) -> Result<(), Error<IF::Error>> {
        self.update_register(
            CTRL_REG1,
            !(CTRL_REG1_ACTIVE | CTRL_REG1_ODR_MASK),
            CTRL_REG1_ACTIVE | output_data_rate.register_bits(),
        )
    }

    /// Enables or disables block data update.
    pub fn set_block_data_update(&mut self, enabled: bool) -> Result<(), Error<IF::Error>> {
        let bits = if enabled { CTRL_REG1_BDU } else { 0 };
        self.update_register(CTRL_REG1, !CTRL_REG1_BDU, bits)
    }

    /// Enables or disables differential pressure output mode.
    pub fn set_differential_output(&mut self, enabled: bool) -> Result<(), Error<IF::Error>> {
        let bits = if enabled { CTRL_REG1_DIFF_EN } else { 0 };
        self.update_register(CTRL_REG1, !CTRL_REG1_DIFF_EN, bits)
    }

    /// Issues a software reset.
    pub fn software_reset(&mut self) -> Result<(), Error<IF::Error>> {
        self.update_register(CTRL_REG2, !CTRL_REG2_SWRESET, CTRL_REG2_SWRESET)
    }

    /// Reloads the factory trim values from internal non-volatile memory.
    pub fn reboot_memory(&mut self) -> Result<(), Error<IF::Error>> {
        self.update_register(CTRL_REG2, !CTRL_REG2_BOOT, CTRL_REG2_BOOT)
    }

    /// Copies the current pressure into the reference registers and enables
    /// differential pressure output.
    pub fn enable_autozero(&mut self) -> Result<(), Error<IF::Error>> {
        self.update_register(CTRL_REG2, !CTRL_REG2_AUTOZERO, CTRL_REG2_AUTOZERO)
    }

    /// Triggers a one-shot conversion when the device is configured for
    /// one-shot mode (`ODR = 000`).
    pub fn trigger_one_shot(&mut self) -> Result<(), Error<IF::Error>> {
        self.update_register(CTRL_REG2, !CTRL_REG2_ONE_SHOT, CTRL_REG2_ONE_SHOT)
    }

    /// Reads the status register.
    pub fn read_status(&mut self) -> Result<Status, Error<IF::Error>> {
        let raw = self.read_register(STATUS_REG)?;
        Ok(Status {
            pressure_ready: (raw & 0b0000_0010) != 0,
            temperature_ready: (raw & 0b0000_0001) != 0,
            pressure_overrun: (raw & 0b0010_0000) != 0,
            temperature_overrun: (raw & 0b0001_0000) != 0,
        })
    }

    /// Returns `true` when a fresh pressure sample is available.
    pub fn pressure_data_ready(&mut self) -> Result<bool, Error<IF::Error>> {
        Ok(self.read_status()?.pressure_ready)
    }

    /// Returns `true` when a fresh temperature sample is available.
    pub fn temperature_data_ready(&mut self) -> Result<bool, Error<IF::Error>> {
        Ok(self.read_status()?.temperature_ready)
    }

    /// Reads raw pressure and temperature in one burst.
    pub fn read_raw_measurement(&mut self) -> Result<RawMeasurement, Error<IF::Error>> {
        let mut buffer = [0u8; 5];
        self.read_many(PRESS_OUT_XL, &mut buffer)?;

        let mut pressure_raw =
            ((buffer[2] as i32) << 16) | ((buffer[1] as i32) << 8) | buffer[0] as i32;
        if (pressure_raw & 0x0080_0000) != 0 {
            pressure_raw |= !0x00FF_FFFF;
        }

        Ok(RawMeasurement {
            pressure_raw,
            temperature_raw: i16::from_le_bytes([buffer[3], buffer[4]]),
        })
    }

    /// Reads both channels and converts them to hPa and degrees Celsius.
    pub fn read_measurement(&mut self) -> Result<Measurement, Error<IF::Error>> {
        let raw = self.read_raw_measurement()?;
        Ok(Measurement {
            pressure_hpa: raw_pressure_to_hpa(raw.pressure_raw),
            temperature_c: raw_temperature_to_celsius(raw.temperature_raw),
        })
    }

    /// Reads only the pressure value in hPa.
    pub fn read_pressure_hpa(&mut self) -> Result<f32, Error<IF::Error>> {
        Ok(self.read_measurement()?.pressure_hpa)
    }

    /// Reads only the temperature value in degrees Celsius.
    pub fn read_temperature_c(&mut self) -> Result<f32, Error<IF::Error>> {
        Ok(self.read_measurement()?.temperature_c)
    }

    /// Reads the 16-bit RPDS offset register.
    pub fn read_pressure_offset_counts(&mut self) -> Result<i16, Error<IF::Error>> {
        let mut buffer = [0u8; 2];
        self.read_many(RPDS_L, &mut buffer)?;
        Ok(i16::from_le_bytes(buffer))
    }

    /// Writes the 16-bit RPDS offset register used for one-point calibration.
    pub fn write_pressure_offset_counts(&mut self, counts: i16) -> Result<(), Error<IF::Error>> {
        self.write_many(RPDS_L, &counts.to_le_bytes())
    }

    /// Applies the RPDS one-point calibration from a measured and a trusted
    /// reference pressure.
    pub fn apply_one_point_calibration(
        &mut self,
        measured_pressure_hpa: f32,
        reference_pressure_hpa: f32,
    ) -> Result<(), Error<IF::Error>> {
        self.write_pressure_offset_counts(one_point_calibration_rpds(
            measured_pressure_hpa,
            reference_pressure_hpa,
        ))
    }

    fn read_register(&mut self, register: u8) -> Result<u8, Error<IF::Error>> {
        self.interface.read_register(register).map_err(Error::Bus)
    }

    fn read_many(&mut self, start_register: u8, buffer: &mut [u8]) -> Result<(), Error<IF::Error>> {
        self.interface
            .read_many(start_register, buffer)
            .map_err(Error::Bus)
    }

    fn write_register(&mut self, register: u8, value: u8) -> Result<(), Error<IF::Error>> {
        self.interface
            .write_register(register, value)
            .map_err(Error::Bus)
    }

    fn write_many(&mut self, start_register: u8, values: &[u8]) -> Result<(), Error<IF::Error>> {
        self.interface
            .write_many(start_register, values)
            .map_err(Error::Bus)
    }

    fn update_register(
        &mut self,
        register: u8,
        keep_mask: u8,
        set_bits: u8,
    ) -> Result<(), Error<IF::Error>> {
        let current = self.read_register(register)?;
        self.write_register(register, (current & keep_mask) | set_bits)
    }
}

#[allow(dead_code)]
type _Interfaces<I2C, SPI, CS> = (
    core::marker::PhantomData<I2cInterface<I2C>>,
    core::marker::PhantomData<SpiInterface<SPI, CS>>,
);

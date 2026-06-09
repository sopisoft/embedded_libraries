//! SPI transport for the LPS25HB driver.
//!
//! This module implements the 4-wire SPI protocol described in the ST
//! datasheet. The read/write header layout is:
//! - bit 0: read/write
//! - bit 1: address auto-increment for burst transfers
//! - bit 2..=7: register address

use embedded_hal::{digital::OutputPin, spi::SpiBus};

use crate::interface::RegisterInterface;

/// SPI transport error.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SpiBusError<SpiError, ChipSelectError> {
    Spi(SpiError),
    ChipSelect(ChipSelectError),
}

/// SPI register interface for `Lps25hb`.
#[derive(Debug)]
pub struct SpiInterface<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS> SpiInterface<SPI, CS> {
    /// Creates a new 4-wire SPI transport wrapper.
    pub const fn new(spi: SPI, cs: CS) -> Self {
        Self { spi, cs }
    }

    /// Releases the SPI bus and chip-select pin.
    pub fn release(self) -> (SPI, CS) {
        (self.spi, self.cs)
    }
}

impl<SPI, CS> SpiInterface<SPI, CS>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
{
    fn with_chip_selected<T, F>(&mut self, f: F) -> Result<T, SpiBusError<SPI::Error, CS::Error>>
    where
        F: FnOnce(&mut SPI) -> Result<T, SPI::Error>,
    {
        self.cs.set_low().map_err(SpiBusError::ChipSelect)?;
        let spi_result = f(&mut self.spi).map_err(SpiBusError::Spi);
        let cs_result = self.cs.set_high().map_err(SpiBusError::ChipSelect);

        match (spi_result, cs_result) {
            (Ok(value), Ok(())) => Ok(value),
            (Err(err), _) => Err(err),
            (Ok(_), Err(err)) => Err(err),
        }
    }

    const fn command(register: u8, is_read: bool, auto_increment: bool) -> u8 {
        (register << 2) | ((auto_increment as u8) << 1) | is_read as u8
    }
}

impl<SPI, CS> RegisterInterface for SpiInterface<SPI, CS>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
{
    type Error = SpiBusError<SPI::Error, CS::Error>;

    fn read_register(&mut self, register: u8) -> Result<u8, Self::Error> {
        let mut buffer = [0u8; 1];
        self.with_chip_selected(|spi| {
            spi.write(&[Self::command(register, true, false)])?;
            spi.read(&mut buffer)
        })?;
        Ok(buffer[0])
    }

    fn read_many(&mut self, start_register: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.with_chip_selected(|spi| {
            spi.write(&[Self::command(start_register, true, true)])?;
            spi.read(buffer)
        })
    }

    fn write_register(&mut self, register: u8, value: u8) -> Result<(), Self::Error> {
        self.with_chip_selected(|spi| spi.write(&[Self::command(register, false, false), value]))
    }

    fn write_many(&mut self, start_register: u8, values: &[u8]) -> Result<(), Self::Error> {
        self.with_chip_selected(|spi| {
            spi.write(&[Self::command(start_register, false, values.len() > 1)])?;
            spi.write(values)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;
    use embedded_hal::{
        digital::ErrorType as DigitalErrorType,
        spi::{ErrorType as SpiErrorType, SpiBus},
    };

    use crate::registers::PRESS_OUT_XL;
    use crate::{Config, DEVICE_ID, Lps25hb, Measurement, raw_pressure_to_hpa};

    #[derive(Debug)]
    struct MockSpi {
        next_read: [u8; 8],
        next_read_len: usize,
        last_write: [u8; 8],
        last_write_len: usize,
    }

    impl MockSpi {
        fn new() -> Self {
            Self {
                next_read: [0; 8],
                next_read_len: 0,
                last_write: [0; 8],
                last_write_len: 0,
            }
        }
    }

    impl SpiErrorType for MockSpi {
        type Error = Infallible;
    }

    impl SpiBus<u8> for MockSpi {
        fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            let len = core::cmp::min(words.len(), self.next_read_len);
            let mut index = 0usize;
            while index < len {
                words[index] = self.next_read[index];
                index += 1;
            }
            Ok(())
        }

        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            self.last_write_len = core::cmp::min(words.len(), self.last_write.len());
            let mut index = 0usize;
            while index < self.last_write_len {
                self.last_write[index] = words[index];
                index += 1;
            }
            Ok(())
        }

        fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
            self.write(write)?;
            self.read(read)
        }

        fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            self.write(words)?;
            self.read(words)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[derive(Debug)]
    struct MockPin {
        high: bool,
    }

    impl DigitalErrorType for MockPin {
        type Error = Infallible;
    }

    impl OutputPin for MockPin {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.high = false;
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.high = true;
            Ok(())
        }
    }

    #[test]
    fn spi_read_register_uses_expected_command_header() {
        let mut spi = MockSpi::new();
        spi.next_read[0] = DEVICE_ID;
        spi.next_read_len = 1;

        let cs = MockPin { high: true };
        let mut sensor = Lps25hb::new_spi(spi, cs);
        let who_am_i = sensor.who_am_i().unwrap();
        assert_eq!(who_am_i, DEVICE_ID);

        let (spi, cs) = sensor.release();
        assert_eq!(spi.last_write_len, 1);
        assert_eq!(spi.last_write[0], (0x0F << 2) | 0x01);
        assert!(cs.high);
    }

    #[test]
    fn spi_write_register_uses_expected_command_header() {
        let spi = MockSpi::new();
        let cs = MockPin { high: true };
        let mut sensor = Lps25hb::new_spi(spi, cs);
        sensor.apply_config(Config::akizuki_style()).unwrap();

        let (spi, _) = sensor.release();
        assert_eq!(spi.last_write_len, 2);
        assert_eq!(spi.last_write[0], 0x20 << 2);
        assert_eq!(spi.last_write[1], 0x94);
    }

    #[test]
    fn spi_burst_read_returns_pressure_and_temperature() {
        let mut spi = MockSpi::new();
        let pressure_raw = (1000.5 * 4096.0) as i32;
        let pressure = pressure_raw.to_le_bytes();
        let temperature_raw = ((21.5 - 42.5) * 480.0) as i16;
        let temperature = temperature_raw.to_le_bytes();
        spi.next_read[..5].copy_from_slice(&[
            pressure[0],
            pressure[1],
            pressure[2],
            temperature[0],
            temperature[1],
        ]);
        spi.next_read_len = 5;

        let cs = MockPin { high: true };
        let mut sensor = Lps25hb::new_spi(spi, cs);
        let measurement = sensor.read_measurement().unwrap();
        assert_eq!(
            measurement,
            Measurement {
                pressure_hpa: raw_pressure_to_hpa(pressure_raw),
                temperature_c: 21.5,
            }
        );

        let (spi, _) = sensor.release();
        assert_eq!(spi.last_write_len, 1);
        assert_eq!(spi.last_write[0], (PRESS_OUT_XL << 2) | 0x03);
    }
}

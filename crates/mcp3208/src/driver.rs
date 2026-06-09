//! Driver for the MCP3208 12-bit SPI ADC.

use embedded_hal::{digital::OutputPin, spi::SpiBus};

/// ADC input selection.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Channel {
    /// Single-ended input channel 0..=7.
    SingleEnded(u8),
    /// Differential pair. The negative input must be the adjacent complement.
    Differential { positive: u8, negative: u8 },
}

/// Driver error.
#[derive(Debug, PartialEq, Eq)]
pub enum Error<SpiError, PinError> {
    /// SPI bus error.
    Spi(SpiError),
    /// Chip-select GPIO error.
    Pin(PinError),
    /// Invalid channel selection.
    InvalidChannel,
}

/// MCP3208 driver.
pub struct Mcp3208<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS> Mcp3208<SPI, CS> {
    /// Creates a new driver.
    pub const fn new(spi: SPI, cs: CS) -> Self {
        Self { spi, cs }
    }

    /// Releases the inner bus and chip-select pin.
    pub fn release(self) -> (SPI, CS) {
        (self.spi, self.cs)
    }
}

impl<SPI, CS> Mcp3208<SPI, CS>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
{
    /// Reads a raw 12-bit conversion result.
    pub fn read_raw(&mut self, channel: Channel) -> Result<u16, Error<SPI::Error, CS::Error>> {
        let command = Self::encode_command(channel).ok_or(Error::InvalidChannel)?;
        let mut frame = [command[0], command[1], 0x00];

        self.cs.set_low().map_err(Error::Pin)?;
        let spi_result = self.spi.transfer_in_place(&mut frame).map_err(Error::Spi);
        let cs_result = self.cs.set_high().map_err(Error::Pin);

        match (spi_result, cs_result) {
            (Ok(()), Ok(())) => Ok(((frame[1] as u16 & 0x0F) << 8) | frame[2] as u16),
            (Err(err), _) => Err(err),
            (Ok(()), Err(err)) => Err(err),
        }
    }

    /// Reads a normalized ratio in `[0.0, 1.0]`.
    pub fn read_normalized(
        &mut self,
        channel: Channel,
    ) -> Result<f32, Error<SPI::Error, CS::Error>> {
        Ok(self.read_raw(channel)? as f32 / 4095.0)
    }

    /// Reads the voltage in millivolts given a millivolt reference.
    pub fn read_voltage_mv(
        &mut self,
        channel: Channel,
        vref_mv: u16,
    ) -> Result<u16, Error<SPI::Error, CS::Error>> {
        let raw = self.read_raw(channel)? as u32;
        let mv = (raw * vref_mv as u32) / 4095;
        Ok(mv as u16)
    }

    fn encode_command(channel: Channel) -> Option<[u8; 2]> {
        match channel {
            Channel::SingleEnded(index) if index < 8 => {
                Some([0x06 | ((index & 0x04) >> 2), (index & 0x03) << 6])
            }
            Channel::Differential { positive, negative }
                if positive < 8 && negative < 8 && negative == (positive ^ 1) =>
            {
                Some([0x04 | ((positive & 0x04) >> 2), (positive & 0x03) << 6])
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;
    use embedded_hal::{
        digital::ErrorType as DigitalErrorType,
        digital::OutputPin,
        spi::{ErrorType as SpiErrorType, SpiBus},
    };

    #[derive(Debug)]
    struct MockSpi {
        response: [u8; 3],
        last_tx: [u8; 3],
    }

    impl SpiErrorType for MockSpi {
        type Error = Infallible;
    }

    impl SpiBus<u8> for MockSpi {
        fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            let len = core::cmp::min(words.len(), self.response.len());
            let mut i = 0;
            while i < len {
                words[i] = self.response[i];
                i += 1;
            }
            Ok(())
        }

        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            let len = core::cmp::min(words.len(), self.last_tx.len());
            let mut i = 0;
            while i < len {
                self.last_tx[i] = words[i];
                i += 1;
            }
            Ok(())
        }

        fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
            self.write(write)?;
            self.read(read)
        }

        fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            let len = core::cmp::min(words.len(), self.last_tx.len());
            let mut i = 0;
            while i < len {
                self.last_tx[i] = words[i];
                words[i] = self.response[i];
                i += 1;
            }
            Ok(())
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
    fn read_raw_parses_adc_value() {
        let spi = MockSpi {
            response: [0x00, 0x0A, 0xBC],
            last_tx: [0; 3],
        };
        let cs = MockPin { high: true };
        let mut adc = Mcp3208::new(spi, cs);
        let raw = adc.read_raw(Channel::SingleEnded(3)).unwrap();
        assert_eq!(raw, 0x0ABC);
        let (spi, cs) = adc.release();
        assert_eq!(spi.last_tx, [0x06, 0xC0, 0x00]);
        assert!(cs.high);
    }
}

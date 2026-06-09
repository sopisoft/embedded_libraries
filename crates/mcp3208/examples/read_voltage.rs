use core::convert::Infallible;

use embedded_hal::{
    digital::OutputPin,
    spi::{ErrorType as SpiErrorType, SpiBus},
};
use mcp3208::{Channel, Mcp3208};

// This example is intentionally written with mock SPI and CS types so it can run on a desktop.
// On real hardware, replace `MockSpi` and `MockCs` with the SPI bus and GPIO pin from your HAL.

#[derive(Debug)]
struct MockSpi {
    response: [u8; 3],
}

impl SpiErrorType for MockSpi {
    type Error = Infallible;
}

impl SpiBus<u8> for MockSpi {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let len = words.len().min(self.response.len());
        words[..len].copy_from_slice(&self.response[..len]);
        Ok(())
    }

    fn write(&mut self, _words: &[u8]) -> Result<(), Self::Error> {
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
        self.read(read)
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let len = words.len().min(self.response.len());
        words[..len].copy_from_slice(&self.response[..len]);
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[derive(Debug)]
struct MockCs;

impl embedded_hal::digital::ErrorType for MockCs {
    type Error = Infallible;
}

impl OutputPin for MockCs {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

fn main() {
    // The MCP3208 returns 12-bit samples.
    // Here we fake a mid-scale reading of 0x0800, which is close to half of 3.3 V.
    let spi = MockSpi {
        response: [0x00, 0x08, 0x00],
    };
    let cs = MockCs;
    let mut adc = Mcp3208::new(spi, cs);

    // Step 1: read the raw 12-bit code.
    let raw = adc.read_raw(Channel::SingleEnded(0)).unwrap();

    // Step 2: convert the same channel into engineering units.
    //
    // If your board uses a different ADC reference voltage, pass that value instead.
    let millivolts = adc.read_voltage_mv(Channel::SingleEnded(0), 3300).unwrap();
    let normalized = adc.read_normalized(Channel::SingleEnded(0)).unwrap();

    println!("ADC channel: CH0 single-ended");
    println!("Raw ADC value: {raw} / 4095");
    println!("Normalized reading: {:.3}", normalized);
    println!("Measured voltage with 3.3 V reference: {millivolts} mV");
}

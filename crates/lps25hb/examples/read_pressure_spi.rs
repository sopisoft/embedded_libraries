// This host-runnable example mirrors the I2C getting-started example, but it
// uses the SPI transport wrapper instead.
//
// This is useful when your target board already has a spare SPI bus and you
// want to keep the I2C bus free for IMUs, magnetometers, or other sensors.
//
// On real hardware:
// - connect SPC to your SPI clock
// - connect SDI to your MOSI
// - connect SDO to your MISO
// - connect CS to a GPIO chip-select pin
// - keep the Akizuki module in SPI mode and do not use the I2C pull-up
//   jumpers for the SPI bus

use core::convert::Infallible;

use embedded_hal::{
    digital::{ErrorType as DigitalErrorType, OutputPin},
    spi::{ErrorType as SpiErrorType, SpiBus},
};
use lps25hb::{
    Config, DEVICE_ID, Lps25hb, STANDARD_SEA_LEVEL_PRESSURE_HPA, pressure_to_altitude_m,
};

const WHO_AM_I: u8 = 0x0F;
const PRESS_OUT_XL: u8 = 0x28;
const TEMP_OUT_L: u8 = 0x2B;

#[derive(Debug)]
struct FakeSpi {
    regs: [u8; 256],
    pending_register: u8,
    auto_increment: bool,
}

impl FakeSpi {
    fn from_environment(pressure_hpa: f32, temperature_c: f32) -> Self {
        let mut regs = [0u8; 256];
        regs[WHO_AM_I as usize] = DEVICE_ID;

        let pressure_raw = (pressure_hpa * 4096.0) as i32;
        let pressure_bytes = pressure_raw.to_le_bytes();
        regs[PRESS_OUT_XL as usize] = pressure_bytes[0];
        regs[(PRESS_OUT_XL + 1) as usize] = pressure_bytes[1];
        regs[(PRESS_OUT_XL + 2) as usize] = pressure_bytes[2];

        let temperature_raw = ((temperature_c - 42.5) * 480.0) as i16;
        let temperature_bytes = temperature_raw.to_le_bytes();
        regs[TEMP_OUT_L as usize] = temperature_bytes[0];
        regs[(TEMP_OUT_L + 1) as usize] = temperature_bytes[1];

        Self {
            regs,
            pending_register: 0,
            auto_increment: false,
        }
    }
}

impl SpiErrorType for FakeSpi {
    type Error = Infallible;
}

impl SpiBus<u8> for FakeSpi {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut index = 0usize;
        while index < words.len() {
            words[index] = self.regs[self.pending_register as usize];
            if self.auto_increment {
                self.pending_register = self.pending_register.wrapping_add(1);
            }
            index += 1;
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        if words.is_empty() {
            return Ok(());
        }

        if words.len() == 1 {
            self.pending_register = words[0] >> 2;
            self.auto_increment = (words[0] & 0b10) != 0;
            return Ok(());
        }

        let header = words[0];
        let register = header >> 2;
        let is_read = (header & 0b1) != 0;
        let auto_increment = (header & 0b10) != 0;

        if is_read {
            self.pending_register = register;
            self.auto_increment = auto_increment;
            return Ok(());
        }

        let mut index = 1usize;
        let mut reg = register;
        while index < words.len() {
            self.regs[reg as usize] = words[index];
            if auto_increment {
                reg = reg.wrapping_add(1);
            }
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
struct FakeCs;

impl DigitalErrorType for FakeCs {
    type Error = Infallible;
}

impl OutputPin for FakeCs {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

fn main() {
    let spi = FakeSpi::from_environment(1008.4, 23.5);
    let cs = FakeCs;
    let mut barometer = Lps25hb::new_spi(spi, cs);

    barometer.init(Config::default()).unwrap();
    let measurement = barometer.read_measurement().unwrap();
    let altitude_m =
        pressure_to_altitude_m(measurement.pressure_hpa, STANDARD_SEA_LEVEL_PRESSURE_HPA);

    println!("LPS25HB basic SPI reading");
    println!("  pressure    : {:7.2} hPa", measurement.pressure_hpa);
    println!("  temperature : {:7.2} C", measurement.temperature_c);
    println!("  altitude    : {:7.2} m", altitude_m);
}

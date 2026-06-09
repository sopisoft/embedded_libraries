// This example is intentionally host-runnable so a beginner can follow the
// full flow without hardware first.
//
// It uses a tiny fake I2C device that exposes the same register interface as
// the real sensor. The important part is not the mock itself, but the usage
// pattern:
//
// 1. create the driver with the correct I2C address,
// 2. initialize the sensor,
// 3. read pressure and temperature,
// 4. convert pressure to altitude,
// 5. optionally compute an RPDS offset from a trusted pressure reference.
//
// On a real board, replace `FakeI2c` with your HAL I2C peripheral.

use embedded_hal::i2c::{Error as HalError, ErrorKind, ErrorType, I2c, Operation};
use lps25hb::{
    Address, Config, DEVICE_ID, Lps25hb, STANDARD_SEA_LEVEL_PRESSURE_HPA,
    one_point_calibration_rpds, pressure_to_altitude_m,
};

const WHO_AM_I: u8 = 0x0F;
const PRESS_OUT_XL: u8 = 0x28;
const TEMP_OUT_L: u8 = 0x2B;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
struct FakeError;

impl HalError for FakeError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

#[derive(Debug)]
struct FakeI2c {
    regs: [u8; 256],
}

impl FakeI2c {
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

        Self { regs }
    }
}

impl ErrorType for FakeI2c {
    type Error = FakeError;
}

impl I2c for FakeI2c {
    fn read(&mut self, _address: u8, _read: &mut [u8]) -> Result<(), Self::Error> {
        unreachable!("this example only uses write_read based register access")
    }

    fn write(&mut self, _address: u8, write: &[u8]) -> Result<(), Self::Error> {
        if write.len() == 2 {
            self.regs[write[0] as usize] = write[1];
        } else if write.len() == 3 {
            let start = (write[0] & 0x7F) as usize;
            self.regs[start] = write[1];
            self.regs[start + 1] = write[2];
        }
        Ok(())
    }

    fn write_read(
        &mut self,
        _address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        let start = (write[0] & 0x7F) as usize;
        let auto_increment = (write[0] & 0x80) != 0;
        let mut index = 0usize;
        while index < read.len() {
            read[index] = self.regs[start + if auto_increment { index } else { 0 }];
            index += 1;
        }
        Ok(())
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        for operation in operations {
            match operation {
                Operation::Read(buffer) => self.read(address, buffer)?,
                Operation::Write(buffer) => self.write(address, buffer)?,
            }
        }
        Ok(())
    }
}

fn main() {
    // Example environment:
    // - pressure slightly below standard sea-level pressure
    // - room temperature
    let i2c = FakeI2c::from_environment(1006.8, 24.0);
    let mut barometer = Lps25hb::new_i2c(i2c, Address::Addr5c);

    barometer.init(Config::default()).unwrap();
    let measurement = barometer.read_measurement().unwrap();

    // If you know the local QNH or have a trusted reference station nearby,
    // use that as the sea-level pressure. Here we use the ISA standard value.
    let altitude_m =
        pressure_to_altitude_m(measurement.pressure_hpa, STANDARD_SEA_LEVEL_PRESSURE_HPA);

    println!("LPS25HB basic reading");
    println!("  pressure    : {:7.2} hPa", measurement.pressure_hpa);
    println!("  temperature : {:7.2} C", measurement.temperature_c);
    println!("  altitude    : {:7.2} m", altitude_m);

    // The Akizuki appendix describes one-point calibration with the RPDS
    // register. Imagine you trust a weather station that reports 1008.2 hPa.
    let rpds = one_point_calibration_rpds(measurement.pressure_hpa, 1008.2);
    println!("  rpds offset : {:7} counts", rpds);
}

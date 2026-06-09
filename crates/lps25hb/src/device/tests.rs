use embedded_hal::i2c::{Error as HalError, ErrorKind, ErrorType, I2c, Operation};
use libm::fabsf;

use crate::STANDARD_SEA_LEVEL_PRESSURE_HPA;
use crate::registers::{CTRL_REG1, PRESS_OUT_XL, RES_CONF, WHO_AM_I};
use crate::spi::SpiBusError;
use crate::{
    Address, Config, DEVICE_ID, Lps25hb, altitude_to_pressure_hpa, one_point_calibration_rpds,
    pressure_to_altitude_m, raw_pressure_to_hpa, raw_temperature_to_celsius,
    rpds_counts_to_pressure_hpa,
};

const TEMP_OUT_L: u8 = 0x2B;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
struct MockError;

impl HalError for MockError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

#[derive(Debug)]
struct MockI2c {
    regs: [u8; 256],
}

impl MockI2c {
    fn new() -> Self {
        let mut regs = [0u8; 256];
        regs[WHO_AM_I as usize] = DEVICE_ID;
        Self { regs }
    }
}

impl ErrorType for MockI2c {
    type Error = MockError;
}

impl I2c for MockI2c {
    fn read(&mut self, _address: u8, _read: &mut [u8]) -> Result<(), Self::Error> {
        unreachable!("driver only uses write_read for register reads")
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

fn approx(a: f32, b: f32) -> bool {
    fabsf(a - b) < 1.0e-3
}

#[test]
fn init_programs_expected_registers() {
    let i2c = MockI2c::new();
    let mut sensor = Lps25hb::new_i2c(i2c, Address::Addr5c);
    sensor.init(Config::default()).unwrap();
    let i2c = sensor.destroy();

    assert_eq!(i2c.regs[RES_CONF as usize], 0x0F);
    assert_eq!(i2c.regs[CTRL_REG1 as usize], 0x94);
}

#[test]
fn raw_measurement_converts_to_units() {
    let mut i2c = MockI2c::new();
    let pressure_raw = (1013.25 * 4096.0) as i32;
    let pressure_bytes = pressure_raw.to_le_bytes();
    i2c.regs[PRESS_OUT_XL as usize] = pressure_bytes[0];
    i2c.regs[(PRESS_OUT_XL + 1) as usize] = pressure_bytes[1];
    i2c.regs[(PRESS_OUT_XL + 2) as usize] = pressure_bytes[2];

    let temperature_raw = ((25.0 - 42.5) * 480.0) as i16;
    let temp_bytes = temperature_raw.to_le_bytes();
    i2c.regs[TEMP_OUT_L as usize] = temp_bytes[0];
    i2c.regs[(TEMP_OUT_L + 1) as usize] = temp_bytes[1];

    let mut sensor = Lps25hb::new_i2c(i2c, Address::Addr5d);
    let measurement = sensor.read_measurement().unwrap();
    assert!(approx(measurement.pressure_hpa, 1013.25));
    assert!(approx(measurement.temperature_c, 25.0));
    assert!(approx(raw_pressure_to_hpa(pressure_raw), 1013.25));
    assert!(approx(raw_temperature_to_celsius(temperature_raw), 25.0));
}

#[test]
fn rpds_helper_matches_appendix_formula() {
    let counts = one_point_calibration_rpds(1018.0, 1013.5);
    assert_eq!(counts, 72);
    assert!(approx(rpds_counts_to_pressure_hpa(counts), 4.5));
}

#[test]
fn altitude_helper_is_zero_at_sea_level_reference() {
    let altitude = pressure_to_altitude_m(
        STANDARD_SEA_LEVEL_PRESSURE_HPA,
        STANDARD_SEA_LEVEL_PRESSURE_HPA,
    );
    assert!(approx(altitude, 0.0));
    assert!(approx(
        altitude_to_pressure_hpa(altitude, STANDARD_SEA_LEVEL_PRESSURE_HPA),
        STANDARD_SEA_LEVEL_PRESSURE_HPA,
    ));
}

#[test]
fn spi_error_type_is_publicly_reachable() {
    let _: Option<SpiBusError<(), ()>> = None;
}

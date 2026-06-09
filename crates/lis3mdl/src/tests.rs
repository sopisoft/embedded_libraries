use embedded_hal::i2c::{Error as HalError, ErrorKind, ErrorType, I2c, Operation};

use crate::registers::{
    CTRL_REG1, CTRL_REG2, CTRL_REG3, CTRL_REG4, CTRL_REG5, OUT_X_H, OUT_X_L, OUT_Y_H, OUT_Y_L,
    OUT_Z_H, OUT_Z_L, STATUS_REG, WHO_AM_I,
};
use crate::{Address, Config, DEVICE_ID, FullScale, Lis3mdl};

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
        }
        Ok(())
    }

    fn write_read(
        &mut self,
        _address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        read[0] = self.regs[write[0] as usize];
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

#[test]
fn init_programs_default_registers() {
    let i2c = MockI2c::new();
    let mut lis3mdl = Lis3mdl::new(i2c, Address::Addr1c);
    lis3mdl.init(Config::default()).unwrap();
    let i2c = lis3mdl.destroy();

    assert_eq!(i2c.regs[CTRL_REG1 as usize], 0xE2);
    assert_eq!(i2c.regs[CTRL_REG2 as usize], 0x40);
    assert_eq!(i2c.regs[CTRL_REG3 as usize], 0x00);
    assert_eq!(i2c.regs[CTRL_REG4 as usize], 0x0C);
    assert_eq!(i2c.regs[CTRL_REG5 as usize], 0x40);
}

#[test]
fn read_magnetic_mgauss_uses_selected_scale() {
    let mut i2c = MockI2c::new();
    i2c.regs[OUT_X_L as usize] = 0xBA;
    i2c.regs[OUT_X_H as usize] = 0x1A;
    i2c.regs[OUT_Y_L as usize] = 0;
    i2c.regs[OUT_Y_H as usize] = 0;
    i2c.regs[OUT_Z_L as usize] = 0;
    i2c.regs[OUT_Z_H as usize] = 0;

    let mut lis3mdl = Lis3mdl::new(i2c, Address::Addr1c);
    lis3mdl
        .init(Config {
            full_scale: FullScale::Gauss4,
            ..Config::default()
        })
        .unwrap();

    let field = lis3mdl.read_magnetic_mgauss().unwrap();
    assert!((field.x_mgauss - 1_000.0).abs() < 1.0);
}

#[test]
fn data_ready_reads_status_register() {
    let mut i2c = MockI2c::new();
    i2c.regs[STATUS_REG as usize] = 0b0000_1000;
    let mut lis3mdl = Lis3mdl::new(i2c, Address::Addr1e);
    assert!(lis3mdl.magnetic_data_ready().unwrap());
}

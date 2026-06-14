use lis3mdl::{Address, Config, Lis3mdl};

fn main() {
    // This host example uses a tiny fake I2C bus so you can understand the API
    // without owning the hardware yet.
    //
    // On real hardware you would replace `MockI2c` with the I2C peripheral from
    // your HAL and keep the driver calls exactly the same.
    let mut i2c = MockI2c::new();

    // Pretend the sensor already contains one sample:
    // X = 2281 counts, Y = 0, Z = -2281 counts.
    i2c.set_reg(0x28, 0xE9);
    i2c.set_reg(0x29, 0x08);
    i2c.set_reg(0x2A, 0x00);
    i2c.set_reg(0x2B, 0x00);
    i2c.set_reg(0x2C, 0x17);
    i2c.set_reg(0x2D, 0xF7);

    let mut magnetometer = Lis3mdl::new(i2c, Address::Addr1c);
    magnetometer.init(Config::default()).unwrap();

    let raw = magnetometer.read_raw_magnetic().unwrap();
    println!("Raw counts: x={} y={} z={}", raw.x, raw.y, raw.z);

    let field = magnetometer.read_magnetic_mgauss().unwrap();
    println!(
        "Field [mGauss]: x={:.1} y={:.1} z={:.1}",
        field.x_mgauss, field.y_mgauss, field.z_mgauss
    );

    // In a real AHRS pipeline you would usually convert those values into the
    // vector type used by your estimator and feed them into a fusion filter.
}

use core::convert::Infallible;
use embedded_hal::i2c::{ErrorType, I2c, Operation};

#[derive(Debug)]
struct MockI2c {
    regs: [u8; 256],
}

impl MockI2c {
    fn new() -> Self {
        let mut regs = [0u8; 256];
        regs[0x0F] = 0x3D;
        Self { regs }
    }

    fn set_reg(&mut self, register: u8, value: u8) {
        self.regs[register as usize] = value;
    }
}

impl ErrorType for MockI2c {
    type Error = Infallible;
}

impl I2c for MockI2c {
    fn read(&mut self, _address: u8, _read: &mut [u8]) -> Result<(), Self::Error> {
        unreachable!("the driver uses write_read for register reads")
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

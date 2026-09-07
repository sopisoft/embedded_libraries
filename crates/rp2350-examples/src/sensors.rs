use core::cell::RefCell;

use embedded_hal::i2c::I2c;
use imu::{
    AccelGyroSample, Acceleration, AngularVelocity, MagneticField, MargSample, SharedI2c, Vec3,
};
use lis3mdl::{Address as Lis3mdlAddress, Config as Lis3mdlConfig, Lis3mdl};
use lps25hb::{Address as Lps25hbAddress, Config as Lps25hbConfig, Lps25hb};
use lsm6ds3tr::{
    AccelSampleRate, AccelScale, AccelSettings, GyroSettings, LSM6DS3TR, LsmSettings,
    interface::Interface,
};

pub const DEFAULT_LSM6DS3TR_ADDRESS: u8 = 0x6A;
pub const DEFAULT_LIS3MDL_ADDRESS: Lis3mdlAddress = Lis3mdlAddress::Addr1c;
pub const DEFAULT_LPS25HB_ADDRESS: Lps25hbAddress = Lps25hbAddress::Addr5c;

#[derive(Debug)]
pub enum SensorError<E> {
    AccelGyro(E),
    Magnetometer(lis3mdl::Error<E>),
    Barometer(lps25hb::Error<E>),
    InvalidBarometricAltitude,
}

pub struct Lsm6ds3trI2c<BUS> {
    bus: BUS,
    address: u8,
}

impl<BUS> Lsm6ds3trI2c<BUS> {
    pub const fn new(bus: BUS, address: u8) -> Self {
        Self { bus, address }
    }
}

impl<BUS: I2c> Interface for Lsm6ds3trI2c<BUS> {
    type Error = BUS::Error;

    fn write(&mut self, register: u8, value: u8) -> Result<(), Self::Error> {
        self.bus.write(self.address, &[register, value])
    }

    fn read(&mut self, register: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.write_read(self.address, &[register], buffer)
    }
}

type AccelGyro<'a, BUS> = LSM6DS3TR<Lsm6ds3trI2c<SharedI2c<'a, BUS>>>;
type Magnetometer<'a, BUS> = Lis3mdl<SharedI2c<'a, BUS>>;
type Barometer<'a, BUS> = Lps25hb<lps25hb::i2c::I2cInterface<SharedI2c<'a, BUS>>>;

pub struct Sensors<'a, BUS: I2c> {
    accel_gyro: AccelGyro<'a, BUS>,
    magnetometer: Magnetometer<'a, BUS>,
    barometer: Barometer<'a, BUS>,
}

impl<'a, BUS: I2c> Sensors<'a, BUS> {
    pub fn with_addresses(
        bus: &'a RefCell<BUS>,
        accel_gyro_address: u8,
        magnetometer_address: Lis3mdlAddress,
        barometer_address: Lps25hbAddress,
    ) -> Self {
        let settings = LsmSettings::basic()
            .with_accel(
                AccelSettings::new()
                    .with_sample_rate(AccelSampleRate::_104Hz)
                    .with_scale(AccelScale::_4G),
            )
            .with_gyro(GyroSettings::new());
        Self {
            accel_gyro: LSM6DS3TR::new(Lsm6ds3trI2c::new(SharedI2c::new(bus), accel_gyro_address))
                .with_settings(settings),
            magnetometer: Lis3mdl::new(SharedI2c::new(bus), magnetometer_address),
            barometer: Lps25hb::new_i2c(SharedI2c::new(bus), barometer_address),
        }
    }

    pub fn init(&mut self) -> Result<(), SensorError<BUS::Error>> {
        self.accel_gyro.init().map_err(SensorError::AccelGyro)?;
        self.magnetometer
            .init(Lis3mdlConfig::default())
            .map_err(SensorError::Magnetometer)?;
        self.barometer
            .init(Lps25hbConfig::default_continuous())
            .map_err(SensorError::Barometer)
    }

    pub fn read_marg(&mut self) -> Result<MargSample, SensorError<BUS::Error>> {
        let accel = self
            .accel_gyro
            .read_accel()
            .map_err(SensorError::AccelGyro)?;
        let gyro = self
            .accel_gyro
            .read_gyro()
            .map_err(SensorError::AccelGyro)?;
        let mag = self
            .magnetometer
            .read_magnetic_mgauss()
            .map_err(SensorError::Magnetometer)?;
        Ok(MargSample::new(
            AccelGyroSample::without_temperature(
                Acceleration::new(Vec3::new(accel.x, accel.y, accel.z) * 9.80665),
                AngularVelocity::new(Vec3::new(
                    gyro.x.to_radians(),
                    gyro.y.to_radians(),
                    gyro.z.to_radians(),
                )),
            ),
            MagneticField::new(Vec3::new(mag.x_mgauss, mag.y_mgauss, mag.z_mgauss)),
        ))
    }

    pub fn barometric_altitude(&mut self) -> Result<Option<f32>, SensorError<BUS::Error>> {
        if !self
            .barometer
            .pressure_data_ready()
            .map_err(SensorError::Barometer)?
        {
            return Ok(None);
        }

        let measurement = self
            .barometer
            .read_measurement()
            .map_err(SensorError::Barometer)?;
        let altitude = lps25hb::pressure_to_altitude_m(
            measurement.pressure_hpa,
            lps25hb::STANDARD_SEA_LEVEL_PRESSURE_HPA,
        );
        altitude
            .is_finite()
            .then_some(Some(altitude))
            .ok_or(SensorError::InvalidBarometricAltitude)
    }
}

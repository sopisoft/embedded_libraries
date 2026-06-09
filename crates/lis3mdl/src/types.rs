/// Expected LIS3MDL `WHO_AM_I` value.
pub const DEVICE_ID: u8 = 0x3D;

/// I2C address selection.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Address {
    /// SA1 low: `0x1C`
    Addr1c = 0x1C,
    /// SA1 high: `0x1E`
    Addr1e = 0x1E,
}

impl Address {
    /// Returns the 7-bit I2C address.
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// Magnetic full-scale range.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FullScale {
    Gauss4,
    Gauss8,
    Gauss12,
    Gauss16,
}

impl FullScale {
    pub(crate) const fn register_bits(self) -> u8 {
        match self {
            Self::Gauss4 => 0b00 << 5,
            Self::Gauss8 => 0b01 << 5,
            Self::Gauss12 => 0b10 << 5,
            Self::Gauss16 => 0b11 << 5,
        }
    }

    pub(crate) fn sensitivity_mgauss_per_lsb(self) -> f32 {
        match self {
            Self::Gauss4 => 1_000.0 / 6_842.0,
            Self::Gauss8 => 1_000.0 / 3_421.0,
            Self::Gauss12 => 1_000.0 / 2_281.0,
            Self::Gauss16 => 1_000.0 / 1_711.0,
        }
    }
}

/// Operating mode for the X/Y/Z magnetic channels.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum OperatingMode {
    LowPower,
    MediumPerformance,
    HighPerformance,
    UltraHighPerformance,
}

impl OperatingMode {
    pub(crate) const fn xy_bits(self) -> u8 {
        match self {
            Self::LowPower => 0,
            Self::MediumPerformance => 0b01 << 5,
            Self::HighPerformance => 0b10 << 5,
            Self::UltraHighPerformance => 0b11 << 5,
        }
    }

    pub(crate) const fn z_bits(self) -> u8 {
        match self {
            Self::LowPower => 0,
            Self::MediumPerformance => 0b01 << 2,
            Self::HighPerformance => 0b10 << 2,
            Self::UltraHighPerformance => 0b11 << 2,
        }
    }
}

/// Magnetic measurement state.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MeasurementMode {
    PowerDown,
    Single,
    Continuous,
}

impl MeasurementMode {
    pub(crate) const fn register_bits(self) -> u8 {
        match self {
            Self::PowerDown => 0b10,
            Self::Single => 0b01,
            Self::Continuous => 0b00,
        }
    }
}

/// Output data rate selection.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DataRate {
    Hz0_625,
    Hz1_25,
    Hz2_5,
    Hz5,
    Hz10,
    Hz20,
    Hz40,
    Hz80,
    Fast,
}

impl DataRate {
    pub(crate) const fn register_bits(self) -> u8 {
        match self {
            Self::Hz0_625 => 0,
            Self::Hz1_25 => 0b001 << 2,
            Self::Hz2_5 => 0b010 << 2,
            Self::Hz5 => 0b011 << 2,
            Self::Hz10 => 0b100 << 2,
            Self::Hz20 => 0b101 << 2,
            Self::Hz40 => 0b110 << 2,
            Self::Hz80 => 0b111 << 2,
            Self::Fast => 0b0000_0010,
        }
    }
}

/// Driver configuration.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Config {
    pub full_scale: FullScale,
    pub operating_mode: OperatingMode,
    pub measurement_mode: MeasurementMode,
    pub data_rate: DataRate,
    pub block_data_update: bool,
    pub temperature_enable: bool,
}

impl Config {
    /// Returns the practical default used for a high-quality continuous stream.
    pub const fn default_continuous() -> Self {
        Self {
            full_scale: FullScale::Gauss12,
            operating_mode: OperatingMode::UltraHighPerformance,
            measurement_mode: MeasurementMode::Continuous,
            data_rate: DataRate::Fast,
            block_data_update: true,
            temperature_enable: true,
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::default_continuous()
    }
}

/// Raw signed magnetometer output in sensor counts.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct RawMagneticField {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Magnetic field converted to milli-gauss.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct MagneticField {
    pub x_mgauss: f32,
    pub y_mgauss: f32,
    pub z_mgauss: f32,
}

/// Driver error.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Error<E> {
    Bus(E),
    InvalidDeviceId(u8),
}

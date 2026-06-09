/// Expected `WHO_AM_I` register value.
pub const DEVICE_ID: u8 = 0xBD;

/// Standard sea-level pressure in hPa.
pub const STANDARD_SEA_LEVEL_PRESSURE_HPA: f32 = 1013.25;

/// I2C address selection via `SA0`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Address {
    /// `SA0` low.
    Addr5c = 0x5C,
    /// `SA0` high.
    Addr5d = 0x5D,
}

impl Address {
    /// Returns the 7-bit I2C address.
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// Pressure internal averaging.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PressureAverage {
    Avg8,
    Avg32,
    Avg128,
    Avg512,
}

impl PressureAverage {
    pub(crate) const fn register_bits(self) -> u8 {
        match self {
            Self::Avg8 => 0b00,
            Self::Avg32 => 0b01,
            Self::Avg128 => 0b10,
            Self::Avg512 => 0b11,
        }
    }
}

/// Temperature internal averaging.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TemperatureAverage {
    Avg8,
    Avg16,
    Avg32,
    Avg64,
}

impl TemperatureAverage {
    pub(crate) const fn register_bits(self) -> u8 {
        match self {
            Self::Avg8 => 0b00 << 2,
            Self::Avg16 => 0b01 << 2,
            Self::Avg32 => 0b10 << 2,
            Self::Avg64 => 0b11 << 2,
        }
    }
}

/// Output data rate for continuous measurement.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum OutputDataRate {
    Hz1,
    Hz7,
    Hz12_5,
    Hz25,
}

impl OutputDataRate {
    pub(crate) const fn register_bits(self) -> u8 {
        match self {
            Self::Hz1 => 0b001 << 4,
            Self::Hz7 => 0b010 << 4,
            Self::Hz12_5 => 0b011 << 4,
            Self::Hz25 => 0b100 << 4,
        }
    }
}

/// Driver configuration.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Config {
    pub output_data_rate: OutputDataRate,
    pub pressure_average: PressureAverage,
    pub temperature_average: TemperatureAverage,
    pub block_data_update: bool,
    pub differential_output: bool,
}

impl Config {
    /// Practical continuous mode for embedded altitude and weather logging.
    pub const fn default_continuous() -> Self {
        Self {
            output_data_rate: OutputDataRate::Hz1,
            pressure_average: PressureAverage::Avg512,
            temperature_average: TemperatureAverage::Avg64,
            block_data_update: true,
            differential_output: false,
        }
    }

    /// Close to the simple Akizuki manual setup, but still enables BDU because
    /// it makes multi-byte reads safer.
    pub const fn akizuki_style() -> Self {
        Self {
            output_data_rate: OutputDataRate::Hz1,
            pressure_average: PressureAverage::Avg32,
            temperature_average: TemperatureAverage::Avg16,
            block_data_update: true,
            differential_output: false,
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::default_continuous()
    }
}

/// Status flags from `STATUS_REG`.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct Status {
    pub pressure_ready: bool,
    pub temperature_ready: bool,
    pub pressure_overrun: bool,
    pub temperature_overrun: bool,
}

/// Raw output register contents.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct RawMeasurement {
    pub pressure_raw: i32,
    pub temperature_raw: i16,
}

/// Pressure and temperature converted to engineering units.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Measurement {
    pub pressure_hpa: f32,
    pub temperature_c: f32,
}

/// Driver error.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Error<E> {
    Bus(E),
    InvalidDeviceId(u8),
}

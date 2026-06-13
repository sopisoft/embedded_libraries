use crate::Error;

/// ADC front-end description for the analog MIS-2500 output.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AdcConfig {
    /// Largest ADC code. For a 12-bit ADC this is usually `4095`.
    pub max_code: u16,
    /// ADC reference voltage in volts.
    pub reference_voltage_v: f32,
    /// Multiplier to recover the original sensor voltage after a divider.
    pub input_scale: f32,
}

impl AdcConfig {
    /// Creates an ADC configuration with direct input scaling.
    pub const fn new(max_code: u16, reference_voltage_v: f32) -> Self {
        Self {
            max_code,
            reference_voltage_v,
            input_scale: 1.0,
        }
    }

    /// Applies an input scaling factor.
    pub const fn with_input_scale(mut self, input_scale: f32) -> Self {
        self.input_scale = input_scale;
        self
    }

    /// Converts a raw ADC code into the ADC pin voltage.
    pub fn input_voltage_from_code(self, raw_code: u16) -> Result<f32, Error> {
        if self.max_code == 0 {
            return Err(Error::InvalidAdcRange);
        }
        if self.reference_voltage_v <= 0.0 {
            return Err(Error::InvalidReferenceVoltage);
        }
        Ok(raw_code as f32 * self.reference_voltage_v / self.max_code as f32)
    }

    /// Converts a raw ADC code into the actual sensor output voltage.
    pub fn sensor_output_voltage_from_code(self, raw_code: u16) -> Result<f32, Error> {
        if self.input_scale <= 0.0 {
            return Err(Error::InvalidInputScale);
        }
        Ok(self.input_voltage_from_code(raw_code)? * self.input_scale)
    }

    pub(crate) fn average_sensor_output_voltage(self, raw_codes: &[u16]) -> Result<f32, Error> {
        if raw_codes.is_empty() {
            return Err(Error::EmptySamples);
        }
        let mut sum = 0.0;
        let mut index = 0usize;
        while index < raw_codes.len() {
            sum += self.sensor_output_voltage_from_code(raw_codes[index])?;
            index += 1;
        }
        Ok(sum / raw_codes.len() as f32)
    }
}

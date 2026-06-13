use crate::{AdcConfig, Error, Pressure};

const PSI_TO_PA: f32 = 6_894.757_3;
const MBAR_TO_PA: f32 = 100.0;

/// Nominal supply voltage for the 5 V MIS-2500 family.
pub const NOMINAL_SUPPLY_VOLTAGE_V: f32 = 5.0;
/// Typical zero-pressure output ratio `Voff / Vdd`.
pub const TYPICAL_ZERO_OUTPUT_RATIO: f32 = 0.05;
/// Typical output span ratio `(Vfs - Voff) / Vdd`.
pub const OUTPUT_SPAN_RATIO: f32 = 0.90;
/// Full-scale pressure of the MIS-2500-015G.
pub const FULL_SCALE_PRESSURE_015G_PA: f32 = 15.0 * PSI_TO_PA;
/// Full-scale pressure of the MIS-2500-015V.
pub const FULL_SCALE_PRESSURE_015V_PA: f32 = -1_000.0 * MBAR_TO_PA;

/// Analog MIS-2500 driver model.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Mis2500 {
    zero_output_ratio: f32,
    full_scale_pressure_pa: f32,
}

impl Mis2500 {
    /// Creates a model for the MIS-2500-015G gauge sensor.
    pub const fn mis2500_015g() -> Self {
        Self::new(FULL_SCALE_PRESSURE_015G_PA)
    }

    /// Creates a model for the MIS-2500-015V vacuum sensor.
    pub const fn mis2500_015v() -> Self {
        Self::new(FULL_SCALE_PRESSURE_015V_PA)
    }

    /// Creates a sensor model from a full-scale pressure in pascals.
    pub const fn new(full_scale_pressure_pa: f32) -> Self {
        Self {
            zero_output_ratio: TYPICAL_ZERO_OUTPUT_RATIO,
            full_scale_pressure_pa,
        }
    }

    /// Overrides the zero-pressure output ratio.
    pub const fn with_zero_output_ratio(mut self, zero_output_ratio: f32) -> Self {
        self.zero_output_ratio = zero_output_ratio;
        self
    }

    /// Returns the configured zero-pressure output ratio.
    pub const fn zero_output_ratio(&self) -> f32 {
        self.zero_output_ratio
    }

    /// Converts a measured output ratio into pressure.
    pub fn pressure_from_output_ratio(&self, output_ratio: f32) -> Pressure {
        let pa = ((output_ratio - self.zero_output_ratio) / OUTPUT_SPAN_RATIO)
            * self.full_scale_pressure_pa;
        Pressure::from_pa(pa)
    }

    /// Converts sensor output voltage into pressure.
    pub fn pressure_from_output_voltage(
        &self,
        output_voltage_v: f32,
        supply_voltage_v: f32,
    ) -> Result<Pressure, Error> {
        if supply_voltage_v <= 0.0 {
            return Err(Error::InvalidSupplyVoltage);
        }
        Ok(self.pressure_from_output_ratio(output_voltage_v / supply_voltage_v))
    }

    /// Converts a raw ADC code into pressure.
    pub fn pressure_from_adc_code(
        &self,
        raw_code: u16,
        adc: AdcConfig,
        supply_voltage_v: f32,
    ) -> Result<Pressure, Error> {
        let vout = adc.sensor_output_voltage_from_code(raw_code)?;
        self.pressure_from_output_voltage(vout, supply_voltage_v)
    }

    /// Updates the zero-pressure point from output-ratio samples.
    pub fn calibrate_zero_from_output_ratios(
        &mut self,
        output_ratios: &[f32],
    ) -> Result<f32, Error> {
        if output_ratios.is_empty() {
            return Err(Error::EmptySamples);
        }
        let mut sum = 0.0;
        let mut index = 0usize;
        while index < output_ratios.len() {
            sum += output_ratios[index];
            index += 1;
        }
        self.zero_output_ratio = sum / output_ratios.len() as f32;
        Ok(self.zero_output_ratio)
    }

    /// Updates the zero-pressure point from ADC samples.
    pub fn calibrate_zero_from_adc_codes(
        &mut self,
        raw_codes: &[u16],
        adc: AdcConfig,
        supply_voltage_v: f32,
    ) -> Result<f32, Error> {
        if supply_voltage_v <= 0.0 {
            return Err(Error::InvalidSupplyVoltage);
        }
        let avg_vout = adc.average_sensor_output_voltage(raw_codes)?;
        self.zero_output_ratio = avg_vout / supply_voltage_v;
        Ok(self.zero_output_ratio)
    }
}

impl Default for Mis2500 {
    fn default() -> Self {
        Self::mis2500_015g()
    }
}

/// Driver error.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Error {
    /// The sensor supply voltage must be positive.
    InvalidSupplyVoltage,
    /// The ADC reference voltage must be positive.
    InvalidReferenceVoltage,
    /// The ADC full-scale code must be non-zero.
    InvalidAdcRange,
    /// The ADC input scale must be positive.
    InvalidInputScale,
    /// Calibration requires at least one sample.
    EmptySamples,
}

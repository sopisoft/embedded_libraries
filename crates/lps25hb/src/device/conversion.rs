use libm::{powf, roundf};

/// Converts raw pressure counts to hPa.
pub fn raw_pressure_to_hpa(raw_pressure: i32) -> f32 {
    raw_pressure as f32 / 4096.0
}

/// Converts raw temperature counts to degrees Celsius.
pub fn raw_temperature_to_celsius(raw_temperature: i16) -> f32 {
    42.5 + raw_temperature as f32 / 480.0
}

/// Converts pressure to barometric altitude in meters.
pub fn pressure_to_altitude_m(pressure_hpa: f32, sea_level_pressure_hpa: f32) -> f32 {
    if pressure_hpa <= 0.0 || sea_level_pressure_hpa <= 0.0 {
        return 0.0;
    }
    44_330.0 * (1.0 - powf(pressure_hpa / sea_level_pressure_hpa, 0.190_294_95))
}

/// Converts altitude to pressure in hPa for a given sea-level reference.
pub fn altitude_to_pressure_hpa(altitude_m: f32, sea_level_pressure_hpa: f32) -> f32 {
    if sea_level_pressure_hpa <= 0.0 {
        return 0.0;
    }
    sea_level_pressure_hpa * powf(1.0 - altitude_m / 44_330.0, 5.255)
}

/// Converts an RPDS register value to the pressure correction it represents.
pub fn rpds_counts_to_pressure_hpa(rpds_counts: i16) -> f32 {
    rpds_counts as f32 / 16.0
}

/// Converts a pressure error in hPa to the RPDS register value.
pub fn pressure_error_to_rpds_counts(pressure_error_hpa: f32) -> i16 {
    let counts = roundf(pressure_error_hpa * 16.0);
    counts.clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

/// Computes the RPDS one-point calibration value from a measured and reference
/// pressure pair.
pub fn one_point_calibration_rpds(measured_pressure_hpa: f32, reference_pressure_hpa: f32) -> i16 {
    pressure_error_to_rpds_counts(measured_pressure_hpa - reference_pressure_hpa)
}

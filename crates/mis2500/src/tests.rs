use crate::{
    AdcConfig, Error, FULL_SCALE_PRESSURE_015G_PA, FULL_SCALE_PRESSURE_015V_PA, Mis2500,
    NOMINAL_SUPPLY_VOLTAGE_V, TYPICAL_ZERO_OUTPUT_RATIO,
};

fn approx(a: f32, b: f32) -> bool {
    (a - b).abs() < 1.0e-3
}

#[test]
fn gauge_zero_and_full_scale_match_datasheet() {
    let sensor = Mis2500::mis2500_015g();
    assert!(approx(
        sensor
            .pressure_from_output_ratio(TYPICAL_ZERO_OUTPUT_RATIO)
            .pa(),
        0.0
    ));
    assert!(approx(
        sensor.pressure_from_output_ratio(0.95).pa(),
        FULL_SCALE_PRESSURE_015G_PA
    ));
}

#[test]
fn vacuum_zero_and_full_scale_match_datasheet() {
    let sensor = Mis2500::mis2500_015v();
    assert!(approx(
        sensor
            .pressure_from_output_ratio(TYPICAL_ZERO_OUTPUT_RATIO)
            .pa(),
        0.0
    ));
    assert!(approx(
        sensor.pressure_from_output_ratio(0.95).pa(),
        FULL_SCALE_PRESSURE_015V_PA
    ));
}

#[test]
fn adc_conversion_supports_input_divider() {
    let adc = AdcConfig::new(4095, 3.3).with_input_scale(1.5);
    let sensor = Mis2500::mis2500_015g();
    let pressure = sensor
        .pressure_from_adc_code(224, adc, NOMINAL_SUPPLY_VOLTAGE_V)
        .unwrap();
    assert!(pressure.pa() > 400.0);
    assert!(pressure.pa() < 520.0);
}

#[test]
fn adc_zero_calibration_updates_offset() {
    let adc = AdcConfig::new(4095, 3.3).with_input_scale(1.5);
    let mut sensor = Mis2500::mis2500_015g();
    let zero_ratio = sensor
        .calibrate_zero_from_adc_codes(
            &[205, 206, 206, 207, 206, 206],
            adc,
            NOMINAL_SUPPLY_VOLTAGE_V,
        )
        .unwrap();
    assert!(zero_ratio > 0.049);
    assert!(zero_ratio < 0.051);
}

#[test]
fn pressure_units_are_consistent() {
    let pressure = Mis2500::mis2500_015v().pressure_from_output_ratio(0.50);
    assert!(approx(pressure.kpa(), pressure.pa() / 1_000.0));
    assert!(approx(pressure.hpa(), pressure.mbar()));
}

#[test]
fn invalid_inputs_return_errors() {
    let adc = AdcConfig::new(0, 3.3);
    assert_eq!(adc.input_voltage_from_code(10), Err(Error::InvalidAdcRange));
    assert_eq!(
        Mis2500::mis2500_015g().pressure_from_output_voltage(1.0, 0.0),
        Err(Error::InvalidSupplyVoltage)
    );
}

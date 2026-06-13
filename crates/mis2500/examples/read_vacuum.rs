use mis2500::{AdcConfig, Mis2500, NOMINAL_SUPPLY_VOLTAGE_V};

fn main() {
    let adc = AdcConfig::new(4095, 3.3).with_input_scale(1.5);
    let mut sensor = Mis2500::mis2500_015v();

    sensor
        .calibrate_zero_from_adc_codes(
            &[205, 206, 206, 207, 206, 206],
            adc,
            NOMINAL_SUPPLY_VOLTAGE_V,
        )
        .unwrap();

    let raw_code = 620u16;
    let pressure = sensor
        .pressure_from_adc_code(raw_code, adc, NOMINAL_SUPPLY_VOLTAGE_V)
        .unwrap();

    println!("MIS-2500-015V vacuum example");
    println!("  ADC raw code : {raw_code}");
    println!("  pressure     : {:7.2} Pa", pressure.pa());
    println!("  pressure     : {:7.2} hPa", pressure.hpa());
    println!("  pressure     : {:7.2} mbar", pressure.mbar());
}

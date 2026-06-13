use mis2500::{AdcConfig, Mis2500, NOMINAL_SUPPLY_VOLTAGE_V};

const AIR_DENSITY_KG_PER_M3: f32 = 1.225;

fn airspeed_mps(dynamic_pressure_pa: f32) -> f32 {
    if dynamic_pressure_pa <= 0.0 {
        0.0
    } else {
        (2.0 * dynamic_pressure_pa / AIR_DENSITY_KG_PER_M3).sqrt()
    }
}

fn main() {
    let adc = AdcConfig::new(4095, 3.3).with_input_scale(1.5);
    let mut sensor = Mis2500::mis2500_015v();

    let zero_ratio = sensor
        .calibrate_zero_from_adc_codes(
            &[205, 206, 206, 207, 206, 206, 205, 206],
            adc,
            NOMINAL_SUPPLY_VOLTAGE_V,
        )
        .unwrap();

    let raw_code = 224u16;
    let pressure = sensor
        .pressure_from_adc_code(raw_code, adc, NOMINAL_SUPPLY_VOLTAGE_V)
        .unwrap();
    let airspeed_mps = airspeed_mps(pressure.pa());

    println!("MIS-2500-015V airspeed example");
    println!("  zero ratio       : {:.5}", zero_ratio);
    println!("  ADC raw code     : {raw_code}");
    println!(
        "  dynamic pressure : {:7.2} Pa ({:.4} psi)",
        pressure.pa(),
        pressure.psi()
    );
    println!(
        "  airspeed         : {:6.2} m/s ({:6.2} km/h)",
        airspeed_mps,
        airspeed_mps * 3.6
    );
}

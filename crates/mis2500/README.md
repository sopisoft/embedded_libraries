# mis2500

`no_std` analog pressure driver for the Akizuki `MIS-2500` series.

## What This Crate Contains

- `Mis2500::mis2500_015g()` for the `MIS-2500-015G` gauge sensor
- `Mis2500::mis2500_015v()` for the `MIS-2500-015V` vacuum sensor
- ADC-to-pressure conversion with configurable input scaling
- zero-offset calibration helpers for analog front-ends

## Sensor Notes

The 5 V MIS-2500 transfer function in the datasheet is:

```text
P = ((Vout - Voff) / 4.5) * Prange
```

This crate applies the same formula to both supported parts:

- `MIS-2500-015G`: `Prange = 15 psi = 103.421 kPa`
- `MIS-2500-015V`: `Prange = -1000 mbar = -100 kPa`
- zero-pressure output is typically `0.25 V`
- full-scale output is typically `4.75 V`
- supply voltage is `4.75 .. 5.25 V`
- the sensor is specified for clean, dry air and non-corrosive gases

Because many MCUs only accept `0 .. 3.3 V` ADC input, a resistor divider or an
external ADC is often required. `AdcConfig::input_scale` exists for exactly that
case.

## Typical Flow

1. create `Mis2500::mis2500_015g()` or `Mis2500::mis2500_015v()`
2. optionally call `calibrate_zero_from_adc_codes()`
3. convert ADC codes or analog voltage into `Pressure`
4. do application-specific math outside the driver crate

## Example

- `examples/pitot_airspeed.rs`
- `examples/read_vacuum.rs`

Run the host examples with:

```bash
cargo run -p mis2500 --example pitot_airspeed
cargo run -p mis2500 --example read_vacuum
```

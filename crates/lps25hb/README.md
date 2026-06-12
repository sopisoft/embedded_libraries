# lps25hb

`embedded-hal` 1.0 driver for the ST LPS25HB barometer used on the Akizuki `AE-LPS25HB` (`LPS25HB使用 気圧センサーモジュールDIP化キット`) and similar breakouts.

## What This Crate Contains

- chip-ID check and basic configuration
- both I2C and 4-wire SPI transports
- pressure and temperature reads
- pressure-to-altitude helpers
- RPDS one-point calibration helpers for modules that need a pressure offset

## Akizuki Module Notes

For the Akizuki `AE-LPS25HB` module in I2C mode:

- connect `CS` to `VDD` to use I2C mode
- connect `SA0` to `GND` for address `0x5C`, or to `VDD` for `0x5D`
- `J1` and `J2` are pull-up solder jumpers for `SCL` and `SDA`
- the module is a 3.3 V part

For SPI mode:

- drive `CS` from your MCU
- connect `SPC`, `SDI`, and `SDO`
- do not use the I2C pull-up jumpers as your SPI bus wiring
- the crate currently implements the normal 4-wire SPI mode

The Akizuki appendix notes that a shipped module can have a pressure offset of about `+/- 5` to `+/- 10 hPa`. The `RPDS` helper methods in this crate are intended to make that correction practical.

## Typical Flow

1. create the driver with `Lps25hb::new_i2c(...)` or `Lps25hb::new_spi(...)`
2. call `init(Config::default())`
3. optionally apply `apply_one_point_calibration()` using a trusted local pressure reference
4. read `Measurement`
5. convert pressure to altitude using `pressure_to_altitude_m()`

## Examples

- `examples/read_pressure.rs`
- `examples/read_pressure_spi.rs`
- `examples/rp235x_i2c_lps25hb.rs`
- `examples/rp235x_spi_lps25hb.rs`

Build the RP2350 examples with:

```bash
cargo build -p lps25hb --example rp235x_i2c_lps25hb --target thumbv8m.main-none-eabihf
cargo build -p lps25hb --example rp235x_spi_lps25hb --target thumbv8m.main-none-eabihf
```

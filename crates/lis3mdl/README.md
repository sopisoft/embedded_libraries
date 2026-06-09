# lis3mdl

Local `embedded-hal` 1.0 driver for the ST LIS3MDL magnetometer.

## What This Crate Contains

- I2C driver type `Lis3mdl`
- address selection through `Address`
- configuration through `Config`
- raw and scaled magnetic-field reads

## Supported Addresses

- `0x1C`
- `0x1E`

## Example

- `examples/read_magnetic_field.rs`

This example shows initialization and reading milli-gauss outputs.

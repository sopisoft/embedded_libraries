# mcp3208

`no_std` SPI driver for the MCP3208 12-bit ADC.

## What This Crate Contains

- `Mcp3208` driver
- `Channel` selection
- error handling through `Error`

## Examples

- `examples/read_voltage.rs`
- `examples/rp235x_spi_mcp3208.rs`
- `examples/rp235x_spi_mcp3208_scan_all.rs`

Build the RP2350 examples with:

```bash
cargo build -p mcp3208 --example rp235x_spi_mcp3208 --target thumbv8m.main-none-eabihf
cargo build -p mcp3208 --example rp235x_spi_mcp3208_scan_all --target thumbv8m.main-none-eabihf
```

# mcp3208

`no_std` SPI driver for the MCP3208 12-bit ADC.

## What This Crate Contains

- `Mcp3208` driver
- `Channel` selection
- error handling through `Error`

## Examples

- `examples/read_voltage.rs`
- RP2350 hardware examples are in `rp2350-examples`

Build the RP2350 examples with:

```bash
cargo rp2350 --example rp235x_spi_mcp3208
cargo rp2350 --example rp235x_spi_mcp3208_scan_all
```

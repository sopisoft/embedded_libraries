# elrs

CRSF / ExpressLRS protocol helpers for `no_std` targets.

## What This Crate Contains

- frame encoding and parsing
- RC channel packing and unpacking
- subset RC channel packets
- telemetry packet types
- parameter and device-info packets
- direct commands and MSP-over-CRSF support

## Main Types

- `Frame`, `FrameParser`
- `RcChannels`, `SubsetRcChannels`
- telemetry payload types such as `BatterySensor`, `Gps`, and `Attitude`

## Examples

- `examples/rc_frame_encode.rs`
- `examples/crsf_uart_parser.rs`
- `examples/rp235x_crsf_uart_tx.rs`
- `examples/rp235x_crsf_uart_rx.rs`

Build the RP2350 examples with:

```bash
cargo build -p elrs --example rp235x_crsf_uart_tx --target thumbv8m.main-none-eabihf
cargo build -p elrs --example rp235x_crsf_uart_rx --target thumbv8m.main-none-eabihf
```


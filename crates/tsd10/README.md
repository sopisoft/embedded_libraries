# tsd10

`embedded-io` UART driver for the Guangzhou PONO `TSD10` single-point LiDAR sold by Akizuki as `TSD10`.

## What This Crate Contains

- 4-byte active-frame parser with checksum validation and stream re-synchronization
- blocking distance reads over any `embedded_io::Read`
- start / stop / baud-rate commands over any `embedded_io::Write`
- host-runnable and RP2350 UART examples

## Akizuki Module Notes

- supply `4.5 V` to `5.5 V`
- default UART setting is `460800`, `8N1`
- the module actively outputs one 4-byte frame per sample
- `65535` means out of range
- there is no reverse-polarity or over-voltage protection
- verify your MCU UART IO levels before wiring directly

## Frame Format

- `0x5C`, distance LSB, distance MSB, checksum
- checksum is bitwise `NOT` of the two distance bytes summed with wrapping arithmetic

## Typical Flow

1. power the sensor and open UART at `460800`
2. create `Tsd10::new(uart)`
3. call `read_measurement()`
4. optionally use `stop_measurement()` or `start_measurement()`
5. if you change UART speed, call `set_baud_rate(...)` and then reconfigure the host UART

## Examples

- `examples/read_distance.rs`
- `examples/rp235x_uart_tsd10.rs`

Build the RP2350 example with:

```bash
cargo build -p tsd10 --example rp235x_uart_tsd10 --target thumbv8m.main-none-eabihf
```

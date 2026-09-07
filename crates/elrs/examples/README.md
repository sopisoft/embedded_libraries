# ELRS / CRSF Examples

- `rc_frame_encode`: build a CRSF RC frame from familiar pulse-width values
- `crsf_uart_parser`: parse a byte stream into complete CRSF frames
- RP2350 hardware examples live in the `rp2350-examples` package.

The recommended reading order is:
first `rc_frame_encode`, then `crsf_uart_parser`, then `rp235x_crsf_uart_tx`,
and finally `rp235x_crsf_uart_rx` when you are ready to inspect a live link.

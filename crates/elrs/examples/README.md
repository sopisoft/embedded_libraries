# ELRS / CRSF Examples

- `rc_frame_encode`: build a CRSF RC frame from familiar pulse-width values
- `crsf_uart_parser`: parse a byte stream into complete CRSF frames
- `rp235x_crsf_uart_tx`: send CRSF RC frames from an RP2350 UART
- `rp235x_crsf_uart_rx`: receive CRSF frames on RP2350 and print decoded values

The recommended reading order is:
first `rc_frame_encode`, then `crsf_uart_parser`, then `rp235x_crsf_uart_tx`,
and finally `rp235x_crsf_uart_rx` when you are ready to inspect a live link.

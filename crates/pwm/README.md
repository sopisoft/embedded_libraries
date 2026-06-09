# pwm

PWM helpers for hobby servos and ESCs.

## What This Crate Contains

- `ServoRange`: pulse and angle limits for one servo class
- `ServoSet`: per-channel servo ranges for shared conversions
- `Servo`: one PWM-backed servo output
- `ServoBank`: heterogeneous multi-servo helper for HAL-specific code
- `Esc`: normalized throttle output for electronic speed controllers

## Design Note

The crate keeps servo configuration intentionally small.

It handles:

- frame period
- pulse limits
- angle limits

It does not handle:

- radio trim
- servo reversing
- endpoint tuning beyond the explicit configured range

Those are better handled in the transmitter, mixer, or linkage geometry.

## Examples

- `examples/servo_basic.rs`
- `examples/esc_basic.rs`
- `examples/multi_servo_airframe.rs`
- `examples/rp235x_servo_pico2.rs`
- `examples/rp235x_four_servo_pico2.rs`

Build the RP2350 examples with:

```bash
cargo build -p pwm --example rp235x_servo_pico2 --target thumbv8m.main-none-eabihf
cargo build -p pwm --example rp235x_four_servo_pico2 --target thumbv8m.main-none-eabihf
```

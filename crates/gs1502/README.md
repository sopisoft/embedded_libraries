# gs1502

`no_std` PWM control for the GS-1502 1.5 g micro linear servo.

The default configuration uses a 20 ms frame, 700–2300 µs pulse widths, and
an approximate 7 mm total stroke. Servo units vary, so use `Gs1502::from_range`
to reduce or calibrate the endpoints before applying load.

```rust
use gs1502::Gs1502;

let mut servo = Gs1502::new(pwm_channel);
servo.set_stroke_mm(3.5)?;
```

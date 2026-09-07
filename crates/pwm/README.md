# pwm

PWM conversion for hobby servos and ESCs.

- `ServoRange` defines pulse and angle limits.
- `Servo` drives one PWM output.
- `ServoSet` converts commands for a configured group of channels.
- `ServoBank` writes pulse widths to heterogeneous HAL outputs.
- `Esc` exposes normalized throttle without servo-angle configuration.

Normalized commands use bounded `control` types. Radio trim, reversing, and linkage geometry remain outside this crate.

Host examples cover one servo, one ESC, and a multi-servo airframe. Hardware examples are in [`rp2350-examples`](../rp2350-examples/README.md).

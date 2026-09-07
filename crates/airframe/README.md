# airframe

Sensor-independent fixed-wing RC mapping, stabilization, mixing, and servo pulse generation.

It accepts decoded RC channels, attitude, body rates, and a servo layout. It returns named control outputs and pulse widths ready for `pwm::ServoBank`. Channel indices and normalized commands use validated types at the public boundary.

The default backend is cascaded PID. INDI is available separately:

```bash
cargo build -p airframe --no-default-features --features indi
```

Examples cover conventional tails, elevons, V-tails, CRSF input, and both stabilization backends.

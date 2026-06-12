# Embedded Libraries Workspace

`no_std` Rust crates for embedded sensing, estimation, RC links, and fixed-wing control.

The workspace is split into small crates so applications can depend only on the layers they need:

- `math`: vectors, quaternions, angles, poses, and small matrices
- `kinematics`: motion and fixed-wing state propagation
- `navigation`: inertial and fixed-wing dead reckoning
- `ahrs`: common attitude-estimator traits and a complementary filter
- `madgwick`: Madgwick AHRS filter
- `eskf`: error-state Kalman filter
- `indi`: simplified Incremental Nonlinear Dynamic Inversion rate control
- `imu`: shared-bus helpers and 9-DoF estimation glue
- `lis3mdl`: LIS3MDL driver
- `lps25hb`: LPS25HB barometer driver for the Akizuki `AE-LPS25HB` module, with altitude helpers
- `control`: RC shaping, PID control, and control-surface mixers
- `stabilization`: cascaded attitude and rate control
- `tecs`: Total Energy Control System for altitude and airspeed hold
- `airframe`: ELRS input to actuator-command glue for conventional, elevon, and V-tail aircraft
- `pwm`: servo and ESC PWM helpers
- `mcp3208`: MCP3208 SPI ADC driver
- `tsd10`: TSD10 UART LiDAR driver and frame parser
- `elrs`: CRSF / ELRS frames, parser, RC channels, telemetry, and parameter helpers

Detailed usage notes live in each crate root `README.md`.

Workspace-wide checks:

```bash
cargo fmt-check
cargo lint
cargo test --workspace
cargo check --workspace --examples
cargo doc --workspace --no-deps
```

# math

Small `no_std` math primitives for robotics, control, and estimation.

## What This Crate Contains

- `Vec2`, `Vec3`
- `Quat`, `EulerAngles`
- `Pose2`, `Pose3`, `Twist2`, `Twist3`
- `Angle`
- fixed-size `Matrix`
- helpers such as `deg_to_rad`, `rad_to_deg`, `wrap_pi`, and `wrap_tau`

## Design Goals

- small surface area
- predictable `no_std` behavior
- `libm`-based math without a heavyweight dependency stack

## Example

- `examples/getting_started.rs`

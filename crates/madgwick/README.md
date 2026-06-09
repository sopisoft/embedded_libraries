# madgwick

Madgwick AHRS filter for `no_std` targets.

## What This Crate Contains

- `Madgwick`: attitude estimation from IMU or MARG data

## When To Use It

- You want a lightweight 3D orientation filter
- You have gyroscope and accelerometer data
- You optionally also have a magnetometer for yaw observability

## Example

- `examples/attitude_from_marg.rs`


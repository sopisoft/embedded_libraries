# ahrs

Common attitude-estimation traits and lightweight attitude filters.

## What This Crate Contains

- `AttitudeEstimator`: a small trait for code that produces attitude estimates
- `ComplementaryAttitudeFilter`: a simple filter that fuses gyroscope and accelerometer data
- `Madgwick`: a lightweight IMU / MARG attitude filter

## When To Use It

- You want a small attitude-estimation interface shared across multiple estimators
- You need a simple roll/pitch estimator without the complexity of a full AHRS stack
- You want a small attitude filter without the full navigation state of `eskf`

## Example

- `examples/complementary_filter.rs`
- `examples/attitude_from_marg.rs`

This example shows how to feed body-rate and acceleration samples into the filter and read back Euler angles.

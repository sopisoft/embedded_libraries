# ahrs

Common attitude-estimation traits and a lightweight complementary filter.

## What This Crate Contains

- `AttitudeEstimator`: a small trait for code that produces attitude estimates
- `ComplementaryAttitudeFilter`: a simple filter that fuses gyroscope and accelerometer data

## When To Use It

- You want a small attitude-estimation interface shared across multiple estimators
- You need a simple roll/pitch estimator without the complexity of a full AHRS stack
- You want a baseline filter before moving to `madgwick` or `eskf`

## Example

- `examples/complementary_filter.rs`

This example shows how to feed body-rate and acceleration samples into the filter and read back Euler angles.

# eskf

Error-state Kalman filter for inertial navigation.

## What This Crate Contains

- `Eskf`: a compact 12-state error-state Kalman filter implementation

## Intended Use

Use this crate when you already have:

- inertial samples
- velocity, heading, or altitude-related corrections from another sensor
- enough system knowledge to tune process and measurement noise

This crate is intentionally lower level than `airframe` or `imu`.

## Example

- `examples/inertial_fusion.rs`

The example demonstrates predict and correct steps with synthetic measurements.

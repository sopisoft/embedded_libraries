# airframe

Sensor-agnostic fixed-wing glue between RC input, stabilization, mixers, and servo pulses.

## What This Crate Contains

- ELRS / CRSF input decoding through `RcInputConfig`
- Conventional-tail high-level controller through `FixedWingController`
- Elevon high-level controller through `ElevonController`
- V-tail high-level controller through `VTailController`
- Feature-selected stabilization backend through `DefaultAttitudeController`
- Servo-slot mapping through `ServoMap`, `ElevonServoMap`, and `VTailServoMap`
- Subset-frame merging through `apply_subset_channels`

## Design Intent

This crate does not own the estimator or the board HAL.

You provide:

- decoded RC channels
- estimated attitude
- estimated body rates
- a mixer and servo layout

The crate returns:

- surface commands
- throttle command
- final pulse widths ready for `pwm::ServoBank`

## Stabilization Backend

The default build uses the existing cascaded PID backend:

```bash
cargo build -p airframe
```

To make the default controller type use INDI instead:

```bash
cargo build -p airframe --no-default-features --features indi
```

Existing explicit PID code can still use `CascadeAttitudeController` when the
`cascade-pid` feature is enabled. INDI code can use `IndiAttitudeController`
when the `indi` feature is enabled.

## Examples

- `examples/elrs_attitude_hold_pipeline.rs`
- `examples/indi_attitude_hold_pipeline.rs`
- `examples/elevon_attitude_hold.rs`
- `examples/vtail_attitude_hold.rs`
- `examples/rp235x_elrs_imu_attitude_hold.rs`

The RP2350 example shows a full control chain:

`ELRS RX -> IMU estimator -> airframe controller -> pwm::ServoBank`

Build it with:

```bash
cargo build -p airframe --example rp235x_elrs_imu_attitude_hold --target thumbv8m.main-none-eabihf
```

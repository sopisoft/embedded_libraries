# IMU Examples

- `marg_estimation`: smallest useful 9-DoF attitude and relative-altitude workflow
- `rp235x_stemma_qt_static_calibration`: stationary noise / bias capture for threshold tuning
- `rp235x_stemma_qt_dynamic_calibration`: motion / magnetometer span capture for dynamic tuning
- `rp235x_stemma_qt_9dof`: RP2350 + STEMMA QT example for the Adafruit LSM6DS3TR-C + LIS3MDL board
- `rp235x_stemma_qt_9dof_lps25hb`: RP2350 + STEMMA QT example that adds the LPS25HB barometer and emits human-readable `state ...` lines compatible with `imu-viz`

Read `marg_estimation` first to understand the data flow. Then move to the
RP2350 example when you are ready to wire a real board over I2C.

`rp235x_stemma_qt_9dof` and `rp235x_stemma_qt_9dof_lps25hb` emit human-readable
`defmt` state lines in this form:

`state t_ms=... quat=(w, x, y, z) euler_deg=(roll, pitch, yaw) velocity_m_s=(vx, vy, vz) altitude_m=...`

## Calibration Procedure

1. IMU bias calibration
   - Run `rp235x_stemma_qt_static_calibration`.
   - Keep the board fully still during the startup capture.
   - Record the reported gyro bias, accel bias, and Allan-based stationary-threshold suggestions.

2. Stationary threshold tuning
   - Use the suggested `accel_tolerance_m_s2` and `gyro_tolerance_rad_s` from the static calibration log.
   - If the runtime still reports motion while stationary, raise the thresholds slightly.
   - If motion is missed too often, lower them slightly.

3. Magnetometer calibration
   - Run `rp235x_stemma_qt_dynamic_calibration`.
   - Move the board through many orientations, including all six faces and several diagonal poses.
   - Wait until the reported magnetometer span is ready and use the suggested `mag_calibration_min_span_mgauss`.

4. Runtime verification
   - Run `rp235x_stemma_qt_9dof` or `rp235x_stemma_qt_9dof_lps25hb`.
   - Confirm that roll and pitch settle near zero on a level surface.
   - Confirm that yaw remains stable after the magnetometer reports ready.

For real-time visualization on the host, run:

`cargo run -p imu-viz`

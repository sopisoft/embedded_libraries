# Airframe Examples

- `elrs_attitude_hold_pipeline`: ELRS input -> attitude hold -> surface mixing -> servo pulse output
- `indi_attitude_hold_pipeline`: the same pipeline using the INDI backend
- `elevon_attitude_hold`: the same high-level pipeline for a delta wing / flying wing
- `vtail_attitude_hold`: the same high-level pipeline for a V-tail aircraft
- `rp235x_elrs_imu_attitude_hold`: RP2350 integration example from ELRS RX and IMU to live PWM outputs

Read this example when you want to build a flight stack without hard-coding the
sensor layer. It expects estimated attitude and body rates from any estimator.

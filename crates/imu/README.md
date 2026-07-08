# imu

Generic IMU sample types, shared-bus helpers, and lightweight 9-DoF estimation glue.

## What This Crate Contains

- `SharedI2c` for reusing one I2C peripheral across multiple devices
- sample structs such as `AccelGyroSample` and `MargSample`
- `MargEstimator` and `EskfEstimator` for attitude, world velocity, and relative-altitude estimation
- `ImuEstimator` for switching estimator backends without changing call sites
- traits for accelerometer, gyroscope, and magnetometer sources

## Notes

- The altitude output is relative altitude derived from inertial integration
- Without a barometer or other correction source, altitude will drift over time

## Examples

- `examples/marg_estimation.rs`
- `examples/rp235x_stemma_qt_9dof.rs`
- `examples/rp235x_stemma_qt_9dof_lps25hb.rs`

Build the RP2350 + barometer example with:

```bash
cargo run -p imu --example rp235x_stemma_qt_9dof_lps25hb --target thumbv8m.main-none-eabihf
```

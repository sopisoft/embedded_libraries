# imu

Generic IMU sample types, shared-bus helpers, and lightweight 9-DoF estimation glue.

## What This Crate Contains

- `SharedI2c` for reusing one I2C peripheral across multiple devices
- sample structs such as `AccelGyroSample` and `MargSample`
- `MargEstimator` for attitude and relative-altitude estimation
- traits for accelerometer, gyroscope, and magnetometer sources

## Notes

- The altitude output is relative altitude derived from inertial integration
- Without a barometer or other correction source, altitude will drift over time

## Examples

- `examples/marg_estimation.rs`
- `examples/rp235x_stemma_qt_9dof.rs`

Build the RP2350 example with:

```bash
cargo build -p imu --example rp235x_stemma_qt_9dof --target thumbv8m.main-none-eabihf
```


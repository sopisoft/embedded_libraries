# imu

IMU samples, shared-I2C access, calibration, and lightweight 9-DoF estimation.

Sensor values use unit-specific types. `MargEstimator` and `EskfEstimator` produce attitude, world velocity, and relative altitude; altitude drifts without an external reference.

RP2350 acquisition and calibration examples are in [`rp2350-examples`](../rp2350-examples/README.md).

```bash
cargo rp2350 --example rp235x_stemma_qt_9dof_lps25hb
```

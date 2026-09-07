# RP2350 examples

Firmware examples share the workspace target, features, and `probe-rs` runner.

```bash
cargo rp2350-check
cargo rp2350-clippy
cargo rp2350 --example rp2350
```

The `rp2350` flight-controller example uses GPIO2–5 for ailerons, elevator, and rudder; GPIO1 for GS-1502; GPIO0 for ESC; GPIO12–13 for CRSF UART; GPIO18–19 for the IMU and magnetometer I2C1 bus; GPIO8–9 for the LPS25HB I2C0 bus; and GPIO6–7 as PWM3A/PWM3B status outputs for NJW4617. Servo power uses an external 5 V BEC with a common ground.

The `rp235x_four_servo_pico2` example uses GPIO2–5 so the four servo signals are consecutive on the Pico 2 header (physical pins 4–7), without a GND pin between them.

| Source | Destination |
| --- | --- |
| Left aileron servo signal | Pico 2 GPIO2 (PWM1 A, physical pin 4) |
| Right aileron servo signal | Pico 2 GPIO3 (PWM1 B, physical pin 5) |
| Elevator servo signal | Pico 2 GPIO4 (PWM2 A, physical pin 6) |
| Rudder servo signal | Pico 2 GPIO5 (PWM2 B, physical pin 7) |
| GS-1502 signal wire | Pico 2 GPIO1 (PWM0 B, physical pin 2) |
| ELRS receiver CRSF TX | Pico 2 GPIO13 (UART0 RX) |
| Pico 2 GPIO12 (UART0 TX, optional telemetry) | ELRS receiver CRSF RX |
| LSM6DS3TR-C + LIS3MDL SDA | Pico 2 GPIO18 (I2C1 SDA) |
| LSM6DS3TR-C + LIS3MDL SCL | Pico 2 GPIO19 (I2C1 SCL) |
| LPS25HB SDA | Pico 2 GPIO8 (I2C0 SDA) |
| LPS25HB SCL | Pico 2 GPIO9 (I2C0 SCL) |

The four servo signals are consecutive on the Pico 2 header, with no GND pin between them.

For the `rp2350` flight-controller example, connect the GS-1502 signal to GPIO1 and the ESC signal to GPIO0. Connect CRSF UART to GPIO12 (TX) / GPIO13 (RX), IMU/LIS3MDL I2C to GPIO18 (SDA) / GPIO19 (SCL), and LPS25HB I2C to GPIO8 (SDA) / GPIO9 (SCL).

Enabling attitude hold with a valid LPS25HB reading captures the current barometric altitude and holds it with TECS pitch and throttle corrections. If barometer data becomes unavailable, control falls back to attitude hold with pilot throttle and pitch input.

For `rp235x_stemma_qt_9dof_lps25hb`, connect the 9-DoF breakout SDA/SCL to GPIO16/17 (I2C0) and the LPS25HB SDA/SCL to GPIO18/19 (I2C1). The default I2C addresses are LSM6DS3TR-C `0x6A`, LIS3MDL `0x1C`, and LPS25HB `0x5C`.

The IMU examples include static bias capture, dynamic magnetometer calibration, 9-DoF estimation, and USB telemetry for `imu-viz`.

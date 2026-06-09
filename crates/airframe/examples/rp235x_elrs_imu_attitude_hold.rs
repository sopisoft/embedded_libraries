#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

// This example combines the full fixed-wing control chain on RP2350 hardware:
//
// 1. receive ELRS / CRSF RC frames over UART,
// 2. read an LSM6DS3TR-C + LIS3MDL 9-DoF IMU over I2C,
// 3. estimate attitude and relative altitude,
// 4. run the conventional-tail attitude-hold controller,
// 5. write pulse widths to multiple PWM outputs through `pwm::ServoBank`.
//
// Suggested wiring on a Pico 2 style board:
// - ELRS RX/TX module:
//   - GPIO12 -> CRSF TX from receiver or transmitter device
//   - GPIO13 -> CRSF RX if you later add replies (optional in this example)
// - Adafruit 9-DoF board:
//   - GPIO18 -> SDA
//   - GPIO19 -> SCL
//   - 3V3 -> VIN
//   - GND -> GND
// - Servo / ESC outputs:
//   - GPIO0 -> PWM0 A -> left aileron
//   - GPIO1 -> PWM0 B -> right aileron
//   - GPIO2 -> PWM1 A -> elevator
//   - GPIO3 -> PWM1 B -> rudder
//   - GPIO4 -> PWM2 A -> throttle / ESC
// - Debug UART:
//   - GPIO8 -> UART1 TX
//   - GPIO9 -> UART1 RX

#[cfg(all(target_arch = "arm", target_os = "none"))]
#[path = "rp235x_elrs_imu_attitude_hold/embedded_main.rs"]
mod embedded_main;

#[cfg(all(target_arch = "arm", target_os = "none"))]
#[rp235x_hal::entry]
fn main() -> ! {
    embedded_main::run()
}

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn main() {
    println!(
        "This example is for RP2350 hardware. Build it with \
         `cargo build -p airframe --example rp235x_elrs_imu_attitude_hold --target thumbv8m.main-none-eabihf`."
    );
}

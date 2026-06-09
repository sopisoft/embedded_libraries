// This example shows the smallest useful INDI loop:
//
// 1. choose a control effectiveness for the axis,
// 2. feed desired and measured body rates,
// 3. let INDI estimate angular acceleration from the gyro rate history,
// 4. apply the returned actuator command to your plant.
//
// The "plant" here is only a tiny numerical toy model so the example can run
// on the host. On real hardware:
//
// - `target_rate_rad_s` comes from your pilot command, attitude controller, or
//   navigation controller
// - `measured_rate_rad_s` comes from the gyro
// - the returned `actuator` goes to a mixer, servo, ESC, or other actuator

use fugit::MicrosDurationU32;
use indi::{IndiAxis, IndiAxisConfig};

fn main() {
    // Assume a normalized elevator or aileron command where +1.0 produces
    // roughly +18 rad/s^2 angular acceleration around this operating point.
    let config = IndiAxisConfig::symmetric(18.0, 7.0, 25.0, 12.0);
    let mut controller = IndiAxis::new(config);

    let dt = MicrosDurationU32::from_millis(20);
    let target_rate_rad_s = 1.2;
    let mut measured_rate_rad_s = 0.0;

    println!("step  target_rate  measured_rate  actuator");
    for step in 0..60 {
        let output = controller.update_rate(target_rate_rad_s, measured_rate_rad_s, dt);

        // Toy plant:
        // - the actuator produces angular acceleration,
        // - a little damping keeps the response finite.
        let measured_accel_rad_s2 = output.actuator * 18.0 - measured_rate_rad_s * 1.2;
        measured_rate_rad_s += measured_accel_rad_s2 * dt.as_secs_f32();

        println!(
            "{step:>4}  {target_rate_rad_s:>11.3}  {measured_rate_rad_s:>13.3}  {actuator:>8.3}",
            actuator = output.actuator,
        );
    }
}

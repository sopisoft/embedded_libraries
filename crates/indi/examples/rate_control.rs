use fugit::MicrosDurationU32;
use indi::{IndiAxis, IndiAxisConfig};

fn main() {
    let config = IndiAxisConfig::symmetric(18.0, 7.0, 25.0, 12.0);
    let mut controller = IndiAxis::new(config);

    let dt = MicrosDurationU32::from_millis(20);
    let target_rate_rad_s = 1.2;
    let mut measured_rate_rad_s = 0.0;

    println!("step  target_rate  measured_rate  actuator");
    for step in 0..60 {
        let output = controller.update_rate(target_rate_rad_s, measured_rate_rad_s, dt);

        let measured_accel_rad_s2 = output.actuator * 18.0 - measured_rate_rad_s * 1.2;
        measured_rate_rad_s += measured_accel_rad_s2 * dt.as_secs_f32();

        println!(
            "{step:>4}  {target_rate_rad_s:>11.3}  {measured_rate_rad_s:>13.3}  {actuator:>8.3}",
            actuator = output.actuator,
        );
    }
}

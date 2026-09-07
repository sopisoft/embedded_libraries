use core::f32::consts::FRAC_PI_4;

use eskf::Eskf;
use fugit::MicrosDurationU32;
use glam::{EulerRot, Quat, Vec3};

fn euler_deg(q: Quat) -> Vec3 {
    let (roll, pitch, yaw) = q.to_euler(EulerRot::XYZ);
    Vec3::new(roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees())
}

fn main() {
    let mut filter = Eskf::new();
    filter.set_noise(0.3, 0.03, 0.01, 0.001);

    let dt = MicrosDurationU32::from_millis(20);

    for _ in 0..100 {
        filter.predict(Vec3::new(0.0, 0.0, 0.01), Vec3::new(0.2, 0.0, 9.80665), dt);
    }

    filter.correct_velocity(Vec3::new(21.0, 0.5, -0.1), 1.0);
    filter.correct_forward_speed(20.5, 0.5);
    filter.correct_heading(FRAC_PI_4, 0.2);
    filter.correct_orientation(
        Quat::from_euler(
            EulerRot::XYZ,
            2.0f32.to_radians(),
            (-3.0f32).to_radians(),
            45.0f32.to_radians(),
        ),
        0.1,
    );

    let attitude_deg = euler_deg(filter.orientation);
    println!(
        "Velocity estimate: ({:.2}, {:.2}, {:.2}) m/s",
        filter.velocity.x, filter.velocity.y, filter.velocity.z
    );
    println!(
        "Attitude estimate: roll={:.2} deg, pitch={:.2} deg, yaw={:.2} deg",
        attitude_deg.x, attitude_deg.y, attitude_deg.z
    );
}

use core::f32::consts::FRAC_PI_4;

use eskf::Eskf;
use fugit::MicrosDurationU32;
use glam::{EulerRot, Quat, Vec3};

fn euler_deg(q: Quat) -> Vec3 {
    let (roll, pitch, yaw) = q.to_euler(EulerRot::XYZ);
    Vec3::new(roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees())
}

fn main() {
    // An ESKF is useful when you want to:
    // - propagate state with high-rate IMU data,
    // - correct drift with slower sensors such as GPS, barometer, heading, or airspeed.
    //
    // This example follows a common flight-control pattern:
    // 1. run many prediction steps from IMU data,
    // 2. apply a few external corrections,
    // 3. inspect the updated navigation state.

    let mut filter = Eskf::new();
    filter.set_noise(0.3, 0.03, 0.01, 0.001);

    let dt = MicrosDurationU32::from_millis(20);

    // Prediction:
    // - small forward specific force,
    // - slight positive yaw rate,
    // - repeated for 2 seconds.
    //
    // The accelerometer input is "specific force", not raw world acceleration.
    // In this coordinate convention, a stationary level IMU would read +9.80665 on Z.
    for _ in 0..100 {
        filter.predict(Vec3::new(0.0, 0.0, 0.01), Vec3::new(0.2, 0.0, 9.80665), dt);
    }

    // Correction:
    // Pretend that slower sensors now provide absolute information.
    filter.correct_position(Vec3::new(42.0, 8.0, 120.0), 5.0);
    filter.correct_velocity(Vec3::new(21.0, 0.5, -0.1), 1.0);
    filter.correct_altitude(118.0, 0.3);
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
        "Position estimate: ({:.2}, {:.2}, {:.2}) m",
        filter.position.x, filter.position.y, filter.position.z
    );
    println!(
        "Velocity estimate: ({:.2}, {:.2}, {:.2}) m/s",
        filter.velocity.x, filter.velocity.y, filter.velocity.z
    );
    println!(
        "Attitude estimate: roll={:.2} deg, pitch={:.2} deg, yaw={:.2} deg",
        attitude_deg.x, attitude_deg.y, attitude_deg.z
    );
}

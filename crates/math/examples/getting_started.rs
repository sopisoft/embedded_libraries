use math::{Angle, EulerAngles, Matrix, Pose2, Quat, Twist2, Vec2, Vec3};

fn main() {
    // This example shows the three things you usually need first:
    // 1. store angles without manually converting degrees to radians,
    // 2. rotate vectors with a quaternion,
    // 3. integrate a small pose update step.
    //
    // In a real project these values would come from your IMU, GPS, wheel encoder,
    // or air-data sensors. Here we use hard-coded numbers so the flow is easy to read.

    // Step 1: work with angles.
    let heading = Angle::from_degrees(90.0);
    let (sin_heading, cos_heading) = heading.sin_cos();
    println!(
        "Heading: {:.1} deg = {:.3} rad, sin={:.1}, cos={:.1}",
        heading.degrees(),
        heading.radians(),
        sin_heading,
        cos_heading
    );

    // Step 2: represent a 3D attitude and rotate a forward vector into the world frame.
    // This attitude means:
    // - roll   = 5 deg
    // - pitch  = -2 deg
    // - yaw    = 30 deg
    let attitude = Quat::from_euler(EulerAngles::from_degrees(5.0, -2.0, 30.0));
    let body_forward_velocity = Vec3::new(20.0, 0.0, 0.0);
    let world_velocity = attitude.rotate_vec3(body_forward_velocity);
    println!(
        "Forward body velocity becomes world velocity ({:.2}, {:.2}, {:.2}) m/s",
        world_velocity.x, world_velocity.y, world_velocity.z
    );

    // Step 3: integrate a planar pose from a body-frame command.
    // Imagine a ground robot commanded to move forward at 1.5 m/s
    // while turning left at 0.25 rad/s for one second.
    let mut pose = Pose2::identity();
    let body_twist = Twist2::new(Vec2::new(1.5, 0.0), 0.25);
    for _ in 0..10 {
        pose.integrate_twist(body_twist, 0.1);
    }
    println!(
        "Integrated planar pose: x={:.2} m, y={:.2} m, heading={:.1} deg",
        pose.position.x,
        pose.position.y,
        Angle::from_radians(pose.heading).degrees()
    );

    // Step 4: apply a small matrix as a calibration transform.
    // A 2x2 matrix is often enough for a simple 2D sensor correction.
    let raw_measurement = Vec2::new(3.0, -1.0);
    let calibration = Matrix::<2, 2>::new([[1.02, 0.01], [-0.02, 0.99]]);
    let corrected = calibration * [raw_measurement.x, raw_measurement.y];
    println!(
        "Calibrated 2D measurement: ({:.3}, {:.3})",
        corrected[0], corrected[1]
    );
}

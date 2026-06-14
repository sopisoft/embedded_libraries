use fugit::MicrosDurationU32;
use glam::{EulerRot, Quat, Vec3};
use madgwick::Madgwick;

fn euler_deg(q: Quat) -> Vec3 {
    let (roll, pitch, yaw) = q.to_euler(EulerRot::XYZ);
    Vec3::new(roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees())
}

fn main() {
    // Madgwick is a practical "drop in" filter when you have:
    // - gyro,
    // - accelerometer,
    // - optionally magnetometer.
    //
    // This example simulates a stationary sensor at a known attitude and shows
    // how repeated MARG updates make the estimate converge.

    let true_orientation = Quat::from_euler(
        EulerRot::XYZ,
        10.0f32.to_radians(),
        (-8.0f32).to_radians(),
        60.0f32.to_radians(),
    );
    let gravity_world = Vec3::Z;
    let magnetic_world = Vec3::X;

    let accel_body = true_orientation.conjugate().mul_vec3(gravity_world);
    let mag_body = true_orientation.conjugate().mul_vec3(magnetic_world);

    let mut filter = Madgwick::new(0.1);
    let dt = MicrosDurationU32::from_millis(5);

    for _ in 0..1_000 {
        filter.update_marg(Vec3::ZERO, accel_body, mag_body, dt);
    }

    let estimate_deg = euler_deg(filter.orientation());
    println!(
        "Madgwick estimate: roll={:.1} deg, pitch={:.1} deg, yaw={:.1} deg",
        estimate_deg.x, estimate_deg.y, estimate_deg.z
    );

    // If you do not have a magnetometer, use update_imu().
    filter.update_imu(Vec3::new(0.0, 0.0, 0.05), accel_body, dt);
    let imu_only_deg = euler_deg(filter.orientation());
    println!(
        "After one IMU-only step: roll={:.1} deg, pitch={:.1} deg, yaw={:.1} deg",
        imu_only_deg.x, imu_only_deg.y, imu_only_deg.z
    );
}

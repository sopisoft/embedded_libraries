use fugit::MicrosDurationU32;
use madgwick::Madgwick;
use math::{EulerAngles, Quat, Vec3};

fn main() {
    // Madgwick is a practical "drop in" filter when you have:
    // - gyro,
    // - accelerometer,
    // - optionally magnetometer.
    //
    // This example simulates a stationary sensor at a known attitude and shows
    // how repeated MARG updates make the estimate converge.

    let true_orientation = Quat::from_euler(EulerAngles::from_degrees(10.0, -8.0, 60.0));
    let gravity_world = Vec3::unit_z();
    let magnetic_world = Vec3::unit_x();

    let accel_body = true_orientation.rotate_inverse_vec3(gravity_world);
    let mag_body = true_orientation.rotate_inverse_vec3(magnetic_world);

    let mut filter = Madgwick::new(0.1);
    let dt = MicrosDurationU32::from_millis(5);

    for _ in 0..1_000 {
        filter.update_marg(Vec3::zero(), accel_body, mag_body, dt);
    }

    let estimate_deg = filter.orientation().to_euler().to_degrees();
    println!(
        "Madgwick estimate: roll={:.1} deg, pitch={:.1} deg, yaw={:.1} deg",
        estimate_deg.roll, estimate_deg.pitch, estimate_deg.yaw
    );

    // If you do not have a magnetometer, use update_imu().
    filter.update_imu(Vec3::new(0.0, 0.0, 0.05), accel_body, dt);
    let imu_only_deg = filter.orientation().to_euler().to_degrees();
    println!(
        "After one IMU-only step: roll={:.1} deg, pitch={:.1} deg, yaw={:.1} deg",
        imu_only_deg.roll, imu_only_deg.pitch, imu_only_deg.yaw
    );
}

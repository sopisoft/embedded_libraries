use ahrs::{AttitudeEstimator, ComplementaryAttitudeFilter};
use fugit::MicrosDurationU32;
use math::{EulerAngles, Quat, Vec3};

fn main() {
    // The complementary filter is a good first AHRS because the inputs are simple:
    // - gyroscope in rad/s,
    // - accelerometer direction,
    // - optionally magnetometer direction for yaw.
    //
    // This example simulates a stationary vehicle with a known attitude.
    // We generate the "measured" gravity and magnetic field vectors in the body frame,
    // feed them into the filter many times, and print the converged estimate.

    let true_orientation = Quat::from_euler(EulerAngles::from_degrees(12.0, -4.0, 35.0));

    // In this crate, a level stationary accelerometer points along +Z.
    let gravity_world = Vec3::unit_z();
    let magnetic_world = Vec3::unit_x();

    // Convert world-frame reference vectors into body-frame sensor readings.
    let accel_body = true_orientation.rotate_inverse_vec3(gravity_world);
    let mag_body = true_orientation.rotate_inverse_vec3(magnetic_world);

    let mut filter = ComplementaryAttitudeFilter::new(0.05);
    let dt = MicrosDurationU32::from_millis(10);

    for _ in 0..400 {
        filter.update_marg(Vec3::zero(), accel_body, mag_body, dt);
    }

    let estimate_deg = filter.orientation().to_euler().to_degrees();
    println!(
        "Estimated attitude: roll={:.1} deg, pitch={:.1} deg, yaw={:.1} deg",
        estimate_deg.roll, estimate_deg.pitch, estimate_deg.yaw
    );

    // The same filter can also be used with IMU-only updates when no magnetometer is present.
    filter.update_imu(Vec3::new(0.0, 0.0, 0.1), accel_body, dt);
    let after_gyro = filter.orientation().to_euler().to_degrees();
    println!(
        "After one IMU-only step: roll={:.1} deg, pitch={:.1} deg, yaw={:.1} deg",
        after_gyro.roll, after_gyro.pitch, after_gyro.yaw
    );
}

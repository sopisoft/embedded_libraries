use ahrs::ComplementaryAttitudeFilter;
use fugit::MicrosDurationU32;
use glam::{EulerRot, Quat, Vec3};

fn euler_deg(q: Quat) -> Vec3 {
    let (roll, pitch, yaw) = q.to_euler(EulerRot::XYZ);
    Vec3::new(roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees())
}

fn main() {
    let true_orientation = Quat::from_euler(
        EulerRot::XYZ,
        12.0f32.to_radians(),
        (-4.0f32).to_radians(),
        35.0f32.to_radians(),
    );

    let gravity_world = Vec3::Z;
    let magnetic_world = Vec3::X;

    let accel_body = true_orientation.conjugate().mul_vec3(gravity_world);
    let mag_body = true_orientation.conjugate().mul_vec3(magnetic_world);

    let mut filter = ComplementaryAttitudeFilter::with_correction_gain(0.05);
    let dt = MicrosDurationU32::from_millis(10);

    for _ in 0..400 {
        filter.update_marg(Vec3::ZERO, accel_body, mag_body, dt);
    }

    let estimate_deg = euler_deg(filter.orientation());
    println!(
        "Estimated attitude: roll={:.1} deg, pitch={:.1} deg, yaw={:.1} deg",
        estimate_deg.x, estimate_deg.y, estimate_deg.z
    );

    filter.update_imu(Vec3::new(0.0, 0.0, 0.1), accel_body, dt);
    let after_gyro = euler_deg(filter.orientation());
    println!(
        "After one IMU-only step: roll={:.1} deg, pitch={:.1} deg, yaw={:.1} deg",
        after_gyro.x, after_gyro.y, after_gyro.z
    );
}

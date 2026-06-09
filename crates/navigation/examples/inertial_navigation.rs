use fugit::MicrosDurationU32;
use math::{EulerAngles, Quat, Vec3};
use navigation::InertialNavigator;

fn main() {
    // This is the smallest useful navigation workflow in the workspace.
    // The navigator keeps:
    // - position,
    // - orientation,
    // - world-frame velocity.
    //
    // A common pattern is:
    // 1. run many high-rate IMU prediction steps,
    // 2. occasionally blend in slower external measurements.
    let mut nav = InertialNavigator::new();
    let dt = MicrosDurationU32::from_millis(20);

    // Predict for one second with slight forward specific force and a small yaw rate.
    for _ in 0..50 {
        nav.predict_imu(Vec3::new(0.3, 0.0, 9.80665), Vec3::new(0.0, 0.0, 0.02), dt);
    }

    println!(
        "After IMU prediction: position=({:.2}, {:.2}, {:.2}) m",
        nav.pose.position.x, nav.pose.position.y, nav.pose.position.z
    );
    println!(
        "After IMU prediction: velocity=({:.2}, {:.2}, {:.2}) m/s",
        nav.velocity_world.x, nav.velocity_world.y, nav.velocity_world.z
    );

    // Now inject slower absolute information, for example from GPS, barometer, or airspeed.
    nav.correct_position(Vec3::new(10.0, 2.0, 120.0), 0.3);
    nav.correct_velocity(Vec3::new(14.0, 0.5, -0.2), 0.4);
    nav.correct_altitude(118.5, 0.5);
    nav.correct_heading(0.8, 0.2);
    nav.correct_forward_speed(15.0, 0.6);

    let euler_deg = nav.pose.orientation.to_euler().to_degrees();
    println!(
        "Corrected position: ({:.2}, {:.2}, {:.2}) m",
        nav.pose.position.x, nav.pose.position.y, nav.pose.position.z
    );
    println!(
        "Corrected attitude: roll={:.2} deg, pitch={:.2} deg, yaw={:.2} deg",
        euler_deg.roll, euler_deg.pitch, euler_deg.yaw
    );

    // You can also replace the orientation directly if a higher-level estimator owns it.
    nav.pose.orientation = Quat::from_euler(EulerAngles::from_degrees(1.0, -2.0, 45.0));
    println!(
        "Manually set yaw for downstream logic: {:.2} deg",
        nav.pose.orientation.to_euler().to_degrees().yaw
    );
}

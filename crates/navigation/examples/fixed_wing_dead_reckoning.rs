use fugit::MicrosDurationU32;
use math::{EulerAngles, Vec2, Vec3};
use navigation::FixedWingNavigator;

fn main() {
    // This example demonstrates a very common fixed-wing navigation pattern:
    // 1. set an initial attitude estimate,
    // 2. propagate position from airspeed, gyro, and wind,
    // 3. apply slower correction terms when extra information is available.
    //
    // The same pattern works for RC aircraft, small UAVs, and log replay tools.

    let mut navigator = FixedWingNavigator::new();
    navigator.set_attitude(EulerAngles::from_degrees(2.0, 5.0, 45.0));

    // High-rate propagation inputs.
    let dt = MicrosDurationU32::from_millis(20);
    let wind = Vec2::new(4.0, -1.0);
    let gyro = Vec3::new(0.0, 0.0, 0.03);

    // Propagate for one second at 22 m/s airspeed.
    let mut step = 0;
    while step < 50 {
        navigator.predict_airspeed(22.0, gyro, wind, dt);
        step += 1;
    }

    // Later, a slower sensor such as GPS can tell us the actual ground velocity.
    // Use that to refine the horizontal wind estimate.
    navigator.correct_wind_from_groundspeed(Vec3::new(18.0, 14.0, 1.5), 0.2);

    // The underlying inertial navigator is public so you can apply additional corrections.
    navigator.navigator.correct_altitude(115.0, 0.3);
    navigator.navigator.correct_heading(0.80, 0.2);

    let state = navigator.as_state();

    println!(
        "Position: ({:.1}, {:.1}, {:.1}) m",
        state.pose.position.x, state.pose.position.y, state.pose.position.z
    );
    println!(
        "Velocity: ({:.1}, {:.1}, {:.1}) m/s",
        state.ground_velocity.x, state.ground_velocity.y, state.ground_velocity.z
    );
    println!(
        "Estimated wind: ({:.1}, {:.1}) m/s",
        state.wind_xy.x, state.wind_xy.y
    );
    println!(
        "Estimated Euler attitude: roll={:.1} deg, pitch={:.1} deg, yaw={:.1} deg",
        state.pose.orientation.to_euler().to_degrees().roll,
        state.pose.orientation.to_euler().to_degrees().pitch,
        state.pose.orientation.to_euler().to_degrees().yaw
    );
}

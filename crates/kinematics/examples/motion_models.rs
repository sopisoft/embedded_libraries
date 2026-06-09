use fugit::MicrosDurationU32;
use kinematics::{FixedWingState, MotionState2, MotionState3, coordinated_turn_rate};
use math::{EulerAngles, Pose2, Pose3, Twist2, Twist3, Vec2, Vec3};

fn main() {
    // This example shows three different motion models:
    // 1. a planar robot driven by a body-frame velocity command,
    // 2. a full 3D body driven by a body-frame twist,
    // 3. a fixed-wing aircraft driven by airspeed, turn rate, and wind.
    //
    // You do not need all three in one project. Pick the model that matches
    // your vehicle and sensor set.

    // ---------------------------------------------------------------------
    // 1) Planar motion
    // ---------------------------------------------------------------------
    let dt_planar = MicrosDurationU32::from_millis(100);
    let mut rover = MotionState2::new(Pose2::identity());
    let rover_command = Twist2::new(Vec2::new(2.0, 0.0), 0.3);

    for _ in 0..20 {
        rover.step_twist(rover_command, dt_planar);
    }

    println!(
        "Planar model after 2.0 s: x={:.2} m, y={:.2} m, heading={:.1} deg",
        rover.pose.position.x,
        rover.pose.position.y,
        rover.pose.heading.to_degrees()
    );

    // ---------------------------------------------------------------------
    // 2) Spatial motion
    // ---------------------------------------------------------------------
    // This is useful when you already know the body-frame linear and angular velocity,
    // for example from a flight controller state estimate.
    let mut body3d = MotionState3::new(Pose3::identity());
    let twist3 = Twist3::new(Vec3::new(5.0, 0.0, -0.5), Vec3::new(0.0, 0.0, 0.2));
    body3d.step_twist(twist3, MicrosDurationU32::from_secs(1));

    println!(
        "3D twist model after 1.0 s: position=({:.2}, {:.2}, {:.2}) m",
        body3d.pose.position.x, body3d.pose.position.y, body3d.pose.position.z
    );

    // ---------------------------------------------------------------------
    // 3) Fixed-wing motion
    // ---------------------------------------------------------------------
    // For an aircraft, a common minimal model is:
    // - current attitude,
    // - scalar airspeed along body X,
    // - horizontal wind estimate,
    // - gyro turn rate for attitude propagation.
    let mut aircraft = FixedWingState::new();
    let banked_turn = EulerAngles::from_degrees(20.0, 0.0, 45.0);
    aircraft.set_euler(banked_turn);

    let airspeed_m_s = 25.0;
    let wind_xy = Vec2::new(4.0, -1.5);
    let yaw_rate = coordinated_turn_rate(banked_turn.roll, airspeed_m_s, 9.80665);

    for _ in 0..50 {
        aircraft.step_with_airspeed(
            airspeed_m_s,
            wind_xy,
            Vec3::new(0.0, 0.0, yaw_rate),
            MicrosDurationU32::from_millis(20),
        );
    }

    println!(
        "Fixed-wing model after 1.0 s: position=({:.2}, {:.2}, {:.2}) m",
        aircraft.pose.position.x, aircraft.pose.position.y, aircraft.pose.position.z
    );
    println!(
        "Fixed-wing ground velocity: ({:.2}, {:.2}, {:.2}) m/s",
        aircraft.ground_velocity.x, aircraft.ground_velocity.y, aircraft.ground_velocity.z
    );
}

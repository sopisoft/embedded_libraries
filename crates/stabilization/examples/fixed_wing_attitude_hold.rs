use control::{ControlAxes, ConventionalTailMixer};
use fugit::MicrosDurationU32;
use stabilization::{Attitude, AxisErrorMode, CascadeAttitudeController, CascadeAxis, Vector3};

fn main() {
    // This example shows a complete fixed-wing stabilization chain:
    // 1. start from a desired aircraft attitude,
    // 2. run cascaded attitude -> rate PID loops,
    // 3. feed the resulting axis commands into a conventional tail mixer.

    let mut roll_axis = CascadeAxis::new(
        control::PidController::new(5.0, 0.2, 0.0),
        control::PidController::new(0.8, 0.05, 0.01),
        2.5,
    );
    roll_axis.attitude_pid.set_output_limits(-2.5, 2.5);
    roll_axis.rate_pid.set_output_limits(-1.0, 1.0);

    let mut pitch_axis = CascadeAxis::new(
        control::PidController::new(6.0, 0.2, 0.0),
        control::PidController::new(0.9, 0.08, 0.02),
        2.0,
    );
    pitch_axis.attitude_pid.set_output_limits(-2.0, 2.0);
    pitch_axis.rate_pid.set_output_limits(-1.0, 1.0);

    let mut yaw_axis = CascadeAxis::new(
        control::PidController::new(3.0, 0.0, 0.0),
        control::PidController::new(0.4, 0.02, 0.0),
        1.5,
    )
    .with_error_mode(AxisErrorMode::WrappedAngle);
    yaw_axis.rate_pid.set_output_limits(-1.0, 1.0);

    let mut controller = CascadeAttitudeController::new(roll_axis, pitch_axis, yaw_axis);

    // Example target: a shallow right bank with a small nose-up pitch target.
    let target_roll = 20.0f32.to_radians();
    let target_pitch = 5.0f32.to_radians();
    let target_yaw_rate = 10.0f32.to_radians();

    // Example estimate from your AHRS and gyro:
    let measured_attitude = Attitude::new(
        8.0f32.to_radians(),
        1.0f32.to_radians(),
        15.0f32.to_radians(),
    );
    let measured_rates = Vector3::new(
        12.0f32.to_radians(),
        (-2.0f32).to_radians(),
        3.0f32.to_radians(),
    );

    let dt = MicrosDurationU32::from_millis(10);
    let stabilized = controller.update_fixed_wing(
        target_roll,
        target_pitch,
        target_yaw_rate,
        measured_attitude,
        measured_rates,
        dt,
    );

    // The inner loop produces generic roll / pitch / yaw control efforts.
    println!(
        "Desired body rates [deg/s]: roll={:.1}, pitch={:.1}, yaw={:.1}",
        stabilized.desired_rates_rad_s.x.to_degrees(),
        stabilized.desired_rates_rad_s.y.to_degrees(),
        stabilized.desired_rates_rad_s.z.to_degrees()
    );
    println!(
        "Controller outputs: roll={:.3}, pitch={:.3}, yaw={:.3}",
        stabilized.actuator.x, stabilized.actuator.y, stabilized.actuator.z
    );

    // In a real aircraft, these outputs usually go to a mixer next.
    let mixer = ConventionalTailMixer::new();
    let surfaces = mixer.mix(ControlAxes::new(
        stabilized.actuator.x,
        stabilized.actuator.y,
        stabilized.actuator.z,
        0.55,
        0.0,
    ));

    println!("Left aileron:  {:.3}", surfaces.left_aileron);
    println!("Right aileron: {:.3}", surfaces.right_aileron);
    println!("Elevator:      {:.3}", surfaces.elevator);
    println!("Rudder:        {:.3}", surfaces.rudder);
    println!("Throttle:      {:.3}", surfaces.throttle);
}

use control::{
    ControlAxes, ConventionalTailMixer, Normalized, PidController, SignedNormalized,
    SurfaceChannel, shape_rc_command,
};
use fugit::MicrosDurationU32;

fn main() {
    let pilot_roll = shape_rc_command(0.35, 0.05, 0.4, 0.8);
    let pilot_pitch = shape_rc_command(-0.20, 0.05, 0.3, 0.7);
    let pilot_yaw = shape_rc_command(0.10, 0.03, 0.2, 0.9);
    let pilot_throttle = 0.65;

    let mut pitch_rate_pid = PidController::new(0.8, 0.2, 0.02);
    pitch_rate_pid.set_output_limits(-1.0, 1.0);
    pitch_rate_pid.set_integral_limits(-0.4, 0.4);
    let dt = MicrosDurationU32::from_millis(20);
    let measured_pitch_rate = -0.05;
    let pitch_correction = pitch_rate_pid.update(pilot_pitch.get(), measured_pitch_rate, dt);

    let mixer = ConventionalTailMixer::new()
        .with_right_aileron(SurfaceChannel::new(1.0).with_reverse(true))
        .with_elevator(SurfaceChannel::new(0.8).with_trim(0.02))
        .with_differential(0.25)
        .with_flaperon_mix(0.2);

    let outputs = mixer.mix(ControlAxes::new(
        pilot_roll,
        SignedNormalized::saturated(pitch_correction),
        pilot_yaw,
        Normalized::saturated(pilot_throttle),
        Normalized::saturated(0.3),
    ));

    println!("Left aileron command:  {:.3}", outputs.left_aileron.get());
    println!("Right aileron command: {:.3}", outputs.right_aileron.get());
    println!("Elevator command:      {:.3}", outputs.elevator.get());
    println!("Rudder command:        {:.3}", outputs.rudder.get());
    println!("Throttle command:      {:.3}", outputs.throttle.get());
}

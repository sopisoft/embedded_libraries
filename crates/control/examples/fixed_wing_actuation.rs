use control::{
    ControlAxes, ConventionalTailMixer, PidController, SurfaceChannel, shape_rc_command,
};
use fugit::MicrosDurationU32;

fn main() {
    // This example shows the missing layer between estimation and actuators:
    // 1. shape pilot commands,
    // 2. run a control loop,
    // 3. mix the resulting axes into servo-friendly outputs.

    // Pretend these come from an RC receiver in normalized form.
    let pilot_roll = shape_rc_command(0.35, 0.05, 0.4, 0.8);
    let pilot_pitch = shape_rc_command(-0.20, 0.05, 0.3, 0.7);
    let pilot_yaw = shape_rc_command(0.10, 0.03, 0.2, 0.9);
    let pilot_throttle = 0.65;

    // Add a simple pitch-rate controller.
    let mut pitch_rate_pid = PidController::new(0.8, 0.2, 0.02);
    pitch_rate_pid.set_output_limits(-1.0, 1.0);
    pitch_rate_pid.set_integral_limits(-0.4, 0.4);
    let dt = MicrosDurationU32::from_millis(20);
    let measured_pitch_rate = -0.05;
    let pitch_correction = pitch_rate_pid.update(pilot_pitch, measured_pitch_rate, dt);

    // Configure the output mixer for a conventional fixed-wing tail.
    let mut mixer = ConventionalTailMixer::new();
    mixer.right_aileron.reversed = true;
    mixer.elevator = SurfaceChannel {
        scale: 0.8,
        trim: 0.02,
        reversed: false,
        min: -1.0,
        max: 1.0,
    };
    mixer.differential = 0.25;
    mixer.flaperon_mix = 0.2;

    let outputs = mixer.mix(ControlAxes::new(
        pilot_roll,
        pitch_correction,
        pilot_yaw,
        pilot_throttle,
        0.3,
    ));

    println!("Left aileron command:  {:.3}", outputs.left_aileron);
    println!("Right aileron command: {:.3}", outputs.right_aileron);
    println!("Elevator command:      {:.3}", outputs.elevator);
    println!("Rudder command:        {:.3}", outputs.rudder);
    println!("Throttle command:      {:.3}", outputs.throttle);
}

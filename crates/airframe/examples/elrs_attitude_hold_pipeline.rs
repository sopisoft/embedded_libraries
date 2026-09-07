use airframe::{AttitudeHoldLimits, FixedWingController, RcInputConfig, ServoMap};
use control::{ConventionalTailMixer, PidController};
use elrs::RcChannels;
use fugit::MicrosDurationU32;
use pwm::{ServoRange, ServoSet};
use stabilization::{AxisErrorMode, CascadeAttitudeController, CascadeAxis};

fn main() {
    let rc_config = RcInputConfig::conventional_aetr();

    let channels = RcChannels::from_micros([
        1_700, 1_450, 1_350, 1_520, 1_800, 1_250, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000,
        1_000, 1_000, 1_000,
    ]);
    let pilot = rc_config.decode(&channels);

    let mut roll = CascadeAxis::new(
        PidController::new(5.0, 0.2, 0.0),
        PidController::new(0.8, 0.05, 0.01),
        2.5,
    );
    roll.attitude_pid.set_output_limits(-2.5, 2.5);
    roll.rate_pid.set_output_limits(-1.0, 1.0);

    let mut pitch = CascadeAxis::new(
        PidController::new(6.0, 0.2, 0.0),
        PidController::new(0.9, 0.08, 0.02),
        2.0,
    );
    pitch.attitude_pid.set_output_limits(-2.0, 2.0);
    pitch.rate_pid.set_output_limits(-1.0, 1.0);

    let mut yaw = CascadeAxis::new(
        PidController::new(3.0, 0.0, 0.0),
        PidController::new(0.4, 0.02, 0.0),
        1.5,
    )
    .with_error_mode(AxisErrorMode::WrappedAngle);
    yaw.rate_pid.set_output_limits(-1.0, 1.0);

    let servos = ServoSet::new([
        ServoRange::default(),
        ServoRange::default(),
        ServoRange::default(),
        ServoRange::default(),
        ServoRange::new(
            MicrosDurationU32::from_micros(20_000),
            MicrosDurationU32::from_micros(1_000),
            MicrosDurationU32::from_micros(2_000),
            0.0,
            90.0,
        ),
        ServoRange::default(),
        ServoRange::default(),
    ]);

    let mut controller = FixedWingController::new(
        CascadeAttitudeController::new(roll, pitch, yaw),
        ConventionalTailMixer::new(),
        servos,
        ServoMap::conventional_7ch(),
        AttitudeHoldLimits::default(),
    );

    let measured_attitude = airframe::Vec3::new(
        10.0f32.to_radians(),
        2.0f32.to_radians(),
        30.0f32.to_radians(),
    );
    let measured_rates = airframe::Vec3::new(0.15, -0.05, 0.04);

    let output = controller.update_selected(
        pilot,
        measured_attitude,
        measured_rates,
        MicrosDurationU32::from_millis(10),
    );

    println!("Vec3-hold enabled: {}", pilot.attitude_hold_enabled);
    println!(
        "Surface commands: ailL={:.3} ailR={:.3} ele={:.3} rud={:.3} thr={:.3}",
        output.surfaces.left_aileron.get(),
        output.surfaces.right_aileron.get(),
        output.surfaces.elevator.get(),
        output.surfaces.rudder.get(),
        output.surfaces.throttle.get()
    );
    println!(
        "Servo pulses [us]: {:?}",
        output.pulses.map(|pulse| pulse.as_micros())
    );
}

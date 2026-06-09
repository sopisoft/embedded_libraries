use airframe::{AttitudeHoldLimits, ElevonController, ElevonServoMap, RcInputConfig};
use control::{ElevonMixer, PidController};
use elrs::RcChannels;
use fugit::MicrosDurationU32;
use math::{EulerAngles, Vec3};
use pwm::{ServoRange, ServoSet};
use stabilization::{AxisErrorMode, CascadeAttitudeController, CascadeAxis};

fn main() {
    // This example targets delta-wing or flying-wing aircraft with two elevons.
    // The controller handles:
    // 1. ELRS stick decoding,
    // 2. optional roll/pitch attitude hold,
    // 3. final left/right elevon commands plus throttle pulse generation.

    let rc_config = RcInputConfig::conventional_aetr();
    let channels = RcChannels::from_micros([
        1_650, 1_420, 1_400, 1_500, 1_800, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000,
        1_000, 1_000, 1_000,
    ]);
    let pilot = rc_config.decode(&channels);

    let mut roll = CascadeAxis::new(
        PidController::new(5.0, 0.1, 0.0),
        PidController::new(0.8, 0.05, 0.01),
        2.5,
    );
    roll.attitude_pid.set_output_limits(-2.5, 2.5);
    roll.rate_pid.set_output_limits(-1.0, 1.0);

    let mut pitch = CascadeAxis::new(
        PidController::new(6.0, 0.1, 0.0),
        PidController::new(0.9, 0.06, 0.01),
        2.0,
    );
    pitch.attitude_pid.set_output_limits(-2.0, 2.0);
    pitch.rate_pid.set_output_limits(-1.0, 1.0);

    let mut yaw = CascadeAxis::new(
        PidController::new(1.0, 0.0, 0.0),
        PidController::new(0.2, 0.0, 0.0),
        1.0,
    )
    .with_error_mode(AxisErrorMode::WrappedAngle);
    yaw.rate_pid.set_output_limits(-1.0, 1.0);

    let base = ServoRange::default();
    let servos = ServoSet::new([
        base,
        base,
        ServoRange::new(
            MicrosDurationU32::from_micros(20_000),
            MicrosDurationU32::from_micros(1_000),
            MicrosDurationU32::from_micros(2_000),
            0.0,
            90.0,
        ),
    ]);

    let mut controller = ElevonController::new(
        CascadeAttitudeController::new(roll, pitch, yaw),
        ElevonMixer::new(),
        servos,
        ElevonServoMap::three_channel(),
        AttitudeHoldLimits::default(),
    );

    let output = controller.update_selected(
        pilot,
        EulerAngles::from_degrees(5.0, 1.0, 20.0),
        Vec3::new(0.10, -0.02, 0.01),
        MicrosDurationU32::from_millis(10),
    );

    println!(
        "Elevons: left={:.3} right={:.3} throttle={:.3}",
        output.surfaces.left_elevon, output.surfaces.right_elevon, output.surfaces.throttle
    );
    println!(
        "Pulse widths [us]: {:?}",
        output.pulses.map(|pulse| pulse.as_micros())
    );
}

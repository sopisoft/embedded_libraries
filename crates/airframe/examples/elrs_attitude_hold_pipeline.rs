use airframe::{AttitudeHoldLimits, FixedWingController, RcInputConfig, ServoMap};
use control::{ConventionalTailMixer, PidController};
use elrs::RcChannels;
use fugit::MicrosDurationU32;
use math::{EulerAngles, Vec3};
use pwm::{ServoRange, ServoSet};
use stabilization::{AxisErrorMode, CascadeAttitudeController, CascadeAxis};

fn main() {
    // This example shows the main benefit of the `airframe` crate:
    // it removes boilerplate between four independent domains:
    //
    // 1. ELRS / CRSF receiver channels,
    // 2. pilot input shaping and mode selection,
    // 3. cascaded attitude hold,
    // 4. final surface mixing and servo pulse generation.
    //
    // The estimator is intentionally not hard-coded here. You can feed attitude
    // and body rates from any IMU / AHRS stack you prefer.

    let rc_config = RcInputConfig::conventional_aetr();

    // Example ELRS packet:
    // CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw, CH5 mode switch, CH6 flaps.
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

    // These come from your estimator, not from the receiver.
    let measured_attitude = EulerAngles::from_degrees(10.0, 2.0, 30.0);
    let measured_rates = Vec3::new(0.15, -0.05, 0.04);

    let output = controller.update_selected(
        pilot,
        measured_attitude,
        measured_rates,
        MicrosDurationU32::from_millis(10),
    );

    println!("Attitude-hold enabled: {}", pilot.attitude_hold_enabled);
    println!(
        "Surface commands: ailL={:.3} ailR={:.3} ele={:.3} rud={:.3} thr={:.3}",
        output.surfaces.left_aileron,
        output.surfaces.right_aileron,
        output.surfaces.elevator,
        output.surfaces.rudder,
        output.surfaces.throttle
    );
    println!(
        "Servo pulses [us]: {:?}",
        output.pulses.map(|pulse| pulse.as_micros())
    );

    // Those pulse widths can be written directly to `pwm::ServoBank` in your
    // board-specific firmware.
}

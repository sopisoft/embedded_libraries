use airframe::{AttitudeHoldLimits, RcInputConfig, VTailController, VTailServoMap};
use control::{PidController, VTailMixer};
use elrs::RcChannels;
use fugit::MicrosDurationU32;
use math::{EulerAngles, Vec3};
use pwm::{ServoRange, ServoSet};
use stabilization::{AxisErrorMode, CascadeAttitudeController, CascadeAxis};

fn main() {
    // This example targets V-tail aircraft.
    // The controller converts the same pilot command style into:
    // - aileron output,
    // - left/right V-tail surfaces,
    // - throttle pulse output.

    let rc_config = RcInputConfig::conventional_aetr();
    let channels = RcChannels::from_micros([
        1_550, 1_600, 1_350, 1_700, 1_800, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000,
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
        PidController::new(3.0, 0.0, 0.0),
        PidController::new(0.4, 0.02, 0.0),
        1.5,
    )
    .with_error_mode(AxisErrorMode::WrappedAngle);
    yaw.rate_pid.set_output_limits(-1.0, 1.0);

    let servos = ServoSet::new([ServoRange::default(); 4]);
    let mut controller = VTailController::new(
        CascadeAttitudeController::new(roll, pitch, yaw),
        VTailMixer::new(),
        servos,
        VTailServoMap::four_channel(),
        AttitudeHoldLimits::default(),
    );

    let output = controller.update_selected(
        pilot,
        EulerAngles::from_degrees(3.0, 0.0, 15.0),
        Vec3::new(0.05, -0.03, 0.08),
        MicrosDurationU32::from_millis(10),
    );

    println!(
        "V-tail: left={:.3} right={:.3} aileron={:.3} throttle={:.3}",
        output.surfaces.left_tail,
        output.surfaces.right_tail,
        output.surfaces.aileron,
        output.surfaces.throttle
    );
    println!(
        "Pulse widths [us]: {:?}",
        output.pulses.map(|pulse| pulse.as_micros())
    );
}

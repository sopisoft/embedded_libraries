use airframe::{AttitudeHoldLimits, FixedWingController, RcInputConfig, ServoMap};
use control::ConventionalTailMixer;
use elrs::RcChannels;
use fugit::MicrosDurationU32;
use indi::{IndiAttitudeConfig, IndiAttitudeController, IndiAxisConfig, IndiRateController};
use math::{EulerAngles, Vec3};
use pwm::{ServoRange, ServoSet};

fn main() {
    // This example uses the same high-level airframe pipeline as the cascaded
    // PID examples, but the attitude backend is INDI.
    //
    // In real firmware, the measured attitude and rates come from your AHRS.
    // INDI works best when gyro rates are high quality and actuator response is
    // reasonably repeatable around the current flight condition.

    let rc_config = RcInputConfig::conventional_aetr();
    let channels = RcChannels::from_micros([
        1_700, 1_450, 1_350, 1_520, 1_800, 1_250, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000,
        1_000, 1_000, 1_000,
    ]);
    let pilot = rc_config.decode(&channels);

    let roll = IndiAxisConfig::symmetric(18.0, 7.0, 35.0, 18.0);
    let pitch = IndiAxisConfig::symmetric(16.0, 7.5, 30.0, 18.0);
    let yaw = IndiAxisConfig::symmetric(10.0, 5.0, 20.0, 12.0);
    let attitude_hold = IndiAttitudeController::new(
        IndiRateController::from_configs(roll, pitch, yaw),
        IndiAttitudeConfig::fixed_wing_default(),
    );

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
        attitude_hold,
        ConventionalTailMixer::new(),
        servos,
        ServoMap::conventional_7ch(),
        AttitudeHoldLimits::default(),
    );

    let output = controller.update_selected(
        pilot,
        EulerAngles::from_degrees(10.0, 2.0, 30.0),
        Vec3::new(0.15, -0.05, 0.04),
        MicrosDurationU32::from_millis(10),
    );

    println!(
        "INDI attitude-hold enabled: {}",
        pilot.attitude_hold_enabled
    );
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
}

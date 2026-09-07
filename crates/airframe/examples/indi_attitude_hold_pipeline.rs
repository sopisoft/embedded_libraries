use airframe::{AttitudeHoldLimits, FixedWingController, RcInputConfig, ServoMap};
use control::ConventionalTailMixer;
use elrs::RcChannels;
use fugit::MicrosDurationU32;
use indi::{IndiAttitudeConfig, IndiAttitudeController, IndiAxisConfig, IndiRateController};
use pwm::{ServoRange, ServoSet};

fn main() {
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
        airframe::Vec3::new(
            10.0f32.to_radians(),
            2.0f32.to_radians(),
            30.0f32.to_radians(),
        ),
        airframe::Vec3::new(0.15, -0.05, 0.04),
        MicrosDurationU32::from_millis(10),
    );

    println!(
        "INDI attitude-hold enabled: {}",
        pilot.attitude_hold_enabled
    );
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

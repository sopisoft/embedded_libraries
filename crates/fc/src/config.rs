use airframe::{AttitudeHoldLimits, RcChannel, RcInputConfig, ServoMap};
use control::ConventionalTailMixer;
use control::PidController;
use fugit::MicrosDurationU32;
use pwm::{ServoRange, ServoSet};
use stabilization::{AxisErrorMode, CascadeAttitudeController, CascadeAxis};
use tecs::{TecsConfig, TecsController};

/// Number of conventional fixed-wing outputs.
pub const OUTPUT_COUNT: usize = 5;

/// Default CRSF-to-output control period.
pub const CONTROL_PERIOD: MicrosDurationU32 = MicrosDurationU32::from_millis(10);

/// Default time after which a missing RC frame activates failsafe.
pub const DEFAULT_FAILSAFE_TIMEOUT: MicrosDurationU32 = MicrosDurationU32::from_millis(100);

pub const GS1502_CHANNEL: RcChannel = RcChannel::Ch5;
pub const GS1502_SWITCH_THRESHOLD_US: u16 = 1_600;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum OutputChannel {
    LeftAileron = 0,
    RightAileron = 1,
    Elevator = 2,
    Rudder = 3,
    Throttle = 4,
}

impl OutputChannel {
    pub const fn index(self) -> usize {
        self as usize
    }
}

/// Runtime settings for a five-output conventional fixed-wing controller.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Config {
    pub rc: RcInputConfig,
    pub attitude_controller: CascadeAttitudeController,
    pub altitude_controller: TecsController,
    pub attitude_correction_gain: f32,
    pub failsafe_timeout: MicrosDurationU32,
    pub attitude_limits: AttitudeHoldLimits,
    pub mixer: ConventionalTailMixer,
    pub servos: ServoSet<OUTPUT_COUNT>,
    pub servo_map: ServoMap,
    pub gs1502_channel: RcChannel,
    pub gs1502_switch_threshold_us: u16,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            rc: RcInputConfig::conventional_aetr(),
            attitude_controller: default_attitude_controller(),
            altitude_controller: default_altitude_controller(),
            attitude_correction_gain: 0.08,
            failsafe_timeout: DEFAULT_FAILSAFE_TIMEOUT,
            attitude_limits: AttitudeHoldLimits::default(),
            mixer: ConventionalTailMixer::new(),
            servos: default_servos(),
            servo_map: ServoMap::conventional_5ch(),
            gs1502_channel: GS1502_CHANNEL,
            gs1502_switch_threshold_us: GS1502_SWITCH_THRESHOLD_US,
        }
    }
}

/// Standard 20 ms servo ranges for the five conventional outputs.
pub fn default_servos() -> ServoSet<OUTPUT_COUNT> {
    let standard = ServoRange::new(
        MicrosDurationU32::from_micros(20_000),
        MicrosDurationU32::from_micros(1_000),
        MicrosDurationU32::from_micros(2_000),
        -60.0,
        60.0,
    );
    ServoSet::new([standard; OUTPUT_COUNT])
}

fn default_attitude_controller() -> CascadeAttitudeController {
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

    CascadeAttitudeController::new(roll, pitch, yaw)
}

fn default_altitude_controller() -> TecsController {
    let mut total_energy = PidController::new(0.004, 0.001, 0.0);
    total_energy.set_output_limits(-0.35, 0.35);
    total_energy.set_integral_limits(-50.0, 50.0);

    let mut energy_balance = PidController::new(0.003, 0.0005, 0.0);
    energy_balance.set_output_limits(-0.3, 0.3);
    energy_balance.set_integral_limits(-50.0, 50.0);

    TecsController::new(
        total_energy,
        energy_balance,
        TecsConfig {
            throttle_min: 0.5,
            pitch_min_rad: -20.0f32.to_radians(),
            pitch_max_rad: 20.0f32.to_radians(),
            speed_weight: 0.0,
            ..TecsConfig::default()
        },
    )
}

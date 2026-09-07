use airframe::{
    FixedWingControlOutput, FixedWingController, PilotCommand, RcChannel, RcInputConfig,
};
use elrs::{
    FRAME_TYPE_RC_CHANNELS_PACKED, FRAME_TYPE_SUBSET_RC_CHANNELS_PACKED, FrameParser, RcChannels,
    SubsetRcChannels,
};
use fugit::MicrosDurationU32;
use imu::{ImuEstimate, MargEstimator, MargSample};
use tecs::{TecsController, TecsOutput, TecsState, TecsTarget};

use crate::config::{Config, OUTPUT_COUNT};

/// Controller state selected for the current update.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Mode {
    Manual,
    AttitudeHold,
    AltitudeHold,
    Failsafe,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Gs1502Position {
    Retracted,
    Extended,
}

impl Gs1502Position {
    pub const fn normalized(self) -> control::Normalized {
        match self {
            Self::Retracted => control::Normalized::ZERO,
            Self::Extended => control::Normalized::ONE,
        }
    }
}

/// Result of one complete sensor/control update.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Output {
    pub mode: Mode,
    pub pilot: PilotCommand,
    pub estimate: ImuEstimate,
    pub control: FixedWingControlOutput<OUTPUT_COUNT>,
    pub gs1502_position: Gs1502Position,
    pub altitude_target_m: Option<f32>,
}

/// CRSF, attitude estimation, failsafe, and conventional-tail control runtime.
#[derive(Debug)]
pub struct FlightController {
    estimator: MargEstimator,
    controller: FixedWingController<OUTPUT_COUNT>,
    altitude_hold: AltitudeHold,
    rc: RcInputConfig,
    receiver: CrsfReceiver,
    gs1502_channel: RcChannel,
    gs1502_switch_threshold_us: u16,
}

#[derive(Debug)]
struct AltitudeHold {
    controller: TecsController,
    target_m: Option<f32>,
}

#[derive(Debug)]
struct CrsfReceiver {
    parser: FrameParser,
    channels: RcChannels,
    last_rc_us: Option<u32>,
    failsafe_timeout_us: u32,
}

impl FlightController {
    /// Creates a controller with the supplied settings.
    pub fn new(config: Config) -> Self {
        Self {
            estimator: MargEstimator::with_attitude_correction_gain(
                config.attitude_correction_gain,
            ),
            controller: FixedWingController::new(
                config.attitude_controller,
                config.mixer,
                config.servos,
                config.servo_map,
                config.attitude_limits,
            ),
            altitude_hold: AltitudeHold::new(config.altitude_controller),
            rc: config.rc,
            receiver: CrsfReceiver::new(config.failsafe_timeout.as_micros()),
            gs1502_channel: config.gs1502_channel,
            gs1502_switch_threshold_us: config.gs1502_switch_threshold_us,
        }
    }

    /// Feeds one UART byte and timestamps valid RC frames.
    pub fn push_crsf_byte(&mut self, byte: u8, now_us: u32) -> bool {
        self.receiver.push(byte, now_us)
    }

    /// Returns whether a valid RC frame is still fresh at `now_us`.
    pub fn rc_is_alive(&self, now_us: u32) -> bool {
        self.receiver.is_alive(now_us)
    }

    /// Runs estimation, mode selection, mixing, and pulse generation.
    pub fn update(&mut self, sample: MargSample, now_us: u32, dt: MicrosDurationU32) -> Output {
        self.update_with_altitude(sample, None, now_us, dt)
    }

    pub fn update_with_altitude(
        &mut self,
        sample: MargSample,
        barometric_altitude_m: Option<f32>,
        now_us: u32,
        dt: MicrosDurationU32,
    ) -> Output {
        let estimate = self.estimator.update_marg(sample, dt);
        let pilot = self.rc.decode(&self.receiver.channels);
        if !self.rc_is_alive(now_us) {
            self.controller.attitude_hold.reset();
            self.altitude_hold.reset();
            let pilot = failsafe_pilot();
            return Output {
                mode: Mode::Failsafe,
                pilot,
                estimate,
                control: self.controller.update_manual(pilot),
                gs1502_position: Gs1502Position::Retracted,
                altitude_target_m: None,
            };
        }
        let measured_rates = sample.accel_gyro.gyro_rad_s.vector() - self.estimator.gyro_bias();
        let altitude = barometric_altitude_m.filter(|altitude| altitude.is_finite());
        let (control, mode) = if pilot.attitude_hold_enabled {
            if let Some(altitude_m) = altitude {
                let tecs = self.altitude_hold.update(
                    altitude_m,
                    pilot.throttle.get(),
                    estimate.euler.y,
                    dt,
                );
                let pitch_limit = self.controller.limits.max_pitch_rad();
                let pitch = if pitch_limit > f32::EPSILON {
                    tecs.pitch_rad / pitch_limit
                } else {
                    0.0
                };
                let controlled_pilot = PilotCommand::new(
                    pilot.roll.get(),
                    pitch,
                    pilot.yaw.get(),
                    tecs.throttle,
                    pilot.flaps.get(),
                    true,
                );
                (
                    self.controller.update_selected(
                        controlled_pilot,
                        estimate.euler,
                        measured_rates,
                        dt,
                    ),
                    Mode::AltitudeHold,
                )
            } else {
                self.altitude_hold.reset();
                (
                    self.controller
                        .update_selected(pilot, estimate.euler, measured_rates, dt),
                    Mode::AttitudeHold,
                )
            }
        } else {
            self.altitude_hold.reset();
            (self.controller.update_manual(pilot), Mode::Manual)
        };
        Output {
            mode,
            pilot,
            estimate,
            control,
            gs1502_position: if self
                .receiver
                .channels
                .micros(self.gs1502_channel.index())
                .unwrap_or(1_000)
                >= self.gs1502_switch_threshold_us
            {
                Gs1502Position::Extended
            } else {
                Gs1502Position::Retracted
            },
            altitude_target_m: self.altitude_hold.target_m,
        }
    }

    /// Returns the most recently decoded channels.
    pub const fn channels(&self) -> &RcChannels {
        &self.receiver.channels
    }
}

impl AltitudeHold {
    const fn new(controller: TecsController) -> Self {
        Self {
            controller,
            target_m: None,
        }
    }

    fn update(
        &mut self,
        altitude_m: f32,
        throttle_trim: f32,
        pitch_trim_rad: f32,
        dt: MicrosDurationU32,
    ) -> TecsOutput {
        let target_m = *self.target_m.get_or_insert_with(|| {
            let mut config = self.controller.config();
            config.throttle_trim = throttle_trim;
            config.pitch_trim_rad =
                pitch_trim_rad.clamp(config.pitch_min_rad, config.pitch_max_rad);
            self.controller.set_config(config);
            self.controller.reset();
            altitude_m
        });
        self.controller.update(
            TecsTarget::new(target_m, 0.0),
            TecsState::new(altitude_m, 0.0),
            dt,
        )
    }

    fn reset(&mut self) {
        if self.target_m.take().is_some() {
            self.controller.reset();
        }
    }
}

impl CrsfReceiver {
    fn new(failsafe_timeout_us: u32) -> Self {
        Self {
            parser: FrameParser::new(),
            channels: neutral_channels(),
            last_rc_us: None,
            failsafe_timeout_us,
        }
    }

    fn push(&mut self, byte: u8, now_us: u32) -> bool {
        let Some(result) = self.parser.push(byte) else {
            return false;
        };
        let Ok(frame) = result else {
            return false;
        };
        let updated = match frame.frame_type {
            FRAME_TYPE_RC_CHANNELS_PACKED => {
                let Ok(payload) = frame.payload().try_into() else {
                    return false;
                };
                self.channels = RcChannels::unpack(payload);
                true
            }
            FRAME_TYPE_SUBSET_RC_CHANNELS_PACKED => {
                let Ok(subset) = SubsetRcChannels::decode(frame.payload()) else {
                    return false;
                };
                airframe::apply_subset_channels(&mut self.channels, &subset);
                true
            }
            _ => false,
        };
        if updated {
            self.last_rc_us = Some(now_us);
        }
        updated
    }

    fn is_alive(&self, now_us: u32) -> bool {
        self.last_rc_us
            .is_some_and(|last| now_us.wrapping_sub(last) <= self.failsafe_timeout_us)
    }
}

fn neutral_channels() -> RcChannels {
    RcChannels::from_micros([
        1_500, 1_500, 1_000, 1_500, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000,
        1_000, 1_000, 1_000,
    ])
}

fn failsafe_pilot() -> PilotCommand {
    PilotCommand::new(0.0, 0.0, 0.0, 0.5, 0.0, false)
}

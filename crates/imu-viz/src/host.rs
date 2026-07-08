use std::io::{BufReader, Read};
use std::process::{Child, Command, Stdio};
use std::sync::mpsc::{self, Sender};
use std::thread;

use eframe::egui;
use glam::{EulerRot, Quat, Vec3};

mod app;
mod draw;
mod parse;
mod plots;
mod ui;

use app::ImuVizApp;
use parse::{parse_args, push_line_from_bytes};

pub(crate) const HISTORY_LIMIT: usize = 600;
pub(crate) const LOG_LIMIT: usize = 24;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum Preset {
    ImuOnly,
    Fusion,
}

impl Preset {
    fn from_cli(value: &str) -> Result<Self, String> {
        match value {
            "imu" | "imu-only" => Ok(Self::ImuOnly),
            "fusion" => Ok(Self::Fusion),
            _ => Err(format!(
                "unknown mode `{value}`; expected `imu` or `fusion`"
            )),
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::ImuOnly => "imu",
            Self::Fusion => "fusion",
        }
    }

    fn command(self) -> Vec<String> {
        match self {
            Self::ImuOnly => vec![
                "cargo".into(),
                "run".into(),
                "-p".into(),
                "imu".into(),
                "--example".into(),
                "rp235x_stemma_qt_9dof".into(),
                "--target".into(),
                "thumbv8m.main-none-eabihf".into(),
            ],
            Self::Fusion => vec![
                "cargo".into(),
                "run".into(),
                "-p".into(),
                "imu".into(),
                "--example".into(),
                "rp235x_stemma_qt_9dof_lps25hb".into(),
                "--target".into(),
                "thumbv8m.main-none-eabihf".into(),
            ],
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) enum CommandSource {
    Preset(Preset),
    Custom,
}

impl CommandSource {
    pub(crate) fn label(&self) -> &'static str {
        match self {
            Self::Preset(preset) => preset.label(),
            Self::Custom => "custom",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct LaunchConfig {
    pub(crate) source: CommandSource,
    pub(crate) command: Vec<String>,
}

#[derive(Debug)]
pub(crate) enum StartupAction {
    Run(LaunchConfig),
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Sample {
    pub(crate) elapsed_ms: u32,
    pub(crate) orientation: Quat,
    pub(crate) roll_deg: f32,
    pub(crate) pitch_deg: f32,
    pub(crate) yaw_deg: f32,
    pub(crate) velocity_world_m_s: Vec3,
    pub(crate) altitude_m: f32,
}

impl Default for Sample {
    fn default() -> Self {
        Self {
            elapsed_ms: 0,
            orientation: Quat::IDENTITY,
            roll_deg: 0.0,
            pitch_deg: 0.0,
            yaw_deg: 0.0,
            velocity_world_m_s: Vec3::ZERO,
            altitude_m: 0.0,
        }
    }
}

impl Sample {
    pub(crate) fn from_quaternion_log(
        elapsed_ms: u32,
        orientation: Quat,
        velocity_world_m_s: Vec3,
        altitude_m: f32,
    ) -> Self {
        let orientation = if orientation.length_squared() > 1.0e-12 {
            orientation.normalize()
        } else {
            Quat::IDENTITY
        };
        let (roll_rad, pitch_rad, yaw_rad) = orientation.to_euler(EulerRot::XYZ);
        Self {
            elapsed_ms,
            orientation,
            roll_deg: roll_rad.to_degrees(),
            pitch_deg: pitch_rad.to_degrees(),
            yaw_deg: yaw_rad.to_degrees(),
            velocity_world_m_s,
            altitude_m,
        }
    }

    pub(crate) fn summary_line(self) -> String {
        format!(
            "t={:.2}s attitude=({:.1}, {:.1}, {:.1}) deg velocity=({:.2}, {:.2}, {:.2}) m/s altitude={:.3} m",
            self.elapsed_ms as f32 / 1000.0,
            self.roll_deg,
            self.pitch_deg,
            self.yaw_deg,
            self.velocity_world_m_s.x,
            self.velocity_world_m_s.y,
            self.velocity_world_m_s.z,
            self.altitude_m,
        )
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum FeatureState {
    Unknown,
    Enabled,
    Disabled,
}

impl FeatureState {
    pub(crate) fn label(self) -> &'static str {
        match self {
            Self::Unknown => "unknown",
            Self::Enabled => "enabled",
            Self::Disabled => "disabled",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum YawState {
    Unknown,
    Relative,
    Absolute,
}

impl YawState {
    pub(crate) fn label(self) -> &'static str {
        match self {
            Self::Unknown => "unknown",
            Self::Relative => "relative",
            Self::Absolute => "absolute",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct FeatureStatus {
    pub(crate) yaw: YawState,
    pub(crate) magnetometer: FeatureState,
    pub(crate) barometer: FeatureState,
}

impl FeatureStatus {
    pub(crate) fn from_launch(launch: &LaunchConfig) -> Self {
        match launch.source {
            CommandSource::Preset(Preset::ImuOnly) => Self {
                yaw: YawState::Relative,
                magnetometer: FeatureState::Unknown,
                barometer: FeatureState::Disabled,
            },
            CommandSource::Preset(Preset::Fusion) => Self {
                yaw: YawState::Relative,
                magnetometer: FeatureState::Unknown,
                barometer: FeatureState::Unknown,
            },
            CommandSource::Custom => Self {
                yaw: YawState::Unknown,
                magnetometer: FeatureState::Unknown,
                barometer: FeatureState::Unknown,
            },
        }
    }

    pub(crate) fn update_from_log(&mut self, line: &str) {
        if line.contains("LIS3MDL WHO_AM_I=") || line.contains("mag re-enabled") {
            self.magnetometer = FeatureState::Enabled;
        }
        if line.contains("mag init failed") || line.contains("mag read failed") {
            self.magnetometer = FeatureState::Disabled;
            self.yaw = YawState::Relative;
        }
        if line.contains("mag calibration ready") {
            self.magnetometer = FeatureState::Enabled;
            self.yaw = YawState::Absolute;
        }
        if line.contains("LPS25HB WHO_AM_I=") {
            self.barometer = FeatureState::Enabled;
        }
    }
}

#[derive(Debug)]
pub(crate) enum AppEvent {
    Log(String),
    Sample(Sample),
}

fn spawn_reader<R: Read + Send + 'static>(reader: R, tx: Sender<AppEvent>) {
    thread::spawn(move || {
        let mut reader = BufReader::new(reader);
        let mut chunk = [0u8; 1024];
        let mut pending = Vec::new();

        loop {
            match reader.read(&mut chunk) {
                Ok(0) => {
                    push_line_from_bytes(&mut pending, &tx);
                    break;
                }
                Ok(count) => {
                    for byte in &chunk[..count] {
                        if matches!(*byte, b'\n' | b'\r') {
                            push_line_from_bytes(&mut pending, &tx);
                        } else {
                            pending.push(*byte);
                        }
                    }
                }
                Err(error) => {
                    let _ = tx.send(AppEvent::Log(format!("reader error: {error}")));
                    break;
                }
            }
        }
    });
}

fn spawn_child(command: &[String], tx: Sender<AppEvent>) -> Result<Child, String> {
    let program = command
        .first()
        .ok_or_else(|| "empty command".to_string())?
        .clone();
    let mut child = Command::new(program);
    child
        .args(&command[1..])
        .env("CARGO_TERM_COLOR", "never")
        .env("CARGO_TERM_PROGRESS_WHEN", "never")
        .env("CLICOLOR", "0")
        .env("NO_COLOR", "1")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());

    let mut child = child
        .spawn()
        .map_err(|error| format!("failed to spawn command: {error}"))?;

    if let Some(stdout) = child.stdout.take() {
        spawn_reader(stdout, tx.clone());
    }
    if let Some(stderr) = child.stderr.take() {
        spawn_reader(stderr, tx.clone());
    }

    Ok(child)
}

pub fn run() -> Result<(), String> {
    let StartupAction::Run(launch) = parse_args()?;

    let (tx, rx) = mpsc::channel();
    let child = spawn_child(&launch.command, tx)?;

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("imu-viz")
            .with_inner_size([1400.0, 900.0]),
        ..Default::default()
    };

    eframe::run_native(
        "imu-viz",
        options,
        Box::new(move |_cc| Ok(Box::new(ImuVizApp::new(launch, rx, child)))),
    )
    .map_err(|error| error.to_string())
}

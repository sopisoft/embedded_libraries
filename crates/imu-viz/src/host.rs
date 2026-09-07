use std::io::{BufReader, ErrorKind, Read};
use std::sync::mpsc::{self, SyncSender};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::thread;
use std::time::Duration;

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
const MAX_LINE_LENGTH: usize = 512;
const EVENT_QUEUE_LIMIT: usize = 256;

#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct PortConfig {
    pub(crate) path: String,
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
    ) -> Option<Self> {
        let norm_squared = orientation.length_squared();
        if !norm_squared.is_finite() || norm_squared <= 1.0e-12 {
            return None;
        }
        let orientation = orientation.normalize();
        let (roll_rad, pitch_rad, yaw_rad) = orientation.to_euler(EulerRot::XYZ);
        Some(Self {
            elapsed_ms,
            orientation,
            roll_deg: roll_rad.to_degrees(),
            pitch_deg: pitch_rad.to_degrees(),
            yaw_deg: yaw_rad.to_degrees(),
            velocity_world_m_s,
            altitude_m,
        })
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
}

impl FeatureStatus {
    pub(crate) const fn unknown() -> Self {
        Self {
            yaw: YawState::Unknown,
            magnetometer: FeatureState::Unknown,
        }
    }

    pub(crate) fn update_from_log(&mut self, line: &str) {
        if line.contains("mag_enabled=true") {
            self.magnetometer = FeatureState::Enabled;
            self.yaw = YawState::Relative;
        }
        if line.contains("mag_enabled=false") {
            self.magnetometer = FeatureState::Disabled;
            self.yaw = YawState::Relative;
        }
        if line.contains("mag_ready=true") {
            self.magnetometer = FeatureState::Enabled;
            self.yaw = YawState::Absolute;
        }
    }
}

#[derive(Debug)]
pub(crate) enum AppEvent {
    Log(String),
    Sample(Sample, FeatureStatus),
    Disconnected(String),
}

fn spawn_reader<R: Read + Send + 'static>(
    reader: R,
    tx: SyncSender<AppEvent>,
    stop: Arc<AtomicBool>,
) {
    thread::spawn(move || {
        let mut reader = BufReader::new(reader);
        let mut chunk = [0u8; 1024];
        let mut pending = Vec::new();

        while stop.load(Ordering::Relaxed) {
            match reader.read(&mut chunk) {
                Ok(0) => {
                    push_line_from_bytes(&mut pending, &tx);
                    let _ = tx.try_send(AppEvent::Disconnected("serial port closed".into()));
                    break;
                }
                Ok(count) => {
                    for byte in &chunk[..count] {
                        if matches!(*byte, b'\n' | b'\r') {
                            push_line_from_bytes(&mut pending, &tx);
                        } else if pending.len() < MAX_LINE_LENGTH {
                            pending.push(*byte);
                        }
                    }
                }
                Err(error)
                    if matches!(error.kind(), ErrorKind::TimedOut | ErrorKind::WouldBlock) => {}
                Err(error) => {
                    let _ = tx.try_send(AppEvent::Log(format!("reader error: {error}")));
                    let _ = tx.try_send(AppEvent::Disconnected(error.to_string()));
                    break;
                }
            }
        }
    });
}

pub fn run() -> Result<(), String> {
    let port = parse_args()?;
    let serial = serialport::new(&port.path, 115_200)
        .timeout(Duration::from_millis(100))
        .open()
        .map_err(|error| format!("failed to open {}: {error}", port.path))?;

    let (tx, rx) = mpsc::sync_channel(EVENT_QUEUE_LIMIT);
    let stop = Arc::new(AtomicBool::new(true));
    spawn_reader(serial, tx, stop.clone());

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("imu-viz")
            .with_inner_size([1400.0, 900.0]),
        ..Default::default()
    };

    eframe::run_native(
        "imu-viz",
        options,
        Box::new(move |_cc| Ok(Box::new(ImuVizApp::new(port, rx, stop)))),
    )
    .map_err(|error| error.to_string())
}

use std::sync::mpsc::Sender;

use glam::{Quat, Vec3};

use super::{AppEvent, CommandSource, LaunchConfig, Preset, Sample, StartupAction};

pub(crate) fn parse_sample(line: &str) -> Option<Sample> {
    let elapsed_ms = parse_u32_after(line, "t_ms=")?;
    let quat = parse_tuple4_after(line, "quat=(")?;
    let velocity = parse_tuple3_after(line, "velocity_m_s=(").unwrap_or([0.0, 0.0, 0.0]);
    let altitude_m = parse_f32_after(line, "altitude_m=")?;
    let orientation = Quat::from_xyzw(quat[1], quat[2], quat[3], quat[0]);

    Some(Sample::from_quaternion_log(
        elapsed_ms,
        orientation,
        Vec3::from_array(velocity),
        altitude_m,
    ))
}

fn parse_u32_field(field: &str) -> Option<u32> {
    field.split_ascii_whitespace().next()?.parse().ok()
}

fn parse_f32_field(field: &str) -> Option<f32> {
    field.split_ascii_whitespace().next()?.parse().ok()
}

fn parse_u32_after(line: &str, key: &str) -> Option<u32> {
    parse_u32_field(line.split_once(key)?.1)
}

fn parse_f32_after(line: &str, key: &str) -> Option<f32> {
    parse_f32_field(line.split_once(key)?.1)
}

fn parse_tuple4_after(line: &str, key: &str) -> Option<[f32; 4]> {
    let values = line.split_once(key)?.1;
    let end = values.find(')')?;
    let mut fields = values[..end].split(',');
    Some([
        parse_f32_field(fields.next()?)?,
        parse_f32_field(fields.next()?)?,
        parse_f32_field(fields.next()?)?,
        parse_f32_field(fields.next()?)?,
    ])
}

fn parse_tuple3_after(line: &str, key: &str) -> Option<[f32; 3]> {
    let values = line.split_once(key)?.1;
    let end = values.find(')')?;
    let mut fields = values[..end].split(',');
    Some([
        parse_f32_field(fields.next()?)?,
        parse_f32_field(fields.next()?)?,
        parse_f32_field(fields.next()?)?,
    ])
}

pub(crate) fn parse_args() -> Result<StartupAction, String> {
    parse_args_from(std::env::args().skip(1))
}

pub(crate) fn parse_args_from<I>(args: I) -> Result<StartupAction, String>
where
    I: IntoIterator<Item = String>,
{
    let mut preset = Preset::Fusion;
    let mut custom_command = Vec::new();
    let mut iter = args.into_iter();

    while let Some(arg) = iter.next() {
        match arg.as_str() {
            "--mode" => {
                let value = iter
                    .next()
                    .ok_or_else(|| "--mode requires `imu` or `fusion`".to_string())?;
                preset = Preset::from_cli(&value)?;
            }
            "--help" | "-h" => return Err(help_text()),
            "--" => {
                custom_command.extend(iter);
                break;
            }
            _ => {
                custom_command.push(arg);
                custom_command.extend(iter);
                break;
            }
        }
    }

    let (source, command) = if custom_command.is_empty() {
        (CommandSource::Preset(preset), preset.command())
    } else {
        (CommandSource::Custom, custom_command)
    };

    Ok(StartupAction::Run(LaunchConfig { source, command }))
}

fn help_text() -> String {
    [
        "imu-viz",
        "  --mode imu|fusion    Select a built-in firmware preset",
        "  --help               Show this help",
        "",
        "Examples:",
        "  cargo run -p imu-viz",
        "  cargo run -p imu-viz -- --mode imu",
        "  cargo run -p imu-viz -- cargo run -p imu --example rp235x_stemma_qt_9dof_lps25hb --target thumbv8m.main-none-eabihf",
    ]
    .join("\n")
}

pub(crate) fn format_command(command: &[String]) -> String {
    command
        .iter()
        .map(|arg| shell_escape(arg))
        .collect::<Vec<_>>()
        .join(" ")
}

fn shell_escape(arg: &str) -> String {
    if arg.is_empty() {
        return "''".to_string();
    }
    if arg
        .bytes()
        .all(|byte| byte.is_ascii_alphanumeric() || b"-_./:=+".contains(&byte))
    {
        return arg.to_string();
    }
    format!("'{}'", arg.replace('\'', "'\"'\"'"))
}

pub(crate) fn push_line_from_bytes(buffer: &mut Vec<u8>, tx: &Sender<AppEvent>) {
    if buffer.is_empty() {
        return;
    }

    let raw = String::from_utf8_lossy(buffer);
    let cleaned = strip_ansi_and_controls(&raw);
    buffer.clear();

    if cleaned.is_empty() {
        return;
    }
    if let Some(sample) = parse_sample(&cleaned) {
        let _ = tx.send(AppEvent::Sample(sample));
        let _ = tx.send(AppEvent::Log(sample.summary_line()));
        return;
    }
    let _ = tx.send(AppEvent::Log(cleaned));
}

pub(crate) fn strip_ansi_and_controls(input: &str) -> String {
    #[derive(Clone, Copy)]
    enum EscapeState {
        None,
        Esc,
        Csi,
        Osc,
    }

    let mut out = String::with_capacity(input.len());
    let mut state = EscapeState::None;
    for ch in input.chars() {
        state = match state {
            EscapeState::None => {
                if ch == '\u{1b}' {
                    EscapeState::Esc
                } else {
                    if !ch.is_control() || ch == '\t' {
                        out.push(ch);
                    }
                    EscapeState::None
                }
            }
            EscapeState::Esc => match ch {
                '[' => EscapeState::Csi,
                ']' => EscapeState::Osc,
                _ => EscapeState::None,
            },
            EscapeState::Csi => {
                if ('@'..='~').contains(&ch) {
                    EscapeState::None
                } else {
                    EscapeState::Csi
                }
            }
            EscapeState::Osc => {
                if ch == '\u{7}' {
                    EscapeState::None
                } else {
                    EscapeState::Osc
                }
            }
        };
    }
    out.trim().to_string()
}

#[cfg(test)]
mod tests {
    use std::sync::mpsc;

    use super::{
        CommandSource, Preset, StartupAction, parse_args_from, parse_sample, push_line_from_bytes,
        strip_ansi_and_controls,
    };
    use crate::host::{FeatureState, FeatureStatus, YawState};

    #[test]
    fn parse_sample_accepts_probe_rs_defmt_line() {
        let line = "7 [INFO ] state t_ms=100 quat=(0.9991181, -0.0028702, 0.0108203, 0.0401180) euler_deg=(0.1, 0.2, 2.3) velocity_m_s=(0.4, -0.2, 0.1) altitude_m=0.123 (rp235x_stemma_qt_9dof.rs:233)";
        let sample = parse_sample(line).expect("sample should parse");

        assert_eq!(sample.elapsed_ms, 100);
        assert!(sample.orientation.length_squared() > 0.99);
        assert!((sample.velocity_world_m_s.x - 0.4).abs() < 1e-6);
        assert!((sample.altitude_m - 0.123).abs() < 1e-6);
    }

    #[test]
    fn parse_sample_accepts_altitude() {
        let line = "7 [INFO ] state t_ms=100 quat=(0.9991181,-0.0028702,0.0108203,0.0401180) euler_deg=(0.1,0.2,2.3) velocity_m_s=(0.0,0.0,-0.2) altitude_m=0.33";
        let sample = parse_sample(line).expect("sample should parse");

        assert_eq!(sample.elapsed_ms, 100);
        assert!((sample.velocity_world_m_s.z + 0.2).abs() < 1e-6);
        assert!((sample.altitude_m - 0.33).abs() < 1e-6);
    }

    #[test]
    fn strip_ansi_preserves_probe_rs_log_text() {
        let line = "\u{1b}[1m[\u{1b}[32mINFO \u{1b}[0m\u{1b}[1m]\u{1b}[0m boot";
        assert_eq!(strip_ansi_and_controls(line), "[INFO ] boot");
    }

    #[test]
    fn push_line_parses_sample_after_ansi_cleanup() {
        let (tx, rx) = mpsc::channel();
        let mut line =
            b"\x1b[1m[\x1b[32mINFO \x1b[0m] state t_ms=100 quat=(0.9991181,-0.0028702,0.0108203,0.0401180) euler_deg=(0.1,0.2,2.3) velocity_m_s=(0.1,0.2,0.3) altitude_m=0.123"
                .to_vec();

        push_line_from_bytes(&mut line, &tx);

        let mut saw_sample = false;
        let mut saw_log = false;
        while let Ok(event) = rx.try_recv() {
            match event {
                super::AppEvent::Sample(sample) => {
                    saw_sample = true;
                    assert_eq!(sample.elapsed_ms, 100);
                }
                super::AppEvent::Log(line) => {
                    saw_log = true;
                    assert!(line.contains("t=0.10s"));
                    assert!(line.contains("attitude="));
                    assert!(line.contains("velocity="));
                }
            }
        }

        assert!(saw_sample);
        assert!(saw_log);
    }

    #[test]
    fn parse_args_defaults_to_fusion_preset() {
        let startup = parse_args_from(Vec::<String>::new()).expect("args should parse");

        match startup {
            StartupAction::Run(launch) => {
                assert_eq!(launch.source, CommandSource::Preset(Preset::Fusion));
                assert!(
                    launch
                        .command
                        .iter()
                        .any(|arg| arg == "rp235x_stemma_qt_9dof_lps25hb")
                );
            }
        }
    }

    #[test]
    fn parse_args_accepts_imu_preset() {
        let startup =
            parse_args_from(vec!["--mode".to_string(), "imu".to_string()]).expect("args parse");

        match startup {
            StartupAction::Run(launch) => {
                assert_eq!(launch.source, CommandSource::Preset(Preset::ImuOnly));
                assert!(
                    launch
                        .command
                        .iter()
                        .any(|arg| arg == "rp235x_stemma_qt_9dof")
                );
            }
        }
    }

    #[test]
    fn parse_args_help_returns_error_text() {
        let help = parse_args_from(vec!["--help".to_string()]).expect_err("help text");
        assert!(help.contains("imu-viz"));
        assert!(help.contains("--mode imu|fusion"));
    }

    #[test]
    fn parse_args_treats_remaining_values_as_custom_command() {
        let startup = parse_args_from(vec![
            "--mode".to_string(),
            "imu".to_string(),
            "echo".to_string(),
            "hello world".to_string(),
        ])
        .expect("args parse");

        match startup {
            StartupAction::Run(launch) => {
                assert_eq!(launch.source, CommandSource::Custom);
                assert_eq!(launch.command, vec!["echo", "hello world"]);
            }
        }
    }

    #[test]
    fn feature_status_tracks_magnetometer_and_yaw_mode() {
        let mut status = FeatureStatus::from_launch(&super::LaunchConfig {
            source: CommandSource::Preset(Preset::ImuOnly),
            command: Vec::new(),
        });

        status.update_from_log("LIS3MDL WHO_AM_I=61");
        assert_eq!(status.magnetometer, FeatureState::Enabled);
        assert_eq!(status.yaw, YawState::Relative);

        status.update_from_log("mag calibration ready");
        assert_eq!(status.yaw, YawState::Absolute);

        status.update_from_log("mag read failed");
        assert_eq!(status.magnetometer, FeatureState::Disabled);
        assert_eq!(status.yaw, YawState::Relative);
    }

    #[test]
    fn feature_status_tracks_barometer_presence() {
        let mut status = FeatureStatus::from_launch(&super::LaunchConfig {
            source: CommandSource::Preset(Preset::Fusion),
            command: Vec::new(),
        });

        status.update_from_log("LPS25HB WHO_AM_I=189");
        assert_eq!(status.barometer, FeatureState::Enabled);
    }
}

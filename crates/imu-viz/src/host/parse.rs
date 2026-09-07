use std::sync::mpsc::SyncSender;

use glam::{Quat, Vec3};

use super::{AppEvent, FeatureStatus, PortConfig, Sample};

pub(crate) fn parse_sample(line: &str) -> Option<Sample> {
    if !line.starts_with("state ") {
        return None;
    }
    let elapsed_ms = parse_u32_after(line, "t_ms=")?;
    let quat = parse_tuple4_after(line, "quat=(")?;
    let velocity = parse_tuple3_after(line, "velocity_m_s=(")?;
    let altitude_m = parse_f32_after(line, "altitude_m=")?;
    if quat.iter().any(|value| !value.is_finite())
        || velocity.iter().any(|value| !value.is_finite())
        || !altitude_m.is_finite()
    {
        return None;
    }
    let orientation = Quat::from_xyzw(quat[1], quat[2], quat[3], quat[0]);

    Sample::from_quaternion_log(
        elapsed_ms,
        orientation,
        Vec3::from_array(velocity),
        altitude_m,
    )
}

fn parse_u32_field(field: &str) -> Option<u32> {
    field.split_ascii_whitespace().next()?.parse().ok()
}

fn parse_f32_field(field: &str) -> Option<f32> {
    field.split_ascii_whitespace().next()?.parse().ok()
}

fn parse_tuple_value(field: &str) -> Option<f32> {
    let mut fields = field.split_ascii_whitespace();
    let value = fields.next()?.parse().ok()?;
    fields.next().is_none().then_some(value)
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
    let values = [
        parse_tuple_value(fields.next()?)?,
        parse_tuple_value(fields.next()?)?,
        parse_tuple_value(fields.next()?)?,
        parse_tuple_value(fields.next()?)?,
    ];
    fields.next().is_none().then_some(values)
}

fn parse_tuple3_after(line: &str, key: &str) -> Option<[f32; 3]> {
    let values = line.split_once(key)?.1;
    let end = values.find(')')?;
    let mut fields = values[..end].split(',');
    let values = [
        parse_tuple_value(fields.next()?)?,
        parse_tuple_value(fields.next()?)?,
        parse_tuple_value(fields.next()?)?,
    ];
    fields.next().is_none().then_some(values)
}

pub(crate) fn parse_args() -> Result<PortConfig, String> {
    parse_args_from(std::env::args().skip(1))
}

pub(crate) fn parse_args_from<I>(args: I) -> Result<PortConfig, String>
where
    I: IntoIterator<Item = String>,
{
    let mut port = None;
    let mut iter = args.into_iter();

    while let Some(arg) = iter.next() {
        match arg.as_str() {
            "--port" => {
                port = Some(
                    iter.next()
                        .ok_or_else(|| "--port requires a serial device path".to_string())?,
                );
            }
            "--help" | "-h" => return Err(help_text()),
            other => return Err(format!("unknown argument `{other}`\n\n{}", help_text())),
        }
    }

    let path = port.ok_or_else(help_text)?;
    Ok(PortConfig { path })
}

fn help_text() -> String {
    [
        "imu-viz",
        "  --port PATH          Read telemetry from a USB CDC serial device",
        "  --help               Show this help",
        "",
        "Example:",
        "  cargo run -p imu-viz -- --port /dev/ttyACM0",
    ]
    .join("\n")
}

pub(crate) fn push_line_from_bytes(buffer: &mut Vec<u8>, tx: &SyncSender<AppEvent>) {
    if buffer.is_empty() {
        return;
    }

    let cleaned = String::from_utf8_lossy(buffer).trim().to_string();
    buffer.clear();

    if cleaned.is_empty() {
        return;
    }
    if let Some(sample) = parse_sample(&cleaned) {
        let mut status = FeatureStatus::unknown();
        status.update_from_log(&cleaned);
        let _ = tx.try_send(AppEvent::Sample(sample, status));
        return;
    }
    let _ = tx.try_send(AppEvent::Log(cleaned));
}

#[cfg(test)]
mod tests {
    use std::sync::mpsc;

    use super::{parse_args_from, parse_sample, push_line_from_bytes};
    use crate::host::{FeatureState, FeatureStatus, YawState};

    #[test]
    fn parse_sample_accepts_usb_line() {
        let line = "state t_ms=100 quat=(0.9991181, -0.0028702, 0.0108203, 0.0401180) velocity_m_s=(0.4, -0.2, 0.1) altitude_m=0.123 mag_enabled=true mag_ready=false";
        let sample = parse_sample(line).expect("sample should parse");

        assert_eq!(sample.elapsed_ms, 100);
        assert!(sample.orientation.length_squared() > 0.99);
        assert!((sample.velocity_world_m_s.x - 0.4).abs() < 1e-6);
        assert!((sample.altitude_m - 0.123).abs() < 1e-6);
    }

    #[test]
    fn parse_sample_rejects_non_finite_values() {
        let line =
            "state t_ms=100 quat=(1.0,0.0,0.0,0.0) velocity_m_s=(0.0,0.0,0.0) altitude_m=NaN";
        assert!(parse_sample(line).is_none());
    }

    #[test]
    fn parse_sample_rejects_zero_quaternion() {
        let line =
            "state t_ms=100 quat=(0.0,0.0,0.0,0.0) velocity_m_s=(0.0,0.0,0.0) altitude_m=0.0";
        assert!(parse_sample(line).is_none());
    }

    #[test]
    fn push_line_parses_usb_sample() {
        let (tx, rx) = mpsc::sync_channel(4);
        let mut line = b"state t_ms=100 quat=(0.9991181,-0.0028702,0.0108203,0.0401180) velocity_m_s=(0.1,0.2,0.3) altitude_m=0.123 mag_enabled=true mag_ready=false".to_vec();

        push_line_from_bytes(&mut line, &tx);

        let mut saw_sample = false;
        let mut saw_status = false;
        while let Ok(event) = rx.try_recv() {
            match event {
                super::AppEvent::Sample(sample, status) => {
                    saw_sample = true;
                    saw_status = status.magnetometer == FeatureState::Enabled;
                    assert_eq!(sample.elapsed_ms, 100);
                }
                super::AppEvent::Log(_) => panic!("sample must not be copied to logs"),
                super::AppEvent::Disconnected(_) => panic!("unexpected disconnect"),
            }
        }

        assert!(saw_sample);
        assert!(saw_status);
    }

    #[test]
    fn parse_args_requires_port() {
        let error = parse_args_from(Vec::<String>::new()).expect_err("port should be required");
        assert!(error.contains("--port PATH"));
    }

    #[test]
    fn parse_args_accepts_port() {
        let startup = parse_args_from(vec!["--port".to_string(), "/dev/ttyACM0".to_string()])
            .expect("args parse");

        assert_eq!(startup.path, "/dev/ttyACM0");
    }

    #[test]
    fn parse_args_help_returns_error_text() {
        let help = parse_args_from(vec!["--help".to_string()]).expect_err("help text");
        assert!(help.contains("imu-viz"));
        assert!(help.contains("--port PATH"));
    }

    #[test]
    fn parse_args_rejects_unknown_arguments() {
        let error = parse_args_from(vec!["--mode".to_string()]).expect_err("argument rejected");
        assert!(error.contains("unknown argument"));
    }

    #[test]
    fn feature_status_tracks_usb_metadata() {
        let mut status = FeatureStatus::unknown();

        status.update_from_log("state mag_enabled=true mag_ready=false");
        assert_eq!(status.magnetometer, FeatureState::Enabled);
        assert_eq!(status.yaw, YawState::Relative);

        status.update_from_log("state mag_enabled=true mag_ready=true");
        assert_eq!(status.yaw, YawState::Absolute);

        status.update_from_log("state mag_enabled=false mag_ready=false");
        assert_eq!(status.magnetometer, FeatureState::Disabled);
        assert_eq!(status.yaw, YawState::Relative);
    }
}

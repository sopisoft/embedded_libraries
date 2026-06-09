//! Pilot input shaping helpers.

/// Applies a symmetric deadband around zero.
pub fn apply_deadband(value: f32, deadband: f32) -> f32 {
    let deadband = deadband.clamp(0.0, 0.9999);
    let magnitude = value.abs();
    if magnitude <= deadband {
        0.0
    } else {
        let scaled = (magnitude - deadband) / (1.0 - deadband);
        scaled.copysign(value)
    }
}

/// Applies a simple exponential feel curve.
///
/// `expo = 0` leaves the signal unchanged.
/// `expo = 1` yields a fully cubic response.
pub fn apply_expo(value: f32, expo: f32) -> f32 {
    let expo = expo.clamp(0.0, 1.0);
    value * (1.0 - expo) + value * value * value * expo
}

/// Applies a dual-rate scale factor.
pub fn apply_dual_rate(value: f32, rate: f32) -> f32 {
    value * rate.clamp(0.0, 1.5)
}

/// Applies deadband, expo, and dual-rate in one step.
pub fn shape_rc_command(value: f32, deadband: f32, expo: f32, rate: f32) -> f32 {
    let value = value.clamp(-1.0, 1.0);
    let value = apply_deadband(value, deadband);
    let value = apply_expo(value, expo);
    apply_dual_rate(value, rate).clamp(-1.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deadband_zeroes_small_values() {
        assert_eq!(apply_deadband(0.05, 0.1), 0.0);
        assert!(apply_deadband(0.5, 0.1) > 0.0);
    }

    #[test]
    fn expo_keeps_center_soft() {
        assert!(apply_expo(0.2, 0.7).abs() < 0.2);
        assert_eq!(apply_expo(1.0, 0.7), 1.0);
    }

    #[test]
    fn combined_shape_stays_bounded() {
        let shaped = shape_rc_command(0.8, 0.05, 0.4, 1.2);
        assert!((-1.0..=1.0).contains(&shaped));
    }
}

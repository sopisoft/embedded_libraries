//! Pilot input shaping helpers.

use crate::SignedNormalized;

/// Applies a symmetric deadband around zero.
pub fn apply_deadband(value: SignedNormalized, deadband: f32) -> SignedNormalized {
    let value = value.get();
    let deadband = deadband.clamp(0.0, 0.9999);
    let magnitude = value.abs();
    let output = if magnitude <= deadband {
        0.0
    } else {
        let scaled = (magnitude - deadband) / (1.0 - deadband);
        scaled.copysign(value)
    };
    SignedNormalized::saturated(output)
}

/// Applies a simple exponential feel curve.
/// `expo = 0` leaves the signal unchanged.
/// `expo = 1` yields a fully cubic response.
pub fn apply_expo(value: SignedNormalized, expo: f32) -> SignedNormalized {
    let value = value.get();
    let expo = expo.clamp(0.0, 1.0);
    SignedNormalized::saturated(value * (1.0 - expo) + value * value * value * expo)
}

/// Applies a dual-rate scale factor.
pub fn apply_dual_rate(value: SignedNormalized, rate: f32) -> SignedNormalized {
    SignedNormalized::saturated(value.get() * rate.clamp(0.0, 1.5))
}

/// Applies deadband, expo, and dual-rate in one step.
pub fn shape_rc_command(value: f32, deadband: f32, expo: f32, rate: f32) -> SignedNormalized {
    let value = SignedNormalized::saturated(value);
    let value = apply_deadband(value, deadband);
    let value = apply_expo(value, expo);
    apply_dual_rate(value, rate)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deadband_zeroes_small_values() {
        assert_eq!(
            apply_deadband(SignedNormalized::saturated(0.05), 0.1).get(),
            0.0
        );
        assert!(apply_deadband(SignedNormalized::saturated(0.5), 0.1).get() > 0.0);
    }

    #[test]
    fn expo_keeps_center_soft() {
        assert!(apply_expo(SignedNormalized::saturated(0.2), 0.7).abs() < 0.2);
        assert_eq!(apply_expo(SignedNormalized::saturated(1.0), 0.7).get(), 1.0);
    }

    #[test]
    fn combined_shape_stays_bounded() {
        let shaped = shape_rc_command(0.8, 0.05, 0.4, 1.2);
        assert!((-1.0..=1.0).contains(&shaped.get()));
    }
}

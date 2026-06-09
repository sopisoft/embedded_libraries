//! Angle helpers and a small `Angle` newtype.

use libm::{fabsf, fmodf, sincosf};

/// Two times pi.
pub const TAU: f32 = core::f32::consts::PI * 2.0;

/// Angle stored in radians.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Angle {
    radians: f32,
}

impl Angle {
    /// Creates an angle from radians.
    pub const fn from_radians(radians: f32) -> Self {
        Self { radians }
    }

    /// Creates an angle from degrees.
    pub fn from_degrees(degrees: f32) -> Self {
        Self::from_radians(deg_to_rad(degrees))
    }

    /// Returns the underlying radians value.
    pub const fn radians(self) -> f32 {
        self.radians
    }

    /// Returns the value in degrees.
    pub fn degrees(self) -> f32 {
        rad_to_deg(self.radians)
    }

    /// Normalizes the angle into `[-pi, pi)`.
    pub fn normalized_signed(self) -> Self {
        Self::from_radians(wrap_pi(self.radians))
    }

    /// Normalizes the angle into `[0, 2pi)`.
    pub fn normalized_unsigned(self) -> Self {
        Self::from_radians(wrap_tau(self.radians))
    }

    /// Computes sine and cosine together.
    pub fn sin_cos(self) -> (f32, f32) {
        sincosf(self.radians)
    }
}

/// Converts degrees to radians.
pub const fn deg_to_rad(deg: f32) -> f32 {
    deg * core::f32::consts::PI / 180.0
}

/// Converts radians to degrees.
pub const fn rad_to_deg(rad: f32) -> f32 {
    rad * 180.0 / core::f32::consts::PI
}

/// Wraps an angle into `[-pi, pi)`.
pub fn wrap_pi(angle: f32) -> f32 {
    let mut wrapped = fmodf(angle + core::f32::consts::PI, TAU);
    if wrapped < 0.0 {
        wrapped += TAU;
    }
    wrapped - core::f32::consts::PI
}

/// Wraps an angle into `[0, 2pi)`.
pub fn wrap_tau(angle: f32) -> f32 {
    let mut wrapped = fmodf(angle, TAU);
    if wrapped < 0.0 {
        wrapped += TAU;
    }
    if fabsf(wrapped - TAU) < f32::EPSILON {
        0.0
    } else {
        wrapped
    }
}

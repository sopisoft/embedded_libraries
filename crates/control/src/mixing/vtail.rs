use super::{ControlAxes, Normalized, SignedNormalized, SurfaceChannel, ThrottleChannel};

/// Output bundle for a V-tail aircraft.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VTailOutputs {
    pub left_tail: SignedNormalized,
    pub right_tail: SignedNormalized,
    pub aileron: SignedNormalized,
    pub throttle: Normalized,
}

/// Mixer for V-tail aircraft.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VTailMixer {
    pub left_tail: SurfaceChannel,
    pub right_tail: SurfaceChannel,
    pub aileron: SurfaceChannel,
    pub throttle: ThrottleChannel,
}

impl VTailMixer {
    /// Creates a default V-tail mixer.
    pub const fn new() -> Self {
        Self {
            left_tail: SurfaceChannel::new(1.0),
            right_tail: SurfaceChannel::new(1.0),
            aileron: SurfaceChannel::new(1.0),
            throttle: ThrottleChannel::new(),
        }
    }

    /// Mixes pitch and yaw into V-tail surfaces.
    pub fn mix(&self, axes: ControlAxes) -> VTailOutputs {
        let pitch = axes.pitch.get();
        let yaw = axes.yaw.get();
        VTailOutputs {
            left_tail: self
                .left_tail
                .apply(SignedNormalized::saturated(pitch + yaw)),
            right_tail: self
                .right_tail
                .apply(SignedNormalized::saturated(pitch - yaw)),
            aileron: self.aileron.apply(axes.roll),
            throttle: self.throttle.apply(axes.throttle),
        }
    }
}

impl Default for VTailMixer {
    fn default() -> Self {
        Self::new()
    }
}

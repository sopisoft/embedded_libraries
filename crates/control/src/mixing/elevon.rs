use super::{ControlAxes, Normalized, SignedNormalized, SurfaceChannel, ThrottleChannel};

/// Output bundle for an elevon delta wing.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ElevonOutputs {
    pub left_elevon: SignedNormalized,
    pub right_elevon: SignedNormalized,
    pub throttle: Normalized,
}

/// Mixer for elevon-equipped aircraft.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ElevonMixer {
    pub left_elevon: SurfaceChannel,
    pub right_elevon: SurfaceChannel,
    pub throttle: ThrottleChannel,
}

impl ElevonMixer {
    /// Creates a default elevon mixer.
    pub const fn new() -> Self {
        Self {
            left_elevon: SurfaceChannel::new(1.0),
            right_elevon: SurfaceChannel::new(1.0),
            throttle: ThrottleChannel::new(),
        }
    }

    /// Mixes roll and pitch into left/right elevons.
    pub fn mix(&self, axes: ControlAxes) -> ElevonOutputs {
        let roll = axes.roll.get();
        let pitch = axes.pitch.get();
        ElevonOutputs {
            left_elevon: self
                .left_elevon
                .apply(SignedNormalized::saturated(pitch + roll)),
            right_elevon: self
                .right_elevon
                .apply(SignedNormalized::saturated(pitch - roll)),
            throttle: self.throttle.apply(axes.throttle),
        }
    }
}

impl Default for ElevonMixer {
    fn default() -> Self {
        Self::new()
    }
}

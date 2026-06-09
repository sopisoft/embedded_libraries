use super::{ControlAxes, SurfaceChannel, ThrottleChannel};

/// Output bundle for an elevon delta wing.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ElevonOutputs {
    pub left_elevon: f32,
    pub right_elevon: f32,
    pub throttle: f32,
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
        let roll = axes.roll.clamp(-1.0, 1.0);
        let pitch = axes.pitch.clamp(-1.0, 1.0);
        ElevonOutputs {
            left_elevon: self.left_elevon.apply(pitch + roll),
            right_elevon: self.right_elevon.apply(pitch - roll),
            throttle: self.throttle.apply(axes.throttle),
        }
    }
}

impl Default for ElevonMixer {
    fn default() -> Self {
        Self::new()
    }
}

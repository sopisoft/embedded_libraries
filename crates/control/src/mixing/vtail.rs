use super::{ControlAxes, SurfaceChannel, ThrottleChannel};

/// Output bundle for a V-tail aircraft.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VTailOutputs {
    pub left_tail: f32,
    pub right_tail: f32,
    pub aileron: f32,
    pub throttle: f32,
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
        let pitch = axes.pitch.clamp(-1.0, 1.0);
        let yaw = axes.yaw.clamp(-1.0, 1.0);
        VTailOutputs {
            left_tail: self.left_tail.apply(pitch + yaw),
            right_tail: self.right_tail.apply(pitch - yaw),
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

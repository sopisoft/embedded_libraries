use super::{ControlAxes, Normalized, SignedNormalized, SurfaceChannel, ThrottleChannel};

/// Output bundle for a conventional fixed-wing tail.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ConventionalTailOutputs {
    pub left_aileron: SignedNormalized,
    pub right_aileron: SignedNormalized,
    pub elevator: SignedNormalized,
    pub rudder: SignedNormalized,
    pub throttle: Normalized,
    pub left_flap: SignedNormalized,
    pub right_flap: SignedNormalized,
}

/// Mixer for a conventional fixed-wing tail with optional flaperons.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ConventionalTailMixer {
    pub left_aileron: SurfaceChannel,
    pub right_aileron: SurfaceChannel,
    pub elevator: SurfaceChannel,
    pub rudder: SurfaceChannel,
    pub throttle: ThrottleChannel,
    pub left_flap: SurfaceChannel,
    pub right_flap: SurfaceChannel,
    /// `0` means symmetric ailerons, `1` means full down-going aileron suppression.
    differential: f32,
    /// Flap contribution mixed into the ailerons.
    flaperon_mix: f32,
}

impl ConventionalTailMixer {
    /// Creates a conventional mixer with default symmetric channels.
    pub const fn new() -> Self {
        Self {
            left_aileron: SurfaceChannel::new(1.0),
            right_aileron: SurfaceChannel::new(1.0),
            elevator: SurfaceChannel::new(1.0),
            rudder: SurfaceChannel::new(1.0),
            throttle: ThrottleChannel::new(),
            left_flap: SurfaceChannel::new(1.0),
            right_flap: SurfaceChannel::new(1.0),
            differential: 0.0,
            flaperon_mix: 0.0,
        }
    }

    pub const fn with_differential(mut self, differential: f32) -> Self {
        self.differential = Normalized::saturated(differential).get();
        self
    }

    pub const fn with_flaperon_mix(mut self, flaperon_mix: f32) -> Self {
        self.flaperon_mix = SignedNormalized::saturated(flaperon_mix).get();
        self
    }

    pub const fn with_right_aileron(mut self, channel: SurfaceChannel) -> Self {
        self.right_aileron = channel;
        self
    }

    pub const fn with_elevator(mut self, channel: SurfaceChannel) -> Self {
        self.elevator = channel;
        self
    }

    /// Mixes pilot/autopilot axes into actuator outputs.
    pub fn mix(&self, axes: ControlAxes) -> ConventionalTailOutputs {
        let roll = axes.roll.get();
        let pitch = axes.pitch.get();
        let yaw = axes.yaw.get();
        let flaps = axes.flaps.get();

        let left_roll = apply_aileron_differential(roll, self.differential, true);
        let right_roll = apply_aileron_differential(-roll, self.differential, false);
        let flaperon = flaps * self.flaperon_mix;

        ConventionalTailOutputs {
            left_aileron: self
                .left_aileron
                .apply(SignedNormalized::saturated(left_roll + flaperon)),
            right_aileron: self
                .right_aileron
                .apply(SignedNormalized::saturated(right_roll + flaperon)),
            elevator: self.elevator.apply(SignedNormalized::saturated(pitch)),
            rudder: self.rudder.apply(SignedNormalized::saturated(yaw)),
            throttle: self.throttle.apply(axes.throttle),
            left_flap: self.left_flap.apply(SignedNormalized::saturated(flaps)),
            right_flap: self.right_flap.apply(SignedNormalized::saturated(flaps)),
        }
    }
}

impl Default for ConventionalTailMixer {
    fn default() -> Self {
        Self::new()
    }
}

fn apply_aileron_differential(command: f32, differential: f32, is_left: bool) -> f32 {
    let command = command.clamp(-1.0, 1.0);
    let differential = differential.clamp(0.0, 1.0);
    let down_scale = 1.0 - differential;

    if is_left {
        if command < 0.0 {
            command * down_scale
        } else {
            command
        }
    } else if command > 0.0 {
        command * down_scale
    } else {
        command
    }
}

use super::{ControlAxes, SurfaceChannel, ThrottleChannel};

/// Output bundle for a conventional fixed-wing tail.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ConventionalTailOutputs {
    pub left_aileron: f32,
    pub right_aileron: f32,
    pub elevator: f32,
    pub rudder: f32,
    pub throttle: f32,
    pub left_flap: f32,
    pub right_flap: f32,
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
    pub differential: f32,
    /// Flap contribution mixed into the ailerons.
    pub flaperon_mix: f32,
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

    /// Mixes pilot/autopilot axes into actuator outputs.
    pub fn mix(&self, axes: ControlAxes) -> ConventionalTailOutputs {
        let roll = axes.roll.clamp(-1.0, 1.0);
        let pitch = axes.pitch.clamp(-1.0, 1.0);
        let yaw = axes.yaw.clamp(-1.0, 1.0);
        let flaps = axes.flaps.clamp(0.0, 1.0);

        let left_roll = apply_aileron_differential(roll, self.differential, true);
        let right_roll = apply_aileron_differential(-roll, self.differential, false);
        let flaperon = flaps * self.flaperon_mix;

        ConventionalTailOutputs {
            left_aileron: self.left_aileron.apply(left_roll + flaperon),
            right_aileron: self.right_aileron.apply(right_roll + flaperon),
            elevator: self.elevator.apply(pitch),
            rudder: self.rudder.apply(yaw),
            throttle: self.throttle.apply(axes.throttle),
            left_flap: self.left_flap.apply(flaps),
            right_flap: self.right_flap.apply(flaps),
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

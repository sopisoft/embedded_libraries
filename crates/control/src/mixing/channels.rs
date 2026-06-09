/// Pilot or autopilot axis commands.
///
/// Roll, pitch, and yaw are expected in `[-1, 1]`.
/// Throttle and flaps are expected in `[0, 1]`.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct ControlAxes {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
    pub flaps: f32,
}

impl ControlAxes {
    /// Creates a new command block.
    pub const fn new(roll: f32, pitch: f32, yaw: f32, throttle: f32, flaps: f32) -> Self {
        Self {
            roll,
            pitch,
            yaw,
            throttle,
            flaps,
        }
    }
}

/// Output shaping for a symmetric servo channel.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SurfaceChannel {
    pub scale: f32,
    pub trim: f32,
    pub reversed: bool,
    pub min: f32,
    pub max: f32,
}

impl SurfaceChannel {
    /// Creates a symmetric surface channel.
    pub const fn new(scale: f32) -> Self {
        Self {
            scale,
            trim: 0.0,
            reversed: false,
            min: -1.0,
            max: 1.0,
        }
    }

    /// Shapes a symmetric command into a bounded output.
    pub fn apply(&self, command: f32) -> f32 {
        let mut output = command * self.scale + self.trim;
        if self.reversed {
            output = -output;
        }
        output.clamp(self.min, self.max)
    }
}

/// Output shaping for throttle-like channels.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ThrottleChannel {
    pub min: f32,
    pub max: f32,
    pub reversed: bool,
}

impl ThrottleChannel {
    /// Creates a unit-range throttle channel.
    pub const fn new() -> Self {
        Self {
            min: 0.0,
            max: 1.0,
            reversed: false,
        }
    }

    /// Shapes a unipolar command into a bounded output.
    pub fn apply(&self, command: f32) -> f32 {
        let command = command.clamp(0.0, 1.0);
        let command = if self.reversed {
            1.0 - command
        } else {
            command
        };
        (self.min + (self.max - self.min) * command).clamp(self.min, self.max)
    }
}

impl Default for ThrottleChannel {
    fn default() -> Self {
        Self::new()
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct SignedNormalized(f32);

impl SignedNormalized {
    pub const ZERO: Self = Self(0.0);

    pub const fn new(value: f32) -> Option<Self> {
        if value >= -1.0 && value <= 1.0 {
            Some(Self(value))
        } else {
            None
        }
    }

    pub const fn saturated(value: f32) -> Self {
        if value.is_nan() {
            Self::ZERO
        } else if value < -1.0 {
            Self(-1.0)
        } else if value > 1.0 {
            Self(1.0)
        } else {
            Self(value)
        }
    }

    pub const fn get(self) -> f32 {
        self.0
    }

    pub fn abs(self) -> f32 {
        self.0.abs()
    }
}

impl From<SignedNormalized> for f32 {
    fn from(value: SignedNormalized) -> Self {
        value.get()
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct Normalized(f32);

impl Normalized {
    pub const ZERO: Self = Self(0.0);
    pub const ONE: Self = Self(1.0);

    pub const fn new(value: f32) -> Option<Self> {
        if value >= 0.0 && value <= 1.0 {
            Some(Self(value))
        } else {
            None
        }
    }

    pub const fn saturated(value: f32) -> Self {
        if value.is_nan() || value < 0.0 {
            Self::ZERO
        } else if value > 1.0 {
            Self(1.0)
        } else {
            Self(value)
        }
    }

    pub const fn get(self) -> f32 {
        self.0
    }
}

impl From<Normalized> for f32 {
    fn from(value: Normalized) -> Self {
        value.get()
    }
}

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct ControlAxes {
    pub roll: SignedNormalized,
    pub pitch: SignedNormalized,
    pub yaw: SignedNormalized,
    pub throttle: Normalized,
    pub flaps: Normalized,
}

impl ControlAxes {
    /// Creates a new command block.
    pub const fn new(
        roll: SignedNormalized,
        pitch: SignedNormalized,
        yaw: SignedNormalized,
        throttle: Normalized,
        flaps: Normalized,
    ) -> Self {
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
    scale: f32,
    trim: f32,
    reversed: bool,
    min: f32,
    max: f32,
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

    pub const fn with_trim(mut self, trim: f32) -> Self {
        self.trim = trim;
        self
    }

    pub const fn with_reverse(mut self, reversed: bool) -> Self {
        self.reversed = reversed;
        self
    }

    pub fn with_limits(mut self, min: f32, max: f32) -> Self {
        let min = SignedNormalized::saturated(min).get();
        let max = SignedNormalized::saturated(max).get();
        self.min = min.min(max);
        self.max = min.max(max);
        self
    }

    /// Shapes a symmetric command into a bounded output.
    pub fn apply(&self, command: SignedNormalized) -> SignedNormalized {
        let command = command.get();
        let mut output = command * self.scale + self.trim;
        if self.reversed {
            output = -output;
        }
        SignedNormalized::saturated(output.clamp(self.min, self.max))
    }
}

/// Output shaping for throttle-like channels.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ThrottleChannel {
    min: f32,
    max: f32,
    reversed: bool,
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

    pub const fn with_reverse(mut self, reversed: bool) -> Self {
        self.reversed = reversed;
        self
    }

    pub fn with_limits(mut self, min: f32, max: f32) -> Self {
        let min = Normalized::saturated(min).get();
        let max = Normalized::saturated(max).get();
        self.min = min.min(max);
        self.max = min.max(max);
        self
    }

    /// Shapes a unipolar command into a bounded output.
    pub fn apply(&self, command: Normalized) -> Normalized {
        let command = command.get();
        let command = if self.reversed {
            1.0 - command
        } else {
            command
        };
        Normalized::saturated(
            (self.min + (self.max - self.min) * command).clamp(self.min, self.max),
        )
    }
}

impl Default for ThrottleChannel {
    fn default() -> Self {
        Self::new()
    }
}

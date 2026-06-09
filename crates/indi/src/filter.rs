use fugit::MicrosDurationU32;

/// First-order low-pass filter for noisy differentiated gyro signals.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LowPassFilter {
    tau_s: f32,
    state: f32,
    initialized: bool,
}

impl LowPassFilter {
    /// Creates a filter with the given time constant in seconds.
    pub const fn new(tau_s: f32) -> Self {
        Self {
            tau_s,
            state: 0.0,
            initialized: false,
        }
    }

    /// Creates a pass-through filter.
    pub const fn disabled() -> Self {
        Self::new(0.0)
    }

    /// Clears the internal state.
    pub fn reset(&mut self) {
        self.state = 0.0;
        self.initialized = false;
    }

    /// Changes the filter time constant.
    pub fn set_tau_s(&mut self, tau_s: f32) {
        self.tau_s = tau_s.max(0.0);
    }

    /// Updates the filter.
    pub fn update(&mut self, input: f32, dt: MicrosDurationU32) -> f32 {
        let dt_s = dt.as_secs_f32();
        if !self.initialized || self.tau_s <= 0.0 || dt_s <= 0.0 {
            self.state = input;
            self.initialized = true;
            return self.state;
        }

        let alpha = dt_s / (self.tau_s + dt_s);
        self.state += alpha * (input - self.state);
        self.state
    }
}

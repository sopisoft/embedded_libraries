use fugit::MicrosDurationU32;

use super::ServoRange;

/// Shared limits for multiple servos.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ServoSet<const N: usize> {
    ranges: [ServoRange; N],
}

impl<const N: usize> ServoSet<N> {
    /// Creates a set from per-servo ranges.
    pub const fn new(ranges: [ServoRange; N]) -> Self {
        Self { ranges }
    }

    /// Returns one per-channel range.
    pub const fn range(&self, index: usize) -> ServoRange {
        self.ranges[index]
    }

    /// Converts degree commands into per-servo pulse widths.
    pub fn pulse_widths_from_angles_degrees(&self, angles_deg: [f32; N]) -> [MicrosDurationU32; N] {
        let mut pulses = [MicrosDurationU32::from_micros(0); N];
        let mut i = 0;
        while i < N {
            pulses[i] = self.ranges[i].pulse_for_angle_degrees(angles_deg[i]);
            i += 1;
        }
        pulses
    }

    /// Converts symmetric commands in `[-1, 1]` into per-servo pulse widths.
    pub fn pulse_widths_from_symmetric(&self, commands: [f32; N]) -> [MicrosDurationU32; N] {
        let mut pulses = [MicrosDurationU32::from_micros(0); N];
        let mut i = 0;
        while i < N {
            pulses[i] = self.ranges[i].pulse_for_symmetric(commands[i]);
            i += 1;
        }
        pulses
    }
}

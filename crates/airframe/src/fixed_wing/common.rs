use control::{Normalized, SignedNormalized};
use fugit::MicrosDurationU32;
use pwm::ServoSet;

/// How one actuator value should be converted into a servo pulse.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ServoCommandMode {
    Symmetric,
    Normalized,
}

/// One servo assignment.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ServoAssignment {
    pub index: usize,
    pub mode: ServoCommandMode,
}

impl ServoAssignment {
    pub const fn symmetric(index: usize) -> Self {
        Self {
            index,
            mode: ServoCommandMode::Symmetric,
        }
    }

    pub const fn normalized(index: usize) -> Self {
        Self {
            index,
            mode: ServoCommandMode::Normalized,
        }
    }
}

pub(crate) fn apply_assignment<const N: usize>(
    pulses: &mut [MicrosDurationU32; N],
    servos: &ServoSet<N>,
    assignment: ServoAssignment,
    value: impl Copy + Into<f32>,
) {
    if assignment.index >= N {
        return;
    }
    let Some(range) = servos.get(assignment.index) else {
        return;
    };
    let value: f32 = value.into();
    pulses[assignment.index] = match assignment.mode {
        ServoCommandMode::Symmetric => {
            range.pulse_for_symmetric(SignedNormalized::saturated(value))
        }
        ServoCommandMode::Normalized => range.pulse_for_normalized(Normalized::saturated(value)),
    };
}

pub(crate) fn neutral_pulses<const N: usize>(servos: &ServoSet<N>) -> [MicrosDurationU32; N] {
    let mut pulses = [MicrosDurationU32::from_micros(1_500); N];
    let mut i = 0;
    while i < N {
        if let Some(range) = servos.get(i) {
            pulses[i] = range.pulse_for_symmetric(SignedNormalized::ZERO);
        }
        i += 1;
    }
    pulses
}

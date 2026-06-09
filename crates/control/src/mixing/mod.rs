//! Fixed-wing output mixing helpers.

mod channels;
mod conventional;
mod elevon;
#[cfg(test)]
mod tests;
mod vtail;

pub use channels::{ControlAxes, SurfaceChannel, ThrottleChannel};
pub use conventional::{ConventionalTailMixer, ConventionalTailOutputs};
pub use elevon::{ElevonMixer, ElevonOutputs};
pub use vtail::{VTailMixer, VTailOutputs};

//! Servo wrapper that maps angles or normalized positions into pulse widths.

mod device;
mod output;
mod range;
mod set;
#[cfg(test)]
mod tests;

pub use device::Servo;
pub use output::{ServoBank, ServoOutput};
pub use range::ServoRange;
pub use set::ServoSet;

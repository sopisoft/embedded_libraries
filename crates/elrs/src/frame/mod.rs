//! CRSF frame encoding and incremental parsing.

mod constants;
mod parser;
#[cfg(test)]
mod tests;
mod types;

pub use constants::*;
pub use parser::{FrameParser, ParseError};
pub use types::{
    Frame, FrameError, MAX_BODY_LEN, MAX_EXTENDED_PAYLOAD_LEN, MAX_FRAME_SIZE, MAX_LENGTH_FIELD,
};

#![no_std]

//! `embedded-io` driver for the TSD10 single-point LiDAR UART module.

#[cfg(test)]
extern crate std;

mod driver;
mod parser;
mod types;

pub use driver::Tsd10;
pub use parser::FrameParser;
pub use types::{
    BaudRate, COMMAND_HEADER, DEFAULT_BAUD_RATE, Error, FRAME_HEADER, Measurement, OUT_OF_RANGE_MM,
    ParseError,
};

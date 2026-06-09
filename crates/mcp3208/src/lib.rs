#![no_std]

//! `no_std` MCP3208 ADC driver.

#[cfg(test)]
extern crate std;

pub mod driver;

pub use driver::{Channel, Error, Mcp3208};

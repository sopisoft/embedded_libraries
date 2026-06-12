// This example is intentionally host-runnable so you can understand the TSD10
// API without hardware first.
//
// On a real board, replace `FakeUart` with your UART peripheral or USB-serial
// adapter type that implements `embedded_io::Read + embedded_io::Write`.

use core::convert::Infallible;
use embedded_io::{ErrorType, Read, Write};
use std::{collections::VecDeque, vec::Vec};
use tsd10::Tsd10;

#[derive(Debug)]
struct FakeUart {
    rx: VecDeque<u8>,
    tx: Vec<u8>,
}

impl FakeUart {
    fn from_rx(rx: &[u8]) -> Self {
        Self {
            rx: rx.iter().copied().collect(),
            tx: Vec::new(),
        }
    }
}

impl ErrorType for FakeUart {
    type Error = Infallible;
}

impl Read for FakeUart {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        match self.rx.pop_front() {
            Some(byte) => {
                buf[0] = byte;
                Ok(1)
            }
            None => Ok(0),
        }
    }
}

impl Write for FakeUart {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.extend_from_slice(buf);
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

fn main() {
    // Pretend the UART stream starts mid-frame, then eventually delivers one
    // valid distance sample of 1234 mm.
    let uart = FakeUart::from_rx(&[0x99, 0x00, 0x5C, 0xD2, 0x04, 0x29]);
    let mut lidar = Tsd10::new(uart);

    let measurement = lidar.read_measurement().unwrap();

    println!("TSD10 basic reading");
    println!("  distance_mm : {}", measurement.distance_mm);
    println!("  distance_m  : {:.3}", measurement.distance_m().unwrap());
}

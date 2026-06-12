use embedded_io::{Read, Write};

use crate::{BaudRate, COMMAND_HEADER, Error, FrameParser, Measurement, types::checksum};

const COMMAND_ID_BAUD: u8 = 0x06;
const COMMAND_ID_MEASURE: u8 = 0x0A;
const COMMAND_REPLY_MASK: u8 = 0x80;
const COMMAND_PAYLOAD_LEN: u8 = 0x02;

/// Blocking TSD10 UART driver.
pub struct Tsd10<SERIAL> {
    serial: SERIAL,
    parser: FrameParser,
}

impl<SERIAL> Tsd10<SERIAL> {
    /// Creates a driver instance without touching the UART yet.
    pub const fn new(serial: SERIAL) -> Self {
        Self {
            serial,
            parser: FrameParser::new(),
        }
    }

    /// Releases the inner UART transport.
    pub fn release(self) -> SERIAL {
        self.serial
    }

    /// Resets the internal stream parser.
    pub fn reset_parser(&mut self) {
        self.parser.reset();
    }
}

impl<SERIAL> Tsd10<SERIAL>
where
    SERIAL: Read,
{
    /// Reads the next valid measurement frame from the active output stream.
    pub fn read_measurement(&mut self) -> Result<Measurement, Error<SERIAL::Error>> {
        loop {
            let byte = self.read_byte()?;
            if let Some(measurement) = self.parser.push(byte) {
                return Ok(measurement);
            }
        }
    }

    /// Reads only the distance field in millimeters.
    pub fn read_distance_mm(&mut self) -> Result<u16, Error<SERIAL::Error>> {
        Ok(self.read_measurement()?.distance_mm)
    }

    fn read_byte(&mut self) -> Result<u8, Error<SERIAL::Error>> {
        let mut byte = [0u8; 1];
        match self.serial.read(&mut byte).map_err(Error::Io)? {
            0 => Err(Error::UnexpectedEof),
            _ => Ok(byte[0]),
        }
    }
}

impl<SERIAL> Tsd10<SERIAL>
where
    SERIAL: Read + Write,
{
    /// Sends the `start measure` command and waits for its reply.
    pub fn start_measurement(&mut self) -> Result<(), Error<SERIAL::Error>> {
        let payload = [0x02, 0x00];
        self.write_command(COMMAND_ID_MEASURE, payload)?;
        self.wait_for_reply(COMMAND_ID_MEASURE | COMMAND_REPLY_MASK, payload)?;
        self.parser.reset();
        Ok(())
    }

    /// Sends the `stop measure` command and waits for its reply.
    pub fn stop_measurement(&mut self) -> Result<(), Error<SERIAL::Error>> {
        let payload = [0x00, 0x00];
        self.write_command(COMMAND_ID_MEASURE, payload)?;
        self.wait_for_reply(COMMAND_ID_MEASURE | COMMAND_REPLY_MASK, payload)?;
        self.parser.reset();
        Ok(())
    }

    /// Sends the baud-rate command described in the TSD10 manual.
    ///
    /// The sensor may switch UART speed immediately after this write, so this
    /// method does not wait for the reply frame. Reconfigure the host UART
    /// before reading again.
    pub fn set_baud_rate(&mut self, baud_rate: BaudRate) -> Result<(), Error<SERIAL::Error>> {
        self.write_command(COMMAND_ID_BAUD, baud_rate.payload_bytes())?;
        self.parser.reset();
        Ok(())
    }

    /// Sends the baud-rate command and waits for the matching reply.
    ///
    /// Use this only if your transport can still receive the confirmation at
    /// the old speed, or if you intentionally switch the host side in time.
    pub fn set_baud_rate_with_ack(
        &mut self,
        baud_rate: BaudRate,
    ) -> Result<(), Error<SERIAL::Error>> {
        let payload = baud_rate.payload_bytes();
        self.write_command(COMMAND_ID_BAUD, payload)?;
        self.wait_for_reply(COMMAND_ID_BAUD | COMMAND_REPLY_MASK, payload)?;
        self.parser.reset();
        Ok(())
    }

    fn write_command(&mut self, command: u8, payload: [u8; 2]) -> Result<(), Error<SERIAL::Error>> {
        let frame = build_command(command, payload);
        self.serial.write_all(&frame).map_err(Error::Io)?;
        self.serial.flush().map_err(Error::Io)
    }

    fn wait_for_reply(
        &mut self,
        expected_command: u8,
        expected_payload: [u8; 2],
    ) -> Result<(), Error<SERIAL::Error>> {
        let mut window = [0u8; 6];
        let mut len = 0usize;

        loop {
            let byte = self.read_byte()?;
            if len < window.len() {
                window[len] = byte;
                len += 1;
            } else {
                window.copy_within(1.., 0);
                window[window.len() - 1] = byte;
            }

            if len < window.len() || window[0] != COMMAND_HEADER {
                continue;
            }

            if window[5] != checksum(&window[1..5]) {
                continue;
            }

            let actual_command = window[1];
            let payload = [window[3], window[4]];
            if window[2] != COMMAND_PAYLOAD_LEN
                || actual_command != expected_command
                || payload != expected_payload
            {
                return Err(Error::UnexpectedReply {
                    expected_command,
                    actual_command,
                    payload,
                });
            }

            return Ok(());
        }
    }
}

fn build_command(command: u8, payload: [u8; 2]) -> [u8; 6] {
    let mut frame = [0u8; 6];
    frame[0] = COMMAND_HEADER;
    frame[1] = command;
    frame[2] = COMMAND_PAYLOAD_LEN;
    frame[3] = payload[0];
    frame[4] = payload[1];
    frame[5] = checksum(&frame[1..5]);
    frame
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;
    use embedded_io::ErrorType;
    use std::{collections::VecDeque, vec::Vec};

    #[derive(Debug)]
    struct MockSerial {
        rx: VecDeque<u8>,
        tx: Vec<u8>,
    }

    impl MockSerial {
        fn from_rx(rx: &[u8]) -> Self {
            Self {
                rx: rx.iter().copied().collect(),
                tx: Vec::new(),
            }
        }
    }

    impl ErrorType for MockSerial {
        type Error = Infallible;
    }

    impl Read for MockSerial {
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

    impl Write for MockSerial {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.tx.extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn read_measurement_ignores_noise_and_resynchronizes() {
        let rx = [0x01, 0x02, 0x03, 0x5C, 0xD2, 0x04, 0x29];
        let mut lidar = Tsd10::new(MockSerial::from_rx(&rx));
        let measurement = lidar.read_measurement().unwrap();
        assert_eq!(measurement.distance_mm, 1_234);
        assert_eq!(measurement.distance_m(), Some(1.234));
    }

    #[test]
    fn start_measurement_writes_manual_command_and_reads_reply() {
        let rx = [0xAA, 0x5A, 0x8A, 0x02, 0x02, 0x00, 0x71];
        let mut lidar = Tsd10::new(MockSerial::from_rx(&rx));
        lidar.start_measurement().unwrap();

        let serial = lidar.release();
        assert_eq!(serial.tx, [0x5A, 0x0A, 0x02, 0x02, 0x00, 0xF1]);
    }

    #[test]
    fn stop_measurement_writes_manual_command_and_reads_reply() {
        let rx = [0x5A, 0x8A, 0x02, 0x00, 0x00, 0x73];
        let mut lidar = Tsd10::new(MockSerial::from_rx(&rx));
        lidar.stop_measurement().unwrap();

        let serial = lidar.release();
        assert_eq!(serial.tx, [0x5A, 0x0A, 0x02, 0x00, 0x00, 0xF3]);
    }

    #[test]
    fn set_baud_rate_matches_manual_command_bytes() {
        let mut lidar = Tsd10::new(MockSerial::from_rx(&[]));
        lidar.set_baud_rate(BaudRate::B115200).unwrap();

        let serial = lidar.release();
        assert_eq!(serial.tx, [0x5A, 0x06, 0x02, 0x80, 0x04, 0x73]);
    }

    #[test]
    fn set_baud_rate_with_ack_accepts_matching_reply() {
        let rx = [0x5A, 0x86, 0x02, 0x80, 0x04, 0xF3];
        let mut lidar = Tsd10::new(MockSerial::from_rx(&rx));
        lidar.set_baud_rate_with_ack(BaudRate::B115200).unwrap();

        let serial = lidar.release();
        assert_eq!(serial.tx, [0x5A, 0x06, 0x02, 0x80, 0x04, 0x73]);
    }
}

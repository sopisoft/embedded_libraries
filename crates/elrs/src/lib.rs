#![no_std]

//! ExpressLRS / CRSF protocol primitives for `no_std` targets.
//!
//! The crate focuses on the UART-visible CRSF layer used by ELRS systems:
//! frame encoding and parsing, RC channel packing, common telemetry packets,
//! parameter access, direct commands, and MSP-over-CRSF transport.

#[cfg(test)]
extern crate std;

use fugit::HertzU32;

pub mod address;
pub mod command;
pub mod crc;
pub mod frame;
pub mod msp;
pub mod parameter;
pub mod rc;
pub mod telemetry;

pub use address::DeviceAddress;
pub use command::{DirectCommand, DirectCommandError};
pub use crc::{CommandCrc, Crc8, FrameCrc};
pub use frame::{
    FRAME_TYPE_AIRSPEED, FRAME_TYPE_ATTITUDE, FRAME_TYPE_BAROMETRIC_ALTITUDE,
    FRAME_TYPE_BATTERY_SENSOR, FRAME_TYPE_DEVICE_INFO, FRAME_TYPE_DIRECT_COMMAND,
    FRAME_TYPE_FLIGHT_MODE, FRAME_TYPE_GPS, FRAME_TYPE_LINK_STATISTICS,
    FRAME_TYPE_LINK_STATISTICS_RX, FRAME_TYPE_LINK_STATISTICS_TX, FRAME_TYPE_MSP_REQUEST,
    FRAME_TYPE_MSP_RESPONSE, FRAME_TYPE_PARAMETER_ENTRY, FRAME_TYPE_PARAMETER_PING,
    FRAME_TYPE_PARAMETER_READ, FRAME_TYPE_PARAMETER_WRITE, FRAME_TYPE_RC_CHANNELS_PACKED,
    FRAME_TYPE_SUBSET_RC_CHANNELS_PACKED, Frame, FrameError, FrameParser, MAX_BODY_LEN,
    MAX_EXTENDED_PAYLOAD_LEN, MAX_FRAME_SIZE, ParseError,
};
pub use msp::{MspError, MspFrame, MspStatus};
pub use parameter::{
    CommandStatus, DeviceInfo, ParameterEntry, ParameterError, ParameterRead, ParameterType,
    ParameterWrite,
};
pub use rc::{
    CHANNEL_COUNT, RcChannels, RcError, SubsetRcChannels, SubsetResolution, micros_to_ticks,
    ticks_to_micros,
};
pub use telemetry::{
    Airspeed, Attitude, BarometricAltitude, BatterySensor, FlightMode, Gps, LinkStatistics,
    LinkStatisticsRx, LinkStatisticsTx, TelemetryError, encode_flight_mode,
};

/// Common CRSF sync byte used by flight controllers.
pub const SYNC_BYTE: u8 = DeviceAddress::FLIGHT_CONTROLLER.as_u8();

/// A common ELRS RC update rate.
pub const DEFAULT_RC_RATE: HertzU32 = HertzU32::from_raw(250);

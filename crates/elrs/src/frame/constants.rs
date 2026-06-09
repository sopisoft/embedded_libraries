/// GPS telemetry frame.
pub const FRAME_TYPE_GPS: u8 = 0x02;
/// Battery telemetry frame.
pub const FRAME_TYPE_BATTERY_SENSOR: u8 = 0x08;
/// Barometric altitude and vario telemetry frame.
pub const FRAME_TYPE_BAROMETRIC_ALTITUDE: u8 = 0x09;
/// Airspeed telemetry frame.
pub const FRAME_TYPE_AIRSPEED: u8 = 0x0A;
/// Link statistics frame.
pub const FRAME_TYPE_LINK_STATISTICS: u8 = 0x14;
/// RC channels packed frame.
pub const FRAME_TYPE_RC_CHANNELS_PACKED: u8 = 0x16;
/// Subset RC channels frame.
pub const FRAME_TYPE_SUBSET_RC_CHANNELS_PACKED: u8 = 0x17;
/// RX link statistics frame.
pub const FRAME_TYPE_LINK_STATISTICS_RX: u8 = 0x1C;
/// TX link statistics frame.
pub const FRAME_TYPE_LINK_STATISTICS_TX: u8 = 0x1D;
/// Attitude frame.
pub const FRAME_TYPE_ATTITUDE: u8 = 0x1E;
/// Flight mode frame.
pub const FRAME_TYPE_FLIGHT_MODE: u8 = 0x21;
/// Parameter ping frame.
pub const FRAME_TYPE_PARAMETER_PING: u8 = 0x28;
/// Parameter device information frame.
pub const FRAME_TYPE_DEVICE_INFO: u8 = 0x29;
/// Parameter entry frame.
pub const FRAME_TYPE_PARAMETER_ENTRY: u8 = 0x2B;
/// Parameter read frame.
pub const FRAME_TYPE_PARAMETER_READ: u8 = 0x2C;
/// Parameter write frame.
pub const FRAME_TYPE_PARAMETER_WRITE: u8 = 0x2D;
/// Direct command frame.
pub const FRAME_TYPE_DIRECT_COMMAND: u8 = 0x32;
/// MSP request frame.
pub const FRAME_TYPE_MSP_REQUEST: u8 = 0x7A;
/// MSP response frame.
pub const FRAME_TYPE_MSP_RESPONSE: u8 = 0x7B;
/// ArduPilot passthrough frame.
pub const FRAME_TYPE_ARDUPILOT_PASSTHROUGH: u8 = 0x80;

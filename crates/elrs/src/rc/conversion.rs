use crate::frame::FrameError;

/// Number of RC channels in the standard CRSF packed frame.
pub const CHANNEL_COUNT: usize = 16;
/// Size of the standard CRSF channel payload.
pub const RC_CHANNELS_PAYLOAD_LEN: usize = 22;

/// Errors returned by RC helpers.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RcError {
    /// The payload length does not match the expected format.
    InvalidLength,
    /// The subset frame configuration is invalid.
    InvalidConfiguration,
    /// Too many channels were supplied for the payload format.
    TooManyChannels,
    /// The frame operation failed.
    Frame(FrameError),
}

impl From<FrameError> for RcError {
    fn from(value: FrameError) -> Self {
        Self::Frame(value)
    }
}

/// Converts CRSF channel ticks to microseconds.
pub const fn ticks_to_micros(ticks: u16) -> u16 {
    let delta = ticks as i32 - 992;
    let micros = delta * 5 / 8 + 1500;
    if micros < 0 { 0 } else { micros as u16 }
}

/// Converts microseconds to CRSF channel ticks.
pub const fn micros_to_ticks(micros: u16) -> u16 {
    let delta = micros as i32 - 1500;
    let ticks = delta * 8 / 5 + 992;
    if ticks < 0 {
        0
    } else if ticks > 0x07FF {
        0x07FF
    } else {
        ticks as u16
    }
}

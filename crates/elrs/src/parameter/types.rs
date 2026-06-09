use crate::frame::FrameError;

/// Errors returned by parameter helpers.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ParameterError {
    /// The payload length was invalid.
    InvalidLength,
    /// The payload text was not valid UTF-8.
    InvalidText,
    /// The payload could not fit into the available buffer.
    PayloadTooLong,
    /// The frame operation failed.
    Frame(FrameError),
}

impl From<FrameError> for ParameterError {
    fn from(value: FrameError) -> Self {
        Self::Frame(value)
    }
}

/// Parameter data types defined by CRSF.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ParameterType {
    UInt8,
    Int8,
    UInt16,
    Int16,
    UInt32,
    Int32,
    Float,
    TextSelection,
    String,
    Folder,
    Info,
    Command,
    OutOfRange,
    Unknown(u8),
}

impl ParameterType {
    /// Returns whether the hidden bit is set.
    pub const fn is_hidden(raw: u8) -> bool {
        (raw & 0x80) != 0
    }

    /// Returns the parameter kind from the raw data type byte.
    pub const fn from_raw(raw: u8) -> Self {
        match raw & 0x7F {
            0 => Self::UInt8,
            1 => Self::Int8,
            2 => Self::UInt16,
            3 => Self::Int16,
            4 => Self::UInt32,
            5 => Self::Int32,
            8 => Self::Float,
            9 => Self::TextSelection,
            10 => Self::String,
            11 => Self::Folder,
            12 => Self::Info,
            13 => Self::Command,
            127 => Self::OutOfRange,
            other => Self::Unknown(other),
        }
    }
}

/// Command parameter state.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CommandStatus {
    Ready,
    Start,
    Progress,
    ConfirmationNeeded,
    Confirm,
    Cancel,
    Poll,
    Unknown(u8),
}

impl CommandStatus {
    /// Parses the raw command status value.
    pub const fn from_raw(raw: u8) -> Self {
        match raw {
            0 => Self::Ready,
            1 => Self::Start,
            2 => Self::Progress,
            3 => Self::ConfirmationNeeded,
            4 => Self::Confirm,
            5 => Self::Cancel,
            6 => Self::Poll,
            other => Self::Unknown(other),
        }
    }

    /// Returns the raw command status value.
    pub const fn as_raw(self) -> u8 {
        match self {
            Self::Ready => 0,
            Self::Start => 1,
            Self::Progress => 2,
            Self::ConfirmationNeeded => 3,
            Self::Confirm => 4,
            Self::Cancel => 5,
            Self::Poll => 6,
            Self::Unknown(value) => value,
        }
    }
}

//! CRSF parameter and device-information payloads.

mod device_info;
mod entry;
mod requests;
#[cfg(test)]
mod tests;
mod types;

pub use device_info::DeviceInfo;
pub use entry::ParameterEntry;
pub use requests::{ParameterRead, ParameterWrite, encode_parameter_ping_frame};
pub use types::{CommandStatus, ParameterError, ParameterType};

//! RC channel payload helpers.

mod conversion;
mod standard;
mod subset;
#[cfg(test)]
mod tests;

pub use conversion::{
    CHANNEL_COUNT, RC_CHANNELS_PAYLOAD_LEN, RcError, micros_to_ticks, ticks_to_micros,
};
pub use standard::RcChannels;
pub use subset::{SubsetRcChannels, SubsetResolution};

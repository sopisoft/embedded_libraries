//! Common CRSF telemetry payloads.

mod airspeed;
mod attitude;
mod baro;
mod battery;
mod flight_mode;
mod gps;
mod link;
mod shared;
#[cfg(test)]
mod tests;

pub use airspeed::Airspeed;
pub use attitude::Attitude;
pub use baro::BarometricAltitude;
pub use battery::BatterySensor;
pub use flight_mode::{FlightMode, encode_flight_mode};
pub use gps::Gps;
pub use link::{LinkStatistics, LinkStatisticsRx, LinkStatisticsTx};
pub use shared::TelemetryError;

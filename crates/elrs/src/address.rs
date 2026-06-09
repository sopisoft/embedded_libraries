//! Known CRSF device addresses.

/// A CRSF device address.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct DeviceAddress(u8);

impl DeviceAddress {
    /// Broadcast address.
    pub const BROADCAST: Self = Self(0x00);
    /// Cloud endpoint.
    pub const CLOUD: Self = Self(0x0E);
    /// USB device.
    pub const USB: Self = Self(0x10);
    /// Bluetooth or Wi-Fi module.
    pub const BLUETOOTH_OR_WIFI: Self = Self(0x12);
    /// Wi-Fi receiver for simulator or mobile use.
    pub const WIFI_RECEIVER: Self = Self(0x13);
    /// Video receiver.
    pub const VIDEO_RECEIVER: Self = Self(0x14);
    /// OSD or CORE.
    pub const OSD: Self = Self(0x80);
    /// ESC 1.
    pub const ESC1: Self = Self(0x90);
    /// ESC 2.
    pub const ESC2: Self = Self(0x91);
    /// ESC 3.
    pub const ESC3: Self = Self(0x92);
    /// ESC 4.
    pub const ESC4: Self = Self(0x93);
    /// ESC 5.
    pub const ESC5: Self = Self(0x94);
    /// ESC 6.
    pub const ESC6: Self = Self(0x95);
    /// ESC 7.
    pub const ESC7: Self = Self(0x96);
    /// ESC 8.
    pub const ESC8: Self = Self(0x97);
    /// Voltage and current sensor.
    pub const CURRENT_SENSOR: Self = Self(0xC0);
    /// GPS.
    pub const GPS: Self = Self(0xC2);
    /// Blackbox.
    pub const BLACKBOX: Self = Self(0xC4);
    /// Flight controller.
    pub const FLIGHT_CONTROLLER: Self = Self(0xC8);
    /// Race tag.
    pub const RACE_TAG: Self = Self(0xCC);
    /// Video transmitter.
    pub const VTX: Self = Self(0xCE);
    /// Remote control handset.
    pub const REMOTE_CONTROL: Self = Self(0xEA);
    /// Repeater receiver.
    pub const REPEATER_RECEIVER: Self = Self(0xEB);
    /// CRSF receiver.
    pub const RECEIVER: Self = Self(0xEC);
    /// Repeater transmitter module.
    pub const REPEATER_TRANSMITTER: Self = Self(0xED);
    /// CRSF transmitter module.
    pub const TRANSMITTER: Self = Self(0xEE);

    /// Creates an address from a raw byte.
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    /// Returns the raw byte value.
    pub const fn as_u8(self) -> u8 {
        self.0
    }
}

impl From<u8> for DeviceAddress {
    fn from(value: u8) -> Self {
        Self(value)
    }
}

impl From<DeviceAddress> for u8 {
    fn from(value: DeviceAddress) -> Self {
        value.0
    }
}

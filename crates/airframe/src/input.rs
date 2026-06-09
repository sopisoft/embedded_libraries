//! ELRS / CRSF input decoding for fixed-wing applications.

use control::shape_rc_command;
use elrs::{RcChannels, SubsetRcChannels};

/// Symmetric RC axis configuration.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AxisConfig {
    pub min_us: u16,
    pub center_us: u16,
    pub max_us: u16,
    pub deadband: f32,
    pub expo: f32,
    pub rate: f32,
    pub reversed: bool,
}

impl AxisConfig {
    /// Common `1000 / 1500 / 2000 us` symmetric axis.
    pub const fn standard() -> Self {
        Self {
            min_us: 1_000,
            center_us: 1_500,
            max_us: 2_000,
            deadband: 0.03,
            expo: 0.25,
            rate: 1.0,
            reversed: false,
        }
    }

    /// Decodes one PWM-style channel into `[-1, 1]`.
    pub fn decode(self, pulse_us: u16) -> f32 {
        let pulse_us = pulse_us.clamp(self.min_us, self.max_us);
        let mut normalized = if pulse_us >= self.center_us {
            let span = (self.max_us - self.center_us).max(1) as f32;
            (pulse_us - self.center_us) as f32 / span
        } else {
            let span = (self.center_us - self.min_us).max(1) as f32;
            -((self.center_us - pulse_us) as f32 / span)
        };
        if self.reversed {
            normalized = -normalized;
        }
        shape_rc_command(normalized, self.deadband, self.expo, self.rate)
    }
}

/// Simple boolean switch decode.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SwitchConfig {
    pub threshold_us: u16,
    pub reversed: bool,
}

impl SwitchConfig {
    /// Returns `true` when the channel is "high".
    pub const fn decode(self, pulse_us: u16) -> bool {
        let state = pulse_us >= self.threshold_us;
        if self.reversed { !state } else { state }
    }
}

/// Channel assignment for a conventional fixed-wing receiver layout.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RcChannelMap {
    pub roll: usize,
    pub pitch: usize,
    pub throttle: usize,
    pub yaw: usize,
    pub flaps: Option<usize>,
    pub attitude_hold: Option<usize>,
}

impl RcChannelMap {
    /// Typical AETR-style mapping with CH5 as attitude-hold and CH6 as flaps.
    pub const fn conventional_aetr() -> Self {
        Self {
            roll: 0,
            pitch: 1,
            throttle: 2,
            yaw: 3,
            attitude_hold: Some(4),
            flaps: Some(5),
        }
    }
}

/// Decoded pilot command block.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PilotCommand {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
    pub flaps: f32,
    pub attitude_hold_enabled: bool,
}

/// Input decoder configuration.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RcInputConfig {
    pub map: RcChannelMap,
    pub roll: AxisConfig,
    pub pitch: AxisConfig,
    pub yaw: AxisConfig,
    pub throttle_reversed: bool,
    pub flaps_reversed: bool,
    pub attitude_hold_switch: SwitchConfig,
}

impl RcInputConfig {
    /// Practical default for a 6-channel ELRS receiver on a conventional aircraft.
    pub const fn conventional_aetr() -> Self {
        Self {
            map: RcChannelMap::conventional_aetr(),
            roll: AxisConfig::standard(),
            pitch: AxisConfig::standard(),
            yaw: AxisConfig::standard(),
            throttle_reversed: false,
            flaps_reversed: false,
            attitude_hold_switch: SwitchConfig {
                threshold_us: 1_600,
                reversed: false,
            },
        }
    }

    /// Decodes CRSF channels into pilot commands.
    pub fn decode(&self, channels: &RcChannels) -> PilotCommand {
        let roll = self
            .roll
            .decode(read_channel(channels, self.map.roll, 1_500));
        let pitch = self
            .pitch
            .decode(read_channel(channels, self.map.pitch, 1_500));
        let yaw = self.yaw.decode(read_channel(channels, self.map.yaw, 1_500));
        let throttle = decode_unipolar(
            read_channel(channels, self.map.throttle, 1_000),
            self.throttle_reversed,
        );
        let flaps = self
            .map
            .flaps
            .map(|index| decode_unipolar(read_channel(channels, index, 1_000), self.flaps_reversed))
            .unwrap_or(0.0);
        let attitude_hold_enabled = self
            .map
            .attitude_hold
            .map(|index| {
                self.attitude_hold_switch
                    .decode(read_channel(channels, index, 1_000))
            })
            .unwrap_or(false);

        PilotCommand {
            roll,
            pitch,
            yaw,
            throttle,
            flaps,
            attitude_hold_enabled,
        }
    }
}

/// Applies one subset CRSF channel update onto a full 16-channel state block.
pub fn apply_subset_channels(channels: &mut RcChannels, subset: &SubsetRcChannels) {
    let start = subset.starting_channel as usize;
    let mut i = 0usize;
    while i < subset.values.len() {
        let channel_index = start + i;
        if channel_index < channels.values.len() {
            channels.values[channel_index] = subset.values[i];
        }
        i += 1;
    }
}

fn read_channel(channels: &RcChannels, index: usize, default_us: u16) -> u16 {
    channels.micros(index).unwrap_or(default_us)
}

fn decode_unipolar(pulse_us: u16, reversed: bool) -> f32 {
    let pulse_us = pulse_us.clamp(1_000, 2_000);
    let normalized = (pulse_us - 1_000) as f32 / 1_000.0;
    if reversed {
        1.0 - normalized
    } else {
        normalized
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axis_decode_maps_center_to_zero() {
        let axis = AxisConfig::standard();
        assert!(axis.decode(1_500).abs() < 1.0e-6);
        assert!(axis.decode(2_000) > 0.9);
        assert!(axis.decode(1_000) < -0.9);
    }

    #[test]
    fn rc_input_config_decodes_aetr_layout() {
        let config = RcInputConfig::conventional_aetr();
        let channels = RcChannels::from_micros([
            1_700, 1_400, 1_300, 1_550, 1_800, 1_600, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000,
            1_000, 1_000, 1_000, 1_000,
        ]);
        let command = config.decode(&channels);
        assert!(command.roll > 0.0);
        assert!(command.pitch < 0.0);
        assert!(command.attitude_hold_enabled);
        assert!(command.flaps > 0.5);
    }

    #[test]
    fn subset_update_overwrites_only_target_channels() {
        let mut channels = RcChannels::from_micros([
            1_500, 1_500, 1_000, 1_500, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000,
            1_000, 1_000, 1_000, 1_000,
        ]);
        let mut subset = SubsetRcChannels::new(1, elrs::SubsetResolution::Bits11, false).unwrap();
        subset.push_micros(1_700).unwrap();
        subset.push_micros(1_300).unwrap();

        apply_subset_channels(&mut channels, &subset);
        assert_eq!(channels.micros(0).unwrap(), 1_500);
        assert_eq!(channels.micros(1).unwrap(), 1_700);
        assert_eq!(channels.micros(2).unwrap(), 1_300);
    }
}

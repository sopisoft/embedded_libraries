use super::super::{
    RcChannels, SubsetRcChannels, SubsetResolution, micros_to_ticks, ticks_to_micros,
};

#[test]
fn rc_pack_roundtrip_works() {
    let mut values = [0u16; 16];
    let mut i = 0;
    while i < 16 {
        values[i] = (i as u16 * 111) & 0x07FF;
        i += 1;
    }
    let packet = RcChannels::new(values);
    let unpacked = RcChannels::unpack(packet.pack());
    assert_eq!(packet, unpacked);
}

#[test]
fn microsecond_conversion_is_stable_around_center() {
    let ticks = micros_to_ticks(1500);
    assert_eq!(ticks, 992);
    assert_eq!(ticks_to_micros(ticks), 1500);
}

#[test]
fn subset_roundtrip_works() {
    let mut subset = SubsetRcChannels::new(0, SubsetResolution::Bits11, false).unwrap();
    subset.push_micros(1000).unwrap();
    subset.push_micros(1500).unwrap();
    subset.push_micros(2000).unwrap();
    let payload = subset.encode_payload().unwrap();
    let decoded = SubsetRcChannels::decode(payload.as_slice()).unwrap();
    assert_eq!(decoded.starting_channel, 0);
    assert_eq!(decoded.resolution, SubsetResolution::Bits11);
    assert_eq!(decoded.values.len(), 3);
    assert_eq!(decoded.values[1], 992);
}

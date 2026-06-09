use elrs::{
    DeviceAddress, FRAME_TYPE_LINK_STATISTICS, FRAME_TYPE_RC_CHANNELS_PACKED, Frame, FrameParser,
    LinkStatistics, RcChannels,
};

fn main() {
    // This example shows the most common ELRS / CRSF workflow on a microcontroller:
    // 1. produce CRSF frames,
    // 2. feed bytes into a UART parser,
    // 3. inspect complete frames once they arrive.
    //
    // The example uses synthetic frames so it can run on a desktop.

    // ---------------------------------------------------------------------
    // 1) Build an RC channel frame
    // ---------------------------------------------------------------------
    // `from_micros()` is the most convenient entry point if your application already uses
    // standard servo pulse widths such as 1000 / 1500 / 2000 us.
    let rc = RcChannels::from_micros([
        1000, 1500, 2000, 1500, 1000, 1500, 2000, 1500, 1000, 1500, 2000, 1500, 1000, 1500, 2000,
        1500,
    ]);
    let rc_frame = rc.encode_frame(DeviceAddress::FLIGHT_CONTROLLER).unwrap();

    // ---------------------------------------------------------------------
    // 2) Build a telemetry frame
    // ---------------------------------------------------------------------
    let link_stats = LinkStatistics {
        up_rssi_ant1: 55,
        up_rssi_ant2: 58,
        up_link_quality: 100,
        up_snr: 12,
        active_antenna: 1,
        rf_profile: 2,
        up_rf_power: 3,
        down_rssi: 60,
        down_link_quality: 100,
        down_snr: 8,
    };
    let link_stats_frame = Frame::new(
        DeviceAddress::RECEIVER,
        FRAME_TYPE_LINK_STATISTICS,
        &link_stats.encode_payload(),
    )
    .unwrap();

    // Concatenate both frames into a single byte stream, exactly like a UART would deliver them.
    let mut stream = rc_frame.to_bytes().unwrap();
    stream
        .extend_from_slice(link_stats_frame.to_bytes().unwrap().as_slice())
        .unwrap();

    // ---------------------------------------------------------------------
    // 3) Parse the byte stream
    // ---------------------------------------------------------------------
    let mut parser = FrameParser::new();
    for byte in stream {
        if let Some(frame) = parser.push(byte) {
            let frame = frame.unwrap();
            match frame.frame_type {
                FRAME_TYPE_RC_CHANNELS_PACKED => {
                    // Standard RC payloads use 22 bytes for 16 channels.
                    let payload: [u8; 22] = frame.payload().try_into().unwrap();
                    let decoded = RcChannels::unpack(payload);
                    println!(
                        "RC channel 1/2/3: {} {} {} us",
                        decoded.micros(0).unwrap(),
                        decoded.micros(1).unwrap(),
                        decoded.micros(2).unwrap()
                    );
                }
                FRAME_TYPE_LINK_STATISTICS => {
                    let stats = LinkStatistics::decode(frame.payload()).unwrap();
                    println!(
                        "Link stats: uplink={}%, downlink={}%, active antenna={}",
                        stats.up_link_quality, stats.down_link_quality, stats.active_antenna
                    );
                }
                other => {
                    println!("Unhandled frame type: 0x{other:02X}");
                }
            }
        }
    }
}

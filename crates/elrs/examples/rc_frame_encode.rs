use elrs::{DeviceAddress, RcChannels};

fn main() {
    // Many RC applications already think in microseconds:
    // - 1000 us: low
    // - 1500 us: center
    // - 2000 us: high
    //
    // `RcChannels::from_micros()` converts those familiar values into the
    // packed CRSF representation used by ELRS links.
    let channels = RcChannels::from_micros([
        1000, 1500, 2000, 1500, 1000, 1500, 2000, 1500, 1000, 1500, 2000, 1500, 1000, 1500, 2000,
        1500,
    ]);

    let frame = channels
        .encode_frame(DeviceAddress::FLIGHT_CONTROLLER)
        .unwrap();
    let bytes = frame.to_bytes().unwrap();

    println!("Encoded CRSF RC frame length: {} bytes", bytes.len());
    println!("Address byte: 0x{:02X}", bytes[0]);
    println!("Frame type:  0x{:02X}", bytes[2]);
    println!("CRC byte:    0x{:02X}", bytes[bytes.len() - 1]);

    // Round-trip decode the 22-byte RC payload so you can confirm the mapping.
    let payload: [u8; 22] = frame.payload().try_into().unwrap();
    let decoded = RcChannels::unpack(payload);
    println!(
        "Decoded channel 1/2/3 in microseconds: {} {} {}",
        decoded.micros(0).unwrap(),
        decoded.micros(1).unwrap(),
        decoded.micros(2).unwrap()
    );
}

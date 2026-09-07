#![no_std]
#![no_main]
// Wiring idea:
// - CRSF input  -> UART0 RX on GPIO1
// - CRSF source <- UART0 TX on GPIO0 (optional, reserved if you later add replies)

mod embedded_example {
    use core::sync::atomic::{AtomicU32, Ordering};

    use defmt_rtt as _;
    use panic_probe as _;
    use rp235x_hal as hal;

    use elrs::{
        FRAME_TYPE_LINK_STATISTICS, FRAME_TYPE_RC_CHANNELS_PACKED, FrameParser, LinkStatistics,
        RcChannels,
    };
    use hal::clocks::Clock;
    use hal::fugit::RateExtU32;
    use hal::uart::{DataBits, StopBits, UartConfig};

    #[unsafe(link_section = ".start_block")]
    #[used]
    pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

    static DEFMT_TIMESTAMP: AtomicU32 = AtomicU32::new(0);
    defmt::timestamp!("{=u32}", DEFMT_TIMESTAMP.fetch_add(1, Ordering::Relaxed));

    const XTAL_FREQ_HZ: u32 = 12_000_000;
    const CHANNEL_MIN_US: u16 = 1000;
    const CHANNEL_MAX_US: u16 = 2000;

    fn normalized_channel_01(micros: u16) -> (u8, u8) {
        let clamped = micros.clamp(CHANNEL_MIN_US, CHANNEL_MAX_US);
        let scaled =
            ((clamped - CHANNEL_MIN_US) as u32 * 100) / (CHANNEL_MAX_US - CHANNEL_MIN_US) as u32;
        let whole = (scaled / 100) as u8;
        let frac = (scaled % 100) as u8;
        (whole, frac)
    }

    #[hal::entry]
    fn main() -> ! {
        let mut pac = hal::pac::Peripherals::take().unwrap();
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

        let clocks = hal::clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .unwrap();

        let sio = hal::Sio::new(pac.SIO);
        let pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let crsf_pins = (pins.gpio0.into_function(), pins.gpio1.into_function());
        let crsf_uart = hal::uart::UartPeripheral::new(pac.UART0, crsf_pins, &mut pac.RESETS)
            .enable(
                UartConfig::new(420000u32.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        let mut parser = FrameParser::new();
        let mut byte = [0u8; 1];

        defmt::info!("ELRS / CRSF RX example ready");

        loop {
            if !crsf_uart.uart_is_readable() {
                continue;
            }

            if crsf_uart.read_full_blocking(&mut byte).is_err() {
                continue;
            }

            if let Some(parsed) = parser.push(byte[0]) {
                match parsed {
                    Ok(frame) => match frame.frame_type {
                        FRAME_TYPE_RC_CHANNELS_PACKED => {
                            let Ok(payload) = frame.payload().try_into() else {
                                continue;
                            };
                            let channels = RcChannels::unpack(payload);
                            let ch1 = normalized_channel_01(channels.micros(0).unwrap());
                            let ch2 = normalized_channel_01(channels.micros(1).unwrap());
                            let ch3 = normalized_channel_01(channels.micros(2).unwrap());
                            let ch4 = normalized_channel_01(channels.micros(3).unwrap());
                            defmt::info!(
                                "Ch1={:?}.{:02} Ch2={:?}.{:02} Ch3={:?}.{:02} Ch4={:?}.{:02}",
                                ch1.0,
                                ch1.1,
                                ch2.0,
                                ch2.1,
                                ch3.0,
                                ch3.1,
                                ch4.0,
                                ch4.1
                            );
                        }
                        FRAME_TYPE_LINK_STATISTICS => {
                            if let Ok(stats) = LinkStatistics::decode(frame.payload()) {
                                defmt::info!(
                                    "Link: uplink={:?} downlink={:?} snr={:?}",
                                    stats.up_link_quality,
                                    stats.down_link_quality,
                                    stats.up_snr
                                );
                            }
                        }
                        other => {
                            defmt::info!("Frame type {:?}", other);
                        }
                    },
                    Err(_error) => defmt::warn!("Parse error"),
                }
            }
        }
    }
}

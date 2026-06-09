#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

// This example receives ELRS / CRSF frames on UART0 and prints decoded
// information on UART1.
//
// Wiring idea:
// - CRSF input  -> UART0 RX on GPIO1
// - CRSF source <- UART0 TX on GPIO0 (optional, reserved if you later add replies)
// - Debug TX    -> UART1 TX on GPIO4
// - Debug RX    -> UART1 RX on GPIO5
//
// Use this when bringing up a receiver or flight-controller CRSF port and you
// want to confirm that RC frames and telemetry are arriving correctly.

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod embedded_example {
    use core::fmt::Write;

    use panic_halt as _;
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

    const XTAL_FREQ_HZ: u32 = 12_000_000;

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

        let debug_pins = (pins.gpio4.into_function(), pins.gpio5.into_function());
        let mut debug_uart = hal::uart::UartPeripheral::new(pac.UART1, debug_pins, &mut pac.RESETS)
            .enable(
                UartConfig::new(115200u32.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        let mut parser = FrameParser::new();
        let mut byte = [0u8; 1];

        writeln!(debug_uart, "\r\nELRS / CRSF RX example ready\r").ok();

        loop {
            if !crsf_uart.uart_is_readable() {
                continue;
            }

            if crsf_uart.read_full_blocking(&mut byte).is_err() {
                writeln!(debug_uart, "UART read error\r").ok();
                continue;
            }

            if let Some(parsed) = parser.push(byte[0]) {
                match parsed {
                    Ok(frame) => match frame.frame_type {
                        FRAME_TYPE_RC_CHANNELS_PACKED => {
                            let payload: [u8; 22] = frame.payload().try_into().unwrap();
                            let channels = RcChannels::unpack(payload);
                            writeln!(
                                debug_uart,
                                "RC: roll={} pitch={} throttle={} yaw={}\r",
                                channels.micros(0).unwrap(),
                                channels.micros(1).unwrap(),
                                channels.micros(2).unwrap(),
                                channels.micros(3).unwrap()
                            )
                            .ok();
                        }
                        FRAME_TYPE_LINK_STATISTICS => {
                            if let Ok(stats) = LinkStatistics::decode(frame.payload()) {
                                writeln!(
                                    debug_uart,
                                    "Link: uplink={} downlink={} snr={}\r",
                                    stats.up_link_quality, stats.down_link_quality, stats.up_snr
                                )
                                .ok();
                            }
                        }
                        other => {
                            writeln!(debug_uart, "Frame type 0x{other:02X}\r").ok();
                        }
                    },
                    Err(error) => {
                        writeln!(debug_uart, "Parse error: {:?}\r", error).ok();
                    }
                }
            }
        }
    }
}

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn main() {
    println!(
        "This example is for RP2350 hardware. Build it with \
         `cargo build -p elrs --example rp235x_crsf_uart_rx --target thumbv8m.main-none-eabihf`."
    );
}

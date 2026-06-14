#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

// This example is a more practical RP2350 + MCP3208 setup than the single-
// channel introduction. It scans all eight single-ended channels and prints
// the result table over UART, which is often how you start bringing up sensors
// such as potentiometers, pressure transducers, or current monitors.

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod embedded_example {
    use core::fmt::Write;

    use mcp3208::{Channel, Mcp3208};
    use panic_halt as _;
    use rp235x_hal as hal;

    use hal::clocks::Clock;
    use hal::fugit::RateExtU32;
    use hal::uart::{DataBits, StopBits, UartConfig};

    #[unsafe(link_section = ".start_block")]
    #[used]
    pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

    const XTAL_FREQ_HZ: u32 = 12_000_000;
    const VREF_MV: u16 = 3300;

    fn wait_until(timer: &hal::Timer<hal::timer::CopyableTimer0>, deadline: hal::timer::Instant) {
        while timer.get_counter() < deadline {
            core::hint::spin_loop();
        }
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

        let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
        let sio = hal::Sio::new(pac.SIO);
        let pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let uart_pins = (pins.gpio0.into_function(), pins.gpio1.into_function());
        let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
            .enable(
                UartConfig::new(115200u32.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        let spi_miso = pins.gpio4.into_function::<hal::gpio::FunctionSpi>();
        let spi_sclk = pins.gpio6.into_function::<hal::gpio::FunctionSpi>();
        let spi_mosi = pins.gpio7.into_function::<hal::gpio::FunctionSpi>();
        let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk)).init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
            1u32.MHz(),
            embedded_hal::spi::MODE_0,
        );

        let cs = pins.gpio8.into_function::<hal::gpio::FunctionSioOutput>();
        let mut adc = Mcp3208::new(spi, cs);

        writeln!(uart, "\r\nScanning MCP3208 channels on RP2350\r").ok();
        let period = hal::fugit::MicrosDurationU32::from_ticks(1_000_000);
        let mut next_tick = timer.get_counter() + period;

        loop {
            for channel_index in 0..8u8 {
                let channel = Channel::SingleEnded(channel_index);
                let raw = adc.read_raw(channel).unwrap_or(0);
                let mv = adc.read_voltage_mv(channel, VREF_MV).unwrap_or(0);
                writeln!(uart, "CH{channel_index}: raw={raw:4} voltage={mv:4} mV\r").ok();
            }
            writeln!(uart, "\r").ok();
            wait_until(&timer, next_tick);
            next_tick += period;
        }
    }
}

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn main() {
    println!(
        "This example is for RP2350 hardware. Build it with \
         `cargo build -p mcp3208 --example rp235x_spi_mcp3208_scan_all --target thumbv8m.main-none-eabihf`."
    );
}

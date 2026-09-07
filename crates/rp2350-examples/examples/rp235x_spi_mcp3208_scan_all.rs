#![no_std]
#![no_main]

mod embedded_example {
    use core::sync::atomic::{AtomicU32, Ordering};

    use defmt_rtt as _;
    use mcp3208::{Channel, Mcp3208};
    use panic_probe as _;
    use rp235x_hal as hal;

    use hal::clocks::Clock;
    use hal::fugit::RateExtU32;

    #[unsafe(link_section = ".start_block")]
    #[used]
    pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

    static DEFMT_TIMESTAMP: AtomicU32 = AtomicU32::new(0);
    defmt::timestamp!("{=u32}", DEFMT_TIMESTAMP.fetch_add(1, Ordering::Relaxed));

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

        defmt::info!("Scanning MCP3208 channels on RP2350");
        let period = hal::fugit::MicrosDurationU32::from_ticks(1_000_000);
        let mut next_tick = timer.get_counter() + period;

        loop {
            for channel_index in 0..8u8 {
                let channel = Channel::SingleEnded(channel_index);
                let raw = adc.read_raw(channel).unwrap_or(0);
                let mv = adc.read_voltage_mv(channel, VREF_MV).unwrap_or(0);
                defmt::info!("CH{:?}: raw={:?} voltage={:?} mV", channel_index, raw, mv);
            }
            wait_until(&timer, next_tick);
            next_tick += period;
        }
    }
}

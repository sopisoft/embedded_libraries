#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

// This example uses the Akizuki AE-LPS25HB module over SPI on an RP2350-class
// MCU such as Raspberry Pi Pico 2.
//
// Suggested wiring:
// - GPIO6  -> SPC / SCLK
// - GPIO7  -> SDI / MOSI
// - GPIO4  -> SDO / MISO
// - GPIO8  -> CS
// - 3V3    -> VDD
// - GND    -> GND
//
// Important module-specific notes:
// - use the module in SPI mode
// - do not short `J1` and `J2`; they are I2C pull-up jumpers
// - use the 4-wire SPI connection shown above

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod embedded_example {
    use core::sync::atomic::{AtomicU32, Ordering};

    use defmt_rtt as _;
    use lps25hb::{
        Config, DEVICE_ID, Lps25hb, STANDARD_SEA_LEVEL_PRESSURE_HPA, pressure_to_altitude_m,
    };
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
    const SEA_LEVEL_PRESSURE_HPA: f32 = STANDARD_SEA_LEVEL_PRESSURE_HPA;

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
            1.MHz(),
            embedded_hal::spi::MODE_0,
        );
        let cs = pins.gpio8.into_function::<hal::gpio::FunctionSioOutput>();

        let mut barometer = Lps25hb::new_spi(spi, cs);
        barometer.init(Config::akizuki_style()).unwrap();

        defmt::info!("RP2350 + LPS25HB SPI example");
        defmt::info!(
            "WHO_AM_I = {:?} (expected {:?})",
            barometer.who_am_i().unwrap(),
            DEVICE_ID
        );

        let period = hal::fugit::MicrosDurationU32::from_ticks(250_000);
        let mut next_tick = timer.get_counter() + period;
        loop {
            let measurement = barometer.read_measurement().unwrap();
            let altitude_m =
                pressure_to_altitude_m(measurement.pressure_hpa, SEA_LEVEL_PRESSURE_HPA);
            defmt::info!(
                "pressure={:?} hPa temp={:?} C altitude={:?} m",
                measurement.pressure_hpa,
                measurement.temperature_c,
                altitude_m,
            );
            wait_until(&timer, next_tick);
            next_tick += period;
        }
    }
}

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn main() {
    println!(
        "This example is for RP2350 hardware. Build it with \
         `cargo build -p lps25hb --example rp235x_spi_lps25hb --target thumbv8m.main-none-eabihf`."
    );
}

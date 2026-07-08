#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

// This example targets an RP2350-class MCU such as Raspberry Pi Pico 2.
//
// It connects to the Akizuki AE-LPS25HB module over I2C and logs pressure,
// temperature, and barometric altitude with `defmt`.
//
// Suggested wiring:
// - GPIO18 -> SDA
// - GPIO19 -> SCL
// - 3V3    -> VDD
// - GND    -> GND
//
// Important module-specific notes:
// - connect `CS` to `VDD` to select I2C mode
// - connect `SA0` to `GND` for I2C address `0x5C`
//   or to `VDD` for `0x5D`
// - enable the `J1` / `J2` pull-up jumpers only if your bus does not already
//   provide I2C pull-ups
//
// If you know your local field pressure or QNH, replace
// `SEA_LEVEL_PRESSURE_HPA` below so the altitude estimate becomes meaningful.

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod embedded_example {
    use core::sync::atomic::{AtomicU32, Ordering};

    use defmt_rtt as _;
    use lps25hb::{
        Address, Config, DEVICE_ID, Lps25hb, STANDARD_SEA_LEVEL_PRESSURE_HPA,
        pressure_to_altitude_m,
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

        let i2c = hal::i2c::I2C::i2c1(
            pac.I2C1,
            pins.gpio18.reconfigure(),
            pins.gpio19.reconfigure(),
            400u32.kHz(),
            &mut pac.RESETS,
            clocks.system_clock.freq(),
        );

        let mut barometer = Lps25hb::new_i2c(i2c, Address::Addr5c);
        barometer.init(Config::akizuki_style()).unwrap();

        defmt::info!("RP2350 + LPS25HB example");
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

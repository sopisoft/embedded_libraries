#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

// This example targets an RP2350-class MCU such as Raspberry Pi Pico 2.
//
// It connects to the Akizuki AE-LPS25HB module over I2C and prints pressure,
// temperature, and barometric altitude over UART.
//
// Suggested wiring:
// - GPIO18 -> SDA
// - GPIO19 -> SCL
// - GPIO0  -> UART0 TX for debug prints
// - GPIO1  -> UART0 RX for an optional console
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
    use core::fmt::Write;

    use embedded_hal::delay::DelayNs;
    use lps25hb::{
        Address, Config, DEVICE_ID, Lps25hb, STANDARD_SEA_LEVEL_PRESSURE_HPA,
        pressure_to_altitude_m,
    };
    use panic_halt as _;
    use rp235x_hal as hal;

    use hal::clocks::Clock;
    use hal::fugit::RateExtU32;
    use hal::uart::{DataBits, StopBits, UartConfig};

    #[unsafe(link_section = ".start_block")]
    #[used]
    pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

    const XTAL_FREQ_HZ: u32 = 12_000_000;
    const SEA_LEVEL_PRESSURE_HPA: f32 = STANDARD_SEA_LEVEL_PRESSURE_HPA;

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

        let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
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

        writeln!(uart, "\r\nRP2350 + LPS25HB example\r").ok();
        writeln!(
            uart,
            "WHO_AM_I = 0x{:02X} (expected 0x{:02X})\r",
            barometer.who_am_i().unwrap(),
            DEVICE_ID
        )
        .ok();

        loop {
            let measurement = barometer.read_measurement().unwrap();
            let altitude_m =
                pressure_to_altitude_m(measurement.pressure_hpa, SEA_LEVEL_PRESSURE_HPA);
            writeln!(
                uart,
                "pressure={:>7.2} hPa  temp={:>6.2} C  altitude={:>7.2} m\r",
                measurement.pressure_hpa, measurement.temperature_c, altitude_m,
            )
            .ok();
            timer.delay_ms(250);
        }
    }
}

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn main() {
    println!(
        "This example is for RP2350 hardware. Build it with \
         `cargo build -p lps25hb --example rp235x_i2c_lps25hb --target thumbv8m.main-none-eabihf`."
    );
}

#![no_std]
#![no_main]

// Wiring idea:
// - GPIO0  -> TSD10 RX
// - GPIO1  <- TSD10 TX
// - 5V     -> TSD10 VCC
// - GND    -> TSD10 GND
// The TSD10 itself requires 4.5 V to 5.5 V power. Confirm the UART IO voltage compatibility of your exact board before wiring it directly to a 3.3 V MCU.

mod embedded_example {
    use core::sync::atomic::{AtomicU32, Ordering};

    use defmt_rtt as _;
    use panic_probe as _;
    use rp235x_hal as hal;
    use tsd10::Tsd10;

    use hal::clocks::Clock;
    use hal::fugit::RateExtU32;
    use hal::uart::{DataBits, StopBits, UartConfig};

    #[unsafe(link_section = ".start_block")]
    #[used]
    pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

    static DEFMT_TIMESTAMP: AtomicU32 = AtomicU32::new(0);
    defmt::timestamp!("{=u32}", DEFMT_TIMESTAMP.fetch_add(1, Ordering::Relaxed));

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

        let lidar_pins = (pins.gpio0.into_function(), pins.gpio1.into_function());
        let lidar_uart = hal::uart::UartPeripheral::new(pac.UART0, lidar_pins, &mut pac.RESETS)
            .enable(
                UartConfig::new(460800u32.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        let mut lidar = Tsd10::new(lidar_uart);

        defmt::info!("RP2350 + TSD10 UART example");

        loop {
            match lidar.read_measurement() {
                Ok(measurement) if measurement.is_out_of_range() => {
                    defmt::warn!("distance=out_of_range");
                }
                Ok(measurement) => {
                    defmt::info!(
                        "distance={:?} mm range={:?} m",
                        measurement.distance_mm,
                        measurement.distance_m().unwrap()
                    );
                }
                Err(_) => {
                    defmt::error!("UART / parser error");
                }
            }
        }
    }
}

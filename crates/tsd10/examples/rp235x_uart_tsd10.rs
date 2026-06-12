#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

// This example targets an RP2350-class MCU such as Raspberry Pi Pico 2.
// It reads the Akizuki TSD10 LiDAR over UART0 and prints the decoded distance
// over UART1.
//
// Wiring idea:
// - GPIO0  -> TSD10 RX
// - GPIO1  <- TSD10 TX
// - 5V     -> TSD10 VCC
// - GND    -> TSD10 GND
// - GPIO4  -> UART1 TX for debug prints
// - GPIO5  -> UART1 RX for an optional console
//
// The TSD10 itself requires 4.5 V to 5.5 V power. Confirm the UART IO voltage
// compatibility of your exact board before wiring it directly to a 3.3 V MCU.

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod embedded_example {
    use core::fmt::Write;

    use panic_halt as _;
    use rp235x_hal as hal;
    use tsd10::Tsd10;

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

        let lidar_pins = (pins.gpio0.into_function(), pins.gpio1.into_function());
        let lidar_uart = hal::uart::UartPeripheral::new(pac.UART0, lidar_pins, &mut pac.RESETS)
            .enable(
                UartConfig::new(460800u32.Hz(), DataBits::Eight, None, StopBits::One),
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

        let mut lidar = Tsd10::new(lidar_uart);

        writeln!(debug_uart, "\r\nRP2350 + TSD10 UART example\r").ok();

        loop {
            match lidar.read_measurement() {
                Ok(measurement) if measurement.is_out_of_range() => {
                    writeln!(debug_uart, "distance=out_of_range\r").ok();
                }
                Ok(measurement) => {
                    writeln!(
                        debug_uart,
                        "distance={:>5} mm  range={:>4.2} m\r",
                        measurement.distance_mm,
                        measurement.distance_m().unwrap()
                    )
                    .ok();
                }
                Err(_) => {
                    writeln!(debug_uart, "UART / parser error\r").ok();
                }
            }
        }
    }
}

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn main() {
    println!(
        "This example is for RP2350 hardware. Build it with \
         `cargo build -p tsd10 --example rp235x_uart_tsd10 --target thumbv8m.main-none-eabihf`."
    );
}

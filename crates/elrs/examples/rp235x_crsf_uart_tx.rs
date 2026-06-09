#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

// This example targets an RP2350-class MCU such as Raspberry Pi Pico 2.
// It shows the simplest useful ELRS / CRSF task on a microcontroller:
// generating RC channel frames and sending them over UART.
//
// Wiring idea:
// - UART0 TX (GPIO0) -> CRSF RX of the connected device
// - UART0 RX (GPIO1) -> optional, only needed if you later add telemetry input
// - GND            -> shared ground
//
// The example sends one CRSF RC frame every 4 ms, which matches a 250 Hz
// update rate often used by ELRS receivers.

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod embedded_example {
    use embedded_hal::delay::DelayNs;
    use panic_halt as _;
    use rp235x_hal as hal;

    use elrs::{DeviceAddress, RcChannels};
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

        let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
        let sio = hal::Sio::new(pac.SIO);
        let pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let uart_pins = (pins.gpio0.into_function(), pins.gpio1.into_function());
        let uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
            .enable(
                // CRSF uses 420000 baud, 8 data bits, no parity, 1 stop bit.
                UartConfig::new(420000u32.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        let mut aileron = 1000u16;
        let mut direction = 1i16;

        loop {
            // Channel mapping example:
            // CH1 aileron, CH2 elevator, CH3 throttle, CH4 rudder, CH5+ auxiliary.
            let channels = RcChannels::from_micros([
                aileron, 1500, 1300, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                1000, 1000, 1000,
            ]);

            let frame = channels
                .encode_frame(DeviceAddress::FLIGHT_CONTROLLER)
                .unwrap();
            let bytes = frame.to_bytes().unwrap();
            uart.write_full_blocking(bytes.as_slice());

            // Sweep CH1 slowly so you can observe changing values on the receiver
            // or on a flight controller that displays CRSF input channels.
            let next = aileron as i16 + 25 * direction;
            if next >= 2000 {
                aileron = 2000;
                direction = -1;
            } else if next <= 1000 {
                aileron = 1000;
                direction = 1;
            } else {
                aileron = next as u16;
            }

            timer.delay_ms(4);
        }
    }
}

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn main() {
    println!(
        "This example is for RP2350 hardware. Build it with \
         `cargo build -p elrs --example rp235x_crsf_uart_tx --target thumbv8m.main-none-eabihf`."
    );
}

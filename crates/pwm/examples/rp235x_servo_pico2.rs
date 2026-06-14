#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

// This example is meant for an RP2350-class target such as Raspberry Pi Pico 2.
//
// On a host machine it builds as a small placeholder binary so the workspace can
// still be checked and tested normally.

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod embedded_example {
    use panic_halt as _;
    use pwm::Servo;
    use rp235x_hal as hal;

    /// Tell the RP2350 boot ROM how to start this program.
    #[unsafe(link_section = ".start_block")]
    #[used]
    pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

    const XTAL_FREQ_HZ: u32 = 12_000_000;

    fn wait_until(timer: &hal::Timer<hal::timer::CopyableTimer0>, deadline: hal::timer::Instant) {
        while timer.get_counter() < deadline {
            core::hint::spin_loop();
        }
    }

    #[hal::entry]
    fn main() -> ! {
        // The structure below follows the official rp-hal examples:
        // - take PAC peripherals,
        // - configure watchdog and clocks,
        // - create the GPIO bank,
        // - configure one PWM slice and route one channel to a GPIO pin.
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

        let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

        // We use PWM slice 0 channel A and route it to GPIO0.
        // GPIO0 is normally easier to access on a Pico-style board than GPIO24/25.
        let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
        let mut slice = pwm_slices.pwm0;

        // Make the PWM counter run at 1 MHz:
        //   125 MHz system clock / 125 = 1 MHz
        // Then set TOP to 20_000 so the PWM period is about 20 ms (50 Hz).
        slice.set_div_int(125);
        slice.set_top(20_000);
        slice.enable();

        let mut channel = slice.channel_a;
        let _servo_pin = channel.output_to(pins.gpio0);

        let mut servo = Servo::new(
            channel,
            fugit::MicrosDurationU32::from_micros(20_000),
            fugit::MicrosDurationU32::from_micros(1_000),
            fugit::MicrosDurationU32::from_micros(2_000),
            -90.0,
            90.0,
        );
        let period = hal::fugit::MicrosDurationU32::from_ticks(800_000);
        let mut next_tick = timer.get_counter() + period;

        loop {
            // Sweep the servo slowly so it is easy to observe with a hobby servo.
            servo.set_angle_degrees(-45.0).unwrap();
            wait_until(&timer, next_tick);
            next_tick += period;

            servo.set_angle_degrees(0.0).unwrap();
            wait_until(&timer, next_tick);
            next_tick += period;

            servo.set_angle_degrees(45.0).unwrap();
            wait_until(&timer, next_tick);
            next_tick += period;
        }
    }
}

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn main() {
    println!(
        "This example is for RP2350 hardware. Build it with \
         `cargo build -p pwm --example rp235x_servo_pico2 --target thumbv8m.main-none-eabihf`."
    );
}

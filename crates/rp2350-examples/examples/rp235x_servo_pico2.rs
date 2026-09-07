#![no_std]
#![no_main]

mod embedded_example {
    use panic_halt as _;
    use pwm::Servo;
    use rp235x_hal as hal;

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

        let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
        let mut slice = pwm_slices.pwm0;

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

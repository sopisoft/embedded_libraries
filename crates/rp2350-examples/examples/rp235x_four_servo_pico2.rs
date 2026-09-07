#![no_std]
#![no_main]

// Pin usage:
// - GPIO2 -> PWM1 A -> left aileron
// - GPIO3 -> PWM1 B -> right aileron
// - GPIO4 -> PWM2 A -> elevator
// - GPIO5 -> PWM2 B -> rudder

mod embedded_example {
    use core::convert::Infallible;

    use control::SignedNormalized;
    use panic_halt as _;
    use pwm::{Servo, ServoBank, ServoOutput, ServoRange, ServoSet};
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

        let mut slice1 = pwm_slices.pwm1;
        slice1.set_div_int(125);
        slice1.set_top(20_000);
        slice1.enable();

        let mut slice2 = pwm_slices.pwm2;
        slice2.set_div_int(125);
        slice2.set_top(20_000);
        slice2.enable();

        let mut left_aileron_pwm = slice1.channel_a;
        let _left_aileron_pin = left_aileron_pwm.output_to(pins.gpio2);

        let mut right_aileron_pwm = slice1.channel_b;
        let _right_aileron_pin = right_aileron_pwm.output_to(pins.gpio3);

        let mut elevator_pwm = slice2.channel_a;
        let _elevator_pin = elevator_pwm.output_to(pins.gpio4);

        let mut rudder_pwm = slice2.channel_b;
        let _rudder_pin = rudder_pwm.output_to(pins.gpio5);

        let base = ServoRange::new(
            fugit::MicrosDurationU32::from_micros(20_000),
            fugit::MicrosDurationU32::from_micros(1_000),
            fugit::MicrosDurationU32::from_micros(2_000),
            -60.0,
            60.0,
        );
        let ranges = ServoSet::new([base, base, base, base]);

        let mut left_aileron = Servo::from_range(left_aileron_pwm, ranges.get(0).unwrap());
        let mut right_aileron = Servo::from_range(right_aileron_pwm, ranges.get(1).unwrap());
        let mut elevator = Servo::from_range(elevator_pwm, ranges.get(2).unwrap());
        let mut rudder = Servo::from_range(rudder_pwm, ranges.get(3).unwrap());
        let period = hal::fugit::MicrosDurationU32::from_ticks(1_000_000);
        let mut next_tick = timer.get_counter() + period;

        loop {
            {
                let pulses = ranges.pulse_widths_from_symmetric([
                    SignedNormalized::ZERO,
                    SignedNormalized::ZERO,
                    SignedNormalized::ZERO,
                    SignedNormalized::ZERO,
                ]);
                let mut bank = ServoBank::new([
                    &mut left_aileron as &mut dyn ServoOutput<Error = Infallible>,
                    &mut right_aileron as &mut dyn ServoOutput<Error = Infallible>,
                    &mut elevator as &mut dyn ServoOutput<Error = Infallible>,
                    &mut rudder as &mut dyn ServoOutput<Error = Infallible>,
                ]);
                bank.set_pulse_widths(pulses).unwrap();
            }
            wait_until(&timer, next_tick);
            next_tick += period;

            {
                let pulses = ranges.pulse_widths_from_symmetric([
                    SignedNormalized::saturated(0.5),
                    SignedNormalized::saturated(-0.5),
                    SignedNormalized::saturated(0.15),
                    SignedNormalized::saturated(0.35),
                ]);
                let mut bank = ServoBank::new([
                    &mut left_aileron as &mut dyn ServoOutput<Error = Infallible>,
                    &mut right_aileron as &mut dyn ServoOutput<Error = Infallible>,
                    &mut elevator as &mut dyn ServoOutput<Error = Infallible>,
                    &mut rudder as &mut dyn ServoOutput<Error = Infallible>,
                ]);
                bank.set_pulse_widths(pulses).unwrap();
            }
            wait_until(&timer, next_tick);
            next_tick += period;

            {
                let pulses = ranges.pulse_widths_from_symmetric([
                    SignedNormalized::saturated(-0.5),
                    SignedNormalized::saturated(0.5),
                    SignedNormalized::saturated(0.15),
                    SignedNormalized::saturated(-0.35),
                ]);
                let mut bank = ServoBank::new([
                    &mut left_aileron as &mut dyn ServoOutput<Error = Infallible>,
                    &mut right_aileron as &mut dyn ServoOutput<Error = Infallible>,
                    &mut elevator as &mut dyn ServoOutput<Error = Infallible>,
                    &mut rudder as &mut dyn ServoOutput<Error = Infallible>,
                ]);
                bank.set_pulse_widths(pulses).unwrap();
            }
            wait_until(&timer, next_tick);
            next_tick += period;
        }
    }
}

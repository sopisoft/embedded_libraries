#![no_std]
#![no_main]

mod embedded_example {
    use core::cell::RefCell;
    use core::convert::Infallible;
    use core::sync::atomic::{AtomicU32, Ordering};

    use defmt_rtt as _;
    use embedded_hal::digital::OutputPin;
    use fc::{Config, FlightController, Gs1502Position, Mode};
    use gs1502::Gs1502;
    use panic_probe as _;
    use pwm::{Servo, ServoBank, ServoOutput};
    use rp235x_hal as hal;
    use rp2350_examples::pio_servo::{PioServo, servo_program};
    use rp2350_examples::sensors::{
        DEFAULT_LIS3MDL_ADDRESS, DEFAULT_LPS25HB_ADDRESS, DEFAULT_LSM6DS3TR_ADDRESS, Sensors,
    };

    use hal::clocks::Clock;
    use hal::fugit::RateExtU32;
    use hal::pio::{PIOBuilder, PIOExt};
    use hal::uart::{DataBits, StopBits, UartConfig};

    const XTAL_FREQ_HZ: u32 = 12_000_000;
    const SAMPLE_PERIOD_MS: u32 = 10;
    const BLUE_TOGGLE_TICKS: u8 = 25;
    const HEAP_SIZE: usize = 4096;

    #[unsafe(link_section = ".start_block")]
    #[used]
    static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

    static DEFMT_TIMESTAMP: AtomicU32 = AtomicU32::new(0);
    defmt::timestamp!("{=u32}", DEFMT_TIMESTAMP.fetch_add(1, Ordering::Relaxed));

    #[global_allocator]
    static HEAP: linked_list_allocator::LockedHeap = linked_list_allocator::LockedHeap::empty();

    #[unsafe(link_section = ".uninit")]
    static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    #[rp235x_hal::entry]
    fn main() -> ! {
        unsafe {
            HEAP.lock()
                .init(core::ptr::addr_of_mut!(HEAP_MEM) as *mut u8, HEAP_SIZE);
        }

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

        let crsf_uart = hal::uart::UartPeripheral::new(
            pac.UART0,
            (pins.gpio12.into_function(), pins.gpio13.into_function()),
            &mut pac.RESETS,
        )
        .enable(
            UartConfig::new(420000u32.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
        let i2c = hal::i2c::I2C::i2c1_with_external_pull_up(
            pac.I2C1,
            pins.gpio18.into_function(),
            pins.gpio19.into_function(),
            400u32.kHz(),
            &mut pac.RESETS,
            clocks.system_clock.freq(),
        );
        let shared_bus = RefCell::new(i2c);
        let mut sensors = Sensors::with_addresses(
            &shared_bus,
            DEFAULT_LSM6DS3TR_ADDRESS,
            DEFAULT_LIS3MDL_ADDRESS,
            DEFAULT_LPS25HB_ADDRESS,
        );
        if let Err(error) = sensors.init() {
            defmt::panic!(
                "sensor initialization failed: {:?}",
                defmt::Debug2Format(&error)
            );
        }

        let _left_aileron_pin = pins.gpio2.into_function::<hal::gpio::FunctionPio0>();
        let _right_aileron_pin = pins.gpio3.into_function::<hal::gpio::FunctionPio0>();
        let _elevator_pin = pins.gpio4.into_function::<hal::gpio::FunctionPio0>();
        let _rudder_pin = pins.gpio5.into_function::<hal::gpio::FunctionPio0>();
        let _linear_pin = pins.gpio1.into_function::<hal::gpio::FunctionPio1>();
        let _throttle_pin = pins.gpio0.into_function::<hal::gpio::FunctionPio1>();

        let program = servo_program();
        let (mut pio0, sm0, sm1, sm2, sm3) = pac.PIO0.split(&mut pac.RESETS);
        let (mut pio1, sm0_1, sm1_1, _, _) = pac.PIO1.split(&mut pac.RESETS);
        let program0 = pio0.install(&program).unwrap();
        let program0_1 = unsafe { program0.share() };
        let program0_2 = unsafe { program0.share() };
        let program0_3 = unsafe { program0.share() };
        let program1 = pio1.install(&program).unwrap();
        let program1_1 = unsafe { program1.share() };
        let (left_sm, _, left_tx) = PIOBuilder::from_installed_program(program0)
            .set_pins(2, 1)
            .clock_divisor_fixed_point(125, 0)
            .build(sm0);
        let (right_sm, _, right_tx) = PIOBuilder::from_installed_program(program0_1)
            .set_pins(3, 1)
            .clock_divisor_fixed_point(125, 0)
            .build(sm1);
        let (elevator_sm, _, elevator_tx) = PIOBuilder::from_installed_program(program0_2)
            .set_pins(4, 1)
            .clock_divisor_fixed_point(125, 0)
            .build(sm2);
        let (rudder_sm, _, rudder_tx) = PIOBuilder::from_installed_program(program0_3)
            .set_pins(5, 1)
            .clock_divisor_fixed_point(125, 0)
            .build(sm3);
        let (linear_sm, _, linear_tx) = PIOBuilder::from_installed_program(program1)
            .set_pins(1, 1)
            .clock_divisor_fixed_point(125, 0)
            .build(sm0_1);
        let (throttle_sm, _, throttle_tx) = PIOBuilder::from_installed_program(program1_1)
            .set_pins(0, 1)
            .clock_divisor_fixed_point(125, 0)
            .build(sm1_1);

        let ranges = fc::default_servos();
        let mut left_aileron = Servo::from_range(
            PioServo::new(left_sm.start(), left_tx),
            ranges.get(0).unwrap(),
        );
        let mut right_aileron = Servo::from_range(
            PioServo::new(right_sm.start(), right_tx),
            ranges.get(1).unwrap(),
        );
        let mut elevator = Servo::from_range(
            PioServo::new(elevator_sm.start(), elevator_tx),
            ranges.get(2).unwrap(),
        );
        let mut rudder = Servo::from_range(
            PioServo::new(rudder_sm.start(), rudder_tx),
            ranges.get(3).unwrap(),
        );
        let mut throttle = Servo::from_range(
            PioServo::new(throttle_sm.start(), throttle_tx),
            ranges.get(4).unwrap(),
        );
        let mut linear_servo = Gs1502::from_range(
            PioServo::new(linear_sm.start(), linear_tx),
            gs1502::DEFAULT_RANGE,
        );
        let mut controller = FlightController::new(Config::default());
        let mut red_led = pins.gpio6.into_push_pull_output();
        let mut blue_led = pins.gpio7.into_push_pull_output();

        let mut byte = [0u8; 1];
        let dt = fugit::MicrosDurationU32::from_millis(SAMPLE_PERIOD_MS);
        let tick_period = hal::fugit::MicrosDurationU32::from_ticks(SAMPLE_PERIOD_MS * 1_000);
        let mut next_tick = timer.get_counter() + tick_period;
        let mut now_us = 0u32;
        let mut blue_on = true;
        let mut blue_ticks = 0;
        let mut update_servos = true;

        loop {
            while crsf_uart.uart_is_readable() {
                if crsf_uart.read_full_blocking(&mut byte).is_ok() {
                    controller.push_crsf_byte(byte[0], now_us);
                }
            }
            let sample = match sensors.read_marg() {
                Ok(sample) => sample,
                Err(error) => {
                    defmt::panic!("sensor read failed: {:?}", defmt::Debug2Format(&error));
                }
            };
            let output = controller.update(sample, now_us, dt);
            let red =
                output.mode != Mode::Failsafe && output.gs1502_position == Gs1502Position::Extended;
            let _ = red_led.set_state(red.into());
            let _ = blue_led.set_state((!red && blue_on).into());
            if update_servos {
                let mut bank = ServoBank::new([
                    &mut left_aileron as &mut dyn ServoOutput<Error = Infallible>,
                    &mut right_aileron as &mut dyn ServoOutput<Error = Infallible>,
                    &mut elevator as &mut dyn ServoOutput<Error = Infallible>,
                    &mut rudder as &mut dyn ServoOutput<Error = Infallible>,
                    &mut throttle as &mut dyn ServoOutput<Error = Infallible>,
                ]);
                bank.set_pulse_widths(output.control.pulses).unwrap();
                linear_servo
                    .set_position(output.gs1502_position.normalized())
                    .unwrap();
            }
            update_servos = !update_servos;

            blue_ticks += 1;
            if blue_ticks == BLUE_TOGGLE_TICKS {
                blue_ticks = 0;
                blue_on = !blue_on;
            }
            wait_until(&timer, next_tick);
            next_tick += tick_period;
            now_us = now_us.wrapping_add(SAMPLE_PERIOD_MS * 1_000);
        }
    }

    fn wait_until(timer: &hal::Timer<hal::timer::CopyableTimer0>, deadline: hal::timer::Instant) {
        while timer.get_counter() < deadline {
            core::hint::spin_loop();
        }
    }
}

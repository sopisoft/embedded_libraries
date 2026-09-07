#![no_std]
#![no_main]

mod embedded_example {
    use core::cell::RefCell;
    use core::convert::Infallible;

    use defmt_rtt as _;
    use embedded_hal::{i2c::I2c, pwm::SetDutyCycle};
    use fc::{Config, FlightController, Gs1502Position, Mode};
    use gs1502::Gs1502;
    use panic_probe as _;
    use pwm::{Servo, ServoBank, ServoOutput};
    use rp235x_hal as hal;
    use rp2350_examples::sensors::{
        DEFAULT_LIS3MDL_ADDRESS, DEFAULT_LPS25HB_ADDRESS, DEFAULT_LSM6DS3TR_ADDRESS, Sensors,
    };

    use hal::clocks::Clock;
    use hal::dma::DMAExt;
    use hal::fugit::RateExtU32;
    use hal::timer::Alarm;
    use hal::uart::{DataBits, StopBits, UartConfig};

    const XTAL_FREQ_HZ: u32 = 12_000_000;
    const SAMPLE_PERIOD_MS: u32 = 5;
    const STARTUP_LIGHT_TICKS: u16 = 300;
    const BLUE_TOGGLE_TICKS: u8 = 25;
    const ESC_ARM_TIME_MS: u32 = 3_000;
    const NEUTRAL_PULSE_US: u32 = 1_500;
    const SERVO_PWM_DIVIDER: u8 = 150;
    const SERVO_PWM_TOP: u16 = 20_000;
    const HEAP_SIZE: usize = 4096;

    #[unsafe(link_section = ".start_block")]
    #[used]
    static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

    #[global_allocator]
    static HEAP: linked_list_allocator::LockedHeap = linked_list_allocator::LockedHeap::empty();

    #[unsafe(link_section = ".uninit")]
    static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    use hal::pac::interrupt;

    #[interrupt]
    fn TIMER0_IRQ_0() {
        unsafe {
            (*hal::pac::TIMER0::ptr())
                .intr()
                .write_with_zero(|w| w.alarm_0().clear_bit_by_one());
        }
    }

    struct DmaRxBuffer<'a>(&'a mut [u8; 64]);

    struct TimedAltitude {
        sample: Option<(f32, u32)>,
    }

    impl TimedAltitude {
        const fn new() -> Self {
            Self { sample: None }
        }

        fn update(&mut self, altitude_m: f32, now_us: u32) {
            self.sample = Some((altitude_m, now_us));
        }

        fn fresh(&self, now_us: u32) -> Option<f32> {
            self.sample
                .filter(|(_, updated_us)| now_us.wrapping_sub(*updated_us) <= 250_000)
                .map(|(altitude_m, _)| altitude_m)
        }
    }

    unsafe impl hal::dma::WriteTarget for DmaRxBuffer<'_> {
        type TransmittedWord = u8;

        fn tx_treq() -> Option<u8> {
            None
        }

        fn tx_address_count(&mut self) -> (u32, u32) {
            (self.0.as_mut_ptr() as u32, self.0.len() as u32)
        }

        fn tx_increment(&self) -> bool {
            true
        }
    }

    impl hal::dma::EndlessWriteTarget for DmaRxBuffer<'_> {}

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
        let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
        let mut timer_alarm = timer.alarm_0().unwrap();
        timer_alarm.enable_interrupt();
        unsafe {
            hal::arch::interrupt_unmask(hal::pac::Interrupt::TIMER0_IRQ_0);
        }
        let sio = hal::Sio::new(pac.SIO);
        let pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
        let mut led_pwm = pwm_slices.pwm3;
        led_pwm.set_div_int(125);
        led_pwm.set_top(1_000);
        led_pwm.enable();
        let mut red_led = led_pwm.channel_a;
        let _red_led_pin = red_led.output_to(pins.gpio6);
        let mut blue_led = led_pwm.channel_b;
        let _blue_led_pin = blue_led.output_to(pins.gpio7);
        let _ = red_led.set_duty_cycle(600);
        let _ = blue_led.set_duty_cycle(600);

        let mut pwm0 = pwm_slices.pwm0;
        let mut pwm1 = pwm_slices.pwm1;
        let mut pwm2 = pwm_slices.pwm2;
        pwm0.set_div_int(SERVO_PWM_DIVIDER);
        pwm0.set_top(SERVO_PWM_TOP);
        pwm0.enable();
        pwm1.set_div_int(SERVO_PWM_DIVIDER);
        pwm1.set_top(SERVO_PWM_TOP);
        pwm1.enable();
        pwm2.set_div_int(SERVO_PWM_DIVIDER);
        pwm2.set_top(SERVO_PWM_TOP);
        pwm2.enable();

        let mut throttle_pwm = pwm0.channel_a;
        let _throttle_pin = throttle_pwm.output_to(pins.gpio0);
        let mut linear_pwm = pwm0.channel_b;
        let _linear_pin = linear_pwm.output_to(pins.gpio1);
        let mut left_aileron_pwm = pwm1.channel_a;
        let _left_aileron_pin = left_aileron_pwm.output_to(pins.gpio2);
        let mut right_aileron_pwm = pwm1.channel_b;
        let _right_aileron_pin = right_aileron_pwm.output_to(pins.gpio3);
        let mut elevator_pwm = pwm2.channel_a;
        let _elevator_pin = elevator_pwm.output_to(pins.gpio4);
        let mut rudder_pwm = pwm2.channel_b;
        let _rudder_pin = rudder_pwm.output_to(pins.gpio5);
        let neutral_duty = NEUTRAL_PULSE_US as u16;
        let _ = throttle_pwm.set_duty_cycle(neutral_duty);
        let _ = linear_pwm.set_duty_cycle(neutral_duty);
        let _ = left_aileron_pwm.set_duty_cycle(neutral_duty);
        let _ = right_aileron_pwm.set_duty_cycle(neutral_duty);
        let _ = elevator_pwm.set_duty_cycle(neutral_duty);
        let _ = rudder_pwm.set_duty_cycle(neutral_duty);
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
        let (crsf_reader, _crsf_writer) = crsf_uart.split();
        let imu_i2c = hal::i2c::I2C::i2c1_with_external_pull_up(
            pac.I2C1,
            pins.gpio18.into_function(),
            pins.gpio19.into_function(),
            400u32.kHz(),
            &mut pac.RESETS,
            clocks.system_clock.freq(),
        );
        let mut baro_i2c = hal::i2c::I2C::i2c0_with_external_pull_up(
            pac.I2C0,
            pins.gpio8.into_function(),
            pins.gpio9.into_function(),
            100u32.kHz(),
            &mut pac.RESETS,
            clocks.system_clock.freq(),
        );
        let lps25hb_address = detect_lps25hb_address(&mut baro_i2c);
        let imu_bus = RefCell::new(imu_i2c);
        let baro_bus = RefCell::new(baro_i2c);
        let mut sensors = Sensors::with_addresses(
            &imu_bus,
            &baro_bus,
            DEFAULT_LSM6DS3TR_ADDRESS,
            DEFAULT_LIS3MDL_ADDRESS,
            lps25hb_address.unwrap_or(DEFAULT_LPS25HB_ADDRESS),
        );
        if let Err(error) = sensors.init() {
            defmt::panic!(
                "sensor initialization failed: {:?}",
                defmt::Debug2Format(&error)
            );
        }
        if !sensors.barometer_available() {
            defmt::warn!("barometer unavailable; altitude hold disabled");
        }
        let dma = pac.DMA.split(&mut pac.RESETS);
        let mut crsf_rx_a = [0u8; 64];
        let mut crsf_rx_b = [0u8; 64];
        let mut crsf_dma = hal::dma::double_buffer::Config::new(
            (dma.ch0, dma.ch1),
            crsf_reader,
            DmaRxBuffer(&mut crsf_rx_a),
        )
        .start()
        .write_next(DmaRxBuffer(&mut crsf_rx_b));

        let ranges = fc::default_servos();
        let mut left_aileron = Servo::from_range(left_aileron_pwm, ranges.get(0).unwrap());
        let mut right_aileron = Servo::from_range(right_aileron_pwm, ranges.get(1).unwrap());
        let mut elevator = Servo::from_range(elevator_pwm, ranges.get(2).unwrap());
        let mut rudder = Servo::from_range(rudder_pwm, ranges.get(3).unwrap());
        let mut throttle = Servo::from_range(throttle_pwm, ranges.get(4).unwrap());
        let mut linear_servo = Gs1502::from_range(linear_pwm, gs1502::DEFAULT_RANGE);
        let mut controller = FlightController::new(Config::default());
        let mut startup_ticks = 0u16;
        let mut blue_on = true;
        let mut blue_ticks = 0u8;

        let timer_arm_period = hal::fugit::MicrosDurationU32::millis(SAMPLE_PERIOD_MS);
        let neutral_pulse = fugit::MicrosDurationU32::from_micros(NEUTRAL_PULSE_US);
        set_neutral_servos(
            [
                &mut left_aileron,
                &mut right_aileron,
                &mut elevator,
                &mut rudder,
                &mut throttle,
            ],
            neutral_pulse,
        );
        linear_servo
            .set_position(Gs1502Position::Retracted.normalized())
            .unwrap();
        let arm_deadline =
            timer.get_counter() + hal::fugit::MicrosDurationU32::millis(ESC_ARM_TIME_MS);
        let mut next_arm_tick = timer.get_counter();
        while timer.get_counter() < arm_deadline {
            let now_us = timer.get_counter().ticks() as u32;
            if crsf_dma.is_done() {
                let (completed, next) = crsf_dma.wait();
                push_crsf_bytes(&mut controller, completed.0, now_us);
                crsf_dma = next.write_next(completed);
            }
            next_arm_tick += timer_arm_period;
            wait_until(&timer, &mut timer_alarm, next_arm_tick);
        }

        let tick_period = hal::fugit::MicrosDurationU32::from_ticks(SAMPLE_PERIOD_MS * 1_000);
        let mut next_tick = timer.get_counter() + tick_period;
        let mut previous_update_us =
            (timer.get_counter().ticks() as u32).wrapping_sub(SAMPLE_PERIOD_MS * 1_000);
        let mut barometric_altitude = TimedAltitude::new();
        let mut barometer_warned = false;

        loop {
            let now_us = timer.get_counter().ticks() as u32;
            let dt = fugit::MicrosDurationU32::from_micros(now_us.wrapping_sub(previous_update_us));
            previous_update_us = now_us;
            if crsf_dma.is_done() {
                let (completed, next) = crsf_dma.wait();
                push_crsf_bytes(&mut controller, completed.0, now_us);
                crsf_dma = next.write_next(completed);
            }
            match sensors.barometric_altitude() {
                Ok(Some(altitude_m)) => {
                    barometric_altitude.update(altitude_m, now_us);
                    barometer_warned = false;
                }
                Ok(None) => {}
                Err(error) => {
                    if !barometer_warned {
                        defmt::warn!("barometer read failed: {:?}", defmt::Debug2Format(&error));
                        barometer_warned = true;
                    }
                }
            }
            let fresh_barometric_altitude_m = barometric_altitude.fresh(now_us);
            let sample = match sensors.read_marg() {
                Ok(sample) => sample,
                Err(error) => {
                    set_neutral_servos(
                        [
                            &mut left_aileron,
                            &mut right_aileron,
                            &mut elevator,
                            &mut rudder,
                            &mut throttle,
                        ],
                        neutral_pulse,
                    );
                    linear_servo
                        .set_position(Gs1502Position::Retracted.normalized())
                        .unwrap();
                    defmt::panic!("sensor read failed: {:?}", defmt::Debug2Format(&error));
                }
            };
            let output =
                controller.update_with_altitude(sample, fresh_barometric_altitude_m, now_us, dt);
            if startup_ticks < STARTUP_LIGHT_TICKS {
                let _ = red_led.set_duty_cycle(600);
                let _ = blue_led.set_duty_cycle(600);
            } else {
                let red = output.mode != Mode::Failsafe
                    && output.gs1502_position == Gs1502Position::Extended;
                let blue = !red && blue_on;
                let _ = red_led.set_duty_cycle(if red { 600 } else { 0 });
                let _ = blue_led.set_duty_cycle(if blue { 600 } else { 0 });
            }
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
            startup_ticks = startup_ticks.saturating_add(1);
            blue_ticks += 1;
            if blue_ticks == BLUE_TOGGLE_TICKS {
                blue_ticks = 0;
                blue_on = !blue_on;
            }
            wait_until(&timer, &mut timer_alarm, next_tick);
            next_tick = timer.get_counter() + tick_period;
        }
    }

    fn push_crsf_bytes(controller: &mut FlightController, bytes: &[u8], now_us: u32) {
        for &byte in bytes {
            controller.push_crsf_byte(byte, now_us);
        }
    }

    fn set_neutral_servos(
        servos: [&mut dyn ServoOutput<Error = Infallible>; 5],
        neutral_pulse: fugit::MicrosDurationU32,
    ) {
        ServoBank::new(servos)
            .set_pulse_widths([neutral_pulse; 5])
            .unwrap();
    }

    fn wait_until(
        timer: &hal::Timer<hal::timer::CopyableTimer0>,
        alarm: &mut impl Alarm,
        deadline: hal::timer::Instant,
    ) {
        if timer.get_counter() >= deadline {
            return;
        }
        alarm.schedule_at(deadline).unwrap();
        while timer.get_counter() < deadline {
            hal::arch::wfi();
        }
        alarm.clear_interrupt();
    }

    fn detect_lps25hb_address<I2C: I2c>(i2c: &mut I2C) -> Option<lps25hb::Address> {
        for address in [lps25hb::Address::Addr5c, lps25hb::Address::Addr5d] {
            let mut who_am_i = [0u8; 1];
            if i2c
                .write_read(address.as_u8(), &[0x0F], &mut who_am_i)
                .is_ok()
                && who_am_i[0] == lps25hb::DEVICE_ID
            {
                return Some(address);
            }
        }
        None
    }
}

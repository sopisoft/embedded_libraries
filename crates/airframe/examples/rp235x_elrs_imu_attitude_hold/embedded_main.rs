mod support;

use core::{cell::RefCell, convert::Infallible, fmt::Write};

use airframe::{RcInputConfig, Vector3, apply_subset_channels};
use elrs::{
    FRAME_TYPE_RC_CHANNELS_PACKED, FRAME_TYPE_SUBSET_RC_CHANNELS_PACKED, FrameParser, RcChannels,
    SubsetRcChannels,
};
use imu::{AccelGyroSample, MargEstimator, MargSample, SharedI2c};
use lis3mdl::{Config as Lis3mdlConfig, Lis3mdl};
use lsm6ds3tr::LSM6DS3TR;
use panic_halt as _;
use pwm::{Servo, ServoBank, ServoOutput};
use rp235x_hal as hal;

use hal::clocks::Clock;
use hal::fugit::RateExtU32;
use hal::uart::{DataBits, StopBits, UartConfig};
use support::{
    GRAVITY_M_S2, LIS3MDL_ADDR, LSM6DS3TR_ADDR, SAMPLE_PERIOD_MS, XTAL_FREQ_HZ, build_controller,
    init_heap, lsm_settings, servo_ranges,
};

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

fn wait_until(timer: &hal::Timer<hal::timer::CopyableTimer0>, deadline: hal::timer::Instant) {
    while timer.get_counter() < deadline {
        core::hint::spin_loop();
    }
}

pub fn run() -> ! {
    init_heap();

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

    let crsf_pins = (pins.gpio12.into_function(), pins.gpio13.into_function());
    let crsf_uart = hal::uart::UartPeripheral::new(pac.UART0, crsf_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(420000u32.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    let debug_pins = (pins.gpio8.into_function(), pins.gpio9.into_function());
    let mut debug_uart = hal::uart::UartPeripheral::new(pac.UART1, debug_pins, &mut pac.RESETS)
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
    let shared_bus = RefCell::new(i2c);
    let mut accel_gyro = LSM6DS3TR::new(support::Lsm6ds3trI2c::new(
        SharedI2c::new(&shared_bus),
        LSM6DS3TR_ADDR,
    ))
    .with_settings(lsm_settings());
    let mut magnetometer = Lis3mdl::new(SharedI2c::new(&shared_bus), LIS3MDL_ADDR);

    accel_gyro.init().unwrap();
    magnetometer.init(Lis3mdlConfig::default()).unwrap();

    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let mut slice0 = pwm_slices.pwm0;
    slice0.set_div_int(125);
    slice0.set_top(20_000);
    slice0.enable();
    let mut slice1 = pwm_slices.pwm1;
    slice1.set_div_int(125);
    slice1.set_top(20_000);
    slice1.enable();
    let mut slice2 = pwm_slices.pwm2;
    slice2.set_div_int(125);
    slice2.set_top(20_000);
    slice2.enable();

    let mut left_aileron_pwm = slice0.channel_a;
    let _left_aileron_pin = left_aileron_pwm.output_to(pins.gpio0);
    let mut right_aileron_pwm = slice0.channel_b;
    let _right_aileron_pin = right_aileron_pwm.output_to(pins.gpio1);
    let mut elevator_pwm = slice1.channel_a;
    let _elevator_pin = elevator_pwm.output_to(pins.gpio2);
    let mut rudder_pwm = slice1.channel_b;
    let _rudder_pin = rudder_pwm.output_to(pins.gpio3);
    let mut throttle_pwm = slice2.channel_a;
    let _throttle_pin = throttle_pwm.output_to(pins.gpio4);

    let servos = servo_ranges();
    let mut left_aileron = Servo::from_range(left_aileron_pwm, servos.range(0));
    let mut right_aileron = Servo::from_range(right_aileron_pwm, servos.range(1));
    let mut elevator = Servo::from_range(elevator_pwm, servos.range(2));
    let mut rudder = Servo::from_range(rudder_pwm, servos.range(3));
    let mut throttle = Servo::from_range(throttle_pwm, servos.range(4));
    let mut controller = build_controller(servos);

    let rc_config = RcInputConfig::conventional_aetr();
    let mut parser = FrameParser::new();
    let mut rc_channels = RcChannels::from_micros([
        1_500, 1_500, 1_000, 1_500, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000, 1_000,
        1_000, 1_000, 1_000,
    ]);
    let mut byte = [0u8; 1];
    let dt = fugit::MicrosDurationU32::from_millis(SAMPLE_PERIOD_MS);
    let mut estimator = MargEstimator::new(0.08);
    let mut report_divider = 0u8;
    let tick_period = hal::fugit::MicrosDurationU32::from_ticks(SAMPLE_PERIOD_MS * 1_000);
    let mut next_tick = timer.get_counter() + tick_period;

    writeln!(debug_uart, "\r\nIntegrated ELRS + IMU + servo example\r").ok();

    loop {
        while crsf_uart.uart_is_readable() {
            if crsf_uart.read_full_blocking(&mut byte).is_err() {
                break;
            }
            if let Some(Ok(frame)) = parser.push(byte[0]) {
                match frame.frame_type {
                    FRAME_TYPE_RC_CHANNELS_PACKED => {
                        let payload: [u8; 22] = frame.payload().try_into().unwrap();
                        rc_channels = RcChannels::unpack(payload);
                    }
                    FRAME_TYPE_SUBSET_RC_CHANNELS_PACKED => {
                        if let Ok(subset) = SubsetRcChannels::decode(frame.payload()) {
                            apply_subset_channels(&mut rc_channels, &subset);
                        }
                    }
                    _ => {}
                }
            }
        }

        let accel_g = accel_gyro.read_accel().unwrap();
        let gyro_dps = accel_gyro.read_gyro().unwrap();
        let mag_mgauss = magnetometer.read_magnetic_mgauss().unwrap();
        let gyro_rad_s = Vector3::new(
            gyro_dps.x.to_radians(),
            gyro_dps.y.to_radians(),
            gyro_dps.z.to_radians(),
        );
        let sample = MargSample::new(
            AccelGyroSample::without_temperature(
                Vector3::new(accel_g.x, accel_g.y, accel_g.z) * GRAVITY_M_S2,
                gyro_rad_s,
            ),
            Vector3::new(
                mag_mgauss.x_mgauss,
                mag_mgauss.y_mgauss,
                mag_mgauss.z_mgauss,
            ),
        );
        let estimate = estimator.update_marg(sample, dt);
        let pilot = rc_config.decode(&rc_channels);
        let output = controller.update_selected(pilot, estimate.euler, gyro_rad_s, dt);

        {
            let mut bank = ServoBank::new([
                &mut left_aileron as &mut dyn ServoOutput<Error = Infallible>,
                &mut right_aileron as &mut dyn ServoOutput<Error = Infallible>,
                &mut elevator as &mut dyn ServoOutput<Error = Infallible>,
                &mut rudder as &mut dyn ServoOutput<Error = Infallible>,
                &mut throttle as &mut dyn ServoOutput<Error = Infallible>,
            ]);
            output.apply_to_servo_bank(&mut bank).unwrap();
        }

        report_divider = report_divider.wrapping_add(1);
        if report_divider >= 20 {
            report_divider = 0;
            let euler_deg = estimate.euler.to_degrees();
            writeln!(
                debug_uart,
                "mode={} roll={:>6.1} pitch={:>6.1} yaw={:>6.1} thr={:.2} ail={:.2}/{:.2} ele={:.2} rud={:.2}\r",
                if pilot.attitude_hold_enabled { "hold" } else { "manual" },
                euler_deg.x,
                euler_deg.y,
                euler_deg.z,
                output.surfaces.throttle,
                output.surfaces.left_aileron,
                output.surfaces.right_aileron,
                output.surfaces.elevator,
                output.surfaces.rudder,
            )
            .ok();
        }

        wait_until(&timer, next_tick);
        next_tick += tick_period;
    }
}

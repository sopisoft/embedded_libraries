#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

// This example connects an RP2350 board to the Adafruit
// LSM6DS3TR-C + LIS3MDL 9-DoF breakout over I2C and performs:
//
// 1. attitude estimation with online gyro-bias calibration and optional
//    magnetometer fusion after hard-iron calibration has enough span,
// 2. relative altitude estimation by integrating gravity-compensated
//    vertical acceleration.
//
// Important limitation:
// - There is no barometer on this board.
// - The "altitude" printed by this example is therefore only a relative,
//   IMU-integrated estimate and will drift over time.
// - Absolute yaw requires moving the sensor through several orientations so
//   the example can estimate a magnetometer hard-iron offset online.
//
// Suggested wiring for a Pico 2 style board:
// - GPIO16 -> STEMMA QT SDA
// - GPIO17 -> STEMMA QT SCL
// - 3V3    -> VIN
// - GND    -> GND
//
// This example assumes the default breakout strap state:
// - LSM6DS3TR-C: 0x6A
// - LIS3MDL:     0x1C

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod stemma_qt_9dof;

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod embedded_example {
    use core::cell::RefCell;
    use core::sync::atomic::{AtomicU32, Ordering};

    use defmt_rtt as _;
    use embedded_hal::digital::OutputPin as _;
    use imu::{
        EskfEstimator, MagnetometerCalibrator, SharedI2c, Vector3,
        stemma_qt_9dof_accel_gyro_sample, stemma_qt_9dof_body_vector,
    };
    use linked_list_allocator::LockedHeap;
    use lis3mdl::{Address as Lis3mdlAddress, Lis3mdl};
    use lsm6ds3tr::{
        AccelSampleRate, AccelScale, AccelSettings, GyroSettings, LSM6DS3TR, LsmSettings,
    };
    use panic_probe as _;
    use rp235x_hal as hal;

    use crate::stemma_qt_9dof::*;
    use hal::clocks::Clock;
    use hal::fugit::RateExtU32;

    #[unsafe(link_section = ".start_block")]
    #[used]
    pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

    static DEFMT_TIMESTAMP: AtomicU32 = AtomicU32::new(0);
    defmt::timestamp!("{=u32}", DEFMT_TIMESTAMP.fetch_add(1, Ordering::Relaxed));

    #[global_allocator]
    static HEAP: LockedHeap = LockedHeap::empty();

    #[unsafe(link_section = ".uninit")]
    static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    #[hal::entry]
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
        defmt::info!("boot");

        let sio = hal::Sio::new(pac.SIO);
        let pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
        let sda_pin = pins.gpio16.into_pull_up_input();
        let scl_pin = pins.gpio17.into_pull_up_input();
        let mut ready_led = pins.gpio25.into_push_pull_output();
        let _ = ready_led.set_low();
        let mut gpio_in = hal::Sio::read_bank0();
        defmt::info!(
            "imu gpio16_sda_high={:?} gpio17_scl_high={:?}",
            ((gpio_in >> 16) & 1) != 0,
            ((gpio_in >> 17) & 1) != 0
        );
        defmt::info!("attempt imu i2c0 bus recovery");
        let mut scl_recovery = scl_pin.into_push_pull_output_in_state(hal::gpio::PinState::High);
        for _ in 0..16 {
            let _ = scl_recovery.set_low();
            for _ in 0..1024 {
                core::hint::spin_loop();
            }
            let _ = scl_recovery.set_high();
            for _ in 0..1024 {
                core::hint::spin_loop();
            }
            gpio_in = hal::Sio::read_bank0();
            if ((gpio_in >> 16) & 1) != 0 {
                break;
            }
        }
        defmt::info!(
            "imu recovery result: gpio16_sda_high={:?} gpio17_scl_high={:?}",
            ((gpio_in >> 16) & 1) != 0,
            ((gpio_in >> 17) & 1) != 0
        );
        let scl_pin = scl_recovery.into_pull_up_input();

        let i2c = hal::i2c::I2C::i2c0(
            pac.I2C0,
            sda_pin.reconfigure(),
            scl_pin.reconfigure(),
            100u32.kHz(),
            &mut pac.RESETS,
            clocks.system_clock.freq(),
        );

        defmt::info!("imu i2c0 ready");
        let shared_bus = RefCell::new(i2c);
        let lsm6ds3tr_addr = LSM6DS3TR_ADDR;
        let detected_lis3mdl_addr = detect_lis3mdl_address(&shared_bus);
        match detected_lis3mdl_addr {
            Some(address) => defmt::info!("detected LIS3MDL address={:#04x}", address.as_u8()),
            None => defmt::warn!("LIS3MDL not found at 0x1C or 0x1E during startup scan"),
        }

        let lsm_settings = LsmSettings::basic()
            .with_accel(
                AccelSettings::new()
                    .with_sample_rate(AccelSampleRate::_104Hz)
                    .with_scale(AccelScale::_2G),
            )
            .with_gyro(GyroSettings::new());

        let mut accel_gyro = LSM6DS3TR::new(Lsm6ds3trI2c::new(
            SharedI2c::new(&shared_bus),
            lsm6ds3tr_addr,
        ))
        .with_settings(lsm_settings);

        let mut lis3mdl_addr = detected_lis3mdl_addr.unwrap_or(Lis3mdlAddress::Addr1c);
        let mut magnetometer = Lis3mdl::new(SharedI2c::new(&shared_bus), lis3mdl_addr);

        defmt::info!("init LSM6DS3TR");
        accel_gyro.init().unwrap();
        defmt::info!("init LIS3MDL");
        let mut magnetometer_enabled = if detected_lis3mdl_addr.is_some() {
            match init_magnetometer(&mut magnetometer) {
                Ok(()) => true,
                Err(error) => {
                    defmt::warn!(
                        "mag init failed at {:#04x}, disabling magnetic yaw fusion: {:?}",
                        lis3mdl_addr.as_u8(),
                        defmt::Debug2Format(&error)
                    );
                    false
                }
            }
        } else {
            false
        };

        defmt::info!("9-DoF estimation example");
        defmt::info!("LSM6DS3TR addr={:?}", lsm6ds3tr_addr);
        defmt::info!(
            "LIS3MDL addr={:?} enabled={:?}",
            lis3mdl_addr.as_u8(),
            magnetometer_enabled
        );
        match accel_gyro.is_reachable() {
            Ok(reachable) => defmt::info!("LSM6DS3TR reachable={:?}", reachable),
            Err(error) => defmt::warn!(
                "LSM6DS3TR reachability check failed after init: {:?}",
                defmt::Debug2Format(&error)
            ),
        }
        if magnetometer_enabled {
            match magnetometer.who_am_i() {
                Ok(device_id) => defmt::info!("LIS3MDL WHO_AM_I={:?}", device_id),
                Err(error) => {
                    magnetometer_enabled = false;
                    defmt::warn!(
                        "mag who_am_i failed after init, disabling magnetic fusion: {:?}",
                        defmt::Debug2Format(&error)
                    );
                }
            }
        }
        defmt::info!(
            "state log format: t_ms, quaternion, euler_deg, altitude_m"
        );
        defmt::info!(
            "mag calibration: move the sensor through several orientations to enable absolute yaw"
        );
        log_filter_configuration();

        let period = hal::fugit::MicrosDurationU32::from_ticks(SAMPLE_PERIOD_MS * 1_000);
        let (gyro_bias, accel_bias) = calibrate_imu_biases(&mut accel_gyro, &timer, period);

        let mut estimator = EskfEstimator::new()
            .with_stationary_detection(FILTER_STATIONARY_DETECTION)
            .with_tuning(FILTER_ESKF_TUNING);
        estimator.set_gyro_bias(gyro_bias);
        estimator.set_accel_bias(accel_bias);
        let _ = ready_led.set_high();

        let mut mag_calibration =
            MagnetometerCalibrator::new(FILTER_MAG_CALIBRATION_MIN_SPAN_MGAUSS);
        let mut magnetometer_enabled = magnetometer_enabled;
        let mut mag_ready_logged = false;
        let mut last_mag_retry_sample = 0u32;
        let mut sample_count = 0u32;
        let mut last_sample_tick = timer.get_counter();
        let mut next_tick = timer.get_counter() + period;

        loop {
            let now = timer.get_counter();
            let dt = fugit::MicrosDurationU32::from_ticks(
                now.ticks().wrapping_sub(last_sample_tick.ticks()) as u32,
            );
            last_sample_tick = now;
            sample_count = sample_count.wrapping_add(1);

            if !magnetometer_enabled
                && sample_count.wrapping_sub(last_mag_retry_sample) >= MAG_RETRY_PERIOD_SAMPLES
            {
                last_mag_retry_sample = sample_count;
                if let Some(detected_addr) = detect_lis3mdl_address(&shared_bus) {
                    if detected_addr != lis3mdl_addr {
                        lis3mdl_addr = detected_addr;
                        magnetometer = Lis3mdl::new(SharedI2c::new(&shared_bus), lis3mdl_addr);
                    }
                    match init_magnetometer(&mut magnetometer) {
                        Ok(()) => {
                            magnetometer_enabled = true;
                            defmt::info!(
                                "mag re-enabled at {:#04x}; absolute yaw will resume after calibration if available",
                                lis3mdl_addr.as_u8()
                            );
                            match magnetometer.who_am_i() {
                                Ok(device_id) => defmt::info!("LIS3MDL WHO_AM_I={:?}", device_id),
                                Err(error) => defmt::warn!(
                                    "mag who_am_i failed after re-enable: {:?}",
                                    defmt::Debug2Format(&error)
                                ),
                            }
                            if mag_calibration.is_ready() {
                                let offset = mag_calibration.offset_mgauss();
                                defmt::info!(
                                    "mag calibration ready, offset [mgauss]=({:?}, {:?}, {:?})",
                                    offset.x,
                                    offset.y,
                                    offset.z
                                );
                                mag_ready_logged = true;
                            }
                        }
                        Err(error) => defmt::warn!(
                            "mag retry init failed at {:#04x}: {:?}",
                            lis3mdl_addr.as_u8(),
                            defmt::Debug2Format(&error)
                        ),
                    }
                } else {
                    defmt::warn!("mag retry scan: LIS3MDL still not found at 0x1C or 0x1E");
                }
            }

            let accel_g = accel_gyro.read_accel().unwrap();
            let gyro_dps = accel_gyro.read_gyro().unwrap();
            let corrected_mag_mgauss = if magnetometer_enabled {
                match magnetometer.read_magnetic_mgauss() {
                    Ok(mag_mgauss) => {
                        let corrected = mag_calibration.update(stemma_qt_9dof_body_vector(Vector3::new(
                            mag_mgauss.x_mgauss,
                            mag_mgauss.y_mgauss,
                            mag_mgauss.z_mgauss,
                        )));
                        mag_calibration.is_ready().then_some(corrected)
                    }
                    Err(error) => {
                        magnetometer_enabled = false;
                        defmt::warn!(
                            "mag read failed, disabling magnetic yaw fusion: {:?}",
                            defmt::Debug2Format(&error)
                        );
                        None
                    }
                }
            } else {
                None
            };

            let accel_gyro_sample = stemma_qt_9dof_accel_gyro_sample(
                Vector3::new(accel_g.x, accel_g.y, accel_g.z),
                Vector3::new(gyro_dps.x, gyro_dps.y, gyro_dps.z),
                GRAVITY_M_S2,
            );
            let estimate = update_estimate(
                &mut estimator,
                accel_gyro_sample,
                corrected_mag_mgauss,
                &mag_calibration,
                &mut mag_ready_logged,
                dt,
            );
            if sample_count % REPORT_PERIOD_SAMPLES == 0 {
                log_report(
                    sample_count * SAMPLE_PERIOD_MS,
                    estimate,
                    estimate.orientation,
                    estimate.relative_altitude_m,
                );
            }

            wait_until(&timer, next_tick);
            next_tick += period;
        }
    }
}

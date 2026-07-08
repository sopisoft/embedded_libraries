#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

// This example connects an RP2350 board to the Adafruit LSM6DS3TR-C + LIS3MDL 9-DoF breakout and an LPS25HB barometer over I2C.
//
// It performs:
//
// 1. attitude estimation with online gyro-bias calibration and optional
//    magnetometer fusion after hard-iron calibration has enough span,
// 2. barometric altitude tracking from the LPS25HB, with a relative
//    altitude reference taken from the first pressure sample,
// 3. plot-friendly `defmt` output for use with `imu-viz`.
//
// Suggested wiring for a Pico 2 style board:
// - 9-DoF breakout:
//   - GPIO16 -> STEMMA QT SDA
//   - GPIO17 -> STEMMA QT SCL
// - LPS25HB breakout:
//   - GPIO18 -> SDA
//   - GPIO19 -> SCL
// - 3V3    -> VIN / VDD
// - GND    -> GND
//
// This example assumes the default breakout strap state:
// - LSM6DS3TR-C: 0x6A
// - LIS3MDL:     0x1C
// - LPS25HB:     0x5C

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod stemma_qt_9dof_lps25hb;

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod embedded_example {
    use core::cell::RefCell;
    use core::sync::atomic::{AtomicU32, Ordering};

    use defmt_rtt as _;
    use embedded_hal::digital::OutputPin as _;
    use imu::{EskfEstimator, MagnetometerCalibrator, SharedI2c};
    use linked_list_allocator::LockedHeap;
    use lis3mdl::{Address as Lis3mdlAddress, Lis3mdl};
    use lps25hb::{
        Config as Lps25hbConfig, Lps25hb, OutputDataRate as Lps25hbOutputDataRate,
        PressureAverage as Lps25hbPressureAverage, TemperatureAverage as Lps25hbTemperatureAverage,
    };
    use lsm6ds3tr::{
        AccelSampleRate, AccelScale, AccelSettings, GyroSettings, LSM6DS3TR, LsmSettings,
    };
    use panic_probe as _;
    use rp235x_hal as hal;

    use crate::stemma_qt_9dof_lps25hb::*;
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
        let imu_sda_pin = pins.gpio16.into_pull_up_input();
        let imu_scl_pin = pins.gpio17.into_pull_up_input();
        let baro_sda_pin = pins.gpio18.into_pull_up_input();
        let baro_scl_pin = pins.gpio19.into_pull_up_input();
        let mut ready_led = pins.gpio25.into_push_pull_output();
        let _ = ready_led.set_low();
        let mut gpio_in = hal::Sio::read_bank0();
        defmt::info!(
            "imu gpio16_sda_high={:?} gpio17_scl_high={:?}",
            ((gpio_in >> 16) & 1) != 0,
            ((gpio_in >> 17) & 1) != 0
        );
        defmt::info!("attempt imu i2c0 bus recovery");
        let mut imu_scl_recovery =
            imu_scl_pin.into_push_pull_output_in_state(hal::gpio::PinState::High);
        for _ in 0..16 {
            let _ = imu_scl_recovery.set_low();
            for _ in 0..1024 {
                core::hint::spin_loop();
            }
            let _ = imu_scl_recovery.set_high();
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
        let imu_scl_pin = imu_scl_recovery.into_pull_up_input();

        let imu_i2c = hal::i2c::I2C::i2c0(
            pac.I2C0,
            imu_sda_pin.reconfigure(),
            imu_scl_pin.reconfigure(),
            100u32.kHz(),
            &mut pac.RESETS,
            clocks.system_clock.freq(),
        );

        gpio_in = hal::Sio::read_bank0();
        defmt::info!(
            "baro gpio18_sda_high={:?} gpio19_scl_high={:?}",
            ((gpio_in >> 18) & 1) != 0,
            ((gpio_in >> 19) & 1) != 0
        );
        defmt::info!("attempt baro i2c1 bus recovery");
        let mut baro_scl_recovery =
            baro_scl_pin.into_push_pull_output_in_state(hal::gpio::PinState::High);
        for _ in 0..16 {
            let _ = baro_scl_recovery.set_low();
            for _ in 0..1024 {
                core::hint::spin_loop();
            }
            let _ = baro_scl_recovery.set_high();
            for _ in 0..1024 {
                core::hint::spin_loop();
            }
            gpio_in = hal::Sio::read_bank0();
            if ((gpio_in >> 18) & 1) != 0 {
                break;
            }
        }
        defmt::info!(
            "baro recovery result: gpio18_sda_high={:?} gpio19_scl_high={:?}",
            ((gpio_in >> 18) & 1) != 0,
            ((gpio_in >> 19) & 1) != 0
        );
        let baro_scl_pin = baro_scl_recovery.into_pull_up_input();

        let baro_i2c = hal::i2c::I2C::i2c1(
            pac.I2C1,
            baro_sda_pin.reconfigure(),
            baro_scl_pin.reconfigure(),
            100u32.kHz(),
            &mut pac.RESETS,
            clocks.system_clock.freq(),
        );

        defmt::info!("imu i2c0 ready");
        defmt::info!("baro i2c1 ready");
        let imu_shared_bus = RefCell::new(imu_i2c);
        let baro_shared_bus = RefCell::new(baro_i2c);
        let lsm6ds3tr_addr = LSM6DS3TR_ADDR;
        let detected_lis3mdl_addr = detect_lis3mdl_address(&imu_shared_bus);
        let lps25hb_addr = LPS25HB_ADDR;
        match detected_lis3mdl_addr {
            Some(address) => defmt::info!("detected LIS3MDL address={:#04x}", address.as_u8()),
            None => defmt::warn!("LIS3MDL not found at 0x1C or 0x1E during startup scan"),
        }
        defmt::info!("using fixed LPS25HB address={:#04x}", lps25hb_addr.as_u8());

        let lsm_settings = LsmSettings::basic()
            .with_accel(
                AccelSettings::new()
                    .with_sample_rate(AccelSampleRate::_104Hz)
                    .with_scale(AccelScale::_2G),
            )
            .with_gyro(GyroSettings::new());

        let mut accel_gyro = LSM6DS3TR::new(Lsm6ds3trI2c::new(
            SharedI2c::new(&imu_shared_bus),
            lsm6ds3tr_addr,
        ))
        .with_settings(lsm_settings);

        let mut lis3mdl_addr = detected_lis3mdl_addr.unwrap_or(Lis3mdlAddress::Addr1c);
        let mut magnetometer = Lis3mdl::new(SharedI2c::new(&imu_shared_bus), lis3mdl_addr);
        let mut barometer = Lps25hb::new_i2c(SharedI2c::new(&baro_shared_bus), lps25hb_addr);

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
        defmt::info!("init LPS25HB");
        barometer
            .init(Lps25hbConfig {
                output_data_rate: Lps25hbOutputDataRate::Hz7,
                pressure_average: Lps25hbPressureAverage::Avg512,
                temperature_average: Lps25hbTemperatureAverage::Avg32,
                block_data_update: true,
                differential_output: false,
            })
            .unwrap();

        defmt::info!("9-DoF + LPS25HB estimation example");
        defmt::info!("LSM6DS3TR addr={:?}", lsm6ds3tr_addr);
        defmt::info!(
            "LIS3MDL addr={:?} enabled={:?}",
            lis3mdl_addr.as_u8(),
            magnetometer_enabled
        );
        defmt::info!("LPS25HB addr={:?}", lps25hb_addr.as_u8());
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
        match barometer.who_am_i() {
            Ok(device_id) => defmt::info!("LPS25HB WHO_AM_I={:?}", device_id),
            Err(error) => defmt::warn!(
                "barometer who_am_i failed after init: {:?}",
                defmt::Debug2Format(&error)
            ),
        }
        defmt::info!(
            "state log format: t_ms, quaternion, euler_deg, altitude_m"
        );
        defmt::info!(
            "baro altitude is referenced to the first pressure sample, so the plot starts near zero"
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
        run_estimation_loop(
            &mut accel_gyro,
            &mut magnetometer,
            &mut barometer,
            &mut estimator,
            &mut mag_calibration,
            &mut magnetometer_enabled,
            &mut lis3mdl_addr,
            &imu_shared_bus,
            &timer,
            period,
        )
    }
}

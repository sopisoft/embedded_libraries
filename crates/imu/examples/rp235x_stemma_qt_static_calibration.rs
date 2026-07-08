#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

// Static calibration helper for RP2350 + Adafruit STEMMA QT LSM6DS3TR-C/LIS3MDL.
//
// Use this when the board is resting on the bench and you want to tune:
// - startup accel / gyro biases
// - stationary-detection thresholds
// - vertical deadband values

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod embedded_example {
    use core::cell::RefCell;
    use core::sync::atomic::{AtomicU32, Ordering};

    use defmt_rtt as _;
    use embedded_hal::i2c::I2c;
    use imu::{
        AllanImuCalibrator, SharedI2c, StationaryDetection, Vector3, find_i2c_address_by_id,
        stemma_qt_9dof_accel_gyro_sample, stemma_qt_9dof_body_vector,
    };
    use libm::{fabsf, sqrtf};
    use linked_list_allocator::LockedHeap;
    use lis3mdl::{
        Address as Lis3mdlAddress, Config as Lis3mdlConfig, DataRate as Lis3mdlDataRate,
        FullScale as Lis3mdlFullScale, Lis3mdl, MeasurementMode as Lis3mdlMeasurementMode,
        OperatingMode as Lis3mdlOperatingMode,
    };
    use lsm6ds3tr::{
        AccelSampleRate, AccelScale, AccelSettings, GyroSettings, LSM6DS3TR, LsmSettings,
        interface::Interface,
    };
    use panic_probe as _;
    use rp235x_hal as hal;

    use hal::clocks::Clock;
    use hal::fugit::RateExtU32;
    #[unsafe(link_section = ".start_block")]
    #[used]
    pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

    static DEFMT_TIMESTAMP: AtomicU32 = AtomicU32::new(0);
    defmt::timestamp!("{=u32}", DEFMT_TIMESTAMP.fetch_add(1, Ordering::Relaxed));

    #[global_allocator]
    static HEAP: LockedHeap = LockedHeap::empty();

    const XTAL_FREQ_HZ: u32 = 12_000_000;
    const SAMPLE_PERIOD_MS: u32 = 10;
    const STARTUP_CALIBRATION_SAMPLES: u32 = 500;
    const STATIC_CAPTURE_SAMPLES: u32 = 12_000;
    const REPORT_PERIOD_SAMPLES: u32 = 20;
    const VALIDATION_REPORTS: u32 = 5;
    const GRAVITY_M_S2: f32 = 9.80665;
    const HEAP_SIZE: usize = 4096;
    const WHO_AM_I_REGISTER: u8 = 0x0F;
    const LSM6DS3TR_DEVICE_ID: u8 = 0x6A;
    const LSM6DS3TR_ADDR_CANDIDATES: [u8; 2] = [0x6A, 0x6B];
    const LIS3MDL_ADDR_CANDIDATES: [u8; 2] = [0x1C, 0x1E];
    const ALLAN_CLUSTER_SIZES_SAMPLES: [u32; 13] =
        [1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1_024, 2_048, 4_096];

    #[unsafe(link_section = ".uninit")]
    static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    #[derive(Copy, Clone, Debug)]
    struct RunningVec3Stats {
        count: u32,
        sum: Vector3,
        sum_sq: Vector3,
        max_abs: Vector3,
    }

    impl RunningVec3Stats {
        const fn new() -> Self {
            Self {
                count: 0,
                sum: Vector3::ZERO,
                sum_sq: Vector3::ZERO,
                max_abs: Vector3::ZERO,
            }
        }

        fn update(&mut self, sample: Vector3) {
            self.count = self.count.saturating_add(1);
            self.sum += sample;
            self.sum_sq += sample * sample;
            self.max_abs = self.max_abs.max(sample.abs());
        }

        fn mean(&self) -> Vector3 {
            if self.count == 0 {
                Vector3::ZERO
            } else {
                self.sum / self.count as f32
            }
        }

        fn stddev(&self) -> Vector3 {
            if self.count == 0 {
                return Vector3::ZERO;
            }

            let inv_count = 1.0 / self.count as f32;
            let mean = self.mean();
            let variance = self.sum_sq * inv_count - mean * mean;
            Vector3::new(
                sqrtf(variance.x.max(0.0)),
                sqrtf(variance.y.max(0.0)),
                sqrtf(variance.z.max(0.0)),
            )
        }
    }

    struct Lsm6ds3trI2c<BUS> {
        bus: BUS,
        address: u8,
    }

    impl<BUS> Lsm6ds3trI2c<BUS> {
        const fn new(bus: BUS, address: u8) -> Self {
            Self { bus, address }
        }
    }

    impl<BUS> Interface for Lsm6ds3trI2c<BUS>
    where
        BUS: I2c,
    {
        type Error = BUS::Error;

        fn write(&mut self, addr: u8, value: u8) -> Result<(), Self::Error> {
            self.bus.write(self.address, &[addr, value])
        }

        fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
            self.bus.write_read(self.address, &[addr], buffer)
        }
    }

    fn log_allan_summary<const LEVELS: usize>(
        prefix: &str,
        calibration: &imu::AllanImuCalibration<LEVELS>,
    ) {
        let summary = calibration.summary;
        defmt::info!(
            "{} allan accel noise_density [m/s2*sqrt(s)]=({:?}, {:?}, {:?})",
            prefix,
            summary.accel_noise_density_m_s2_sqrt_s.x,
            summary.accel_noise_density_m_s2_sqrt_s.y,
            summary.accel_noise_density_m_s2_sqrt_s.z
        );
        defmt::info!(
            "{} allan gyro noise_density [rad/s*sqrt(s)]=({:?}, {:?}, {:?})",
            prefix,
            summary.gyro_noise_density_rad_s_sqrt_s.x,
            summary.gyro_noise_density_rad_s_sqrt_s.y,
            summary.gyro_noise_density_rad_s_sqrt_s.z
        );
        defmt::info!(
            "{} suggested stationary thresholds accel_tolerance_m_s2={:?} gyro_tolerance_rad_s={:?} ({:?} deg/s)",
            prefix,
            summary.recommended_accel_stationary_tolerance_m_s2,
            summary.recommended_gyro_stationary_tolerance_rad_s,
            summary.recommended_gyro_stationary_tolerance_rad_s.to_degrees(),
        );
    }

    fn wait_until(timer: &hal::Timer<hal::timer::CopyableTimer0>, deadline: hal::timer::Instant) {
        while timer.get_counter() < deadline {
            core::hint::spin_loop();
        }
    }

    fn detect_lsm6ds3tr_address<BUS>(shared_bus: &RefCell<BUS>) -> u8
    where
        BUS: I2c,
    {
        find_i2c_address_by_id(
            shared_bus,
            &LSM6DS3TR_ADDR_CANDIDATES,
            WHO_AM_I_REGISTER,
            LSM6DS3TR_DEVICE_ID,
        )
        .unwrap_or_else(|| panic!("LSM6DS3TR-C not found at 0x6A or 0x6B"))
    }

    fn detect_lis3mdl_address<BUS>(shared_bus: &RefCell<BUS>) -> Lis3mdlAddress
    where
        BUS: I2c,
    {
        let address = find_i2c_address_by_id(
            shared_bus,
            &LIS3MDL_ADDR_CANDIDATES,
            WHO_AM_I_REGISTER,
            lis3mdl::DEVICE_ID,
        )
        .unwrap_or_else(|| panic!("LIS3MDL not found at 0x1C or 0x1E"));

        Lis3mdlAddress::from_u8(address).unwrap()
    }

    fn calibrate_imu_biases<IFACE>(
        accel_gyro: &mut LSM6DS3TR<IFACE>,
        timer: &hal::Timer<hal::timer::CopyableTimer0>,
        period: hal::fugit::MicrosDurationU32,
    ) -> (Vector3, Vector3)
    where
        IFACE: Interface,
        IFACE::Error: core::fmt::Debug,
    {
        let mut calibrator = AllanImuCalibrator::new(
            STARTUP_CALIBRATION_SAMPLES,
            GRAVITY_M_S2,
            SAMPLE_PERIOD_MS as f32 * 1.0e-3,
            ALLAN_CLUSTER_SIZES_SAMPLES,
        );
        let mut next_tick = timer.get_counter() + period;

        defmt::info!(
            "static calibration: keep the IMU still for {=u32} samples ({:?} s)",
            STARTUP_CALIBRATION_SAMPLES,
            STARTUP_CALIBRATION_SAMPLES as f32 * SAMPLE_PERIOD_MS as f32 * 1.0e-3,
        );

        for _ in 0..STARTUP_CALIBRATION_SAMPLES {
            let accel_g = accel_gyro.read_accel().unwrap();
            let gyro_dps = accel_gyro.read_gyro().unwrap();
            calibrator.update(stemma_qt_9dof_accel_gyro_sample(
                Vector3::new(accel_g.x, accel_g.y, accel_g.z),
                Vector3::new(gyro_dps.x, gyro_dps.y, gyro_dps.z),
                GRAVITY_M_S2,
            ));
            wait_until(timer, next_tick);
            next_tick += period;
        }

        let calibration = calibrator.finish().unwrap();
        let biases = calibration.biases;
        log_allan_summary("startup", &calibration);
        (biases.gyro_bias_rad_s, biases.accel_bias_m_s2)
    }

    fn is_stationary(
        sample: imu::AccelGyroSample,
        accel_bias: Vector3,
        gyro_bias: Vector3,
        stationary: StationaryDetection,
    ) -> bool {
        let corrected_accel = sample.accel_m_s2 - accel_bias;
        let corrected_gyro = sample.gyro_rad_s - gyro_bias;
        fabsf(corrected_accel.length() - GRAVITY_M_S2) < stationary.accel_tolerance_m_s2
            && corrected_gyro.length() < stationary.gyro_tolerance_rad_s
    }

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
        let sio = hal::Sio::new(pac.SIO);
        let pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
        let i2c = hal::i2c::I2C::i2c0(
            pac.I2C0,
            pins.gpio16.reconfigure(),
            pins.gpio17.reconfigure(),
            100u32.kHz(),
            &mut pac.RESETS,
            clocks.system_clock.freq(),
        );

        let shared_bus = RefCell::new(i2c);
        let lsm6ds3tr_addr = detect_lsm6ds3tr_address(&shared_bus);
        let lis3mdl_addr = detect_lis3mdl_address(&shared_bus);

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
        let mut magnetometer = Lis3mdl::new(SharedI2c::new(&shared_bus), lis3mdl_addr);

        accel_gyro.init().unwrap();
        magnetometer
            .init(Lis3mdlConfig {
                full_scale: Lis3mdlFullScale::Gauss4,
                operating_mode: Lis3mdlOperatingMode::MediumPerformance,
                measurement_mode: Lis3mdlMeasurementMode::Continuous,
                data_rate: Lis3mdlDataRate::Fast,
                ..Lis3mdlConfig::default()
            })
            .unwrap();

        defmt::info!("boot");
        defmt::info!("static calibration example");
        defmt::info!("LSM6DS3TR addr={:?}", lsm6ds3tr_addr);
        defmt::info!("LIS3MDL addr={:?}", lis3mdl_addr.as_u8());

        let period = hal::fugit::MicrosDurationU32::from_ticks(SAMPLE_PERIOD_MS * 1_000);
        let (gyro_bias, accel_bias) = calibrate_imu_biases(&mut accel_gyro, &timer, period);

        defmt::info!(
            "estimated gyro bias [rad/s]=({:?}, {:?}, {:?})",
            gyro_bias.x,
            gyro_bias.y,
            gyro_bias.z,
        );
        defmt::info!(
            "estimated accel bias [m/s2]=({:?}, {:?}, {:?})",
            accel_bias.x,
            accel_bias.y,
            accel_bias.z,
        );
        defmt::info!(
            "capture still data for {=u32} samples ({:?} s) to estimate noise",
            STATIC_CAPTURE_SAMPLES,
            STATIC_CAPTURE_SAMPLES as f32 * SAMPLE_PERIOD_MS as f32 * 1.0e-3,
        );
        defmt::info!(
            "allan longest cluster = {=u32} samples ({:?} s)",
            ALLAN_CLUSTER_SIZES_SAMPLES[ALLAN_CLUSTER_SIZES_SAMPLES.len() - 1],
            ALLAN_CLUSTER_SIZES_SAMPLES[ALLAN_CLUSTER_SIZES_SAMPLES.len() - 1] as f32
                * SAMPLE_PERIOD_MS as f32
                * 1.0e-3,
        );

        let mut accel_stats = RunningVec3Stats::new();
        let mut gyro_stats = RunningVec3Stats::new();
        let mut allan_capture = AllanImuCalibrator::new(
            STATIC_CAPTURE_SAMPLES,
            GRAVITY_M_S2,
            SAMPLE_PERIOD_MS as f32 * 1.0e-3,
            ALLAN_CLUSTER_SIZES_SAMPLES,
        );
        let mut next_tick = timer.get_counter() + period;

        for _ in 0..STATIC_CAPTURE_SAMPLES {
            let accel_g = accel_gyro.read_accel().unwrap();
            let gyro_dps = accel_gyro.read_gyro().unwrap();
            let raw_sample = stemma_qt_9dof_accel_gyro_sample(
                Vector3::new(accel_g.x, accel_g.y, accel_g.z),
                Vector3::new(gyro_dps.x, gyro_dps.y, gyro_dps.z),
                GRAVITY_M_S2,
            );
            let corrected_accel = raw_sample.accel_m_s2 - accel_bias;
            let corrected_gyro = raw_sample.gyro_rad_s - gyro_bias;

            accel_stats.update(corrected_accel - Vector3::new(0.0, 0.0, GRAVITY_M_S2));
            gyro_stats.update(corrected_gyro);
            allan_capture.update(raw_sample);

            wait_until(&timer, next_tick);
            next_tick += period;
        }

        let accel_mean = accel_stats.mean();
        let accel_std = accel_stats.stddev();
        let gyro_mean = gyro_stats.mean();
        let gyro_std = gyro_stats.stddev();
        let allan_capture = allan_capture.finish().unwrap();

        let suggested_accel_tolerance_m_s2 = allan_capture
            .summary
            .recommended_accel_stationary_tolerance_m_s2
            .max(0.15);
        let suggested_gyro_tolerance_rad_s = allan_capture
            .summary
            .recommended_gyro_stationary_tolerance_rad_s
            .max(0.5f32.to_radians());
        let suggested_vertical_deadband_m_s2 = (accel_std.z * 4.0).max(0.05);

        defmt::info!(
            "accel residual mean [m/s2]=({:?}, {:?}, {:?})",
            accel_mean.x,
            accel_mean.y,
            accel_mean.z,
        );
        defmt::info!(
            "accel residual stddev [m/s2]=({:?}, {:?}, {:?})",
            accel_std.x,
            accel_std.y,
            accel_std.z,
        );
        defmt::info!(
            "gyro residual mean [rad/s]=({:?}, {:?}, {:?})",
            gyro_mean.x,
            gyro_mean.y,
            gyro_mean.z,
        );
        defmt::info!(
            "gyro residual stddev [rad/s]=({:?}, {:?}, {:?})",
            gyro_std.x,
            gyro_std.y,
            gyro_std.z,
        );
        log_allan_summary("capture", &allan_capture);
        defmt::info!("recommended runtime settings:");
        defmt::info!("  StationaryDetection {{");
        defmt::info!(
            "    accel_tolerance_m_s2: {:?},",
            suggested_accel_tolerance_m_s2,
        );
        defmt::info!(
            "    gyro_tolerance_rad_s: {:?}, // {:?} deg/s",
            suggested_gyro_tolerance_rad_s,
            suggested_gyro_tolerance_rad_s.to_degrees(),
        );
        defmt::info!(
            "    vertical_accel_deadband_m_s2: {:?},",
            suggested_vertical_deadband_m_s2,
        );
        defmt::info!("    ..StationaryDetection::default()");
        defmt::info!("  }}");
        defmt::info!("  beta:");
        defmt::info!("    smooth = 0.10");
        defmt::info!("    balanced = 0.15");
        defmt::info!("    responsive = 0.20");
        defmt::info!(
            "capture length summary: startup={:?} s still={:?} s",
            STARTUP_CALIBRATION_SAMPLES as f32 * SAMPLE_PERIOD_MS as f32 * 1.0e-3,
            STATIC_CAPTURE_SAMPLES as f32 * SAMPLE_PERIOD_MS as f32 * 1.0e-3,
        );

        let suggested_stationary = StationaryDetection {
            accel_tolerance_m_s2: suggested_accel_tolerance_m_s2,
            gyro_tolerance_rad_s: suggested_gyro_tolerance_rad_s,
            vertical_accel_deadband_m_s2: suggested_vertical_deadband_m_s2,
            ..StationaryDetection::default()
        };
        let validation_samples = VALIDATION_REPORTS * REPORT_PERIOD_SAMPLES;
        defmt::info!(
            "validate suggested thresholds for {=u32} samples",
            validation_samples,
        );

        let mut sample_count = 0u32;
        let mut stationary_count = 0u32;
        next_tick = timer.get_counter() + period;

        while sample_count < validation_samples {
            let accel_g = accel_gyro.read_accel().unwrap();
            let gyro_dps = accel_gyro.read_gyro().unwrap();
            let mag_mgauss = magnetometer.read_magnetic_mgauss().unwrap();

            let corrected_sample = stemma_qt_9dof_accel_gyro_sample(
                Vector3::new(accel_g.x, accel_g.y, accel_g.z),
                Vector3::new(gyro_dps.x, gyro_dps.y, gyro_dps.z),
                GRAVITY_M_S2,
            );
            let corrected_accel = corrected_sample.accel_m_s2 - accel_bias;
            let corrected_gyro = corrected_sample.gyro_rad_s - gyro_bias;
            let mag = stemma_qt_9dof_body_vector(Vector3::new(
                mag_mgauss.x_mgauss,
                mag_mgauss.y_mgauss,
                mag_mgauss.z_mgauss,
            ));

            let stationary_now = is_stationary(
                corrected_sample,
                accel_bias,
                gyro_bias,
                suggested_stationary,
            );
            if stationary_now {
                stationary_count = stationary_count.saturating_add(1);
            }

            sample_count = sample_count.wrapping_add(1);
            if sample_count % REPORT_PERIOD_SAMPLES == 0 {
                defmt::info!(
                    "static validation sample_ms={=u32} accel_norm_error_m_s2={:?} gyro_norm_deg_s={:?} mag_norm_mgauss={:?} stationary={:?}",
                    sample_count * SAMPLE_PERIOD_MS,
                    fabsf(corrected_accel.length() - GRAVITY_M_S2),
                    corrected_gyro.length().to_degrees(),
                    mag.length(),
                    stationary_now,
                );
            }

            wait_until(&timer, next_tick);
            next_tick += period;
        }

        defmt::info!(
            "validation stationary ratio={=u32}/{=u32}",
            stationary_count,
            validation_samples,
        );
        defmt::info!("static calibration complete; logging stopped");
        loop {
            core::hint::spin_loop();
        }
    }
}

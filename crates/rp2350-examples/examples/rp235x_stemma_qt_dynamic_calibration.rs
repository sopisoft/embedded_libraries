#![no_std]
#![no_main]

mod embedded_example {
    use core::cell::RefCell;
    use core::sync::atomic::{AtomicU32, Ordering};

    use defmt_rtt as _;
    use embedded_hal::i2c::I2c;
    use imu::{
        AllanImuCalibrator, MagnetometerCalibrator, SharedI2c, Vec3, find_i2c_address_by_id,
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
    use rp2350_examples::sensors::Lsm6ds3trI2c;

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
    const STARTUP_CALIBRATION_SAMPLES: u32 = 300;
    const REPORT_PERIOD_SAMPLES: u32 = 20;
    const MAG_CALIBRATION_MIN_SPAN_MGAUSS: f32 = 200.0;
    const GRAVITY_M_S2: f32 = 9.80665;
    const HEAP_SIZE: usize = 4096;
    const WHO_AM_I_REGISTER: u8 = 0x0F;
    const LSM6DS3TR_DEVICE_ID: u8 = 0x6A;
    const LSM6DS3TR_ADDR_CANDIDATES: [u8; 2] = [0x6A, 0x6B];
    const LIS3MDL_ADDR_CANDIDATES: [u8; 2] = [0x1C, 0x1E];
    const ALLAN_CLUSTER_SIZES_SAMPLES: [u32; 6] = [1, 2, 4, 8, 16, 32];

    #[unsafe(link_section = ".uninit")]
    static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    #[derive(Copy, Clone, Debug)]
    struct RunningScalarStats {
        count: u32,
        sum: f32,
        sum_sq: f32,
        min: f32,
        max: f32,
        initialized: bool,
    }

    impl RunningScalarStats {
        const fn new() -> Self {
            Self {
                count: 0,
                sum: 0.0,
                sum_sq: 0.0,
                min: 0.0,
                max: 0.0,
                initialized: false,
            }
        }

        fn update(&mut self, value: f32) {
            self.count = self.count.saturating_add(1);
            self.sum += value;
            self.sum_sq += value * value;
            if !self.initialized {
                self.min = value;
                self.max = value;
                self.initialized = true;
            } else {
                self.min = self.min.min(value);
                self.max = self.max.max(value);
            }
        }

        fn mean(&self) -> f32 {
            if self.count == 0 {
                0.0
            } else {
                self.sum / self.count as f32
            }
        }

        fn stddev(&self) -> f32 {
            if self.count == 0 {
                0.0
            } else {
                let mean = self.mean();
                let variance = self.sum_sq / self.count as f32 - mean * mean;
                sqrtf(variance.max(0.0))
            }
        }
    }

    fn min_component(vec: Vec3) -> f32 {
        vec.x.min(vec.y).min(vec.z)
    }

    fn suggested_mag_calibration_min_span_mgauss(span_mgauss: Vec3) -> f32 {
        (min_component(span_mgauss) * 0.7).max(120.0)
    }

    fn log_allan_summary<const LEVELS: usize>(calibration: &imu::AllanImuCalibration<LEVELS>) {
        let summary = calibration.summary;
        defmt::info!(
            "startup allan accel noise_density [m/s2*sqrt(s)]=({:?}, {:?}, {:?})",
            summary.accel_noise_density_m_s2_sqrt_s.x,
            summary.accel_noise_density_m_s2_sqrt_s.y,
            summary.accel_noise_density_m_s2_sqrt_s.z
        );
        defmt::info!(
            "startup allan gyro noise_density [rad/s*sqrt(s)]=({:?}, {:?}, {:?})",
            summary.gyro_noise_density_rad_s_sqrt_s.x,
            summary.gyro_noise_density_rad_s_sqrt_s.y,
            summary.gyro_noise_density_rad_s_sqrt_s.z
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
    ) -> (Vec3, Vec3)
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
            "dynamic calibration: keep the IMU still for {=u32} startup samples",
            STARTUP_CALIBRATION_SAMPLES,
        );

        while !calibrator.is_complete() {
            let sample = match accel_gyro.read_accel() {
                Ok(accel_g) => match accel_gyro.read_gyro() {
                    Ok(gyro_dps) => Some(stemma_qt_9dof_accel_gyro_sample(
                        Vec3::new(accel_g.x, accel_g.y, accel_g.z),
                        Vec3::new(gyro_dps.x, gyro_dps.y, gyro_dps.z),
                        GRAVITY_M_S2,
                    )),
                    Err(error) => {
                        defmt::warn!(
                            "gyroscope read failed during calibration: {:?}",
                            defmt::Debug2Format(&error)
                        );
                        None
                    }
                },
                Err(error) => {
                    defmt::warn!(
                        "accelerometer read failed during calibration: {:?}",
                        defmt::Debug2Format(&error)
                    );
                    None
                }
            };
            if let Some(sample) = sample {
                calibrator.update(sample);
            }
            wait_until(timer, next_tick);
            next_tick += period;
        }

        let calibration = match calibrator.finish() {
            Some(calibration) => calibration,
            None => defmt::panic!("IMU calibration did not collect enough samples"),
        };
        let biases = calibration.biases;
        log_allan_summary(&calibration);
        (biases.gyro_bias_rad_s, biases.accel_bias_m_s2)
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
        defmt::info!("dynamic calibration example");
        defmt::info!("LSM6DS3TR addr={:?}", lsm6ds3tr_addr);
        defmt::info!("LIS3MDL addr={:?}", lis3mdl_addr.as_u8());
        defmt::info!(
            "after startup, move the sensor through many orientations and a few brisk rotations"
        );

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

        let mut mag_calibration = MagnetometerCalibrator::new(MAG_CALIBRATION_MIN_SPAN_MGAUSS);
        let mut mag_norm_stats = RunningScalarStats::new();
        let mut max_gyro_norm_deg_s = 0.0f32;
        let mut max_accel_err_m_s2 = 0.0f32;
        let mut ready_logged = false;
        let mut sample_count = 0u32;
        let mut next_tick = timer.get_counter() + period;

        loop {
            let accel_g = match accel_gyro.read_accel() {
                Ok(value) => value,
                Err(error) => {
                    defmt::warn!(
                        "accelerometer read failed: {:?}",
                        defmt::Debug2Format(&error)
                    );
                    wait_until(&timer, next_tick);
                    next_tick += period;
                    continue;
                }
            };
            let gyro_dps = match accel_gyro.read_gyro() {
                Ok(value) => value,
                Err(error) => {
                    defmt::warn!("gyroscope read failed: {:?}", defmt::Debug2Format(&error));
                    wait_until(&timer, next_tick);
                    next_tick += period;
                    continue;
                }
            };
            let mag_mgauss = match magnetometer.read_magnetic_mgauss() {
                Ok(value) => value,
                Err(error) => {
                    defmt::warn!(
                        "magnetometer read failed: {:?}",
                        defmt::Debug2Format(&error)
                    );
                    wait_until(&timer, next_tick);
                    next_tick += period;
                    continue;
                }
            };

            let corrected_sample = stemma_qt_9dof_accel_gyro_sample(
                Vec3::new(accel_g.x, accel_g.y, accel_g.z),
                Vec3::new(gyro_dps.x, gyro_dps.y, gyro_dps.z),
                GRAVITY_M_S2,
            );
            let corrected_accel = corrected_sample.accel_m_s2.vector() - accel_bias;
            let corrected_gyro = corrected_sample.gyro_rad_s.vector() - gyro_bias;
            let corrected_mag = mag_calibration.update(stemma_qt_9dof_body_vector(Vec3::new(
                mag_mgauss.x_mgauss,
                mag_mgauss.y_mgauss,
                mag_mgauss.z_mgauss,
            )));

            let gyro_norm_deg_s = corrected_gyro.length().to_degrees();
            let accel_err_m_s2 = fabsf(corrected_accel.length() - GRAVITY_M_S2);
            max_gyro_norm_deg_s = max_gyro_norm_deg_s.max(gyro_norm_deg_s);
            max_accel_err_m_s2 = max_accel_err_m_s2.max(accel_err_m_s2);

            if mag_calibration.is_ready() {
                mag_norm_stats.update(corrected_mag.length());
                if !ready_logged {
                    let offset = mag_calibration.offset_mgauss();
                    let span = mag_calibration.span_mgauss();
                    defmt::info!(
                        "mag calibration ready, offset [mgauss]=({:?}, {:?}, {:?})",
                        offset.x,
                        offset.y,
                        offset.z,
                    );
                    defmt::info!(
                        "mag span [mgauss]=({:?}, {:?}, {:?}) min_axis={:?}",
                        span.x,
                        span.y,
                        span.z,
                        min_component(span),
                    );
                    defmt::info!(
                        "suggested mag_calibration_min_span_mgauss={:?}",
                        suggested_mag_calibration_min_span_mgauss(span),
                    );
                    defmt::info!(
                        "copy this into FILTER_MAG_CALIBRATION_MIN_SPAN_MGAUSS in rp235x_stemma_qt_9dof.rs"
                    );
                    ready_logged = true;
                }
            }

            sample_count = sample_count.wrapping_add(1);
            if sample_count.is_multiple_of(REPORT_PERIOD_SAMPLES) {
                let span = mag_calibration.span_mgauss();
                defmt::info!(
                    "dynamic sample_ms={=u32} gyro_norm_deg_s={:?} accel_norm_error_m_s2={:?} mag_span_min_mgauss={:?} mag_norm_mgauss={:?}",
                    sample_count * SAMPLE_PERIOD_MS,
                    gyro_norm_deg_s,
                    accel_err_m_s2,
                    min_component(span),
                    corrected_mag.length(),
                );
            }

            if sample_count.is_multiple_of(REPORT_PERIOD_SAMPLES * 10) {
                let span = mag_calibration.span_mgauss();
                let offset = mag_calibration.offset_mgauss();
                defmt::info!(
                    "peak gyro={:?} deg/s, peak accel error={:?} m/s2",
                    max_gyro_norm_deg_s,
                    max_accel_err_m_s2,
                );
                defmt::info!(
                    "current mag offset [mgauss]=({:?}, {:?}, {:?})",
                    offset.x,
                    offset.y,
                    offset.z,
                );
                defmt::info!(
                    "current mag span [mgauss]=({:?}, {:?}, {:?}) min_axis={:?} ready={:?}",
                    span.x,
                    span.y,
                    span.z,
                    min_component(span),
                    mag_calibration.is_ready(),
                );
                defmt::info!(
                    "suggested mag_calibration_min_span_mgauss={:?}",
                    suggested_mag_calibration_min_span_mgauss(span),
                );
                if mag_norm_stats.initialized {
                    defmt::info!(
                        "corrected mag norm mean/stddev [mgauss]=({:?}, {:?})",
                        mag_norm_stats.mean(),
                        mag_norm_stats.stddev(),
                    );
                }
            }

            wait_until(&timer, next_tick);
            next_tick += period;
        }
    }
}

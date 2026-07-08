use core::cell::RefCell;

use embedded_hal::i2c::I2c;
use fugit::MicrosDurationU32;
use imu::{
    AccelGyroSample, AllanImuCalibrator, EskfEstimator, EskfTuning, ImuEstimate,
    MagnetometerCalibrator, MargSample, StationaryDetection, Vector3, find_i2c_address_by_id,
    display_attitude, display_orientation, stemma_qt_9dof_accel_gyro_sample,
};
use lis3mdl::{Address as Lis3mdlAddress, Config as Lis3mdlConfig, Lis3mdl};
use lsm6ds3tr::{LSM6DS3TR, interface::Interface};
use rp235x_hal as hal;

pub const XTAL_FREQ_HZ: u32 = 12_000_000;
pub const SAMPLE_PERIOD_MS: u32 = 10;
pub const REPORT_PERIOD_SAMPLES: u32 = 10;
pub const MAG_RETRY_PERIOD_SAMPLES: u32 = 100;
pub const STARTUP_CALIBRATION_SAMPLES: u32 = 200;
pub const FILTER_STATIONARY_DETECTION: StationaryDetection = StationaryDetection {
    accel_tolerance_m_s2: 0.3,
    gyro_tolerance_rad_s: 3.0f32.to_radians(),
    zero_altitude_hold_gain: 0.32,
    vertical_accel_lowpass_gain: 0.1,
    vertical_accel_deadband_m_s2: 0.18,
    vertical_accel_bias_learning_gain: 0.03,
    accel_bias_learning_gain: 0.015,
    gyro_bias_learning_gain: 0.03,
};
pub const FILTER_ESKF_TUNING: EskfTuning = EskfTuning {
    accel_tilt_noise_rad: 1.5f32.to_radians(),
    mag_heading_noise_rad: 2.5f32.to_radians(),
    stationary_velocity_noise_m_s: 0.015,
    stationary_altitude_noise_m: 0.05,
    accel_tilt_gate_m_s2: 0.6,
};
pub const FILTER_MAG_CALIBRATION_MIN_SPAN_MGAUSS: f32 = 200.0;
pub const GRAVITY_M_S2: f32 = 9.80665;
pub const HEAP_SIZE: usize = 4096;
pub const LSM6DS3TR_ADDR: u8 = 0x6A;
pub const WHO_AM_I_REGISTER: u8 = 0x0F;
pub const LIS3MDL_ADDR_CANDIDATES: [u8; 2] = [0x1C, 0x1E];
pub const ALLAN_CLUSTER_SIZES_SAMPLES: [u32; 6] = [1, 2, 4, 8, 16, 32];

pub struct Lsm6ds3trI2c<BUS> {
    bus: BUS,
    address: u8,
}

impl<BUS> Lsm6ds3trI2c<BUS> {
    pub const fn new(bus: BUS, address: u8) -> Self {
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

pub fn wait_until(timer: &hal::Timer<hal::timer::CopyableTimer0>, deadline: hal::timer::Instant) {
    while timer.get_counter() < deadline {
        core::hint::spin_loop();
    }
}

pub fn init_magnetometer<I2C>(
    magnetometer: &mut Lis3mdl<I2C>,
) -> Result<(), lis3mdl::Error<I2C::Error>>
where
    I2C: I2c,
{
    magnetometer.init(Lis3mdlConfig::default())
}

pub fn detect_lis3mdl_address<BUS>(shared_bus: &RefCell<BUS>) -> Option<Lis3mdlAddress>
where
    BUS: I2c,
{
    find_i2c_address_by_id(
        shared_bus,
        &LIS3MDL_ADDR_CANDIDATES,
        WHO_AM_I_REGISTER,
        lis3mdl::DEVICE_ID,
    )
    .and_then(Lis3mdlAddress::from_u8)
}

pub fn log_filter_configuration() {
    defmt::info!("filter backend=eskf");
    defmt::info!(
        "filter stationary accel_tolerance_m_s2={:?}",
        FILTER_STATIONARY_DETECTION.accel_tolerance_m_s2
    );
    defmt::info!(
        "filter stationary gyro_tolerance_rad_s={:?} ({:?} deg/s)",
        FILTER_STATIONARY_DETECTION.gyro_tolerance_rad_s,
        FILTER_STATIONARY_DETECTION
            .gyro_tolerance_rad_s
            .to_degrees(),
    );
    defmt::info!(
        "filter stationary zero_altitude_hold_gain={:?}",
        FILTER_STATIONARY_DETECTION.zero_altitude_hold_gain
    );
    defmt::info!(
        "filter stationary world_accel_lowpass_gain={:?}",
        FILTER_STATIONARY_DETECTION.vertical_accel_lowpass_gain
    );
    defmt::info!(
        "filter stationary world_accel_deadband_m_s2={:?}",
        FILTER_STATIONARY_DETECTION.vertical_accel_deadband_m_s2
    );
    defmt::info!(
        "filter stationary world_accel_bias_learning_gain={:?}",
        FILTER_STATIONARY_DETECTION.vertical_accel_bias_learning_gain
    );
    defmt::info!(
        "filter stationary accel_bias_learning_gain={:?}",
        FILTER_STATIONARY_DETECTION.accel_bias_learning_gain
    );
    defmt::info!(
        "filter stationary gyro_bias_learning_gain={:?}",
        FILTER_STATIONARY_DETECTION.gyro_bias_learning_gain
    );
    defmt::info!(
        "filter mag_calibration_min_span_mgauss={:?}",
        FILTER_MAG_CALIBRATION_MIN_SPAN_MGAUSS
    );
    defmt::info!(
        "filter eskf tilt_noise_deg={:?} mag_noise_deg={:?} tilt_gate_m_s2={:?}",
        FILTER_ESKF_TUNING.accel_tilt_noise_rad.to_degrees(),
        FILTER_ESKF_TUNING.mag_heading_noise_rad.to_degrees(),
        FILTER_ESKF_TUNING.accel_tilt_gate_m_s2,
    );
}

fn log_allan_summary<const LEVELS: usize>(calibration: &imu::AllanImuCalibration<LEVELS>) {
    let summary = calibration.summary;
    defmt::info!(
        "allan accel noise_density [m/s2*sqrt(s)]=({:?}, {:?}, {:?})",
        summary.accel_noise_density_m_s2_sqrt_s.x,
        summary.accel_noise_density_m_s2_sqrt_s.y,
        summary.accel_noise_density_m_s2_sqrt_s.z
    );
    defmt::info!(
        "allan accel bias_instability [m/s2]=({:?}, {:?}, {:?})",
        summary.accel_bias_instability_m_s2.x,
        summary.accel_bias_instability_m_s2.y,
        summary.accel_bias_instability_m_s2.z
    );
    defmt::info!(
        "allan gyro noise_density [rad/s*sqrt(s)]=({:?}, {:?}, {:?})",
        summary.gyro_noise_density_rad_s_sqrt_s.x,
        summary.gyro_noise_density_rad_s_sqrt_s.y,
        summary.gyro_noise_density_rad_s_sqrt_s.z
    );
    defmt::info!(
        "allan gyro bias_instability [rad/s]=({:?}, {:?}, {:?})",
        summary.gyro_bias_instability_rad_s.x,
        summary.gyro_bias_instability_rad_s.y,
        summary.gyro_bias_instability_rad_s.z
    );
    defmt::info!(
        "suggested stationary thresholds accel_tolerance_m_s2={:?} gyro_tolerance_rad_s={:?} ({:?} deg/s)",
        summary.recommended_accel_stationary_tolerance_m_s2,
        summary.recommended_gyro_stationary_tolerance_rad_s,
        summary.recommended_gyro_stationary_tolerance_rad_s.to_degrees(),
    );
}

pub fn calibrate_imu_biases<IFACE>(
    accel_gyro: &mut LSM6DS3TR<IFACE>,
    timer: &hal::Timer<hal::timer::CopyableTimer0>,
    period: hal::fugit::MicrosDurationU32,
) -> (Vector3, Vector3)
where
    IFACE: Interface,
    IFACE::Error: core::fmt::Debug,
{
    defmt::info!(
        "imu calibration: keep the IMU still for {=u32} samples",
        STARTUP_CALIBRATION_SAMPLES,
    );

    let mut calibrator = AllanImuCalibrator::new(
        STARTUP_CALIBRATION_SAMPLES,
        GRAVITY_M_S2,
        SAMPLE_PERIOD_MS as f32 * 1.0e-3,
        ALLAN_CLUSTER_SIZES_SAMPLES,
    );
    let mut next_tick = timer.get_counter() + period;
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
    defmt::info!(
        "gyro bias [rad/s]=({:?}, {:?}, {:?})",
        biases.gyro_bias_rad_s.x,
        biases.gyro_bias_rad_s.y,
        biases.gyro_bias_rad_s.z
    );
    defmt::info!(
        "accel bias [m/s2]=({:?}, {:?}, {:?})",
        biases.accel_bias_m_s2.x,
        biases.accel_bias_m_s2.y,
        biases.accel_bias_m_s2.z
    );
    log_allan_summary(&calibration);
    (biases.gyro_bias_rad_s, biases.accel_bias_m_s2)
}

pub fn update_estimate(
    estimator: &mut EskfEstimator,
    accel_gyro_sample: AccelGyroSample,
    corrected_mag_mgauss: Option<Vector3>,
    mag_calibration: &MagnetometerCalibrator,
    mag_ready_logged: &mut bool,
    dt: MicrosDurationU32,
) -> ImuEstimate {
    if let Some(corrected_mag_mgauss) = corrected_mag_mgauss {
        if mag_calibration.is_ready() && !*mag_ready_logged {
            let offset = mag_calibration.offset_mgauss();
            defmt::info!(
                "mag calibration ready, offset [mgauss]=({:?}, {:?}, {:?})",
                offset.x,
                offset.y,
                offset.z
            );
            *mag_ready_logged = true;
        }
        estimator.update_marg(MargSample::new(accel_gyro_sample, corrected_mag_mgauss), dt)
    } else {
        estimator.update_imu(accel_gyro_sample, dt)
    }
}

pub fn log_report(
    elapsed_ms: u32,
    estimate: ImuEstimate,
    orientation: imu::Quaternion,
    altitude_m: f32,
) {
    let orientation = display_orientation(orientation);
    let euler = display_attitude(estimate.orientation);
    defmt::info!(
        "state t_ms={=u32} quat=({:?}, {:?}, {:?}, {:?}) euler_deg=({:?}, {:?}, {:?}) velocity_m_s=({:?}, {:?}, {:?}) altitude_m={:?}",
        elapsed_ms,
        orientation.w,
        orientation.x,
        orientation.y,
        orientation.z,
        euler.x.to_degrees(),
        euler.y.to_degrees(),
        euler.z.to_degrees(),
        estimate.velocity_world.x,
        estimate.velocity_world.y,
        estimate.velocity_world.z,
        altitude_m,
    );
}

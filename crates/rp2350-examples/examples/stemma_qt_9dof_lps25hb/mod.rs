use core::cell::RefCell;
use core::fmt::Write as _;

use embedded_hal::i2c::I2c;
use fugit::MicrosDurationU32;
use imu::{
    AccelGyroSample, AllanImuCalibrator, EskfEstimator, EskfTuning, ImuEstimate,
    MagnetometerCalibrator, MargSample, SharedI2c, StationaryDetection, Vec3, display_attitude,
    display_orientation, find_i2c_address_by_id, stemma_qt_9dof_accel_gyro_sample,
    stemma_qt_9dof_body_vector,
};
use lis3mdl::{Address as Lis3mdlAddress, Config as Lis3mdlConfig, Lis3mdl};
use lps25hb::{Address as Lps25hbAddress, Lps25hb, pressure_to_altitude_m};
use lsm6ds3tr::{LSM6DS3TR, interface::Interface};
use rp235x_hal as hal;
pub use rp2350_examples::sensors::Lsm6ds3trI2c;
use usbd_serial::SerialPort;

mod altitude;
mod runtime;

pub type UsbSerial<'a> = SerialPort<'a, hal::usb::UsbBus, [u8; 512], [u8; 512]>;
pub type UsbDevice<'a> = usb_device::device::UsbDevice<'a, hal::usb::UsbBus>;

pub use altitude::{AltitudeComplementaryFilter, BaroState};
pub use runtime::run_estimation_loop;

pub const XTAL_FREQ_HZ: u32 = 12_000_000;
pub const SAMPLE_PERIOD_MS: u32 = 10;
pub const BARO_SAMPLE_PERIOD_SAMPLES: u32 = 4;
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
    accel_tilt_gate_m_s2: 0.6,
};
pub const FILTER_MAG_CALIBRATION_MIN_SPAN_MGAUSS: f32 = 200.0;
pub const BARO_REFERENCE_SAMPLES: u32 = 16;
pub const BARO_PRESSURE_LOWPASS_GAIN: f32 = 0.2;
pub const ALTITUDE_COMPLEMENTARY_POSITION_GAIN: f32 = 0.3;
pub const ALTITUDE_COMPLEMENTARY_VELOCITY_GAIN: f32 = 0.08;
pub const ALTITUDE_ACCEL_BIAS_GAIN: f32 = 0.03;
pub const ALTITUDE_ACCEL_LOWPASS_GAIN: f32 = 0.18;
pub const ALTITUDE_ACCEL_DEADBAND_M_S2: f32 = 0.06;
pub const ALTITUDE_ZERO_VELOCITY_GAIN: f32 = 0.18;
pub const ESKF_VERTICAL_SPEED_FEEDBACK_NOISE_M_S: f32 = 0.08;
pub const GRAVITY_M_S2: f32 = 9.80665;
pub const HEAP_SIZE: usize = 4096;
pub const LSM6DS3TR_ADDR: u8 = 0x6A;
pub const WHO_AM_I_REGISTER: u8 = 0x0F;
pub const LIS3MDL_ADDR_CANDIDATES: [u8; 2] = [0x1C, 0x1E];
pub const LPS25HB_ADDR: Lps25hbAddress = Lps25hbAddress::Addr5c;
pub const ALLAN_CLUSTER_SIZES_SAMPLES: [u32; 6] = [1, 2, 4, 8, 16, 32];

pub fn write_usb_state(
    serial: &mut UsbSerial<'_>,
    elapsed_ms: u32,
    estimate: ImuEstimate,
    altitude_m: f32,
    magnetometer_enabled: bool,
    magnetometer_ready: bool,
) {
    let orientation = display_orientation(estimate.orientation);
    let mut line = heapless::String::<256>::new();
    let _ = writeln!(
        line,
        "state t_ms={} quat=({:.7},{:.7},{:.7},{:.7}) velocity_m_s=({:.5},{:.5},{:.5}) altitude_m={:.5} mag_enabled={} mag_ready={}",
        elapsed_ms,
        orientation.w,
        orientation.x,
        orientation.y,
        orientation.z,
        estimate.velocity_world.x,
        estimate.velocity_world.y,
        estimate.velocity_world.z,
        altitude_m,
        magnetometer_enabled,
        magnetometer_ready,
    );

    let mut remaining = line.as_bytes();
    while !remaining.is_empty() {
        match serial.write(remaining) {
            Ok(written) if written > 0 => remaining = &remaining[written..],
            _ => break,
        }
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
    defmt::info!("filter backend=eskf+altitude_complementary");
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
    defmt::info!(
        "filter altitude gains pos={:?} vel={:?} accel_bias={:?} accel_lp={:?} accel_deadband={:?} zero_vz={:?}",
        ALTITUDE_COMPLEMENTARY_POSITION_GAIN,
        ALTITUDE_COMPLEMENTARY_VELOCITY_GAIN,
        ALTITUDE_ACCEL_BIAS_GAIN,
        ALTITUDE_ACCEL_LOWPASS_GAIN,
        ALTITUDE_ACCEL_DEADBAND_M_S2,
        ALTITUDE_ZERO_VELOCITY_GAIN,
    );
    defmt::info!(
        "filter eskf vertical_speed_feedback_noise_m_s={:?}",
        ESKF_VERTICAL_SPEED_FEEDBACK_NOISE_M_S,
    );
    defmt::info!(
        "baro reference_samples={=u32} pressure_lowpass_gain={:?}",
        BARO_REFERENCE_SAMPLES,
        BARO_PRESSURE_LOWPASS_GAIN,
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
        summary
            .recommended_gyro_stationary_tolerance_rad_s
            .to_degrees(),
    );
}

pub fn calibrate_imu_biases<IFACE>(
    accel_gyro: &mut LSM6DS3TR<IFACE>,
    timer: &hal::Timer<hal::timer::CopyableTimer0>,
    period: hal::fugit::MicrosDurationU32,
) -> (Vec3, Vec3)
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
    corrected_mag_mgauss: Option<Vec3>,
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
        estimator.update_marg(
            MargSample::from_vectors(accel_gyro_sample, corrected_mag_mgauss),
            dt,
        )
    } else {
        estimator.update_imu(accel_gyro_sample, dt)
    }
}

pub fn log_report(elapsed_ms: u32, estimate: ImuEstimate, orientation: imu::Quat, altitude_m: f32) {
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

use super::*;

pub fn run_estimation_loop<'a, IFACE, IMUBUS, BAROBUS>(
    accel_gyro: &mut LSM6DS3TR<IFACE>,
    magnetometer: &mut Lis3mdl<SharedI2c<'a, IMUBUS>>,
    barometer: &mut Lps25hb<lps25hb::i2c::I2cInterface<SharedI2c<'a, BAROBUS>>>,
    estimator: &mut EskfEstimator,
    mag_calibration: &mut MagnetometerCalibrator,
    magnetometer_enabled: &mut bool,
    lis3mdl_addr: &mut Lis3mdlAddress,
    imu_shared_bus: &'a RefCell<IMUBUS>,
    timer: &hal::Timer<hal::timer::CopyableTimer0>,
    period: hal::fugit::MicrosDurationU32,
) -> !
where
    IFACE: Interface,
    IFACE::Error: core::fmt::Debug,
    IMUBUS: I2c,
    BAROBUS: I2c,
{
    fn sample_is_stationary(
        sample: AccelGyroSample,
        accel_bias_m_s2: Vector3,
        gyro_bias_rad_s: Vector3,
    ) -> bool {
        let corrected_accel_m_s2 = sample.accel_m_s2 - accel_bias_m_s2;
        let corrected_gyro_rad_s = sample.gyro_rad_s - gyro_bias_rad_s;
        (corrected_accel_m_s2.length() - GRAVITY_M_S2).abs()
            < FILTER_STATIONARY_DETECTION.accel_tolerance_m_s2
            && corrected_gyro_rad_s.length() < FILTER_STATIONARY_DETECTION.gyro_tolerance_rad_s
    }

    let mut mag_ready_logged = false;
    let mut last_mag_retry_sample = 0u32;
    let mut sample_count = 0u32;
    let mut last_sample_tick = timer.get_counter();
    let mut last_baro_tick = timer.get_counter();
    let mut baro_state = BaroState::new();
    let mut altitude_filter = AltitudeComplementaryFilter::new();
    let mut latest_baro_pressure_hpa = 0.0;
    let mut latest_baro_temperature_c = 0.0;
    let mut next_tick = timer.get_counter() + period;

    loop {
        let now = timer.get_counter();
        let dt = MicrosDurationU32::from_ticks(
            now.ticks().wrapping_sub(last_sample_tick.ticks()) as u32
        );
        last_sample_tick = now;
        sample_count = sample_count.wrapping_add(1);

        if !*magnetometer_enabled
            && sample_count.wrapping_sub(last_mag_retry_sample) >= MAG_RETRY_PERIOD_SAMPLES
        {
            last_mag_retry_sample = sample_count;
            if let Some(detected_addr) = detect_lis3mdl_address(imu_shared_bus) {
                if detected_addr != *lis3mdl_addr {
                    *lis3mdl_addr = detected_addr;
                    *magnetometer = Lis3mdl::new(SharedI2c::new(imu_shared_bus), *lis3mdl_addr);
                }
                match init_magnetometer(magnetometer) {
                    Ok(()) => {
                        *magnetometer_enabled = true;
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
        let corrected_mag_mgauss = if *magnetometer_enabled {
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
                    *magnetometer_enabled = false;
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
            estimator,
            accel_gyro_sample,
            corrected_mag_mgauss,
            mag_calibration,
            &mut mag_ready_logged,
            dt,
        );
        let stationary = sample_is_stationary(
            accel_gyro_sample,
            estimator.accel_bias(),
            estimator.gyro_bias(),
        );
        let corrected_accel_world_z = estimate
            .orientation
            .mul_vec3(accel_gyro_sample.accel_m_s2 - estimator.accel_bias())
            .z
            - GRAVITY_M_S2;
        altitude_filter.predict(dt.as_secs_f32(), corrected_accel_world_z, stationary);

        if sample_count % BARO_SAMPLE_PERIOD_SAMPLES == 0
            && barometer.pressure_data_ready().unwrap()
        {
            let baro_dt = MicrosDurationU32::from_ticks(
                now.ticks().wrapping_sub(last_baro_tick.ticks()) as u32,
            );
            last_baro_tick = now;
            let measurement = barometer.read_measurement().unwrap();
            latest_baro_pressure_hpa = measurement.pressure_hpa;
            latest_baro_temperature_c = measurement.temperature_c;
            let (baro_altitude_m, reference_ready) =
                baro_state.update_baro(measurement.pressure_hpa);
            if let Some(baro_altitude_m) = baro_altitude_m {
                altitude_filter.update_altitude(baro_altitude_m, baro_dt.as_secs_f32());
            }
            if reference_ready {
                defmt::info!(
                    "baro reference ready after {=u32} fresh samples: {:?} hPa",
                    BARO_REFERENCE_SAMPLES,
                    latest_baro_pressure_hpa
                );
            }
        }

        if sample_count % REPORT_PERIOD_SAMPLES == 0 {
            let (altitude_m, vertical_speed_m_s) = altitude_filter.current().unwrap_or((
                estimator.navigator_state().relative_altitude_m,
                estimator.navigator_state().velocity_world.z,
            ));
            estimator.correct_altitude(altitude_m, ESKF_ALTITUDE_FEEDBACK_NOISE_M);
            estimator.correct_vertical_velocity(
                vertical_speed_m_s,
                ESKF_VERTICAL_SPEED_FEEDBACK_NOISE_M_S,
            );
            let navigator_state = estimator.navigator_state();
            defmt::info!(
                "baro pressure={:?} hPa temp={:?} C baro_z={:?} m altitude={:?} m",
                latest_baro_pressure_hpa,
                latest_baro_temperature_c,
                baro_state.current_altitude().unwrap_or(0.0),
                navigator_state.relative_altitude_m,
            );
            log_report(
                sample_count * SAMPLE_PERIOD_MS,
                estimate,
                estimate.orientation,
                navigator_state.relative_altitude_m,
            );
        }

        wait_until(timer, next_tick);
        next_tick += period;
    }
}

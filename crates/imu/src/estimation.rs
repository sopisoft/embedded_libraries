//! Small estimation helpers that connect sensor samples to the existing fusion crates.

use eskf::Eskf;
use fugit::MicrosDurationU32;
use glam::{EulerRot, Mat3, Quat, Vec3};
use libm::{atan2f, fabsf, sqrtf};

use crate::sample::{AccelGyroSample, Acceleration, AngularVelocity, MargSample};

/// One fused estimate produced by [`MargEstimator`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ImuEstimate {
    pub orientation: Quat,
    pub euler: Vec3,
    pub relative_altitude_m: f32,
    pub vertical_speed_m_s: f32,
    pub velocity_world: Vec3,
}

/// Snapshot of the internal inertial navigator state.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NavigatorState {
    pub relative_altitude_m: f32,
    pub velocity_world: Vec3,
    pub gravity_world: Vec3,
}

/// Heuristics used to slow relative-altitude drift when the IMU is stationary.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StationaryDetection {
    /// Allowed deviation from 1 g before the sample is considered "moving".
    pub accel_tolerance_m_s2: f32,
    /// Allowed angular-rate magnitude before the sample is considered "moving".
    pub gyro_tolerance_rad_s: f32,
    /// Gain used when holding altitude while stationary.
    pub zero_altitude_hold_gain: f32,
    /// Low-pass gain applied to world-frame vertical acceleration before integration.
    pub vertical_accel_lowpass_gain: f32,
    /// Deadband applied to filtered vertical acceleration.
    pub vertical_accel_deadband_m_s2: f32,
    /// Gain used to learn residual world-frame vertical acceleration while stationary.
    pub vertical_accel_bias_learning_gain: f32,
    /// Gain used to adapt the accelerometer bias estimate while stationary.
    pub accel_bias_learning_gain: f32,
    /// Gain used to adapt the gyroscope bias estimate while stationary.
    pub gyro_bias_learning_gain: f32,
}

impl Default for StationaryDetection {
    fn default() -> Self {
        Self {
            accel_tolerance_m_s2: 0.25,
            gyro_tolerance_rad_s: 3.0f32.to_radians(),
            zero_altitude_hold_gain: 0.25,
            vertical_accel_lowpass_gain: 0.12,
            vertical_accel_deadband_m_s2: 0.15,
            vertical_accel_bias_learning_gain: 0.02,
            accel_bias_learning_gain: 0.01,
            gyro_bias_learning_gain: 0.02,
        }
    }
}

/// Tuning parameters for the ESKF-backed estimator.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EskfTuning {
    /// Standard deviation of the roll/pitch pseudo-measurement from accelerometer tilt.
    pub accel_tilt_noise_rad: f32,
    /// Standard deviation of the yaw pseudo-measurement from the magnetometer.
    pub mag_heading_noise_rad: f32,
    /// Velocity measurement noise used for zero-velocity updates while stationary.
    pub stationary_velocity_noise_m_s: f32,
    /// Accelerometer magnitude gate for tilt corrections.
    pub accel_tilt_gate_m_s2: f32,
}

impl Default for EskfTuning {
    fn default() -> Self {
        Self {
            accel_tilt_noise_rad: 2.0f32.to_radians(),
            mag_heading_noise_rad: 8.0f32.to_radians(),
            stationary_velocity_noise_m_s: 0.02,
            accel_tilt_gate_m_s2: 0.8,
        }
    }
}

fn is_stationary_sample(
    accel_m_s2: Vec3,
    gyro_rad_s: Vec3,
    gravity_m_s2: f32,
    stationary: StationaryDetection,
) -> bool {
    let accel_norm = accel_m_s2.length();
    let gyro_norm = gyro_rad_s.length();
    fabsf(accel_norm - gravity_m_s2) < stationary.accel_tolerance_m_s2
        && gyro_norm < stationary.gyro_tolerance_rad_s
}

fn accel_tilt_is_usable(accel_m_s2: Vec3, gravity_m_s2: f32, gate_m_s2: f32) -> bool {
    let accel_norm = accel_m_s2.length();
    accel_norm > 1.0 && fabsf(accel_norm - gravity_m_s2) < gate_m_s2
}

fn orientation_from_accel_and_yaw(accel_m_s2: Vec3, yaw_rad: f32) -> Option<Quat> {
    let accel_norm = accel_m_s2.length();
    if accel_norm <= 1.0e-6 {
        return None;
    }

    let accel = accel_m_s2 / accel_norm;
    let roll_rad = atan2f(accel.y, accel.z);
    let pitch_rad = atan2f(-accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z));
    Some(Quat::from_euler(EulerRot::XYZ, roll_rad, pitch_rad, yaw_rad).normalize())
}

fn orientation_from_accel_and_reference_heading(
    accel_m_s2: Vec3,
    reference_orientation: Quat,
) -> Option<Quat> {
    let up_body = accel_m_s2.normalize_or_zero();
    if up_body.length_squared() <= 1.0e-6 {
        return None;
    }

    let north_guess_body = reference_orientation.conjugate().mul_vec3(Vec3::X);
    let north_projected = north_guess_body - up_body * up_body.dot(north_guess_body);
    let north_body = north_projected.normalize_or_zero();
    if north_body.length_squared() <= 1.0e-6 {
        return None;
    }

    let east_body = up_body.cross(north_body).normalize_or_zero();
    if east_body.length_squared() <= 1.0e-6 {
        return None;
    }

    let north_body = east_body.cross(up_body).normalize_or_zero();
    let body_from_world = Mat3::from_cols(north_body, east_body, up_body);
    let world_from_body = body_from_world.transpose();
    let orientation = Quat::from_mat3(&world_from_body).normalize();
    orientation.is_finite().then_some(orientation)
}

fn orientation_from_accel_and_mag(accel_m_s2: Vec3, mag_body: Vec3) -> Option<Quat> {
    let up_body = accel_m_s2.normalize_or_zero();
    let mag_body = mag_body.normalize_or_zero();
    if up_body.length_squared() <= 1.0e-6 || mag_body.length_squared() <= 1.0e-6 {
        return None;
    }

    let east_body = up_body.cross(mag_body).normalize_or_zero();
    if east_body.length_squared() <= 1.0e-6 {
        return None;
    }
    let north_body = east_body.cross(up_body).normalize_or_zero();
    let body_from_world = Mat3::from_cols(north_body, east_body, up_body);
    let world_from_body = body_from_world.transpose();
    let orientation = Quat::from_mat3(&world_from_body).normalize();
    orientation.is_finite().then_some(orientation)
}

fn yaw_from_mag_and_tilt(orientation: Quat, mag_body: Vec3) -> Option<f32> {
    let mag_world = orientation.mul_vec3(mag_body.normalize_or_zero());
    let horizontal_sq = mag_world.x * mag_world.x + mag_world.y * mag_world.y;
    (horizontal_sq > 1.0e-6).then_some(atan2f(mag_world.y, mag_world.x))
}

fn heading_error_abs_rad(measured_yaw_rad: f32, reference_yaw_rad: f32) -> f32 {
    let mut error = measured_yaw_rad - reference_yaw_rad;
    while error > core::f32::consts::PI {
        error -= core::f32::consts::TAU;
    }
    while error < -core::f32::consts::PI {
        error += core::f32::consts::TAU;
    }
    error.abs()
}

fn estimate_from_state(
    orientation: Quat,
    relative_altitude_m: f32,
    velocity_world: Vec3,
) -> ImuEstimate {
    let (roll_rad, pitch_rad, yaw_rad) = orientation.to_euler(EulerRot::XYZ);
    ImuEstimate {
        euler: Vec3::new(roll_rad, pitch_rad, yaw_rad),
        orientation,
        relative_altitude_m,
        vertical_speed_m_s: velocity_world.z,
        velocity_world,
    }
}

/// Lightweight MARG estimator for attitude and drift-prone relative altitude.
#[derive(Debug)]
pub struct MargEstimator {
    orientation: Quat,
    velocity_world: Vec3,
    gravity_world: Vec3,
    relative_altitude_m: f32,
    gravity_m_s2: f32,
    attitude_correction_gain: f32,
    stationary: StationaryDetection,
    accel_bias_m_s2: Vec3,
    gyro_bias_rad_s: Vec3,
    initialized: bool,
    mag_reference_norm: Option<f32>,
    stationary_altitude_reference_m: Option<f32>,
    filtered_linear_accel_world_m_s2: Vec3,
    linear_accel_bias_world_m_s2: Vec3,
}

impl MargEstimator {
    /// Creates an estimator with the supplied attitude correction gain.
    pub fn with_attitude_correction_gain(attitude_correction_gain: f32) -> Self {
        Self {
            orientation: Quat::IDENTITY,
            velocity_world: Vec3::ZERO,
            gravity_world: Vec3::new(0.0, 0.0, -9.80665),
            relative_altitude_m: 0.0,
            gravity_m_s2: 9.80665,
            attitude_correction_gain: attitude_correction_gain.max(0.0),
            stationary: StationaryDetection::default(),
            accel_bias_m_s2: Vec3::ZERO,
            gyro_bias_rad_s: Vec3::ZERO,
            initialized: false,
            mag_reference_norm: None,
            stationary_altitude_reference_m: None,
            filtered_linear_accel_world_m_s2: Vec3::ZERO,
            linear_accel_bias_world_m_s2: Vec3::ZERO,
        }
    }

    /// Replaces the stationary-detection parameters.
    pub fn with_stationary_detection(mut self, stationary: StationaryDetection) -> Self {
        self.stationary = stationary;
        self
    }

    /// Returns the current fused orientation.
    pub fn orientation(&self) -> Quat {
        self.orientation
    }

    /// Returns the current gyroscope bias estimate in rad/s.
    pub fn gyro_bias(&self) -> Vec3 {
        self.gyro_bias_rad_s
    }

    /// Returns the current accelerometer bias estimate in m/s^2.
    pub fn accel_bias(&self) -> Vec3 {
        self.accel_bias_m_s2
    }

    /// Sets the accelerometer bias estimate in m/s^2.
    pub fn set_accel_bias(&mut self, accel_bias_m_s2: Vec3) {
        self.accel_bias_m_s2 = accel_bias_m_s2;
    }

    /// Sets the gyroscope bias estimate in rad/s.
    pub fn set_gyro_bias(&mut self, gyro_bias_rad_s: Vec3) {
        self.gyro_bias_rad_s = gyro_bias_rad_s;
    }

    /// Returns a snapshot of the internal inertial navigator state.
    pub fn navigator_state(&self) -> NavigatorState {
        NavigatorState {
            relative_altitude_m: self.relative_altitude_m,
            velocity_world: self.velocity_world,
            gravity_world: self.gravity_world,
        }
    }

    /// Updates the estimator using a full 9-DoF sample.
    pub fn update_marg(&mut self, sample: MargSample, dt: MicrosDurationU32) -> ImuEstimate {
        let sample = self.correct_sample(sample.accel_gyro, sample.mag_body.vector());
        self.initialize_orientation(
            sample.accel_gyro.accel_m_s2.vector(),
            Some(sample.mag_body.vector()),
        );
        self.update_orientation(
            sample.accel_gyro.accel_m_s2.vector(),
            sample.accel_gyro.gyro_rad_s.vector(),
            Some(sample.mag_body.vector()),
            dt,
        );
        self.integrate_linear_motion(sample.accel_gyro, dt)
    }

    /// Updates the estimator using only accelerometer and gyroscope data.
    pub fn update_imu(&mut self, sample: AccelGyroSample, dt: MicrosDurationU32) -> ImuEstimate {
        let sample = self.correct_accel_gyro_sample(sample);
        self.initialize_orientation(sample.accel_m_s2.vector(), None);
        self.update_orientation(
            sample.accel_m_s2.vector(),
            sample.gyro_rad_s.vector(),
            None,
            dt,
        );
        self.integrate_linear_motion(sample, dt)
    }

    fn correct_sample(&mut self, sample: AccelGyroSample, mag_body: Vec3) -> MargSample {
        MargSample::new(
            self.correct_accel_gyro_sample(sample),
            crate::MagneticField::new(mag_body),
        )
    }

    fn initialize_orientation(&mut self, accel_m_s2: Vec3, mag_body: Option<Vec3>) {
        if self.initialized || !accel_tilt_is_usable(accel_m_s2, self.gravity_m_s2, 3.0) {
            return;
        }

        let mag_norm = mag_body
            .map(|value| value.length())
            .filter(|value| *value > 1.0e-6);
        let orientation = mag_body
            .and_then(|mag_body| orientation_from_accel_and_mag(accel_m_s2, mag_body))
            .or_else(|| orientation_from_accel_and_yaw(accel_m_s2, 0.0))
            .unwrap_or(Quat::IDENTITY);

        self.orientation = orientation;
        self.velocity_world = Vec3::ZERO;
        self.relative_altitude_m = 0.0;
        self.filtered_linear_accel_world_m_s2 = Vec3::ZERO;
        self.linear_accel_bias_world_m_s2 = Vec3::ZERO;
        self.mag_reference_norm = mag_norm;
        self.stationary_altitude_reference_m = None;
        self.initialized = true;
    }

    fn update_orientation(
        &mut self,
        accel_m_s2: Vec3,
        gyro_rad_s: Vec3,
        mag_body: Option<Vec3>,
        dt: MicrosDurationU32,
    ) {
        let dt_s = dt.as_secs_f32();
        if dt_s <= 0.0 {
            return;
        }

        self.orientation =
            (self.orientation * Quat::from_scaled_axis(gyro_rad_s * dt_s)).normalize();

        if accel_tilt_is_usable(accel_m_s2, self.gravity_m_s2, 2.5) {
            let measured_orientation = if let Some(mag_body) = mag_body {
                let mag_norm = mag_body.length();
                let reference_norm = self
                    .mag_reference_norm
                    .map(|reference| reference + (mag_norm - reference) * 0.02)
                    .unwrap_or(mag_norm);
                self.mag_reference_norm = Some(reference_norm);

                let norm_ratio = if reference_norm > 1.0e-6 {
                    mag_norm / reference_norm
                } else {
                    1.0
                };

                if (0.7..=1.3).contains(&norm_ratio) {
                    orientation_from_accel_and_mag(accel_m_s2, mag_body).or_else(|| {
                        yaw_from_mag_and_tilt(self.orientation, mag_body)
                            .and_then(|yaw_rad| orientation_from_accel_and_yaw(accel_m_s2, yaw_rad))
                    })
                } else {
                    let (_, _, yaw_rad) = self.orientation.to_euler(EulerRot::XYZ);
                    orientation_from_accel_and_yaw(accel_m_s2, yaw_rad)
                }
            } else {
                let (_, _, yaw_rad) = self.orientation.to_euler(EulerRot::XYZ);
                orientation_from_accel_and_yaw(accel_m_s2, yaw_rad)
            };

            if let Some(measured_orientation) = measured_orientation {
                let alpha = (self.attitude_correction_gain * dt_s).clamp(0.0, 1.0);
                self.orientation = self
                    .orientation
                    .slerp(measured_orientation, alpha)
                    .normalize();
            }
        }
    }

    fn correct_accel_gyro_sample(&mut self, sample: AccelGyroSample) -> AccelGyroSample {
        let accel_m_s2 = sample.accel_m_s2.vector();
        let gyro_rad_s = sample.gyro_rad_s.vector();
        let corrected_accel_m_s2 = accel_m_s2 - self.accel_bias_m_s2;
        let corrected_gyro_rad_s = gyro_rad_s - self.gyro_bias_rad_s;

        if self.is_stationary(corrected_accel_m_s2, corrected_gyro_rad_s) {
            let gravity_body = corrected_accel_m_s2.normalize_or_zero() * self.gravity_m_s2;
            let accel_bias_candidate = accel_m_s2 - gravity_body;
            self.accel_bias_m_s2 = self.accel_bias_m_s2.lerp(
                accel_bias_candidate,
                self.stationary.accel_bias_learning_gain,
            );
            self.gyro_bias_rad_s = self
                .gyro_bias_rad_s
                .lerp(gyro_rad_s, self.stationary.gyro_bias_learning_gain);
        }

        AccelGyroSample::new(
            Acceleration::new(accel_m_s2 - self.accel_bias_m_s2),
            AngularVelocity::new(gyro_rad_s - self.gyro_bias_rad_s),
            sample.temperature_c,
        )
    }

    fn is_stationary(&self, accel_m_s2: Vec3, gyro_rad_s: Vec3) -> bool {
        is_stationary_sample(accel_m_s2, gyro_rad_s, self.gravity_m_s2, self.stationary)
    }

    fn integrate_linear_motion(
        &mut self,
        sample: AccelGyroSample,
        dt: MicrosDurationU32,
    ) -> ImuEstimate {
        let dt_s = dt.as_secs_f32();
        let accel_m_s2 = sample.accel_m_s2.vector();
        let gyro_rad_s = sample.gyro_rad_s.vector();
        let orientation = self.orientation;

        let accel_world = orientation.mul_vec3(accel_m_s2) + self.gravity_world;
        let is_stationary = self.is_stationary(accel_m_s2, gyro_rad_s);
        if is_stationary {
            self.linear_accel_bias_world_m_s2 = self.linear_accel_bias_world_m_s2.lerp(
                accel_world,
                self.stationary.vertical_accel_bias_learning_gain,
            );
        }

        let accel_world_unbiased = accel_world - self.linear_accel_bias_world_m_s2;
        self.filtered_linear_accel_world_m_s2 = self.filtered_linear_accel_world_m_s2.lerp(
            accel_world_unbiased,
            self.stationary.vertical_accel_lowpass_gain,
        );
        let corrected_accel_world = self.filtered_linear_accel_world_m_s2.map(|value| {
            if fabsf(value) < self.stationary.vertical_accel_deadband_m_s2 {
                0.0
            } else {
                value
            }
        });
        let previous_vertical_speed_m_s = self.velocity_world.z;
        self.velocity_world += corrected_accel_world * dt_s;
        self.relative_altitude_m +=
            0.5 * (previous_vertical_speed_m_s + self.velocity_world.z) * dt_s;

        if is_stationary {
            let reference_altitude_m = *self
                .stationary_altitude_reference_m
                .get_or_insert(self.relative_altitude_m);
            self.velocity_world = Vec3::ZERO;
            self.relative_altitude_m = self.relative_altitude_m
                + (reference_altitude_m - self.relative_altitude_m)
                    * self.stationary.zero_altitude_hold_gain;
            self.filtered_linear_accel_world_m_s2 = Vec3::ZERO;
        } else {
            self.stationary_altitude_reference_m = None;
        }

        estimate_from_state(orientation, self.relative_altitude_m, self.velocity_world)
    }
}

/// ESKF-backed estimator with accelerometer and magnetometer pseudo-measurements.
#[derive(Debug)]
pub struct EskfEstimator {
    filter: Eskf,
    relative_altitude_m: f32,
    gravity_m_s2: f32,
    stationary: StationaryDetection,
    tuning: EskfTuning,
    initialized: bool,
    mag_reference_norm: Option<f32>,
    stationary_altitude_reference_m: Option<f32>,
}

impl EskfEstimator {
    /// Creates an estimator with conservative process and measurement tuning.
    pub fn new() -> Self {
        let mut filter = Eskf::new();
        filter.set_noise(0.25, 0.03, 0.003, 0.0005);
        Self {
            filter,
            relative_altitude_m: 0.0,
            gravity_m_s2: 9.80665,
            stationary: StationaryDetection::default(),
            tuning: EskfTuning::default(),
            initialized: false,
            mag_reference_norm: None,
            stationary_altitude_reference_m: None,
        }
    }

    /// Replaces the stationary-detection parameters.
    pub fn with_stationary_detection(mut self, stationary: StationaryDetection) -> Self {
        self.stationary = stationary;
        self
    }

    /// Replaces the measurement tuning parameters.
    pub fn with_tuning(mut self, tuning: EskfTuning) -> Self {
        self.tuning = tuning;
        self
    }

    /// Returns the current fused orientation.
    pub fn orientation(&self) -> Quat {
        self.filter.orientation
    }

    /// Returns the current gyroscope bias estimate in rad/s.
    pub fn gyro_bias(&self) -> Vec3 {
        self.filter.gyro_bias
    }

    /// Returns the current accelerometer bias estimate in m/s^2.
    pub fn accel_bias(&self) -> Vec3 {
        self.filter.accel_bias
    }

    /// Sets the accelerometer bias estimate in m/s^2.
    pub fn set_accel_bias(&mut self, accel_bias_m_s2: Vec3) {
        self.filter.accel_bias = accel_bias_m_s2;
    }

    /// Sets the gyroscope bias estimate in rad/s.
    pub fn set_gyro_bias(&mut self, gyro_bias_rad_s: Vec3) {
        self.filter.gyro_bias = gyro_bias_rad_s;
    }

    /// Returns a snapshot of the internal navigation state.
    pub fn navigator_state(&self) -> NavigatorState {
        NavigatorState {
            relative_altitude_m: self.relative_altitude_m,
            velocity_world: self.filter.velocity,
            gravity_world: self.filter.gravity,
        }
    }

    /// Corrects the velocity state from an external measurement.
    pub fn correct_velocity(&mut self, velocity_world: Vec3, noise_m_s: f32) {
        self.filter.correct_velocity(velocity_world, noise_m_s);
    }

    /// Sets the relative altitude from an external measurement.
    pub fn set_altitude(&mut self, altitude_m: f32) {
        self.relative_altitude_m = altitude_m;
    }

    /// Corrects vertical speed from an external measurement.
    pub fn correct_vertical_velocity(&mut self, vertical_speed_m_s: f32, noise_m_s: f32) {
        self.filter
            .correct_vertical_velocity(vertical_speed_m_s, noise_m_s);
    }

    /// Updates the estimator using only accelerometer and gyroscope data.
    pub fn update_imu(&mut self, sample: AccelGyroSample, dt: MicrosDurationU32) -> ImuEstimate {
        self.update(sample, None, dt)
    }

    /// Updates the estimator using a full 9-DoF sample.
    pub fn update_marg(&mut self, sample: MargSample, dt: MicrosDurationU32) -> ImuEstimate {
        self.update(sample.accel_gyro, Some(sample.mag_body.vector()), dt)
    }

    fn update(
        &mut self,
        sample: AccelGyroSample,
        mag_body: Option<Vec3>,
        dt: MicrosDurationU32,
    ) -> ImuEstimate {
        let accel_m_s2 = sample.accel_m_s2.vector();
        let gyro_rad_s = sample.gyro_rad_s.vector();
        let corrected_accel_m_s2 = accel_m_s2 - self.filter.accel_bias;
        let corrected_gyro_rad_s = gyro_rad_s - self.filter.gyro_bias;
        let dt_s = dt.as_secs_f32();

        self.initialize_orientation(corrected_accel_m_s2, mag_body);

        let is_stationary = is_stationary_sample(
            corrected_accel_m_s2,
            corrected_gyro_rad_s,
            self.gravity_m_s2,
            self.stationary,
        );

        if is_stationary {
            let gravity_body = corrected_accel_m_s2.normalize_or_zero() * self.gravity_m_s2;
            let accel_bias_candidate = accel_m_s2 - gravity_body;
            self.filter.accel_bias = self.filter.accel_bias.lerp(
                accel_bias_candidate,
                self.stationary.accel_bias_learning_gain,
            );
            self.filter.gyro_bias = self
                .filter
                .gyro_bias
                .lerp(gyro_rad_s, self.stationary.gyro_bias_learning_gain);
        }

        let previous_vertical_speed_m_s = self.filter.velocity.z;
        self.filter.predict(gyro_rad_s, accel_m_s2, dt);
        self.relative_altitude_m +=
            0.5 * (previous_vertical_speed_m_s + self.filter.velocity.z) * dt_s;

        if let Some(tilt_measurement) = orientation_from_accel_and_reference_heading(
            corrected_accel_m_s2,
            self.filter.orientation,
        )
        .or_else(|| {
            let (_, _, current_yaw_rad) = self.filter.orientation.to_euler(EulerRot::XYZ);
            orientation_from_accel_and_yaw(corrected_accel_m_s2, current_yaw_rad)
        }) {
            self.filter
                .correct_orientation(tilt_measurement, self.tuning.accel_tilt_noise_rad);
            if !self.filter.orientation.is_finite() {
                self.filter.orientation = tilt_measurement;
            }
        }

        if let Some(yaw_rad) = mag_body
            .and_then(|value| self.filtered_mag(value))
            .and_then(|value| yaw_from_mag_and_tilt(self.filter.orientation, value))
        {
            let (_, _, current_yaw_rad) = self.filter.orientation.to_euler(EulerRot::XYZ);
            if heading_error_abs_rad(yaw_rad, current_yaw_rad) <= 35.0f32.to_radians() {
                self.filter
                    .correct_heading(yaw_rad, self.tuning.mag_heading_noise_rad);
            }
        }

        if is_stationary {
            let reference_altitude_m = *self
                .stationary_altitude_reference_m
                .get_or_insert(self.relative_altitude_m);
            self.filter
                .correct_velocity(Vec3::ZERO, self.tuning.stationary_velocity_noise_m_s);
            self.relative_altitude_m = self.relative_altitude_m
                + (reference_altitude_m - self.relative_altitude_m)
                    * self.stationary.zero_altitude_hold_gain;
        } else {
            self.stationary_altitude_reference_m = None;
        }

        estimate_from_state(
            self.filter.orientation,
            self.relative_altitude_m,
            self.filter.velocity,
        )
    }

    fn initialize_orientation(&mut self, accel_m_s2: Vec3, mag_body: Option<Vec3>) {
        if self.initialized || !accel_tilt_is_usable(accel_m_s2, self.gravity_m_s2, 3.0) {
            return;
        }

        let orientation = orientation_from_accel_and_reference_heading(accel_m_s2, Quat::IDENTITY)
            .or_else(|| orientation_from_accel_and_yaw(accel_m_s2, 0.0))
            .unwrap_or(Quat::IDENTITY);
        self.filter.orientation = orientation;
        self.filter.velocity = Vec3::ZERO;
        self.relative_altitude_m = 0.0;
        self.mag_reference_norm = mag_body
            .map(|value| value.length())
            .filter(|value| *value > 1.0e-6);
        self.stationary_altitude_reference_m = None;
        self.initialized = true;
    }

    fn filtered_mag(&mut self, mag_body: Vec3) -> Option<Vec3> {
        let mag_norm = mag_body.length();
        if mag_norm <= 1.0e-6 {
            return None;
        }

        let reference_norm = self
            .mag_reference_norm
            .map(|reference| reference + (mag_norm - reference) * 0.02)
            .unwrap_or(mag_norm);
        self.mag_reference_norm = Some(reference_norm);

        let norm_ratio = if reference_norm > 1.0e-6 {
            mag_norm / reference_norm
        } else {
            1.0
        };
        ((0.7..=1.3).contains(&norm_ratio)).then_some(mag_body)
    }
}

impl Default for EskfEstimator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vec3;

    #[test]
    fn stationary_sample_keeps_relative_altitude_small() {
        let mut estimator = MargEstimator::with_attitude_correction_gain(0.08);
        let sample = MargSample::from_vectors(
            AccelGyroSample::from_vectors_without_temperature(
                Vec3::new(0.0, 0.0, 9.80665),
                Vec3::ZERO,
            ),
            Vec3::X,
        );
        let mut estimate = estimator.update_marg(sample, MicrosDurationU32::from_millis(10));
        for _ in 0..199 {
            estimate = estimator.update_marg(sample, MicrosDurationU32::from_millis(10));
        }

        assert!(estimate.relative_altitude_m.abs() < 1.0e-3);
        assert!(estimate.vertical_speed_m_s.abs() < 1.0e-3);
        assert!(estimate.velocity_world.x.abs() < 1.0e-3);
        assert!(estimate.velocity_world.y.abs() < 1.0e-3);
    }

    #[test]
    fn stationary_gyro_bias_learning_reduces_yaw_drift() {
        let sample = AccelGyroSample::from_vectors_without_temperature(
            Vec3::new(0.0, 0.0, 9.80665),
            Vec3::new(0.0, 0.0, 0.02),
        );
        let dt = MicrosDurationU32::from_millis(10);

        let mut without_bias_learning = MargEstimator::with_attitude_correction_gain(0.08)
            .with_stationary_detection(StationaryDetection {
                gyro_bias_learning_gain: 0.0,
                ..StationaryDetection::default()
            });
        let mut with_bias_learning = MargEstimator::with_attitude_correction_gain(0.08);

        let mut estimate_without = without_bias_learning.update_imu(sample, dt);
        let mut estimate_with = with_bias_learning.update_imu(sample, dt);
        for _ in 0..499 {
            estimate_without = without_bias_learning.update_imu(sample, dt);
            estimate_with = with_bias_learning.update_imu(sample, dt);
        }

        assert!(estimate_without.euler.z.abs() > 3.0f32.to_radians());
        assert!(estimate_with.euler.z.abs() < 1.0f32.to_radians());
        assert!(estimate_with.euler.z.abs() < estimate_without.euler.z.abs() * 0.25);
        assert!(with_bias_learning.gyro_bias().z > 0.015);
    }

    #[test]
    fn stationary_altitude_hold_limits_vertical_drift() {
        let mut estimator = MargEstimator::with_attitude_correction_gain(0.08);
        let sample = MargSample::from_vectors(
            AccelGyroSample::from_vectors_without_temperature(
                Vec3::new(0.0, 0.0, 9.83),
                Vec3::ZERO,
            ),
            Vec3::X,
        );
        let mut estimate = estimator.update_marg(sample, MicrosDurationU32::from_millis(10));
        for _ in 0..999 {
            estimate = estimator.update_marg(sample, MicrosDurationU32::from_millis(10));
        }

        assert!(estimate.relative_altitude_m.abs() < 0.05);
        assert!(estimate.vertical_speed_m_s.abs() < 0.01);
    }

    #[test]
    fn stationary_vertical_drift_stays_bounded() {
        let dt = MicrosDurationU32::from_millis(10);
        let sample = AccelGyroSample::from_vectors_without_temperature(
            Vec3::new(0.0, 0.0, 9.92),
            Vec3::ZERO,
        );

        let mut estimator = MargEstimator::with_attitude_correction_gain(0.08)
            .with_stationary_detection(StationaryDetection {
                zero_altitude_hold_gain: 0.0,
                vertical_accel_deadband_m_s2: 0.0,
                ..StationaryDetection::default()
            });
        let mut estimate = estimator.update_imu(sample, dt);
        for _ in 0..999 {
            estimate = estimator.update_imu(sample, dt);
        }

        assert!(estimate.relative_altitude_m.abs() < 0.05);
        assert!(estimate.vertical_speed_m_s.abs() < 0.01);
    }

    #[test]
    fn stationary_horizontal_velocity_stays_bounded() {
        let dt = MicrosDurationU32::from_millis(10);
        let sample = AccelGyroSample::from_vectors_without_temperature(
            Vec3::new(0.04, -0.03, 9.80665),
            Vec3::ZERO,
        );

        let mut estimator = MargEstimator::with_attitude_correction_gain(0.08);
        let mut estimate = estimator.update_imu(sample, dt);
        for _ in 0..999 {
            estimate = estimator.update_imu(sample, dt);
        }

        assert!(estimate.velocity_world.truncate().length() < 0.01);
    }

    #[test]
    fn magnetic_norm_gate_rejects_yaw_spikes() {
        let mut estimator = MargEstimator::with_attitude_correction_gain(0.08);
        let dt = MicrosDurationU32::from_millis(10);
        let stable_sample = MargSample::from_vectors(
            AccelGyroSample::from_vectors_without_temperature(
                Vec3::new(0.0, 0.0, 9.80665),
                Vec3::ZERO,
            ),
            Vec3::new(1.0, 0.0, 0.0),
        );

        for _ in 0..200 {
            estimator.update_marg(stable_sample, dt);
        }

        let yaw_before = estimator.update_marg(stable_sample, dt).euler.z;
        let disturbed_sample = MargSample::from_vectors(
            AccelGyroSample::from_vectors_without_temperature(
                Vec3::new(0.0, 0.0, 9.80665),
                Vec3::ZERO,
            ),
            Vec3::new(-4.0, 0.0, 0.0),
        );
        let yaw_after = estimator.update_marg(disturbed_sample, dt).euler.z;

        assert!((yaw_after - yaw_before).abs() < 5.0f32.to_radians());
    }

    #[test]
    fn eskf_stationary_sample_keeps_relative_altitude_small() {
        let mut estimator = EskfEstimator::new();
        let sample = MargSample::from_vectors(
            AccelGyroSample::from_vectors_without_temperature(
                Vec3::new(0.0, 0.0, 9.80665),
                Vec3::ZERO,
            ),
            Vec3::X,
        );
        let mut estimate = estimator.update_marg(sample, MicrosDurationU32::from_millis(10));
        for _ in 0..499 {
            estimate = estimator.update_marg(sample, MicrosDurationU32::from_millis(10));
        }

        assert!(estimate.relative_altitude_m.abs() < 0.05);
        assert!(estimate.vertical_speed_m_s.abs() < 0.05);
        assert!(estimate.euler.x.abs() < 1.0f32.to_radians());
        assert!(estimate.euler.y.abs() < 1.0f32.to_radians());
    }

    #[test]
    fn eskf_magnetometer_correction_limits_yaw_drift() {
        let mut estimator = EskfEstimator::new();
        let sample = MargSample::from_vectors(
            AccelGyroSample::from_vectors_without_temperature(
                Vec3::new(0.0, 0.0, 9.80665),
                Vec3::new(0.0, 0.0, 0.02),
            ),
            Vec3::X,
        );

        let mut estimate = estimator.update_marg(sample, MicrosDurationU32::from_millis(10));
        for _ in 0..499 {
            estimate = estimator.update_marg(sample, MicrosDurationU32::from_millis(10));
        }

        assert!(estimate.euler.z.abs() < 5.0f32.to_radians());
        assert!(estimator.gyro_bias().z > 0.01);
    }
}

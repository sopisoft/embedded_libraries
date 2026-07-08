//! Reusable calibration helpers for IMU and magnetometer startup flows.

use crate::{AccelGyroSample, Vector3};
use libm::sqrtf;

/// Bias estimates for a stationary accelerometer and gyroscope.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct ImuBiases {
    pub accel_bias_m_s2: Vector3,
    pub gyro_bias_rad_s: Vector3,
}

/// Accumulates stationary IMU samples and derives bias estimates.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StationaryImuCalibrator {
    required_samples: u32,
    collected_samples: u32,
    accel_sum_m_s2: Vector3,
    gyro_sum_rad_s: Vector3,
    gravity_m_s2: f32,
}

impl StationaryImuCalibrator {
    pub const fn new(required_samples: u32, gravity_m_s2: f32) -> Self {
        Self {
            required_samples,
            collected_samples: 0,
            accel_sum_m_s2: Vector3::ZERO,
            gyro_sum_rad_s: Vector3::ZERO,
            gravity_m_s2,
        }
    }

    pub const fn required_samples(&self) -> u32 {
        self.required_samples
    }

    pub const fn collected_samples(&self) -> u32 {
        self.collected_samples
    }

    pub const fn is_complete(&self) -> bool {
        self.collected_samples >= self.required_samples && self.required_samples > 0
    }

    pub fn update(&mut self, sample: AccelGyroSample) {
        self.accel_sum_m_s2 += sample.accel_m_s2;
        self.gyro_sum_rad_s += sample.gyro_rad_s;
        self.collected_samples = self.collected_samples.saturating_add(1);
    }

    pub fn finish(&self) -> Option<ImuBiases> {
        if !self.is_complete() {
            return None;
        }

        let inv_samples = 1.0 / self.collected_samples as f32;
        let accel_average = self.accel_sum_m_s2 * inv_samples;
        let gyro_average = self.gyro_sum_rad_s * inv_samples;
        let accel_bias = accel_average - accel_average.normalize_or_zero() * self.gravity_m_s2;

        Some(ImuBiases {
            accel_bias_m_s2: accel_bias,
            gyro_bias_rad_s: gyro_average,
        })
    }
}

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct AllanDeviationPoint {
    pub tau_s: f32,
    pub deviation: Vector3,
    pub pairs: u32,
}

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct AllanNoiseSummary {
    pub accel_noise_density_m_s2_sqrt_s: Vector3,
    pub accel_bias_instability_m_s2: Vector3,
    pub gyro_noise_density_rad_s_sqrt_s: Vector3,
    pub gyro_bias_instability_rad_s: Vector3,
    pub recommended_accel_stationary_tolerance_m_s2: f32,
    pub recommended_gyro_stationary_tolerance_rad_s: f32,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AllanImuCalibration<const LEVELS: usize> {
    pub biases: ImuBiases,
    pub accel_points: [AllanDeviationPoint; LEVELS],
    pub gyro_points: [AllanDeviationPoint; LEVELS],
    pub summary: AllanNoiseSummary,
}

#[derive(Copy, Clone, Debug, Default, PartialEq)]
struct AllanLevel {
    cluster_len_samples: u32,
    cluster_count: u32,
    cluster_sum: Vector3,
    previous_average: Vector3,
    has_previous_average: bool,
    delta_sum_sq: Vector3,
    pairs: u32,
}

impl AllanLevel {
    const fn new(cluster_len_samples: u32) -> Self {
        Self {
            cluster_len_samples,
            cluster_count: 0,
            cluster_sum: Vector3::ZERO,
            previous_average: Vector3::ZERO,
            has_previous_average: false,
            delta_sum_sq: Vector3::ZERO,
            pairs: 0,
        }
    }

    fn update(&mut self, sample: Vector3) {
        self.cluster_sum += sample;
        self.cluster_count = self.cluster_count.saturating_add(1);
        if self.cluster_count < self.cluster_len_samples {
            return;
        }

        let cluster_average = self.cluster_sum / self.cluster_len_samples as f32;
        if self.has_previous_average {
            let delta = cluster_average - self.previous_average;
            self.delta_sum_sq += delta * delta;
            self.pairs = self.pairs.saturating_add(1);
        }

        self.previous_average = cluster_average;
        self.has_previous_average = true;
        self.cluster_sum = Vector3::ZERO;
        self.cluster_count = 0;
    }

    fn point(&self, sample_period_s: f32) -> AllanDeviationPoint {
        let tau_s = self.cluster_len_samples as f32 * sample_period_s;
        let deviation = if self.pairs == 0 {
            Vector3::ZERO
        } else {
            let inv_pairs = 0.5 / self.pairs as f32;
            let variance = self.delta_sum_sq * inv_pairs;
            Vector3::new(
                sqrtf(variance.x.max(0.0)),
                sqrtf(variance.y.max(0.0)),
                sqrtf(variance.z.max(0.0)),
            )
        };

        AllanDeviationPoint {
            tau_s,
            deviation,
            pairs: self.pairs,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct AllanAxisAnalyzer<const LEVELS: usize> {
    sample_period_s: f32,
    levels: [AllanLevel; LEVELS],
}

impl<const LEVELS: usize> AllanAxisAnalyzer<LEVELS> {
    fn new(sample_period_s: f32, cluster_lens_samples: [u32; LEVELS]) -> Self {
        Self {
            sample_period_s,
            levels: core::array::from_fn(|index| AllanLevel::new(cluster_lens_samples[index])),
        }
    }

    fn update(&mut self, sample: Vector3) {
        let mut index = 0usize;
        while index < LEVELS {
            self.levels[index].update(sample);
            index += 1;
        }
    }

    fn points(&self) -> [AllanDeviationPoint; LEVELS] {
        core::array::from_fn(|index| self.levels[index].point(self.sample_period_s))
    }
}

/// Stationary IMU calibrator that also computes non-overlapping Allan deviation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AllanImuCalibrator<const LEVELS: usize> {
    stationary: StationaryImuCalibrator,
    accel_allan: AllanAxisAnalyzer<LEVELS>,
    gyro_allan: AllanAxisAnalyzer<LEVELS>,
    sample_period_s: f32,
}

impl<const LEVELS: usize> AllanImuCalibrator<LEVELS> {
    pub fn new(
        required_samples: u32,
        gravity_m_s2: f32,
        sample_period_s: f32,
        cluster_lens_samples: [u32; LEVELS],
    ) -> Self {
        Self {
            stationary: StationaryImuCalibrator::new(required_samples, gravity_m_s2),
            accel_allan: AllanAxisAnalyzer::new(sample_period_s, cluster_lens_samples),
            gyro_allan: AllanAxisAnalyzer::new(sample_period_s, cluster_lens_samples),
            sample_period_s,
        }
    }

    pub const fn required_samples(&self) -> u32 {
        self.stationary.required_samples()
    }

    pub const fn collected_samples(&self) -> u32 {
        self.stationary.collected_samples()
    }

    pub const fn is_complete(&self) -> bool {
        self.stationary.is_complete()
    }

    pub fn update(&mut self, sample: AccelGyroSample) {
        self.stationary.update(sample);
        self.accel_allan.update(sample.accel_m_s2);
        self.gyro_allan.update(sample.gyro_rad_s);
    }

    pub fn finish(&self) -> Option<AllanImuCalibration<LEVELS>> {
        let biases = self.stationary.finish()?;
        let accel_points = self.accel_allan.points();
        let gyro_points = self.gyro_allan.points();
        let summary =
            AllanNoiseSummary::from_points(self.sample_period_s, accel_points, gyro_points);

        Some(AllanImuCalibration {
            biases,
            accel_points,
            gyro_points,
            summary,
        })
    }
}

impl AllanNoiseSummary {
    fn from_points<const LEVELS: usize>(
        sample_period_s: f32,
        accel_points: [AllanDeviationPoint; LEVELS],
        gyro_points: [AllanDeviationPoint; LEVELS],
    ) -> Self {
        let accel_noise_density_m_s2_sqrt_s = min_noise_density(accel_points);
        let gyro_noise_density_rad_s_sqrt_s = min_noise_density(gyro_points);
        let accel_bias_instability_m_s2 = min_bias_instability(accel_points);
        let gyro_bias_instability_rad_s = min_bias_instability(gyro_points);
        let sample_period_s_sqrt = sqrtf(sample_period_s.max(1.0e-6));

        let accel_sample_std = accel_noise_density_m_s2_sqrt_s / sample_period_s_sqrt;
        let gyro_sample_std = gyro_noise_density_rad_s_sqrt_s / sample_period_s_sqrt;

        Self {
            accel_noise_density_m_s2_sqrt_s,
            accel_bias_instability_m_s2,
            gyro_noise_density_rad_s_sqrt_s,
            gyro_bias_instability_rad_s,
            recommended_accel_stationary_tolerance_m_s2: 3.0 * max_component(accel_sample_std),
            recommended_gyro_stationary_tolerance_rad_s: 3.0 * max_component(gyro_sample_std),
        }
    }
}

/// Online hard/soft-iron style magnetometer calibration from min/max tracking.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MagnetometerCalibrator {
    min_mgauss: Vector3,
    max_mgauss: Vector3,
    initialized: bool,
    min_span_mgauss: f32,
}

impl MagnetometerCalibrator {
    pub const fn new(min_span_mgauss: f32) -> Self {
        Self {
            min_mgauss: Vector3::ZERO,
            max_mgauss: Vector3::ZERO,
            initialized: false,
            min_span_mgauss,
        }
    }

    pub fn update(&mut self, raw_mgauss: Vector3) -> Vector3 {
        if !self.initialized {
            self.min_mgauss = raw_mgauss;
            self.max_mgauss = raw_mgauss;
            self.initialized = true;
        } else if !self.is_ready() {
            self.min_mgauss = self.min_mgauss.min(raw_mgauss);
            self.max_mgauss = self.max_mgauss.max(raw_mgauss);
        }

        let centered = raw_mgauss - self.offset_mgauss();
        if !self.is_ready() {
            return centered;
        }

        let half_span = (self.max_mgauss - self.min_mgauss) * 0.5;
        let average_radius = (half_span.x + half_span.y + half_span.z) / 3.0;
        Vector3::new(
            scale_axis(centered.x, half_span.x, average_radius),
            scale_axis(centered.y, half_span.y, average_radius),
            scale_axis(centered.z, half_span.z, average_radius),
        )
    }

    pub const fn initialized(&self) -> bool {
        self.initialized
    }

    pub fn is_ready(&self) -> bool {
        if !self.initialized {
            return false;
        }

        let span = self.max_mgauss - self.min_mgauss;
        span.x >= self.min_span_mgauss
            && span.y >= self.min_span_mgauss
            && span.z >= self.min_span_mgauss
    }

    pub fn offset_mgauss(&self) -> Vector3 {
        if self.initialized {
            (self.min_mgauss + self.max_mgauss) * 0.5
        } else {
            Vector3::ZERO
        }
    }

    pub fn span_mgauss(&self) -> Vector3 {
        if self.initialized {
            self.max_mgauss - self.min_mgauss
        } else {
            Vector3::ZERO
        }
    }
}

fn scale_axis(value: f32, radius: f32, target_radius: f32) -> f32 {
    if radius > 1.0 {
        value * target_radius / radius
    } else {
        value
    }
}

fn min_noise_density<const LEVELS: usize>(points: [AllanDeviationPoint; LEVELS]) -> Vector3 {
    let mut best = Vector3::splat(f32::INFINITY);
    let mut index = 0usize;
    while index < LEVELS {
        let point = points[index];
        if point.pairs > 0 && point.tau_s > 0.0 {
            let tau_s_sqrt = sqrtf(point.tau_s);
            let candidate = point.deviation * tau_s_sqrt;
            best = best.min(candidate);
        }
        index += 1;
    }

    best.map(|value| if value.is_finite() { value } else { 0.0 })
}

fn min_bias_instability<const LEVELS: usize>(points: [AllanDeviationPoint; LEVELS]) -> Vector3 {
    let mut best = Vector3::splat(f32::INFINITY);
    let mut index = 0usize;
    while index < LEVELS {
        let point = points[index];
        if point.pairs > 0 {
            best = best.min(point.deviation / 0.664);
        }
        index += 1;
    }

    best.map(|value| if value.is_finite() { value } else { 0.0 })
}

fn max_component(value: Vector3) -> f32 {
    value.x.max(value.y).max(value.z)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stationary_calibrator_estimates_biases() {
        let mut calibrator = StationaryImuCalibrator::new(4, 9.80665);
        let sample = AccelGyroSample::without_temperature(
            Vector3::new(0.1, -0.2, 9.90665),
            Vector3::new(0.01, -0.02, 0.03),
        );

        for _ in 0..4 {
            calibrator.update(sample);
        }

        let biases = calibrator.finish().expect("biases");
        assert!((biases.gyro_bias_rad_s.x - 0.01).abs() < 1.0e-6);
        assert!((biases.gyro_bias_rad_s.y + 0.02).abs() < 1.0e-6);
        assert!((biases.gyro_bias_rad_s.z - 0.03).abs() < 1.0e-6);
        assert!(biases.accel_bias_m_s2.length() > 0.05);
    }

    #[test]
    fn magnetometer_calibrator_removes_offset_after_span() {
        let mut calibrator = MagnetometerCalibrator::new(50.0);
        let samples = [
            Vector3::new(-70.0, -60.0, -55.0),
            Vector3::new(130.0, -60.0, -55.0),
            Vector3::new(-70.0, 140.0, -55.0),
            Vector3::new(-70.0, -60.0, 145.0),
        ];

        let mut corrected = Vector3::ZERO;
        for sample in samples {
            corrected = calibrator.update(sample);
        }

        assert!(calibrator.is_ready());
        assert!(
            calibrator
                .offset_mgauss()
                .distance(Vector3::new(30.0, 40.0, 45.0))
                < 1.0e-3
        );
        assert!(corrected.length() > 0.0);
    }

    #[test]
    fn allan_calibrator_reports_bias_and_noise_summary() {
        let mut calibrator =
            AllanImuCalibrator::<4>::new(16, 9.80665, 0.01, [1, 2, 4, 8]);
        let sample = AccelGyroSample::without_temperature(
            Vector3::new(0.02, -0.01, 9.82665),
            Vector3::new(0.005, -0.004, 0.003),
        );

        for _ in 0..16 {
            calibrator.update(sample);
        }

        let report = calibrator.finish().expect("report");
        assert!(report.biases.accel_bias_m_s2.length() > 0.0);
        assert!(report.summary.recommended_accel_stationary_tolerance_m_s2 >= 0.0);
        assert!(report.summary.recommended_gyro_stationary_tolerance_rad_s >= 0.0);
        assert_eq!(report.accel_points[0].tau_s, 0.01);
        assert!(report.accel_points[0].pairs > 0);
    }
}

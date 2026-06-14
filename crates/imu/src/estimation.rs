//! Small estimation helpers that connect sensor samples to the existing fusion crates.

use fugit::MicrosDurationU32;
use glam::Vec3;
use libm::fabsf;
use madgwick::Madgwick;
use navigation::InertialNavigator;

use crate::sample::{AccelGyroSample, MargSample};
use crate::{Attitude, Quaternion, Vector3};

/// One fused estimate produced by [`MargEstimator`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ImuEstimate {
    pub orientation: Quaternion,
    pub euler: Attitude,
    pub relative_altitude_m: f32,
    pub vertical_speed_m_s: f32,
    pub velocity_world: Vector3,
}

/// Snapshot of the internal inertial navigator state.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NavigatorState {
    pub position_world: Vector3,
    pub velocity_world: Vector3,
    pub gravity_world: Vector3,
}

/// Heuristics used to slow relative-altitude drift when the IMU is stationary.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StationaryDetection {
    /// Allowed deviation from 1 g before the sample is considered "moving".
    pub accel_tolerance_m_s2: f32,
    /// Allowed angular-rate magnitude before the sample is considered "moving".
    pub gyro_tolerance_rad_s: f32,
    /// Gain used when forcing vertical velocity back toward zero.
    pub zero_vertical_velocity_gain: f32,
}

impl Default for StationaryDetection {
    fn default() -> Self {
        Self {
            accel_tolerance_m_s2: 0.25,
            gyro_tolerance_rad_s: 3.0f32.to_radians(),
            zero_vertical_velocity_gain: 0.1,
        }
    }
}

/// Lightweight MARG estimator for attitude and drift-prone relative altitude.
#[derive(Debug)]
pub struct MargEstimator {
    attitude: Madgwick,
    navigator: InertialNavigator,
    gravity_m_s2: f32,
    stationary: StationaryDetection,
}

impl MargEstimator {
    /// Creates an estimator with the supplied Madgwick beta.
    pub fn new(beta: f32) -> Self {
        Self {
            attitude: Madgwick::new(beta),
            navigator: InertialNavigator::new(),
            gravity_m_s2: 9.80665,
            stationary: StationaryDetection::default(),
        }
    }

    /// Replaces the stationary-detection parameters.
    pub fn with_stationary_detection(mut self, stationary: StationaryDetection) -> Self {
        self.stationary = stationary;
        self
    }

    /// Returns the current fused orientation.
    pub fn orientation(&self) -> Quaternion {
        self.attitude.orientation()
    }

    /// Returns a snapshot of the internal inertial navigator state.
    pub fn navigator_state(&self) -> NavigatorState {
        NavigatorState {
            position_world: self.navigator.pose.position,
            velocity_world: self.navigator.velocity_world,
            gravity_world: self.navigator.gravity_world,
        }
    }

    /// Updates the estimator using a full 9-DoF sample.
    pub fn update_marg(&mut self, sample: MargSample, dt: MicrosDurationU32) -> ImuEstimate {
        self.attitude.update_marg(
            sample.accel_gyro.gyro_rad_s,
            sample.accel_gyro.accel_m_s2,
            sample.mag_body,
            dt,
        );
        self.integrate_linear_motion(sample.accel_gyro, dt)
    }

    /// Updates the estimator using only accelerometer and gyroscope data.
    pub fn update_imu(&mut self, sample: AccelGyroSample, dt: MicrosDurationU32) -> ImuEstimate {
        self.attitude
            .update_imu(sample.gyro_rad_s, sample.accel_m_s2, dt);
        self.integrate_linear_motion(sample, dt)
    }

    fn integrate_linear_motion(
        &mut self,
        sample: AccelGyroSample,
        dt: MicrosDurationU32,
    ) -> ImuEstimate {
        let accel_m_s2 = sample.accel_m_s2;
        let gyro_rad_s = sample.gyro_rad_s;
        self.navigator.pose.orientation = self.attitude.orientation();
        self.navigator.predict_imu(accel_m_s2, Vec3::ZERO, dt);
        self.navigator.pose.orientation = self.attitude.orientation();

        let accel_norm = accel_m_s2.length();
        let gyro_norm = gyro_rad_s.length();
        if fabsf(accel_norm - self.gravity_m_s2) < self.stationary.accel_tolerance_m_s2
            && gyro_norm < self.stationary.gyro_tolerance_rad_s
        {
            self.navigator.correct_velocity(
                Vec3::new(
                    self.navigator.velocity_world.x,
                    self.navigator.velocity_world.y,
                    0.0,
                ),
                self.stationary.zero_vertical_velocity_gain,
            );
        }

        let orientation = self.attitude.orientation();
        let (roll, pitch, yaw) = orientation.to_euler(glam::EulerRot::XYZ);
        ImuEstimate {
            euler: Attitude::new(roll, pitch, yaw),
            orientation,
            relative_altitude_m: self.navigator.pose.position.z,
            vertical_speed_m_s: self.navigator.velocity_world.z,
            velocity_world: self.navigator.velocity_world,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vector3;

    #[test]
    fn stationary_sample_keeps_relative_altitude_small() {
        let mut estimator = MargEstimator::new(0.08);
        let sample = MargSample::new(
            AccelGyroSample::without_temperature(Vector3::new(0.0, 0.0, 9.80665), Vector3::ZERO),
            Vector3::X,
        );
        let mut estimate = estimator.update_marg(sample, MicrosDurationU32::from_millis(10));
        for _ in 0..199 {
            estimate = estimator.update_marg(sample, MicrosDurationU32::from_millis(10));
        }

        assert!(estimate.relative_altitude_m.abs() < 1.0e-3);
        assert!(estimate.vertical_speed_m_s.abs() < 1.0e-3);
    }
}

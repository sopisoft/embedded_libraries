//! Complementary attitude filter using gravity and optional magnetometer fusion.

use fugit::MicrosDurationU32;
use glam::{EulerRot, Quat, Vec3};
use libm::{atan2f, sqrtf};

use crate::traits::AttitudeEstimator;

/// Complementary filter that blends integrated gyro attitude with gravity and magnetic heading.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ComplementaryAttitudeFilter {
    orientation: Quat,
    gain: f32,
}

impl ComplementaryAttitudeFilter {
    /// Creates a filter with a correction gain in `[0, 1]`.
    pub const fn new(gain: f32) -> Self {
        Self {
            orientation: Quat::IDENTITY,
            gain,
        }
    }

    /// Sets the orientation directly.
    pub fn set_orientation(&mut self, orientation: Quat) {
        self.orientation = orientation.normalize();
    }

    /// Updates the correction gain.
    pub fn set_gain(&mut self, gain: f32) {
        self.gain = gain.clamp(0.0, 1.0);
    }

    /// Updates the estimate with gyroscope, accelerometer, and magnetometer data.
    pub fn update_marg(&mut self, gyro_rad_s: Vec3, accel: Vec3, mag: Vec3, dt: MicrosDurationU32) {
        self.update_imu(gyro_rad_s, accel, dt);

        let accel_norm = accel.normalize_or_zero();
        let mag_norm = mag.normalize_or_zero();
        if accel_norm == Vec3::ZERO || mag_norm == Vec3::ZERO {
            return;
        }

        let roll = atan2f(accel_norm.y, accel_norm.z);
        let pitch = atan2f(
            -accel_norm.x,
            sqrtf(accel_norm.y * accel_norm.y + accel_norm.z * accel_norm.z),
        );

        let (sr, cr) = libm::sincosf(roll);
        let (sp, cp) = libm::sincosf(pitch);
        let mx2 = mag_norm.x * cp + mag_norm.z * sp;
        let my2 = mag_norm.x * sr * sp + mag_norm.y * cr - mag_norm.z * sr * cp;
        let yaw = atan2f(-my2, mx2);

        let target = Quat::from_euler(EulerRot::XYZ, roll, pitch, yaw);
        self.orientation = self.orientation.slerp(target, self.gain).normalize();
    }
}

impl AttitudeEstimator for ComplementaryAttitudeFilter {
    fn update_imu(&mut self, gyro_rad_s: Vec3, accel: Vec3, dt: MicrosDurationU32) {
        let dt = dt.as_secs_f32();
        self.orientation = (self.orientation * Quat::from_scaled_axis(gyro_rad_s * dt)).normalize();

        let accel_norm = accel.normalize_or_zero();
        if accel_norm == Vec3::ZERO {
            return;
        }

        let roll = atan2f(accel_norm.y, accel_norm.z);
        let pitch = atan2f(
            -accel_norm.x,
            sqrtf(accel_norm.y * accel_norm.y + accel_norm.z * accel_norm.z),
        );
        let (_, _, yaw) = self.orientation.to_euler(EulerRot::XYZ);
        let target = Quat::from_euler(EulerRot::XYZ, roll, pitch, yaw);
        self.orientation = self.orientation.slerp(target, self.gain).normalize();
    }

    fn orientation(&self) -> Quat {
        self.orientation
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fugit::MicrosDurationU32;
    use libm::fabsf;

    #[test]
    fn stationary_imu_keeps_identity() {
        let mut filter = ComplementaryAttitudeFilter::new(0.1);
        filter.update_imu(
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, 1.0),
            MicrosDurationU32::from_millis(10),
        );
        let q = filter.orientation();
        assert!(fabsf(q.w - 1.0) < 1.0e-6);
        assert!(fabsf(q.x) < 1.0e-6);
        assert!(fabsf(q.y) < 1.0e-6);
        assert!(fabsf(q.z) < 1.0e-6);
    }
}

//! Madgwick attitude estimator for IMU and MARG data.

use fugit::MicrosDurationU32;
use glam::{Quat, Vec3};
use libm::sqrtf;

/// Madgwick AHRS filter.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Madgwick {
    orientation: Quat,
    beta: f32,
}

impl Madgwick {
    /// Creates a filter with the given gradient-descent gain.
    pub const fn new(beta: f32) -> Self {
        Self {
            orientation: Quat::IDENTITY,
            beta,
        }
    }

    /// Returns the current orientation estimate.
    pub const fn orientation(&self) -> Quat {
        self.orientation
    }

    /// Sets the orientation directly.
    pub fn set_orientation(&mut self, orientation: Quat) {
        self.orientation = orientation.normalize();
    }

    /// Updates the filter gain.
    pub fn set_beta(&mut self, beta: f32) {
        self.beta = beta.max(0.0);
    }

    /// Updates the filter from gyroscope and accelerometer data.
    pub fn update_imu(&mut self, gyro_rad_s: Vec3, accel: Vec3, dt: MicrosDurationU32) {
        let dt = dt.as_secs_f32();
        let mut q0 = self.orientation.w;
        let mut q1 = self.orientation.x;
        let mut q2 = self.orientation.y;
        let mut q3 = self.orientation.z;

        let mut q_dot1 = 0.5 * (-q1 * gyro_rad_s.x - q2 * gyro_rad_s.y - q3 * gyro_rad_s.z);
        let mut q_dot2 = 0.5 * (q0 * gyro_rad_s.x + q2 * gyro_rad_s.z - q3 * gyro_rad_s.y);
        let mut q_dot3 = 0.5 * (q0 * gyro_rad_s.y - q1 * gyro_rad_s.z + q3 * gyro_rad_s.x);
        let mut q_dot4 = 0.5 * (q0 * gyro_rad_s.z + q1 * gyro_rad_s.y - q2 * gyro_rad_s.x);

        if accel != Vec3::ZERO {
            let recip_norm = inv_sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
            let ax = accel.x * recip_norm;
            let ay = accel.y * recip_norm;
            let az = accel.z * recip_norm;

            let _2q0 = 2.0 * q0;
            let _2q1 = 2.0 * q1;
            let _2q2 = 2.0 * q2;
            let _2q3 = 2.0 * q3;
            let _4q0 = 4.0 * q0;
            let _4q1 = 4.0 * q1;
            let _4q2 = 4.0 * q2;
            let _8q1 = 8.0 * q1;
            let _8q2 = 8.0 * q2;
            let q0q0 = q0 * q0;
            let q1q1 = q1 * q1;
            let q2q2 = q2 * q2;
            let q3q3 = q3 * q3;

            let mut s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            let mut s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1
                + _8q1 * q1q1
                + _8q1 * q2q2
                + _4q1 * az;
            let mut s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
                + _8q2 * q1q1
                + _8q2 * q2q2
                + _4q2 * az;
            let mut s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;

            let recip_norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= recip_norm;
            s1 *= recip_norm;
            s2 *= recip_norm;
            s3 *= recip_norm;

            q_dot1 -= self.beta * s0;
            q_dot2 -= self.beta * s1;
            q_dot3 -= self.beta * s2;
            q_dot4 -= self.beta * s3;
        }

        q0 += q_dot1 * dt;
        q1 += q_dot2 * dt;
        q2 += q_dot3 * dt;
        q3 += q_dot4 * dt;

        let recip_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        self.orientation = Quat::from_xyzw(
            q1 * recip_norm,
            q2 * recip_norm,
            q3 * recip_norm,
            q0 * recip_norm,
        );
    }

    /// Updates the filter from gyroscope, accelerometer, and magnetometer data.
    pub fn update_marg(&mut self, gyro_rad_s: Vec3, accel: Vec3, mag: Vec3, dt: MicrosDurationU32) {
        if mag == Vec3::ZERO {
            self.update_imu(gyro_rad_s, accel, dt);
            return;
        }

        let dt = dt.as_secs_f32();
        let mut q0 = self.orientation.w;
        let mut q1 = self.orientation.x;
        let mut q2 = self.orientation.y;
        let mut q3 = self.orientation.z;

        let mut q_dot1 = 0.5 * (-q1 * gyro_rad_s.x - q2 * gyro_rad_s.y - q3 * gyro_rad_s.z);
        let mut q_dot2 = 0.5 * (q0 * gyro_rad_s.x + q2 * gyro_rad_s.z - q3 * gyro_rad_s.y);
        let mut q_dot3 = 0.5 * (q0 * gyro_rad_s.y - q1 * gyro_rad_s.z + q3 * gyro_rad_s.x);
        let mut q_dot4 = 0.5 * (q0 * gyro_rad_s.z + q1 * gyro_rad_s.y - q2 * gyro_rad_s.x);

        if accel != Vec3::ZERO {
            let recip_norm = inv_sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
            let ax = accel.x * recip_norm;
            let ay = accel.y * recip_norm;
            let az = accel.z * recip_norm;

            let recip_norm = inv_sqrt(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
            let mx = mag.x * recip_norm;
            let my = mag.y * recip_norm;
            let mz = mag.z * recip_norm;

            let _2q0mx = 2.0 * q0 * mx;
            let _2q0my = 2.0 * q0 * my;
            let _2q0mz = 2.0 * q0 * mz;
            let _2q1mx = 2.0 * q1 * mx;
            let _2q0 = 2.0 * q0;
            let _2q1 = 2.0 * q1;
            let _2q2 = 2.0 * q2;
            let _2q3 = 2.0 * q3;
            let _2q0q2 = 2.0 * q0 * q2;
            let _2q2q3 = 2.0 * q2 * q3;
            let q0q0 = q0 * q0;
            let q0q1 = q0 * q1;
            let q0q2 = q0 * q2;
            let q0q3 = q0 * q3;
            let q1q1 = q1 * q1;
            let q1q2 = q1 * q2;
            let q1q3 = q1 * q3;
            let q2q2 = q2 * q2;
            let q2q3 = q2 * q3;
            let q3q3 = q3 * q3;

            let hx =
                mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3
                    - mx * q2q2
                    - mx * q3q3;
            let hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1
                + my * q2q2
                + _2q2 * mz * q3
                - my * q3q3;
            let _2bx = sqrtf(hx * hx + hy * hy);
            let _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1
                + _2q2 * my * q3
                - mz * q2q2
                + mz * q3q3;
            let _4bx = 2.0 * _2bx;
            let _4bz = 2.0 * _2bz;

            let mut s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay)
                - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
            let mut s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay)
                - 4.0 * q1 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az)
                + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                + (_2bx * q3 - _4bz * q1)
                    * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
            let mut s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay)
                - 4.0 * q2 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az)
                + (-_4bx * q2 - _2bz * q0)
                    * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                + (_2bx * q0 - _4bz * q2)
                    * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
            let mut s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax)
                + _2q2 * (2.0 * q0q1 + _2q2q3 - ay)
                + (-_4bx * q3 + _2bz * q1)
                    * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);

            let recip_norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= recip_norm;
            s1 *= recip_norm;
            s2 *= recip_norm;
            s3 *= recip_norm;

            q_dot1 -= self.beta * s0;
            q_dot2 -= self.beta * s1;
            q_dot3 -= self.beta * s2;
            q_dot4 -= self.beta * s3;
        }

        q0 += q_dot1 * dt;
        q1 += q_dot2 * dt;
        q2 += q_dot3 * dt;
        q3 += q_dot4 * dt;

        let recip_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        self.orientation = Quat::from_xyzw(
            q1 * recip_norm,
            q2 * recip_norm,
            q3 * recip_norm,
            q0 * recip_norm,
        );
    }
}

fn inv_sqrt(value: f32) -> f32 {
    if value > 0.0 { 1.0 / sqrtf(value) } else { 0.0 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fugit::MicrosDurationU32;
    use libm::fabsf;

    #[test]
    fn stationary_update_keeps_identity() {
        let mut filter = Madgwick::new(0.1);
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

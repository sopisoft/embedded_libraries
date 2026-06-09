use fugit::MicrosDurationU32;
use math::{EulerAngles, Matrix, Quat, Vec3, wrap_pi};

use super::Eskf;

impl Eskf {
    /// Runs the prediction step with IMU measurements.
    ///
    /// `accel_meas` is expected to be specific force.
    pub fn predict(&mut self, gyro_meas: Vec3, accel_meas: Vec3, dt: MicrosDurationU32) {
        let dt = dt.as_secs_f32();
        if dt <= 0.0 {
            return;
        }

        let omega = gyro_meas - self.gyro_bias;
        let accel_body = accel_meas - self.accel_bias;

        self.orientation = self.orientation.integrate_gyro(omega, dt);
        let accel_world = self.orientation.rotate_vec3(accel_body) + self.gravity;

        self.position += self.velocity * dt + accel_world * (0.5 * dt * dt);
        self.velocity += accel_world * dt;

        let f = Self::system_matrix(self.orientation, accel_body, dt);
        let q = Self::process_noise(
            self.accel_noise,
            self.gyro_noise,
            self.accel_bias_noise,
            self.gyro_bias_noise,
            dt,
        );
        self.covariance = (f * self.covariance * f.transpose()) + q;
        self.covariance = self.symmetrize(self.covariance);
    }

    /// Corrects the position state.
    pub fn correct_position(&mut self, measurement: Vec3, noise: f32) {
        let residual = measurement - self.position;
        self.correct_with_matrix(residual, Self::h_position(), noise, |state, delta| {
            state.position += Vec3::new(delta[0], delta[1], delta[2]);
        });
    }

    /// Corrects the velocity state.
    pub fn correct_velocity(&mut self, measurement: Vec3, noise: f32) {
        let residual = measurement - self.velocity;
        self.correct_with_matrix(residual, Self::h_velocity(), noise, |state, delta| {
            state.velocity += Vec3::new(delta[3], delta[4], delta[5]);
        });
    }

    /// Corrects altitude only.
    pub fn correct_altitude(&mut self, altitude_m: f32, noise: f32) {
        let residual = Vec3::new(0.0, 0.0, altitude_m - self.position.z);
        let h = Matrix::<3, 15>::from_fn(|r, c| if r == 2 && c == 2 { 1.0 } else { 0.0 });
        self.correct_with_matrix(residual, h, noise, |state, delta| {
            state.position.z += delta[2];
        });
    }

    /// Corrects the forward speed along the body X axis.
    pub fn correct_forward_speed(&mut self, speed_m_s: f32, noise: f32) {
        let body_velocity = self.orientation.rotate_inverse_vec3(self.velocity);
        let residual = Vec3::new(speed_m_s - body_velocity.x, 0.0, 0.0);
        let rotation = Self::rotation_matrix(self.orientation).transpose();
        let h = Matrix::<3, 15>::from_fn(|r, c| {
            if (3..6).contains(&c) {
                rotation.data[r][c - 3]
            } else {
                0.0
            }
        });
        self.correct_with_matrix(residual, h, noise, |state, delta| {
            state.velocity += Vec3::new(delta[3], delta[4], delta[5]);
        });
    }

    /// Corrects yaw while leaving roll and pitch untouched.
    pub fn correct_heading(&mut self, yaw_rad: f32, noise_rad: f32) {
        let current = self.orientation.to_euler();
        let yaw_error = wrap_pi(yaw_rad - current.yaw);
        self.correct_with_matrix(
            Vec3::new(0.0, 0.0, yaw_error),
            Self::h_orientation(),
            noise_rad,
            |state, delta| {
                let corrected = state.orientation
                    * Quat::from_small_angle(Vec3::new(delta[6], delta[7], delta[8]));
                let mut euler = corrected.to_euler();
                euler.roll = current.roll;
                euler.pitch = current.pitch;
                state.orientation =
                    Quat::from_euler(EulerAngles::new(euler.roll, euler.pitch, euler.yaw));
            },
        );
    }

    /// Corrects the full orientation state.
    pub fn correct_orientation(&mut self, measurement: Quat, noise_rad: f32) {
        let q_err = measurement * self.orientation.conjugate();
        let sign = if q_err.w < 0.0 { -1.0 } else { 1.0 };
        let residual = Vec3::new(
            2.0 * sign * q_err.x,
            2.0 * sign * q_err.y,
            2.0 * sign * q_err.z,
        );
        self.correct_with_matrix(
            residual,
            Self::h_orientation(),
            noise_rad,
            |state, delta| {
                state.orientation = (state.orientation
                    * Quat::from_small_angle(Vec3::new(delta[6], delta[7], delta[8])))
                .normalized();
            },
        );
    }

    fn correct_with_matrix<F>(
        &mut self,
        residual: Vec3,
        h: Matrix<3, 15>,
        noise: f32,
        apply_delta: F,
    ) where
        F: FnOnce(&mut Self, [f32; 15]),
    {
        let r = Matrix::<3, 3>::from_diagonal([noise * noise; 3]);
        let s = (h * self.covariance * h.transpose()) + r;
        let Some(s_inv) = s.inverse() else {
            return;
        };

        let k = self.covariance * h.transpose() * s_inv;
        let delta = k * [residual.x, residual.y, residual.z];
        apply_delta(self, delta);

        let i = Matrix::<15, 15>::identity();
        let kh = k * h;
        self.covariance = (i - kh) * self.covariance * (i - kh).transpose() + k * r * k.transpose();
        self.covariance = self.symmetrize(self.covariance);
    }

    fn h_position() -> Matrix<3, 15> {
        Matrix::from_fn(|r, c| if r == c { 1.0 } else { 0.0 })
    }

    fn h_velocity() -> Matrix<3, 15> {
        Matrix::from_fn(|r, c| if c == 3 + r { 1.0 } else { 0.0 })
    }

    fn h_orientation() -> Matrix<3, 15> {
        Matrix::from_fn(|r, c| if c == 6 + r { 1.0 } else { 0.0 })
    }
}

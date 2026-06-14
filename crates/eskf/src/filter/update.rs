use fugit::MicrosDurationU32;
use glam::{EulerRot, Quat, Vec3};

use super::{Eskf, Matrix, Vector};

fn wrap_pi(angle_rad: f32) -> f32 {
    let mut wrapped = angle_rad;
    while wrapped > core::f32::consts::PI {
        wrapped -= core::f32::consts::TAU;
    }
    while wrapped < -core::f32::consts::PI {
        wrapped += core::f32::consts::TAU;
    }
    wrapped
}

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

        self.orientation = (self.orientation * Quat::from_scaled_axis(omega * dt)).normalize();
        let accel_world = self.orientation.mul_vec3(accel_body) + self.gravity;

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
        let mut h = Matrix::<3, 15>::zeros();
        h[(2, 2)] = 1.0;
        self.correct_with_matrix(residual, h, noise, |state, delta| {
            state.position.z += delta[2];
        });
    }

    /// Corrects the forward speed along the body X axis.
    pub fn correct_forward_speed(&mut self, speed_m_s: f32, noise: f32) {
        let body_velocity = self.orientation.conjugate().mul_vec3(self.velocity);
        let residual = Vec3::new(speed_m_s - body_velocity.x, 0.0, 0.0);
        let rotation = Self::rotation_matrix(self.orientation).transpose();
        let mut h = Matrix::<3, 15>::zeros();
        let mut row = 0usize;
        while row < 3 {
            let mut col = 0usize;
            while col < 3 {
                h[(row, 3 + col)] = rotation[(row, col)];
                col += 1;
            }
            row += 1;
        }
        self.correct_with_matrix(residual, h, noise, |state, delta| {
            state.velocity += Vec3::new(delta[3], delta[4], delta[5]);
        });
    }

    /// Corrects yaw while leaving roll and pitch untouched.
    pub fn correct_heading(&mut self, yaw_rad: f32, noise_rad: f32) {
        let (current_roll, current_pitch, current_yaw) = self.orientation.to_euler(EulerRot::XYZ);
        let yaw_error = wrap_pi(yaw_rad - current_yaw);
        self.correct_with_matrix(
            Vec3::new(0.0, 0.0, yaw_error),
            Self::h_orientation(),
            noise_rad,
            |state, delta| {
                let corrected = (state.orientation
                    * Quat::from_scaled_axis(Vec3::new(delta[6], delta[7], delta[8])))
                .normalize();
                let (_, _, corrected_yaw) = corrected.to_euler(EulerRot::XYZ);
                state.orientation =
                    Quat::from_euler(EulerRot::XYZ, current_roll, current_pitch, corrected_yaw)
                        .normalize();
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
                    * Quat::from_scaled_axis(Vec3::new(delta[6], delta[7], delta[8])))
                .normalize();
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
        F: FnOnce(&mut Self, &Vector<15>),
    {
        let r = Matrix::<3, 3>::identity() * (noise * noise);
        let s = (h * self.covariance * h.transpose()) + r;
        let Some(s_inv) = s.try_inverse() else {
            return;
        };

        let k = self.covariance * h.transpose() * s_inv;
        let delta = k * Vector::<3>::new(residual.x, residual.y, residual.z);
        apply_delta(self, &delta);

        let i = Matrix::<15, 15>::identity();
        let kh = k * h;
        self.covariance = (i - kh) * self.covariance * (i - kh).transpose() + k * r * k.transpose();
        self.covariance = self.symmetrize(self.covariance);
    }

    fn h_position() -> Matrix<3, 15> {
        let mut h = Matrix::<3, 15>::zeros();
        let mut axis = 0usize;
        while axis < 3 {
            h[(axis, axis)] = 1.0;
            axis += 1;
        }
        h
    }

    fn h_velocity() -> Matrix<3, 15> {
        let mut h = Matrix::<3, 15>::zeros();
        let mut axis = 0usize;
        while axis < 3 {
            h[(axis, 3 + axis)] = 1.0;
            axis += 1;
        }
        h
    }

    fn h_orientation() -> Matrix<3, 15> {
        let mut h = Matrix::<3, 15>::zeros();
        let mut axis = 0usize;
        while axis < 3 {
            h[(axis, 6 + axis)] = 1.0;
            axis += 1;
        }
        h
    }
}

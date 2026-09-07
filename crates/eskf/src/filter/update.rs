use fugit::MicrosDurationU32;
use glam::{EulerRot, Mat3, Quat, Vec3};

use super::Eskf;
use super::covariance::ProcessNoise;
use super::math::{outer, wrap_pi};

const VELOCITY_BLOCK: usize = 0;
const ATTITUDE_BLOCK: usize = 1;
const ACCEL_BIAS_BLOCK: usize = 2;
const GYRO_BIAS_BLOCK: usize = 3;
const BLOCK_COUNT: usize = 4;

impl Eskf {
    /// Runs the prediction step with IMU measurements.
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

        self.velocity += accel_world * dt;

        self.covariance = self.covariance.predict(
            self.orientation,
            accel_body,
            ProcessNoise {
                accel: self.accel_noise,
                gyro: self.gyro_noise,
                accel_bias: self.accel_bias_noise,
                gyro_bias: self.gyro_bias_noise,
            },
            dt,
        );
    }

    /// Corrects the velocity state.
    pub fn correct_velocity(&mut self, measurement: Vec3, noise: f32) {
        let residual = measurement - self.velocity;
        self.correct_block_vector(residual, VELOCITY_BLOCK, noise);
    }

    /// Corrects vertical speed only.
    pub fn correct_vertical_velocity(&mut self, vertical_speed_m_s: f32, noise: f32) {
        let residual = vertical_speed_m_s - self.velocity.z;
        self.correct_block_scalar_direction(residual, VELOCITY_BLOCK, Vec3::Z, noise);
    }

    /// Corrects the forward speed along the body X axis.
    pub fn correct_forward_speed(&mut self, speed_m_s: f32, noise: f32) {
        let forward_axis_world = self.orientation.mul_vec3(Vec3::X);
        let residual = speed_m_s - forward_axis_world.dot(self.velocity);
        self.correct_block_scalar_direction(residual, VELOCITY_BLOCK, forward_axis_world, noise);
    }

    /// Corrects yaw while leaving roll and pitch untouched.
    pub fn correct_heading(&mut self, yaw_rad: f32, noise_rad: f32) {
        let (_, _, current_yaw) = self.orientation.to_euler(EulerRot::XYZ);
        let yaw_error = wrap_pi(yaw_rad - current_yaw);
        let measurement = (Quat::from_rotation_z(yaw_error) * self.orientation).normalize();
        self.correct_orientation(measurement, noise_rad);
    }

    /// Corrects the full orientation state.
    pub fn correct_orientation(&mut self, measurement: Quat, noise_rad: f32) {
        let q_err = self.orientation.conjugate() * measurement;
        let sign = if q_err.w < 0.0 { -1.0 } else { 1.0 };
        let residual = Vec3::new(
            2.0 * sign * q_err.x,
            2.0 * sign * q_err.y,
            2.0 * sign * q_err.z,
        );
        self.correct_block_vector(residual, ATTITUDE_BLOCK, noise_rad);
    }

    fn correct_block_vector(&mut self, residual: Vec3, measured_block: usize, noise: f32) {
        let prior = self.covariance;
        let innovation_covariance =
            prior.block(measured_block, measured_block) + (Mat3::IDENTITY * (noise * noise));
        let determinant = innovation_covariance.determinant();
        if determinant.abs() <= 1.0e-9 {
            return;
        }
        let innovation_covariance_inv = innovation_covariance.inverse();

        let mut gains = [Mat3::ZERO; BLOCK_COUNT];
        let mut delta = [Vec3::ZERO; BLOCK_COUNT];
        let mut block = 0usize;
        while block < BLOCK_COUNT {
            gains[block] = prior.block(block, measured_block) * innovation_covariance_inv;
            delta[block] = gains[block] * residual;
            block += 1;
        }

        self.apply_state_delta(delta);
        self.update_covariance_vector(prior, gains, innovation_covariance);
    }

    fn correct_block_scalar_direction(
        &mut self,
        residual: f32,
        measured_block: usize,
        direction: Vec3,
        noise: f32,
    ) {
        let prior = self.covariance;
        let projected_covariance = prior.block(measured_block, measured_block) * direction;
        let innovation_variance = direction.dot(projected_covariance) + (noise * noise);
        if innovation_variance <= 1.0e-9 {
            return;
        }

        let inv_innovation_variance = innovation_variance.recip();
        let mut gains = [Vec3::ZERO; BLOCK_COUNT];
        let mut delta = [Vec3::ZERO; BLOCK_COUNT];
        let mut block = 0usize;
        while block < BLOCK_COUNT {
            gains[block] =
                (prior.block(block, measured_block) * direction) * inv_innovation_variance;
            delta[block] = gains[block] * residual;
            block += 1;
        }

        self.apply_state_delta(delta);
        self.update_covariance_scalar(prior, gains, innovation_variance);
    }

    fn apply_state_delta(&mut self, delta: [Vec3; BLOCK_COUNT]) {
        self.velocity += delta[VELOCITY_BLOCK];
        self.orientation =
            (self.orientation * Quat::from_scaled_axis(delta[ATTITUDE_BLOCK])).normalize();
        self.accel_bias += delta[ACCEL_BIAS_BLOCK];
        self.gyro_bias += delta[GYRO_BIAS_BLOCK];
    }

    fn update_covariance_vector(
        &mut self,
        prior: super::Covariance,
        gains: [Mat3; BLOCK_COUNT],
        innovation_covariance: Mat3,
    ) {
        let mut posterior = prior;
        let mut row_block = 0usize;
        while row_block < BLOCK_COUNT {
            let mut col_block = row_block;
            while col_block < BLOCK_COUNT {
                let updated = prior.block(row_block, col_block)
                    - (gains[row_block] * innovation_covariance * gains[col_block].transpose());
                posterior.set_symmetric_block(row_block, col_block, updated);
                col_block += 1;
            }
            row_block += 1;
        }
        posterior.symmetrize();
        self.covariance = posterior;
    }

    fn update_covariance_scalar(
        &mut self,
        prior: super::Covariance,
        gains: [Vec3; BLOCK_COUNT],
        innovation_variance: f32,
    ) {
        let mut posterior = prior;
        let mut row_block = 0usize;
        while row_block < BLOCK_COUNT {
            let mut col_block = row_block;
            while col_block < BLOCK_COUNT {
                let updated = prior.block(row_block, col_block)
                    - (outer(gains[row_block], gains[col_block]) * innovation_variance);
                posterior.set_symmetric_block(row_block, col_block, updated);
                col_block += 1;
            }
            row_block += 1;
        }
        posterior.symmetrize();
        self.covariance = posterior;
    }
}

//! General-purpose inertial navigation with lightweight measurement correction.

use fugit::MicrosDurationU32;
use glam::{EulerRot, Quat, Vec3};
use kinematics::Pose3;

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

/// A lightweight dead-reckoning navigator with complementary correction hooks.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct InertialNavigator {
    /// Current pose estimate.
    pub pose: Pose3,
    /// Current world-frame velocity estimate.
    pub velocity_world: Vec3,
    /// Gravity expressed in the world frame.
    pub gravity_world: Vec3,
}

impl InertialNavigator {
    /// Creates a navigator with zero pose and ENU-style gravity.
    pub const fn new() -> Self {
        Self {
            pose: Pose3::identity(),
            velocity_world: Vec3::ZERO,
            gravity_world: Vec3::new(0.0, 0.0, -9.80665),
        }
    }

    /// Integrates IMU data where accelerometer samples are specific force.
    pub fn predict_imu(&mut self, accel_body: Vec3, gyro_rad_s: Vec3, dt: MicrosDurationU32) {
        let dt = dt.as_secs_f32();
        self.pose.orientation =
            (self.pose.orientation * Quat::from_scaled_axis(gyro_rad_s * dt)).normalize();
        let accel_world = self.pose.orientation.mul_vec3(accel_body) + self.gravity_world;
        self.pose.position += self.velocity_world * dt + accel_world * (0.5 * dt * dt);
        self.velocity_world += accel_world * dt;
    }

    /// Integrates an externally-estimated world-frame velocity.
    pub fn predict_world_velocity(
        &mut self,
        velocity_world: Vec3,
        gyro_rad_s: Vec3,
        dt: MicrosDurationU32,
    ) {
        let dt = dt.as_secs_f32();
        self.pose.orientation =
            (self.pose.orientation * Quat::from_scaled_axis(gyro_rad_s * dt)).normalize();
        self.pose.position += velocity_world * dt;
        self.velocity_world = velocity_world;
    }

    /// Blends the position estimate toward a measurement.
    pub fn correct_position(&mut self, position_world: Vec3, gain: f32) {
        let gain = gain.clamp(0.0, 1.0);
        self.pose.position = self.pose.position.lerp(position_world, gain);
    }

    /// Blends the velocity estimate toward a measurement.
    pub fn correct_velocity(&mut self, velocity_world: Vec3, gain: f32) {
        let gain = gain.clamp(0.0, 1.0);
        self.velocity_world = self.velocity_world.lerp(velocity_world, gain);
    }

    /// Blends altitude toward a measurement.
    pub fn correct_altitude(&mut self, altitude_m: f32, gain: f32) {
        let gain = gain.clamp(0.0, 1.0);
        self.pose.position.z += (altitude_m - self.pose.position.z) * gain;
    }

    /// Corrects yaw without affecting roll or pitch.
    pub fn correct_heading(&mut self, yaw_rad: f32, gain: f32) {
        let gain = gain.clamp(0.0, 1.0);
        let (roll, pitch, yaw) = self.pose.orientation.to_euler(EulerRot::XYZ);
        let corrected_yaw = wrap_pi(yaw + wrap_pi(yaw_rad - yaw) * gain);
        self.pose.orientation = Quat::from_euler(EulerRot::XYZ, roll, pitch, corrected_yaw);
    }

    /// Corrects the body-X speed component, useful for wheel odometry or airspeed.
    pub fn correct_forward_speed(&mut self, forward_speed_m_s: f32, gain: f32) {
        let gain = gain.clamp(0.0, 1.0);
        let body_velocity = self
            .pose
            .orientation
            .conjugate()
            .mul_vec3(self.velocity_world);
        let corrected = Vec3::new(
            body_velocity.x + (forward_speed_m_s - body_velocity.x) * gain,
            body_velocity.y,
            body_velocity.z,
        );
        self.velocity_world = self.pose.orientation.mul_vec3(corrected);
    }
}

impl Default for InertialNavigator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fugit::MicrosDurationU32;
    use libm::fabsf;

    #[test]
    fn imu_prediction_stays_still_for_gravity_only() {
        let mut nav = InertialNavigator::new();
        nav.predict_imu(
            Vec3::new(0.0, 0.0, 9.80665),
            Vec3::ZERO,
            MicrosDurationU32::from_secs(1),
        );
        assert!(fabsf(nav.pose.position.z) < 1.0e-6);
        assert!(fabsf(nav.velocity_world.z) < 1.0e-6);
    }

    #[test]
    fn position_correction_moves_estimate() {
        let mut nav = InertialNavigator::new();
        nav.correct_position(Vec3::new(10.0, 0.0, 0.0), 0.5);
        assert!(fabsf(nav.pose.position.x - 5.0) < 1.0e-6);
    }

    #[test]
    fn forward_speed_correction_updates_world_velocity() {
        let mut nav = InertialNavigator::new();
        nav.correct_forward_speed(20.0, 1.0);
        assert!(fabsf(nav.velocity_world.x - 20.0) < 1.0e-6);
    }
}

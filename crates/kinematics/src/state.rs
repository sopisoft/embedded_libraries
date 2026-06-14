//! Generic motion state integration.

use fugit::MicrosDurationU32;
use glam::{Quat, Vec2, Vec3};
use libm::sincosf;

use crate::{Pose2, Pose3, Twist2, Twist3};

/// Planar position, velocity, and yaw-rate state.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MotionState2 {
    /// Pose in the world frame.
    pub pose: Pose2,
    /// Linear velocity in the world frame.
    pub velocity: Vec2,
    /// Yaw rate in radians per second.
    pub angular_velocity: f32,
}

impl MotionState2 {
    /// Creates a new state from a pose.
    pub const fn new(pose: Pose2) -> Self {
        Self {
            pose,
            velocity: Vec2::ZERO,
            angular_velocity: 0.0,
        }
    }

    /// Integrates a body-frame twist.
    pub fn step_twist(&mut self, twist: Twist2, dt: MicrosDurationU32) {
        let dt = dt.as_secs_f32();
        self.pose.integrate_twist(twist, dt);
        let (s, c) = sincosf(self.pose.heading);
        self.velocity = Vec2::new(
            c * twist.linear.x - s * twist.linear.y,
            s * twist.linear.x + c * twist.linear.y,
        );
        self.angular_velocity = twist.angular;
    }
}

/// Spatial position, velocity, and angular-rate state.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MotionState3 {
    /// Pose in the world frame.
    pub pose: Pose3,
    /// Linear velocity in the world frame.
    pub velocity: Vec3,
    /// Angular velocity in the body frame.
    pub angular_velocity: Vec3,
}

impl MotionState3 {
    /// Creates a new state from a pose.
    pub const fn new(pose: Pose3) -> Self {
        Self {
            pose,
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
        }
    }

    /// Integrates IMU data using specific force and body-frame angular rate.
    pub fn step_imu(
        &mut self,
        accel_body: Vec3,
        gyro_rad_s: Vec3,
        gravity_world: Vec3,
        dt: MicrosDurationU32,
    ) {
        let dt = dt.as_secs_f32();
        self.pose.orientation =
            (self.pose.orientation * Quat::from_scaled_axis(gyro_rad_s * dt)).normalize();
        let accel_world = self.pose.orientation.mul_vec3(accel_body) + gravity_world;
        self.pose.position += self.velocity * dt + accel_world * (0.5 * dt * dt);
        self.velocity += accel_world * dt;
        self.angular_velocity = gyro_rad_s;
    }

    /// Integrates a body-frame twist.
    pub fn step_twist(&mut self, twist: Twist3, dt: MicrosDurationU32) {
        let dt = dt.as_secs_f32();
        self.pose.integrate_twist(twist, dt);
        self.velocity = self.pose.orientation.mul_vec3(twist.linear);
        self.angular_velocity = twist.angular;
    }
}

/// Alias for planar motion state.
pub type PlanarMotion = MotionState2;

/// Alias for 3D motion state.
pub type SpatialMotion = MotionState3;

#[cfg(test)]
mod tests {
    use super::*;
    use fugit::MicrosDurationU32;
    use libm::fabsf;

    #[test]
    fn planar_state_updates() {
        let mut state = MotionState2::new(Pose2::identity());
        state.step_twist(
            Twist2::new(Vec2::new(1.0, 0.0), 0.0),
            MicrosDurationU32::from_secs(1),
        );
        assert!(fabsf(state.pose.position.x - 1.0) < 1.0e-6);
        assert!(fabsf(state.velocity.x - 1.0) < 1.0e-6);
    }

    #[test]
    fn spatial_state_updates_without_accel() {
        let mut state = MotionState3::new(Pose3::new(Vec3::ZERO, Quat::IDENTITY));
        state.step_imu(
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::ZERO,
            MicrosDurationU32::from_secs(1),
        );
        assert!(fabsf(state.pose.position.x) < 1.0e-6);
        assert!(fabsf(state.pose.position.y) < 1.0e-6);
        assert!(fabsf(state.pose.position.z) < 1.0e-6);
    }

    #[test]
    fn spatial_twist_updates() {
        let mut state = MotionState3::new(Pose3::identity());
        state.step_twist(
            Twist3::new(Vec3::new(1.0, 0.0, 0.0), Vec3::ZERO),
            MicrosDurationU32::from_secs(1),
        );
        assert!(fabsf(state.pose.position.x - 1.0) < 1.0e-6);
    }
}

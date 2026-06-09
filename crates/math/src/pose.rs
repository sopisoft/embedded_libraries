//! Pose and twist types.

use libm::sincosf;

use crate::{
    angle::wrap_pi,
    quat::Quat,
    vector::{Vec2, Vec3},
};

/// A planar body twist.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Twist2 {
    /// Linear velocity expressed in the body frame.
    pub linear: Vec2,
    /// Yaw rate in radians per second.
    pub angular: f32,
}

impl Twist2 {
    /// Creates a new planar twist.
    pub const fn new(linear: Vec2, angular: f32) -> Self {
        Self { linear, angular }
    }
}

/// A planar pose.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Pose2 {
    /// Position in the world frame.
    pub position: Vec2,
    /// Heading angle in radians.
    pub heading: f32,
}

impl Pose2 {
    /// Creates a new planar pose.
    pub const fn new(position: Vec2, heading: f32) -> Self {
        Self { position, heading }
    }

    /// Returns the identity pose.
    pub const fn identity() -> Self {
        Self::new(Vec2::zero(), 0.0)
    }

    /// Integrates a body-frame twist.
    pub fn integrate_twist(&mut self, twist: Twist2, dt: f32) {
        let heading_mid = self.heading + twist.angular * dt * 0.5;
        let (s, c) = sincosf(heading_mid);
        let world_linear = Vec2::new(
            c * twist.linear.x - s * twist.linear.y,
            s * twist.linear.x + c * twist.linear.y,
        );
        self.position += world_linear * dt;
        self.heading = wrap_pi(self.heading + twist.angular * dt);
    }
}

/// A spatial body twist.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Twist3 {
    /// Linear velocity expressed in the body frame.
    pub linear: Vec3,
    /// Angular velocity in radians per second.
    pub angular: Vec3,
}

impl Twist3 {
    /// Creates a new spatial twist.
    pub const fn new(linear: Vec3, angular: Vec3) -> Self {
        Self { linear, angular }
    }
}

/// A 3D pose.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Pose3 {
    /// Position in the world frame.
    pub position: Vec3,
    /// Orientation as a unit quaternion.
    pub orientation: Quat,
}

impl Pose3 {
    /// Creates a new pose.
    pub const fn new(position: Vec3, orientation: Quat) -> Self {
        Self {
            position,
            orientation,
        }
    }

    /// Returns the identity pose.
    pub const fn identity() -> Self {
        Self::new(Vec3::zero(), Quat::identity())
    }

    /// Integrates body linear and angular velocity.
    pub fn integrate_twist(&mut self, twist: Twist3, dt: f32) {
        self.position += self.orientation.rotate_vec3(twist.linear) * dt;
        self.orientation = self.orientation.integrate_gyro(twist.angular, dt);
    }
}

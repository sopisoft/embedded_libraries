//! Pose and twist types built on `glam`.

use glam::{Quat, Vec2, Vec3};
use libm::sincosf;

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

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Twist2 {
    pub linear: Vec2,
    pub angular: f32,
}

impl Twist2 {
    pub const fn new(linear: Vec2, angular: f32) -> Self {
        Self { linear, angular }
    }
}

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Pose2 {
    pub position: Vec2,
    pub heading: f32,
}

impl Pose2 {
    pub const fn new(position: Vec2, heading: f32) -> Self {
        Self { position, heading }
    }

    pub const fn identity() -> Self {
        Self::new(Vec2::ZERO, 0.0)
    }

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

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Twist3 {
    pub linear: Vec3,
    pub angular: Vec3,
}

impl Twist3 {
    pub const fn new(linear: Vec3, angular: Vec3) -> Self {
        Self { linear, angular }
    }
}

#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Pose3 {
    pub position: Vec3,
    pub orientation: Quat,
}

impl Pose3 {
    pub const fn new(position: Vec3, orientation: Quat) -> Self {
        Self {
            position,
            orientation,
        }
    }

    pub const fn identity() -> Self {
        Self::new(Vec3::ZERO, Quat::IDENTITY)
    }

    pub fn integrate_twist(&mut self, twist: Twist3, dt: f32) {
        self.position += self.orientation.mul_vec3(twist.linear) * dt;
        self.orientation =
            (self.orientation * Quat::from_scaled_axis(twist.angular * dt)).normalize();
    }
}

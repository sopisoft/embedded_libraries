//! Quaternion and Euler-angle helpers.

use core::ops::Mul;
use libm::{acosf, asinf, atan2f, cosf, fabsf, sincosf, sinf, sqrtf};

use crate::{
    angle::{deg_to_rad, rad_to_deg},
    vector::Vec3,
};

/// Roll, pitch, and yaw in radians.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct EulerAngles {
    /// Rotation around the X axis.
    pub roll: f32,
    /// Rotation around the Y axis.
    pub pitch: f32,
    /// Rotation around the Z axis.
    pub yaw: f32,
}

impl EulerAngles {
    /// Creates a new set of Euler angles.
    pub const fn new(roll: f32, pitch: f32, yaw: f32) -> Self {
        Self { roll, pitch, yaw }
    }

    /// Creates angles from degree values.
    pub fn from_degrees(roll: f32, pitch: f32, yaw: f32) -> Self {
        Self::new(deg_to_rad(roll), deg_to_rad(pitch), deg_to_rad(yaw))
    }

    /// Converts the angles to degrees.
    pub fn to_degrees(self) -> Self {
        Self::new(
            rad_to_deg(self.roll),
            rad_to_deg(self.pitch),
            rad_to_deg(self.yaw),
        )
    }
}

/// Right-handed unit quaternion.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Quat {
    /// Scalar part.
    pub w: f32,
    /// X component.
    pub x: f32,
    /// Y component.
    pub y: f32,
    /// Z component.
    pub z: f32,
}

impl Quat {
    /// Creates a new quaternion.
    pub const fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    /// Returns the identity quaternion.
    pub const fn identity() -> Self {
        Self::new(1.0, 0.0, 0.0, 0.0)
    }

    /// Returns the squared norm.
    pub fn norm_squared(self) -> f32 {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Returns the norm.
    pub fn norm(self) -> f32 {
        sqrtf(self.norm_squared())
    }

    /// Returns a normalized quaternion.
    pub fn normalized(self) -> Self {
        let n = self.norm();
        if n > 0.0 {
            Self::new(self.w / n, self.x / n, self.y / n, self.z / n)
        } else {
            Self::identity()
        }
    }

    /// Returns the conjugate.
    pub const fn conjugate(self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }

    /// Returns the inverse quaternion.
    pub fn inverse(self) -> Self {
        let n2 = self.norm_squared();
        if n2 > 0.0 {
            self.conjugate() / n2
        } else {
            Self::identity()
        }
    }

    /// Creates a quaternion from an axis-angle pair.
    pub fn from_axis_angle(axis: Vec3, angle: f32) -> Self {
        let axis = axis.normalized();
        let half = angle * 0.5;
        let (s, c) = sincosf(half);
        Self::new(c, axis.x * s, axis.y * s, axis.z * s).normalized()
    }

    /// Creates a quaternion from a small-angle rotation vector.
    pub fn from_small_angle(delta: Vec3) -> Self {
        let angle = delta.norm();
        if angle > 1.0e-12 {
            Self::from_axis_angle(delta / angle, angle)
        } else {
            Self::new(1.0, delta.x * 0.5, delta.y * 0.5, delta.z * 0.5).normalized()
        }
    }

    /// Creates a quaternion from Euler angles.
    pub fn from_euler(euler: EulerAngles) -> Self {
        let (sr, cr) = sincosf(euler.roll * 0.5);
        let (sp, cp) = sincosf(euler.pitch * 0.5);
        let (sy, cy) = sincosf(euler.yaw * 0.5);

        Self::new(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
        .normalized()
    }

    /// Returns Euler angles in radians.
    pub fn to_euler(self) -> EulerAngles {
        let q = self.normalized();

        let sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
        let cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        let roll = atan2f(sinr_cosp, cosr_cosp);

        let sinp = 2.0 * (q.w * q.y - q.z * q.x);
        let pitch = asinf(sinp.clamp(-1.0, 1.0));

        let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        let yaw = atan2f(siny_cosp, cosy_cosp);

        EulerAngles::new(roll, pitch, yaw)
    }

    /// Rotates a vector by the quaternion.
    pub fn rotate_vec3(self, v: Vec3) -> Vec3 {
        let qv = Vec3::new(self.x, self.y, self.z);
        let t = qv.cross(v) * 2.0;
        v + t * self.w + qv.cross(t)
    }

    /// Rotates a vector by the inverse quaternion.
    pub fn rotate_inverse_vec3(self, v: Vec3) -> Vec3 {
        self.conjugate().rotate_vec3(v)
    }

    /// Integrates angular velocity in radians per second.
    pub fn integrate_gyro(self, omega: Vec3, dt: f32) -> Self {
        let delta = Vec3::new(omega.x * dt, omega.y * dt, omega.z * dt);
        (self * Self::from_small_angle(delta)).normalized()
    }

    /// Spherical linear interpolation.
    pub fn slerp(self, other: Self, t: f32) -> Self {
        let mut end = other;
        let mut dot = self.w * end.w + self.x * end.x + self.y * end.y + self.z * end.z;

        if dot < 0.0 {
            dot = -dot;
            end = -end;
        }

        if dot > 0.9995 {
            return (self + (end - self) * t).normalized();
        }

        let theta_0 = acosf(dot.clamp(-1.0, 1.0));
        let theta = theta_0 * t;
        let sin_theta = sinf(theta);
        let sin_theta_0 = sinf(theta_0);

        if fabsf(sin_theta_0) < f32::EPSILON {
            return self;
        }

        let s0 = cosf(theta) - dot * sin_theta / sin_theta_0;
        let s1 = sin_theta / sin_theta_0;
        (self * s0 + end * s1).normalized()
    }
}

impl core::ops::Add for Quat {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(
            self.w + rhs.w,
            self.x + rhs.x,
            self.y + rhs.y,
            self.z + rhs.z,
        )
    }
}

impl core::ops::Sub for Quat {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(
            self.w - rhs.w,
            self.x - rhs.x,
            self.y - rhs.y,
            self.z - rhs.z,
        )
    }
}

impl core::ops::Mul<f32> for Quat {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self::new(self.w * rhs, self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl core::ops::Div<f32> for Quat {
    type Output = Self;

    fn div(self, rhs: f32) -> Self::Output {
        Self::new(self.w / rhs, self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl core::ops::Neg for Quat {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self::new(-self.w, -self.x, -self.y, -self.z)
    }
}

impl Mul for Quat {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self::new(
            self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        )
    }
}

#![no_std]

//! Lightweight `no_std` math primitives for embedded robotics and control code.
//!
//! This crate intentionally stays small and depends only on `libm`.

#[cfg(test)]
extern crate std;

pub mod angle;
pub mod linalg;
pub mod pose;
pub mod quat;
pub mod vector;

pub use angle::{Angle, TAU, deg_to_rad, rad_to_deg, wrap_pi, wrap_tau};
pub use linalg::Matrix;
pub use pose::{Pose2, Pose3, Twist2, Twist3};
pub use quat::{EulerAngles, Quat};
pub use vector::{Vec2, Vec3};

#[cfg(test)]
mod tests {
    use super::*;
    use libm::fabsf;

    fn approx(a: f32, b: f32) -> bool {
        fabsf(a - b) < 1.0e-4
    }

    #[test]
    fn angle_helpers_work() {
        assert!(approx(deg_to_rad(180.0), core::f32::consts::PI));
        assert!(approx(rad_to_deg(core::f32::consts::PI), 180.0));
        assert!(approx(
            wrap_pi(3.0 * core::f32::consts::PI),
            -core::f32::consts::PI
        ));
        assert!(approx(
            Angle::from_degrees(90.0).radians(),
            core::f32::consts::FRAC_PI_2
        ));
    }

    #[test]
    fn vector_cross_and_norm_work() {
        let v = Vec3::new(1.0, 0.0, 0.0).cross(Vec3::new(0.0, 1.0, 0.0));
        assert_eq!(v, Vec3::new(0.0, 0.0, 1.0));
        assert!(approx(Vec3::new(3.0, 4.0, 0.0).norm(), 5.0));
    }

    #[test]
    fn quaternion_roundtrip_work() {
        let e = EulerAngles::new(0.2, -0.4, 0.6);
        let q = Quat::from_euler(e);
        let e2 = q.to_euler();
        assert!(approx(e.roll, e2.roll));
        assert!(approx(e.pitch, e2.pitch));
        assert!(approx(e.yaw, e2.yaw));
    }

    #[test]
    fn matrix_inverse_works() {
        let m = Matrix::<3, 3>::new([[4.0, 7.0, 2.0], [3.0, 6.0, 1.0], [2.0, 5.0, 1.0]]);
        let inv = m.inverse().unwrap();
        let id = m * inv;
        let mut r = 0;
        while r < 3 {
            let mut c = 0;
            while c < 3 {
                if r == c {
                    assert!(approx(id.data[r][c], 1.0));
                } else {
                    assert!(fabsf(id.data[r][c]) < 1.0e-3);
                }
                c += 1;
            }
            r += 1;
        }
    }

    #[test]
    fn pose_integration_works() {
        let mut pose = Pose2::identity();
        pose.integrate_twist(Twist2::new(Vec2::new(1.0, 0.0), 0.0), 1.0);
        assert!(approx(pose.position.x, 1.0));
        assert!(approx(pose.position.y, 0.0));
    }
}

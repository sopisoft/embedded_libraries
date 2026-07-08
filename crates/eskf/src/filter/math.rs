use glam::{Mat3, Quat, Vec3};

pub(crate) fn outer(a: Vec3, b: Vec3) -> Mat3 {
    Mat3::from_cols(a * b.x, a * b.y, a * b.z)
}

pub(crate) fn rotation_matrix(q: Quat) -> Mat3 {
    Mat3::from_quat(q.normalize())
}

pub(crate) fn skew(v: Vec3) -> Mat3 {
    Mat3::from_cols(
        Vec3::new(0.0, v.z, -v.y),
        Vec3::new(-v.z, 0.0, v.x),
        Vec3::new(v.y, -v.x, 0.0),
    )
}

pub(crate) fn symmetrize_mat3(m: Mat3) -> Mat3 {
    (m + m.transpose()) * 0.5
}

pub(crate) fn wrap_pi(angle_rad: f32) -> f32 {
    let mut wrapped = angle_rad;
    while wrapped > core::f32::consts::PI {
        wrapped -= core::f32::consts::TAU;
    }
    while wrapped < -core::f32::consts::PI {
        wrapped += core::f32::consts::TAU;
    }
    wrapped
}

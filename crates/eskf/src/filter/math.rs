use math::{Matrix, Quat, Vec3};

use super::Eskf;

impl Eskf {
    pub(crate) fn system_matrix(orientation: Quat, accel_body: Vec3, dt: f32) -> Matrix<15, 15> {
        let mut f = Matrix::<15, 15>::identity();
        let r = Self::rotation_matrix(orientation);
        let a_skew = Self::skew(accel_body);

        let dt2 = dt * dt;
        let mut i = 0usize;
        while i < 3 {
            let mut j = 0usize;
            while j < 3 {
                let row_pos = i;
                let row_vel = 3 + i;
                let row_att = 6 + i;
                let col_vel = 3 + j;
                let col_att = 6 + j;
                let col_ba = 9 + j;
                let col_bg = 12 + j;
                let rskew = r.data[i][0] * a_skew.data[0][j]
                    + r.data[i][1] * a_skew.data[1][j]
                    + r.data[i][2] * a_skew.data[2][j];

                if i == j {
                    f.data[row_pos][col_vel] = dt;
                    f.data[row_att][col_bg] = -dt;
                }
                f.data[row_pos][col_att] = -0.5 * dt2 * rskew;
                f.data[row_pos][col_ba] = -0.5 * dt2 * r.data[i][j];
                f.data[row_vel][col_att] = -dt * rskew;
                f.data[row_vel][col_ba] = -dt * r.data[i][j];
                j += 1;
            }
            i += 1;
        }
        f
    }

    pub(crate) fn process_noise(
        accel_noise: f32,
        gyro_noise: f32,
        accel_bias_noise: f32,
        gyro_bias_noise: f32,
        dt: f32,
    ) -> Matrix<15, 15> {
        let dt2 = dt * dt;
        let dt4 = dt2 * dt2;
        let mut q = Matrix::<15, 15>::zeros();

        let pos_var = 0.25 * accel_noise * accel_noise * dt4;
        let vel_var = accel_noise * accel_noise * dt2;
        let att_var = gyro_noise * gyro_noise * dt2;
        let ba_var = accel_bias_noise * accel_bias_noise * dt.max(1.0e-6);
        let bg_var = gyro_bias_noise * gyro_bias_noise * dt.max(1.0e-6);

        let mut i = 0usize;
        while i < 3 {
            q.data[i][i] = pos_var;
            q.data[3 + i][3 + i] = vel_var;
            q.data[6 + i][6 + i] = att_var;
            q.data[9 + i][9 + i] = ba_var;
            q.data[12 + i][12 + i] = bg_var;
            i += 1;
        }
        q
    }

    pub(crate) fn rotation_matrix(q: Quat) -> Matrix<3, 3> {
        let q = q.normalized();
        let xx = q.x * q.x;
        let yy = q.y * q.y;
        let zz = q.z * q.z;
        let xy = q.x * q.y;
        let xz = q.x * q.z;
        let yz = q.y * q.z;
        let wx = q.w * q.x;
        let wy = q.w * q.y;
        let wz = q.w * q.z;

        Matrix::new([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ])
    }

    pub(crate) fn skew(v: Vec3) -> Matrix<3, 3> {
        Matrix::new([[0.0, -v.z, v.y], [v.z, 0.0, -v.x], [-v.y, v.x, 0.0]])
    }

    pub(crate) fn symmetrize(&self, p: Matrix<15, 15>) -> Matrix<15, 15> {
        (p + p.transpose()) * 0.5
    }
}

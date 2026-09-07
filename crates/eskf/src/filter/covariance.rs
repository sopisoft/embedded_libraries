use core::ops::{Index, IndexMut};

use glam::{Mat3, Vec3};

use super::math::{rotation_matrix, skew, symmetrize_mat3};

const BLOCK_COUNT: usize = 4;
const BLOCK_SIZE: usize = 3;
const STATE_DIM: usize = BLOCK_COUNT * BLOCK_SIZE;

#[derive(Copy, Clone)]
pub(super) struct ProcessNoise {
    pub accel: f32,
    pub gyro: f32,
    pub accel_bias: f32,
    pub gyro_bias: f32,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Covariance {
    data: [f32; STATE_DIM * STATE_DIM],
}

impl Covariance {
    pub fn zeros() -> Self {
        Self {
            data: [0.0; STATE_DIM * STATE_DIM],
        }
    }

    pub fn identity_scaled(diagonal: f32) -> Self {
        let mut covariance = Self::zeros();
        let mut index = 0usize;
        while index < STATE_DIM {
            covariance[(index, index)] = diagonal;
            index += 1;
        }
        covariance
    }

    pub fn block(&self, row_block: usize, col_block: usize) -> Mat3 {
        let row = row_block * BLOCK_SIZE;
        let col = col_block * BLOCK_SIZE;
        Mat3::from_cols(
            Vec3::new(self[(row, col)], self[(row + 1, col)], self[(row + 2, col)]),
            Vec3::new(
                self[(row, col + 1)],
                self[(row + 1, col + 1)],
                self[(row + 2, col + 1)],
            ),
            Vec3::new(
                self[(row, col + 2)],
                self[(row + 1, col + 2)],
                self[(row + 2, col + 2)],
            ),
        )
    }

    pub fn set_block(&mut self, row_block: usize, col_block: usize, block: Mat3) {
        let row = row_block * BLOCK_SIZE;
        let col = col_block * BLOCK_SIZE;
        self[(row, col)] = block.x_axis.x;
        self[(row + 1, col)] = block.x_axis.y;
        self[(row + 2, col)] = block.x_axis.z;
        self[(row, col + 1)] = block.y_axis.x;
        self[(row + 1, col + 1)] = block.y_axis.y;
        self[(row + 2, col + 1)] = block.y_axis.z;
        self[(row, col + 2)] = block.z_axis.x;
        self[(row + 1, col + 2)] = block.z_axis.y;
        self[(row + 2, col + 2)] = block.z_axis.z;
    }

    pub fn set_symmetric_block(&mut self, row_block: usize, col_block: usize, block: Mat3) {
        self.set_block(row_block, col_block, block);
        if row_block != col_block {
            self.set_block(col_block, row_block, block.transpose());
        }
    }

    pub fn symmetrize(&mut self) {
        let mut row_block = 0usize;
        while row_block < BLOCK_COUNT {
            let diagonal = symmetrize_mat3(self.block(row_block, row_block));
            self.set_block(row_block, row_block, diagonal);

            let mut col_block = row_block + 1;
            while col_block < BLOCK_COUNT {
                let block = (self.block(row_block, col_block)
                    + self.block(col_block, row_block).transpose())
                    * 0.5;
                self.set_symmetric_block(row_block, col_block, block);
                col_block += 1;
            }
            row_block += 1;
        }
    }

    pub(super) fn predict(
        &self,
        orientation: glam::Quat,
        accel_body: Vec3,
        noise: ProcessNoise,
        dt: f32,
    ) -> Self {
        let rotation = rotation_matrix(orientation);
        let accel_skew = skew(accel_body);
        let dt2 = dt * dt;

        let f01 = rotation * accel_skew * (-dt);
        let f02 = rotation * (-dt);
        let f13 = Mat3::IDENTITY * (-dt);

        let mut fp = [[Mat3::ZERO; BLOCK_COUNT]; BLOCK_COUNT];
        let mut col_block = 0usize;
        while col_block < BLOCK_COUNT {
            let p0 = self.block(0, col_block);
            let p1 = self.block(1, col_block);
            let p2 = self.block(2, col_block);
            let p3 = self.block(3, col_block);

            fp[0][col_block] = p0 + (f01 * p1) + (f02 * p2);
            fp[1][col_block] = p1 + (f13 * p3);
            fp[2][col_block] = p2;
            fp[3][col_block] = p3;
            col_block += 1;
        }

        let mut propagated = Self::zeros();
        let mut row_block = 0usize;
        while row_block < BLOCK_COUNT {
            propagated.set_block(row_block, 0, fp[row_block][0]);
            propagated.set_block(
                row_block,
                1,
                (fp[row_block][0] * f01.transpose()) + fp[row_block][1],
            );
            propagated.set_block(
                row_block,
                2,
                (fp[row_block][0] * f02.transpose()) + fp[row_block][2],
            );
            propagated.set_block(
                row_block,
                3,
                (fp[row_block][1] * f13.transpose()) + fp[row_block][3],
            );
            row_block += 1;
        }

        let vel_var = noise.accel * noise.accel * dt2;
        let att_var = noise.gyro * noise.gyro * dt2;
        let ba_var = noise.accel_bias * noise.accel_bias * dt.max(1.0e-6);
        let bg_var = noise.gyro_bias * noise.gyro_bias * dt.max(1.0e-6);

        propagated.set_block(0, 0, propagated.block(0, 0) + (Mat3::IDENTITY * vel_var));
        propagated.set_block(1, 1, propagated.block(1, 1) + (Mat3::IDENTITY * att_var));
        propagated.set_block(2, 2, propagated.block(2, 2) + (Mat3::IDENTITY * ba_var));
        propagated.set_block(3, 3, propagated.block(3, 3) + (Mat3::IDENTITY * bg_var));

        propagated.symmetrize();
        propagated
    }
}

impl Default for Covariance {
    fn default() -> Self {
        Self::zeros()
    }
}

impl Index<(usize, usize)> for Covariance {
    type Output = f32;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        &self.data[(index.0 * STATE_DIM) + index.1]
    }
}

impl IndexMut<(usize, usize)> for Covariance {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        &mut self.data[(index.0 * STATE_DIM) + index.1]
    }
}

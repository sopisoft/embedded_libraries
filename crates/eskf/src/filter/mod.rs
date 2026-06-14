//! Fifteen-state error-state Kalman filter.

use nalgebra::{SMatrix, SVector};

type Matrix<const R: usize, const C: usize> = SMatrix<f32, R, C>;
type Vector<const N: usize> = SVector<f32, N>;

mod math;
mod state;
#[cfg(test)]
mod tests;
mod update;

pub use state::Eskf;

//! Fixed-size matrix helpers.

use core::ops::{Add, Mul, Sub};
use libm::fabsf;

/// Fixed-size row-major matrix.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Matrix<const R: usize, const C: usize> {
    /// Row-major storage.
    pub data: [[f32; C]; R],
}

impl<const R: usize, const C: usize> Matrix<R, C> {
    /// Creates a matrix from row-major data.
    pub const fn new(data: [[f32; C]; R]) -> Self {
        Self { data }
    }

    /// Returns a zero matrix.
    pub const fn zeros() -> Self {
        Self {
            data: [[0.0; C]; R],
        }
    }

    /// Builds a matrix by evaluating a function for each element.
    pub fn from_fn(mut f: impl FnMut(usize, usize) -> f32) -> Self {
        let mut out = [[0.0; C]; R];
        let mut r = 0;
        while r < R {
            let mut c = 0;
            while c < C {
                out[r][c] = f(r, c);
                c += 1;
            }
            r += 1;
        }
        Self::new(out)
    }

    /// Returns the transposed matrix.
    pub fn transpose(self) -> Matrix<C, R> {
        Matrix::from_fn(|r, c| self.data[c][r])
    }

    /// Multiplies the matrix by a column vector.
    pub fn apply(self, rhs: [f32; C]) -> [f32; R] {
        let mut out = [0.0; R];
        let mut r = 0;
        while r < R {
            let mut acc = 0.0;
            let mut c = 0;
            while c < C {
                acc += self.data[r][c] * rhs[c];
                c += 1;
            }
            out[r] = acc;
            r += 1;
        }
        out
    }
}

impl<const N: usize> Matrix<N, N> {
    /// Returns the identity matrix.
    pub fn identity() -> Self {
        Self::from_fn(|r, c| if r == c { 1.0 } else { 0.0 })
    }

    /// Creates a diagonal matrix.
    pub fn from_diagonal(diagonal: [f32; N]) -> Self {
        Self::from_fn(|r, c| if r == c { diagonal[r] } else { 0.0 })
    }

    /// Computes the inverse using Gauss-Jordan elimination.
    pub fn inverse(self) -> Option<Self> {
        let mut a = self.data;
        let mut inv = Self::identity().data;
        let mut i = 0;
        while i < N {
            let mut pivot = i;
            let mut best = fabsf(a[i][i]);
            let mut row = i + 1;
            while row < N {
                let candidate = fabsf(a[row][i]);
                if candidate > best {
                    best = candidate;
                    pivot = row;
                }
                row += 1;
            }

            if best <= 1.0e-12 {
                return None;
            }

            if pivot != i {
                a.swap(i, pivot);
                inv.swap(i, pivot);
            }

            let pivot_value = a[i][i];
            let inv_pivot = 1.0 / pivot_value;

            let mut col = 0;
            while col < N {
                a[i][col] *= inv_pivot;
                inv[i][col] *= inv_pivot;
                col += 1;
            }

            let mut elim = 0;
            while elim < N {
                if elim != i {
                    let factor = a[elim][i];
                    if factor != 0.0 {
                        let mut col = 0;
                        while col < N {
                            a[elim][col] -= factor * a[i][col];
                            inv[elim][col] -= factor * inv[i][col];
                            col += 1;
                        }
                    }
                }
                elim += 1;
            }

            i += 1;
        }

        Some(Self::new(inv))
    }
}

impl<const R: usize, const C: usize, const K: usize> Mul<Matrix<C, K>> for Matrix<R, C> {
    type Output = Matrix<R, K>;

    fn mul(self, rhs: Matrix<C, K>) -> Self::Output {
        Matrix::from_fn(|r, k| {
            let mut acc = 0.0;
            let mut c = 0;
            while c < C {
                acc += self.data[r][c] * rhs.data[c][k];
                c += 1;
            }
            acc
        })
    }
}

impl<const R: usize, const C: usize> Mul<[f32; C]> for Matrix<R, C> {
    type Output = [f32; R];

    fn mul(self, rhs: [f32; C]) -> Self::Output {
        self.apply(rhs)
    }
}

impl<const R: usize, const C: usize> Add for Matrix<R, C> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::from_fn(|r, c| self.data[r][c] + rhs.data[r][c])
    }
}

impl<const R: usize, const C: usize> Sub for Matrix<R, C> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::from_fn(|r, c| self.data[r][c] - rhs.data[r][c])
    }
}

impl<const R: usize, const C: usize> Mul<f32> for Matrix<R, C> {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self::from_fn(|r, c| self.data[r][c] * rhs)
    }
}

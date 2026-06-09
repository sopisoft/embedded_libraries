use fugit::MicrosDurationU32;
use math::{Matrix, Vec3};

use crate::LowPassFilter;

/// 3xN control effectiveness matrix.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ControlEffectiveness<const N: usize> {
    pub matrix: Matrix<3, N>,
    pub regularization: f32,
}

impl<const N: usize> ControlEffectiveness<N> {
    /// Creates a matrix where each column maps one actuator increment to
    /// roll, pitch, and yaw angular acceleration.
    pub const fn new(matrix: Matrix<3, N>, regularization: f32) -> Self {
        Self {
            matrix,
            regularization,
        }
    }

    /// Allocates an actuator increment using damped least squares.
    pub fn allocate_delta(&self, acceleration_error_rad_s2: Vec3) -> Option<[f32; N]> {
        let transpose = self.matrix.transpose();
        let mut normal = self.matrix * transpose;
        let lambda2 = self.regularization.max(0.0) * self.regularization.max(0.0);
        let mut axis = 0usize;
        while axis < 3 {
            normal.data[axis][axis] += lambda2;
            axis += 1;
        }

        let inverse = normal.inverse()?;
        let axis_solution = inverse
            * [
                acceleration_error_rad_s2.x,
                acceleration_error_rad_s2.y,
                acceleration_error_rad_s2.z,
            ];
        Some(transpose * axis_solution)
    }
}

impl ControlEffectiveness<3> {
    /// Creates a diagonal roll/pitch/yaw effectiveness model.
    pub fn diagonal(effectiveness: Vec3, regularization: f32) -> Self {
        Self::new(
            Matrix::new([
                [effectiveness.x, 0.0, 0.0],
                [0.0, effectiveness.y, 0.0],
                [0.0, 0.0, effectiveness.z],
            ]),
            regularization,
        )
    }
}

/// Configuration for a 3xN INDI allocator.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct IndiAllocatorConfig<const N: usize> {
    pub effectiveness: ControlEffectiveness<N>,
    pub actuator_min: [f32; N],
    pub actuator_trim: [f32; N],
    pub actuator_max: [f32; N],
    pub actuator_slew_rate_per_s: [f32; N],
    pub acceleration_filter_tau_s: f32,
}

impl<const N: usize> IndiAllocatorConfig<N> {
    /// Creates a normalized actuator allocator.
    pub const fn normalized(
        effectiveness: ControlEffectiveness<N>,
        actuator_slew_rate_per_s: f32,
    ) -> Self {
        Self {
            effectiveness,
            actuator_min: [-1.0; N],
            actuator_trim: [0.0; N],
            actuator_max: [1.0; N],
            actuator_slew_rate_per_s: [actuator_slew_rate_per_s; N],
            acceleration_filter_tau_s: 0.02,
        }
    }
}

/// Output of a 3xN control allocation step.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct IndiAllocatorOutput<const N: usize> {
    pub actuator: [f32; N],
    pub actuator_delta: [f32; N],
    pub desired_angular_accel_rad_s2: Vec3,
    pub measured_angular_accel_rad_s2: Vec3,
    pub acceleration_error_rad_s2: Vec3,
}

/// INDI allocator for coupled or over-actuated vehicles.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct IndiAllocator<const N: usize> {
    config: IndiAllocatorConfig<N>,
    actuator: [f32; N],
    acceleration_filter: [LowPassFilter; 3],
}

impl<const N: usize> IndiAllocator<N> {
    /// Creates a new allocator.
    pub const fn new(config: IndiAllocatorConfig<N>) -> Self {
        Self {
            actuator: config.actuator_trim,
            acceleration_filter: [
                LowPassFilter::new(config.acceleration_filter_tau_s),
                LowPassFilter::new(config.acceleration_filter_tau_s),
                LowPassFilter::new(config.acceleration_filter_tau_s),
            ],
            config,
        }
    }

    /// Returns the current actuator command vector.
    pub const fn actuator(&self) -> [f32; N] {
        self.actuator
    }

    /// Resets actuator and filter state.
    pub fn reset(&mut self) {
        self.actuator = self.config.actuator_trim;
        for filter in &mut self.acceleration_filter {
            filter.reset();
        }
    }

    /// Runs one allocation step.
    pub fn update(
        &mut self,
        desired_angular_accel_rad_s2: Vec3,
        measured_angular_accel_rad_s2: Vec3,
        dt: MicrosDurationU32,
    ) -> Option<IndiAllocatorOutput<N>> {
        let measured_angular_accel_rad_s2 = Vec3::new(
            self.acceleration_filter[0].update(measured_angular_accel_rad_s2.x, dt),
            self.acceleration_filter[1].update(measured_angular_accel_rad_s2.y, dt),
            self.acceleration_filter[2].update(measured_angular_accel_rad_s2.z, dt),
        );
        let acceleration_error_rad_s2 =
            desired_angular_accel_rad_s2 - measured_angular_accel_rad_s2;
        let requested_delta = self
            .config
            .effectiveness
            .allocate_delta(acceleration_error_rad_s2)?;

        let dt_s = dt.as_secs_f32();
        let mut actuator_delta = [0.0; N];
        let mut index = 0usize;
        while index < N {
            let max_delta = self.config.actuator_slew_rate_per_s[index].abs() * dt_s;
            let delta = if max_delta > 0.0 {
                requested_delta[index].clamp(-max_delta, max_delta)
            } else {
                requested_delta[index]
            };
            let previous = self.actuator[index];
            self.actuator[index] = (self.actuator[index] + delta).clamp(
                self.config.actuator_min[index],
                self.config.actuator_max[index],
            );
            actuator_delta[index] = self.actuator[index] - previous;
            index += 1;
        }

        Some(IndiAllocatorOutput {
            actuator: self.actuator,
            actuator_delta,
            desired_angular_accel_rad_s2,
            measured_angular_accel_rad_s2,
            acceleration_error_rad_s2,
        })
    }
}

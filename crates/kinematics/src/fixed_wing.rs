//! Fixed-wing specific kinematic helpers.

use fugit::MicrosDurationU32;
use glam::{EulerRot, Quat, Vec2, Vec3};
use libm::{sinf, tanf};

use crate::Pose3;

/// Fixed-wing state propagated from airspeed, attitude, and wind.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FixedWingState {
    /// Vehicle pose.
    pub pose: Pose3,
    /// Ground-relative velocity in the world frame.
    pub ground_velocity: Vec3,
    /// Estimated horizontal wind in the world frame.
    pub wind_xy: Vec2,
    /// Scalar airspeed along the body X axis.
    pub airspeed_m_s: f32,
}

impl FixedWingState {
    /// Creates a new state at the origin with zero wind and zero airspeed.
    pub const fn new() -> Self {
        Self {
            pose: Pose3::identity(),
            ground_velocity: Vec3::ZERO,
            wind_xy: Vec2::ZERO,
            airspeed_m_s: 0.0,
        }
    }

    /// Sets the attitude from Euler angles.
    pub fn set_euler(&mut self, euler_rad: Vec3) {
        self.pose.orientation =
            Quat::from_euler(EulerRot::XYZ, euler_rad.x, euler_rad.y, euler_rad.z);
    }

    /// Propagates the state using body attitude, airspeed, and horizontal wind.
    pub fn step_with_airspeed(
        &mut self,
        airspeed_m_s: f32,
        wind_xy: Vec2,
        gyro_rad_s: Vec3,
        dt: MicrosDurationU32,
    ) {
        let dt = dt.as_secs_f32();
        self.pose.orientation =
            (self.pose.orientation * Quat::from_scaled_axis(gyro_rad_s * dt)).normalize();
        self.airspeed_m_s = airspeed_m_s;
        self.wind_xy = wind_xy;

        let air_velocity_world = self
            .pose
            .orientation
            .mul_vec3(Vec3::new(airspeed_m_s, 0.0, 0.0));
        self.ground_velocity = Vec3::new(
            air_velocity_world.x + wind_xy.x,
            air_velocity_world.y + wind_xy.y,
            air_velocity_world.z,
        );
        self.pose.position += self.ground_velocity * dt;
    }
}

impl Default for FixedWingState {
    fn default() -> Self {
        Self::new()
    }
}

/// Computes coordinated-turn yaw rate from bank angle and airspeed.
pub fn coordinated_turn_rate(bank_angle_rad: f32, airspeed_m_s: f32, gravity_m_s2: f32) -> f32 {
    if airspeed_m_s <= 0.0 {
        0.0
    } else {
        tanf(bank_angle_rad) * gravity_m_s2 / airspeed_m_s
    }
}

/// Returns the lateral acceleration needed for a coordinated turn.
pub fn lateral_acceleration_for_turn(bank_angle_rad: f32, gravity_m_s2: f32) -> f32 {
    sinf(bank_angle_rad) * gravity_m_s2
}

#[cfg(test)]
mod tests {
    use super::*;
    use fugit::MicrosDurationU32;
    use libm::fabsf;

    #[test]
    fn fixed_wing_steps_forward() {
        let mut state = FixedWingState::new();
        state.step_with_airspeed(
            20.0,
            Vec2::ZERO,
            Vec3::ZERO,
            MicrosDurationU32::from_secs(1),
        );
        assert!(fabsf(state.pose.position.x - 20.0) < 1.0e-6);
    }

    #[test]
    fn coordinated_turn_rate_is_zero_at_zero_bank() {
        assert!(fabsf(coordinated_turn_rate(0.0, 20.0, 9.80665)) < 1.0e-6);
    }
}

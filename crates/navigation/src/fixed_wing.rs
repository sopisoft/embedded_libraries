//! Fixed-wing dead reckoning on top of the inertial navigator.

use fugit::MicrosDurationU32;
use kinematics::FixedWingState;
use math::{EulerAngles, Quat, Vec2, Vec3};

use crate::InertialNavigator;

/// A simple fixed-wing navigator using attitude, airspeed, and horizontal wind.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FixedWingNavigator {
    /// Underlying inertial navigator state.
    pub navigator: InertialNavigator,
    /// Horizontal wind estimate in the world frame.
    pub wind_xy: Vec2,
    /// Last airspeed value used for propagation.
    pub airspeed_m_s: f32,
}

impl FixedWingNavigator {
    /// Creates a new fixed-wing navigator.
    pub const fn new() -> Self {
        Self {
            navigator: InertialNavigator::new(),
            wind_xy: Vec2::zero(),
            airspeed_m_s: 0.0,
        }
    }

    /// Sets the attitude estimate directly from Euler angles.
    pub fn set_attitude(&mut self, euler: EulerAngles) {
        self.navigator.pose.orientation = Quat::from_euler(euler);
    }

    /// Propagates position from airspeed, gyro, and wind.
    pub fn predict_airspeed(
        &mut self,
        airspeed_m_s: f32,
        gyro_rad_s: Vec3,
        wind_xy: Vec2,
        dt: MicrosDurationU32,
    ) {
        let dt_s = dt.as_secs_f32();
        self.navigator.pose.orientation = self
            .navigator
            .pose
            .orientation
            .integrate_gyro(gyro_rad_s, dt_s);
        self.airspeed_m_s = airspeed_m_s;
        self.wind_xy = wind_xy;

        let air_velocity_world =
            self.navigator
                .pose
                .orientation
                .rotate_vec3(Vec3::new(airspeed_m_s, 0.0, 0.0));
        self.navigator.velocity_world = Vec3::new(
            air_velocity_world.x + wind_xy.x,
            air_velocity_world.y + wind_xy.y,
            air_velocity_world.z,
        );
        self.navigator.pose.position += self.navigator.velocity_world * dt_s;
    }

    /// Corrects the horizontal wind estimate from observed ground velocity.
    pub fn correct_wind_from_groundspeed(&mut self, observed_ground_velocity: Vec3, gain: f32) {
        let air_velocity_world =
            self.navigator
                .pose
                .orientation
                .rotate_vec3(Vec3::new(self.airspeed_m_s, 0.0, 0.0));
        let observed_wind = Vec2::new(
            observed_ground_velocity.x - air_velocity_world.x,
            observed_ground_velocity.y - air_velocity_world.y,
        );
        self.wind_xy = self.wind_xy + (observed_wind - self.wind_xy) * gain.clamp(0.0, 1.0);
    }

    /// Returns a kinematic snapshot for downstream code.
    pub fn as_state(&self) -> FixedWingState {
        FixedWingState {
            pose: self.navigator.pose,
            ground_velocity: self.navigator.velocity_world,
            wind_xy: self.wind_xy,
            airspeed_m_s: self.airspeed_m_s,
        }
    }
}

impl Default for FixedWingNavigator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fugit::MicrosDurationU32;
    use libm::fabsf;

    #[test]
    fn airspeed_prediction_moves_forward() {
        let mut nav = FixedWingNavigator::new();
        nav.predict_airspeed(
            15.0,
            Vec3::zero(),
            Vec2::zero(),
            MicrosDurationU32::from_secs(1),
        );
        assert!(fabsf(nav.navigator.pose.position.x - 15.0) < 1.0e-6);
    }

    #[test]
    fn wind_correction_updates_estimate() {
        let mut nav = FixedWingNavigator::new();
        nav.airspeed_m_s = 10.0;
        nav.correct_wind_from_groundspeed(Vec3::new(12.0, 3.0, 0.0), 1.0);
        assert!(fabsf(nav.wind_xy.x - 2.0) < 1.0e-6);
        assert!(fabsf(nav.wind_xy.y - 3.0) < 1.0e-6);
    }
}

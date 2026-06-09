//! Fixed-wing TECS implementation.

use control::PidController;
use fugit::MicrosDurationU32;

/// Default gravitational acceleration in `m/s^2`.
pub const STANDARD_GRAVITY_M_S2: f32 = 9.80665;

/// Computes specific potential energy in `J/kg`.
pub fn specific_potential_energy(height_m: f32, gravity_m_s2: f32) -> f32 {
    gravity_m_s2 * height_m
}

/// Computes specific kinetic energy in `J/kg`.
pub fn specific_kinetic_energy(airspeed_m_s: f32) -> f32 {
    0.5 * airspeed_m_s * airspeed_m_s
}

/// Computes total specific energy in `J/kg`.
pub fn specific_total_energy(height_m: f32, airspeed_m_s: f32, gravity_m_s2: f32) -> f32 {
    specific_potential_energy(height_m, gravity_m_s2) + specific_kinetic_energy(airspeed_m_s)
}

/// Commanded fixed-wing energy state.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TecsTarget {
    pub altitude_m: f32,
    pub airspeed_m_s: f32,
}

impl TecsTarget {
    pub const fn new(altitude_m: f32, airspeed_m_s: f32) -> Self {
        Self {
            altitude_m,
            airspeed_m_s,
        }
    }
}

/// Measured fixed-wing energy state.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TecsState {
    pub altitude_m: f32,
    pub airspeed_m_s: f32,
}

impl TecsState {
    pub const fn new(altitude_m: f32, airspeed_m_s: f32) -> Self {
        Self {
            altitude_m,
            airspeed_m_s,
        }
    }
}

/// Output limits and static trims for TECS.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TecsConfig {
    pub throttle_min: f32,
    pub throttle_trim: f32,
    pub throttle_max: f32,
    pub pitch_min_rad: f32,
    pub pitch_trim_rad: f32,
    pub pitch_max_rad: f32,
    /// `0.0` prioritizes height, `2.0` prioritizes speed.
    pub speed_weight: f32,
    pub gravity_m_s2: f32,
}

impl TecsConfig {
    pub const fn new(
        throttle_min: f32,
        throttle_trim: f32,
        throttle_max: f32,
        pitch_min_rad: f32,
        pitch_trim_rad: f32,
        pitch_max_rad: f32,
        speed_weight: f32,
    ) -> Self {
        Self {
            throttle_min,
            throttle_trim,
            throttle_max,
            pitch_min_rad,
            pitch_trim_rad,
            pitch_max_rad,
            speed_weight,
            gravity_m_s2: STANDARD_GRAVITY_M_S2,
        }
    }

    pub const fn with_gravity(mut self, gravity_m_s2: f32) -> Self {
        self.gravity_m_s2 = gravity_m_s2;
        self
    }

    pub fn speed_weight(self) -> f32 {
        self.speed_weight.clamp(0.0, 2.0)
    }
}

impl Default for TecsConfig {
    fn default() -> Self {
        Self::new(0.0, 0.5, 1.0, -0.35, 0.0, 0.35, 1.0)
    }
}

/// Full result of one TECS update.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TecsOutput {
    pub throttle: f32,
    pub pitch_rad: f32,
    pub potential_energy_error: f32,
    pub kinetic_energy_error: f32,
    pub total_energy_error: f32,
    pub balance_error: f32,
}

/// Lightweight TECS controller for altitude and airspeed hold.
///
/// The controller follows the standard TECS split:
/// - throttle tracks total energy,
/// - pitch tracks the balance between potential and kinetic energy.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TecsController {
    total_energy_pid: PidController,
    balance_pid: PidController,
    config: TecsConfig,
}

impl TecsController {
    pub const fn new(
        total_energy_pid: PidController,
        balance_pid: PidController,
        config: TecsConfig,
    ) -> Self {
        Self {
            total_energy_pid,
            balance_pid,
            config,
        }
    }

    pub const fn config(&self) -> TecsConfig {
        self.config
    }

    pub fn set_config(&mut self, config: TecsConfig) {
        self.config = config;
    }

    pub fn set_speed_weight(&mut self, speed_weight: f32) {
        self.config.speed_weight = speed_weight.clamp(0.0, 2.0);
    }

    pub fn reset(&mut self) {
        self.total_energy_pid.reset();
        self.balance_pid.reset();
    }

    pub fn total_energy_pid(&self) -> &PidController {
        &self.total_energy_pid
    }

    pub fn total_energy_pid_mut(&mut self) -> &mut PidController {
        &mut self.total_energy_pid
    }

    pub fn balance_pid(&self) -> &PidController {
        &self.balance_pid
    }

    pub fn balance_pid_mut(&mut self) -> &mut PidController {
        &mut self.balance_pid
    }

    /// Computes throttle and pitch commands for the requested energy state.
    pub fn update(
        &mut self,
        target: TecsTarget,
        state: TecsState,
        dt: MicrosDurationU32,
    ) -> TecsOutput {
        let gravity = self.config.gravity_m_s2;
        let speed_weight = self.config.speed_weight();
        let height_weight = 2.0 - speed_weight;

        let target_potential = specific_potential_energy(target.altitude_m, gravity);
        let state_potential = specific_potential_energy(state.altitude_m, gravity);
        let target_kinetic = specific_kinetic_energy(target.airspeed_m_s);
        let state_kinetic = specific_kinetic_energy(state.airspeed_m_s);

        let potential_energy_error = target_potential - state_potential;
        let kinetic_energy_error = target_kinetic - state_kinetic;
        let total_energy_error = potential_energy_error + kinetic_energy_error;
        let balance_error =
            height_weight * potential_energy_error - speed_weight * kinetic_energy_error;

        let throttle_correction = self.total_energy_pid.update(0.0, -total_energy_error, dt);
        let pitch_correction = self.balance_pid.update(0.0, -balance_error, dt);

        let throttle = (self.config.throttle_trim + throttle_correction)
            .clamp(self.config.throttle_min, self.config.throttle_max);
        let pitch_rad = (self.config.pitch_trim_rad + pitch_correction)
            .clamp(self.config.pitch_min_rad, self.config.pitch_max_rad);

        TecsOutput {
            throttle,
            pitch_rad,
            potential_energy_error,
            kinetic_energy_error,
            total_energy_error,
            balance_error,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_controller() -> TecsController {
        let mut total_energy_pid = PidController::new(0.004, 0.001, 0.0);
        total_energy_pid.set_output_limits(-0.4, 0.4);
        total_energy_pid.set_integral_limits(-50.0, 50.0);

        let mut balance_pid = PidController::new(0.003, 0.0005, 0.0);
        balance_pid.set_output_limits(-0.3, 0.3);
        balance_pid.set_integral_limits(-50.0, 50.0);

        TecsController::new(total_energy_pid, balance_pid, TecsConfig::default())
    }

    #[test]
    fn climb_request_raises_throttle_and_pitch() {
        let mut controller = make_controller();
        let output = controller.update(
            TecsTarget::new(120.0, 18.0),
            TecsState::new(100.0, 18.0),
            MicrosDurationU32::from_millis(20),
        );
        assert!(output.total_energy_error > 0.0);
        assert!(output.throttle > controller.config().throttle_trim);
        assert!(output.pitch_rad > controller.config().pitch_trim_rad);
    }

    #[test]
    fn speed_deficit_pushes_pitch_down_to_recover_energy_balance() {
        let mut controller = make_controller();
        let output = controller.update(
            TecsTarget::new(100.0, 22.0),
            TecsState::new(100.0, 16.0),
            MicrosDurationU32::from_millis(20),
        );
        assert!(output.kinetic_energy_error > 0.0);
        assert!(output.throttle > controller.config().throttle_trim);
        assert!(output.pitch_rad < controller.config().pitch_trim_rad);
    }

    #[test]
    fn speed_weight_zero_removes_speed_priority_from_pitch_balance() {
        let mut controller = make_controller();
        controller.set_config(TecsConfig::default().with_gravity(STANDARD_GRAVITY_M_S2));
        controller.set_speed_weight(0.0);
        let output = controller.update(
            TecsTarget::new(100.0, 22.0),
            TecsState::new(100.0, 16.0),
            MicrosDurationU32::from_millis(20),
        );
        assert!(output.pitch_rad.abs() < 1.0e-6);
    }

    #[test]
    fn output_respects_config_limits() {
        let mut controller = make_controller();
        controller.set_config(TecsConfig::new(0.2, 0.4, 0.6, -0.1, 0.0, 0.1, 1.0));
        let output = controller.update(
            TecsTarget::new(1_000.0, 40.0),
            TecsState::new(0.0, 5.0),
            MicrosDurationU32::from_millis(20),
        );
        assert_eq!(output.throttle, 0.6);
        assert_eq!(output.pitch_rad, 0.1);
    }
}

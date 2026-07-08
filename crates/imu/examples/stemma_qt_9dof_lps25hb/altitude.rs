use super::*;

#[derive(Clone, Copy, Debug)]
pub struct BaroState {
    reference_pressure_hpa: Option<f32>,
    reference_pressure_sum_hpa: f32,
    reference_pressure_samples: u32,
    filtered_pressure_hpa: Option<f32>,
    latest_baro_altitude_m: f32,
    initialized: bool,
}

impl BaroState {
    pub const fn new() -> Self {
        Self {
            reference_pressure_hpa: None,
            reference_pressure_sum_hpa: 0.0,
            reference_pressure_samples: 0,
            filtered_pressure_hpa: None,
            latest_baro_altitude_m: 0.0,
            initialized: false,
        }
    }

    pub fn update_baro(&mut self, pressure_hpa: f32) -> (Option<f32>, bool) {
        let filtered_pressure_hpa = if let Some(previous_pressure_hpa) = self.filtered_pressure_hpa
        {
            previous_pressure_hpa
                + (pressure_hpa - previous_pressure_hpa) * BARO_PRESSURE_LOWPASS_GAIN
        } else {
            pressure_hpa
        };
        self.filtered_pressure_hpa = Some(filtered_pressure_hpa);

        if self.reference_pressure_hpa.is_none() {
            self.reference_pressure_sum_hpa += filtered_pressure_hpa;
            self.reference_pressure_samples = self.reference_pressure_samples.saturating_add(1);
            if self.reference_pressure_samples < BARO_REFERENCE_SAMPLES {
                return (None, false);
            }
            self.reference_pressure_hpa =
                Some(self.reference_pressure_sum_hpa / self.reference_pressure_samples as f32);
            self.latest_baro_altitude_m = 0.0;
            self.initialized = true;
            return (Some(self.latest_baro_altitude_m), true);
        }

        let reference_pressure_hpa = self.reference_pressure_hpa.unwrap();
        self.latest_baro_altitude_m =
            pressure_to_altitude_m(filtered_pressure_hpa, reference_pressure_hpa);
        self.initialized = true;
        (Some(self.latest_baro_altitude_m), false)
    }

    pub fn current_altitude(&self) -> Option<f32> {
        self.initialized.then_some(self.latest_baro_altitude_m)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AltitudeComplementaryFilter {
    altitude_m: f32,
    vertical_speed_m_s: f32,
    accel_bias_m_s2: f32,
    filtered_vertical_accel_m_s2: f32,
    initialized: bool,
}

impl AltitudeComplementaryFilter {
    pub const fn new() -> Self {
        Self {
            altitude_m: 0.0,
            vertical_speed_m_s: 0.0,
            accel_bias_m_s2: 0.0,
            filtered_vertical_accel_m_s2: 0.0,
            initialized: false,
        }
    }

    pub fn predict(&mut self, dt_s: f32, vertical_accel_world_m_s2: f32, stationary: bool) {
        if dt_s <= 0.0 {
            return;
        }

        if stationary {
            self.accel_bias_m_s2 +=
                (vertical_accel_world_m_s2 - self.accel_bias_m_s2) * ALTITUDE_ACCEL_BIAS_GAIN;
        }

        let unbiased_accel = vertical_accel_world_m_s2 - self.accel_bias_m_s2;
        self.filtered_vertical_accel_m_s2 +=
            (unbiased_accel - self.filtered_vertical_accel_m_s2) * ALTITUDE_ACCEL_LOWPASS_GAIN;

        let corrected_accel =
            if self.filtered_vertical_accel_m_s2.abs() < ALTITUDE_ACCEL_DEADBAND_M_S2 {
                0.0
            } else {
                self.filtered_vertical_accel_m_s2
            };

        if self.initialized {
            self.altitude_m += self.vertical_speed_m_s * dt_s + 0.5 * corrected_accel * dt_s * dt_s;
        }
        self.vertical_speed_m_s += corrected_accel * dt_s;

        if stationary {
            self.vertical_speed_m_s *= 1.0 - ALTITUDE_ZERO_VELOCITY_GAIN;
        }
    }

    pub fn update_altitude(&mut self, measured_altitude_m: f32, dt_s: f32) {
        if !self.initialized {
            self.altitude_m = measured_altitude_m;
            self.vertical_speed_m_s = 0.0;
            self.initialized = true;
            return;
        }

        let residual = measured_altitude_m - self.altitude_m;
        self.altitude_m += ALTITUDE_COMPLEMENTARY_POSITION_GAIN * residual;
        if dt_s > 0.0 {
            self.vertical_speed_m_s += ALTITUDE_COMPLEMENTARY_VELOCITY_GAIN * residual / dt_s;
        }
    }

    pub fn current(&self) -> Option<(f32, f32)> {
        self.initialized
            .then_some((self.altitude_m, self.vertical_speed_m_s))
    }
}

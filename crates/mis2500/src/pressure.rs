const PA_PER_KPA: f32 = 1_000.0;
const PA_PER_HPA: f32 = 100.0;
const PA_PER_PSI: f32 = 6_894.757_3;

/// Pressure value with convenience accessors.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Pressure {
    pa: f32,
}

impl Pressure {
    /// Creates a pressure value from pascals.
    pub const fn from_pa(pa: f32) -> Self {
        Self { pa }
    }

    /// Returns the pressure in pascals.
    pub const fn pa(self) -> f32 {
        self.pa
    }

    /// Returns the pressure in kilopascals.
    pub fn kpa(self) -> f32 {
        self.pa / PA_PER_KPA
    }

    /// Returns the pressure in hectopascals.
    pub fn hpa(self) -> f32 {
        self.pa / PA_PER_HPA
    }

    /// Returns the pressure in millibars.
    pub fn mbar(self) -> f32 {
        self.hpa()
    }

    /// Returns the pressure in psi.
    pub fn psi(self) -> f32 {
        self.pa / PA_PER_PSI
    }
}

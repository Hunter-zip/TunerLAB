//! Driver / test-cell inputs: key, pedal, environment and the load the engine drives.

use super::thermo::{STANDARD_PRESSURE_PA, ZERO_CELSIUS_K};

/// Ambient conditions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ambient {
    /// Barometric pressure \[Pa\].
    pub pressure_pa: f32,
    /// Air temperature \[K\].
    pub temperature_k: f32,
    /// Relative humidity 0..1.
    pub relative_humidity: f32,
}

impl Default for Ambient {
    /// SAE J1349 reference conditions: 99 kPa dry-air partial pressure, 25 °C. We use
    /// standard sea-level pressure with 40 % RH, which yields almost the same dry-air
    /// density.
    fn default() -> Self {
        Self {
            pressure_pa: STANDARD_PRESSURE_PA,
            temperature_k: ZERO_CELSIUS_K + 25.0,
            relative_humidity: 0.4,
        }
    }
}

/// Engine-dynamometer settings (steady-state eddy-current absorber).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DynoParams {
    /// Speed the absorber holds \[rpm\]. Ramp this value for a sweep test.
    pub target_rpm: f32,
    /// Maximum absorber torque \[N·m\].
    pub max_torque_nm: f32,
    /// Absorber rotor inertia coupled to the crankshaft \[kg·m²\].
    pub absorber_inertia_kg_m2: f32,
    /// Proportional gain of the absorber speed controller \[N·m/rpm\].
    pub kp_nm_per_rpm: f32,
    /// Integral gain of the absorber speed controller [N·m/(rpm·s)].
    pub ki_nm_per_rpm_s: f32,
    /// Cooling-fan air speed through the radiator in the test cell \[m/s\].
    pub cooling_air_speed_m_s: f32,
}

impl Default for DynoParams {
    fn default() -> Self {
        Self {
            target_rpm: 3000.0,
            max_torque_nm: 800.0,
            absorber_inertia_kg_m2: 0.25,
            kp_nm_per_rpm: 1.5,
            ki_nm_per_rpm_s: 6.0,
            cooling_air_speed_m_s: 12.0,
        }
    }
}

/// Longitudinal vehicle model parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VehicleParams {
    /// Kerb mass plus driver \[kg\].
    pub mass_kg: f32,
    /// Dynamic rolling radius of the driven wheels \[m\].
    pub wheel_radius_m: f32,
    /// Gearbox ratios, 1st..6th.
    pub gear_ratios: [f32; 6],
    /// Final-drive ratio.
    pub final_drive: f32,
    /// Selected gear: 0 = neutral, 1..=6.
    pub gear: u8,
    /// Clutch engagement 0 (pedal down) .. 1 (fully engaged).
    pub clutch: f32,
    /// Clutch torque capacity when fully engaged \[N·m\].
    pub clutch_capacity_nm: f32,
    /// Aerodynamic drag area Cd·A \[m²\].
    pub drag_area_m2: f32,
    /// Rolling-resistance coefficient C_rr.
    pub rolling_coefficient: f32,
    /// Driveline mechanical efficiency.
    pub driveline_efficiency: f32,
    /// Service brake application 0..1.
    pub brake: f32,
    /// Road gradient \[rad\], positive uphill.
    pub grade_rad: f32,
}

impl Default for VehicleParams {
    /// A 1300 kg compact hatchback with a six-speed manual gearbox.
    fn default() -> Self {
        Self {
            mass_kg: 1300.0,
            // 205/55 R16 → 0.316 m loaded rolling radius.
            wheel_radius_m: 0.316,
            gear_ratios: [3.58, 2.02, 1.35, 1.03, 0.84, 0.69],
            final_drive: 4.06,
            gear: 0,
            clutch: 1.0,
            clutch_capacity_nm: 350.0,
            // Cd 0.31 × 2.2 m² frontal area.
            drag_area_m2: 0.68,
            rolling_coefficient: 0.011,
            driveline_efficiency: 0.92,
            brake: 0.0,
            grade_rad: 0.0,
        }
    }
}

/// What the engine is connected to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LoadModel {
    /// Free revving (gearbox in neutral, no load beyond accessories).
    Neutral,
    /// Engine dynamometer holding a target speed.
    Dyno(DynoParams),
    /// Full longitudinal vehicle with clutch and gearbox.
    Vehicle(VehicleParams),
}

/// All operator inputs of the simulation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Controls {
    /// Ignition key in RUN.
    pub ignition_on: bool,
    /// Ignition key in START (starter motor engaged).
    pub starter: bool,
    /// Accelerator pedal position 0..1.
    pub pedal: f32,
    /// Connected load.
    pub load: LoadModel,
    /// Environment.
    pub ambient: Ambient,
    /// Research octane number of the fuel in the tank.
    pub fuel_octane_ron: f32,
    /// Vehicle electrical consumers (lights, fans, heaters) \[W\].
    pub electrical_load_w: f32,
}

impl Default for Controls {
    fn default() -> Self {
        Self {
            ignition_on: false,
            starter: false,
            pedal: 0.0,
            load: LoadModel::Neutral,
            ambient: Ambient::default(),
            // EN 228 premium unleaded.
            fuel_octane_ron: 95.0,
            electrical_load_w: 300.0,
        }
    }
}

impl Controls {
    /// Returns a copy with every value forced into its valid range (NaN-safe), so the
    /// simulation never has to trust UI input.
    pub(crate) fn sanitized(&self) -> Self {
        let mut c = *self;
        c.pedal = clamp_or(c.pedal, 0.0, 1.0, 0.0);
        c.ambient.pressure_pa = clamp_or(
            c.ambient.pressure_pa,
            50_000.0,
            110_000.0,
            STANDARD_PRESSURE_PA,
        );
        c.ambient.temperature_k = clamp_or(c.ambient.temperature_k, 233.15, 328.15, 298.15);
        c.ambient.relative_humidity = clamp_or(c.ambient.relative_humidity, 0.0, 1.0, 0.4);
        c.fuel_octane_ron = clamp_or(c.fuel_octane_ron, 80.0, 110.0, 95.0);
        c.electrical_load_w = clamp_or(c.electrical_load_w, 0.0, 3000.0, 300.0);
        match &mut c.load {
            LoadModel::Neutral => {}
            LoadModel::Dyno(d) => {
                d.target_rpm = clamp_or(d.target_rpm, 0.0, 12_000.0, 3000.0);
                d.max_torque_nm = clamp_or(d.max_torque_nm, 0.0, 5000.0, 800.0);
                d.absorber_inertia_kg_m2 = clamp_or(d.absorber_inertia_kg_m2, 0.0, 10.0, 0.25);
                d.kp_nm_per_rpm = clamp_or(d.kp_nm_per_rpm, 0.0, 50.0, 1.5);
                d.ki_nm_per_rpm_s = clamp_or(d.ki_nm_per_rpm_s, 0.0, 200.0, 6.0);
                d.cooling_air_speed_m_s = clamp_or(d.cooling_air_speed_m_s, 0.0, 40.0, 12.0);
            }
            LoadModel::Vehicle(v) => {
                v.mass_kg = clamp_or(v.mass_kg, 100.0, 40_000.0, 1300.0);
                v.wheel_radius_m = clamp_or(v.wheel_radius_m, 0.1, 1.5, 0.316);
                for r in &mut v.gear_ratios {
                    *r = clamp_or(*r, 0.2, 10.0, 1.0);
                }
                v.final_drive = clamp_or(v.final_drive, 1.0, 15.0, 4.06);
                v.gear = v.gear.min(6);
                v.clutch = clamp_or(v.clutch, 0.0, 1.0, 1.0);
                v.clutch_capacity_nm = clamp_or(v.clutch_capacity_nm, 10.0, 5000.0, 350.0);
                v.drag_area_m2 = clamp_or(v.drag_area_m2, 0.0, 20.0, 0.68);
                v.rolling_coefficient = clamp_or(v.rolling_coefficient, 0.0, 0.1, 0.011);
                v.driveline_efficiency = clamp_or(v.driveline_efficiency, 0.5, 1.0, 0.92);
                v.brake = clamp_or(v.brake, 0.0, 1.0, 0.0);
                v.grade_rad = clamp_or(v.grade_rad, -0.35, 0.35, 0.0);
            }
        }
        c
    }
}

fn clamp_or(x: f32, lo: f32, hi: f32, fallback: f32) -> f32 {
    if x.is_finite() {
        x.clamp(lo, hi)
    } else {
        fallback
    }
}

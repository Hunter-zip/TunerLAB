//! Lumped-capacitance thermal network.
//!
//! Nodes: engine metal, coolant, oil, one piston crown per cylinder, the catalyst and the
//! exhaust manifold wall. Each node obeys `C·dT/dt = ΣQ̇`.
//!
//! The network is advanced at [`THERMAL_STEP_S`] rather than at the 0.25 ms physics step,
//! with every heat input integrated as energy over the interval by the caller. Two reasons:
//!
//! * **Precision.** At 0.25 ms a 3 kW coolant input raises 12 kJ/K of coolant by
//!   6·10⁻⁵ K per step, the same order as the f32 spacing at 360 K (3·10⁻⁵ K). Rounding
//!   would stall the warm-up tens of kelvin short of the thermostat. At 20 ms the
//!   increment is 80× larger and the rounding error falls below 0.5 %.
//! * **Stability margin.** The fastest node (coolant against the metal at high pump
//!   speed, τ ≈ 3 s) is still 150 steps long, so forward Euler stays accurate to < 1 %.

use super::faults::ThermostatFault;
use super::math::{approach, clampf, smoothstep};
use super::spec::{ThermalSpec, MAX_CYLINDERS};
use super::thermo::{CP_AIR, CP_COOLANT, CP_EXHAUST};

/// Wax-element thermostat response time constant \[s\].
const THERMOSTAT_TAU_S: f32 = 8.0;
/// Share of friction power dissipated into the oil (bearings, cam/follower contacts);
/// the remainder (rings/skirts) goes to the cylinder walls and hence the coolant.
const FRICTION_TO_OIL: f32 = 0.6;
/// Radiator core air speed produced by the electric fan \[m/s\].
const FAN_AIR_SPEED: f32 = 5.0;
/// Natural-convection air speed through a stationary radiator \[m/s\].
const NATURAL_AIR_SPEED: f32 = 0.5;
/// Ambient air density used for radiator air-side capacity \[kg/m³\].
const AIR_DENSITY: f32 = 1.2;
/// Fraction of metal→coolant conductance remaining when the coolant film boils
/// (vapour blanketing: transition from nucleate to film boiling).
const FILM_BOILING_FACTOR: f32 = 0.35;
/// Coolant lost through the overflow per second per 10 K of superheat (fraction of fill).
const BOILOVER_RATE: f32 = 0.003;
/// Heat capacity of the cast-iron exhaust manifold (≈ 5 kg × 460 J/(kg·K)) \[J/K\].
const MANIFOLD_CAPACITY: f32 = 2_300.0;
/// Emissivity × radiating area of the exhaust manifold (oxidised cast iron ε ≈ 0.8,
/// ≈ 0.10 m² outer surface) \[m²\].
const MANIFOLD_EMISSIVE_AREA: f32 = 0.08;
/// Still-air convective conductance of the exhaust manifold \[W/K\].
const MANIFOLD_CONVECTION_UA: f32 = 8.0;
/// Stefan–Boltzmann constant [W/(m²·K⁴)].
const STEFAN_BOLTZMANN: f32 = 5.670_374e-8;
/// Catalyst monolith gas-side conductance at 0.02 kg/s exhaust flow \[W/K\]; laminar
/// channel flow with entrance effects scales roughly with ṁ^0.8.
const CATALYST_UA_REF: f32 = 100.0;
/// Still-air shell loss of the insulated, heat-shielded close-coupled converter: the
/// ≈ 6 mm intumescent mat (≈ 2.5 W/K over 0.15 m²) in series with skin convection and
/// radiation (≈ 2 W/K) \[W/K\].
const CATALYST_SHELL_UA: f32 = 1.0;
/// Growth of the catalyst shell loss per m/s of air speed. The converter sits behind the
/// engine under a heat shield, so radiator-fan and ram air reach it only weakly.
const CATALYST_SHELL_AIR_GAIN: f32 = 0.05;
/// Integration interval of the thermal network \[s\] (see module docs).
pub(crate) const THERMAL_STEP_S: f32 = 0.02;

/// Effectiveness of a single-pass cross-flow heat exchanger with both fluids unmixed
/// (finned-tube radiator), Incropera's approximation
/// `ε = 1 − exp[(NTU^0.22 / C_r)·(exp(−C_r·NTU^0.78) − 1)]`, which tends to
/// `1 − exp(−NTU)` as `C_r → 0`.
fn crossflow_unmixed_effectiveness(ntu: f32, c_r: f32) -> f32 {
    let ntu = ntu.max(0.0);
    if c_r < 1.0e-4 {
        return 1.0 - (-ntu).exp();
    }
    let c_r = c_r.min(1.0);
    clampf(
        1.0 - ((ntu.powf(0.22) / c_r) * ((-c_r * ntu.powf(0.78)).exp() - 1.0)).exp(),
        0.0,
        1.0,
    )
}

/// Thermal state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ThermalState {
    pub t_metal: f32,
    pub t_coolant: f32,
    pub t_oil: f32,
    pub thermostat_pos: f32,
    /// Coolant fill level 0..1.
    pub coolant_level: f32,
    pub t_catalyst: f32,
    pub t_piston: [f32; MAX_CYLINDERS],
    /// Exhaust manifold wall temperature \[K\].
    pub t_manifold: f32,
    pub radiator_heat_w: f32,
}

impl ThermalState {
    pub(crate) fn uniform(t: f32) -> Self {
        Self {
            t_metal: t,
            t_coolant: t,
            t_oil: t,
            thermostat_pos: 0.0,
            coolant_level: 1.0,
            t_catalyst: t,
            t_piston: [t; MAX_CYLINDERS],
            t_manifold: t,
            radiator_heat_w: 0.0,
        }
    }
}

/// Boundary conditions of one thermal step. Heat inputs are energies integrated over the
/// step; flows and speeds are averages over it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ThermalInputs {
    /// Combustion and exhaust-port heat deposited into the metal during this step \[J\].
    pub wall_heat_j: f32,
    /// Mechanical friction work dissipated during this step \[J\].
    pub friction_heat_j: f32,
    /// Mean engine speed \[rpm\].
    pub rpm: f32,
    pub ambient_t: f32,
    /// Ram/test-cell air speed through the radiator before the fan \[m/s\].
    pub ram_air_speed: f32,
    pub fan_on: bool,
    pub thermostat_fault: ThermostatFault,
    /// Coolant loss from damage (fraction of fill per second).
    pub coolant_leak_per_s: f32,
    /// Mean exhaust mass flow through the catalyst \[kg/s\].
    pub exhaust_mass_flow: f32,
    /// Mass-flow-weighted mean catalyst inlet temperature \[K\].
    pub exhaust_temp: f32,
    /// Catalytic exotherm deposited during this step \[J\].
    pub catalyst_exotherm_j: f32,
    /// Heat transferred from the exhaust gas to the manifold wall during this step \[J\].
    pub manifold_heat_j: f32,
    pub cylinders: usize,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ThermalModel {
    spec: ThermalSpec,
}

impl ThermalModel {
    pub(crate) fn new(spec: &ThermalSpec) -> Self {
        Self { spec: *spec }
    }

    /// Adds a combustion heat pulse to one piston crown \[J\].
    #[inline]
    pub(crate) fn add_piston_heat(&self, st: &mut ThermalState, cylinder: usize, joules: f32) {
        st.t_piston[cylinder] += joules / self.spec.piston_capacity;
    }

    pub(crate) fn step(&self, st: &mut ThermalState, inp: &ThermalInputs, dt: f32) {
        let s = &self.spec;
        let rpm = inp.rpm.max(0.0);
        let t_amb = inp.ambient_t;

        // Coolant pump: positive-displacement-like delivery ∝ speed.
        let pump_flow = s.pump_flow_per_krpm * rpm / 1000.0;
        let level = clampf(st.coolant_level, 0.0, 1.0);
        let boiling = st.t_coolant > s.coolant_boil_k;

        // Forced-convection coolant-side film coefficient h ∝ v^0.8 (Dittus–Boelter),
        // with a natural-convection floor when the pump is stopped.
        let flow_factor = 0.3 + 0.7 * (rpm / 3000.0).min(2.5).powf(0.8);
        let ua_mc = s.metal_coolant_ua
            * flow_factor
            * level.max(0.05)
            * if boiling { FILM_BOILING_FACTOR } else { 1.0 };
        let q_mc = ua_mc * (st.t_metal - st.t_coolant);
        let q_mo = s.metal_oil_ua * (1.0 + rpm / 3000.0) * (st.t_metal - st.t_oil);

        // Thermostat: wax element opening between start and full temperatures.
        let thermostat_target = match inp.thermostat_fault {
            ThermostatFault::None => {
                smoothstep(s.thermostat_start_k, s.thermostat_full_k, st.t_coolant)
            }
            ThermostatFault::StuckOpen => 1.0,
            ThermostatFault::StuckClosed => 0.0,
        };
        st.thermostat_pos = approach(st.thermostat_pos, thermostat_target, dt, THERMOSTAT_TAU_S);

        // Radiator as a cross-flow heat exchanger (ε–NTU): NTU = UA/C_min,
        // Q = ε(NTU, C_min/C_max)·C_min·(T_coolant − T_ambient).
        let mut air_speed = inp.ram_air_speed.max(NATURAL_AIR_SPEED);
        if inp.fan_on {
            air_speed = air_speed.max(FAN_AIR_SPEED);
        }
        // Air-side coefficient of louvered fins scales with velocity^0.6.
        let ua_rad = s.radiator_ua * (air_speed / 10.0).powf(0.6);
        let c_air = AIR_DENSITY * s.radiator_area_m2 * air_speed * CP_AIR;
        let c_cool = pump_flow * st.thermostat_pos * level * CP_COOLANT;
        let c_min = c_air.min(c_cool);
        let c_max = c_air.max(c_cool);
        let q_rad = if c_min > 1.0e-3 {
            crossflow_unmixed_effectiveness(ua_rad / c_min, c_min / c_max)
                * c_min
                * (st.t_coolant - t_amb)
        } else {
            0.0
        };
        st.radiator_heat_w = q_rad;

        // Convection from block and sump grows with air movement under the car.
        let q_ma = s.metal_ambient_ua * (1.0 + 0.1 * air_speed) * (st.t_metal - t_amb);
        let q_oa = s.oil_ambient_ua * (1.0 + 0.08 * air_speed) * (st.t_oil - t_amb);

        // Piston crowns cool into the oil (ring pack, skirt, oil jets); that heat is one of
        // the oil's main sources at load.
        let k_p = (s.piston_cooling_ua * dt / s.piston_capacity).min(1.0);
        let n = inp.cylinders.min(MAX_CYLINDERS);
        let mut piston_to_oil_j = 0.0;
        for t in &mut st.t_piston[..n] {
            let d = k_p * (*t - st.t_oil);
            *t -= d;
            piston_to_oil_j += d * s.piston_capacity;
        }

        let e_f = inp.friction_heat_j.max(0.0);
        st.t_metal += (inp.wall_heat_j.max(0.0) + (1.0 - FRICTION_TO_OIL) * e_f
            - (q_mc + q_mo + q_ma) * dt)
            / s.metal_capacity;
        st.t_coolant += (q_mc - q_rad) * dt / (s.coolant_capacity * level.max(0.05));
        st.t_oil += (FRICTION_TO_OIL * e_f + piston_to_oil_j + (q_mo - q_oa) * dt) / s.oil_capacity;

        // Coolant inventory: boil-over through the expansion tank and damage leaks.
        let mut loss = inp.coolant_leak_per_s.max(0.0);
        if boiling {
            loss += BOILOVER_RATE * ((st.t_coolant - s.coolant_boil_k) / 10.0).min(3.0);
        }
        st.coolant_level = clampf(level - loss * dt, 0.0, 1.0);

        // Catalyst: convective exchange with the exhaust (h ∝ ṁ^0.8 through the monolith
        // channels), exotherm from oxidising HC/CO, and shell losses to ambient. The gas
        // can at most reach the substrate temperature, so the exchange is capped by the
        // ε–NTU effectiveness of the monolith against the gas capacity rate (the substrate
        // is the C_r → 0 side): Q = ṁ·c_p·(1 − exp(−UA/(ṁ·c_p)))·(T_gas − T_cat).
        let mdot = inp.exhaust_mass_flow.max(0.0);
        let c_gas = mdot * CP_EXHAUST;
        let q_gas = if c_gas > 1.0e-6 {
            let ua_cat = CATALYST_UA_REF * (mdot / 0.02).powf(0.8);
            c_gas * (1.0 - (-ua_cat / c_gas).exp()) * (inp.exhaust_temp - st.t_catalyst)
        } else {
            0.0
        };
        let q_cat = q_gas
            - CATALYST_SHELL_UA
                * (1.0 + CATALYST_SHELL_AIR_GAIN * air_speed)
                * (st.t_catalyst - t_amb);
        st.t_catalyst += (q_cat * dt + inp.catalyst_exotherm_j.max(0.0)) / s.catalyst_capacity;

        // Exhaust manifold wall: heated by the gas, cooled by convection and by radiation
        // (dominant above ≈ 700 K, which is why manifolds glow at sustained full load).
        let tm = st.t_manifold;
        let q_rad = MANIFOLD_EMISSIVE_AREA * STEFAN_BOLTZMANN * (tm.powi(4) - t_amb.powi(4));
        let q_conv = MANIFOLD_CONVECTION_UA * (1.0 + 0.08 * air_speed) * (tm - t_amb);
        st.t_manifold += (inp.manifold_heat_j - (q_rad + q_conv) * dt) / MANIFOLD_CAPACITY;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine_sim::spec::ThermalSpec;

    #[test]
    fn idle_warm_up_reaches_thermostat_and_regulates() {
        let model = ThermalModel::new(&ThermalSpec::two_litre());
        let mut st = ThermalState::uniform(293.0);
        let dt = THERMAL_STEP_S;
        // What the plant delivers at a warm 800 rpm idle: ≈ 2.3 kW of combustion and port
        // heat to the walls, 0.7 kW of friction and 0.5 kW through the piston crowns.
        for _ in 0..(1800.0 / dt) as usize {
            let fan_on = st.t_coolant > 373.0;
            model.step(
                &mut st,
                &ThermalInputs {
                    wall_heat_j: 2800.0 * dt,
                    friction_heat_j: 700.0 * dt,
                    rpm: 800.0,
                    ambient_t: 293.0,
                    ram_air_speed: 0.0,
                    fan_on,
                    thermostat_fault: ThermostatFault::None,
                    coolant_leak_per_s: 0.0,
                    exhaust_mass_flow: 0.004,
                    exhaust_temp: 700.0,
                    catalyst_exotherm_j: 0.0,
                    manifold_heat_j: 0.0,
                    cylinders: 4,
                },
                dt,
            );
        }
        // The thermostat opens and regulates in its 88–98 °C band.
        assert!(
            st.t_coolant > 361.0 && st.t_coolant < 373.0,
            "coolant {}",
            st.t_coolant
        );
        assert!(st.thermostat_pos > 0.02, "thermostat {}", st.thermostat_pos);
    }
}

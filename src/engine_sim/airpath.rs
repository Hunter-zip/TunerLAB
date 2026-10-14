//! Mean-value air path: throttle, intake manifold, boost system, exhaust manifold and
//! turbocharger.
//!
//! Each gas volume is a lumped "filling-and-emptying" reservoir, isothermal within a step:
//! `dp/dt = (R·T/V)·(Σṁ_in − Σṁ_out)`. Orifice flows (throttle, leaks, turbine, wastegate,
//! blow-off valve) use the compressible isentropic flow function. Reservoir pressures are
//! advanced with a linearly-implicit (Rosenbrock-Euler) step,
//! `p⁺ = p + dt·f(p)/(1 − dt·∂f/∂p)`, because near pressure ratio 1 the flow function is
//! very steep and explicit Euler would oscillate at the 4 kHz step size.

use super::math::{approach, clampf};
use super::spec::{EngineSpec, ThrottleSpec, TurboSpec};
use super::thermo::{orifice_flow, CP_AIR, CP_EXHAUST, GAMMA_AIR, GAMMA_EXHAUST, R_AIR, R_EXHAUST};

/// Discharge coefficient of the idle-air valve and leak holes (sharp-edged orifice ≈ 0.6–0.8).
const AUX_DISCHARGE_COEFF: f32 = 0.75;
/// Idle-air valve stepper/solenoid response time \[s\].
const IDLE_VALVE_TAU_S: f32 = 0.1;
/// Thermal time constant of the intake manifold air temperature \[s\].
const MANIFOLD_TEMP_TAU_S: f32 = 1.5;
/// Exhaust manifold gas mixing / transport time constant \[s\].
const EXHAUST_TEMP_TAU_S: f32 = 0.08;
/// Pneumatic wastegate actuator time constant \[s\].
const WASTEGATE_TAU_S: f32 = 0.05;
/// Blow-off valve diaphragm time constant \[s\].
const BOV_TAU_S: f32 = 0.02;
/// Intercooler core thermal time constant \[s\].
const INTERCOOLER_TAU_S: f32 = 0.5;
/// Reverse-flow gain of the compressor in deep surge (fraction of forward flow scale).
const SURGE_BACKFLOW_GAIN: f32 = 0.15;
/// Compressor flow coefficient (relative to choke) at peak efficiency.
const COMPRESSOR_PEAK_PHI: f32 = 0.55;
/// Turbine blade-speed ratio U/c_s at peak efficiency (radial turbines peak at ≈ 0.7).
const TURBINE_PEAK_BSR: f32 = 0.65;
/// Shaft speed floor used in dω/dt = P/(J·ω) to avoid the singularity at standstill \[rad/s\].
const TURBO_OMEGA_FLOOR: f32 = 500.0;

/// Air-path state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AirPathState {
    pub throttle_pos: f32,
    pub idle_valve_pos: f32,
    pub p_boost: f32,
    pub t_boost: f32,
    pub p_man: f32,
    pub t_man: f32,
    pub p_exh: f32,
    pub t_exh: f32,
    /// Gas temperature downstream of the turbine (= `t_exh` on NA engines) \[K\].
    pub t_post_turbine: f32,
    pub turbo_omega: f32,
    pub wastegate_pos: f32,
    pub bov_pos: f32,
    pub compressor_surge: bool,
    pub compressor_pr: f32,
    pub turbine_power_w: f32,
    pub compressor_power_w: f32,
    pub mdot_throttle: f32,
    pub mdot_cyl: f32,
    pub mdot_compressor: f32,
    pub mdot_turbine: f32,
    pub mdot_wastegate: f32,
}

impl AirPathState {
    pub(crate) fn at_ambient(p_amb: f32, t_amb: f32) -> Self {
        Self {
            throttle_pos: 0.0,
            idle_valve_pos: 0.0,
            p_boost: p_amb,
            t_boost: t_amb,
            p_man: p_amb,
            t_man: t_amb,
            p_exh: p_amb,
            t_exh: t_amb,
            t_post_turbine: t_amb,
            turbo_omega: 0.0,
            wastegate_pos: 0.0,
            bov_pos: 0.0,
            compressor_surge: false,
            compressor_pr: 1.0,
            turbine_power_w: 0.0,
            compressor_power_w: 0.0,
            mdot_throttle: 0.0,
            mdot_cyl: 0.0,
            mdot_compressor: 0.0,
            mdot_turbine: 0.0,
            mdot_wastegate: 0.0,
        }
    }
}

/// Per-step boundary conditions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AirPathInputs {
    pub throttle_cmd: f32,
    pub idle_valve_cmd: f32,
    pub wastegate_duty: f32,
    pub ambient_p: f32,
    pub ambient_t: f32,
    /// Speed-density pumping coefficient: ṁ_cyl = k·p_man [kg/(s·Pa)].
    pub cylinder_flow_coeff: f32,
    pub exhaust_mass_flow: f32,
    /// Gas temperature leaving the exhaust manifold runners \[K\].
    pub exhaust_gas_temp: f32,
    pub vacuum_leak_area: f32,
    pub boost_leak_area: f32,
    pub exhaust_restriction: f32,
    pub wastegate_stuck_closed: bool,
    pub turbo_failed: bool,
    /// Temperature of surfaces that heat-soak the intake (engine metal) \[K\].
    pub engine_bay_temp: f32,
}

/// Air-path model parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AirPath {
    throttle: ThrottleSpec,
    closed_angle: f32,
    manifold_volume: f32,
    filter_k: f32,
    exhaust_k: f32,
    exhaust_volume: f32,
    turbo: Option<TurboSpec>,
}

/// Linearly-implicit Euler step for a scalar reservoir equation dp/dt = f(p).
/// Uses a central-difference Jacobian. For ∂f/∂p > 0 (e.g. compressor surge branch) it
/// degrades gracefully to explicit Euler instead of amplifying.
#[inline]
fn implicit_pressure_step(p: f32, dt: f32, lo: f32, hi: f32, f: impl Fn(f32) -> f32) -> f32 {
    let delta = (p * 2.0e-4).max(10.0);
    let f0 = f(p);
    let dfdp = (f(p + delta) - f(p - delta)) / (2.0 * delta);
    let denom = (1.0 - dt * dfdp).max(1.0);
    clampf(p + dt * f0 / denom, lo, hi)
}

impl AirPath {
    pub(crate) fn new(spec: &EngineSpec) -> Self {
        Self {
            throttle: spec.throttle,
            closed_angle: spec.throttle.closed_angle_deg.to_radians(),
            manifold_volume: spec.intake.manifold_volume_m3,
            filter_k: spec.intake.filter_restriction,
            exhaust_k: spec.exhaust.backpressure,
            exhaust_volume: spec.exhaust.manifold_volume_m3,
            turbo: spec.turbo,
        }
    }

    /// Effective flow area Cd·A of throttle plate + idle valve + leakage \[m²\].
    ///
    /// Thin butterfly plate in a round bore (Heywood App. C):
    /// `A(α) = π/4·D²·(1 − cos α / cos α₀)`, α measured from the plane normal to the bore,
    /// α₀ the closed-stop angle. The cosine law makes the first few degrees of opening
    /// dominate airflow, which is why drive-by-wire maps are progressive.
    pub(crate) fn effective_area(&self, throttle_pos: f32, idle_pos: f32) -> f32 {
        let a0 = self.closed_angle;
        let alpha = a0 + clampf(throttle_pos, 0.0, 1.0) * (core::f32::consts::FRAC_PI_2 - a0);
        let bore = self.throttle.bore_m;
        let plate = core::f32::consts::FRAC_PI_4 * bore * bore * (1.0 - alpha.cos() / a0.cos());
        self.throttle.discharge_coefficient * plate.max(0.0)
            + AUX_DISCHARGE_COEFF
                * (self.throttle.idle_valve_area_m2 * clampf(idle_pos, 0.0, 1.0)
                    + self.throttle.leak_area_m2)
    }

    pub(crate) fn step(&self, st: &mut AirPathState, inp: &AirPathInputs, dt: f32) {
        let p_amb = inp.ambient_p;
        let t_amb = inp.ambient_t;

        st.throttle_pos = approach(
            st.throttle_pos,
            clampf(inp.throttle_cmd, 0.0, 1.0),
            dt,
            self.throttle.actuator_time_constant_s,
        );
        st.idle_valve_pos = approach(
            st.idle_valve_pos,
            clampf(inp.idle_valve_cmd, 0.0, 1.0),
            dt,
            IDLE_VALVE_TAU_S,
        );
        let cd_a = self.effective_area(st.throttle_pos, st.idle_valve_pos);
        let leak_cd_a = AUX_DISCHARGE_COEFF * inp.vacuum_leak_area;

        // ---- Boost system (pre-throttle volume) --------------------------------------
        // Naturally aspirated engines have no reservoir between filter and throttle: the
        // filter's quadratic restriction Δp = k·ṁ² acts as an orifice of equivalent area
        // A_f = 1/√(2·ρ·k) in series with the throttle, combined as for incompressible
        // flow, 1/A² = 1/A_thr² + 1/A_f² (exact as Δp/p → 0, which is where the filter
        // matters: at wide-open throttle). Solving the series path inside the implicit
        // manifold step avoids the explicit filter↔throttle coupling, which at wide-open
        // throttle has a loop gain above one and would oscillate from step to step.
        let (p_up, t_up, path_cd_a) = if let Some(turbo) = &self.turbo {
            self.step_compressor_side(st, inp, turbo, cd_a, dt);
            (st.p_boost, st.t_boost, cd_a)
        } else {
            let rho_amb = p_amb / (R_AIR * t_amb);
            let path = if self.filter_k > 0.0 {
                let a_f = 1.0 / (2.0 * rho_amb * self.filter_k).sqrt();
                cd_a * a_f / (cd_a * cd_a + a_f * a_f).sqrt()
            } else {
                cd_a
            };
            (p_amb, t_amb, path)
        };

        // ---- Intake manifold -----------------------------------------------------------
        let k_cyl = inp.cylinder_flow_coeff.max(0.0);
        let t_man = st.t_man;
        let c_man = R_AIR * t_man / self.manifold_volume;
        let manifold_rhs = |p: f32| -> f32 {
            let thr = orifice_flow(path_cd_a, p_up, t_up, p, t_man, GAMMA_AIR, R_AIR);
            let leak = orifice_flow(leak_cd_a, p_amb, t_amb, p, t_man, GAMMA_AIR, R_AIR);
            c_man * (thr + leak - k_cyl * p)
        };
        st.p_man = implicit_pressure_step(st.p_man, dt, 500.0, 600_000.0, manifold_rhs);
        st.mdot_throttle = orifice_flow(path_cd_a, p_up, t_up, st.p_man, t_man, GAMMA_AIR, R_AIR);
        st.mdot_cyl = k_cyl * st.p_man;
        if self.turbo.is_none() {
            // Pressure between filter and throttle, for telemetry.
            let drop = self.filter_k * st.mdot_throttle * st.mdot_throttle.abs();
            st.p_boost = clampf(p_amb - drop, 0.5 * p_amb, 1.5 * p_amb);
            st.t_boost = t_amb;
            st.compressor_pr = 1.0;
            st.mdot_compressor = st.mdot_throttle;
        }

        // Manifold air temperature: inflow temperature plus heat soak from the engine,
        // which dominates at low flow (hot idle raises IAT by 5–15 K on real engines).
        let inflow_t = if st.mdot_throttle > 0.0 {
            st.t_boost
        } else {
            st.t_man
        };
        let soak =
            0.12 * (inp.engine_bay_temp - inflow_t) * (-st.mdot_throttle.max(0.0) / 0.01).exp();
        st.t_man = approach(st.t_man, inflow_t + soak, dt, MANIFOLD_TEMP_TAU_S);

        // ---- Exhaust -------------------------------------------------------------------
        st.t_exh = approach(st.t_exh, inp.exhaust_gas_temp, dt, EXHAUST_TEMP_TAU_S);
        let k_exh = self.exhaust_k * inp.exhaust_restriction.max(1.0);
        if let Some(turbo) = &self.turbo {
            self.step_turbine_side(st, inp, turbo, k_exh, dt);
        } else {
            let flow = inp.exhaust_mass_flow.max(0.0);
            let target = p_amb + k_exh * flow * flow;
            // Exhaust manifold filling time constant ≈ V·p/(R·T·ṁ) is a few milliseconds.
            st.p_exh = approach(st.p_exh, target, dt, 0.01);
            st.mdot_turbine = 0.0;
            st.mdot_wastegate = 0.0;
            st.t_post_turbine = st.t_exh;
        }
    }

    /// Compressor, intercooler, blow-off valve and boost-pipe reservoir.
    fn step_compressor_side(
        &self,
        st: &mut AirPathState,
        inp: &AirPathInputs,
        t: &TurboSpec,
        cd_a: f32,
        dt: f32,
    ) {
        let p_amb = inp.ambient_p;
        let t_amb = inp.ambient_t;
        let p_in =
            (p_amb - self.filter_k * st.mdot_compressor * st.mdot_compressor).max(0.5 * p_amb);
        let omega = st.turbo_omega;
        let (p_man, t_man) = (st.p_man, st.t_man);
        let t_boost = st.t_boost;

        // Blow-off valve: diaphragm referenced to manifold pressure; opens when the
        // throttle snaps shut and boost-minus-manifold exceeds the crack pressure.
        let dp_bov = st.p_boost - st.p_man;
        st.bov_pos = approach(
            st.bov_pos,
            clampf((dp_bov - t.bov_crack_pa) / t.bov_crack_pa, 0.0, 1.0),
            dt,
            BOV_TAU_S,
        );
        let bov_cd_a = AUX_DISCHARGE_COEFF * t.bov_area_m2 * st.bov_pos;
        let leak_cd_a = AUX_DISCHARGE_COEFF * inp.boost_leak_area;
        let failed = inp.turbo_failed;

        let c_boost = R_AIR * t_boost / t.boost_volume_m3;
        let rhs = |p: f32| -> f32 {
            let (comp, _, _) = compressor_flow(t, if failed { 0.0 } else { omega }, p_in, t_amb, p);
            let thr = orifice_flow(cd_a, p, t_boost, p_man, t_man, GAMMA_AIR, R_AIR);
            let bov = orifice_flow(bov_cd_a, p, t_boost, p_amb, t_amb, GAMMA_AIR, R_AIR);
            let leak = orifice_flow(leak_cd_a, p, t_boost, p_amb, t_amb, GAMMA_AIR, R_AIR);
            c_boost * (comp - thr - bov - leak)
        };
        st.p_boost = implicit_pressure_step(st.p_boost, dt, 0.4 * p_amb, 500_000.0, rhs);

        let (mdot, dh_is, surge) =
            compressor_flow(t, if failed { 0.0 } else { omega }, p_in, t_amb, st.p_boost);
        st.mdot_compressor = mdot;
        st.compressor_surge =
            surge && !failed && omega > 0.1 * t.max_speed_rpm * core::f32::consts::TAU / 60.0;
        st.compressor_pr = st.p_boost / p_in;

        // Isentropic efficiency: parabolic island around the peak-efficiency flow
        // coefficient, a standard reduced representation of a compressor map.
        let u = omega * 0.5 * t.compressor_diameter_m;
        let rho_in = p_in / (R_AIR * t_amb);
        let phi_rel = mdot
            / (rho_in
                * t.compressor_diameter_m
                * t.compressor_diameter_m
                * t.choke_flow_coefficient
                * u.max(20.0));
        let eta = clampf(
            t.compressor_peak_efficiency * (1.0 - ((phi_rel - COMPRESSOR_PEAK_PHI) / 0.6).powi(2)),
            0.45,
            t.compressor_peak_efficiency,
        );
        st.compressor_power_w = if failed {
            0.0
        } else {
            mdot.max(0.0) * dh_is.max(0.0) / eta
        };
        // Compressor outlet temperature T₂ = T₁·(1 + (PR^((γ−1)/γ) − 1)/η_c), then the
        // intercooler removes ε of the excess over ambient.
        let pr_term = (st.compressor_pr.max(1.0)).powf((GAMMA_AIR - 1.0) / GAMMA_AIR) - 1.0;
        let t_co = t_amb * (1.0 + pr_term / eta);
        let t_ic = t_co - t.intercooler_effectiveness * (t_co - t_amb);
        st.t_boost = approach(st.t_boost, t_ic, dt, INTERCOOLER_TAU_S);
    }

    /// Exhaust manifold reservoir, turbine, wastegate and rotor dynamics.
    fn step_turbine_side(
        &self,
        st: &mut AirPathState,
        inp: &AirPathInputs,
        t: &TurboSpec,
        k_exh: f32,
        dt: f32,
    ) {
        let p_amb = inp.ambient_p;
        let flow_out = st.mdot_turbine + st.mdot_wastegate;
        let p4 = p_amb + k_exh * flow_out * flow_out;
        let t3 = st.t_exh.max(300.0);

        // Pneumatic wastegate: the actuator sees boost gauge pressure, reduced by the
        // boost-control solenoid bleeding a duty-proportional share of it.
        let p_act = (st.p_boost - p_amb)
            * (1.0 - clampf(inp.wastegate_duty, 0.0, 1.0) * t.wastegate_bleed_authority);
        let wg_target = if inp.wastegate_stuck_closed {
            0.0
        } else {
            clampf(
                (p_act - t.wastegate_spring_pa) / t.wastegate_span_pa,
                0.0,
                1.0,
            )
        };
        st.wastegate_pos = approach(st.wastegate_pos, wg_target, dt, WASTEGATE_TAU_S);

        let turbine_cd_a = t.turbine_area_m2;
        let wg_cd_a = AUX_DISCHARGE_COEFF * t.wastegate_area_m2 * st.wastegate_pos;
        let m_in = inp.exhaust_mass_flow.max(0.0);
        let c_exh = R_EXHAUST * t3 / self.exhaust_volume;
        let rhs = |p3: f32| -> f32 {
            let turb = orifice_flow(turbine_cd_a, p3, t3, p4, t3, GAMMA_EXHAUST, R_EXHAUST);
            let wg = orifice_flow(wg_cd_a, p3, t3, p4, t3, GAMMA_EXHAUST, R_EXHAUST);
            c_exh * (m_in - turb - wg)
        };
        st.p_exh = implicit_pressure_step(st.p_exh, dt, 0.8 * p_amb, 600_000.0, rhs);
        st.mdot_turbine =
            orifice_flow(turbine_cd_a, st.p_exh, t3, p4, t3, GAMMA_EXHAUST, R_EXHAUST);
        st.mdot_wastegate = orifice_flow(wg_cd_a, st.p_exh, t3, p4, t3, GAMMA_EXHAUST, R_EXHAUST);

        // Turbine power P_t = ṁ·c_p·T₃·η_t·(1 − (p₄/p₃)^((γ−1)/γ)). Efficiency depends on
        // the blade-speed ratio U/c_s, c_s = √(2·Δh_is) being the isentropic spouting
        // velocity.
        let omega = st.turbo_omega;
        let power_t = if st.p_exh > p4 && st.mdot_turbine > 0.0 && !inp.turbo_failed {
            let expansion = 1.0 - (p4 / st.p_exh).powf((GAMMA_EXHAUST - 1.0) / GAMMA_EXHAUST);
            let dh_is = CP_EXHAUST * t3 * expansion;
            let c_s = (2.0 * dh_is).max(1.0).sqrt();
            let bsr = omega * 0.5 * t.turbine_diameter_m / c_s;
            let eta_t = clampf(
                t.turbine_peak_efficiency * (1.0 - ((bsr - TURBINE_PEAK_BSR) / 0.55).powi(2)),
                0.15,
                t.turbine_peak_efficiency,
            );
            st.mdot_turbine * dh_is * eta_t
        } else {
            0.0
        };
        st.turbine_power_w = power_t;
        // Enthalpy extracted by the turbine cools the gas; the wastegate stream bypasses
        // it and remixes downstream: T₄ = T₃ − P_t/((ṁ_t + ṁ_wg)·c_p).
        let total_flow = (st.mdot_turbine + st.mdot_wastegate).max(1.0e-4);
        st.t_post_turbine = (t3 - power_t / (total_flow * CP_EXHAUST)).max(inp.ambient_t);

        // Rotor: J·ω·dω/dt = P_t − P_c − k·ω² (bearing friction).
        if inp.turbo_failed {
            st.turbo_omega = approach(omega, 0.0, dt, 0.3);
        } else {
            let net = power_t - st.compressor_power_w - t.bearing_loss * omega * omega;
            let omega_max = t.max_speed_rpm * core::f32::consts::TAU / 60.0;
            st.turbo_omega = clampf(
                omega + net / (t.rotor_inertia_kg_m2 * omega.max(TURBO_OMEGA_FLOOR)) * dt,
                0.0,
                1.5 * omega_max,
            );
        }
    }
}

/// Compressor speed line model. With blade tip speed U and isentropic head
/// Δh_is = c_p·T₁·(PR^((γ−1)/γ) − 1), the head coefficient is ψ = Δh_is/U². A parabolic
/// speed line ψ = ψ₀·(1 − (φ/φ_c)²) inverted for the flow coefficient gives
/// `ṁ = ρ₁·D²·φ_c·√(U² − Δh_is/ψ₀)`. A negative radicand means the requested pressure ratio
/// exceeds what the wheel can sustain: surge, modelled as weak reverse flow.
/// Returns (ṁ, Δh_is, surge).
fn compressor_flow(
    t: &TurboSpec,
    omega: f32,
    p_in: f32,
    t_in: f32,
    p_out: f32,
) -> (f32, f32, bool) {
    let u = omega * 0.5 * t.compressor_diameter_m;
    let pr = p_out / p_in;
    let dh_is = CP_AIR * t_in * (pr.max(1.0e-3).powf((GAMMA_AIR - 1.0) / GAMMA_AIR) - 1.0);
    let rho = p_in / (R_AIR * t_in);
    let scale = rho * t.compressor_diameter_m * t.compressor_diameter_m * t.choke_flow_coefficient;
    let s = u * u - dh_is / t.head_coefficient;
    if s >= 0.0 {
        (scale * s.sqrt(), dh_is, false)
    } else {
        (-SURGE_BACKFLOW_GAIN * scale * (-s).sqrt(), dh_is, true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine_sim::thermo::STANDARD_PRESSURE_PA;

    fn inputs(throttle: f32, k_cyl: f32) -> AirPathInputs {
        AirPathInputs {
            throttle_cmd: throttle,
            idle_valve_cmd: 0.2,
            wastegate_duty: 0.0,
            ambient_p: STANDARD_PRESSURE_PA,
            ambient_t: 298.0,
            cylinder_flow_coeff: k_cyl,
            exhaust_mass_flow: 0.0,
            exhaust_gas_temp: 900.0,
            vacuum_leak_area: 0.0,
            boost_leak_area: 0.0,
            exhaust_restriction: 1.0,
            wastegate_stuck_closed: false,
            turbo_failed: false,
            engine_bay_temp: 350.0,
        }
    }

    #[test]
    fn closed_throttle_pulls_vacuum_and_wot_recovers() {
        let spec = EngineSpec::naturally_aspirated_2l();
        let ap = AirPath::new(&spec);
        let mut st = AirPathState::at_ambient(STANDARD_PRESSURE_PA, 298.0);
        // k for 800 rpm, VE 0.8: 0.8·V_d·n/120/(R·T).
        let k = 0.8 * 1.998e-3 * 800.0 / 120.0 / (R_AIR * 298.0);
        for _ in 0..8000 {
            ap.step(&mut st, &inputs(0.0, k), 2.5e-4);
        }
        assert!(
            st.p_man > 20_000.0 && st.p_man < 50_000.0,
            "idle MAP {}",
            st.p_man
        );
        let k_wot = 0.95 * 1.998e-3 * 4000.0 / 120.0 / (R_AIR * 298.0);
        for _ in 0..8000 {
            ap.step(&mut st, &inputs(1.0, k_wot), 2.5e-4);
        }
        assert!(st.p_man > 90_000.0, "WOT MAP {}", st.p_man);
        assert!(st.p_man.is_finite());
    }

    #[test]
    fn compressor_surges_at_low_speed_high_ratio() {
        let t = EngineSpec::turbocharged_2l().turbo.unwrap();
        let (m, _, surge) = compressor_flow(&t, 2000.0, 100_000.0, 298.0, 200_000.0);
        assert!(surge && m < 0.0);
        let (m2, _, surge2) = compressor_flow(&t, 18_000.0, 100_000.0, 298.0, 200_000.0);
        assert!(!surge2 && m2 > 0.1, "{m2}");
    }
}

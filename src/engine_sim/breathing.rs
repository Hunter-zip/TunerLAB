//! Intake-port physics: true volumetric efficiency, charge heating and fuel wall wetting.
//!
//! These functions describe the *hidden truth* that the student approximates with the
//! ECU's VE table and transient-fuel calibration.

use super::math::clampf;
use super::spec::EngineSpec;
use super::thermo::{GAMMA_AIR, GAMMA_EXHAUST, R_AIR, STANDARD_PRESSURE_PA};

/// Operating point for the volumetric-efficiency model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct BreathingInputs {
    pub rpm: f32,
    pub p_man: f32,
    pub p_exh: f32,
    pub t_man: f32,
    pub t_fresh: f32,
    pub cam_retard_deg: f32,
    pub valve_float: f32,
}

/// Fraction of the port-wall-to-manifold temperature difference picked up by the fresh
/// charge. Heat per unit mass ∝ h·A·t_res/m with h ∝ Re^0.8 ∝ ṁ^0.8, residence time
/// t_res ∝ 1/n and mass per cycle m ∝ ṁ/n, hence Q/m ∝ ṁ^−0.2. Air flow is approximated by
/// `p_man · n` (speed-density), normalised to 1 atm and 3000 rpm.
pub(crate) fn charge_heating_fraction(spec: &EngineSpec, rpm: f32, p_man: f32) -> f32 {
    let flow_ratio = (p_man / STANDARD_PRESSURE_PA) * (rpm.max(100.0) / 3000.0);
    clampf(
        spec.breathing.charge_heating * flow_ratio.max(0.02).powf(-0.2),
        0.02,
        0.45,
    )
}

/// Fresh-charge temperature after picking up heat in the hot intake port \[K\].
pub(crate) fn fresh_charge_temperature(
    spec: &EngineSpec,
    rpm: f32,
    p_man: f32,
    t_man: f32,
    t_wall: f32,
) -> f32 {
    t_man + charge_heating_fraction(spec, rpm, p_man) * (t_wall - t_man)
}

/// True volumetric efficiency referred to manifold density, `m_air = η_v · ρ_man · V_d`.
///
/// Product of physically separable effects:
///
/// 1. **Quasi-steady breathing + wave ram**: `η_0 + g_ram·exp(−((n − n_t)/w)²)`. The
///    runner's pressure wave arrives at the valve in phase with IVC only around the tuned
///    speed (Helmholtz resonator analogy, Heywood §7.6).
/// 2. **Overlap reversion**: `1 − L·e^(−n/n_o)`. At very low speed, exhaust gas flows back
///    into the intake during valve overlap.
/// 3. **Inlet Mach index choking** (Taylor): `Z = (B/D_v)²·Ū_p/(C_i·a)`; VE falls steeply
///    once the mean valve-curtain flow approaches sonic velocity. Modelled as
///    `1/(1 + (Z/Z_c)^6)`.
/// 4. **Residual-gas expansion** (Heywood eq. 6.6 for the ideal intake process):
///    `η_res = (r_c − (p_e/p_i)^(1/γ)) / (r_c − 1)`. At part load the residual left at
///    exhaust pressure expands into the low-pressure intake and displaces fresh charge;
///    under boost (p_i > p_e) the term exceeds 1 (scavenging).
/// 5. **Charge heating**: density ratio `T_man / T_fresh`.
/// 6. **Cam retard** (timing-chain stretch): later IVC shifts the ram peak up the speed
///    range and pushes charge back into the port at low speed.
/// 7. **Valve float**: valves bouncing off their seats lose trapped charge.
pub(crate) fn volumetric_efficiency(spec: &EngineSpec, i: &BreathingInputs) -> f32 {
    let b = &spec.breathing;
    let g = &spec.geometry;
    let n = i.rpm.max(50.0);

    let tuned = b.ram_tuned_rpm * (1.0 + i.cam_retard_deg / 60.0);
    let dn = (n - tuned) / b.ram_width_rpm;
    let ram = b.ram_gain * (-dn * dn).exp();

    let overlap = 1.0 - b.overlap_loss * (-n / b.overlap_decay_rpm).exp();

    let sound_speed = (GAMMA_AIR * R_AIR * i.t_man.max(200.0)).sqrt();
    let valve_area_ratio = g.bore_m * g.bore_m
        / (f32::from(spec.valves.intake_valves_per_cylinder.max(1))
            * spec.valves.intake_valve_diameter_m
            * spec.valves.intake_valve_diameter_m);
    let z = valve_area_ratio * g.mean_piston_speed(n) / (b.inlet_flow_coefficient * sound_speed);
    let choke = 1.0 / (1.0 + (z / b.critical_mach_index).powi(6));

    let rc = g.compression_ratio;
    let pr = clampf(i.p_exh / i.p_man.max(1000.0), 0.2, 10.0);
    let residual = clampf((rc - pr.powf(1.0 / GAMMA_EXHAUST)) / (rc - 1.0), 0.05, 1.15);

    let heating = i.t_man / i.t_fresh.max(i.t_man);

    let cam = 1.0 - 0.006 * i.cam_retard_deg * (-n / 3000.0).exp();
    let float = 1.0 - 0.35 * clampf(i.valve_float, 0.0, 1.0);

    ((b.base_ve + ram) * overlap * choke * residual * heating * cam * float).max(0.0)
}

/// Fraction X of injected fuel that lands on and wets the port wall (Aquino, 1981).
/// Cold walls evaporate less, and higher manifold pressure raises the fuel's boiling range
/// and suppresses flash evaporation.
pub(crate) fn wall_film_fraction(t_port: f32, p_man: f32) -> f32 {
    let cold = clampf((363.15 - t_port) / 90.0, 0.0, 1.4);
    clampf(
        (0.15 + 0.25 * cold) * (0.7 + 0.3 * p_man / STANDARD_PRESSURE_PA),
        0.05,
        0.8,
    )
}

/// Evaporation time constant τ of the port wall film \[s\] (Aquino X–τ model). The vapour
/// pressure of the heavy gasoline fractions falls roughly exponentially with temperature,
/// so τ rises steeply on a cold engine.
pub(crate) fn wall_film_tau(t_port: f32) -> f32 {
    let cold = clampf((363.15 - t_port) / 90.0, 0.0, 1.4);
    0.15 + 1.4 * cold.powf(1.5)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wot(spec: &EngineSpec, rpm: f32) -> f32 {
        let t_fresh = fresh_charge_temperature(spec, rpm, 100_000.0, 300.0, 363.0);
        volumetric_efficiency(
            spec,
            &BreathingInputs {
                rpm,
                p_man: 100_000.0,
                p_exh: 106_000.0,
                t_man: 300.0,
                t_fresh,
                cam_retard_deg: 0.0,
                valve_float: 0.0,
            },
        )
    }

    #[test]
    fn ve_curve_has_peak_near_tuned_speed() {
        let s = EngineSpec::naturally_aspirated_2l();
        let peak = wot(&s, 4600.0);
        assert!(peak > 0.9 && peak < 1.05, "{peak}");
        assert!(wot(&s, 2000.0) < peak);
        assert!(wot(&s, 7000.0) < peak);
    }

    #[test]
    fn part_load_residual_reduces_ve() {
        let s = EngineSpec::naturally_aspirated_2l();
        let mut i = BreathingInputs {
            rpm: 2000.0,
            p_man: 100_000.0,
            p_exh: 104_000.0,
            t_man: 300.0,
            t_fresh: 310.0,
            cam_retard_deg: 0.0,
            valve_float: 0.0,
        };
        let full = volumetric_efficiency(&s, &i);
        i.p_man = 35_000.0;
        assert!(volumetric_efficiency(&s, &i) < full * 0.92);
    }

    #[test]
    fn cold_port_wets_more() {
        assert!(wall_film_fraction(273.0, 60_000.0) > wall_film_fraction(363.0, 60_000.0));
        assert!(wall_film_tau(273.0) > 4.0 * wall_film_tau(363.0));
    }
}

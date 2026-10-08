//! Gas properties and compressible-flow relations used by the air path and combustion
//! models. All quantities are SI (Pa, K, kg, s, J).

/// Specific gas constant of dry air [J/(kg·K)]: R̄ / M = 8.314 462 / 0.028 964 7.
pub(crate) const R_AIR: f32 = 287.05;

/// Specific gas constant of gasoline combustion products [J/(kg·K)]. Stoichiometric
/// products (≈ 71 % N₂, 13 % H₂O, 12 % CO₂, 4 % other by mole) have a molar mass of
/// ≈ 28.7 g/mol, giving R = 8.314 / 0.0287 ≈ 289.7.
pub(crate) const R_EXHAUST: f32 = 289.7;

/// Ratio of specific heats of air at intake temperatures (diatomic ideal gas, 7/5).
pub(crate) const GAMMA_AIR: f32 = 1.4;

/// Ratio of specific heats of exhaust gas at 900–1300 K. Vibrational modes of CO₂/H₂O
/// raise cp with temperature; 1.33 is the standard value used in turbocharger matching.
pub(crate) const GAMMA_EXHAUST: f32 = 1.33;

/// Isobaric specific heat of air near 300 K [J/(kg·K)].
pub(crate) const CP_AIR: f32 = 1005.0;

/// Isobaric specific heat of exhaust gas, cp = γR/(γ−1) ≈ 1168 J/(kg·K).
pub(crate) const CP_EXHAUST: f32 = GAMMA_EXHAUST * R_EXHAUST / (GAMMA_EXHAUST - 1.0);

/// Specific heat of 50/50 ethylene-glycol/water coolant at ≈ 90 °C [J/(kg·K)].
pub(crate) const CP_COOLANT: f32 = 3500.0;

/// ISO 2533 standard sea-level pressure \[Pa\].
pub(crate) const STANDARD_PRESSURE_PA: f32 = 101_325.0;

/// 0 °C in kelvin.
pub(crate) const ZERO_CELSIUS_K: f32 = 273.15;

/// Standard gravitational acceleration \[m/s²\].
pub(crate) const GRAVITY: f32 = 9.806_65;

/// Pressure ratio above which the isentropic flow function is replaced by its secant to
/// zero. The exact Ψ(Pr) has an infinite slope at Pr → 1, which makes the manifold ODE
/// infinitely stiff at wide-open throttle. This is a *numerical* regularisation: throttle
/// Reynolds numbers stay above 10⁵ even at Δp ≈ 1 kPa, so real flow remains turbulent
/// (ṁ ∝ √Δp). Inside the band the secant under-predicts flow (−11 % at Pr = 0.988,
/// −43 % at 0.995), i.e. a given flow needs a slightly larger Δp — at most a few hundred
/// pascals of extra wide-open-throttle loss.
const LINEAR_FLOW_PR: f32 = 0.985;

/// Isentropic orifice flow function Ψ(Pr) (Heywood, *ICE Fundamentals*, App. C):
///
/// * sub-critical: `Ψ = √( 2γ/(γ−1) · (Pr^(2/γ) − Pr^((γ+1)/γ)) )`
/// * choked (Pr ≤ Pr* = (2/(γ+1))^(γ/(γ−1)), 0.528 for air):
///   `Ψ* = √γ · (2/(γ+1))^((γ+1)/(2(γ−1)))`, 0.6847 for air.
///
/// Mass flow then follows as `ṁ = Cd·A · p₀/√(R·T₀) · Ψ`.
pub(crate) fn flow_function(pr: f32, gamma: f32) -> f32 {
    let pr = pr.clamp(0.0, 1.0);
    let g1 = gamma - 1.0;
    let critical = (2.0 / (gamma + 1.0)).powf(gamma / g1);
    let subcritical = |p: f32| -> f32 {
        let v = 2.0 * gamma / g1 * (p.powf(2.0 / gamma) - p.powf((gamma + 1.0) / gamma));
        v.max(0.0).sqrt()
    };
    if pr <= critical {
        gamma.sqrt() * (2.0 / (gamma + 1.0)).powf((gamma + 1.0) / (2.0 * g1))
    } else if pr < LINEAR_FLOW_PR {
        subcritical(pr)
    } else {
        subcritical(LINEAR_FLOW_PR) * (1.0 - pr) / (1.0 - LINEAR_FLOW_PR)
    }
}

/// Signed compressible mass flow through an orifice of effective area `cd_area` \[m²\]
/// between two reservoirs. Positive when gas flows from `p_a` to `p_b`; the upstream
/// stagnation temperature of the respective donor side is used.
pub(crate) fn orifice_flow(
    cd_area: f32,
    p_a: f32,
    t_a: f32,
    p_b: f32,
    t_b: f32,
    gamma: f32,
    r_gas: f32,
) -> f32 {
    if cd_area <= 0.0 || p_a <= 0.0 || p_b <= 0.0 {
        return 0.0;
    }
    if p_a >= p_b {
        cd_area * p_a / (r_gas * t_a.max(1.0)).sqrt() * flow_function(p_b / p_a, gamma)
    } else {
        -cd_area * p_b / (r_gas * t_b.max(1.0)).sqrt() * flow_function(p_a / p_b, gamma)
    }
}

/// Saturation vapour pressure of water \[Pa\], Magnus–Tetens form with the Alduchov–Eskridge
/// (1996) coefficients: `p_sat = 610.94 · exp(17.625·T_c / (T_c + 243.04))`. Accurate to
/// < 0.4 % between −40 °C and +50 °C. Used to remove water vapour from the intake charge:
/// humid air displaces oxygen, which is why engines lose power on muggy days.
pub(crate) fn water_saturation_pressure(t_k: f32) -> f32 {
    let t_c = t_k - ZERO_CELSIUS_K;
    610.94 * (17.625 * t_c / (t_c + 243.04)).exp()
}

/// Dynamic viscosity of engine oil \[Pa·s\] from the Vogel equation
/// `μ = a · exp(b / (T − c))` (Vogel 1921). Three parameters reproduce the
/// several-orders-of-magnitude viscosity change of a multigrade oil between −20 °C and
/// 150 °C far better than a simple Arrhenius law.
pub(crate) fn vogel_viscosity(a: f32, b: f32, c: f32, t_k: f32) -> f32 {
    let denom = (t_k - c).max(5.0);
    a * (b / denom).exp()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn choked_flow_function_matches_textbook() {
        // Ψ* for γ = 1.4 is 0.6847 (Heywood App. C).
        assert!((flow_function(0.3, 1.4) - 0.6847).abs() < 1e-3);
        assert_eq!(flow_function(1.0, 1.4), 0.0);
        // Monotonically decreasing in the subsonic range.
        let mut prev = flow_function(0.53, 1.4);
        let mut pr = 0.55;
        while pr < 1.0 {
            let v = flow_function(pr, 1.4);
            assert!(v <= prev + 1e-6);
            prev = v;
            pr += 0.01;
        }
    }

    #[test]
    fn orifice_flow_is_antisymmetric() {
        let f1 = orifice_flow(1e-3, 100_000.0, 300.0, 60_000.0, 300.0, 1.4, R_AIR);
        let f2 = orifice_flow(1e-3, 60_000.0, 300.0, 100_000.0, 300.0, 1.4, R_AIR);
        assert!(f1 > 0.0);
        assert!((f1 + f2).abs() < 1e-6);
    }

    #[test]
    fn vapour_pressure_reference_points() {
        // 20 °C → 2339 Pa (steam tables).
        let p = water_saturation_pressure(293.15);
        assert!((p - 2339.0).abs() < 15.0, "{p}");
    }
}

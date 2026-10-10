//! Crank-angle-resolved closed-cycle combustion model of one cylinder.
//!
//! Evaluated once per cylinder per cycle (at the start of the compression stroke). It
//! integrates the single-zone first-law pressure equation from intake valve closing (IVC)
//! to exhaust valve opening (EVO) with a Wiebe heat-release law, and evaluates the
//! Livengood–Wu knock integral on the unburned end gas. MBT timing, knock-limited spark,
//! lean/rich misfire limits, exhaust temperature and the effect of retarded spark on EGT
//! are therefore *emergent* properties of the thermodynamics rather than lookup tables.

use super::math::{clampf, DEG_TO_RAD};
use super::spec::{CylinderGeometry, EngineSpec};

/// Integration steps from the start of the knock window to exhaust valve opening. The
/// window spans 195–215 crank degrees, i.e. ≈ 2° per step: fine enough for peak-pressure
/// location and the knock integral, cheap enough to run for every cylinder event
/// (≤ 600 per second on a V8 at 9000 rpm).
pub(crate) const CYCLE_STEPS: usize = 100;

/// Woschni (1967) heat-transfer constant for SI engines, SI units with p in kPa:
/// h = 3.26·B^−0.2·p^0.8·T^−0.55·w^0.8 [W/(m²·K)].
const WOSCHNI_COEFF: f32 = 3.26;
/// Woschni gas-velocity coefficient for compression/combustion/expansion.
const WOSCHNI_C1: f32 = 2.28;
/// Woschni combustion-turbulence coefficient [m/(s·K)].
const WOSCHNI_C2: f32 = 3.24e-3;

/// Start of the knock-integration window [deg ATDC]. Before ≈ 60° BTDC the end gas is
/// below ≈ 650 K, where the Douaud–Eyzat delay exceeds 50 ms, so its contribution to the
/// Livengood–Wu integral is < 0.5 % and can be neglected.
const KNOCK_WINDOW_START_DEG: f32 = -60.0;

/// Gas constant used for the in-cylinder charge [J/(kg·K)]. Fresh charge (287) and burned
/// gas (290) differ by 1 %, below the model's other uncertainties.
const R_CHARGE: f32 = 287.0;

/// Upper bound on λ reported for fuel-less cycles (keeps downstream arithmetic finite).
const MAX_LAMBDA: f32 = 9.99;

/// Metghalchi & Keck (1982) laminar burning velocity of iso-octane at 298 K / 1 atm:
/// `S_L = B_m + B_φ·(φ − φ_m)²` with B_m = 26.3 cm/s, B_φ = −84.7 cm/s, φ_m = 1.13.
/// Used only as a *ratio* against the reference mixture, so the absolute temperature and
/// pressure corrections cancel to first order.
const SL_BM: f32 = 0.263;
const SL_BPHI: f32 = -0.847;
const SL_PHI_M: f32 = 1.13;

/// Below this laminar flame speed the kernel cannot develop before the turbulence quenches
/// it. The Metghalchi–Keck curve crosses 6 cm/s at λ ≈ 0.62 and λ ≈ 1.56, matching the
/// practical rich/lean misfire limits of port-injected gasoline engines.
const MIN_FLAME_SPEED: f32 = 0.06;

/// Reference mixture of `CombustionSpec::burn_duration_deg`.
const REF_LAMBDA: f32 = 0.9;
/// Reference residual-gas fraction of `CombustionSpec::burn_duration_deg`.
const REF_RESIDUAL: f32 = 0.04;

/// Douaud & Eyzat (1978) end-gas ignition delay correlation for gasoline-type fuels:
/// `τ\[ms\] = 17.68 · (ON/100)^3.402 · p\[atm\]^−1.7 · exp(3800 / T\[K\])`.
const DE_A_MS: f32 = 17.68;
const DE_OCTANE_EXP: f32 = 3.402;
const DE_PRESSURE_EXP: f32 = -1.7;
const DE_ACTIVATION_K: f32 = 3800.0;

/// Converts autoignited unburned mass × pressure into a knock intensity comparable to the
/// MAPO (maximum amplitude of pressure oscillation) metric used on real knock rigs:
/// `KI\[bar\] = 0.25 · (1 − x_b,onset) · p_onset\[bar\]`. Late, small knock (x_b ≈ 0.95 at
/// 60 bar) gives ≈ 0.75 bar (trace/audible), early knock with 30 % unburned gives
/// ≈ 4.5 bar (heavy, destructive).
const KNOCK_MAPO_GAIN: f32 = 0.25;

/// Fraction of charge still unburned at EVO that keeps burning in the exhaust port and
/// manifold (afterburning). The rest leaves as hydrocarbons and CO for the catalyst.
const AFTERBURN_FRACTION: f32 = 0.7;

/// λ above which a fuel-starved cycle is classed as fuel cut (e.g. wall-film burn-off
/// during deceleration fuel cut-off) rather than a misfire.
const MISFIRE_LAMBDA_LIMIT: f32 = 2.5;

/// Heat-flux amplification on the piston crown per bar of knock intensity, capped at
/// [`KNOCK_HEAT_GAIN_MAX`]: detonation pressure waves scrub the thermal boundary layer, and
/// measured crown heat flux rises 2–4× in heavy knock (Heywood §9.6.1).
const KNOCK_HEAT_GAIN_PER_BAR: f32 = 0.6;
/// Upper bound of the knock heat-flux amplification.
const KNOCK_HEAT_GAIN_MAX: f32 = 4.0;

/// Extra polytropic index on the expansion of a motored (non-fired) cycle, accounting for
/// heat loss and blow-by so that motoring work is net negative as on a motoring dyno.
const MOTORING_EXPANSION_PENALTY: f32 = 0.04;

/// Inputs describing the trapped charge of one cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CycleInputs {
    /// Engine speed \[rpm\].
    pub rpm: f32,
    /// Fresh dry air trapped \[kg\].
    pub fresh_air_kg: f32,
    /// Evaporated fuel in the charge \[kg\].
    pub fuel_kg: f32,
    /// Residual burned gas retained from the previous cycle \[kg\].
    pub residual_kg: f32,
    /// Mixture temperature at IVC \[K\].
    pub t_ivc_k: f32,
    /// Spark advance [deg BTDC].
    pub spark_btdc_deg: f32,
    /// Spark commanded by the ECU (not cut).
    pub spark_enabled: bool,
    /// Secondary voltage the coil can deliver this cycle \[kV\].
    pub spark_voltage_kv: f32,
    /// Standard-normal noise on the required breakdown voltage.
    pub breakdown_noise: f32,
    /// Fuel research octane number.
    pub octane_ron: f32,
    /// Combustion-chamber wall temperature \[K\].
    pub wall_temp_k: f32,
    /// Exhaust manifold pressure \[Pa\].
    pub exhaust_pressure_pa: f32,
    /// Cycle-to-cycle multiplier on burn duration (≈ 1).
    pub burn_ccv: f32,
    /// Cycle-to-cycle multiplier on end-gas ignition delay (≈ 1).
    pub knock_ccv: f32,
    /// Fraction of the trapped charge lost past rings/valves/gasket 0..1.
    pub compression_leak: f32,
}

/// Outcome of one closed cycle.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct CycleResult {
    pub lambda: f32,
    pub misfire: bool,
    pub partial_burn: bool,
    pub burn_duration_deg: f32,
    pub mass_fraction_burned_evo: f32,
    /// ∫p·dV over the compression stroke (negative) \[J\].
    pub work_compression_j: f32,
    /// ∫p·dV over the expansion stroke (positive) \[J\].
    pub work_expansion_j: f32,
    /// Net closed-cycle indicated work IVC→EVO \[J\].
    pub net_work_j: f32,
    pub fuel_energy_j: f32,
    pub heat_released_j: f32,
    /// Total in-cylinder heat transfer to the walls, including [`Self::knock_wall_heat_j`] \[J\].
    pub wall_heat_j: f32,
    /// Extra piston-crown heat caused by knock (part of `wall_heat_j`) \[J\].
    pub knock_wall_heat_j: f32,
    pub afterburn_j: f32,
    pub unburned_fuel_energy_j: f32,
    pub unused_air_kg: f32,
    pub peak_pressure_pa: f32,
    pub peak_pressure_angle_deg: f32,
    pub knock_intensity: f32,
    pub knock_onset_deg: f32,
    pub exhaust_temp_k: f32,
    pub exhaust_mass_kg: f32,
    pub blowdown_pressure_pa: f32,
    pub residual_next_kg: f32,
    pub residual_temp_next_k: f32,
}

/// Precomputed combustion model of one cylinder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CombustionModel {
    geom: CylinderGeometry,
    theta_ivc_deg: f32,
    theta_evo_deg: f32,
    v_ivc: f32,
    v_evo: f32,
    burn_ref_deg: f32,
    burn_ref_rpm: f32,
    wiebe_a: f32,
    wiebe_exp: f32,
    wiebe_norm: f32,
    bore: f32,
    stroke: f32,
    woschni_prefactor: f32,
    knock_resistance: f32,
    piston_heat_share: f32,
    n_c: f32,
    stoich_afr: f32,
    lhv: f32,
    sl_ref: f32,
    dilution_ref: f32,
}

/// Laminar burning velocity ratio basis (see [`SL_BM`]).
#[inline]
fn laminar_flame_speed(phi: f32) -> f32 {
    let d = phi - SL_PHI_M;
    (SL_BM + SL_BPHI * d * d).max(0.0)
}

/// Metghalchi–Keck dilution factor `1 − 2.06·x_r^0.77`: burned residual gas lowers the
/// flame temperature and the laminar burning velocity.
#[inline]
fn dilution_factor(x_r: f32) -> f32 {
    (1.0 - 2.06 * x_r.max(0.0).powf(0.77)).max(0.1)
}

/// Fraction of the fuel's chemical energy released by in-cylinder combustion.
///
/// * λ ≥ 1: fuel-limited; ≈ 2 % escapes as crevice hydrocarbons (Heywood §4.9.4).
/// * λ < 1: oxygen-limited. Per unit of *air*, the heat release first rises slightly
///   below λ = 1 (excess fuel consumes the O₂ that dissociation leaves unused at
///   stoichiometric) and then falls, because further O₂ forms CO instead of CO₂, which
///   liberates only ≈ 28 % of the heat per mole of O₂. The quadratic fit
///   `0.98·λ·(1 + 0.30x − 1.6x²)`, x = 1 − λ, peaks per unit air at λ ≈ 0.91 (+1.4 %) and
///   gives η ≈ 0.87 at λ = 0.88, consistent with exhaust CO/H₂ measurements.
#[inline]
pub(crate) fn combustion_efficiency(lambda: f32) -> f32 {
    if lambda >= 1.0 {
        0.98
    } else {
        let x = 1.0 - lambda;
        (0.98 * lambda * (1.0 + 0.30 * x - 1.6 * x * x)).max(0.0)
    }
}

/// Ratio of specific heats of the burned charge as a function of temperature and mixture.
///
/// Brunt et al. (1998) single-zone correlation `γ(T) = 1.338 − 6.0·10⁻⁵·T + 1.0·10⁻⁸·T²`
/// captures the rise of c_v with temperature as vibrational modes of CO₂/H₂O are excited
/// (γ ≈ 1.32 at 300 K, 1.29 at 1000 K, 1.25 at 2500 K). A constant γ would overestimate
/// peak and exhaust temperatures by several hundred kelvin. Lean mixtures contain more
/// diatomic N₂/O₂ (higher γ, higher cycle efficiency): +0.04 per unit λ above 1.
#[inline]
fn burned_gamma(t: f32, lambda: f32) -> f32 {
    let t = clampf(t, 250.0, 3000.0);
    clampf(
        1.338 - 6.0e-5 * t + 1.0e-8 * t * t + 0.04 * (lambda.min(2.0) - 1.0),
        1.20,
        1.38,
    )
}

/// Empirical mixture effect on end-gas ignition delay. Knock tendency is highest near
/// stoichiometric; enrichment lowers end-gas temperature (higher heat capacity, latent
/// heat) and slows low-temperature chemistry, which is why tuners run λ 0.78–0.85 under
/// boost. Lean mixtures burn cooler and also knock less (Heywood §9.6.2).
#[inline]
fn knock_mixture_factor(lambda: f32) -> f32 {
    if lambda < 1.0 {
        (1.2 * (1.0 - lambda)).exp()
    } else {
        1.0 + 0.4 * (lambda.min(2.0) - 1.0)
    }
}

impl CombustionModel {
    pub(crate) fn new(spec: &EngineSpec) -> Self {
        let geom = CylinderGeometry::from_spec(&spec.geometry);
        let theta_ivc_deg = -180.0 + spec.valves.ivc_abdc_deg;
        let theta_evo_deg = 180.0 - spec.valves.evo_bbdc_deg;
        let c = &spec.combustion;
        let wiebe_exp = c.wiebe_m + 1.0;
        let wiebe_norm = 1.0 / (1.0 - (-c.wiebe_a).exp());
        Self {
            geom,
            theta_ivc_deg,
            theta_evo_deg,
            v_ivc: geom.volume(theta_ivc_deg * DEG_TO_RAD),
            v_evo: geom.volume(theta_evo_deg * DEG_TO_RAD),
            burn_ref_deg: c.burn_duration_deg,
            burn_ref_rpm: c.burn_reference_rpm,
            wiebe_a: c.wiebe_a,
            wiebe_exp,
            wiebe_norm,
            bore: spec.geometry.bore_m,
            stroke: spec.geometry.stroke_m,
            woschni_prefactor: WOSCHNI_COEFF
                * c.heat_transfer_scale
                * spec.geometry.bore_m.powf(-0.2),
            knock_resistance: c.knock_resistance,
            piston_heat_share: clampf(spec.thermal.piston_heat_share, 0.0, 1.0),
            n_c: c.compression_index,
            stoich_afr: spec.fuel.stoich_afr,
            lhv: spec.fuel.lower_heating_value,
            sl_ref: laminar_flame_speed(1.0 / REF_LAMBDA),
            dilution_ref: dilution_factor(REF_RESIDUAL),
        }
    }

    /// Mean piston speed Ū_p = 2·S·n/60 \[m/s\].
    #[inline]
    fn mean_piston_speed(&self, rpm: f32) -> f32 {
        2.0 * self.stroke * rpm / 60.0
    }

    /// Effective polytropic compression index. At cranking and idle speeds the charge has
    /// several times longer to lose heat to the walls during compression, lowering the
    /// apparent index from ≈ 1.32 to ≈ 1.27 (Heywood §10.6, motored pressure analysis).
    #[inline]
    fn compression_index(&self, rpm: f32) -> f32 {
        self.n_c - 0.07 * (-rpm / 500.0).exp()
    }

    /// Normalised Wiebe mass-fraction-burned function
    /// `x_b = (1 − exp(−a·((θ − θ_s)/Δθ)^(m+1))) / (1 − e^(−a))`, saturating at 1.
    #[inline]
    fn wiebe(&self, theta_deg: f32, spark_deg: f32, duration_deg: f32) -> f32 {
        if theta_deg <= spark_deg {
            return 0.0;
        }
        let r = (theta_deg - spark_deg) / duration_deg;
        if r >= 1.0 {
            1.0
        } else {
            ((1.0 - (-self.wiebe_a * r.powf(self.wiebe_exp)).exp()) * self.wiebe_norm).min(1.0)
        }
    }

    /// Burn duration [crank deg] for given speed, mixture and dilution (see module docs).
    ///
    /// `Δθ = Δθ_ref · (n/n_ref)^0.2 · (S_L,ref/S_L)^0.6 · D_ref/D(x_r)`
    ///
    /// * Speed: turbulence intensity scales with piston speed, so burn *time* falls almost
    ///   as 1/n and burn *angle* grows only weakly (exponent 0.2, Heywood Fig. 9-29:
    ///   10–90 % burn angle rises by only ≈ 15 % from 2000 to 6000 rpm).
    /// * Mixture: turbulent flame speed scales sub-linearly with laminar speed
    ///   (S_T ∝ S_L^0.3…0.5 · u'^0.5…0.7); exponent 0.6 on the duration ratio reproduces
    ///   the measured ≈ 35 % longer burn at λ = 1.3 and ≈ 15 % at λ = 0.7.
    /// * Dilution: Metghalchi–Keck residual-gas factor.
    pub(crate) fn burn_duration_deg(&self, rpm: f32, lambda: f32, residual_fraction: f32) -> f32 {
        let s_l = laminar_flame_speed(1.0 / lambda.max(0.1)).max(1.0e-3);
        let g_n = (rpm.max(300.0) / self.burn_ref_rpm).powf(0.2);
        let g_l = (self.sl_ref / s_l).powf(0.6);
        let g_d = self.dilution_ref / dilution_factor(residual_fraction);
        self.burn_ref_deg * g_n * g_l * g_d
    }

    /// Spark advance for maximum brake torque [deg BTDC] for the given charge, found by
    /// golden-section search on net indicated work with knock and cycle-to-cycle variation
    /// suppressed (MBT is defined on the mean cycle, independent of the knock limit).
    /// Allocation-free; ≈ 14 cycle evaluations.
    pub(crate) fn find_mbt(&self, inp: &CycleInputs) -> f32 {
        const INV_PHI: f32 = 0.618_034;
        let base = CycleInputs {
            spark_enabled: true,
            spark_voltage_kv: 1.0e3,
            breakdown_noise: 0.0,
            octane_ron: 200.0,
            burn_ccv: 1.0,
            knock_ccv: 1.0,
            ..*inp
        };
        let work = |s: f32| {
            self.simulate(&CycleInputs {
                spark_btdc_deg: s,
                ..base
            })
            .net_work_j
        };
        let (mut lo, mut hi) = (-10.0_f32, 65.0_f32);
        let mut x1 = hi - INV_PHI * (hi - lo);
        let mut x2 = lo + INV_PHI * (hi - lo);
        let mut f1 = work(x1);
        let mut f2 = work(x2);
        for _ in 0..12 {
            if f1 < f2 {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = lo + INV_PHI * (hi - lo);
                f2 = work(x2);
            } else {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = hi - INV_PHI * (hi - lo);
                f1 = work(x1);
            }
        }
        0.5 * (lo + hi)
    }

    /// Simulates one closed cycle.
    pub(crate) fn simulate(&self, inp: &CycleInputs) -> CycleResult {
        let mut out = CycleResult::default();
        let leak = clampf(inp.compression_leak, 0.0, 1.0);
        let fresh = inp.fresh_air_kg.max(0.0);
        let fuel = inp.fuel_kg.max(0.0);
        let resid = inp.residual_kg.max(0.0);
        let total = fresh + fuel + resid;
        out.fuel_energy_j = fuel * self.lhv;
        out.lambda = if fuel > 1.0e-12 {
            (fresh / (fuel * self.stoich_afr)).min(MAX_LAMBDA)
        } else {
            MAX_LAMBDA
        };
        let m = total * (1.0 - leak);
        if m < 1.0e-9 || total < 1.0e-9 {
            out.misfire = fuel > 1.0e-10;
            out.unburned_fuel_energy_j = out.fuel_energy_j;
            out.unused_air_kg = fresh;
            out.exhaust_temp_k = inp.t_ivc_k;
            out.exhaust_mass_kg = fresh + fuel;
            out.residual_temp_next_k = inp.t_ivc_k;
            return out;
        }
        let t_ivc = inp.t_ivc_k.max(200.0);
        let p_ivc = m * R_CHARGE * t_ivc / self.v_ivc;
        let x_r = resid / total;
        let lambda = out.lambda;
        let rpm = inp.rpm.max(30.0);

        let spark_deg = clampf(-inp.spark_btdc_deg, -80.0, 45.0);
        let v_spark = self.geom.volume(spark_deg * DEG_TO_RAD);
        let n_spark = self.compression_index(rpm);
        let volume_ratio = self.v_ivc / v_spark;
        let p_spark = p_ivc * volume_ratio.powf(n_spark);
        let t_spark = t_ivc * volume_ratio.powf(n_spark - 1.0);
        // Paschen's law: breakdown voltage of a uniform gap grows linearly with gas
        // *density* × gap (≈ 30 kV/cm at standard density). For a 0.8 mm plug gap:
        // V ≈ 1.5 kV + 2.4 kV × (ρ/ρ₀). Idle needs ≈ 6 kV, NA full load ≈ 16 kV,
        // 2 bar boost ≈ 30 kV (Bosch Gasoline-Engine Management, ignition chapter).
        let density_ratio = (p_spark / 101_325.0) * (293.0 / t_spark.max(250.0));
        let breakdown_kv = (1.5 + 2.4 * density_ratio) * (1.0 + 0.06 * inp.breakdown_noise);
        let sparked = inp.spark_enabled && breakdown_kv <= inp.spark_voltage_kv;
        let s_l = laminar_flame_speed(1.0 / lambda);
        let ignitable = s_l >= MIN_FLAME_SPEED && fuel > 1.0e-10 && leak < 0.7;
        if !(sparked && ignitable) {
            return self.motoring(out, inp, m, p_ivc, fresh, fuel, leak);
        }

        let n_c = self.compression_index(rpm);
        let duration = clampf(
            self.burn_duration_deg(rpm, lambda, x_r) * inp.burn_ccv,
            20.0,
            250.0,
        );
        out.burn_duration_deg = duration;

        let fuel_trapped = fuel * (1.0 - leak);
        let q_total = fuel_trapped * self.lhv * combustion_efficiency(lambda);

        // Integration window: from just before the spark (or the knock-relevant part of
        // compression) to exhaust valve opening.
        let theta_a = (spark_deg - 1.0)
            .min(KNOCK_WINDOW_START_DEG)
            .max(self.theta_ivc_deg + 1.0);
        let theta_b = self.theta_evo_deg;

        // IVC → window start: polytropic compression, W = (p₂V₂ − p₁V₁)/(1 − n). The
        // polytropic index already contains the (small) early-compression heat loss.
        let v_a = self.geom.volume(theta_a * DEG_TO_RAD);
        let p_a = p_ivc * (self.v_ivc / v_a).powf(n_c);
        let mut w_comp = (p_a * v_a - p_ivc * self.v_ivc) / (1.0 - n_c);
        let mut w_exp = 0.0_f32;

        let step = (theta_b - theta_a) / CYCLE_STEPS as f32;
        // Time per crank degree: 1/(6·n) s.
        let s_per_deg = 1.0 / (6.0 * rpm);
        let ms_per_deg = 1000.0 * s_per_deg;
        let tau_base = DE_A_MS
            * (inp.octane_ron / 100.0).powf(DE_OCTANE_EXP)
            * self.knock_resistance
            * knock_mixture_factor(lambda)
            * inp.knock_ccv;
        let unburned_exp = (n_c - 1.0) / n_c;
        // Woschni gas velocity: w = C₁·Ū_p + C₂·(V_d·T_r/(p_r·V_r))·(p − p_motored), with
        // C₁ = 2.28 (compression/expansion) and C₂ = 3.24·10⁻³ m/(s·K) (combustion-induced
        // turbulence), reference state at IVC.
        let piston_speed = self.mean_piston_speed(rpm);
        let combustion_velocity_coeff =
            WOSCHNI_C2 * self.geom.swept_volume * t_ivc / (p_ivc * self.v_ivc);
        let wall_temp = inp.wall_temp_k.max(250.0);

        let mut p = p_a;
        let mut v = v_a;
        let mut xb_prev = self.wiebe(theta_a, spark_deg, duration);
        let mut p_max = p_a;
        let mut theta_pmax = theta_a;
        let mut knock_integral = 0.0_f32;
        let mut knock_found = false;
        let mut wall_heat = 0.0_f32;
        let mut knock_wall_heat = 0.0_f32;
        // Crown heat-flux multiplier, raised from 1 once the end gas has autoignited.
        let mut knock_heat_gain = 1.0_f32;
        let mut gamma_last = burned_gamma(p_a * v_a / (m * R_CHARGE), lambda);
        for k in 0..CYCLE_STEPS {
            let theta1 = theta_a + step * (k + 1) as f32;
            let theta_mid = theta1 - 0.5 * step;
            let v1 = self.geom.volume(theta1 * DEG_TO_RAD);
            let xb1 = self.wiebe(theta1, spark_deg, duration);
            let dq_comb = q_total * (xb1 - xb_prev);
            let xb_mid = 0.5 * (xb_prev + xb1);
            let t_gas = p * v / (m * R_CHARGE);

            // Convective wall heat transfer, Woschni (1967):
            // h = 3.26·B^−0.2·p[kPa]^0.8·T^−0.55·w^0.8  [W/(m²·K)], chamber surface
            // A = head + piston crown + exposed liner ≈ 2·A_p + π·B·V/A_p.
            let p_motored = p_ivc * (self.v_ivc / v).powf(n_c);
            let w =
                WOSCHNI_C1 * piston_speed + combustion_velocity_coeff * (p - p_motored).max(0.0);
            let h = self.woschni_prefactor
                * (p * 1.0e-3).max(1.0).powf(0.8)
                * t_gas.max(250.0).powf(-0.55)
                * w.max(0.1).powf(0.8);
            let area = 2.0 * self.geom.piston_area
                + core::f32::consts::PI * self.bore * (v / self.geom.piston_area);
            let dq_base = h * area * (t_gas - wall_temp) * step * s_per_deg;
            let dq_knock = dq_base.max(0.0) * self.piston_heat_share * (knock_heat_gain - 1.0);
            let dq_wall = dq_base + dq_knock;
            wall_heat += dq_wall;
            knock_wall_heat += dq_knock;

            // Single-zone γ(T) of the whole charge (Brunt's correlation is fitted for exactly
            // this use, compression through expansion). Blending from the polytropic index
            // n_c by mass fraction burned would double-count the compression heat loss that
            // Woschni now models explicitly, and a γ that jumps with x_b rather than with
            // state changes the internal energy pV/(γ−1) without any heat being added.
            let gamma = burned_gamma(t_gas, lambda);
            gamma_last = gamma;
            // Operator splitting of dp = −γ·p·dV/V + (γ−1)·(dQ_comb − dQ_wall)/V: exact
            // isentropic volume change followed by constant-volume net heat addition.
            // Unconditionally stable for any step size.
            let p1 =
                (p * (v / v1).powf(gamma) + (gamma - 1.0) * (dq_comb - dq_wall) / v1).max(1000.0);
            let dw = 0.5 * (p + p1) * (v1 - v);
            if theta_mid < 0.0 {
                w_comp += dw;
            } else {
                w_exp += dw;
            }
            if p1 > p_max {
                p_max = p1;
                theta_pmax = theta1;
            }
            if !knock_found && xb_mid < 0.97 && theta_mid < spark_deg + duration {
                // Livengood–Wu: autoignition when ∫ dt/τ(p, T_u) reaches 1. The end gas is
                // compressed polytropically by the advancing flame, T_u = T_ivc·(p/p_ivc)^((n−1)/n).
                let p_mid = 0.5 * (p + p1);
                let t_u = t_ivc * (p_mid / p_ivc).powf(unburned_exp);
                let tau_ms = tau_base
                    * (p_mid / 101_325.0).powf(DE_PRESSURE_EXP)
                    * (DE_ACTIVATION_K / t_u).exp();
                knock_integral += step * ms_per_deg / tau_ms.max(1.0e-6);
                if knock_integral >= 1.0 {
                    knock_found = true;
                    out.knock_onset_deg = theta_mid;
                    out.knock_intensity = KNOCK_MAPO_GAIN * (1.0 - xb_mid) * p_mid * 1.0e-5;
                    knock_heat_gain = (1.0 + KNOCK_HEAT_GAIN_PER_BAR * out.knock_intensity)
                        .min(KNOCK_HEAT_GAIN_MAX);
                }
            }
            p = p1;
            v = v1;
            xb_prev = xb1;
        }
        let (p_evo, v_evo) = (p, v);

        let xb_evo = self.wiebe(self.theta_evo_deg, spark_deg, duration);
        let released = q_total * xb_evo;
        let afterburn = AFTERBURN_FRACTION * q_total * (1.0 - xb_evo);
        let t_evo = p_evo * v_evo / (m * R_CHARGE);
        let (t_exh, m_out, m_res, t_res) = self.exhaust_state(
            m,
            p_evo,
            v_evo,
            t_evo,
            gamma_last,
            inp.exhaust_pressure_pa,
            afterburn,
        );

        let air_trapped = fresh * (1.0 - leak);
        let burn_fraction = xb_evo + AFTERBURN_FRACTION * (1.0 - xb_evo);
        let consumed_air = air_trapped.min(fuel_trapped * self.stoich_afr) * burn_fraction;

        out.partial_burn = xb_evo < 0.9;
        out.mass_fraction_burned_evo = xb_evo;
        out.work_compression_j = w_comp;
        out.work_expansion_j = w_exp;
        out.net_work_j = w_comp + w_exp;
        out.heat_released_j = released;
        out.wall_heat_j = wall_heat.max(0.0);
        out.knock_wall_heat_j = clampf(knock_wall_heat, 0.0, out.wall_heat_j);
        out.afterburn_j = afterburn;
        out.unburned_fuel_energy_j = (out.fuel_energy_j - released - afterburn).max(0.0);
        out.unused_air_kg = (air_trapped - consumed_air).max(0.0);
        out.peak_pressure_pa = p_max;
        out.peak_pressure_angle_deg = theta_pmax;
        out.exhaust_temp_k = t_exh;
        out.exhaust_mass_kg = m_out;
        out.blowdown_pressure_pa = p_evo;
        out.residual_next_kg = m_res;
        out.residual_temp_next_k = t_res;
        out
    }

    /// Non-fired cycle: compression to TDC and lossy expansion to EVO.
    #[allow(clippy::too_many_arguments)]
    fn motoring(
        &self,
        mut out: CycleResult,
        inp: &CycleInputs,
        m: f32,
        p_ivc: f32,
        fresh: f32,
        fuel: f32,
        leak: f32,
    ) -> CycleResult {
        let n_c = self.compression_index(inp.rpm);
        let n_e = n_c + MOTORING_EXPANSION_PENALTY;
        let v_c = self.geom.clearance_volume;
        let p_tdc = p_ivc * (self.v_ivc / v_c).powf(n_c);
        let w_c = (p_tdc * v_c - p_ivc * self.v_ivc) / (1.0 - n_c);
        let p_evo = p_tdc * (v_c / self.v_evo).powf(n_e);
        let w_e = (p_evo * self.v_evo - p_tdc * v_c) / (1.0 - n_e);
        let t_evo = p_evo * self.v_evo / (m * R_CHARGE);
        let (t_exh, m_out, m_res, t_res) = self.exhaust_state(
            m,
            p_evo,
            self.v_evo,
            t_evo,
            n_c,
            inp.exhaust_pressure_pa,
            0.0,
        );
        out.misfire = fuel > 1.0e-10 && out.lambda < MISFIRE_LAMBDA_LIMIT;
        out.work_compression_j = w_c;
        out.work_expansion_j = w_e;
        out.net_work_j = w_c + w_e;
        out.unburned_fuel_energy_j = out.fuel_energy_j;
        out.unused_air_kg = fresh * (1.0 - leak);
        out.peak_pressure_pa = p_tdc;
        out.peak_pressure_angle_deg = 0.0;
        out.exhaust_temp_k = t_exh;
        out.exhaust_mass_kg = m_out;
        out.blowdown_pressure_pa = p_evo;
        out.residual_next_kg = m_res;
        out.residual_temp_next_k = t_res;
        out
    }

    /// Gas-exchange energy balance from EVO to the end of the exhaust stroke.
    ///
    /// Blow-down: the gas left in the cylinder expands isentropically to exhaust pressure,
    /// `T_bd = T_evo·(p_ex/p_evo)^((γ−1)/γ)`; this state is the residual for the next cycle,
    /// `m_res = p_ex·V_c/(R·T_bd)`. First law for the open system, the enthalpy that leaves
    /// is `H_out = c_v·(m·T_evo − m_res·T_bd) + p_ex·(V_evo − V_c) + Q_afterburn`, giving
    /// the mass-averaged port temperature `T_exh = H_out/(m_out·c_p)`.
    #[allow(clippy::too_many_arguments)]
    fn exhaust_state(
        &self,
        m: f32,
        p_evo: f32,
        v_evo: f32,
        t_evo: f32,
        gamma: f32,
        p_exhaust: f32,
        afterburn_j: f32,
    ) -> (f32, f32, f32, f32) {
        let p_ex = p_exhaust.max(50_000.0);
        let t_evo = t_evo.max(200.0);
        let t_bd = if p_evo > p_ex {
            t_evo * (p_ex / p_evo).powf((gamma - 1.0) / gamma)
        } else {
            t_evo
        };
        let v_c = self.geom.clearance_volume;
        let m_res = (p_ex * v_c / (R_CHARGE * t_bd)).min(0.5 * m);
        let m_out = (m - m_res).max(1.0e-9);
        let cv = R_CHARGE / (gamma - 1.0);
        let cp = gamma * cv;
        let h_out = cv * (m * t_evo - m_res * t_bd) + p_ex * (v_evo - v_c) + afterburn_j;
        let t_exh = clampf(h_out / (m_out * cp), 250.0, 2200.0);
        (t_exh, m_out, m_res, t_bd)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine_sim::spec::EngineSpec;

    fn wot_inputs(model: &CombustionModel, rpm: f32, spark: f32, lambda: f32) -> CycleInputs {
        // 0.95 VE at 1.15 kg/m³ manifold density.
        let air = 0.95 * 1.15 * 4.995e-4;
        let _ = model;
        CycleInputs {
            rpm,
            fresh_air_kg: air,
            fuel_kg: air / (14.7 * lambda),
            residual_kg: 0.04 * air,
            t_ivc_k: 345.0,
            spark_btdc_deg: spark,
            spark_enabled: true,
            spark_voltage_kv: 40.0,
            breakdown_noise: 0.0,
            octane_ron: 95.0,
            wall_temp_k: 400.0,
            exhaust_pressure_pa: 108_000.0,
            burn_ccv: 1.0,
            knock_ccv: 1.0,
            compression_leak: 0.0,
        }
    }

    #[test]
    fn mbt_exists_and_is_plausible() {
        let spec = EngineSpec::naturally_aspirated_2l();
        let m = CombustionModel::new(&spec);
        let mut best = (0.0, f32::MIN);
        for s in 0..60 {
            let mut inp = wot_inputs(&m, 3000.0, s as f32, 0.9);
            inp.octane_ron = 120.0; // remove the knock limit to locate MBT
            let r = m.simulate(&inp);
            if r.net_work_j > best.1 {
                best = (s as f32, r.net_work_j);
            }
        }
        assert!(best.0 > 15.0 && best.0 < 40.0, "MBT {}", best.0);
        // 2.0 L / 4 cyl at WOT: gross indicated work per cylinder ≈ 550–750 J.
        assert!(best.1 > 500.0 && best.1 < 800.0, "work {}", best.1);
    }

    #[test]
    fn advance_causes_knock_at_low_speed() {
        let spec = EngineSpec::naturally_aspirated_2l();
        let m = CombustionModel::new(&spec);
        let safe = m.simulate(&wot_inputs(&m, 2000.0, 5.0, 0.9));
        let hot = m.simulate(&wot_inputs(&m, 2000.0, 45.0, 0.9));
        assert_eq!(safe.knock_intensity, 0.0);
        assert!(hot.knock_intensity > 0.5, "KI {}", hot.knock_intensity);
    }

    #[test]
    fn lean_limit_misfires() {
        let spec = EngineSpec::naturally_aspirated_2l();
        let m = CombustionModel::new(&spec);
        let r = m.simulate(&wot_inputs(&m, 3000.0, 25.0, 1.8));
        assert!(r.misfire);
        assert!(r.net_work_j < 0.0);
    }

    #[test]
    fn retard_raises_exhaust_temperature() {
        let spec = EngineSpec::naturally_aspirated_2l();
        let m = CombustionModel::new(&spec);
        let mbt = m.simulate(&wot_inputs(&m, 4000.0, 28.0, 0.9));
        let late = m.simulate(&wot_inputs(&m, 4000.0, 0.0, 0.9));
        assert!(late.exhaust_temp_k > mbt.exhaust_temp_k + 80.0);
        assert!(late.net_work_j < mbt.net_work_j);
    }

    #[test]
    fn weak_spark_misfires_under_load_only() {
        let spec = EngineSpec::naturally_aspirated_2l();
        let m = CombustionModel::new(&spec);
        let mut inp = wot_inputs(&m, 3000.0, 25.0, 0.9);
        inp.spark_voltage_kv = 10.0;
        assert!(m.simulate(&inp).misfire);
        inp.fresh_air_kg *= 0.3;
        inp.fuel_kg *= 0.3;
        inp.residual_kg *= 0.3;
        assert!(!m.simulate(&inp).misfire);
    }
}

//! Mechanical damage accumulation and failure detection.
//!
//! Every mechanism is driven by a physical load (force, temperature, oil-film adequacy,
//! detonation pressure) and accumulates into a 0..1 health value. Health feeds back into
//! the physics through `DamageModifiers`, so a damaged engine *behaves* damaged:
//! compression loss, extra friction, falling oil pressure, coolant loss, exhaust
//! restriction. A student diagnosing it sees exactly the symptoms a technician would.

use super::combustion::CycleResult;
use super::events::{EventKind, EventLog};
use super::math::{clampf, Pcg32};
use super::spec::{CylinderGeometry, EngineSpec, MechanicalLimits, MAX_CYLINDERS};
use super::status::FailureCause;

/// Knock below this intensity \[bar\] is "trace knock" and causes no measurable erosion.
const KNOCK_DAMAGE_THRESHOLD: f32 = 0.3;
/// Piston erosion per cycle at KI = 1 bar; scales with KI^2.5 (pitting energy grows faster
/// than pressure amplitude). Calibrated to: light knock ≈ 20 min, heavy (3 bar) ≈ 1.5 min,
/// severe (6 bar) ≈ 15 s to a holed piston at 3000 rpm.
const KNOCK_EROSION_PER_CYCLE: f32 = 3.0e-5;
/// Ring-land wear from detonation per cycle at KI = 1 bar.
const RING_KNOCK_WEAR: f32 = 1.0e-5;
/// Head-gasket erosion per cycle per unit relative overpressure.
const GASKET_EROSION: f32 = 2.0e-3;
/// Bearing wear rate at zero oil film, per second per 1000 rpm (Archard wear with
/// boundary lubrication). Zero oil pressure at 3000 rpm destroys bearings in ≈ 20 s.
const BEARING_WEAR_RATE: f32 = 0.015;
/// Health below which a component is considered failed.
const FAILED_HEALTH: f32 = 0.1;
/// Per-cycle valve-to-piston contact probability at full float beyond the clearance margin.
const VALVE_CONTACT_PROBABILITY: f32 = 0.02;
/// Coolant lost through a breached head gasket (fraction of fill per second).
const GASKET_COOLANT_LEAK: f32 = 0.004;

/// Health of every monitored component (1 = new, 0 = destroyed).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HealthReport {
    /// Piston crown / ring-land integrity per cylinder.
    pub piston: [f32; MAX_CYLINDERS],
    /// Piston ring sealing per cylinder.
    pub rings: [f32; MAX_CYLINDERS],
    /// Valve integrity per cylinder (0 = bent).
    pub valves: [f32; MAX_CYLINDERS],
    /// Connecting-rod fatigue life consumed per cylinder (Miner sum, 1 = failure).
    pub rod_life_used: [f32; MAX_CYLINDERS],
    /// Head gasket sealing.
    pub head_gasket: f32,
    /// Cylinder head flatness.
    pub cylinder_head: f32,
    /// Crankshaft bearings.
    pub bearings: f32,
    /// Turbocharger rotating assembly.
    pub turbo: f32,
    /// Catalyst substrate.
    pub catalyst: f32,
}

impl Default for HealthReport {
    fn default() -> Self {
        Self {
            piston: [1.0; MAX_CYLINDERS],
            rings: [1.0; MAX_CYLINDERS],
            valves: [1.0; MAX_CYLINDERS],
            rod_life_used: [0.0; MAX_CYLINDERS],
            head_gasket: 1.0,
            cylinder_head: 1.0,
            bearings: 1.0,
            turbo: 1.0,
            catalyst: 1.0,
        }
    }
}

/// Damage state.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct DamageState {
    pub health: HealthReport,
    pub gasket_breach: Option<u8>,
    pub piston_failed: [bool; MAX_CYLINDERS],
    pub head_warped: bool,
    pub turbo_failed: bool,
    pub catalyst_failed: bool,
    pub fatal: Option<FailureCause>,
}

/// Physical consequences of damage, consumed by the plant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct DamageModifiers {
    pub compression_leak: [f32; MAX_CYLINDERS],
    pub friction_mult: f32,
    pub oil_pressure_mult: f32,
    pub coolant_leak_per_s: f32,
    pub exhaust_restriction_mult: f32,
    pub catalyst_health: f32,
    pub turbo_failed: bool,
    pub seized: bool,
}

/// Slow-varying conditions evaluated every step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct DamageContext {
    pub rpm: f32,
    pub oil_pressure_pa: f32,
    pub t_metal: f32,
    pub t_piston: [f32; MAX_CYLINDERS],
    pub turbo_omega: f32,
    pub turbine_inlet_k: f32,
    pub t_catalyst: f32,
    /// Normalised engine load 0..~2 (IMEP / 10 bar) for bearing loading.
    pub load: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct DamageModel {
    limits: MechanicalLimits,
    geom: CylinderGeometry,
    reciprocating_mass: f32,
    cylinders: usize,
    turbo_limits: Option<(f32, f32)>,
}

impl DamageModel {
    pub(crate) fn new(spec: &EngineSpec) -> Self {
        Self {
            limits: spec.limits,
            geom: CylinderGeometry::from_spec(&spec.geometry),
            reciprocating_mass: spec.geometry.reciprocating_mass_kg,
            cylinders: spec.geometry.cylinders,
            turbo_limits: spec.turbo.map(|t| {
                (
                    t.max_speed_rpm * core::f32::consts::TAU / 60.0,
                    t.turbine_inlet_limit_k,
                )
            }),
        }
    }

    /// Valve float severity 0..1 at the given speed.
    pub(crate) fn valve_float(&self, rpm: f32) -> f32 {
        clampf((rpm - self.limits.valve_float_rpm) / 500.0, 0.0, 1.0)
    }

    fn fail(st: &mut DamageState, cause: FailureCause, log: &mut EventLog, now: f64) {
        if cause.is_fatal() {
            if st.fatal.is_none() {
                st.fatal = Some(cause);
                log.push(now, EventKind::Damage(cause));
            }
        } else {
            log.push(now, EventKind::Damage(cause));
        }
    }

    /// Per-combustion-event loads: rod forces, detonation, peak pressure, valve contact.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn on_cylinder_event(
        &self,
        st: &mut DamageState,
        c: usize,
        r: &CycleResult,
        omega: f32,
        rng: &mut Pcg32,
        log: &mut EventLog,
        now: f64,
    ) {
        if st.fatal.is_some() || c >= self.cylinders {
            return;
        }
        let lim = &self.limits;
        let cyl = c as u8;

        // Rod tension at gas-exchange TDC: F = m·r·ω²·(1 + r/l). Fatigue by Basquin's law
        // N_f = N_e·(F_e/F)^k accumulated with the Palmgren–Miner rule ΣN/N_f.
        let f_inertia = self.geom.inertia_force(self.reciprocating_mass, omega);
        if f_inertia > lim.rod_ultimate_force_n {
            Self::fail(st, FailureCause::ThrownRod { cylinder: cyl }, log, now);
            return;
        }
        if f_inertia > lim.rod_endurance_force_n {
            st.health.rod_life_used[c] += (f_inertia / lim.rod_endurance_force_n)
                .powf(lim.rod_fatigue_exponent)
                / lim.rod_endurance_cycles;
            if st.health.rod_life_used[c] >= 1.0 {
                Self::fail(st, FailureCause::ThrownRod { cylinder: cyl }, log, now);
                return;
            }
        }

        // Rod compression at firing TDC: gas force (including the knock pressure spike)
        // minus the inertia force that partially relieves it.
        let p_eff = r.peak_pressure_pa + r.knock_intensity * 1.0e5;
        let f_gas = p_eff * self.geom.piston_area - f_inertia;
        if f_gas > lim.rod_buckling_force_n {
            Self::fail(st, FailureCause::BentRod { cylinder: cyl }, log, now);
            return;
        }

        // Detonation erosion of crown and ring lands.
        if r.knock_intensity > KNOCK_DAMAGE_THRESHOLD && !st.piston_failed[c] {
            st.health.piston[c] -= KNOCK_EROSION_PER_CYCLE * r.knock_intensity.powf(2.5);
            st.health.rings[c] -= RING_KNOCK_WEAR * r.knock_intensity * r.knock_intensity;
            if st.health.piston[c] < FAILED_HEALTH {
                st.health.piston[c] = 0.0;
                st.piston_failed[c] = true;
                Self::fail(st, FailureCause::HoledPiston { cylinder: cyl }, log, now);
            }
        }
        st.health.rings[c] = st.health.rings[c].max(0.0);

        // Head gasket: peak pressure beyond what the clamp load can seal lifts the head
        // locally and erodes the fire ring.
        let over = p_eff / lim.head_gasket_pressure_pa - 1.0;
        if over > 0.0 && st.gasket_breach.is_none() {
            st.health.head_gasket -= GASKET_EROSION * over;
            if st.health.head_gasket < 0.3 {
                st.gasket_breach = Some(cyl);
                Self::fail(st, FailureCause::HeadGasket { cylinder: cyl }, log, now);
            }
        }

        // Valve float on an interference engine: a valve bouncing off its seat can meet the
        // rising piston. Contact probability grows with float severity squared and is
        // twenty times higher beyond the design clearance margin.
        let rpm = omega * super::math::RAD_S_TO_RPM;
        let severity = self.valve_float(rpm);
        if lim.interference && severity > 0.0 && st.health.valves[c] > 0.0 {
            let margin_factor = if rpm > lim.valve_float_rpm + lim.valve_contact_margin_rpm {
                1.0
            } else {
                0.05
            };
            if rng.uniform() < VALVE_CONTACT_PROBABILITY * severity * severity * margin_factor {
                st.health.valves[c] = 0.0;
                Self::fail(st, FailureCause::BentValve { cylinder: cyl }, log, now);
            }
        }
    }

    /// Continuous loads: temperatures, oil film, turbo speed.
    pub(crate) fn update(
        &self,
        st: &mut DamageState,
        ctx: &DamageContext,
        dt: f32,
        log: &mut EventLog,
        now: f64,
    ) {
        if st.fatal.is_some() {
            return;
        }
        let lim = &self.limits;

        // Piston crown overtemperature: strength loss above the alloy limit (creep and
        // ring-groove collapse), rapid erosion near the solidus.
        for c in 0..self.cylinders {
            if st.piston_failed[c] {
                continue;
            }
            let t = ctx.t_piston[c];
            let h = &mut st.health.piston[c];
            if t > lim.piston_crown_melt_k {
                *h -= 0.4 * dt;
                if *h < FAILED_HEALTH {
                    *h = 0.0;
                    st.piston_failed[c] = true;
                    Self::fail(
                        st,
                        FailureCause::MeltedPiston { cylinder: c as u8 },
                        log,
                        now,
                    );
                }
            } else if t > lim.piston_crown_limit_k {
                let x = (t - lim.piston_crown_limit_k) / 50.0;
                *h -= 0.004 * x * x * dt;
            }
        }

        // Bearings: full hydrodynamic film needs ≈ 10 psi per 1000 rpm. Below that the
        // journal runs in boundary lubrication and wears (Archard), faster under load.
        if ctx.rpm > 200.0 {
            let required = (lim.bearing_oil_per_krpm * ctx.rpm / 1000.0).max(20_000.0);
            let film = clampf(ctx.oil_pressure_pa / required, 0.0, 1.0);
            let starvation = 1.0 - film;
            st.health.bearings -= BEARING_WEAR_RATE
                * starvation
                * starvation
                * (ctx.rpm / 1000.0)
                * (1.0 + ctx.load)
                * dt;
            if st.health.bearings < 0.05 {
                st.health.bearings = 0.0;
                Self::fail(st, FailureCause::SpunBearing, log, now);
                return;
            }
        }

        // Cylinder head distortion from overheated metal; a warped head then attacks
        // the gasket.
        if ctx.t_metal > lim.head_warp_k && !st.head_warped {
            st.health.cylinder_head -= 0.01 * (ctx.t_metal - lim.head_warp_k) / 20.0 * dt;
            if st.health.cylinder_head < 0.5 {
                st.head_warped = true;
                Self::fail(st, FailureCause::WarpedHead, log, now);
            }
        }
        if st.head_warped && st.gasket_breach.is_none() {
            st.health.head_gasket -= 0.02 * dt;
            if st.health.head_gasket < 0.3 {
                st.gasket_breach = Some(0);
                Self::fail(st, FailureCause::HeadGasket { cylinder: 0 }, log, now);
            }
        }

        // Turbocharger: overspeed (blade/bore burst risk grows with centrifugal stress
        // ∝ ω²) and turbine inlet over-temperature (creep of the nickel wheel).
        if let Some((omega_max, t_limit)) = self.turbo_limits {
            if !st.turbo_failed {
                let over = ctx.turbo_omega / omega_max - 1.0;
                if over > 0.0 {
                    let x = over / 0.05;
                    st.health.turbo -= 0.05 * x * x * dt;
                }
                if ctx.turbine_inlet_k > t_limit {
                    st.health.turbo -= 0.002 * (ctx.turbine_inlet_k - t_limit) / 50.0 * dt;
                }
                if st.health.turbo < 0.05 {
                    st.health.turbo = 0.0;
                    st.turbo_failed = true;
                    Self::fail(st, FailureCause::TurboFailure, log, now);
                }
            }
        }

        // Catalyst substrate melting.
        if !st.catalyst_failed && ctx.t_catalyst > lim.catalyst_melt_k {
            st.health.catalyst -= 0.02 * ((ctx.t_catalyst - lim.catalyst_melt_k) / 50.0 + 1.0) * dt;
            if st.health.catalyst < 0.2 {
                st.catalyst_failed = true;
                Self::fail(st, FailureCause::CatalystMeltdown, log, now);
            }
        }
        st.health.catalyst = st.health.catalyst.max(0.0);
        st.health.turbo = st.health.turbo.max(0.0);
        st.health.head_gasket = st.health.head_gasket.max(0.0);
        st.health.cylinder_head = st.health.cylinder_head.max(0.0);
    }

    /// Physical consequences of the current damage state.
    pub(crate) fn modifiers(&self, st: &DamageState) -> DamageModifiers {
        let h = &st.health;
        let mut leak = [0.0; MAX_CYLINDERS];
        let mut piston_mean = 0.0;
        for (c, slot) in leak.iter_mut().enumerate().take(self.cylinders) {
            // Worn rings leak up to half the charge (≈ 50 % leak-down); a holed piston,
            // breached gasket or bent valve lose most or all of it.
            let mut l = 0.5 * (1.0 - h.rings[c]);
            if st.piston_failed[c] {
                l += 0.9;
            }
            if st.gasket_breach == Some(c as u8) {
                l += 0.35;
            }
            if h.valves[c] <= 0.0 {
                l = 1.0;
            }
            *slot = clampf(l, 0.0, 1.0);
            piston_mean += h.piston[c];
        }
        piston_mean /= self.cylinders as f32;
        DamageModifiers {
            compression_leak: leak,
            // Worn bearings run with larger clearance and partial metal contact;
            // scuffed pistons drag on the bores.
            friction_mult: 1.0 + 1.5 * (1.0 - h.bearings) + 0.5 * (1.0 - piston_mean),
            // Oil escapes through enlarged bearing clearances.
            oil_pressure_mult: 0.35 + 0.65 * h.bearings,
            coolant_leak_per_s: if st.gasket_breach.is_some() {
                GASKET_COOLANT_LEAK
            } else {
                0.0
            },
            // Molten substrate blocks the channels.
            exhaust_restriction_mult: 1.0 + 8.0 * (1.0 - h.catalyst) * (1.0 - h.catalyst),
            catalyst_health: h.catalyst,
            turbo_failed: st.turbo_failed,
            seized: st.fatal.is_some(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heavy_knock_holes_a_piston() {
        let spec = EngineSpec::naturally_aspirated_2l();
        let model = DamageModel::new(&spec);
        let mut st = DamageState::default();
        let mut rng = Pcg32::new(1, 1);
        let mut log = EventLog::default();
        let r = CycleResult {
            knock_intensity: 6.0,
            peak_pressure_pa: 70.0e5,
            ..CycleResult::default()
        };
        let mut cycles = 0;
        while !st.piston_failed[1] && cycles < 100_000 {
            model.on_cylinder_event(&mut st, 1, &r, 314.0, &mut rng, &mut log, 0.0);
            cycles += 1;
        }
        // 3000 rpm → 25 cycles/s per cylinder: 6 bar knock fails in well under a minute.
        assert!(st.piston_failed[1]);
        assert!(cycles < 25 * 60, "{cycles}");
        assert!(model.modifiers(&st).compression_leak[1] > 0.8);
    }

    #[test]
    fn extreme_over_rev_throws_a_rod() {
        let spec = EngineSpec::naturally_aspirated_2l();
        let model = DamageModel::new(&spec);
        let mut st = DamageState::default();
        let mut rng = Pcg32::new(1, 1);
        let mut log = EventLog::default();
        let omega = 10_500.0 * core::f32::consts::TAU / 60.0;
        model.on_cylinder_event(
            &mut st,
            0,
            &CycleResult::default(),
            omega,
            &mut rng,
            &mut log,
            0.0,
        );
        assert!(matches!(st.fatal, Some(FailureCause::ThrownRod { .. })));
    }

    #[test]
    fn oil_starvation_spins_bearing() {
        let spec = EngineSpec::naturally_aspirated_2l();
        let model = DamageModel::new(&spec);
        let mut st = DamageState::default();
        let mut log = EventLog::default();
        let ctx = DamageContext {
            rpm: 3000.0,
            oil_pressure_pa: 0.0,
            t_metal: 360.0,
            t_piston: [450.0; MAX_CYLINDERS],
            turbo_omega: 0.0,
            turbine_inlet_k: 900.0,
            t_catalyst: 900.0,
            load: 0.5,
        };
        let mut t = 0.0;
        while st.fatal.is_none() && t < 120.0 {
            model.update(&mut st, &ctx, 0.01, &mut log, 0.0);
            t += 0.01;
        }
        assert_eq!(st.fatal, Some(FailureCause::SpunBearing));
        assert!(t < 60.0, "{t}");
    }
}

//! The physical engine ("plant"): ties together air path, per-cylinder combustion,
//! crank dynamics, thermal network, lubrication, fuel and electrical systems.

use core::f32::consts::PI;

use super::airpath::{AirPath, AirPathInputs, AirPathState};
use super::breathing::{self, BreathingInputs};
use super::combustion::{CombustionModel, CycleInputs, CycleResult};
use super::controls::{Ambient, Controls, LoadModel};
use super::damage::DamageModifiers;
use super::dynamics::{self, CrankInputs, CrankState};
use super::ecu::ActuatorCommands;
use super::faults::FaultState;
use super::math::{
    approach, clampf, finite_or, flush_tiny, lag_alpha, smoothstep, wrap_cycle, Pcg32, CYCLE_RAD,
    RAD_S_TO_RPM, RPM_TO_RAD_S,
};
use super::spec::{CylinderGeometry, EngineSpec, MAX_CYLINDERS};
use super::thermal::{ThermalInputs, ThermalModel, ThermalState, THERMAL_STEP_S};
use super::thermo::{
    vogel_viscosity, water_saturation_pressure, CP_AIR, CP_EXHAUST, GAMMA_EXHAUST, R_AIR,
};

/// Longest firing interval still timed for event prediction (≈ 37 rpm on a four) \[s\].
const MAX_FIRING_INTERVAL_S: f32 = 0.8;
/// Shortest firing interval accepted as a real one (≈ 75 000 rpm on a four) \[s\]: a
/// shorter one is the same event crossed twice through rounding of the crank angle.
const MIN_FIRING_INTERVAL_S: f32 = 1.0e-4;
/// Typical starter cranking speed (≈ 150 rpm), the floor of event-delay predictions
/// \[rad/s\].
const CRANKING_SPEED_RAD_S: f32 = 150.0 * RPM_TO_RAD_S;
/// Time constant of the rock-back of a stopped crank pushed backwards by its trapped
/// charge (a fraction of a revolution against the rotating inertia) \[s\].
const ROCK_BACK_TAU_S: f32 = 0.1;
/// Normalisation of the per-stroke torque shape `g(x) = sin x · (1 − x/π)³` on [0, π]:
/// ∫₀^π g dx = (π² − 6)/π² ≈ 0.392. The shape peaks ≈ 40° from TDC, like the measured
/// gas-torque of a firing cylinder, and its integral equals the stroke work, so the mean
/// crank torque is exactly the cycle work divided by 4π.
const TORQUE_SHAPE_NORM: f32 = (PI * PI - 6.0) / (PI * PI);
/// Ratio of residual-gas to fresh-charge specific heat used in the IVC mixing temperature
/// (burned gas c_p ≈ 1160 vs air 1005 J/(kg·K)).
const RESIDUAL_CP_RATIO: f32 = 1.15;
/// Time constant of the mean-value torque / flow filters reported to telemetry \[s\].
const MEAN_FILTER_TAU: f32 = 0.1;
/// Exhaust pulse smoothing into a mean manifold flow \[s\].
const EXHAUST_FLOW_TAU: f32 = 0.03;
/// Per-event mixing weight of the exhaust collector seen by the O2 sensor: the sensor sees
/// a blend of the last two or three exhaust pulses (cylinder imbalance shows as ripple).
const LAMBDA_MIX_ALPHA: f32 = 0.35;
/// Alternator cut-in speed (≈ 500 rpm crank) \[rad/s\].
const ALTERNATOR_CUT_IN: f32 = 52.0;
/// Fraction of the exhaust gas's excess enthalpy (over port-wall temperature) transferred
/// to the cylinder head in the exhaust port at 3000 rpm. Port heat transfer is a major
/// coolant heat source (≈ 1/3 of coolant heat rejection) and cools the gas by 80–150 K
/// before it reaches the manifold (Caton & Heywood, 1981).
const EXHAUST_PORT_HEAT_FRACTION: f32 = 0.15;
/// Gas-side conductance of the exhaust manifold at 0.05 kg/s \[W/K\]; scales with ṁ^0.8
/// (turbulent pipe flow, Dittus–Boelter).
const MANIFOLD_UA_REF: f32 = 30.0;
/// Fraction of cylinder fuel that fails to vaporise and burn when the engine is fully
/// cold (port wall at ambient), scaled with coldness^1.3. Liquid fuel films on cold
/// cylinder walls and crevices leave as hydrocarbons or wash past the rings into the oil
/// — the physical reason ECUs need warm-up enrichment (Heywood §11.4.3).
const COLD_FUEL_LOSS: f32 = 0.25;
/// Share of the unvaporised fuel that ends up in the crankcase oil rather than the exhaust.
const COLD_FUEL_TO_OIL: f32 = 0.3;
/// Stall (dead-head) pressure of the electric fuel pump relative to regulator pressure.
const PUMP_STALL_RATIO: f32 = 1.5;

/// Rail pressure (fraction of regulator setting) where the pump curve meets injector
/// demand. Positive-displacement electric pumps deliver `Q(x) ∝ 1 − (x/x_stall)²` (internal
/// leakage grows with pressure), normalised so that `capacity` is the delivery *at
/// regulator pressure* (x = 1), as pump ratings are quoted; injectors draw
/// `D(x) = D_reg·√x`. When
/// the pump can supply more than the demand at regulator pressure, the regulator holds
/// p_reg; otherwise pressure falls to the intersection, solved here by bisection (16
/// iterations, < 0.01 % error, allocation-free).
fn fuel_pump_operating_point(capacity: f32, demand_at_reg: f32) -> f32 {
    let norm = 1.0 / (1.0 - (1.0 / PUMP_STALL_RATIO).powi(2));
    let pump = |x: f32| capacity * norm * (1.0 - (x / PUMP_STALL_RATIO).powi(2));
    if pump(1.0) >= demand_at_reg {
        return 1.0;
    }
    let (mut lo, mut hi) = (0.0_f32, 1.0_f32);
    for _ in 0..16 {
        let mid = 0.5 * (lo + hi);
        if pump(mid) > demand_at_reg * mid.sqrt() {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// Oil inventory mass used for the fuel-dilution balance \[kg\].
const OIL_MASS_KG: f32 = 3.8;
/// Speed below which accessories are idealised as a constant-power load \[rad/s\]; avoids
/// the P/ω singularity at standstill.
const ACCESSORY_OMEGA_FLOOR: f32 = 60.0;

/// Per-cylinder state carried between cycles.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct CylinderState {
    pub film_kg: f32,
    pub residual_kg: f32,
    pub residual_temp_k: f32,
    pub work_compression_j: f32,
    pub work_expansion_j: f32,
    pub last: CycleResult,
    pub last_inputs: Option<CycleInputs>,
    pub events: u32,
    pub injected_kg: f32,
    pub trapped_air_kg: f32,
}

/// Torque decomposition \[N·m\].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct TorqueBreakdown {
    pub gas_instant: f32,
    pub indicated_mean: f32,
    pub friction: f32,
    pub pumping: f32,
    pub accessory: f32,
    pub brake_mean: f32,
}

/// Crank speed at firing TDCs for the misfire monitor.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct CrankSegment {
    pub seq: u32,
    pub cylinder: u8,
    pub omega: f32,
}

#[derive(Debug, Clone)]
pub(crate) struct Plant {
    pub spec: EngineSpec,
    pub cylinders: usize,
    geom: CylinderGeometry,
    combustion: CombustionModel,
    airpath: AirPath,
    thermal_model: ThermalModel,
    pub air: AirPathState,
    pub crank: CrankState,
    pub thermal: ThermalState,
    pub cyl: [CylinderState; MAX_CYLINDERS],
    /// Cylinder index at each firing position.
    pub firing_seq: [usize; MAX_CYLINDERS],
    /// Crank angle of each cylinder's firing TDC \[rad\].
    tdc_angle: [f32; MAX_CYLINDERS],
    pub fired: [bool; MAX_CYLINDERS],
    pub fuel_rail_pa: f32,
    pub battery_v: f32,
    pub oil_pressure_pa: f32,
    pub oil_viscosity: f32,
    pub oil_dilution: f32,
    viscosity_ref: f32,
    pub ve: f32,
    pub t_fresh: f32,
    pub valve_float: f32,
    pub exhaust_lambda: f32,
    /// Exhaust λ mixed over cycles the ECU fuelled only: the mixture the engine burns,
    /// without the air of commanded fuel cuts (a failed injector still counts).
    pub exhaust_lambda_fuelled: f32,
    pub exhaust_lambda_apparent: f32,
    pub exhaust_port_temp: f32,
    pub exhaust_mass_flow: f32,
    exhaust_enthalpy_flow: f32,
    pub fuel_mass_flow: f32,
    pub peak_pressure_mean_bar: f32,
    pub torque: TorqueBreakdown,
    pub segment: CrankSegment,
    /// Fraction of the last step at which each cylinder that fired this step crossed its
    /// start of compression (for sub-step event time stamps).
    pub fire_fraction: [f32; MAX_CYLINDERS],
    /// Durations of the last two firing intervals (between consecutive cylinder events),
    /// newest first \[s\] (see [`Plant::time_to_turn`]).
    firing_intervals_s: [f32; 2],
    /// Number of valid entries in `firing_intervals_s`.
    intervals_valid: u8,
    /// Time since the last cylinder event \[s\].
    since_event_s: f32,
    pub catalyst_efficiency: f32,
    hc_store_j: f32,
    o2_store_kg: f32,
    exhaust_mass_accum: f32,
    exhaust_enthalpy_accum: f32,
    fuel_accum: f32,
    thermal_acc: ThermalAccumulator,
}

/// Heat and flow integrals collected at the physics rate between two thermal-network
/// steps (see [`THERMAL_STEP_S`]).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct ThermalAccumulator {
    /// Elapsed time \[s\].
    time_s: f32,
    /// Combustion and exhaust-port heat into the metal \[J\].
    wall_heat_j: f32,
    /// Friction work \[J\].
    friction_j: f32,
    /// Exhaust-gas heat into the manifold wall \[J\].
    manifold_j: f32,
    /// Catalytic exotherm \[J\].
    exotherm_j: f32,
    /// Crank angle travelled \[rad\].
    angle_rad: f32,
    /// Exhaust mass through the catalyst \[kg\].
    exhaust_kg: f32,
    /// Exhaust mass × catalyst inlet temperature \[kg·K\].
    exhaust_kg_k: f32,
}

impl Plant {
    pub(crate) fn new(spec: &EngineSpec, ambient: &Ambient) -> Self {
        let n = spec.geometry.cylinders;
        let geom = CylinderGeometry::from_spec(&spec.geometry);
        let mut firing_seq = [0usize; MAX_CYLINDERS];
        let mut tdc_angle = [0.0f32; MAX_CYLINDERS];
        for (pos, (slot, &cyl)) in firing_seq
            .iter_mut()
            .zip(spec.geometry.firing_order.iter())
            .enumerate()
            .take(n)
        {
            let c = usize::from(cyl);
            *slot = c;
            // Evenly spaced firing: one TDC every 720°/n.
            tdc_angle[c] = pos as f32 * CYCLE_RAD / n as f32;
        }
        let t = ambient.temperature_k;
        let residual = ambient.pressure_pa * geom.clearance_volume / (R_AIR * t);
        let cyl = CylinderState {
            residual_kg: residual,
            residual_temp_k: t,
            ..CylinderState::default()
        };
        let lub = &spec.lubrication;
        Self {
            spec: *spec,
            cylinders: n,
            geom,
            combustion: CombustionModel::new(spec),
            airpath: AirPath::new(spec),
            thermal_model: ThermalModel::new(&spec.thermal),
            air: AirPathState::at_ambient(ambient.pressure_pa, t),
            crank: CrankState::default(),
            thermal: ThermalState::uniform(t),
            cyl: [cyl; MAX_CYLINDERS],
            firing_seq,
            tdc_angle,
            fired: [false; MAX_CYLINDERS],
            fuel_rail_pa: 0.0,
            battery_v: 12.6,
            oil_pressure_pa: 0.0,
            oil_viscosity: vogel_viscosity(lub.vogel_a, lub.vogel_b, lub.vogel_c, t),
            oil_dilution: 0.0,
            viscosity_ref: vogel_viscosity(
                lub.vogel_a,
                lub.vogel_b,
                lub.vogel_c,
                lub.reference_temp_k,
            ),
            ve: 0.0,
            t_fresh: t,
            valve_float: 0.0,
            exhaust_lambda: 1.0,
            exhaust_lambda_fuelled: 1.0,
            exhaust_lambda_apparent: 3.0,
            exhaust_port_temp: t,
            exhaust_mass_flow: 0.0,
            exhaust_enthalpy_flow: 0.0,
            fuel_mass_flow: 0.0,
            peak_pressure_mean_bar: 1.0,
            torque: TorqueBreakdown::default(),
            segment: CrankSegment::default(),
            fire_fraction: [0.0; MAX_CYLINDERS],
            firing_intervals_s: [0.0; 2],
            intervals_valid: 0,
            // No event yet: the first one opens timing, it does not close an interval.
            since_event_s: 2.0 * MAX_FIRING_INTERVAL_S,
            catalyst_efficiency: 0.0,
            hc_store_j: 0.0,
            o2_store_kg: 0.0,
            exhaust_mass_accum: 0.0,
            exhaust_enthalpy_accum: 0.0,
            fuel_accum: 0.0,
            thermal_acc: ThermalAccumulator::default(),
        }
    }

    /// Engine speed \[rpm\].
    #[inline]
    pub(crate) fn rpm(&self) -> f32 {
        self.crank.omega * RAD_S_TO_RPM
    }

    /// MBT spark advance [deg BTDC] for the charge cylinder `c` trapped on its last cycle.
    pub(crate) fn mbt_for_cylinder(&self, c: usize) -> Option<f32> {
        let inputs = self.cyl.get(c)?.last_inputs?;
        if inputs.fuel_kg <= 0.0 {
            return None;
        }
        Some(self.combustion.find_mbt(&inputs))
    }

    /// Brings the crankshaft and any coupled load (dyno absorber, driveline) to rest.
    pub(crate) fn stop_crank(&mut self) {
        let theta = self.crank.theta;
        self.crank = CrankState {
            theta,
            vehicle_speed: self.crank.vehicle_speed,
            ..CrankState::default()
        };
        self.intervals_valid = 0;
        self.since_event_s = 2.0 * MAX_FIRING_INTERVAL_S;
    }

    /// Puts all thermal masses at fully-warm operating temperature.
    pub(crate) fn prewarm(&mut self, ambient_t: f32) {
        self.thermal.t_metal = 368.0;
        self.thermal.t_coolant = 363.0;
        self.thermal.t_oil = 365.0;
        self.thermal.thermostat_pos = 0.3;
        self.thermal.t_catalyst = 650.0;
        self.thermal.t_manifold = 600.0;
        for t in &mut self.thermal.t_piston {
            *t = 420.0;
        }
        self.air.t_man = ambient_t + 8.0;
    }

    /// Sum of the instantaneous gas torques of all cylinders at crank angle `theta`.
    fn gas_torque(&self, theta: f32) -> f32 {
        let mut t = 0.0;
        for c in 0..self.cylinders {
            let mut d = wrap_cycle(theta - self.tdc_angle[c]);
            if d >= 2.0 * PI {
                d -= CYCLE_RAD;
            }
            let cyl = &self.cyl[c];
            if (-PI..0.0).contains(&d) {
                let x = -d;
                let g = x.sin() * (1.0 - x / PI).powi(3);
                t += cyl.work_compression_j * g / TORQUE_SHAPE_NORM;
            } else if (0.0..PI).contains(&d) {
                let g = d.sin() * (1.0 - d / PI).powi(3);
                t += cyl.work_expansion_j * g / TORQUE_SHAPE_NORM;
            }
        }
        t
    }

    /// Books the duration of the firing interval that a cylinder event just closed.
    fn record_firing_interval(&mut self, duration_s: f32) {
        if duration_s > MAX_FIRING_INTERVAL_S {
            self.intervals_valid = 0;
        } else if duration_s < MIN_FIRING_INTERVAL_S
            || (self.intervals_valid > 0 && duration_s < 0.25 * self.firing_intervals_s[0])
        {
            // The same event crossed twice, not a firing interval: keep the history.
        } else {
            self.firing_intervals_s = [duration_s, self.firing_intervals_s[0]];
            self.intervals_valid = (self.intervals_valid + 1).min(2);
        }
    }

    /// Predicted time for the crankshaft to turn through `angle` \[rad\] from the latest
    /// cylinder event.
    ///
    /// Uses the durations of the last two firing intervals, as production ECUs time crank
    /// segments: the mean speed over a whole interval, ω̄ = (4π/n)/T, carries no
    /// firing-pulse ripple because the ripple is periodic in the interval. At steady speed
    /// the prediction is therefore exact for angles that are whole multiples of the firing
    /// interval (TDC on fours and eights); other angles (EVO, TDC with 1–3 or 5–7
    /// cylinders) keep the ripple's phase error, a few crank degrees at low speed. The two
    /// interval means, centred half an interval back, give the acceleration
    /// α = (ω̄₁ − ω̄₀)/((T₁ + T₀)/2) and the speed at the event ω = ω̄₁ + α·T₁/2; the angle
    /// then follows φ = ω·t + ½·α·t², solved in the cancellation-free form
    /// t = 2φ/(ω + √(ω² + 2αφ)). A deceleration that stops the crank short of `angle` (the
    /// event will not come) reports the predicted stop time −ω/α, capped at the delay at
    /// the current speed; fewer than two timed intervals (start, very slow cranking) fall
    /// back to the current speed, floored at cranking speed.
    pub(crate) fn time_to_turn(&self, angle: f32) -> f32 {
        // Below cranking speed the crank is either being spun up by the starter or about to
        // stop; either way it will not take longer than at cranking speed.
        let omega_now = self.crank.omega.max(CRANKING_SPEED_RAD_S);
        if self.intervals_valid < 2 {
            return angle / omega_now;
        }
        let interval_angle = CYCLE_RAD / self.cylinders as f32;
        let [t1, t0] = self.firing_intervals_s;
        let (w1, w0) = (interval_angle / t1, interval_angle / t0);
        let alpha = (w1 - w0) / (0.5 * (t1 + t0));
        let omega = w1 + alpha * 0.5 * t1;
        let disc = omega * omega + 2.0 * alpha * angle;
        if omega > 0.0 && disc > 0.0 {
            2.0 * angle / (omega + disc.sqrt())
        } else {
            // Only a deceleration (α < 0) gets here.
            let stop = omega.max(0.0) / -alpha.min(-1.0e-6);
            stop.min(angle / omega_now)
        }
    }

    /// Lets the charge trapped in a stopped engine leak away (an overnight soak): the
    /// stored compression and expansion works no longer act on the crank.
    pub(crate) fn release_trapped_charge(&mut self) {
        let n = self.cylinders;
        for cyl in &mut self.cyl[..n] {
            cyl.work_compression_j = 0.0;
            cyl.work_expansion_j = 0.0;
        }
        self.torque.gas_instant = 0.0;
    }

    /// Advances the plant by one step.
    pub(crate) fn step(
        &mut self,
        cmd: &ActuatorCommands,
        ctl: &Controls,
        faults: &FaultState,
        dmg: &DamageModifiers,
        rng: &mut Pcg32,
        dt: f32,
    ) {
        let n = self.cylinders;
        self.fired = [false; MAX_CYLINDERS];
        let amb = ctl.ambient;
        let omega0 = self.crank.omega;
        let rpm0 = omega0 * RAD_S_TO_RPM;

        // Water vapour displaces dry air: at a given total pressure and temperature the dry
        // air density falls by the vapour's partial-pressure share (Dalton), 1 − p_v/p.
        let p_vap = amb.relative_humidity * water_saturation_pressure(amb.temperature_k);
        let dry_fraction = clampf(1.0 - p_vap / amb.pressure_pa, 0.85, 1.0);
        self.valve_float = clampf((rpm0 - self.spec.limits.valve_float_rpm) / 500.0, 0.0, 1.0);

        // ---- Breathing ----------------------------------------------------------------
        let t_port = self.thermal.t_metal;
        self.t_fresh = breathing::fresh_charge_temperature(
            &self.spec,
            rpm0,
            self.air.p_man,
            self.air.t_man,
            t_port,
        );
        self.ve = breathing::volumetric_efficiency(
            &self.spec,
            &BreathingInputs {
                rpm: rpm0,
                p_man: self.air.p_man,
                p_exh: self.air.p_exh,
                t_man: self.air.t_man,
                t_fresh: self.t_fresh,
                cam_retard_deg: faults.cam_retard_deg,
                valve_float: self.valve_float,
            },
        );
        let v_d = self.spec.geometry.displacement_m3();
        // Speed-density pumping: ṁ = η_v·ρ_man·V_d·n/120 = k·p_man.
        let k_cyl = self.ve * v_d * rpm0.max(0.0) / 120.0 / (R_AIR * self.air.t_man);

        // Exhaust manifold as a heat exchanger (ε–NTU against its wall temperature): at
        // idle the slow gas loses several hundred kelvin, at full load only ≈ 100 K.
        let c_gas = self.exhaust_mass_flow.max(0.0) * CP_EXHAUST;
        let ua_man = MANIFOLD_UA_REF * (self.exhaust_mass_flow.max(0.0) / 0.05).powf(0.8);
        let eps_man = if c_gas > 1.0e-6 {
            1.0 - (-ua_man / c_gas).exp()
        } else {
            0.0
        };
        let manifold_out =
            self.exhaust_port_temp - eps_man * (self.exhaust_port_temp - self.thermal.t_manifold);
        let manifold_heat_w = c_gas * (self.exhaust_port_temp - manifold_out);

        self.airpath.step(
            &mut self.air,
            &AirPathInputs {
                throttle_cmd: cmd.throttle,
                idle_valve_cmd: cmd.idle_valve,
                wastegate_duty: cmd.wastegate_duty,
                ambient_p: amb.pressure_pa,
                ambient_t: amb.temperature_k,
                cylinder_flow_coeff: k_cyl,
                exhaust_mass_flow: self.exhaust_mass_flow,
                exhaust_gas_temp: manifold_out,
                vacuum_leak_area: faults.vacuum_leak_area_m2,
                boost_leak_area: faults.boost_leak_area_m2,
                exhaust_restriction: faults.exhaust_restriction * dmg.exhaust_restriction_mult,
                wastegate_stuck_closed: faults.wastegate_stuck_closed,
                turbo_failed: dmg.turbo_failed,
                engine_bay_temp: self.thermal.t_metal,
            },
            dt,
        );

        // ---- Crank torques --------------------------------------------------------------
        let gas = self.gas_torque(self.crank.theta + 0.5 * omega0 * dt);
        // Gas exchange work per cycle = −(p_exh − p_man)·V_d (ideal pumping loop).
        let pumping =
            -(self.air.p_exh - self.air.p_man) * v_d / (4.0 * PI) * smoothstep(0.0, 30.0, omega0);
        // Chen–Flynn friction mean effective pressure; the hydrodynamic terms scale with
        // oil viscosity (Petroff/Stribeck regime; exponent 0.3 from cold-start friction
        // measurements: ≈ 2× friction at 20 °C versus 90 °C).
        let fr = &self.spec.friction;
        let up = self.spec.geometry.mean_piston_speed(rpm0);
        let visc = clampf(self.oil_viscosity / self.viscosity_ref, 0.5, 8.0).powf(0.3);
        let fmep = (fr.fmep_constant_pa + fr.fmep_speed * up + fr.fmep_speed_sq * up * up) * visc
            + fr.fmep_peak_pressure_coeff * self.peak_pressure_mean_bar * 1.0e5;
        let friction = fmep * v_d / (4.0 * PI) * dmg.friction_mult;
        let charging = ctl.ignition_on && omega0 > ALTERNATOR_CUT_IN;
        let p_acc = fr.accessory_power_w
            + if charging {
                ctl.electrical_load_w / fr.alternator_efficiency
            } else {
                0.0
            };
        let accessory = if omega0 > 1.0 {
            p_acc / omega0.max(ACCESSORY_OMEGA_FLOOR)
        } else {
            0.0
        };
        let st = &self.spec.starter;
        let starter = if ctl.starter && ctl.ignition_on && !dmg.seized {
            st.stall_torque_nm * (1.0 - omega0 / (st.free_speed_rpm * RPM_TO_RAD_S)).max(0.0)
        } else {
            0.0
        };

        let theta_before = self.crank.theta;
        let crank_pushed_back = dynamics::step(
            &mut self.crank,
            &CrankInputs {
                gas_torque: gas,
                pumping_torque: pumping,
                friction_torque: friction,
                accessory_torque: accessory,
                starter_torque: starter,
                inertia: self.spec.geometry.rotating_inertia_kg_m2,
                air_density: amb.pressure_pa / (R_AIR * amb.temperature_k),
                seized: dmg.seized,
            },
            &ctl.load,
            dt,
        );
        let dtheta = self.crank.omega * dt;
        if self.crank.omega <= 0.0 {
            // A stopped crank has no combustion: the last cycle's expansion work must not
            // keep acting as a phantom forward torque on it. The trapped charge still acts
            // as a lossless air spring (W_exp = −W_comp, an odd torque about TDC with zero
            // net work), which is what holds a car parked in gear. When that spring pushes
            // the crank backwards harder than static friction holds it, a real crank rocks
            // back towards the balance point between the compressing and the expanding
            // cylinder; the forward-only crank model releases the charge instead, until
            // friction can hold what is left.
            let release = if crank_pushed_back {
                lag_alpha(dt, ROCK_BACK_TAU_S)
            } else {
                0.0
            };
            for cyl in &mut self.cyl[..n] {
                cyl.work_expansion_j = cyl.work_expansion_j.min(-cyl.work_compression_j);
                cyl.work_compression_j = flush_tiny(cyl.work_compression_j * (1.0 - release));
                cyl.work_expansion_j = flush_tiny(cyl.work_expansion_j * (1.0 - release));
            }
        }

        // ---- Cylinder events (start of each compression stroke) --------------------------
        if dtheta > 0.0 && !dmg.seized {
            for pos in 0..n {
                let c = self.firing_seq[pos];
                let event = wrap_cycle(self.tdc_angle[c] - PI);
                let to_event = wrap_cycle(event - theta_before);
                if to_event < dtheta {
                    let frac = to_event / dtheta;
                    self.fire_fraction[c] = frac;
                    self.record_firing_interval(self.since_event_s + frac * dt);
                    self.since_event_s = -frac * dt;
                    self.fire(c, cmd, ctl, faults, dmg, rng, dry_fraction);
                }
            }
        }
        self.since_event_s = (self.since_event_s + dt).min(MAX_FIRING_INTERVAL_S * 2.0);
        self.track_segments(theta_before, dtheta, omega0);

        // ---- Catalyst chemistry ---------------------------------------------------------
        let th = &self.spec.thermal;
        let conversion = smoothstep(
            th.catalyst_light_off_k - 50.0,
            th.catalyst_light_off_k + 100.0,
            self.thermal.t_catalyst,
        ) * dmg.catalyst_health;
        self.catalyst_efficiency = conversion;
        let lhv = self.spec.fuel.lower_heating_value;
        let afr = self.spec.fuel.stoich_afr;
        // HC/CO can only burn where oxygen is present: limited by the leaner of the two
        // stores (misfires deliver both fuel and air, which is what melts catalysts).
        let oxidisable = self.hc_store_j.min(self.o2_store_kg / afr * lhv);
        let react = conversion * oxidisable * lag_alpha(dt, 0.05);
        self.hc_store_j -= react;
        self.o2_store_kg -= react / lhv * afr;
        let flush = (-dt / 0.15).exp();
        self.hc_store_j *= flush;
        self.o2_store_kg *= flush;

        // ---- Thermal ----------------------------------------------------------------------
        let omega = self.crank.omega;
        let rpm = omega * RAD_S_TO_RPM;
        let acc = &mut self.thermal_acc;
        acc.time_s += dt;
        acc.friction_j += friction * omega.max(0.0) * dt;
        acc.manifold_j += manifold_heat_w * dt;
        acc.exotherm_j += react;
        acc.angle_rad += omega.max(0.0) * dt;
        acc.exhaust_kg += self.exhaust_mass_flow.max(0.0) * dt;
        acc.exhaust_kg_k += self.exhaust_mass_flow.max(0.0) * self.air.t_post_turbine * dt;
        if acc.time_s >= THERMAL_STEP_S {
            let span = acc.time_s;
            let ram_air = match &ctl.load {
                LoadModel::Neutral => 0.0,
                LoadModel::Dyno(d) => d.cooling_air_speed_m_s,
                // Grille-to-radiator air speed ≈ 35 % of road speed.
                LoadModel::Vehicle(_) => 0.35 * self.crank.vehicle_speed,
            };
            let exhaust_temp = if acc.exhaust_kg > 1.0e-9 {
                acc.exhaust_kg_k / acc.exhaust_kg
            } else {
                self.air.t_post_turbine
            };
            let inputs = ThermalInputs {
                wall_heat_j: acc.wall_heat_j,
                friction_heat_j: acc.friction_j,
                rpm: acc.angle_rad / span * RAD_S_TO_RPM,
                ambient_t: amb.temperature_k,
                ram_air_speed: ram_air,
                fan_on: cmd.fan,
                thermostat_fault: faults.thermostat,
                coolant_leak_per_s: dmg.coolant_leak_per_s,
                exhaust_mass_flow: acc.exhaust_kg / span,
                exhaust_temp,
                catalyst_exotherm_j: acc.exotherm_j,
                manifold_heat_j: acc.manifold_j,
                cylinders: n,
            };
            *acc = ThermalAccumulator::default();
            self.thermal_model.step(&mut self.thermal, &inputs, span);
        }

        // ---- Lubrication ------------------------------------------------------------------
        // Fuel diluting the oil lowers viscosity roughly exponentially with fuel fraction.
        let lub = &self.spec.lubrication;
        self.oil_viscosity =
            vogel_viscosity(lub.vogel_a, lub.vogel_b, lub.vogel_c, self.thermal.t_oil)
                * (-6.0 * self.oil_dilution).exp();
        // Gerotor delivery ∝ speed; pressure = flow × bearing leakage resistance ∝ μ
        // (Hagen–Poiseuille), capped by the relief valve.
        let p_oil_target = if omega > 1.0 {
            (lub.pressure_per_krpm * rpm / 1000.0
                * (self.oil_viscosity / self.viscosity_ref).powf(lub.viscosity_exponent))
            .min(lub.relief_pressure_pa)
                * dmg.oil_pressure_mult
        } else {
            0.0
        };
        self.oil_pressure_pa = approach(self.oil_pressure_pa, p_oil_target, dt, 0.15);
        // Fuel dilution: very rich running washes liquid fuel past the rings (cold-start
        // dilution is booked per cycle in `fire`); hot oil boils the light fractions back
        // out over ≈ 10 minutes at 110 °C. Above that the heavier fractions' vapour
        // pressure (Clausius–Clapeyron, roughly doubling every 15 K) speeds it up.
        let rich = (0.85 - self.exhaust_lambda).max(0.0) / 0.15;
        let wash = 1.2e-5 * self.fuel_mass_flow * 1000.0 * rich;
        let t_oil = self.thermal.t_oil;
        let volatility = ((t_oil - 383.0).clamp(0.0, 60.0) / 15.0).exp2();
        let boil_off = self.oil_dilution * smoothstep(343.0, 383.0, t_oil) * volatility / 600.0;
        self.oil_dilution = clampf(self.oil_dilution + (wash - boil_off) * dt, 0.0, 0.15);

        // ---- Fuel system ------------------------------------------------------------------
        let fs = &self.spec.fuel_system;
        let rail_target = if cmd.fuel_pump {
            let capacity = (fs.pump_capacity_kg_s * faults.fuel_pump_capacity).max(1.0e-6);
            // Injector demand referred to regulator pressure (nozzle flow ∝ √Δp).
            let rail_ratio = (self.fuel_rail_pa / fs.regulator_pressure_pa).max(0.05);
            let demand_at_reg = self.fuel_mass_flow / rail_ratio.sqrt();
            fs.regulator_pressure_pa * fuel_pump_operating_point(capacity, demand_at_reg)
        } else {
            0.0
        };
        self.fuel_rail_pa = approach(
            self.fuel_rail_pa,
            rail_target,
            dt,
            if cmd.fuel_pump { 0.15 } else { 5.0 },
        );

        // ---- Electrical -------------------------------------------------------------------
        let v_target = if !ctl.ignition_on {
            12.6
        } else if ctl.starter {
            // ≈ 150 A starter draw through the battery's internal resistance.
            10.4
        } else if charging {
            14.2 - 0.4 * ctl.electrical_load_w / 1500.0
        } else {
            12.4
        };
        self.battery_v = approach(self.battery_v, v_target, dt, 0.05);

        // ---- Mean flows and torque breakdown ------------------------------------------------
        let a = lag_alpha(dt, EXHAUST_FLOW_TAU);
        self.exhaust_mass_flow += (self.exhaust_mass_accum / dt - self.exhaust_mass_flow) * a;
        self.exhaust_enthalpy_flow +=
            (self.exhaust_enthalpy_accum / dt - self.exhaust_enthalpy_flow) * a;
        if self.exhaust_mass_flow > 1.0e-5 {
            self.exhaust_port_temp = self.exhaust_enthalpy_flow / self.exhaust_mass_flow;
        } else {
            self.exhaust_port_temp = approach(self.exhaust_port_temp, amb.temperature_k, dt, 10.0);
        }
        self.exhaust_mass_accum = 0.0;
        self.exhaust_enthalpy_accum = 0.0;
        self.fuel_mass_flow += (self.fuel_accum / dt - self.fuel_mass_flow) * lag_alpha(dt, 0.2);
        self.fuel_accum = 0.0;

        let m = lag_alpha(dt, MEAN_FILTER_TAU);
        // Cycle means describe a turning engine: a stopped crank does no indicated work,
        // whatever static torque its trapped charge exerts (still shown instantaneously).
        let gas_cycle = if self.crank.omega > 0.0 { gas } else { 0.0 };
        let tq = &mut self.torque;
        tq.gas_instant = gas;
        tq.indicated_mean += (gas_cycle - tq.indicated_mean) * m;
        tq.friction += (friction - tq.friction) * m;
        tq.pumping += (pumping - tq.pumping) * m;
        tq.accessory += (accessory - tq.accessory) * m;
        tq.brake_mean += (gas_cycle + pumping - friction - accessory - tq.brake_mean) * m;

        self.sanitize(&amb);
    }

    /// Last line of defence against non-finite state (e.g. from a pathological user
    /// calibration): any NaN/∞ is replaced by a physically safe value so one bad sample can
    /// never poison the integrators permanently.
    fn sanitize(&mut self, amb: &Ambient) {
        let p0 = amb.pressure_pa;
        let t0 = amb.temperature_k;
        self.crank.omega = finite_or(self.crank.omega, 0.0);
        self.crank.theta = finite_or(self.crank.theta, 0.0);
        self.crank.vehicle_speed = finite_or(self.crank.vehicle_speed, 0.0);
        self.air.p_man = finite_or(self.air.p_man, p0);
        self.air.p_boost = finite_or(self.air.p_boost, p0);
        self.air.p_exh = finite_or(self.air.p_exh, p0);
        self.air.t_man = finite_or(self.air.t_man, t0);
        self.air.t_boost = finite_or(self.air.t_boost, t0);
        self.air.t_exh = finite_or(self.air.t_exh, t0);
        self.air.t_post_turbine = finite_or(self.air.t_post_turbine, t0);
        self.air.turbo_omega = finite_or(self.air.turbo_omega, 0.0);
        self.thermal.t_metal = finite_or(self.thermal.t_metal, t0);
        self.thermal.t_coolant = finite_or(self.thermal.t_coolant, t0);
        self.thermal.t_oil = finite_or(self.thermal.t_oil, t0);
        self.thermal.t_catalyst = finite_or(self.thermal.t_catalyst, t0);
        self.thermal.t_manifold = finite_or(self.thermal.t_manifold, t0);
        self.exhaust_port_temp = finite_or(self.exhaust_port_temp, t0);
        self.exhaust_mass_flow = finite_or(self.exhaust_mass_flow, 0.0);
        self.exhaust_enthalpy_flow = finite_or(self.exhaust_enthalpy_flow, 0.0);
        self.fuel_mass_flow = finite_or(self.fuel_mass_flow, 0.0);
        self.exhaust_lambda = finite_or(self.exhaust_lambda, 1.0);
        self.exhaust_lambda_fuelled = finite_or(self.exhaust_lambda_fuelled, 1.0);
        self.exhaust_lambda_apparent = finite_or(self.exhaust_lambda_apparent, 1.0);
        self.oil_pressure_pa = finite_or(self.oil_pressure_pa, 0.0);
        self.fuel_rail_pa = finite_or(self.fuel_rail_pa, 0.0);
        self.peak_pressure_mean_bar = finite_or(self.peak_pressure_mean_bar, 1.0);
        self.hc_store_j = flush_tiny(finite_or(self.hc_store_j, 0.0));
        self.o2_store_kg = flush_tiny(finite_or(self.o2_store_kg, 0.0));
        // Exponentially decaying flows and filters end in subnormals unless flushed.
        self.exhaust_mass_flow = flush_tiny(self.exhaust_mass_flow);
        self.exhaust_enthalpy_flow = flush_tiny(self.exhaust_enthalpy_flow);
        self.fuel_mass_flow = flush_tiny(self.fuel_mass_flow);
        let tq = &mut self.torque;
        for v in [
            &mut tq.gas_instant,
            &mut tq.indicated_mean,
            &mut tq.friction,
            &mut tq.pumping,
            &mut tq.accessory,
            &mut tq.brake_mean,
        ] {
            *v = flush_tiny(finite_or(*v, 0.0));
        }
        for c in 0..self.cylinders {
            let cyl = &mut self.cyl[c];
            cyl.film_kg = finite_or(cyl.film_kg, 0.0);
            cyl.residual_kg = finite_or(cyl.residual_kg, 0.0);
            cyl.residual_temp_k = finite_or(cyl.residual_temp_k, t0);
            cyl.work_compression_j = finite_or(cyl.work_compression_j, 0.0);
            cyl.work_expansion_j = finite_or(cyl.work_expansion_j, 0.0);
            self.thermal.t_piston[c] = finite_or(self.thermal.t_piston[c], t0);
        }
    }

    /// Runs the closed-cycle model of cylinder `c` at the start of its compression stroke.
    #[allow(clippy::too_many_arguments)]
    fn fire(
        &mut self,
        c: usize,
        cmd: &ActuatorCommands,
        ctl: &Controls,
        faults: &FaultState,
        dmg: &DamageModifiers,
        rng: &mut Pcg32,
        dry_fraction: f32,
    ) {
        let omega = self.crank.omega.max(1.0);
        let rpm = omega * RAD_S_TO_RPM;
        let p_man = self.air.p_man;
        let t_man = self.air.t_man;
        let fresh = self.ve * p_man / (R_AIR * t_man) * self.geom.swept_volume * dry_fraction;
        let cycle_time = 120.0 / rpm.max(30.0);

        // Injector: flow ∝ √Δp (Bernoulli through the nozzle), opening delayed by the
        // voltage-dependent dead time.
        let fs = &self.spec.fuel_system;
        let dead =
            fs.injector_dead_time_s + fs.injector_dead_time_slope_s_per_v * (14.0 - self.battery_v);
        let open = (cmd.injector_pw_s[c].min(cycle_time) - dead).max(0.0);
        let flow = fs.injector_flow_kg_s
            * (self.fuel_rail_pa.max(0.0) / fs.injector_rated_pressure_pa).sqrt()
            * faults.injector_flow[c];
        let m_inj = open * flow;

        // Port wall film (Aquino X–τ): fraction X wets the wall, the film evaporates with
        // time constant τ; the cylinder receives the direct spray plus film evaporation.
        let t_port = self.thermal.t_metal;
        let x = breathing::wall_film_fraction(t_port, p_man);
        let tau = breathing::wall_film_tau(t_port);
        let cyl = &mut self.cyl[c];
        cyl.film_kg += x * m_inj;
        let evap = cyl.film_kg * (1.0 - (-cycle_time / tau).exp());
        cyl.film_kg -= evap;
        let fuel_delivered = (1.0 - x) * m_inj + evap;
        // Cold-wall quenching: part of the fuel never vaporises in time to burn.
        let cold = clampf((353.15 - t_port) / 90.0, 0.0, 1.4);
        let unvaporised = fuel_delivered * COLD_FUEL_LOSS * cold.powf(1.3);
        let fuel = fuel_delivered - unvaporised;

        // Charge temperature at IVC: port-heated fresh charge cooled by in-cylinder fuel
        // evaporation (ΔT = f·m_f·h_fg/(m·c_p)), mixed with hot residual gas.
        let fuel_spec = &self.spec.fuel;
        let dt_evap = fuel_spec.in_cylinder_evaporation * fuel * fuel_spec.latent_heat
            / ((fresh + fuel).max(1.0e-9) * CP_AIR);
        let t_charge = (self.t_fresh - dt_evap).max(200.0);
        let m_res = cyl.residual_kg;
        // The residual left at exhaust pressure expands (part load) or is compressed
        // (boost) isentropically to manifold pressure as the intake stroke begins.
        let res_pr = clampf(p_man / self.air.p_exh.max(1.0e3), 0.1, 4.0);
        let t_res = cyl.residual_temp_k * res_pr.powf((GAMMA_EXHAUST - 1.0) / GAMMA_EXHAUST);
        let t_ivc = (fresh * t_charge + RESIDUAL_CP_RATIO * m_res * t_res)
            / (fresh + RESIDUAL_CP_RATIO * m_res).max(1.0e-12);

        // Ignition: secondary voltage scales with primary current, i.e. battery voltage.
        let spark_kv = self.spec.combustion.coil_output_kv
            * faults.coil_strength[c]
            * (self.battery_v / 14.0).min(1.05);

        // Cycle-to-cycle variation: burn-rate scatter grows with dilution (σ ≈ 3 % at
        // full load, 6–7 % at idle with 12 % residuals), end-gas delay scatter ≈ 15 %.
        let x_r = m_res / (m_res + fresh + fuel).max(1.0e-12);
        let sigma = 0.03 + 0.3 * x_r;
        let burn_ccv = clampf(1.0 + sigma * rng.normal(), 0.75, 1.35);
        let knock_ccv = (0.15 * rng.normal()).exp();
        let breakdown_noise = rng.normal();

        let leak = clampf(
            faults.compression_leak[c] + dmg.compression_leak[c],
            0.0,
            1.0,
        );
        let wall_temp = 0.6 * self.thermal.t_metal + 0.4 * self.thermal.t_piston[c];
        let inputs = CycleInputs {
            rpm,
            fresh_air_kg: fresh,
            fuel_kg: fuel,
            residual_kg: m_res,
            t_ivc_k: t_ivc,
            spark_btdc_deg: cmd.spark_btdc_deg[c],
            spark_enabled: cmd.spark_enabled[c],
            spark_voltage_kv: spark_kv,
            breakdown_noise,
            octane_ron: ctl.fuel_octane_ron,
            wall_temp_k: wall_temp,
            exhaust_pressure_pa: self.air.p_exh,
            burn_ccv,
            knock_ccv,
            compression_leak: leak,
        };
        let r = self.combustion.simulate(&inputs);

        let cyl = &mut self.cyl[c];
        cyl.last_inputs = Some(inputs);
        cyl.residual_kg = r.residual_next_kg;
        cyl.residual_temp_k = r.residual_temp_next_k;
        cyl.work_compression_j = r.work_compression_j;
        cyl.work_expansion_j = r.work_expansion_j;
        cyl.last = r;
        cyl.events = cyl.events.wrapping_add(1);
        cyl.injected_kg = m_inj;
        cyl.trapped_air_kg = fresh;
        self.fired[c] = true;

        // Heat to walls: the piston crown takes its geometric share plus all of the extra
        // flux that detonation drives into it (computed inside the cycle, so it has already
        // been taken out of the gas).
        let share = self.spec.thermal.piston_heat_share;
        let base = r.wall_heat_j - r.knock_wall_heat_j;
        self.thermal_acc.wall_heat_j += base * (1.0 - share);
        self.thermal_model.add_piston_heat(
            &mut self.thermal,
            c,
            base * share + r.knock_wall_heat_j,
        );

        // Exhaust port: longer residence at low speed → larger fraction lost to the head.
        let port_fraction = clampf(
            EXHAUST_PORT_HEAT_FRACTION * (3000.0 / rpm.max(300.0)).powf(0.3),
            0.06,
            0.25,
        );
        let port_heat = port_fraction
            * r.exhaust_mass_kg
            * CP_EXHAUST
            * (r.exhaust_temp_k - self.thermal.t_metal).max(0.0);
        self.thermal_acc.wall_heat_j += port_heat;
        let t_port_exit =
            r.exhaust_temp_k - port_heat / (r.exhaust_mass_kg * CP_EXHAUST).max(1.0e-9);
        self.exhaust_mass_accum += r.exhaust_mass_kg;
        self.exhaust_enthalpy_accum += r.exhaust_mass_kg * t_port_exit;
        self.fuel_accum += m_inj;
        self.hc_store_j += r.unburned_fuel_energy_j
            + unvaporised * (1.0 - COLD_FUEL_TO_OIL) * self.spec.fuel.lower_heating_value;
        self.o2_store_kg += r.unused_air_kg;
        // Oil mass ≈ 3.8 kg: dilution increment = fuel mass / oil mass.
        self.oil_dilution = clampf(
            self.oil_dilution + unvaporised * COLD_FUEL_TO_OIL / OIL_MASS_KG,
            0.0,
            0.15,
        );

        // Exhaust λ: true value for telemetry; the O2 sensor additionally reads unburned
        // oxygen from misfires/partial burns as "lean" because the unburned fuel is not
        // oxidised at the sensor.
        let n = self.cylinders as f32;
        let lam = r.lambda.min(3.0);
        self.exhaust_lambda += (lam - self.exhaust_lambda) / n;
        // Fuelled = fuel commanded by the ECU (commanded cuts excluded), whatever the
        // injector actually delivers: a dead injector is a lean cylinder, not a fuel cut.
        if cmd.injector_pw_s[c] > 0.0 {
            self.exhaust_lambda_fuelled += (lam - self.exhaust_lambda_fuelled) / n;
        }
        // The wide-band sensor's catalytic electrode oxidises exhaust hydrocarbons, so it
        // sees the λ of *all* fuel delivered, including the unvaporised fraction.
        let lam_total = if fuel_delivered > 1.0e-12 {
            (fresh
                / ((fuel_delivered - unvaporised * COLD_FUEL_TO_OIL) * self.spec.fuel.stoich_afr))
                .min(3.0)
        } else {
            3.0
        };
        let burned = if r.misfire {
            0.0
        } else {
            r.mass_fraction_burned_evo + 0.7 * (1.0 - r.mass_fraction_burned_evo)
        };
        let lam_app = (lam_total
            + 1.5 * (1.0 - burned) * if r.fuel_energy_j > 0.0 { 1.0 } else { 0.0 })
        .min(3.0);
        self.exhaust_lambda_apparent += (lam_app - self.exhaust_lambda_apparent) * LAMBDA_MIX_ALPHA;
        self.peak_pressure_mean_bar +=
            (r.peak_pressure_pa * 1.0e-5 - self.peak_pressure_mean_bar) * 0.1;
    }

    /// Crank-wheel speed sampled at every firing TDC, interpolated to the crossing
    /// instant within the step (production ECUs time a short tooth window around TDC).
    /// The change between consecutive TDCs equals the net work of the cylinder whose
    /// power stroke lay between them divided by J·ω — the basis of misfire detection.
    fn track_segments(&mut self, theta0: f32, dtheta: f32, omega0: f32) {
        if dtheta <= 0.0 {
            return;
        }
        let n = self.cylinders;
        let seg_angle = CYCLE_RAD / n as f32;
        for pos in 0..n {
            let d = wrap_cycle(pos as f32 * seg_angle - theta0);
            if d < dtheta {
                let frac = d / dtheta;
                self.segment.omega = omega0 + frac * (self.crank.omega - omega0);
                self.segment.seq = self.segment.seq.wrapping_add(1);
                // The segment that just ended was the power stroke of the cylinder whose
                // TDC opened it: the previous firing position.
                self.segment.cylinder = self.firing_seq[(pos + n - 1) % n] as u8;
                break;
            }
        }
    }
}

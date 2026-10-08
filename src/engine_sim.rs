//! # Engine simulation core (Phase 1)
//!
//! [`EngineSim`] orchestrates four layers, mirroring a real vehicle:
//!
//! ```text
//!   Controls ──► Plant (physics truth) ──► Sensors ──► ECU ──► Actuators ──► Plant …
//!                    │                                   ▲
//!                    └── Damage model ◄──────────────────┘ (loads)     Faults inject into
//!                                                                      plant & sensors
//! ```
//!
//! * The **plant** integrates the mean-value air path, crank dynamics and thermal network
//!   at a fixed 4 kHz step and runs a crank-angle-resolved combustion model for every
//!   cylinder event.
//! * The **sensor** layer adds lags, transport delays, noise and fault offsets.
//! * The **ECU** sees only sensors and its user-editable [`Calibration`].
//! * The **damage** model converts physical loads into component wear and failures.
//!
//! ## Real-time contract
//!
//! [`EngineSim::tick`] performs no heap allocation, no locking and no I/O. All per-cylinder
//! storage is statically sized ([`MAX_CYLINDERS`]); event logs are fixed-size rings.
//! Variable frame times are absorbed by a fixed-step accumulator, so physics, ECU, damage,
//! warnings and the event log are deterministic for a given seed and input sequence
//! regardless of the caller's frame rate; only the [`Telemetry`] snapshot is taken per
//! [`EngineSim::tick`].

mod airpath;
mod breathing;
pub mod calibration;
mod combustion;
pub mod controls;
pub mod damage;
pub mod dtc;
mod dynamics;
pub mod ecu;
pub mod events;
pub mod faults;
pub mod i18n;
mod math;
mod plant;
pub mod sensors;
pub mod spec;
pub mod status;
pub mod telemetry;
mod thermal;
mod thermo;

use core::fmt;

use calibration::{Calibration, CalibrationError};
use controls::Controls;
use damage::{DamageContext, DamageModel, DamageState};
use dtc::DtcSet;
use ecu::{Ecu, EcuMode};
use events::{CylinderEvent, CylinderEventRing, EventKind, EventLog};
use faults::{Fault, FaultError, FaultId, FaultState};
use i18n::{Language, Localize, MessageKey};
use math::{Pcg32, RAD_S_TO_RPM};
use plant::Plant;
use sensors::SensorState;
use spec::{EngineSpec, SpecError, MAX_CYLINDERS};
use status::{EngineCondition, Warning, Warnings};
use telemetry::Telemetry;
use thermo::ZERO_CELSIUS_K;

/// Fixed integration step \[s\]: 4 kHz. At 8000 rpm the crank turns 12° per step, giving
/// ≥ 15 samples per 180° torque pulse; manifold filling (τ ≈ 10–50 ms) and the turbo
/// rotor are resolved with large margins.
pub const SUBSTEP_S: f32 = 2.5e-4;

/// Largest frame time accepted per [`EngineSim::tick`] \[s\]. Longer stalls of the caller
/// (debugger, window drag) are dropped instead of being simulated in one burst, which
/// would blow the audio thread's deadline.
pub const MAX_TICK_S: f32 = 0.25;

/// Exhaust manifold gas temperature above which exhaust valves, manifold and catalyst
/// substrate age rapidly \[°C\].
const HIGH_EGT_C: f32 = 980.0;
/// Warning margin below the turbine inlet temperature limit on turbo engines \[K\].
const TURBINE_EGT_MARGIN_K: f32 = 50.0;
/// Catalyst warning margin below the substrate melting temperature \[K\].
const CATALYST_WARNING_MARGIN_K: f32 = 50.0;

/// Refresh period of the MBT estimate in telemetry \[s\].
const MBT_REFRESH_S: f64 = 0.1;

/// Time a knock or misfire indication is held for warnings \[s\].
const WARNING_HOLD_S: f32 = 0.5;

/// Errors returned by [`EngineSim`] constructors and setters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimError {
    /// Invalid engine specification.
    Spec(SpecError),
    /// Invalid calibration.
    Calibration(CalibrationError),
    /// Invalid fault.
    Fault(FaultError),
    /// Calibration was made for a different cylinder count than the engine.
    CylinderMismatch,
}

impl fmt::Display for SimError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Spec(e) => write!(f, "engine specification: {e}"),
            Self::Calibration(e) => write!(f, "calibration: {e}"),
            Self::Fault(e) => write!(f, "fault: {e}"),
            Self::CylinderMismatch => {
                write!(f, "calibration cylinder count does not match the engine")
            }
        }
    }
}

impl std::error::Error for SimError {}

impl From<SpecError> for SimError {
    fn from(e: SpecError) -> Self {
        Self::Spec(e)
    }
}

impl From<CalibrationError> for SimError {
    fn from(e: CalibrationError) -> Self {
        Self::Calibration(e)
    }
}

impl From<FaultError> for SimError {
    fn from(e: FaultError) -> Self {
        Self::Fault(e)
    }
}

/// The complete engine simulation.
#[derive(Debug, Clone)]
pub struct EngineSim {
    spec: EngineSpec,
    calibration: Calibration,
    controls: Controls,
    plant: Plant,
    ecu: Ecu,
    sensors: SensorState,
    damage_model: DamageModel,
    damage: DamageState,
    faults: FaultState,
    rng: Pcg32,
    seed: u64,
    log: EventLog,
    cylinder_events: CylinderEventRing,
    telemetry: Telemetry,
    time_s: f64,
    accumulator: f64,
    condition: EngineCondition,
    warnings: Warnings,
    knock_hold: [f32; MAX_CYLINDERS],
    misfire_hold: f32,
    fired_prev: [bool; MAX_CYLINDERS],
    mbt_cache_deg: f32,
    mbt_cache_time: f64,
}

impl EngineSim {
    /// Creates a cold engine at ambient temperature with ignition off.
    pub fn new(spec: EngineSpec, calibration: Calibration, seed: u64) -> Result<Self, SimError> {
        spec.validate()?;
        calibration.validate()?;
        if calibration.engine.cylinders != spec.geometry.cylinders {
            return Err(SimError::CylinderMismatch);
        }
        let controls = Controls::default();
        let ambient = controls.ambient;
        let sensors = SensorState::new(ambient.temperature_k, ambient.pressure_pa);
        let telemetry = Telemetry::blank(sensors.readings);
        let mut sim = Self {
            spec,
            calibration,
            controls,
            plant: Plant::new(&spec, &ambient),
            ecu: Ecu::default(),
            sensors,
            damage_model: DamageModel::new(&spec),
            damage: DamageState::default(),
            faults: FaultState::default(),
            rng: Pcg32::new(seed, 0x7475_6e65_726c_6162),
            seed,
            log: EventLog::default(),
            cylinder_events: CylinderEventRing::default(),
            telemetry,
            time_s: 0.0,
            accumulator: 0.0,
            condition: EngineCondition::Off,
            warnings: Warnings::NONE,
            knock_hold: [0.0; MAX_CYLINDERS],
            misfire_hold: 0.0,
            fired_prev: [false; MAX_CYLINDERS],
            mbt_cache_deg: 0.0,
            mbt_cache_time: f64::NEG_INFINITY,
        };
        sim.update_warnings();
        sim.refresh_telemetry();
        Ok(sim)
    }

    /// Creates a simulation with the generated factory base calibration for `spec`.
    pub fn with_base_calibration(spec: EngineSpec, seed: u64) -> Result<Self, SimError> {
        spec.validate()?;
        let cal = Calibration::base_for(&spec);
        Self::new(spec, cal, seed)
    }

    /// Advances the simulation by `delta_time` seconds of wall-clock time.
    ///
    /// Allocation-free. Time is accumulated and consumed in fixed [`SUBSTEP_S`] steps;
    /// non-finite or non-positive inputs are ignored and frame times above
    /// [`MAX_TICK_S`] are truncated.
    pub fn tick(&mut self, delta_time: f32) {
        if !delta_time.is_finite() || delta_time <= 0.0 {
            return;
        }
        self.accumulator += f64::from(delta_time.min(MAX_TICK_S));
        let step = f64::from(SUBSTEP_S);
        while self.accumulator >= step {
            self.substep(SUBSTEP_S);
            self.accumulator -= step;
        }
        self.refresh_telemetry();
    }

    fn substep(&mut self, dt: f32) {
        let ctl = self.controls;
        let n = self.spec.geometry.cylinders;
        let now = self.time_s;

        self.sensors
            .update(&self.plant, &ctl, &self.faults, dt, &mut self.rng);
        let cmd = self.ecu.update(
            &self.sensors.readings,
            &ctl,
            &self.calibration,
            &self.fired_prev,
            dt,
            now,
            &mut self.log,
        );
        let modifiers = self.damage_model.modifiers(&self.damage);
        self.plant
            .step(&cmd, &ctl, &self.faults, &modifiers, &mut self.rng, dt);

        let omega = self.plant.crank.omega;
        // Delays from the cycle computation (start of compression, BDC) to firing TDC and
        // to exhaust valve opening, for audio scheduling.
        let omega_ev = omega.max(1.0);
        let time_to_tdc = core::f32::consts::PI / omega_ev;
        let evo_atdc = (180.0 - self.spec.valves.evo_bbdc_deg).to_radians();
        let time_to_evo = (core::f32::consts::PI + evo_atdc) / omega_ev;
        for c in 0..n {
            if !self.plant.fired[c] {
                continue;
            }
            let r = self.plant.cyl[c].last;
            self.damage_model.on_cylinder_event(
                &mut self.damage,
                c,
                &r,
                omega,
                &mut self.rng,
                &mut self.log,
                now,
            );
            self.cylinder_events.push(CylinderEvent {
                seq: 0,
                time_s: now,
                time_to_tdc_s: time_to_tdc,
                time_to_evo_s: time_to_evo,
                cylinder: c as u8,
                rpm: omega * RAD_S_TO_RPM,
                peak_pressure_bar: r.peak_pressure_pa * 1.0e-5,
                peak_pressure_angle_deg: r.peak_pressure_angle_deg,
                knock_intensity: r.knock_intensity,
                misfire: r.misfire,
                indicated_work_j: r.net_work_j,
                exhaust_temp_k: r.exhaust_temp_k,
                afterburn_j: r.afterburn_j,
                blowdown_pressure_bar: r.blowdown_pressure_pa * 1.0e-5,
            });
            self.knock_hold[c] = self.knock_hold[c].max(r.knock_intensity);
            if r.misfire {
                self.misfire_hold = WARNING_HOLD_S;
            }
        }
        self.fired_prev = self.plant.fired;

        let p = &self.plant;
        let v_d = self.spec.geometry.displacement_m3();
        let imep = p.torque.indicated_mean.max(0.0) * 4.0 * core::f32::consts::PI / v_d;
        let ctx = DamageContext {
            rpm: p.rpm(),
            oil_pressure_pa: p.oil_pressure_pa,
            t_metal: p.thermal.t_metal,
            t_piston: p.thermal.t_piston,
            turbo_omega: p.air.turbo_omega,
            turbine_inlet_k: p.air.t_exh,
            t_catalyst: p.thermal.t_catalyst,
            load: imep / 1.0e6,
        };
        self.damage_model
            .update(&mut self.damage, &ctx, dt, &mut self.log, now);

        let decay = (-dt / WARNING_HOLD_S).exp();
        for k in &mut self.knock_hold[..n] {
            *k *= decay;
        }
        self.misfire_hold = (self.misfire_hold - dt).max(0.0);

        let condition = self.evaluate_condition();
        if condition != self.condition {
            self.condition = condition;
            self.log.push(now, EventKind::ConditionChanged(condition));
        }
        self.time_s += f64::from(dt);
        self.update_warnings();
    }

    /// Re-evaluates the instructor warnings and logs the ones that just became active.
    fn update_warnings(&mut self) {
        let warnings = self.evaluate_warnings();
        for w in warnings.newly_set(self.warnings).iter() {
            self.log.push(self.time_s, EventKind::Warning(w));
        }
        self.warnings = warnings;
    }

    fn evaluate_condition(&self) -> EngineCondition {
        if let Some(cause) = self.damage.fatal {
            return EngineCondition::Failed(cause);
        }
        if !self.controls.ignition_on {
            return EngineCondition::Off;
        }
        match self.ecu.mode {
            EcuMode::Cranking => EngineCondition::Cranking,
            EcuMode::Running => EngineCondition::Running,
            EcuMode::Stopped | EcuMode::Off => EngineCondition::Stalled,
        }
    }

    fn evaluate_warnings(&self) -> Warnings {
        let p = &self.plant;
        let lim = &self.spec.limits;
        let n = self.spec.geometry.cylinders;
        let rpm = p.rpm();
        let running = self.condition == EngineCondition::Running;
        let mut w = Warnings::NONE;
        w.set(
            Warning::Knock,
            self.knock_hold[..n].iter().any(|k| *k > 1.0),
        );
        w.set(Warning::OverRev, rpm > lim.redline_rpm);
        w.set(Warning::ValveFloat, p.valve_float > 0.0);
        w.set(
            Warning::Overheat,
            p.thermal.t_coolant > ZERO_CELSIUS_K + 112.0,
        );
        w.set(
            Warning::CoolantBoiling,
            p.thermal.t_coolant > self.spec.thermal.coolant_boil_k,
        );
        let required_oil = (lim.bearing_oil_per_krpm * rpm / 1000.0).max(20_000.0);
        w.set(
            Warning::LowOilPressure,
            rpm > 500.0 && p.oil_pressure_pa < required_oil,
        );
        w.set(
            Warning::HighOilTemp,
            p.thermal.t_oil > ZERO_CELSIUS_K + 135.0,
        );
        // Not during commanded fuel cuts, whose film-only cycles are lean by design.
        let fuel_cut = self.ecu.rev_cut || self.ecu.dfco || self.ecu.overboost_cut;
        w.set(
            Warning::LeanUnderLoad,
            running
                && !fuel_cut
                && p.exhaust_lambda > 1.05
                && p.air.p_man > 0.8 * self.controls.ambient.pressure_pa
                && rpm > 1500.0,
        );
        w.set(Warning::RichMixture, running && p.exhaust_lambda < 0.7);
        // Turbine wheels set the EGT ceiling on turbo engines, exhaust valves and the
        // catalyst on naturally aspirated ones.
        let egt_limit = match &self.spec.turbo {
            Some(t) => t.turbine_inlet_limit_k - TURBINE_EGT_MARGIN_K,
            None => ZERO_CELSIUS_K + HIGH_EGT_C,
        };
        w.set(Warning::HighEgt, p.air.t_exh > egt_limit);
        w.set(
            Warning::PistonOverheat,
            p.thermal.t_piston[..n]
                .iter()
                .any(|t| *t > lim.piston_crown_limit_k),
        );
        if let Some(t) = &self.spec.turbo {
            // Hardware limit, not the (student-editable) ECU fuel-cut threshold.
            w.set(Warning::Overboost, p.air.p_man > t.max_manifold_pressure_pa);
            w.set(Warning::CompressorSurge, p.air.compressor_surge);
            w.set(
                Warning::TurboOverspeed,
                p.air.turbo_omega * RAD_S_TO_RPM > t.max_speed_rpm,
            );
        }
        w.set(Warning::Misfire, running && self.misfire_hold > 0.0);
        w.set(
            Warning::InjectorDutyHigh,
            running && self.ecu.injector_duty > 0.88,
        );
        w.set(
            Warning::CatalystOverheat,
            p.thermal.t_catalyst > lim.catalyst_melt_k - CATALYST_WARNING_MARGIN_K,
        );
        w.set(Warning::CheckEngine, !self.ecu.dtcs.is_empty());
        w
    }

    fn refresh_telemetry(&mut self) {
        let warnings = self.warnings;
        let p = &self.plant;
        let e = &self.ecu;
        let n = self.spec.geometry.cylinders;
        let c_to = |k: f32| k - ZERO_CELSIUS_K;
        let mut t = Telemetry::blank(self.sensors.readings);
        t.time_s = self.time_s;
        t.condition = self.condition;
        t.ecu_mode = e.mode;
        t.cylinders = n;
        t.rpm = p.rpm();
        t.crank_angle_deg = p.crank.theta.to_degrees();
        t.pedal_pct = self.controls.pedal * 100.0;
        t.throttle_pct = p.air.throttle_pos * 100.0;
        t.idle_valve_pct = p.air.idle_valve_pos * 100.0;
        t.manifold_pressure_kpa = p.air.p_man * 1.0e-3;
        t.boost_pressure_kpa = p.air.p_boost * 1.0e-3;
        t.exhaust_pressure_kpa = p.air.p_exh * 1.0e-3;
        t.intake_air_temp_c = c_to(p.air.t_man);
        t.coolant_temp_c = c_to(p.thermal.t_coolant);
        t.oil_temp_c = c_to(p.thermal.t_oil);
        t.metal_temp_c = c_to(p.thermal.t_metal);
        t.oil_pressure_kpa = p.oil_pressure_pa * 1.0e-3;
        t.fuel_pressure_kpa = p.fuel_rail_pa * 1.0e-3;
        t.battery_v = p.battery_v;
        t.coolant_level = p.thermal.coolant_level;
        t.thermostat_pos = p.thermal.thermostat_pos;
        t.lambda = p.exhaust_lambda;
        t.lambda_target = e.lambda_target;
        t.egt_c = c_to(p.air.t_exh);
        t.catalyst_temp_c = c_to(p.thermal.t_catalyst);
        t.brake_torque_nm = p.torque.brake_mean;
        t.indicated_torque_nm = p.torque.indicated_mean;
        t.friction_torque_nm = p.torque.friction;
        t.pumping_torque_nm = p.torque.pumping;
        t.instantaneous_torque_nm = p.torque.gas_instant;
        t.load_torque_nm = p.crank.load_torque;
        t.power_kw = p.torque.brake_mean * p.crank.omega * 1.0e-3;
        t.volumetric_efficiency = p.ve;
        t.ve_table = e.ve_value;
        t.air_flow_g_s = p.air.mdot_cyl * 1.0e3;
        t.fuel_flow_g_s = p.fuel_mass_flow * 1.0e3;
        t.ignition_advance_deg = e.spark_mean_deg;
        // MBT is an optimisation over the cycle model; refresh it at 10 Hz so the cost
        // stays negligible however often the caller ticks.
        if self.time_s - self.mbt_cache_time >= MBT_REFRESH_S {
            self.mbt_cache_time = self.time_s;
            let running = self.condition == EngineCondition::Running;
            self.mbt_cache_deg = if running {
                self.plant
                    .mbt_for_cylinder(self.plant.firing_seq[0])
                    .unwrap_or(0.0)
            } else {
                0.0
            };
        }
        let p = &self.plant;
        for c in 0..n {
            let last = &p.cyl[c].last;
            t.knock_intensity[c] = last.knock_intensity;
            t.knock_retard_deg[c] = e.knock_retard[c];
            t.peak_pressure_bar[c] = last.peak_pressure_pa * 1.0e-5;
            t.piston_temp_c[c] = c_to(p.thermal.t_piston[c]);
            t.cylinder_misfire[c] = last.misfire;
            t.misfire_count[c] = e.misfire_total[c];
        }
        t.mbt_advance_deg = self.mbt_cache_deg;
        t.injector_pulse_ms = e.pulse_width_s * 1.0e3;
        t.injector_duty_pct = e.injector_duty * 100.0;
        t.stft_pct = e.stft * 100.0;
        t.ltft_pct = e.ltft * 100.0;
        t.closed_loop = e.closed_loop;
        t.turbo_rpm = p.air.turbo_omega * RAD_S_TO_RPM;
        t.wastegate_pct = p.air.wastegate_pos * 100.0;
        t.boost_target_kpa = e.boost_target_kpa;
        t.compressor_surge = p.air.compressor_surge;
        t.vehicle_speed_kph = p.crank.vehicle_speed * 3.6;
        t.warnings = warnings;
        t.health = self.damage.health;
        t.dtc_count = e.dtcs.len();
        self.telemetry = t;
    }

    /// Latest telemetry snapshot.
    pub fn telemetry(&self) -> &Telemetry {
        &self.telemetry
    }

    /// Operator inputs.
    pub fn controls(&self) -> &Controls {
        &self.controls
    }

    /// Replaces the operator inputs (values are sanitised: NaN-safe and range-clamped).
    pub fn set_controls(&mut self, controls: Controls) {
        self.controls = controls.sanitized();
    }

    /// Edits the operator inputs in place; the result is sanitised.
    pub fn update_controls(&mut self, f: impl FnOnce(&mut Controls)) {
        let mut c = self.controls;
        f(&mut c);
        self.controls = c.sanitized();
    }

    /// Active calibration.
    pub fn calibration(&self) -> &Calibration {
        &self.calibration
    }

    /// Flashes a new calibration (validated; rejected calibrations leave the ECU untouched).
    ///
    /// When anything that sets the base fuel quantity changes (engine constants, VE or λ
    /// tables, injector data, temperature model, enrichments, wall-film model), the learned
    /// fuel trims are discarded: they corrected the *old* tables and would otherwise
    /// distort the first drive on the new ones, as a real reflash clears adaptive memory.
    pub fn set_calibration(&mut self, calibration: Calibration) -> Result<(), SimError> {
        calibration.validate()?;
        if calibration.engine.cylinders != self.spec.geometry.cylinders {
            return Err(SimError::CylinderMismatch);
        }
        let old = &self.calibration;
        let fuel_changed = old.engine != calibration.engine
            || old.ve != calibration.ve
            || old.lambda_target != calibration.lambda_target
            || old.injector != calibration.injector
            || old.charge_temp_blend != calibration.charge_temp_blend
            || old.warmup_enrichment != calibration.warmup_enrichment
            || old.after_start_enrichment != calibration.after_start_enrichment
            || old.after_start_decay_s != calibration.after_start_decay_s
            || old.wall_film_fraction != calibration.wall_film_fraction
            || old.wall_film_tau_s != calibration.wall_film_tau_s;
        if fuel_changed {
            self.ecu.reset_fuel_trims();
        }
        self.calibration = calibration;
        Ok(())
    }

    /// Live-edits the calibration; the edit is validated and rolled back if invalid.
    pub fn update_calibration(&mut self, f: impl FnOnce(&mut Calibration)) -> Result<(), SimError> {
        let mut cal = self.calibration;
        f(&mut cal);
        self.set_calibration(cal)
    }

    /// Engine definition.
    pub fn spec(&self) -> &EngineSpec {
        &self.spec
    }

    /// Diagnostic event log.
    pub fn events(&self) -> &EventLog {
        &self.log
    }

    /// Combustion event ring (audio / cycle analysis feed).
    pub fn cylinder_events(&self) -> &CylinderEventRing {
        &self.cylinder_events
    }

    /// Stored diagnostic trouble codes.
    pub fn dtcs(&self) -> &DtcSet {
        &self.ecu.dtcs
    }

    /// Clears stored trouble codes, OBD monitor progress and learned fuel trims (scan-tool
    /// function).
    pub fn clear_dtcs(&mut self) {
        self.ecu.clear_codes();
        self.update_warnings();
        self.refresh_telemetry();
    }

    /// Injects a hidden fault (Engine Autopsy). Rejected when a parameter is out of range
    /// or too mild to have any effect, or when the engine lacks the hardware (turbo faults
    /// on a naturally aspirated engine).
    pub fn inject_fault(&mut self, fault: Fault) -> Result<(), SimError> {
        self.faults.apply(
            fault,
            self.spec.geometry.cylinders,
            self.spec.turbo.is_some(),
        )?;
        self.refresh_telemetry();
        Ok(())
    }

    /// Removes all injected faults.
    pub fn clear_faults(&mut self) {
        self.faults = FaultState::default();
        self.refresh_telemetry();
    }

    /// Whether a fault of the given kind is active (answer checking).
    pub fn is_fault_active(&self, id: FaultId) -> bool {
        self.faults.is_active(id)
    }

    /// Brings all thermal masses to operating temperature (skips the warm-up phase of a
    /// lesson). Has no effect while the engine has failed.
    pub fn prewarm(&mut self) {
        if self.damage.fatal.is_none() {
            self.plant.prewarm(self.controls.ambient.temperature_k);
            self.sensors.settle(&self.plant, &self.faults);
            self.update_warnings();
            self.refresh_telemetry();
        }
    }

    /// Replaces every damaged component and refills the coolant ("rebuild"). A running
    /// engine keeps running; a destroyed one is rebuilt at standstill, ready to restart.
    pub fn repair(&mut self) {
        let was_destroyed = self.damage.fatal.is_some();
        self.damage = DamageState::default();
        self.plant.thermal.coolant_level = 1.0;
        if was_destroyed {
            self.plant.stop_crank();
        }
        self.knock_hold = [0.0; MAX_CYLINDERS];
        self.misfire_hold = 0.0;
        let condition = self.evaluate_condition();
        if condition != self.condition {
            self.condition = condition;
            self.log
                .push(self.time_s, EventKind::ConditionChanged(condition));
        }
        self.update_warnings();
        self.refresh_telemetry();
    }

    /// Cold-soaks the stopped engine: every thermal mass and sensor settles at the current
    /// ambient temperature (e.g. to start a −10 °C cold-start lesson after changing
    /// [`Controls::ambient`]). Ignored while the crankshaft is turning.
    pub fn soak_to_ambient(&mut self) {
        if self.plant.crank.omega > 0.0 {
            return;
        }
        let ambient = self.controls.ambient;
        let t = ambient.temperature_k;
        let coolant_level = self.plant.thermal.coolant_level;
        self.plant.thermal = thermal::ThermalState::uniform(t);
        self.plant.thermal.coolant_level = coolant_level;
        self.plant.air.t_man = t;
        self.plant.air.t_boost = t;
        self.plant.air.t_exh = t;
        self.plant.air.t_post_turbine = t;
        self.plant.exhaust_port_temp = t;
        self.sensors.settle(&self.plant, &self.faults);
        self.sensors.readings.egt_c = t - ZERO_CELSIUS_K;
        self.update_warnings();
        self.refresh_telemetry();
    }

    /// Restarts the simulation from a cold engine with ignition off, keeping engine,
    /// calibration, seed and ambient conditions. Faults, damage, all other controls and the
    /// ECU (including stored trouble codes and learned fuel trims) are reset. Retained
    /// events are discarded, but event sequence numbers keep counting, so cursors held for
    /// [`EventLog::since`] stay valid.
    pub fn reset(&mut self) {
        let ambient = self.controls.ambient;
        self.controls = Controls {
            ambient,
            ..Controls::default()
        };
        self.plant = Plant::new(&self.spec, &ambient);
        self.ecu = Ecu::default();
        self.sensors = SensorState::new(ambient.temperature_k, ambient.pressure_pa);
        self.damage = DamageState::default();
        self.faults = FaultState::default();
        self.rng = Pcg32::new(self.seed, 0x7475_6e65_726c_6162);
        self.log.clear();
        self.cylinder_events.clear();
        self.time_s = 0.0;
        self.accumulator = 0.0;
        self.condition = EngineCondition::Off;
        self.warnings = Warnings::NONE;
        self.knock_hold = [0.0; MAX_CYLINDERS];
        self.misfire_hold = 0.0;
        self.fired_prev = [false; MAX_CYLINDERS];
        self.mbt_cache_deg = 0.0;
        self.mbt_cache_time = f64::NEG_INFINITY;
        self.update_warnings();
        self.refresh_telemetry();
    }

    /// Simulation time \[s\].
    pub fn time_s(&self) -> f64 {
        self.time_s
    }

    /// Current operating condition.
    pub fn condition(&self) -> EngineCondition {
        self.condition
    }

    /// Message key of the one-line engine status (the failure cause when failed), for
    /// rendering through any [`i18n::Localizer`].
    pub fn status_key(&self) -> MessageKey {
        match self.condition {
            EngineCondition::Failed(cause) => MessageKey::Failure(cause),
            c => MessageKey::Condition(c),
        }
    }

    /// Localised one-line engine status from the built-in tables (failure cause when
    /// failed).
    pub fn status_text(&self, lang: Language) -> &'static str {
        self.status_key().localized(lang)
    }
}

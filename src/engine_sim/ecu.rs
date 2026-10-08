//! Engine control unit emulation.
//!
//! The ECU sees only [`SensorReadings`] and its [`Calibration`]. It implements the control
//! strategies of a contemporary port-injected production ECU: speed-density fuelling with
//! transient (wall-film) compensation, closed-loop λ with short/long-term trims, spark
//! scheduling with temperature corrections, per-cylinder knock control, idle-speed
//! control, rev limiter, deceleration fuel cut-off, boost control and OBD-II monitors.

use super::calibration::Calibration;
use super::controls::Controls;
use super::dtc::{DtcCode, DtcSet};
use super::events::{EventKind, EventLog};
use super::math::{approach, clampf, finite_or, smoothstep};
use super::sensors::SensorReadings;
use super::spec::MAX_CYLINDERS;
use super::thermo::{R_AIR, ZERO_CELSIUS_K};

/// ECU torque-model constant: indicated work per kg of trapped air \[J/kg\]
/// (≈ 0.34 indicated efficiency × 2.95 MJ chemical energy per kg of air at λ = 1).
/// Used to scale the misfire monitor's expected crank-speed drop.
const SPECIFIC_INDICATED_WORK: f32 = 1.0e6;
/// Misfire evaluation window: 100 engine cycles, as specified by OBD-II for the
/// catalyst-damaging misfire monitor (200 revolutions).
const MISFIRE_WINDOW_CYCLES: u32 = 100;
/// Misfire rate above which a cylinder-specific code is stored.
const MISFIRE_RATE_LIMIT: f32 = 0.02;
/// Smoothing of the mean segment-to-segment speed change used to remove engine
/// acceleration/deceleration trends from the misfire metric (per segment).
const MISFIRE_TREND_ALPHA: f32 = 0.15;
/// Idle integrator learns only within this band around the target speed \[rpm\].
const IDLE_INTEGRATION_BAND_RPM: f32 = 150.0;
/// Extra idle-air valve opening during cranking: the manifold must not be pulled into deep
/// vacuum while the engine is being started \[duty\].
const CRANKING_IDLE_AIR: f32 = 0.06;
/// λ reading that marks 80 % of the fuel-cut response step.
const O2_TEST_LAMBDA: f32 = 2.6;
/// Maximum response time to [`O2_TEST_LAMBDA`] for a healthy sensor \[s\].
const O2_TEST_LIMIT_S: f32 = 1.0;
/// Misfire monitor hold-off after throttle transients and fuel cuts \[s\].
const MISFIRE_HOLDOFF_S: f32 = 1.0;
/// Closed-loop fuelling hold-off after a fuel cut ends \[s\].
const POST_CUT_CLOSED_LOOP_HOLD_S: f32 = 2.0;
/// Boost-control integrator learns only within this band around the target \[kPa\].
const BOOST_INTEGRATION_BAND_KPA: f32 = 25.0;
/// Time constant of the idle ↔ main spark schedule hand-over \[s\].
const IDLE_SPARK_BLEND_TAU_S: f32 = 0.3;
/// Authority of the proportional idle-valve term \[duty\].
const IDLE_P_LIMIT: f32 = 0.12;
/// With the driveline engaged, idle speed control runs only below this vehicle speed
/// \[km/h\]; while coasting in gear the wheels, not the idle valve, set engine speed, and
/// an integrator left running there winds the valve shut and stalls the engine when the
/// driver declutches.
const IDLE_MAX_VEHICLE_SPEED_KPH: f32 = 4.0;
/// Extra idle-air ("dashpot") opening held while driving or coasting \[duty\]. When the
/// driver lifts off or declutches, the decaying extra air catches the falling engine
/// speed instead of letting it undershoot into a stall.
const DASHPOT_DUTY: f32 = 0.08;
/// Decay time constant of the dashpot air once idle control takes over \[s\].
const DASHPOT_TAU_S: f32 = 1.5;
/// Look-ahead of the DFCO resume decision \[s\]: injection, wall-film build-up and the first
/// combustion need ≈ 2 engine cycles, so at a fast speed decay the fuel must come back
/// before the resume speed is reached.
const DFCO_RESUME_LEAD_S: f32 = 0.15;
/// Smoothing of the engine acceleration estimate used by the DFCO resume look-ahead \[s\].
const RPM_RATE_TAU_S: f32 = 0.05;
/// Barometric pressure is learned from MAP only after the engine has been stopped this
/// long, so a manifold still in post-stall vacuum is not mistaken for altitude \[s\].
const BARO_LEARN_STOPPED_S: f32 = 2.0;
/// Wide-open-throttle intake loss at low speed, used to update the baro estimate while
/// driving on naturally aspirated engines \[kPa\].
const BARO_WOT_LOSS_KPA: f32 = 1.0;
/// Overboost fuel cut, once triggered, stays latched until the pedal falls below this \[%\].
const OVERBOOST_RELEASE_PEDAL_PCT: f32 = 20.0;
/// MAP reading at or below which the sensor is judged shorted low while running \[kPa\]
/// (the 3-bar transfer function bottoms out at 10 kPa).
const MAP_LOW_LIMIT_KPA: f32 = 10.5;
/// IAT reading below which the circuit is judged open (the pull-up drives the input to
/// the cold rail at −50 °C, outside the −40 °C operating range) \[°C\].
const IAT_OPEN_LIMIT_C: f32 = -45.0;
/// Air mass the engine must have consumed per kelvin of required warm-up before the
/// thermostat monitor (P0128) may judge the coolant too cold \[kg/K\]. A healthy engine
/// needs ≈ 0.05 kg/K at idle (the heat rejected to coolant is roughly proportional to
/// fuel and hence air consumed); twice that is the debounce margin.
const THERMOSTAT_MONITOR_AIR_PER_K: f32 = 0.11;
/// Coolant temperature a healthy thermostat must reach for the P0128 monitor \[°C\].
const THERMOSTAT_MONITOR_C: f32 = 70.0;
/// Time constant of the ECU's catalyst-temperature model (substrate heat-up) \[s\].
const CATALYST_MODEL_TAU_S: f32 = 60.0;
/// Modelled catalyst temperature above which the P0420 efficiency monitor runs \[°C\].
const CATALYST_MONITOR_MIN_C: f32 = 400.0;

/// ECU operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EcuMode {
    /// Ignition off: ECU powered down.
    Off,
    /// Ignition on, engine not turning.
    Stopped,
    /// Starter engaged, synchronising and cranking fuel.
    Cranking,
    /// Normal running.
    Running,
}

/// Actuator commands produced every step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ActuatorCommands {
    pub throttle: f32,
    pub idle_valve: f32,
    pub injector_pw_s: [f32; MAX_CYLINDERS],
    pub spark_btdc_deg: [f32; MAX_CYLINDERS],
    pub spark_enabled: [bool; MAX_CYLINDERS],
    pub fuel_pump: bool,
    pub fan: bool,
    pub wastegate_duty: f32,
}

impl ActuatorCommands {
    pub(crate) fn off() -> Self {
        Self {
            throttle: 0.0,
            idle_valve: 0.0,
            injector_pw_s: [0.0; MAX_CYLINDERS],
            spark_btdc_deg: [0.0; MAX_CYLINDERS],
            spark_enabled: [false; MAX_CYLINDERS],
            fuel_pump: false,
            fan: false,
            wastegate_duty: 0.0,
        }
    }
}

/// ECU internal state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Ecu {
    pub mode: EcuMode,
    key_on_time: f32,
    crank_time: f32,
    pub run_time: f32,
    pub stft: f32,
    stft_int: f32,
    pub ltft: f32,
    pub closed_loop: bool,
    idle_int: f32,
    idle_spark: f32,
    idle_blend: f32,
    pub idle_active: bool,
    dashpot: f32,
    rpm_prev: f32,
    rpm_rate: f32,
    stopped_time: f32,
    air_since_start_kg: f32,
    ect_at_start_c: f32,
    catalyst_model_c: f32,
    pub knock_retard: [f32; MAX_CYLINDERS],
    pub knock_count: [u32; MAX_CYLINDERS],
    film_est: [f32; MAX_CYLINDERS],
    last_inj_kg: [f32; MAX_CYLINDERS],
    pub rev_cut: bool,
    pub dfco: bool,
    dfco_timer: f32,
    post_cut_hold: f32,
    boost_int: f32,
    boost_prev_err: f32,
    pub boost_target_kpa: f32,
    pub overboost_cut: bool,
    fan_on: bool,
    last_segment_seq: u32,
    prev_segment_omega: f32,
    segment_trend: f32,
    misfire_holdoff: f32,
    last_pedal: f32,
    misfire_window: [u32; MAX_CYLINDERS],
    pub misfire_total: [u32; MAX_CYLINDERS],
    window_segments: u32,
    pub dtcs: DtcSet,
    dtc_timers: [f32; DtcCode::COUNT],
    o2_test_timer: f32,
    o2_test_active: bool,
    baro_kpa: f32,
    pub lambda_target: f32,
    pub ve_value: f32,
    pub spark_mean_deg: f32,
    pub pulse_width_s: f32,
    pub injector_duty: f32,
    pub air_per_cylinder_kg: f32,
}

impl Default for Ecu {
    fn default() -> Self {
        Self {
            mode: EcuMode::Off,
            key_on_time: 0.0,
            crank_time: 0.0,
            run_time: 0.0,
            stft: 0.0,
            stft_int: 0.0,
            ltft: 0.0,
            closed_loop: false,
            idle_int: 0.0,
            idle_spark: 0.0,
            idle_blend: 0.0,
            idle_active: false,
            dashpot: 0.0,
            rpm_prev: 0.0,
            rpm_rate: 0.0,
            stopped_time: 0.0,
            air_since_start_kg: 0.0,
            ect_at_start_c: 0.0,
            catalyst_model_c: 0.0,
            knock_retard: [0.0; MAX_CYLINDERS],
            knock_count: [0; MAX_CYLINDERS],
            film_est: [0.0; MAX_CYLINDERS],
            last_inj_kg: [0.0; MAX_CYLINDERS],
            rev_cut: false,
            dfco: false,
            dfco_timer: 0.0,
            post_cut_hold: 0.0,
            boost_int: 0.0,
            boost_prev_err: 0.0,
            boost_target_kpa: 0.0,
            overboost_cut: false,
            fan_on: false,
            last_segment_seq: 0,
            prev_segment_omega: 0.0,
            segment_trend: 0.0,
            misfire_holdoff: 0.0,
            last_pedal: 0.0,
            misfire_window: [0; MAX_CYLINDERS],
            misfire_total: [0; MAX_CYLINDERS],
            window_segments: 0,
            dtcs: DtcSet::default(),
            dtc_timers: [0.0; DtcCode::COUNT],
            o2_test_timer: 0.0,
            o2_test_active: false,
            baro_kpa: 101.3,
            lambda_target: 1.0,
            ve_value: 0.0,
            spark_mean_deg: 0.0,
            pulse_width_s: 0.0,
            injector_duty: 0.0,
            air_per_cylinder_kg: 0.0,
        }
    }
}

impl Ecu {
    /// Clears adaptive values and runtime counters on key-off (trims and DTCs persist,
    /// as they do in the keep-alive memory of a real ECU).
    fn power_down(&mut self) {
        self.mode = EcuMode::Off;
        self.key_on_time = 0.0;
        self.crank_time = 0.0;
        self.run_time = 0.0;
        self.stft = 0.0;
        self.stft_int = 0.0;
        self.closed_loop = false;
        self.idle_int = 0.0;
        self.idle_spark = 0.0;
        self.idle_blend = 0.0;
        self.idle_active = false;
        self.dashpot = 0.0;
        self.rpm_prev = 0.0;
        self.rpm_rate = 0.0;
        self.stopped_time = 0.0;
        self.air_since_start_kg = 0.0;
        self.knock_retard = [0.0; MAX_CYLINDERS];
        self.film_est = [0.0; MAX_CYLINDERS];
        self.last_inj_kg = [0.0; MAX_CYLINDERS];
        self.rev_cut = false;
        self.dfco = false;
        self.dfco_timer = 0.0;
        self.post_cut_hold = 0.0;
        self.boost_int = 0.0;
        self.overboost_cut = false;
        self.fan_on = false;
        self.reset_monitor_state();
        self.dtc_timers = [0.0; DtcCode::COUNT];
        self.clear_running_outputs();
    }

    /// Zeroes the reported values that only exist while the engine runs (closed-loop state,
    /// spark, VE, boost target, fuelling), so a stopped engine never shows stale live data.
    fn clear_running_outputs(&mut self) {
        self.closed_loop = false;
        self.stft = 0.0;
        self.stft_int = 0.0;
        self.knock_retard = [0.0; MAX_CYLINDERS];
        self.lambda_target = 1.0;
        self.ve_value = 0.0;
        self.spark_mean_deg = 0.0;
        self.boost_target_kpa = 0.0;
        self.air_per_cylinder_kg = 0.0;
        self.pulse_width_s = 0.0;
        self.injector_duty = 0.0;
    }

    /// Restarts the OBD monitors' evaluation state (misfire window and trend, O2 test).
    fn reset_monitor_state(&mut self) {
        self.misfire_window = [0; MAX_CYLINDERS];
        self.window_segments = 0;
        self.segment_trend = 0.0;
        self.prev_segment_omega = 0.0;
        self.o2_test_active = false;
        self.o2_test_timer = 0.0;
    }

    /// Clears stored trouble codes, monitor progress and learned trims (scan-tool
    /// "clear codes"): every monitor restarts from scratch, as after a real code clear.
    pub(crate) fn clear_codes(&mut self) {
        self.dtcs.clear();
        self.dtc_timers = [0.0; DtcCode::COUNT];
        self.reset_fuel_trims();
        self.misfire_total = [0; MAX_CYLINDERS];
        self.knock_count = [0; MAX_CYLINDERS];
        self.reset_monitor_state();
    }

    /// Forgets the short- and long-term fuel trims (adaptive fuel memory).
    pub(crate) fn reset_fuel_trims(&mut self) {
        self.ltft = 0.0;
        self.stft = 0.0;
        self.stft_int = 0.0;
    }

    fn store(&mut self, code: DtcCode, now: f64, log: &mut EventLog) {
        if self.dtcs.insert(code) {
            log.push(now, EventKind::Dtc(code));
        }
    }

    /// Debounced OBD monitor: stores `code` once `condition` has held for `hold_s`.
    fn monitor(
        &mut self,
        code: DtcCode,
        condition: bool,
        hold_s: f32,
        dt: f32,
        now: f64,
        log: &mut EventLog,
    ) {
        let t = &mut self.dtc_timers[code.index()];
        if condition {
            *t += dt;
            if *t >= hold_s {
                self.store(code, now, log);
            }
        } else {
            *t = 0.0;
        }
    }

    /// Runs one control step.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn update(
        &mut self,
        s: &SensorReadings,
        ctl: &Controls,
        cal: &Calibration,
        fired: &[bool; MAX_CYLINDERS],
        dt: f32,
        now: f64,
        log: &mut EventLog,
    ) -> ActuatorCommands {
        let mut cmd = ActuatorCommands::off();
        if !ctl.ignition_on {
            if self.mode != EcuMode::Off {
                self.power_down();
            }
            return cmd;
        }
        let n = cal.engine.cylinders.clamp(1, MAX_CYLINDERS);
        self.key_on_time += dt;
        let rpm = s.rpm;
        if rpm < 50.0 {
            // Engine stopped: once the manifold has refilled through the throttle and idle
            // valve, the MAP sensor reads barometric pressure.
            self.stopped_time += dt;
            if self.stopped_time > BARO_LEARN_STOPPED_S {
                self.baro_kpa = approach(self.baro_kpa, s.map_kpa, dt, 0.2);
            }
        } else {
            self.stopped_time = 0.0;
        }
        // Engine acceleration estimate [rpm/s].
        if self.rpm_prev > 0.0 {
            self.rpm_rate = approach(
                self.rpm_rate,
                (rpm - self.rpm_prev) / dt.max(1.0e-6),
                dt,
                RPM_RATE_TAU_S,
            );
        }
        self.rpm_prev = rpm;

        self.mode = match self.mode {
            EcuMode::Off => EcuMode::Stopped,
            EcuMode::Stopped => {
                if rpm > 450.0 {
                    EcuMode::Running
                } else if ctl.starter && rpm > 30.0 {
                    EcuMode::Cranking
                } else {
                    EcuMode::Stopped
                }
            }
            EcuMode::Cranking => {
                if rpm > 500.0 {
                    EcuMode::Running
                } else if !ctl.starter && rpm < 50.0 {
                    EcuMode::Stopped
                } else {
                    EcuMode::Cranking
                }
            }
            EcuMode::Running => {
                if rpm < 250.0 {
                    if ctl.starter {
                        EcuMode::Cranking
                    } else {
                        EcuMode::Stopped
                    }
                } else {
                    EcuMode::Running
                }
            }
        };
        if self.mode == EcuMode::Running {
            if self.run_time == 0.0 {
                self.air_since_start_kg = 0.0;
                self.ect_at_start_c = s.ect_c;
                self.catalyst_model_c = s.ect_c;
            }
            self.run_time += dt;
        } else {
            self.run_time = 0.0;
        }
        if self.mode == EcuMode::Cranking {
            self.crank_time += dt;
        } else {
            self.crank_time = 0.0;
        }

        let ect = s.ect_c;
        cmd.fuel_pump =
            self.key_on_time < 2.0 || matches!(self.mode, EcuMode::Cranking | EcuMode::Running);
        if ect > cal.fan.on_c {
            self.fan_on = true;
        } else if ect < cal.fan.off_c {
            self.fan_on = false;
        }
        cmd.fan = self.fan_on;
        cmd.throttle = clampf(cal.throttle_map.lookup(s.pedal_pct) / 100.0, 0.0, 1.0);

        // Transient-fuel estimator: account for the fuel the previous step injected into
        // cylinders whose intake event just occurred.
        let x_e = clampf(cal.wall_film_fraction.lookup(ect), 0.0, 0.95);
        let tau_e = cal.wall_film_tau_s.lookup(ect).max(0.01);
        let cycle_time = 120.0 / rpm.max(30.0);
        let evap_frac = 1.0 - (-cycle_time / tau_e).exp();
        for ((film, &inj), _) in self
            .film_est
            .iter_mut()
            .zip(self.last_inj_kg.iter())
            .zip(fired.iter())
            .take(n)
            .filter(|(_, &f)| f)
        {
            *film += x_e * inj;
            *film -= *film * evap_frac;
        }

        let base_idle_valve = clampf(cal.idle_valve_base.lookup(ect), 0.0, 1.0);
        self.monitor(
            DtcCode::P0113,
            s.iat_c < IAT_OPEN_LIMIT_C,
            2.0,
            dt,
            now,
            log,
        );
        self.monitor(DtcCode::P0217, ect > 118.0, 2.0, dt, now, log);

        match self.mode {
            EcuMode::Off | EcuMode::Stopped => {
                // Pre-position the idle valve for the next start.
                cmd.idle_valve = clampf(base_idle_valve + CRANKING_IDLE_AIR, 0.0, 1.0);
                self.clear_running_outputs();
                return cmd;
            }
            EcuMode::Cranking => {
                // Cranking fuel decays with cranking time so a long crank does not flood the
                // engine; pedal held to the floor is the classic "flood clear" (no fuel).
                let decay = 0.4 + 0.6 * (-self.crank_time / cal.cranking_decay_s.max(0.1)).exp();
                let flood_clear = s.pedal_pct > 90.0;
                let pw = if flood_clear {
                    0.0
                } else {
                    cal.cranking_pulse_ms.lookup(ect).max(0.0) * 1.0e-3 * decay
                };
                let dead = cal.injector.dead_time_ms.lookup(s.battery_v) * 1.0e-3;
                let flow = cal.injector.flow_g_s * 1.0e-3;
                for c in 0..n {
                    cmd.injector_pw_s[c] = pw;
                    cmd.spark_btdc_deg[c] = cal.cranking_spark_deg;
                    cmd.spark_enabled[c] = true;
                    self.last_inj_kg[c] = (pw - dead).max(0.0) * flow;
                }
                cmd.idle_valve = clampf(base_idle_valve + CRANKING_IDLE_AIR, 0.0, 1.0);
                self.clear_running_outputs();
                self.pulse_width_s = pw;
                self.injector_duty = pw / cycle_time;
                self.spark_mean_deg = cal.cranking_spark_deg;
                return cmd;
            }
            EcuMode::Running => {}
        }

        // ---- Speed-density air estimate --------------------------------------------
        let map_kpa = s.map_kpa.max(5.0);
        let map_pa = map_kpa * 1000.0;
        let iat_k = s.iat_c + ZERO_CELSIUS_K;
        let ect_k = ect + ZERO_CELSIUS_K;
        let blend = clampf(cal.charge_temp_blend.lookup(rpm), 0.0, 1.0);
        let t_charge = (iat_k + blend * (ect_k - iat_k)).max(200.0);
        let ve = clampf(cal.ve.lookup(rpm, map_kpa), 0.05, 2.0);
        let v_cyl = cal.engine.displacement_cc * 1.0e-6 / n as f32;
        let air = ve * map_pa * v_cyl / (R_AIR * t_charge);
        self.ve_value = ve;
        self.air_per_cylinder_kg = air;

        // ---- λ target, enrichments and closed loop ------------------------------------
        let lambda_target = clampf(cal.lambda_target.lookup(rpm, map_kpa), 0.5, 1.6);
        self.lambda_target = lambda_target;
        let warmup = cal.warmup_enrichment.lookup(ect).max(0.5);
        let after_start = cal.after_start_enrichment.lookup(ect).max(0.0)
            * (-self.run_time / cal.after_start_decay_s.max(0.1)).exp();
        let cl = &cal.closed_loop;
        let pedal = s.pedal_pct;
        // After any fuel cut the O2 sensor keeps reading the lean cut gas for its transport
        // delay plus response time; closed loop stays frozen meanwhile.
        if self.dfco || self.rev_cut || self.overboost_cut {
            self.post_cut_hold = POST_CUT_CLOSED_LOOP_HOLD_S;
        } else {
            self.post_cut_hold = (self.post_cut_hold - dt).max(0.0);
        }
        self.closed_loop = cl.enabled
            && self.post_cut_hold <= 0.0
            && ect >= cl.min_coolant_c
            && s.lambda_ready
            && (lambda_target - 1.0).abs() < cl.lambda_window
            && self.run_time > 15.0
            && !self.dfco
            && !self.rev_cut
            && !self.overboost_cut
            && warmup < 1.02
            && pedal < 90.0;
        if self.closed_loop {
            // PI control on λ error; positive error (lean) adds fuel.
            let e = s.lambda - lambda_target;
            self.stft_int = clampf(
                self.stft_int + cl.ki * e * dt,
                -cl.trim_limit,
                cl.trim_limit,
            );
            self.stft = clampf(cl.kp * e + self.stft_int, -cl.trim_limit, cl.trim_limit);
            // Long-term learning slowly migrates the short-term correction into the
            // adaptive table so the integrator returns to zero.
            // finite_or: a non-finite term must never reach the keep-alive trim memory.
            let learn = finite_or(cl.ltft_rate * self.stft * dt, 0.0);
            self.ltft = clampf(self.ltft + learn, -cl.ltft_limit, cl.ltft_limit);
            self.stft_int -= learn;
        } else {
            self.stft_int = approach(self.stft_int, 0.0, dt, 1.0);
            self.stft = self.stft_int;
        }
        let fuel_target = air / (cal.engine.stoich_afr * lambda_target)
            * warmup
            * (1.0 + after_start)
            * (1.0 + self.stft + self.ltft);

        // ---- Fuel cuts ------------------------------------------------------------------
        let lim = &cal.limiter;
        if rpm > lim.cut_rpm {
            self.rev_cut = true;
        } else if rpm < lim.cut_rpm - lim.hysteresis_rpm {
            self.rev_cut = false;
        }
        let d = &cal.dfco;
        let dfco_was = self.dfco;
        if d.enabled && pedal < 0.5 && rpm > d.min_rpm && ect > d.min_coolant_c {
            self.dfco_timer += dt;
            if self.dfco_timer > d.delay_s {
                self.dfco = true;
            }
        } else {
            self.dfco_timer = 0.0;
        }
        // Resume early when the engine speed is falling fast (e.g. declutching).
        let resume_rpm = d.resume_rpm + (-self.rpm_rate).max(0.0) * DFCO_RESUME_LEAD_S;
        if self.dfco && (pedal >= 0.5 || rpm < resume_rpm) {
            self.dfco = false;
            self.dfco_timer = 0.0;
        }
        let turbo = cal.engine.turbocharged;
        let ob = cal.boost.overboost_limit_kpa;
        // Overboost protection: cut fuel and keep it cut until the driver lifts off, so the
        // engine does not cycle in and out of the cut while the wastegate fault persists.
        if turbo && map_kpa > ob {
            if !self.overboost_cut {
                self.store(DtcCode::P0234, now, log);
            }
            self.overboost_cut = true;
        } else if self.overboost_cut && pedal < OVERBOOST_RELEASE_PEDAL_PCT && map_kpa < ob - 10.0 {
            self.overboost_cut = false;
        }
        let cut = self.rev_cut || self.dfco || self.overboost_cut;

        // ---- Injection ------------------------------------------------------------------
        let dead = cal.injector.dead_time_ms.lookup(s.battery_v).max(0.0) * 1.0e-3;
        let flow = (cal.injector.flow_g_s * 1.0e-3).max(1.0e-6);
        let max_pw = clampf(cal.injector.max_duty, 0.1, 1.0) * cycle_time;
        let mut pw_sum = 0.0;
        for c in 0..n {
            // Inverse X–τ. Per intake event the cylinder receives the direct spray plus the
            // evaporation of the film *including* this event's wall deposit:
            //   m_cyl = (1 − X)·m_inj + (F + X·m_inj)·e,  e = 1 − exp(−t_cycle/τ)
            // Solving for m_inj: m_inj = (m_cyl − F·e) / (1 − X·(1 − e)).
            let m_inj = ((fuel_target - self.film_est[c] * evap_frac)
                / (1.0 - x_e * (1.0 - evap_frac)))
                .max(0.0);
            let pw = if cut {
                0.0
            } else {
                (m_inj / flow + dead).min(max_pw)
            };
            cmd.injector_pw_s[c] = pw;
            self.last_inj_kg[c] = (pw - dead).max(0.0) * flow;
            pw_sum += pw;
        }
        self.pulse_width_s = pw_sum / n as f32;
        self.injector_duty = self.pulse_width_s / cycle_time;

        // ---- Idle control ---------------------------------------------------------------
        let idle_target = cal.idle_target_rpm.lookup(ect);
        let ic = &cal.idle;
        // Clutch and neutral switches tell the ECU whether the wheels hold the engine speed.
        let driveline_engaged = !s.neutral && !s.clutch_pedal_pressed;
        let free_engine = !driveline_engaged || s.vehicle_speed_kph < IDLE_MAX_VEHICLE_SPEED_KPH;
        self.idle_active =
            pedal < 1.0 && rpm < idle_target + ic.window_rpm && !self.dfco && free_engine;
        // Dashpot: full extra air while driving or coasting, decaying once idle control is
        // in charge; with the integrator frozen outside idle this restores a safe opening.
        if self.idle_active {
            self.dashpot = approach(self.dashpot, 0.0, dt, DASHPOT_TAU_S);
        } else {
            self.dashpot = DASHPOT_DUTY;
        }
        let idle_err = idle_target - rpm;
        let mut idle_valve = base_idle_valve + self.idle_int + self.dashpot;
        if self.idle_active {
            // Conditional integration (anti-windup): full integral gain close to the target,
            // 25 % outside it, so flare-ups and dips (handled by the bounded P term and
            // spark) cannot wind the integrator up, yet large offsets still converge.
            let gain = if idle_err.abs() < IDLE_INTEGRATION_BAND_RPM {
                1.0
            } else {
                0.25
            };
            self.idle_int = clampf(self.idle_int + gain * ic.ki * idle_err * dt, -0.3, 0.5);
            self.idle_spark = clampf(
                ic.spark_gain * idle_err,
                -ic.spark_limit_deg,
                ic.spark_limit_deg,
            );
            idle_valve += clampf(ic.kp * idle_err, -IDLE_P_LIMIT, IDLE_P_LIMIT);
        } else {
            self.idle_spark = approach(self.idle_spark, 0.0, dt, 0.3);
        }
        // Smooth hand-over between the idle spark schedule and the main map.
        self.idle_blend = approach(
            self.idle_blend,
            if self.idle_active { 1.0 } else { 0.0 },
            dt,
            IDLE_SPARK_BLEND_TAU_S,
        );
        cmd.idle_valve = clampf(idle_valve, 0.0, 1.0);

        // ---- Knock control --------------------------------------------------------------
        let kc = &cal.knock;
        let threshold = kc.threshold.lookup(rpm);
        for (c, &fired_now) in fired.iter().enumerate().take(n) {
            if fired_now && kc.enabled && s.knock_signal[c] > threshold {
                self.knock_retard[c] =
                    (self.knock_retard[c] + kc.retard_step_deg).min(kc.max_retard_deg);
                self.knock_count[c] = self.knock_count[c].saturating_add(1);
            }
            self.knock_retard[c] = (self.knock_retard[c] - kc.recovery_deg_per_s * dt).max(0.0);
        }

        // ---- Spark ----------------------------------------------------------------------
        let main_spark = cal.ignition.lookup(rpm, map_kpa);
        let idle_spark = ic.base_spark_deg + self.idle_spark;
        // Each spark term is guarded separately so one bad term cannot poison the sum.
        let base_spark = finite_or(
            main_spark + (idle_spark - main_spark) * self.idle_blend,
            main_spark,
        );
        let corrections = finite_or(
            cal.iat_spark_correction.lookup(s.iat_c) + cal.coolant_spark_correction.lookup(ect),
            0.0,
        );
        let soft = finite_or(
            lim.soft_retard_deg * smoothstep(lim.cut_rpm - lim.soft_window_rpm, lim.cut_rpm, rpm),
            0.0,
        );
        let mut spark_sum = 0.0;
        for c in 0..n {
            let sp = clampf(
                base_spark + corrections - self.knock_retard[c] - soft,
                -15.0,
                55.0,
            );
            cmd.spark_btdc_deg[c] = sp;
            cmd.spark_enabled[c] = true;
            spark_sum += sp;
        }
        self.spark_mean_deg = spark_sum / n as f32;

        // ---- Boost control --------------------------------------------------------------
        if turbo && cal.boost.enabled {
            let b = &cal.boost;
            let target = b.target_kpa.lookup(rpm, pedal);
            self.boost_target_kpa = target;
            let err = target - map_kpa;
            if pedal < 20.0 {
                // Light load: wastegate on spring pressure, integrator parked.
                self.boost_int = approach(self.boost_int, 0.0, dt, 0.5);
                cmd.wastegate_duty = 0.0;
            } else {
                let deriv = (err - self.boost_prev_err) / dt.max(1.0e-6);
                let unclamped =
                    b.base_duty.lookup(rpm) + b.kp * err + self.boost_int + b.kd * deriv;
                // Anti-windup: integrate only near the target and never further into an
                // actuator limit; otherwise the integrator charged during turbo lag dumps
                // into an overboost spike once the turbine spools.
                let saturated = (unclamped >= 1.0 && err > 0.0) || (unclamped <= 0.0 && err < 0.0);
                if err.abs() < BOOST_INTEGRATION_BAND_KPA && !saturated {
                    self.boost_int = clampf(self.boost_int + b.ki * err * dt, -0.5, 0.5);
                }
                cmd.wastegate_duty = clampf(unclamped, 0.0, 1.0);
            }
            self.boost_prev_err = err;
        } else {
            self.boost_target_kpa = 0.0;
        }

        // ---- OBD monitors ---------------------------------------------------------------
        // Misfire monitor enable conditions (as in production OBD): no throttle transient,
        // no fuel cut, and no closed-throttle deceleration above idle, where dilution-driven
        // partial burns and torque reversals would be misread as misfires.
        // A slipping or just-engaged clutch jerks the crank like a misfire does.
        let transient = (pedal - self.last_pedal).abs() > 0.5
            || cut
            || s.clutch_pedal_pressed
            || (pedal < 1.0 && rpm > idle_target + 300.0);
        self.last_pedal = pedal;
        self.misfire_holdoff = if transient {
            MISFIRE_HOLDOFF_S
        } else {
            (self.misfire_holdoff - dt).max(0.0)
        };
        self.misfire_monitor(s, cal, n, air, now, log);
        let total_trim = self.stft + self.ltft;
        let cl_on = self.closed_loop;
        self.monitor(
            DtcCode::P0171,
            cl_on && total_trim > 0.22,
            5.0,
            dt,
            now,
            log,
        );
        self.monitor(
            DtcCode::P0172,
            cl_on && total_trim < -0.22,
            5.0,
            dt,
            now,
            log,
        );
        // Thermostat monitor: compares the coolant temperature with the warm-up a healthy
        // engine must have achieved for the air (≈ fuel heat) consumed since the start.
        self.air_since_start_kg += air * rpm / 120.0 * n as f32 * dt;
        let warmup_needed_k = (THERMOSTAT_MONITOR_C - self.ect_at_start_c).max(0.0);
        let warmup_due =
            self.air_since_start_kg > THERMOSTAT_MONITOR_AIR_PER_K * warmup_needed_k + 0.5;
        self.monitor(
            DtcCode::P0128,
            warmup_due && ect < THERMOSTAT_MONITOR_C,
            5.0,
            dt,
            now,
            log,
        );
        self.monitor(DtcCode::P0219, rpm > lim.cut_rpm + 500.0, 0.3, dt, now, log);
        let underboost = turbo
            && cal.boost.enabled
            && pedal > 80.0
            && rpm > 3000.0
            && self.boost_target_kpa - map_kpa > 30.0;
        self.monitor(DtcCode::P0299, underboost, 4.0, dt, now, log);
        let rail_low = s.fuel_pressure_kpa < 0.75 * cal.injector.rated_pressure_kpa;
        self.monitor(DtcCode::P0087, rail_low, 2.0, dt, now, log);
        // Baro tracking at wide-open throttle and low speed, where MAP ≈ baro − intake loss.
        if !turbo && s.throttle_pct > 80.0 && rpm < 2500.0 {
            self.baro_kpa = approach(self.baro_kpa, map_kpa + BARO_WOT_LOSS_KPA, dt, 2.0);
        }
        let map_implausible =
            (!turbo && map_kpa > self.baro_kpa + 15.0) || s.map_kpa <= MAP_LOW_LIMIT_KPA;
        let idle_map_high = self.idle_active && rpm > 600.0 && map_kpa > 0.85 * self.baro_kpa;
        self.monitor(
            DtcCode::P0106,
            map_implausible || idle_map_high,
            3.0,
            dt,
            now,
            log,
        );
        let knock_bg = s.knock_signal[..n].iter().sum::<f32>() / n as f32;
        self.monitor(
            DtcCode::P0325,
            rpm > 2500.0 && knock_bg < 0.12 * threshold,
            3.0,
            dt,
            now,
            log,
        );
        // Catalyst monitor runs only once the ECU's substrate-temperature model (a lag of
        // the exhaust temperature) says the catalyst must be lit off.
        self.catalyst_model_c = approach(self.catalyst_model_c, s.egt_c, dt, CATALYST_MODEL_TAU_S);
        self.monitor(
            DtcCode::P0420,
            cl_on
                && self.run_time > 120.0
                && self.catalyst_model_c > CATALYST_MONITOR_MIN_C
                && s.rear_o2_activity > 0.6,
            10.0,
            dt,
            now,
            log,
        );
        // OBD enable conditions for the idle monitor: vehicle stationary, pedal released
        // (DFCO cycling caused by the high idle itself must not reset the debounce).
        let idle_high = s.vehicle_speed_kph < 3.0
            && pedal < 1.0
            && rpm - idle_target > 200.0
            && self.run_time > 20.0;
        self.monitor(DtcCode::P0507, idle_high, 10.0, dt, now, log);
        let oil_low = (rpm > 1500.0 && s.oil_pressure_kpa < 70.0)
            || (rpm > 600.0 && s.oil_pressure_kpa < 35.0);
        self.monitor(DtcCode::P0524, oil_low, 2.0, dt, now, log);
        self.monitor(
            DtcCode::P0016,
            s.cam_phase_error_deg.abs() > 5.0,
            2.0,
            dt,
            now,
            log,
        );

        // O2 response test: fuel cut-off is a clean λ step from ≈ 1 to free air (λ ≈ 3).
        // A healthy wide-band sensor covers 80 % of that step (λ > 2.6) within ≈ 0.3 s
        // including transport delay; an aged sensor takes several times longer.
        if self.dfco && !dfco_was && s.lambda_ready && ect > 60.0 {
            self.o2_test_active = true;
            self.o2_test_timer = 0.0;
        }
        if self.o2_test_active {
            self.o2_test_timer += dt;
            let responded = s.lambda > O2_TEST_LAMBDA;
            if responded || !self.dfco {
                self.o2_test_active = false;
                // Without a response the verdict is only valid if the cut lasted long
                // enough for a healthy sensor to have responded.
                if self.o2_test_timer > O2_TEST_LIMIT_S {
                    self.store(DtcCode::P0133, now, log);
                }
            }
        }
        cmd
    }

    /// Crankshaft-speed misfire monitor (OBD-II). Between two firing TDCs the crank gains
    /// the net work of the cylinder whose power stroke lies between them; a misfire leaves
    /// a deficit ΔE ≈ W_cyl, i.e. a speed change of D ≈ W_cyl/(J·ω) relative to the
    /// running mean change (which removes ordinary acceleration and deceleration).
    fn misfire_monitor(
        &mut self,
        s: &SensorReadings,
        cal: &Calibration,
        n: usize,
        air: f32,
        now: f64,
        log: &mut EventLog,
    ) {
        if s.segment_seq == self.last_segment_seq {
            return;
        }
        self.last_segment_seq = s.segment_seq;
        let omega = s.segment_omega;
        let prev = self.prev_segment_omega;
        self.prev_segment_omega = omega;
        if prev <= 0.0 {
            return;
        }
        let delta = omega - prev;
        let deviation = delta - self.segment_trend;
        self.segment_trend += (delta - self.segment_trend) * MISFIRE_TREND_ALPHA;
        let mm = &cal.misfire;
        let valid = mm.enabled
            && self.misfire_holdoff <= 0.0
            && self.run_time > 3.0
            && (500.0..6500.0).contains(&s.rpm)
            && !self.dfco
            && !self.rev_cut
            && !self.overboost_cut;
        if !valid {
            return;
        }
        let cyl = usize::from(s.segment_cylinder).min(n - 1);
        let omega_avg = 0.5 * (omega + prev);
        let expected_drop =
            air * SPECIFIC_INDICATED_WORK / (mm.inertia_kg_m2.max(0.01) * omega_avg.max(10.0));
        if deviation < -mm.threshold_fraction * expected_drop {
            self.misfire_window[cyl] += 1;
            self.misfire_total[cyl] = self.misfire_total[cyl].saturating_add(1);
        }
        self.window_segments += 1;
        if self.window_segments >= MISFIRE_WINDOW_CYCLES * n as u32 {
            let mut flagged = 0;
            for c in 0..n {
                let rate = self.misfire_window[c] as f32 / MISFIRE_WINDOW_CYCLES as f32;
                if rate > MISFIRE_RATE_LIMIT {
                    flagged += 1;
                    if let Some(code) = DtcCode::misfire_for_cylinder(c) {
                        self.store(code, now, log);
                    }
                }
            }
            if flagged >= 2 {
                self.store(DtcCode::P0300, now, log);
            }
            self.misfire_window = [0; MAX_CYLINDERS];
            self.window_segments = 0;
        }
    }
}

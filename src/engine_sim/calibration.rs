//! ECU calibration: every map and scalar a tuner can edit.
//!
//! [`Calibration::base_for`] generates a plausible *factory base map* from an engine
//! definition. It is deliberately imperfect (generic VE table with a few percent error,
//! conservative spark), giving the student real work to do on the dyno.

use core::fmt;

use super::breathing::{self, BreathingInputs};
use super::combustion::{CombustionModel, CycleInputs};
use super::math::{clampf, lerp, smoothstep};
use super::spec::{EngineSpec, MAX_CYLINDERS};
use super::thermo::{R_AIR, STANDARD_PRESSURE_PA, ZERO_CELSIUS_K};

/// Number of engine-speed breakpoints of the main maps.
pub const RPM_POINTS: usize = 16;
/// Number of load (MAP) breakpoints of the main maps.
pub const LOAD_POINTS: usize = 16;
/// Number of pedal breakpoints of the boost-target map.
pub const PEDAL_POINTS: usize = 8;
/// Number of breakpoints of the coolant-temperature curves.
pub const COOLANT_POINTS: usize = 8;

/// Errors from [`Calibration::validate`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationError {
    /// A table axis is not strictly increasing or contains non-finite values.
    InvalidAxis(&'static str),
    /// A table value or scalar is not finite or outside its range.
    InvalidValue(&'static str),
}

impl fmt::Display for CalibrationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidAxis(n) => {
                write!(f, "axis of `{n}` must be finite and strictly increasing")
            }
            Self::InvalidValue(n) => write!(f, "value `{n}` is invalid"),
        }
    }
}

impl std::error::Error for CalibrationError {}

/// Locates `x` on a strictly increasing axis: returns the lower breakpoint index and the
/// interpolation fraction. Values outside the axis are clamped (flat extrapolation, which
/// is how production ECUs treat map edges). NaN maps to the first cell.
#[inline]
fn locate<const N: usize>(axis: &[f32; N], x: f32) -> (usize, f32) {
    if N < 2 || x.is_nan() || x <= axis[0] {
        return (0, 0.0);
    }
    if x >= axis[N - 1] {
        return (N - 2, 1.0);
    }
    let mut i = 0;
    while i < N - 2 && x >= axis[i + 1] {
        i += 1;
    }
    let span = axis[i + 1] - axis[i];
    let t = if span > 0.0 {
        (x - axis[i]) / span
    } else {
        0.0
    };
    (i, clampf(t, 0.0, 1.0))
}

fn axis_valid<const N: usize>(axis: &[f32; N]) -> bool {
    axis.iter().all(|v| v.is_finite()) && axis.windows(2).all(|w| w[1] > w[0])
}

/// One-dimensional calibration curve with linear interpolation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Table1D<const N: usize> {
    /// Breakpoints, strictly increasing.
    pub axis: [f32; N],
    /// Values at the breakpoints.
    pub values: [f32; N],
}

impl<const N: usize> Table1D<N> {
    /// Creates a curve.
    pub const fn new(axis: [f32; N], values: [f32; N]) -> Self {
        Self { axis, values }
    }

    /// Interpolated value at `x`.
    pub fn lookup(&self, x: f32) -> f32 {
        if N == 1 {
            return self.values[0];
        }
        let (i, t) = locate(&self.axis, x);
        lerp(self.values[i], self.values[i + 1], t)
    }

    fn is_valid(&self) -> bool {
        N >= 1 && axis_valid(&self.axis) && self.values.iter().all(|v| v.is_finite())
    }
}

/// Two-dimensional calibration map with bilinear interpolation. `values\[y\]\[x\]`: one row
/// per load breakpoint, one column per speed breakpoint (the layout of WinOLS / HP Tuners
/// tables).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Table2D<const X: usize, const Y: usize> {
    /// Column breakpoints (usually engine speed), strictly increasing.
    pub x_axis: [f32; X],
    /// Row breakpoints (usually load), strictly increasing.
    pub y_axis: [f32; Y],
    /// Cell values, row-major by `y`.
    pub values: [[f32; X]; Y],
}

impl<const X: usize, const Y: usize> Table2D<X, Y> {
    /// Bilinear interpolation at `(x, y)`.
    pub fn lookup(&self, x: f32, y: f32) -> f32 {
        if X < 2 || Y < 2 {
            return self.values[0][0];
        }
        let (ix, tx) = locate(&self.x_axis, x);
        let (iy, ty) = locate(&self.y_axis, y);
        let a = lerp(self.values[iy][ix], self.values[iy][ix + 1], tx);
        let b = lerp(self.values[iy + 1][ix], self.values[iy + 1][ix + 1], tx);
        lerp(a, b, ty)
    }

    /// Builds a map by evaluating `f(x, y)` at every breakpoint.
    pub fn from_fn(x_axis: [f32; X], y_axis: [f32; Y], mut f: impl FnMut(f32, f32) -> f32) -> Self {
        let mut values = [[0.0; X]; Y];
        for (iy, row) in values.iter_mut().enumerate() {
            for (ix, cell) in row.iter_mut().enumerate() {
                *cell = f(x_axis[ix], y_axis[iy]);
            }
        }
        Self {
            x_axis,
            y_axis,
            values,
        }
    }

    fn is_valid(&self) -> bool {
        axis_valid(&self.x_axis)
            && axis_valid(&self.y_axis)
            && self.values.iter().flatten().all(|v| v.is_finite())
    }
}

/// Speed × MAP map used for VE, spark and λ target.
pub type FuelSparkMap = Table2D<RPM_POINTS, LOAD_POINTS>;
/// Speed × pedal map used for boost target.
pub type BoostMap = Table2D<RPM_POINTS, PEDAL_POINTS>;
/// Curve indexed by coolant temperature \[°C\].
pub type CoolantCurve = Table1D<COOLANT_POINTS>;

/// Engine constants programmed into the ECU.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EngineConstants {
    /// Number of cylinders.
    pub cylinders: usize,
    /// Displacement \[cm³\].
    pub displacement_cc: f32,
    /// Stoichiometric AFR of the fuel the ECU assumes.
    pub stoich_afr: f32,
    /// Engine has a turbocharger and boost control.
    pub turbocharged: bool,
}

/// Injector characterisation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InjectorCalibration {
    /// Static flow \[g/s\].
    pub flow_g_s: f32,
    /// Dead time \[ms\] versus battery voltage \[V\].
    pub dead_time_ms: Table1D<6>,
    /// Maximum allowed duty cycle 0..1.
    pub max_duty: f32,
    /// Fuel rail differential pressure at which `flow_g_s` is rated \[kPa\].
    pub rated_pressure_kpa: f32,
}

/// Idle-speed controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IdleControl {
    /// Idle valve proportional gain \[duty/rpm\].
    pub kp: f32,
    /// Idle valve integral gain [duty/(rpm·s)].
    pub ki: f32,
    /// Idle spark advance [deg BTDC]. Deliberately retarded from MBT so that the fast
    /// spark loop can add *and* remove torque (torque reserve).
    pub base_spark_deg: f32,
    /// Spark-based fast idle correction \[deg/rpm\].
    pub spark_gain: f32,
    /// Limit of the spark idle correction \[deg\].
    pub spark_limit_deg: f32,
    /// Idle control is active below target + this window \[rpm\].
    pub window_rpm: f32,
}

/// Closed-loop λ control (short- and long-term fuel trims).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClosedLoopFuel {
    /// Master enable.
    pub enabled: bool,
    /// Minimum coolant temperature \[°C\].
    pub min_coolant_c: f32,
    /// Proportional gain \[trim/λ\].
    pub kp: f32,
    /// Integral gain [trim/(λ·s)].
    pub ki: f32,
    /// Short-term trim authority (fraction).
    pub trim_limit: f32,
    /// Long-term trim learning rate \[1/s\].
    pub ltft_rate: f32,
    /// Long-term trim authority (fraction).
    pub ltft_limit: f32,
    /// Closed loop runs only while |λ_target − 1| is below this.
    pub lambda_window: f32,
}

/// Knock control.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KnockControl {
    /// Master enable.
    pub enabled: bool,
    /// Knock-detection threshold of the sensor signal versus rpm.
    pub threshold: Table1D<8>,
    /// Retard applied per detected knock event \[deg\].
    pub retard_step_deg: f32,
    /// Maximum knock retard \[deg\].
    pub max_retard_deg: f32,
    /// Recovery rate of knock retard \[deg/s\].
    pub recovery_deg_per_s: f32,
}

/// Rev limiter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RevLimiter {
    /// Fuel cut speed \[rpm\].
    pub cut_rpm: f32,
    /// Re-enable fuel below cut − hysteresis \[rpm\].
    pub hysteresis_rpm: f32,
    /// Start of soft (spark-retard) limiting below the cut \[rpm\].
    pub soft_window_rpm: f32,
    /// Spark retard at the cut speed \[deg\].
    pub soft_retard_deg: f32,
}

/// Deceleration fuel cut-off.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Dfco {
    /// Master enable.
    pub enabled: bool,
    /// Minimum speed for entry \[rpm\].
    pub min_rpm: f32,
    /// Fuel resumes below this speed \[rpm\].
    pub resume_rpm: f32,
    /// Minimum coolant temperature \[°C\].
    pub min_coolant_c: f32,
    /// Closed-pedal time before cut \[s\].
    pub delay_s: f32,
}

/// Radiator fan control.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FanControl {
    /// Switch-on coolant temperature \[°C\].
    pub on_c: f32,
    /// Switch-off coolant temperature \[°C\].
    pub off_c: f32,
}

/// Boost control (turbocharged engines).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoostControl {
    /// Master enable.
    pub enabled: bool,
    /// Target manifold pressure [kPa abs] versus rpm and pedal \[%\].
    pub target_kpa: BoostMap,
    /// Feed-forward wastegate solenoid duty versus rpm.
    pub base_duty: Table1D<RPM_POINTS>,
    /// Proportional gain \[duty/kPa\].
    pub kp: f32,
    /// Integral gain [duty/(kPa·s)].
    pub ki: f32,
    /// Derivative gain \[duty·s/kPa\].
    pub kd: f32,
    /// Fuel cut above this manifold pressure [kPa abs].
    pub overboost_limit_kpa: f32,
}

/// Crankshaft-speed-based misfire monitor (OBD-II).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MisfireMonitor {
    /// Master enable.
    pub enabled: bool,
    /// Detection threshold as a fraction of the expected speed drop of a full misfire.
    pub threshold_fraction: f32,
    /// Crank-train inertia programmed into the ECU \[kg·m²\].
    pub inertia_kg_m2: f32,
}

/// Complete ECU calibration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Calibration {
    /// Engine constants.
    pub engine: EngineConstants,
    /// Volumetric efficiency table (fraction) versus rpm × MAP \[kPa\].
    pub ve: FuelSparkMap,
    /// Base spark advance [deg BTDC] versus rpm × MAP \[kPa\].
    pub ignition: FuelSparkMap,
    /// Target λ versus rpm × MAP \[kPa\].
    pub lambda_target: FuelSparkMap,
    /// Injector data.
    pub injector: InjectorCalibration,
    /// Weight of coolant temperature in the charge-temperature estimate versus rpm.
    pub charge_temp_blend: Table1D<8>,
    /// Warm-up fuel multiplier versus coolant \[°C\].
    pub warmup_enrichment: CoolantCurve,
    /// Cranking pulse width \[ms\] versus coolant \[°C\].
    pub cranking_pulse_ms: CoolantCurve,
    /// Time constant over which cranking fuel decays towards 40 % to avoid flooding \[s\].
    pub cranking_decay_s: f32,
    /// Spark advance during cranking [deg BTDC].
    pub cranking_spark_deg: f32,
    /// Additional fuel right after start (fraction) versus coolant \[°C\].
    pub after_start_enrichment: CoolantCurve,
    /// Decay time constant of the after-start enrichment \[s\].
    pub after_start_decay_s: f32,
    /// Transient-fuel model: wall-wetting fraction X versus coolant \[°C\].
    pub wall_film_fraction: CoolantCurve,
    /// Transient-fuel model: film evaporation time constant τ \[s\] versus coolant \[°C\].
    pub wall_film_tau_s: CoolantCurve,
    /// Idle speed target \[rpm\] versus coolant \[°C\].
    pub idle_target_rpm: CoolantCurve,
    /// Idle valve feed-forward duty versus coolant \[°C\].
    pub idle_valve_base: CoolantCurve,
    /// Idle controller gains.
    pub idle: IdleControl,
    /// Spark correction \[deg\] versus intake air temperature \[°C\].
    pub iat_spark_correction: Table1D<8>,
    /// Spark correction \[deg\] versus coolant \[°C\].
    pub coolant_spark_correction: CoolantCurve,
    /// Closed-loop fuel control.
    pub closed_loop: ClosedLoopFuel,
    /// Knock control.
    pub knock: KnockControl,
    /// Rev limiter.
    pub limiter: RevLimiter,
    /// Deceleration fuel cut-off.
    pub dfco: Dfco,
    /// Drive-by-wire map: pedal \[%\] → throttle \[%\].
    pub throttle_map: Table1D<8>,
    /// Radiator fan.
    pub fan: FanControl,
    /// Boost control.
    pub boost: BoostControl,
    /// Misfire monitor.
    pub misfire: MisfireMonitor,
}

const COOLANT_AXIS: [f32; COOLANT_POINTS] = [-30.0, -10.0, 0.0, 20.0, 40.0, 60.0, 80.0, 100.0];
const RPM_AXIS: [f32; RPM_POINTS] = [
    500.0, 800.0, 1200.0, 1600.0, 2000.0, 2400.0, 2800.0, 3200.0, 3600.0, 4000.0, 4500.0, 5000.0,
    5500.0, 6000.0, 6800.0, 7500.0,
];
const NA_LOAD_AXIS: [f32; LOAD_POINTS] = [
    20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 88.0, 96.0, 105.0,
];
const TURBO_LOAD_AXIS: [f32; LOAD_POINTS] = [
    20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 95.0, 110.0, 125.0, 140.0, 160.0, 180.0, 200.0,
    225.0, 250.0,
];
const PEDAL_AXIS: [f32; PEDAL_POINTS] = [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 100.0];

/// Nominal conditions the base map is generated for: 25 °C intake air, 90 °C coolant.
const NOMINAL_IAT_K: f32 = 298.15;
const NOMINAL_ECT_K: f32 = 363.15;
/// Nominal residual-gas temperature used during base-map generation \[K\].
const NOMINAL_RESIDUAL_K: f32 = 1000.0;

impl Calibration {
    /// Generates the factory base calibration for `spec` (see module docs).
    pub fn base_for(spec: &EngineSpec) -> Self {
        let turbo = spec.turbo.is_some();
        let load_axis = if turbo { TURBO_LOAD_AXIS } else { NA_LOAD_AXIS };
        let v_cyl = spec.geometry.cylinder_displacement_m3();
        let stoich = spec.fuel.stoich_afr;

        // Charge-temperature blend: what fraction of (ECT − IAT) the charge picks up, as
        // the OEM would characterise it on a flow bench at mid load.
        let blend_axis = [
            500.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7500.0,
        ];
        let mut blend_values = [0.0; 8];
        for (v, &rpm) in blend_values.iter_mut().zip(blend_axis.iter()) {
            *v = breathing::charge_heating_fraction(spec, rpm, 60_000.0);
        }
        let charge_temp_blend = Table1D::new(blend_axis, blend_values);

        let lambda_fn = |rpm: f32, map_kpa: f32| -> f32 {
            if turbo {
                // Stoichiometric for the catalyst; λ 0.78 under full boost for knock and
                // turbine-inlet-temperature protection.
                1.0 - 0.22 * smoothstep(85.0, 165.0, map_kpa)
                    - 0.02 * smoothstep(5000.0, 6000.0, rpm)
            } else {
                // Power enrichment towards λ 0.88 (best torque) above 80 kPa.
                1.0 - 0.12 * smoothstep(78.0, 98.0, map_kpa)
                    - 0.02 * smoothstep(5000.0, 6500.0, rpm)
            }
        };
        let lambda_target =
            Table2D::from_fn(RPM_AXIS, load_axis, |r, m| round_to(lambda_fn(r, m), 0.01));

        // Exhaust pressure estimate for base-map generation.
        let exhaust_pressure = |rpm: f32, map_pa: f32| -> f32 {
            let flow =
                map_pa / (R_AIR * NOMINAL_IAT_K) * spec.geometry.displacement_m3() * rpm / 120.0;
            let downstream = STANDARD_PRESSURE_PA + spec.exhaust.backpressure * flow * flow;
            if turbo {
                // Turbine inlet pressure runs ≈ 10–30 % above boost pressure on a small turbo.
                downstream.max(1.15 * map_pa)
            } else {
                downstream
            }
        };

        // Trapped air per cylinder at nominal conditions from the true breathing model.
        let trapped_air = |rpm: f32, map_pa: f32| -> (f32, f32) {
            let t_fresh = breathing::fresh_charge_temperature(
                spec,
                rpm,
                map_pa,
                NOMINAL_IAT_K,
                NOMINAL_ECT_K,
            );
            let ve = breathing::volumetric_efficiency(
                spec,
                &BreathingInputs {
                    rpm,
                    p_man: map_pa,
                    p_exh: exhaust_pressure(rpm, map_pa),
                    t_man: NOMINAL_IAT_K,
                    t_fresh,
                    cam_retard_deg: 0.0,
                    valve_float: 0.0,
                },
            );
            (ve * map_pa / (R_AIR * NOMINAL_IAT_K) * v_cyl, t_fresh)
        };

        // ECU VE: the value that makes the speed-density equation with the ECU's own charge
        // temperature estimate reproduce the true trapped mass, multiplied by a smooth
        // ±4 % error pattern representative of a generic engine-family base map.
        let ve = Table2D::from_fn(RPM_AXIS, load_axis, |rpm, map_kpa| {
            let map_pa = map_kpa * 1000.0;
            let (air, _) = trapped_air(rpm, map_pa);
            let t_est =
                NOMINAL_IAT_K + charge_temp_blend.lookup(rpm) * (NOMINAL_ECT_K - NOMINAL_IAT_K);
            let ve_ecu = air * R_AIR * t_est / (map_pa * v_cyl);
            let error = 0.04 * (0.0011 * rpm + 0.6).sin() * (0.035 * map_kpa).cos();
            round_to(ve_ecu * (1.0 + error), 0.001)
        });

        // Spark: MBT located by sweeping the true cycle model, then limited to the
        // knock-limited spark advance (KLSA) on 95 RON with cycle-variation margin.
        let model = CombustionModel::new(spec);
        let ignition = Table2D::from_fn(RPM_AXIS, load_axis, |rpm, map_kpa| {
            let map_pa = map_kpa * 1000.0;
            let (air, t_fresh) = trapped_air(rpm, map_pa);
            let lambda = lambda_fn(rpm, map_kpa);
            let fuel = air / (stoich * lambda);
            let p_exh = exhaust_pressure(rpm, map_pa);
            let residual =
                p_exh * spec.geometry.clearance_volume_m3() / (R_AIR * NOMINAL_RESIDUAL_K);
            let t_ivc =
                (air * t_fresh + residual * NOMINAL_RESIDUAL_K * 1.15) / (air + residual * 1.15);
            let base_inputs = CycleInputs {
                rpm,
                fresh_air_kg: air,
                fuel_kg: fuel,
                residual_kg: residual,
                t_ivc_k: t_ivc,
                spark_btdc_deg: 0.0,
                spark_enabled: true,
                spark_voltage_kv: 1.0e3,
                breakdown_noise: 0.0,
                octane_ron: 95.0,
                wall_temp_k: NOMINAL_ECT_K + 20.0,
                exhaust_pressure_pa: p_exh,
                burn_ccv: 1.0,
                // e^(−1.5·0.15): the 1.5σ fast-autoignition cycle of the CCV model, so
                // fewer than ≈ 7 % of cycles can reach the knock threshold on the base map.
                knock_ccv: 0.80,
                compression_leak: 0.0,
            };
            let mut mbt = 0.0_f32;
            let mut best_work = f32::MIN;
            let mut klsa = 50.0_f32;
            let mut knock_seen = false;
            let mut s = -5.0_f32;
            while s <= 50.0 {
                let r = model.simulate(&CycleInputs {
                    spark_btdc_deg: s,
                    ..base_inputs
                });
                if r.net_work_j > best_work {
                    best_work = r.net_work_j;
                    mbt = s;
                }
                if !knock_seen && r.knock_intensity > 0.0 {
                    knock_seen = true;
                    // 2° safety margin below the first knocking advance.
                    klsa = s - 2.0;
                }
                s += 1.0;
            }
            let spark = (mbt - 1.0).min(klsa);
            round_to(clampf(spark, -5.0, 45.0), 0.5)
        });

        let injector_flow_g_s = spec.fuel_system.injector_flow_kg_s * 1000.0;
        let dead_axis = [8.0, 10.0, 12.0, 13.0, 14.0, 15.0];
        let mut dead_values = [0.0; 6];
        for (v, &volts) in dead_values.iter_mut().zip(dead_axis.iter()) {
            *v = (spec.fuel_system.injector_dead_time_s
                + spec.fuel_system.injector_dead_time_slope_s_per_v * (14.0 - volts))
                * 1000.0;
        }

        // Transient-fuel calibration measured at 60 kPa on the engine.
        let mut x_values = [0.0; COOLANT_POINTS];
        let mut tau_values = [0.0; COOLANT_POINTS];
        for i in 0..COOLANT_POINTS {
            let t = COOLANT_AXIS[i] + ZERO_CELSIUS_K;
            x_values[i] = round_to(breathing::wall_film_fraction(t, 60_000.0), 0.01);
            tau_values[i] = round_to(breathing::wall_film_tau(t), 0.01);
        }

        let boost_target = Table2D::from_fn(RPM_AXIS, PEDAL_AXIS, |rpm, pedal| {
            if !turbo {
                return 100.0;
            }
            // 0.8 bar gauge plateau from 3000 rpm — the knock-safe level for 9.5:1 port
            // injection on 95 RON — tapering above 5500 rpm to protect the turbine;
            // proportional to pedal so part throttle does not over-boost.
            let plateau = 80.0
                * smoothstep(1800.0, 3000.0, rpm)
                * (1.0 - 0.2 * smoothstep(5500.0, 6500.0, rpm));
            round_to(101.3 + plateau * smoothstep(20.0, 85.0, pedal), 1.0)
        });
        let mut base_duty = [0.0; RPM_POINTS];
        for (d, &rpm) in base_duty.iter_mut().zip(RPM_AXIS.iter()) {
            *d = if turbo {
                round_to(0.25 + 0.25 * smoothstep(2000.0, 5000.0, rpm), 0.01)
            } else {
                0.0
            };
        }

        let redline = spec.limits.redline_rpm;
        Self {
            engine: EngineConstants {
                cylinders: spec.geometry.cylinders,
                displacement_cc: spec.geometry.displacement_m3() * 1.0e6,
                stoich_afr: stoich,
                turbocharged: turbo,
            },
            ve,
            ignition,
            lambda_target,
            injector: InjectorCalibration {
                flow_g_s: injector_flow_g_s,
                dead_time_ms: Table1D::new(dead_axis, dead_values),
                max_duty: 0.90,
                rated_pressure_kpa: spec.fuel_system.injector_rated_pressure_pa * 1.0e-3,
            },
            charge_temp_blend,
            warmup_enrichment: Table1D::new(
                COOLANT_AXIS,
                [1.45, 1.32, 1.25, 1.15, 1.08, 1.03, 1.0, 1.0],
            ),
            cranking_pulse_ms: Table1D::new(
                COOLANT_AXIS,
                [26.0, 21.0, 19.0, 16.0, 14.0, 13.0, 12.0, 11.5],
            ),
            cranking_decay_s: 3.0,
            cranking_spark_deg: 8.0,
            after_start_enrichment: Table1D::new(
                COOLANT_AXIS,
                [0.40, 0.35, 0.30, 0.24, 0.17, 0.11, 0.05, 0.03],
            ),
            after_start_decay_s: 8.0,
            wall_film_fraction: Table1D::new(COOLANT_AXIS, x_values),
            wall_film_tau_s: Table1D::new(COOLANT_AXIS, tau_values),
            idle_target_rpm: Table1D::new(
                COOLANT_AXIS,
                [1300.0, 1250.0, 1200.0, 1100.0, 1000.0, 900.0, 820.0, 800.0],
            ),
            idle_valve_base: Table1D::new(
                COOLANT_AXIS,
                [0.42, 0.36, 0.33, 0.28, 0.25, 0.23, 0.21, 0.20],
            ),
            idle: IdleControl {
                kp: 1.5e-4,
                ki: 3.0e-4,
                base_spark_deg: 12.0,
                spark_gain: 0.03,
                spark_limit_deg: 10.0,
                window_rpm: 700.0,
            },
            iat_spark_correction: Table1D::new(
                [-20.0, 0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0],
                [1.0, 0.5, 0.0, -1.0, -3.0, -5.0, -7.0, -9.0],
            ),
            coolant_spark_correction: Table1D::new(
                COOLANT_AXIS,
                [4.0, 3.0, 2.5, 2.0, 1.0, 0.5, 0.0, -1.0],
            ),
            closed_loop: ClosedLoopFuel {
                enabled: true,
                min_coolant_c: 50.0,
                kp: 0.15,
                ki: 0.6,
                trim_limit: 0.25,
                ltft_rate: 0.05,
                ltft_limit: 0.25,
                lambda_window: 0.03,
            },
            knock: KnockControl {
                enabled: true,
                // Mechanical background noise grows with speed (valve closing, piston
                // slap); threshold sits ≈ 0.8 above the expected background.
                threshold: Table1D::new(
                    [
                        500.0, 1500.0, 2500.0, 3500.0, 4500.0, 5500.0, 6500.0, 7500.0,
                    ],
                    [0.95, 0.97, 1.01, 1.07, 1.15, 1.24, 1.36, 1.49],
                ),
                retard_step_deg: 1.5,
                max_retard_deg: 10.0,
                recovery_deg_per_s: 0.6,
            },
            limiter: RevLimiter {
                cut_rpm: redline + 200.0,
                hysteresis_rpm: 150.0,
                soft_window_rpm: 200.0,
                soft_retard_deg: 8.0,
            },
            dfco: Dfco {
                enabled: true,
                min_rpm: 1500.0,
                resume_rpm: 1150.0,
                min_coolant_c: 50.0,
                delay_s: 1.0,
            },
            throttle_map: Table1D::new(
                [0.0, 5.0, 10.0, 20.0, 35.0, 50.0, 75.0, 100.0],
                [0.0, 2.0, 4.5, 10.0, 20.0, 35.0, 62.0, 100.0],
            ),
            fan: FanControl {
                on_c: 100.0,
                off_c: 95.0,
            },
            boost: BoostControl {
                enabled: turbo,
                target_kpa: boost_target,
                base_duty: Table1D::new(RPM_AXIS, base_duty),
                kp: 0.004,
                ki: 0.008,
                kd: 0.0,
                overboost_limit_kpa: if turbo { 225.0 } else { 300.0 },
            },
            misfire: MisfireMonitor {
                enabled: true,
                threshold_fraction: 0.35,
                inertia_kg_m2: spec.geometry.rotating_inertia_kg_m2,
            },
        }
    }

    /// Checks every axis and value. A calibration edited in the UI should be validated
    /// before being flashed with [`EngineSim::set_calibration`](super::EngineSim::set_calibration).
    pub fn validate(&self) -> Result<(), CalibrationError> {
        if self.engine.cylinders == 0 || self.engine.cylinders > MAX_CYLINDERS {
            return Err(CalibrationError::InvalidValue("engine.cylinders"));
        }
        let scalars: [(&'static str, f32, f32, f32); 10] = [
            (
                "engine.displacement_cc",
                self.engine.displacement_cc,
                50.0,
                20_000.0,
            ),
            ("engine.stoich_afr", self.engine.stoich_afr, 5.0, 20.0),
            ("injector.flow_g_s", self.injector.flow_g_s, 0.1, 100.0),
            ("injector.max_duty", self.injector.max_duty, 0.1, 1.0),
            ("limiter.cut_rpm", self.limiter.cut_rpm, 1000.0, 15_000.0),
            (
                "misfire.inertia_kg_m2",
                self.misfire.inertia_kg_m2,
                0.01,
                10.0,
            ),
            ("cranking_spark_deg", self.cranking_spark_deg, -20.0, 40.0),
            ("after_start_decay_s", self.after_start_decay_s, 0.1, 120.0),
            ("cranking_decay_s", self.cranking_decay_s, 0.1, 60.0),
            (
                "boost.overboost_limit_kpa",
                self.boost.overboost_limit_kpa,
                50.0,
                500.0,
            ),
        ];
        for (name, v, lo, hi) in scalars {
            if !(v.is_finite() && v >= lo && v <= hi) {
                return Err(CalibrationError::InvalidValue(name));
            }
        }
        let maps: [(&'static str, bool); 3] = [
            ("ve", self.ve.is_valid()),
            ("ignition", self.ignition.is_valid()),
            ("lambda_target", self.lambda_target.is_valid()),
        ];
        for (name, ok) in maps {
            if !ok {
                return Err(CalibrationError::InvalidAxis(name));
            }
        }
        let curves: [(&'static str, bool); 14] = [
            (
                "injector.dead_time_ms",
                self.injector.dead_time_ms.is_valid(),
            ),
            ("charge_temp_blend", self.charge_temp_blend.is_valid()),
            ("warmup_enrichment", self.warmup_enrichment.is_valid()),
            (
                "after_start_enrichment",
                self.after_start_enrichment.is_valid(),
            ),
            ("cranking_pulse_ms", self.cranking_pulse_ms.is_valid()),
            ("wall_film_fraction", self.wall_film_fraction.is_valid()),
            ("wall_film_tau_s", self.wall_film_tau_s.is_valid()),
            ("idle_target_rpm", self.idle_target_rpm.is_valid()),
            ("idle_valve_base", self.idle_valve_base.is_valid()),
            ("iat_spark_correction", self.iat_spark_correction.is_valid()),
            (
                "coolant_spark_correction",
                self.coolant_spark_correction.is_valid(),
            ),
            ("knock.threshold", self.knock.threshold.is_valid()),
            ("throttle_map", self.throttle_map.is_valid()),
            ("boost.base_duty", self.boost.base_duty.is_valid()),
        ];
        for (name, ok) in curves {
            if !ok {
                return Err(CalibrationError::InvalidAxis(name));
            }
        }
        if !self.boost.target_kpa.is_valid() {
            return Err(CalibrationError::InvalidAxis("boost.target_kpa"));
        }
        if self
            .ve
            .values
            .iter()
            .flatten()
            .any(|v| !(0.0..=2.0).contains(v))
        {
            return Err(CalibrationError::InvalidValue("ve"));
        }
        if self
            .lambda_target
            .values
            .iter()
            .flatten()
            .any(|v| !(0.5..=1.6).contains(v))
        {
            return Err(CalibrationError::InvalidValue("lambda_target"));
        }
        Ok(())
    }
}

#[inline]
fn round_to(x: f32, q: f32) -> f32 {
    (x / q).round() * q
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_interpolation_and_clamping() {
        let t = Table1D::new([0.0, 10.0, 20.0], [0.0, 100.0, 50.0]);
        assert_eq!(t.lookup(-5.0), 0.0);
        assert_eq!(t.lookup(5.0), 50.0);
        assert_eq!(t.lookup(15.0), 75.0);
        assert_eq!(t.lookup(99.0), 50.0);
        assert_eq!(t.lookup(f32::NAN), 0.0);
        let m = Table2D::from_fn([0.0, 1.0], [0.0, 1.0], |x, y| x + 10.0 * y);
        assert!((m.lookup(0.5, 0.5) - 5.5).abs() < 1e-6);
    }

    #[test]
    fn base_calibrations_validate() {
        let na = Calibration::base_for(&EngineSpec::naturally_aspirated_2l());
        na.validate().unwrap();
        let t = Calibration::base_for(&EngineSpec::turbocharged_2l());
        t.validate().unwrap();
        assert!(t.boost.enabled);
    }

    #[test]
    fn base_spark_is_knock_limited_at_low_speed_full_load() {
        let cal = Calibration::base_for(&EngineSpec::naturally_aspirated_2l());
        let low = cal.ignition.lookup(2000.0, 100.0);
        let part = cal.ignition.lookup(2000.0, 45.0);
        assert!(part > low + 5.0, "part {part} wot {low}");
        assert!((5.0..40.0).contains(&low), "wot {low}");
    }
}

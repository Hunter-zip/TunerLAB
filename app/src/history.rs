//! Fixed-length telemetry history for the scrolling charts.

use std::collections::VecDeque;

use tunerlab_core::Telemetry;

/// Values shown on the dashboard: what a datalogger reads from the ECU's sensors, or the
/// physical truth in the instructor view.
#[derive(Debug, Clone, Copy)]
pub struct Readout {
    pub rpm: f32,
    pub map_kpa: f32,
    pub baro_kpa: f32,
    /// `NaN` while the wide-band sensor is not ready (sensor view) or the engine is stopped.
    pub lambda: f32,
    pub coolant_c: f32,
    pub oil_temp_c: f32,
    pub oil_kpa: f32,
    pub iat_c: f32,
    pub egt_c: f32,
    pub fuel_kpa: f32,
    pub battery_v: f32,
    pub speed_kph: f32,
    pub turbo_rpm: f32,
}

impl Readout {
    /// Sensor readings (`truth == false`) or physical truth; `ambient_kpa` is the ambient
    /// pressure shown as barometric pressure in the truth view.
    pub fn new(t: &Telemetry, truth: bool, ambient_kpa: f32) -> Self {
        let running = t.rpm > 1.0;
        if truth {
            Self {
                rpm: t.rpm,
                map_kpa: t.manifold_pressure_kpa,
                baro_kpa: ambient_kpa,
                lambda: if running { t.lambda } else { f32::NAN },
                coolant_c: t.coolant_temp_c,
                oil_temp_c: t.oil_temp_c,
                oil_kpa: t.oil_pressure_kpa,
                iat_c: t.intake_air_temp_c,
                egt_c: t.egt_c,
                fuel_kpa: t.fuel_pressure_kpa,
                battery_v: t.battery_v,
                speed_kph: t.vehicle_speed_kph,
                turbo_rpm: t.turbo_rpm,
            }
        } else {
            let s = &t.sensors;
            Self {
                rpm: s.rpm,
                map_kpa: s.map_kpa,
                baro_kpa: s.baro_kpa,
                lambda: if running && s.lambda_ready {
                    s.lambda
                } else {
                    f32::NAN
                },
                coolant_c: s.ect_c,
                oil_temp_c: s.oil_temp_c,
                oil_kpa: s.oil_pressure_kpa,
                iat_c: s.iat_c,
                egt_c: s.egt_c,
                fuel_kpa: s.fuel_pressure_kpa,
                battery_v: s.battery_v,
                speed_kph: s.vehicle_speed_kph,
                turbo_rpm: s.turbo_rpm,
            }
        }
    }
}

/// MBT advance when it describes a real firing cycle: `None` during fuel cut, deep overrun
/// (MAP below 15 kPa) or when the estimate sits on a bound of the core's −10…65° search.
pub fn mbt(t: &Telemetry) -> Option<f32> {
    let valid = t.rpm > 1.0
        && t.injector_duty_pct > 0.5
        && t.manifold_pressure_kpa > 15.0
        && t.mbt_advance_deg > -9.5
        && t.mbt_advance_deg < 64.5;
    valid.then_some(t.mbt_advance_deg)
}

/// Samples kept per channel: 30 s at 20 Hz.
pub const SAMPLES: usize = 600;
/// Simulated time between samples \[s\].
const PERIOD_S: f64 = 0.05;

/// One ring buffer per charted channel.
#[derive(Default)]
pub struct History {
    pub rpm: VecDeque<f32>,
    pub map_kpa: VecDeque<f32>,
    pub lambda: VecDeque<f32>,
    pub lambda_target: VecDeque<f32>,
    pub spark: VecDeque<f32>,
    pub mbt: VecDeque<f32>,
    last_time_s: Option<f64>,
}

fn push(buf: &mut VecDeque<f32>, v: f32) {
    if buf.len() == SAMPLES {
        buf.pop_front();
    }
    buf.push_back(v);
}

impl History {
    /// Records the readout `r` of `t` if one sample period of simulated time has passed.
    pub fn record(&mut self, t: &Telemetry, r: &Readout) {
        if let Some(last) = self.last_time_s {
            if t.time_s >= last && t.time_s - last < PERIOD_S {
                return;
            }
        }
        self.last_time_s = Some(t.time_s);
        let running = t.rpm > 1.0;
        let gate = |v: f32| if running { v } else { f32::NAN };
        push(&mut self.rpm, r.rpm);
        push(&mut self.map_kpa, r.map_kpa);
        // `f32::min` would turn a missing reading (NaN) into the cap.
        let lambda = if r.lambda.is_finite() {
            r.lambda.min(2.0)
        } else {
            f32::NAN
        };
        push(&mut self.lambda, lambda);
        push(&mut self.lambda_target, gate(t.lambda_target));
        push(&mut self.spark, gate(t.ignition_advance_deg));
        push(&mut self.mbt, mbt(t).unwrap_or(f32::NAN));
    }

    /// Forgets everything (new engine, reset).
    pub fn clear(&mut self) {
        *self = Self::default();
    }
}

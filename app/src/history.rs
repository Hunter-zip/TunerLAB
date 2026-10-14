//! Fixed-length telemetry history for the scrolling charts.

use std::collections::VecDeque;

use tunerlab_core::Telemetry;

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
    /// Records `t` if at least one sample period of simulated time has passed.
    pub fn record(&mut self, t: &Telemetry) {
        if let Some(last) = self.last_time_s {
            if t.time_s >= last && t.time_s - last < PERIOD_S {
                return;
            }
        }
        self.last_time_s = Some(t.time_s);
        let running = t.rpm > 1.0;
        push(&mut self.rpm, t.rpm);
        push(&mut self.map_kpa, t.manifold_pressure_kpa);
        push(
            &mut self.lambda,
            if running { t.lambda.min(2.0) } else { f32::NAN },
        );
        push(
            &mut self.lambda_target,
            if running { t.lambda_target } else { f32::NAN },
        );
        push(
            &mut self.spark,
            if running {
                t.ignition_advance_deg
            } else {
                f32::NAN
            },
        );
        push(
            &mut self.mbt,
            if running { t.mbt_advance_deg } else { f32::NAN },
        );
    }

    /// Forgets everything (new engine, reset).
    pub fn clear(&mut self) {
        *self = Self::default();
    }
}

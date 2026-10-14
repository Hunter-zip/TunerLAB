//! Wide-open-throttle power sweep on a copy of the engine, run on a worker thread so the
//! dashboard keeps updating.

use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::thread;

use tunerlab_core::{
    Ambient, Calibration, DynoParams, EngineCondition, EngineSim, EngineSpec, FailureCause,
    LoadModel,
};

/// Frame period used to drive the simulation copy \[s\].
const FRAME_S: f32 = 1.0 / 60.0;
/// Speed step between measuring points \[rpm\].
const STEP_RPM: f32 = 250.0;
/// First measuring point \[rpm\].
const START_RPM: f32 = 1500.0;
/// Settling time before each measurement \[s\].
const SETTLE_S: f32 = 3.0;
/// Averaging window per point (smooths the firing ripple) \[s\].
const AVERAGE_S: f32 = 1.0;

/// One measured operating point.
#[derive(Debug, Clone, Copy)]
pub struct DynoPoint {
    pub rpm: f32,
    pub torque_nm: f32,
    pub power_kw: f32,
    pub lambda: f32,
    pub spark_deg: f32,
    pub mbt_deg: f32,
    pub map_kpa: f32,
    pub egt_c: f32,
    pub knock_bar: f32,
}

/// A complete (or aborted) sweep.
#[derive(Debug, Clone, Default)]
pub struct DynoRun {
    pub points: Vec<DynoPoint>,
    pub failure: Option<FailureCause>,
}

impl DynoRun {
    pub fn peak_torque(&self) -> Option<&DynoPoint> {
        self.points
            .iter()
            .max_by(|a, b| a.torque_nm.total_cmp(&b.torque_nm))
    }

    pub fn peak_power(&self) -> Option<&DynoPoint> {
        self.points
            .iter()
            .max_by(|a, b| a.power_kw.total_cmp(&b.power_kw))
    }
}

enum Msg {
    Progress(f32),
    Point(DynoPoint),
    Done(Option<FailureCause>),
}

/// Dyno bench: the last two runs and an optional sweep in progress.
#[derive(Default)]
pub struct DynoBench {
    worker: Option<Receiver<Msg>>,
    pub progress: f32,
    pub current: Option<DynoRun>,
    pub previous: Option<DynoRun>,
}

impl DynoBench {
    pub fn running(&self) -> bool {
        self.worker.is_some()
    }

    /// Starts a sweep of `spec` on `calibration` at `ambient`; the current run becomes the
    /// previous one.
    pub fn start(
        &mut self,
        spec: EngineSpec,
        calibration: Calibration,
        ambient: Ambient,
        seed: u64,
    ) {
        if self.running() {
            return;
        }
        let (tx, rx) = mpsc::channel();
        self.previous = self.current.take();
        self.current = Some(DynoRun::default());
        self.progress = 0.0;
        self.worker = Some(rx);
        thread::spawn(move || {
            let failure = sweep(spec, calibration, ambient, seed, |m| {
                // The receiver disappears only when the app closes: stop quietly then.
                tx.send(m).is_ok()
            });
            let _ = tx.send(Msg::Done(failure));
        });
    }

    /// Collects results from the worker; call once per frame.
    pub fn poll(&mut self) {
        let Some(rx) = &self.worker else { return };
        loop {
            match rx.try_recv() {
                Ok(Msg::Progress(p)) => self.progress = p,
                Ok(Msg::Point(pt)) => {
                    if let Some(run) = &mut self.current {
                        run.points.push(pt);
                    }
                }
                Ok(Msg::Done(failure)) => {
                    if let Some(run) = &mut self.current {
                        run.failure = failure;
                    }
                    self.progress = 1.0;
                    self.worker = None;
                    return;
                }
                Err(TryRecvError::Empty) => return,
                Err(TryRecvError::Disconnected) => {
                    self.worker = None;
                    return;
                }
            }
        }
    }
}

fn run_for(sim: &mut EngineSim, seconds: f32) {
    for _ in 0..(seconds / FRAME_S).round() as usize {
        sim.tick(FRAME_S);
    }
}

fn failed(sim: &EngineSim) -> Option<FailureCause> {
    match sim.condition() {
        EngineCondition::Failed(cause) => Some(cause),
        _ => None,
    }
}

/// Runs the sweep, reporting through `send` (which returns `false` to abort).
fn sweep(
    spec: EngineSpec,
    calibration: Calibration,
    ambient: Ambient,
    seed: u64,
    mut send: impl FnMut(Msg) -> bool,
) -> Option<FailureCause> {
    let Ok(mut sim) = EngineSim::new(spec, calibration, seed) else {
        return None;
    };
    sim.update_controls(|c| c.ambient = ambient);
    sim.soak_to_ambient();
    sim.prewarm();
    sim.update_controls(|c| {
        c.ignition_on = true;
        c.starter = true;
    });
    run_for(&mut sim, 1.5);
    sim.update_controls(|c| c.starter = false);
    run_for(&mut sim, 3.0);
    let redline = spec.limits.redline_rpm;
    let points = (((redline - START_RPM) / STEP_RPM).floor() as usize) + 1;
    for i in 0..points {
        let rpm = START_RPM + STEP_RPM * i as f32;
        sim.update_controls(|c| {
            c.pedal = 1.0;
            c.load = LoadModel::Dyno(DynoParams {
                target_rpm: rpm,
                ..DynoParams::default()
            });
        });
        run_for(&mut sim, SETTLE_S);
        let frames = (AVERAGE_S / FRAME_S).round() as usize;
        let mut acc = DynoPoint {
            rpm: 0.0,
            torque_nm: 0.0,
            power_kw: 0.0,
            lambda: 0.0,
            spark_deg: 0.0,
            mbt_deg: 0.0,
            map_kpa: 0.0,
            egt_c: 0.0,
            knock_bar: 0.0,
        };
        for _ in 0..frames {
            sim.tick(FRAME_S);
            let t = sim.telemetry();
            let n = t.cylinders.max(1);
            acc.rpm += t.rpm;
            acc.torque_nm += t.load_torque_nm;
            acc.power_kw += t.load_torque_nm * t.rpm * core::f32::consts::TAU / 60.0 / 1000.0;
            acc.lambda += t.lambda;
            acc.spark_deg += t.ignition_advance_deg;
            acc.mbt_deg += t.mbt_advance_deg;
            acc.map_kpa += t.manifold_pressure_kpa;
            acc.egt_c += t.egt_c;
            acc.knock_bar = acc
                .knock_bar
                .max(t.knock_intensity[..n].iter().copied().fold(0.0, f32::max));
        }
        let k = 1.0 / frames as f32;
        let point = DynoPoint {
            rpm: acc.rpm * k,
            torque_nm: acc.torque_nm * k,
            power_kw: acc.power_kw * k,
            lambda: acc.lambda * k,
            spark_deg: acc.spark_deg * k,
            mbt_deg: acc.mbt_deg * k,
            map_kpa: acc.map_kpa * k,
            egt_c: acc.egt_c * k,
            knock_bar: acc.knock_bar,
        };
        if let Some(cause) = failed(&sim) {
            return Some(cause);
        }
        if !send(Msg::Point(point)) || !send(Msg::Progress((i + 1) as f32 / points as f32)) {
            return None;
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sweep_reproduces_the_base_power_curve() {
        let spec = EngineSpec::naturally_aspirated_2l();
        let cal = Calibration::base_for(&spec);
        let mut points = Vec::new();
        let failure = sweep(spec, cal, Ambient::default(), 7, |m| {
            if let Msg::Point(p) = m {
                points.push(p);
            }
            true
        });
        assert!(failure.is_none());
        assert!(points.len() >= 15, "{} points", points.len());
        let run = DynoRun { points, failure };
        let tq = run.peak_torque().unwrap();
        let pw = run.peak_power().unwrap();
        assert!(
            (180.0..200.0).contains(&tq.torque_nm),
            "peak torque {}",
            tq.torque_nm
        );
        assert!(
            (100.0..115.0).contains(&pw.power_kw),
            "peak power {}",
            pw.power_kw
        );
    }
}

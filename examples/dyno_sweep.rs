//! Headless dynamometer run: starts the engine, warms it up and records a steady-state
//! wide-open-throttle power curve on the base calibration.
//!
//! ```text
//! cargo run --release --example dyno_sweep            # naturally aspirated 2.0 L
//! cargo run --release --example dyno_sweep -- turbo   # turbocharged 2.0 L
//! cargo run --release --example dyno_sweep -- turbo pl
//! ```

use tunerlab_core::engine_sim::controls::{DynoParams, LoadModel};
use tunerlab_core::{EngineSim, EngineSpec, Language, Localize};

/// UI frame period used to drive the simulation \[s\].
const FRAME_S: f32 = 1.0 / 60.0;

fn run(sim: &mut EngineSim, seconds: f32) {
    let frames = (seconds / FRAME_S).round() as usize;
    for _ in 0..frames {
        sim.tick(FRAME_S);
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let turbo = args.iter().any(|a| a == "turbo");
    let lang = args
        .iter()
        .find_map(|a| Language::from_code(a))
        .unwrap_or(Language::En);
    let spec = if turbo {
        EngineSpec::turbocharged_2l()
    } else {
        EngineSpec::naturally_aspirated_2l()
    };
    let redline = spec.limits.redline_rpm;
    let mut sim = EngineSim::with_base_calibration(spec, 2024).expect("valid preset");
    sim.prewarm();

    sim.update_controls(|c| {
        c.ignition_on = true;
        c.starter = true;
    });
    run(&mut sim, 1.5);
    sim.update_controls(|c| c.starter = false);
    run(&mut sim, 5.0);
    let t = sim.telemetry();
    println!(
        "{}: {:.0} rpm, MAP {:.1} kPa, λ {:.3}, spark {:.1}°",
        sim.status_text(lang),
        t.rpm,
        t.manifold_pressure_kpa,
        t.lambda,
        t.ignition_advance_deg
    );

    println!();
    println!(" rpm  | torque N·m | power kW | MAP kPa | λ     | spark° | MBT°  | VE    | EGT °C | knock | boost kPa");
    println!("------+------------+----------+---------+-------+--------+-------+-------+--------+-------+----------");
    let mut rpm = 1500.0_f32;
    while rpm <= redline + 1.0 {
        sim.update_controls(|c| {
            c.load = LoadModel::Dyno(DynoParams {
                target_rpm: rpm,
                ..DynoParams::default()
            });
            c.pedal = 1.0;
        });
        run(&mut sim, 4.0);
        // Average over one second to smooth firing-frequency ripple.
        let (mut tq, mut pw, mut n) = (0.0_f32, 0.0_f32, 0.0_f32);
        for _ in 0..60 {
            sim.tick(FRAME_S);
            tq += sim.telemetry().load_torque_nm;
            pw += sim.telemetry().load_torque_nm * sim.telemetry().rpm * core::f32::consts::TAU
                / 60.0
                / 1000.0;
            n += 1.0;
        }
        let t = sim.telemetry();
        let knock = t.knock_intensity[..t.cylinders]
            .iter()
            .cloned()
            .fold(0.0_f32, f32::max);
        println!(
            "{:5.0} | {:10.1} | {:8.1} | {:7.1} | {:.3} | {:6.1} | {:5.1} | {:.3} | {:6.0} | {:5.2} | {:8.1}",
            t.rpm,
            tq / n,
            pw / n,
            t.manifold_pressure_kpa,
            t.lambda,
            t.ignition_advance_deg,
            t.mbt_advance_deg,
            t.volumetric_efficiency,
            t.egt_c,
            knock,
            t.boost_pressure_kpa
        );
        rpm += 500.0;
    }

    println!();
    for w in sim.telemetry().warnings.iter() {
        println!("! {}", w.localized(lang));
    }
    for code in sim.dtcs().iter() {
        println!("DTC {} – {}", code.code(), code.localized(lang));
    }
}

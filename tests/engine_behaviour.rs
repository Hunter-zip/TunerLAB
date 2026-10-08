//! End-to-end behaviour of the engine simulation: starting, idling, power output,
//! closed-loop control, knock, faults, damage, determinism and input hygiene.

use tunerlab_core::engine_sim::calibration::CalibrationError;
use tunerlab_core::engine_sim::controls::{DynoParams, LoadModel};
use tunerlab_core::engine_sim::dtc::DtcCode;
use tunerlab_core::engine_sim::SimError;
use tunerlab_core::{
    Calibration, EngineCondition, EngineSim, EngineSpec, FailureCause, Fault, FaultId, Language,
    Localize, Warning,
};

const FRAME: f32 = 1.0 / 60.0;

fn run(sim: &mut EngineSim, seconds: f32) {
    for _ in 0..(seconds / FRAME).round() as usize {
        sim.tick(FRAME);
    }
}

/// Mean of `f(telemetry)` over `seconds`.
fn average(sim: &mut EngineSim, seconds: f32, f: impl Fn(&tunerlab_core::Telemetry) -> f32) -> f32 {
    let frames = (seconds / FRAME).round() as usize;
    let mut sum = 0.0;
    for _ in 0..frames {
        sim.tick(FRAME);
        sum += f(sim.telemetry());
    }
    sum / frames as f32
}

fn start(sim: &mut EngineSim) {
    sim.update_controls(|c| {
        c.ignition_on = true;
        c.starter = true;
    });
    run(sim, 1.5);
    sim.update_controls(|c| c.starter = false);
}

fn warm_na(seed: u64) -> EngineSim {
    let mut sim =
        EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), seed).unwrap();
    sim.prewarm();
    start(&mut sim);
    sim
}

fn dyno(sim: &mut EngineSim, rpm: f32, pedal: f32) {
    sim.update_controls(|c| {
        c.pedal = pedal;
        c.load = LoadModel::Dyno(DynoParams {
            target_rpm: rpm,
            ..DynoParams::default()
        });
    });
}

#[test]
fn cold_engine_starts_and_settles_at_fast_idle() {
    let mut sim =
        EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 1).unwrap();
    start(&mut sim);
    assert_eq!(sim.condition(), EngineCondition::Running);
    run(&mut sim, 30.0);
    let rpm = average(&mut sim, 5.0, |t| t.rpm);
    // Cold (≈ 27 °C coolant) idle target ≈ 1100 rpm.
    assert!((950.0..1250.0).contains(&rpm), "cold idle {rpm}");
    assert!(
        sim.dtcs().is_empty(),
        "{:?}",
        sim.dtcs().iter().collect::<Vec<_>>()
    );
}

#[test]
fn engine_starts_at_minus_ten_celsius() {
    let mut sim =
        EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 2).unwrap();
    sim.update_controls(|c| c.ambient.temperature_k = 263.15);
    sim.soak_to_ambient();
    start(&mut sim);
    run(&mut sim, 20.0);
    assert_eq!(sim.condition(), EngineCondition::Running);
    assert!(sim.telemetry().rpm > 1000.0, "{}", sim.telemetry().rpm);
}

#[test]
fn warm_idle_is_stable_and_closed_loop_converges() {
    let mut sim = warm_na(3);
    run(&mut sim, 40.0);
    let t = *sim.telemetry();
    assert!(t.closed_loop);
    let rpm = average(&mut sim, 5.0, |t| t.rpm);
    let lambda = average(&mut sim, 5.0, |t| t.sensors.lambda);
    assert!((740.0..880.0).contains(&rpm), "idle {rpm}");
    assert!((lambda - 1.0).abs() < 0.03, "lambda {lambda}");
    // Realistic hot-idle manifold vacuum and EGT.
    assert!(
        (18.0..40.0).contains(&t.manifold_pressure_kpa),
        "MAP {}",
        t.manifold_pressure_kpa
    );
    assert!((400.0..750.0).contains(&t.egt_c), "EGT {}", t.egt_c);
}

#[test]
fn naturally_aspirated_power_curve_is_plausible() {
    let mut sim = warm_na(4);
    run(&mut sim, 5.0);
    let mut peak_torque = 0.0_f32;
    let mut peak_power = 0.0_f32;
    for rpm in [2000.0, 3500.0, 4500.0, 6000.0] {
        dyno(&mut sim, rpm, 1.0);
        run(&mut sim, 4.0);
        let torque = average(&mut sim, 1.0, |t| t.load_torque_nm);
        let power = torque * rpm * core::f32::consts::TAU / 60.0 / 1000.0;
        peak_torque = peak_torque.max(torque);
        peak_power = peak_power.max(power);
        let t = sim.telemetry();
        assert!(t.peak_pressure_bar[0] > 35.0 && t.peak_pressure_bar[0] < 110.0);
        assert!(
            (650.0..980.0).contains(&t.egt_c),
            "EGT {} at {rpm}",
            t.egt_c
        );
    }
    // 2.0 L naturally aspirated: ≈ 90–100 N·m/L and ≈ 50–60 kW/L.
    assert!(
        (170.0..220.0).contains(&peak_torque),
        "torque {peak_torque}"
    );
    assert!((90.0..125.0).contains(&peak_power), "power {peak_power}");
}

#[test]
fn turbo_engine_reaches_boost_target() {
    let mut sim = EngineSim::with_base_calibration(EngineSpec::turbocharged_2l(), 5).unwrap();
    sim.prewarm();
    start(&mut sim);
    run(&mut sim, 3.0);
    dyno(&mut sim, 4000.0, 1.0);
    run(&mut sim, 8.0);
    let t = *sim.telemetry();
    assert!(
        (t.manifold_pressure_kpa - t.boost_target_kpa).abs() < 10.0,
        "MAP {} target {}",
        t.manifold_pressure_kpa,
        t.boost_target_kpa
    );
    assert!(t.turbo_rpm > 80_000.0);
    let torque = average(&mut sim, 1.0, |t| t.load_torque_nm);
    assert!((260.0..360.0).contains(&torque), "torque {torque}");
    assert!(!sim.dtcs().contains(DtcCode::P0234) && !sim.dtcs().contains(DtcCode::P0299));
}

#[test]
fn excessive_advance_knocks_and_knock_control_retards() {
    let mut sim = warm_na(6);
    sim.update_calibration(|cal| {
        for row in cal.ignition.values.iter_mut() {
            for v in row.iter_mut() {
                *v += 12.0;
            }
        }
    })
    .unwrap();
    dyno(&mut sim, 2000.0, 1.0);
    run(&mut sim, 6.0);
    let retard = sim.telemetry().knock_retard_deg[..4]
        .iter()
        .cloned()
        .fold(0.0_f32, f32::max);
    assert!(retard > 3.0, "knock retard {retard}");
}

#[test]
fn uncontrolled_detonation_damages_pistons() {
    let mut sim = warm_na(7);
    sim.update_calibration(|cal| {
        cal.knock.enabled = false;
        for row in cal.ignition.values.iter_mut() {
            for v in row.iter_mut() {
                *v += 18.0;
            }
        }
    })
    .unwrap();
    dyno(&mut sim, 2000.0, 1.0);
    run(&mut sim, 5.0);
    assert!(sim.telemetry().warnings.contains(Warning::Knock));
    run(&mut sim, 60.0);
    let worst = sim.telemetry().health.piston[..4]
        .iter()
        .cloned()
        .fold(1.0_f32, f32::min);
    assert!(worst < 0.9, "piston health {worst}");
}

#[test]
fn weak_ignition_coil_misfires_under_load_and_sets_cylinder_code() {
    let mut sim = warm_na(8);
    sim.inject_fault(Fault::IgnitionCoilWeak {
        cylinder: 2,
        strength: 0.4,
    })
    .unwrap();
    run(&mut sim, 5.0);
    assert!(!sim.dtcs().contains(DtcCode::P0303), "misfires at idle");
    dyno(&mut sim, 3000.0, 1.0);
    run(&mut sim, 15.0);
    assert!(
        sim.dtcs().contains(DtcCode::P0303),
        "{:?} misfires {:?} rpm {}",
        sim.dtcs().iter().collect::<Vec<_>>(),
        sim.telemetry().misfire_count,
        sim.telemetry().rpm
    );
    assert!(sim.telemetry().misfire_count[2] > sim.telemetry().misfire_count[0]);
}

#[test]
fn vacuum_leak_on_speed_density_raises_idle_not_trims() {
    // Speed-density ECUs meter leak air through MAP, so a vacuum leak shows up as a closed
    // idle valve and, beyond its authority, as high idle (P0507) — not as a lean trim.
    let mut sim = warm_na(9);
    run(&mut sim, 20.0);
    let iac_before = average(&mut sim, 2.0, |t| t.idle_valve_pct);
    sim.inject_fault(Fault::VacuumLeak { diameter_mm: 7.0 })
        .unwrap();
    run(&mut sim, 40.0);
    let t = *sim.telemetry();
    assert!(
        t.idle_valve_pct < iac_before * 0.5,
        "IAC {} vs {iac_before}",
        t.idle_valve_pct
    );
    assert!(t.rpm > 1000.0, "idle {}", t.rpm);
    assert!(sim.dtcs().contains(DtcCode::P0507));
    assert!(
        (t.stft_pct + t.ltft_pct).abs() < 10.0,
        "trim {} {} rpm {} cl {}",
        t.stft_pct,
        t.ltft_pct,
        t.rpm,
        t.closed_loop
    );
    assert!(sim.is_fault_active(FaultId::VacuumLeak));
}

#[test]
fn stretched_timing_chain_sets_cam_correlation_code() {
    let mut sim = warm_na(10);
    sim.inject_fault(Fault::TimingChainStretch { retard_deg: 8.0 })
        .unwrap();
    run(&mut sim, 5.0);
    assert!(sim.dtcs().contains(DtcCode::P0016));
}

#[test]
fn over_rev_without_limiter_destroys_the_engine() {
    let mut sim = warm_na(11);
    sim.update_calibration(|cal| cal.limiter.cut_rpm = 14_000.0)
        .unwrap();
    sim.update_controls(|c| c.pedal = 1.0);
    run(&mut sim, 30.0);
    let t = sim.telemetry();
    let valves_bent = t.health.valves[..4].iter().any(|v| *v <= 0.0);
    let failed = matches!(t.condition, EngineCondition::Failed(_));
    assert!(valves_bent || failed, "{:?}", t.condition);
    let damage_logged = sim.events().iter().any(|e| {
        matches!(
            e.kind,
            tunerlab_core::engine_sim::events::EventKind::Damage(FailureCause::BentValve { .. })
                | tunerlab_core::engine_sim::events::EventKind::Damage(
                    FailureCause::ThrownRod { .. }
                )
        )
    });
    assert!(damage_logged);
}

#[test]
fn simulation_is_deterministic_for_a_seed() {
    let mut a = warm_na(42);
    let mut b = warm_na(42);
    dyno(&mut a, 3000.0, 0.6);
    dyno(&mut b, 3000.0, 0.6);
    run(&mut a, 5.0);
    run(&mut b, 5.0);
    assert_eq!(a.telemetry(), b.telemetry());
    // Frame rate must not matter either: 60 Hz versus 144 Hz frames.
    let mut c = warm_na(42);
    dyno(&mut c, 3000.0, 0.6);
    for _ in 0..720 {
        c.tick(5.0 / 720.0);
    }
    assert!((c.time_s() - a.time_s()).abs() < 1.0e-3);
    assert!((c.telemetry().rpm - a.telemetry().rpm).abs() < 50.0);
}

#[test]
fn hostile_inputs_are_sanitised() {
    let mut sim = warm_na(12);
    sim.update_controls(|c| {
        c.pedal = f32::NAN;
        c.ambient.temperature_k = f32::INFINITY;
        c.fuel_octane_ron = -5.0;
    });
    sim.tick(f32::NAN);
    sim.tick(-1.0);
    sim.tick(1.0e9);
    run(&mut sim, 2.0);
    let t = sim.telemetry();
    assert!(t.rpm.is_finite() && t.brake_torque_nm.is_finite() && t.lambda.is_finite());
    assert_eq!(sim.controls().pedal, 0.0);
}

#[test]
fn invalid_calibrations_are_rejected_atomically() {
    let mut sim = warm_na(13);
    let before = *sim.calibration();
    let err = sim.update_calibration(|cal| cal.ve.x_axis[3] = cal.ve.x_axis[2]);
    assert_eq!(
        err,
        Err(SimError::Calibration(CalibrationError::InvalidAxis("ve")))
    );
    assert_eq!(*sim.calibration(), before);
    let turbo_cal = Calibration::base_for(&EngineSpec::turbocharged_2l());
    let mut v6 = turbo_cal;
    v6.engine.cylinders = 6;
    assert_eq!(sim.set_calibration(v6), Err(SimError::CylinderMismatch));
}

#[test]
fn status_is_localised_in_polish_and_english() {
    let mut sim =
        EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 14).unwrap();
    assert_eq!(sim.status_text(Language::En), "Engine off");
    assert_eq!(sim.status_text(Language::Pl), "Silnik wyłączony");
    start(&mut sim);
    run(&mut sim, 1.0);
    assert_eq!(sim.status_text(Language::Pl), "Silnik pracuje");
    assert_eq!(
        DtcCode::P0171.localized(Language::Pl),
        "Mieszanka zbyt uboga"
    );
}

#[test]
fn combustion_events_are_published_for_audio() {
    let mut sim = warm_na(15);
    run(&mut sim, 1.0);
    let seq0 = sim.cylinder_events().next_seq();
    let rpm = sim.telemetry().rpm;
    run(&mut sim, 1.0);
    let fired = sim.cylinder_events().next_seq() - seq0;
    // Four-stroke I4: two firings per revolution.
    let expected = rpm / 60.0 * 2.0;
    assert!(
        (fired as f32 - expected).abs() < expected * 0.15,
        "{fired} vs {expected}"
    );
    let ev = sim.cylinder_events().since(seq0).last().unwrap();
    assert!(ev.peak_pressure_bar > 3.0 && ev.exhaust_temp_k > 600.0);
}

#[test]
fn dead_knock_sensor_is_detected_and_disables_protection() {
    let mut sim = warm_na(16);
    sim.inject_fault(Fault::KnockSensorDead).unwrap();
    dyno(&mut sim, 3500.0, 0.8);
    run(&mut sim, 8.0);
    assert!(sim.dtcs().contains(DtcCode::P0325));
    sim.update_calibration(|cal| {
        for row in cal.ignition.values.iter_mut() {
            for v in row.iter_mut() {
                *v += 12.0;
            }
        }
    })
    .unwrap();
    dyno(&mut sim, 2000.0, 1.0);
    run(&mut sim, 5.0);
    // No feedback: the ECU cannot retard, the engine keeps knocking.
    assert!(sim.telemetry().knock_retard_deg[..4]
        .iter()
        .all(|r| *r == 0.0));
    assert!(sim.telemetry().warnings.contains(Warning::Knock));
}

#[test]
fn weak_fuel_pump_starves_the_engine_at_full_load() {
    let mut sim = warm_na(17);
    sim.inject_fault(Fault::FuelPumpWeak {
        capacity_fraction: 0.2,
    })
    .unwrap();
    dyno(&mut sim, 2000.0, 0.15);
    run(&mut sim, 5.0);
    assert!(
        !sim.dtcs().contains(DtcCode::P0087),
        "low load must be fine"
    );
    dyno(&mut sim, 6000.0, 1.0);
    run(&mut sim, 8.0);
    let t = sim.telemetry();
    assert!(t.fuel_pressure_kpa < 240.0, "rail {}", t.fuel_pressure_kpa);
    assert!(t.lambda > 0.95, "lambda {}", t.lambda);
    assert!(sim.dtcs().contains(DtcCode::P0087));
}

#[test]
fn slow_oxygen_sensor_fails_the_fuel_cut_response_test() {
    let mut sim = warm_na(18);
    sim.inject_fault(Fault::OxygenSensorSlow { factor: 20.0 })
        .unwrap();
    run(&mut sim, 16.0);
    // Free rev, then snap the throttle shut: deceleration fuel cut-off exercises the
    // sensor's lean response.
    for _ in 0..3 {
        sim.update_controls(|c| c.pedal = 0.35);
        run(&mut sim, 1.5);
        sim.update_controls(|c| c.pedal = 0.0);
        run(&mut sim, 4.0);
    }
    assert!(
        sim.dtcs().contains(DtcCode::P0133),
        "{:?}",
        sim.dtcs().iter().collect::<Vec<_>>()
    );
}

#[test]
fn thermostat_stuck_closed_overheats_under_load() {
    let mut sim = warm_na(19);
    sim.inject_fault(Fault::ThermostatStuckClosed).unwrap();
    dyno(&mut sim, 4000.0, 0.7);
    run(&mut sim, 150.0);
    let t = sim.telemetry();
    assert!(t.coolant_temp_c > 112.0, "coolant {}", t.coolant_temp_c);
    assert!(t.warnings.contains(Warning::Overheat));
    assert!(sim.dtcs().contains(DtcCode::P0217));
}

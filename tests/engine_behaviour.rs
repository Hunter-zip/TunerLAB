//! End-to-end behaviour of the engine simulation: starting, idling, power output,
//! closed-loop control, knock, faults, damage, determinism and input hygiene.

use tunerlab_core::engine_sim::calibration::CalibrationError;
use tunerlab_core::engine_sim::controls::{DynoParams, LoadModel, VehicleParams};
use tunerlab_core::engine_sim::dtc::DtcCode;
use tunerlab_core::engine_sim::events::EventKind;
use tunerlab_core::engine_sim::faults::FaultError;
use tunerlab_core::engine_sim::SimError;
use tunerlab_core::{
    Calibration, EngineCondition, EngineSim, EngineSpec, FailureCause, Fault, FaultId, Language,
    Localize, MessageKey, Warning,
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
    // The start flare must settle onto the cold fast-idle target (≈ 1250 rpm at −10 °C)
    // instead of hanging above the idle window and setting a false high-idle code.
    let rpm = average(&mut sim, 20.0, |t| t.rpm);
    assert!((1050.0..1450.0).contains(&rpm), "cold idle {rpm}");
    assert!(
        sim.dtcs().is_empty(),
        "{:?}",
        sim.dtcs().iter().collect::<Vec<_>>()
    );
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
        capacity_fraction: 0.3,
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

fn warm_turbo(seed: u64) -> EngineSim {
    let mut sim = EngineSim::with_base_calibration(EngineSpec::turbocharged_2l(), seed).unwrap();
    sim.prewarm();
    start(&mut sim);
    sim
}

fn vehicle(sim: &mut EngineSim, gear: u8, clutch: f32, pedal: f32) {
    sim.update_controls(|c| {
        c.pedal = pedal;
        c.load = LoadModel::Vehicle(VehicleParams {
            gear,
            clutch,
            ..VehicleParams::default()
        });
    });
}

/// Pulls away in first gear, letting the clutch in over two seconds.
fn drive_away(sim: &mut EngineSim) {
    for i in 0..=120 {
        vehicle(sim, 1, i as f32 / 120.0, 0.35);
        sim.tick(FRAME);
    }
    run(sim, 2.0);
}

#[test]
fn cold_idle_keeps_warming_up() {
    // Regression: the thermal network once stalled the coolant near 54 °C at idle because
    // per-step heat increments fell below f32 resolution.
    let mut sim =
        EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 20).unwrap();
    start(&mut sim);
    run(&mut sim, 540.0);
    let ect_9min = sim.telemetry().coolant_temp_c;
    run(&mut sim, 60.0);
    let ect_10min = sim.telemetry().coolant_temp_c;
    assert!(ect_10min > 62.0, "coolant after 10 min idle {ect_10min}");
    assert!(ect_10min > ect_9min + 0.5, "warm-up stalled at {ect_10min}");
    // A healthy engine idling from cold has not consumed enough air for the thermostat
    // monitor to judge it, so P0128 must not set.
    assert!(!sim.dtcs().contains(DtcCode::P0128));
}

#[test]
fn thermostat_stuck_open_sets_p0128_under_load() {
    let mut sim =
        EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 21).unwrap();
    sim.inject_fault(Fault::ThermostatStuckOpen).unwrap();
    start(&mut sim);
    dyno(&mut sim, 2500.0, 0.3);
    run(&mut sim, 240.0);
    assert!(sim.telemetry().coolant_temp_c < 60.0);
    assert!(sim.dtcs().contains(DtcCode::P0128));
}

#[test]
fn misfire_is_detected_and_attributed_while_driving_in_gear() {
    let mut sim = warm_na(22);
    sim.inject_fault(Fault::IgnitionCoilWeak {
        cylinder: 2,
        strength: 0.4,
    })
    .unwrap();
    drive_away(&mut sim);
    vehicle(&mut sim, 2, 1.0, 0.6);
    run(&mut sim, 4.0);
    vehicle(&mut sim, 3, 1.0, 1.0);
    run(&mut sim, 8.0);
    let codes: Vec<_> = sim.dtcs().iter().collect();
    assert!(sim.dtcs().contains(DtcCode::P0303), "{codes:?}");
    // The clutch damper must not smear the signature onto other cylinders.
    assert_eq!(codes, vec![DtcCode::P0303]);
    assert!(sim.telemetry().vehicle_speed_kph > 60.0);
}

#[test]
fn declutching_after_a_coast_in_gear_does_not_stall() {
    let mut sim = warm_na(23);
    drive_away(&mut sim);
    vehicle(&mut sim, 2, 1.0, 0.4);
    run(&mut sim, 4.0);
    // Coast in gear with the pedal released (fuel cut), then press the clutch.
    vehicle(&mut sim, 2, 1.0, 0.0);
    run(&mut sim, 8.0);
    vehicle(&mut sim, 2, 0.0, 0.0);
    let mut min_rpm = f32::MAX;
    for _ in 0..(6.0 / FRAME) as usize {
        sim.tick(FRAME);
        min_rpm = min_rpm.min(sim.telemetry().rpm);
    }
    assert_eq!(sim.condition(), EngineCondition::Running);
    assert!(min_rpm > 600.0, "dipped to {min_rpm}");
    let rpm = average(&mut sim, 3.0, |t| t.rpm);
    assert!((700.0..950.0).contains(&rpm), "idle after declutch {rpm}");
}

#[test]
fn stuck_wastegate_trips_the_latched_overboost_cut() {
    let mut sim = warm_turbo(24);
    sim.inject_fault(Fault::WastegateStuckClosed).unwrap();
    dyno(&mut sim, 4500.0, 1.0);
    run(&mut sim, 4.0);
    assert!(sim.dtcs().contains(DtcCode::P0234));
    // Fuel stays cut while the pedal is held, so boost collapses instead of cycling.
    let t = sim.telemetry();
    assert!(
        t.manifold_pressure_kpa < 200.0,
        "MAP {}",
        t.manifold_pressure_kpa
    );
    assert_eq!(t.injector_pulse_ms, 0.0);
    // Lifting off re-arms the fuel.
    sim.update_controls(|c| c.pedal = 0.1);
    run(&mut sim, 1.0);
    assert!(sim.telemetry().injector_pulse_ms > 0.0);
}

#[test]
fn overboost_warning_follows_hardware_not_the_editable_ecu_limit() {
    let mut sim = warm_turbo(25);
    sim.update_calibration(|c| c.boost.overboost_limit_kpa = 400.0)
        .unwrap();
    sim.inject_fault(Fault::WastegateStuckClosed).unwrap();
    dyno(&mut sim, 4500.0, 1.0);
    let mut warned = false;
    for _ in 0..(10.0 / FRAME) as usize {
        sim.tick(FRAME);
        warned |= sim.telemetry().warnings.contains(Warning::Overboost);
    }
    assert!(warned);
}

#[test]
fn healthy_engine_has_enough_oil_pressure_up_to_redline() {
    let mut sim = warm_na(26);
    dyno(&mut sim, 6700.0, 1.0);
    run(&mut sim, 10.0);
    let t = sim.telemetry();
    assert!(!t.warnings.contains(Warning::LowOilPressure));
    assert!(t.health.bearings > 0.999, "bearings {}", t.health.bearings);
}

#[test]
fn clear_dtcs_resets_codes_monitors_and_telemetry_at_once() {
    let mut sim = warm_na(27);
    // Learn a real long-term trim first (O2 sensor reading lean), with its lean code.
    sim.inject_fault(Fault::OxygenSensorBias { lambda: 0.3 })
        .unwrap();
    dyno(&mut sim, 2500.0, 0.2);
    run(&mut sim, 90.0);
    assert!(sim.telemetry().ltft_pct > 10.0);
    assert!(sim.dtcs().contains(DtcCode::P0171));
    sim.clear_faults();
    sim.clear_dtcs();
    // The snapshot reflects the scan-tool action without waiting for a tick.
    assert!(sim.dtcs().is_empty());
    assert_eq!(sim.telemetry().dtc_count, 0);
    assert!(!sim.telemetry().warnings.contains(Warning::CheckEngine));
    assert_eq!(sim.telemetry().ltft_pct, 0.0);
    assert_eq!(sim.telemetry().stft_pct, 0.0);
    // With the fault gone, no code comes back.
    run(&mut sim, 30.0);
    assert!(sim.dtcs().is_empty());
}

#[test]
fn reset_keeps_event_cursors_valid() {
    let mut sim = warm_na(28);
    run(&mut sim, 2.0);
    let log_cursor = sim.events().next_seq();
    let cyl_cursor = sim.cylinder_events().next_seq();
    sim.reset();
    assert_eq!(sim.events().since(0).count(), 0);
    start(&mut sim);
    run(&mut sim, 1.0);
    assert!(sim.events().since(log_cursor).count() > 0);
    assert!(sim.cylinder_events().since(cyl_cursor).count() > 50);
    assert!(sim.events().next_seq() > log_cursor);
}

#[test]
fn repairing_a_running_engine_keeps_it_running() {
    let mut sim = warm_na(29);
    dyno(&mut sim, 4000.0, 0.5);
    run(&mut sim, 3.0);
    let rpm = sim.telemetry().rpm;
    let events = sim.events().next_seq();
    sim.repair();
    assert_eq!(sim.condition(), EngineCondition::Running);
    assert_eq!(
        sim.events().next_seq(),
        events,
        "no spurious condition change"
    );
    run(&mut sim, 0.2);
    assert!((sim.telemetry().rpm - rpm).abs() < 200.0);
}

#[test]
fn stopped_engine_reports_no_live_ecu_values() {
    let mut sim = warm_na(30);
    run(&mut sim, 30.0);
    assert!(sim.telemetry().closed_loop);
    // Stall it with the key on.
    dyno(&mut sim, 300.0, 0.0);
    run(&mut sim, 4.0);
    let t = sim.telemetry();
    assert_eq!(t.condition, EngineCondition::Stalled);
    assert!(!t.closed_loop);
    assert_eq!(t.stft_pct, 0.0);
    assert_eq!(t.ignition_advance_deg, 0.0);
}

#[test]
fn faults_must_be_applicable_and_detectable() {
    let mut sim = warm_na(31);
    assert_eq!(
        sim.inject_fault(Fault::BoostLeak { diameter_mm: 20.0 }),
        Err(SimError::Fault(FaultError::NotApplicable))
    );
    assert_eq!(
        sim.inject_fault(Fault::VacuumLeak { diameter_mm: 0.0 }),
        Err(SimError::Fault(FaultError::InvalidParameter))
    );
    assert!(!sim.is_fault_active(FaultId::BoostLeak));
}

#[test]
fn engine_autopsy_faults_leave_their_signatures() {
    // IAT open circuit → P0113.
    let mut sim = warm_na(32);
    sim.inject_fault(Fault::IntakeAirSensorOpen).unwrap();
    run(&mut sim, 5.0);
    assert!(sim.dtcs().contains(DtcCode::P0113));
    // O2 sensor reading lean → the trims add fuel until the lean code sets.
    let mut sim = warm_na(33);
    sim.inject_fault(Fault::OxygenSensorBias { lambda: 0.3 })
        .unwrap();
    dyno(&mut sim, 2500.0, 0.2);
    run(&mut sim, 120.0);
    assert!(sim.dtcs().contains(DtcCode::P0171));
    assert!(sim.telemetry().ltft_pct > 10.0);
    // Leaking cylinder 2 → its misfire code only.
    let mut sim = warm_na(34);
    sim.inject_fault(Fault::LowCompression {
        cylinder: 1,
        leak_fraction: 0.6,
    })
    .unwrap();
    dyno(&mut sim, 2500.0, 0.5);
    run(&mut sim, 15.0);
    assert_eq!(sim.dtcs().iter().collect::<Vec<_>>(), vec![DtcCode::P0302]);
    // Restricted exhaust → back-pressure up, torque down.
    let mut healthy = warm_na(35);
    let mut blocked = warm_na(35);
    blocked
        .inject_fault(Fault::ExhaustRestriction { factor: 8.0 })
        .unwrap();
    for sim in [&mut healthy, &mut blocked] {
        dyno(sim, 4000.0, 1.0);
        run(sim, 5.0);
    }
    let (h, b) = (healthy.telemetry(), blocked.telemetry());
    assert!(b.exhaust_pressure_kpa > h.exhaust_pressure_kpa + 30.0);
    assert!(b.brake_torque_nm < 0.92 * h.brake_torque_nm);
}

#[test]
fn warnings_and_events_do_not_depend_on_frame_rate() {
    let record = |frame: f32| {
        let mut sim = warm_na(36);
        sim.update_controls(|c| c.pedal = 1.0);
        let frames = (3.0 / frame).round() as usize;
        for _ in 0..frames {
            sim.tick(frame);
        }
        sim.events()
            .iter()
            .map(|e| (e.seq, e.time_s, e.kind))
            .collect::<Vec<_>>()
    };
    let slow = record(1.0 / 30.0);
    let fast = record(1.0 / 240.0);
    assert!(slow.iter().any(|e| matches!(e.2, EventKind::Warning(_))));
    assert_eq!(slow, fast);
}

#[test]
fn status_key_renders_through_any_localizer() {
    use tunerlab_core::engine_sim::i18n::{Localizer, StaticLocalizer};
    let sim = EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 37).unwrap();
    assert_eq!(
        sim.status_key(),
        MessageKey::Condition(EngineCondition::Off)
    );
    let pl = StaticLocalizer {
        language: Language::Pl,
    };
    assert_eq!(pl.text(sim.status_key()), sim.status_text(Language::Pl));
}

#[test]
fn idle_control_holds_creep_speed_in_first_gear() {
    let mut sim = warm_na(40);
    drive_away(&mut sim);
    // Pedal released, clutch up: the idle controller sets the creep speed.
    vehicle(&mut sim, 1, 1.0, 0.0);
    run(&mut sim, 15.0);
    let rpm = average(&mut sim, 5.0, |t| t.rpm);
    assert!((700.0..950.0).contains(&rpm), "creep idle {rpm}");
    assert_eq!(sim.condition(), EngineCondition::Running);
}

#[test]
fn closed_throttle_overrun_does_not_set_a_map_circuit_code() {
    // Pedal released (fuel cut, dashpot) or barely touched (fuelled, throttle almost shut):
    // the manifold falls far below idle vacuum, which a healthy sensor still reads.
    for pedal in [0.0, 0.012, 0.02] {
        let mut sim = warm_na(41);
        drive_away(&mut sim);
        vehicle(&mut sim, 2, 1.0, 1.0);
        while sim.telemetry().rpm < 6000.0 {
            sim.tick(FRAME);
        }
        vehicle(&mut sim, 2, 1.0, pedal);
        run(&mut sim, 12.0);
        let codes: Vec<_> = sim.dtcs().iter().collect();
        assert!(
            !sim.dtcs().contains(DtcCode::P0106),
            "pedal {pedal}: {codes:?}"
        );
        assert!(
            !sim.dtcs().contains(DtcCode::P0107),
            "pedal {pedal}: {codes:?}"
        );
    }
}

#[test]
fn map_sensor_faults_set_their_own_codes() {
    // A MAP sensor reading 20 kPa high cannot drag the baro estimate along at WOT.
    let mut sim = warm_na(50);
    sim.inject_fault(Fault::MapSensorBias { kpa: 20.0 })
        .unwrap();
    dyno(&mut sim, 2000.0, 1.0);
    run(&mut sim, 10.0);
    assert!(sim.dtcs().contains(DtcCode::P0106));
    // A sensor stuck at the low rail is a circuit fault, not a range/performance one,
    // whether it fails while running or before the key is turned.
    let mut sim = warm_na(51);
    sim.inject_fault(Fault::MapSensorBias { kpa: -100.0 })
        .unwrap();
    dyno(&mut sim, 2500.0, 0.3);
    run(&mut sim, 4.0);
    assert!(sim.dtcs().contains(DtcCode::P0107));
    for spec in [
        EngineSpec::naturally_aspirated_2l(),
        EngineSpec::turbocharged_2l(),
    ] {
        let mut sim = EngineSim::with_base_calibration(spec, 59).unwrap();
        sim.inject_fault(Fault::MapSensorBias { kpa: -100.0 })
            .unwrap();
        sim.update_controls(|c| c.ignition_on = true);
        run(&mut sim, 6.0);
        let codes: Vec<_> = sim.dtcs().iter().collect();
        assert_eq!(codes, vec![DtcCode::P0107]);
    }
}

/// A cylinder event's TDC prediction error: (rpm, error in crank degrees between the
/// predicted firing TDC and the half revolution the crank actually turned, event time
/// relative to the start of the recording \[s\]).
type TdcError = (f32, f64, f64);

/// Ticks at the physics step for `seconds`, applying `ctl` before every step, and returns
/// the TDC prediction error of every cylinder event in that time.
fn tdc_prediction_errors(
    sim: &mut EngineSim,
    seconds: f32,
    mut ctl: impl FnMut(&mut EngineSim, f64),
) -> Vec<TdcError> {
    use tunerlab_core::engine_sim::events::CYLINDER_EVENT_CAPACITY;
    use tunerlab_core::engine_sim::SUBSTEP_S;
    // Record the unwrapped crank angle at every physics step, and drain the event ring as
    // it fills so no event is overwritten before it is checked.
    let mut trace: Vec<(f64, f64)> = vec![(sim.time_s(), 0.0)];
    let mut events = Vec::new();
    let mut unwrapped = 0.0_f64;
    let mut last = f64::from(sim.telemetry().crank_angle_deg);
    let mut next = sim.cylinder_events().next_seq();
    let t0 = sim.time_s();
    for _ in 0..(seconds / SUBSTEP_S).round() as usize {
        ctl(sim, sim.time_s() - t0);
        sim.tick(SUBSTEP_S);
        let a = f64::from(sim.telemetry().crank_angle_deg);
        let mut d = a - last;
        if d < -360.0 {
            d += 720.0;
        }
        unwrapped += d;
        last = a;
        trace.push((sim.time_s(), unwrapped));
        let ring = sim.cylinder_events();
        assert!(ring.next_seq() - next <= CYLINDER_EVENT_CAPACITY as u64);
        events.extend(ring.since(next).copied());
        next = ring.next_seq();
    }
    let angle_at = |t: f64| -> Option<f64> {
        let i = trace.iter().position(|(tt, _)| *tt >= t)?;
        if i == 0 {
            return None;
        }
        let (t0, a0) = trace[i - 1];
        let (t1, a1) = trace[i];
        Some(a0 + (a1 - a0) * (t - t0) / (t1 - t0))
    };
    events
        .iter()
        .filter_map(|ev| {
            let tdc = ev.time_s + f64::from(ev.time_to_tdc_s);
            let (a0, a1) = (angle_at(ev.time_s)?, angle_at(tdc)?);
            Some((ev.rpm, a1 - a0 - 180.0, ev.time_s - t0))
        })
        .collect()
}

fn mean_and_worst(errors: &[TdcError]) -> (f64, f64) {
    let mean = errors.iter().map(|e| e.1.abs()).sum::<f64>() / errors.len() as f64;
    let worst = errors.iter().map(|e| e.1.abs()).fold(0.0, f64::max);
    (mean, worst)
}

#[test]
fn audio_event_timing_predicts_firing_tdc() {
    // Steady running: warm idle, and a low-speed dyno pull with a strong firing ripple.
    for (pedal, rpm) in [(0.0, None), (0.6, Some(850.0))] {
        let mut sim = warm_na(52);
        if let Some(r) = rpm {
            dyno(&mut sim, r, pedal);
        }
        run(&mut sim, 6.0);
        let errors = tdc_prediction_errors(&mut sim, 2.0, |_, _| {});
        let (_, worst) = mean_and_worst(&errors);
        assert!(errors.len() > 20, "{} events checked", errors.len());
        assert!(
            worst < 2.0,
            "TDC prediction off by {worst:.1}° (pedal {pedal})"
        );
    }

    // Transients, the most audible events: a WOT blip in neutral and a start flare.
    for seed in [50, 52, 54] {
        let mut sim = warm_na(seed);
        run(&mut sim, 5.0);
        let errors = tdc_prediction_errors(&mut sim, 1.2, |s, t| {
            s.update_controls(|c| c.pedal = if t < 0.35 { 1.0 } else { 0.0 });
        });
        let (mean, _) = mean_and_worst(&errors);
        assert!(errors.len() > 128, "{} blip events", errors.len());
        assert!(mean < 1.0, "blip mean {mean:.1}° (seed {seed})");
        // A past-only predictor cannot foresee the throttle step itself: the first
        // ≈ 0.15 s of the blip are allowed a larger error, the rest must be tight.
        for &(rpm, err, t) in &errors {
            let limit = if t < 0.15 { 60.0 } else { 2.5 };
            assert!(
                err.abs() < limit,
                "blip seed {seed}: {err:.1}° at {t:.3} s, {rpm:.0} rpm"
            );
        }
    }

    let mut sim =
        EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 55).unwrap();
    sim.prewarm();
    sim.update_controls(|c| {
        c.ignition_on = true;
        c.starter = true;
    });
    let errors = tdc_prediction_errors(&mut sim, 1.5, |s, t| {
        if t >= 1.0 {
            s.update_controls(|c| c.starter = false);
        }
    });
    assert_eq!(sim.condition(), EngineCondition::Running);
    // The first firing cycles cannot be foreseen from the cranking speed; once the flare
    // is under way the timed acceleration must carry the prediction.
    let flare: Vec<_> = errors.iter().copied().filter(|e| e.0 > 1000.0).collect();
    let (mean, worst) = mean_and_worst(&flare);
    assert!(flare.len() > 20, "{} flare events", flare.len());
    assert!(
        mean < 2.0 && worst < 5.0,
        "flare: mean {mean:.1}°, worst {worst:.1}°"
    );
}

#[test]
fn audio_event_timing_stays_sane_at_start_and_stop() {
    use tunerlab_core::engine_sim::SUBSTEP_S;
    // From a fresh simulation the crank starts exactly on a cylinder event, and at the end
    // of a spin-down the last events never reach their TDC: neither may schedule sound
    // seconds away.
    let mut sim =
        EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 3).unwrap();
    sim.prewarm();
    sim.update_controls(|c| {
        c.ignition_on = true;
        c.starter = true;
    });
    let mut next = sim.cylinder_events().next_seq();
    let mut worst = 0.0_f32;
    let check = |sim: &EngineSim, next: &mut u64, worst: &mut f32| {
        let ring = sim.cylinder_events();
        for ev in ring.since(*next) {
            *worst = worst.max(ev.time_to_tdc_s).max(ev.time_to_evo_s * 0.5);
        }
        *next = ring.next_seq();
    };
    for _ in 0..(1.5 / SUBSTEP_S) as usize {
        sim.tick(SUBSTEP_S);
        check(&sim, &mut next, &mut worst);
    }
    sim.update_controls(|c| {
        c.starter = false;
        c.ignition_on = false;
    });
    let mut stopped_at = None;
    for _ in 0..(3.0 / SUBSTEP_S) as usize {
        sim.tick(SUBSTEP_S);
        check(&sim, &mut next, &mut worst);
        if stopped_at.is_none() && sim.telemetry().rpm == 0.0 {
            stopped_at = Some(sim.time_s());
        }
    }
    assert!(stopped_at.is_some(), "engine did not stop");
    // Slowest real half revolution while cranking ≈ 0.2 s.
    assert!(worst < 0.5, "an event predicted TDC {worst:.3} s away");
}

#[test]
fn engine_restarts_after_an_overboost_cut_stall() {
    let mut sim = warm_turbo(42);
    sim.inject_fault(Fault::WastegateStuckClosed).unwrap();
    dyno(&mut sim, 4500.0, 1.0);
    // The latched cut holds while the pedal stays down until the dyno stalls the engine.
    for _ in 0..(20.0 / FRAME) as usize {
        sim.tick(FRAME);
        if sim.telemetry().rpm < 1.0 {
            break;
        }
    }
    assert!(sim.dtcs().contains(DtcCode::P0234));
    sim.clear_faults();
    sim.update_controls(|c| {
        c.load = LoadModel::Neutral;
        c.pedal = 0.3;
    });
    start(&mut sim);
    run(&mut sim, 3.0);
    assert_eq!(sim.condition(), EngineCondition::Running);
    // Fuelled (the rev limiter may cut individual samples once it free-revs).
    let pw = average(&mut sim, 1.0, |t| t.injector_pulse_ms);
    assert!(pw > 1.0, "mean pulse {pw} ms");
}

#[test]
fn map_plausibility_follows_the_barometric_sensor() {
    // Key on at altitude, then drive down to sea level with the key off.
    let mut sim = warm_na(43);
    sim.update_controls(|c| {
        c.ignition_on = false;
        c.ambient.pressure_pa = 70_000.0;
    });
    run(&mut sim, 20.0);
    sim.update_controls(|c| c.ignition_on = true);
    run(&mut sim, 3.0);
    sim.update_controls(|c| c.ignition_on = false);
    run(&mut sim, 2.0);
    sim.update_controls(|c| c.ambient.pressure_pa = 101_325.0);
    run(&mut sim, 20.0);
    start(&mut sim);
    dyno(&mut sim, 4000.0, 1.0);
    run(&mut sim, 6.0);
    assert!(!sim.dtcs().contains(DtcCode::P0106));

    // Quick restarts while the manifold is still refilling after the stop.
    for off_s in [0.7, 0.75, 0.8] {
        let mut sim = warm_na(56);
        run(&mut sim, 10.0);
        sim.update_controls(|c| c.ignition_on = false);
        run(&mut sim, off_s);
        start(&mut sim);
        dyno(&mut sim, 2000.0, 1.0);
        run(&mut sim, 10.0);
        let codes: Vec<_> = sim.dtcs().iter().collect();
        assert!(codes.is_empty(), "key off {off_s} s: {codes:?}");
    }
}

#[test]
fn driving_down_a_mountain_sets_no_map_code() {
    let mut sim =
        EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 57).unwrap();
    sim.update_controls(|c| c.ambient.pressure_pa = 80_000.0);
    sim.prewarm();
    run(&mut sim, 5.0);
    start(&mut sim);
    // Ten minutes of part-throttle descent (≈ 1900 m), then the first WOT at low speed.
    dyno(&mut sim, 2500.0, 0.3);
    for i in 1..=600 {
        sim.update_controls(|c| c.ambient.pressure_pa = 80_000.0 + 21_325.0 * i as f32 / 600.0);
        run(&mut sim, 1.0);
    }
    dyno(&mut sim, 2000.0, 1.0);
    run(&mut sim, 15.0);
    // An ambient step while idling (pressure changed from the UI), then WOT.
    dyno(&mut sim, 900.0, 0.0);
    sim.update_controls(|c| {
        c.load = LoadModel::Neutral;
        c.ambient.pressure_pa = 85_000.0;
    });
    run(&mut sim, 20.0);
    dyno(&mut sim, 2000.0, 1.0);
    run(&mut sim, 15.0);
    let codes: Vec<_> = sim.dtcs().iter().collect();
    assert!(!sim.dtcs().contains(DtcCode::P0106), "{codes:?}");
}

#[test]
fn map_sensor_reading_low_with_the_engine_stopped_sets_p0106() {
    // Key on, engine off: the manifold sits at barometric pressure, the sensor reads 20 kPa
    // under it.
    let mut sim =
        EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 58).unwrap();
    sim.inject_fault(Fault::MapSensorBias { kpa: -20.0 })
        .unwrap();
    sim.update_controls(|c| c.ignition_on = true);
    run(&mut sim, 6.0);
    assert!(sim.dtcs().contains(DtcCode::P0106));
}

#[test]
fn misfires_are_counted_with_the_clutch_held_down() {
    let mut sim = warm_na(44);
    sim.inject_fault(Fault::LowCompression {
        cylinder: 1,
        leak_fraction: 0.85,
    })
    .unwrap();
    // Waiting at a light in first gear with the clutch pedal on the floor.
    vehicle(&mut sim, 1, 0.0, 0.0);
    run(&mut sim, 30.0);
    assert!(sim.dtcs().contains(DtcCode::P0302));
}

#[test]
fn clearing_codes_waits_for_a_cold_start_to_rerun_the_thermostat_monitor() {
    let mut sim =
        EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 45).unwrap();
    sim.inject_fault(Fault::ThermostatStuckOpen).unwrap();
    start(&mut sim);
    dyno(&mut sim, 2500.0, 0.3);
    run(&mut sim, 240.0);
    assert!(sim.dtcs().contains(DtcCode::P0128));
    sim.clear_dtcs();
    run(&mut sim, 30.0);
    assert!(!sim.dtcs().contains(DtcCode::P0128));
}

#[test]
fn warnings_near_a_threshold_do_not_flood_the_event_log() {
    let mut sim = warm_na(46);
    let redline = sim.spec().limits.redline_rpm;
    dyno(&mut sim, redline, 1.0);
    run(&mut sim, 3.0);
    let cursor = sim.events().next_seq();
    run(&mut sim, 2.0);
    let logged = sim.events().since(cursor).count();
    assert!(logged < 20, "{logged} events in 2 s at the redline");
}

#[test]
fn warm_idle_regulates_on_the_thermostat() {
    // Block and sump losses must not overwhelm idle heat even in a frost.
    for ambient_c in [25.0, 0.0, -25.0] {
        let mut sim =
            EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 47).unwrap();
        sim.update_controls(|c| c.ambient.temperature_k = 273.15 + ambient_c);
        sim.prewarm();
        start(&mut sim);
        run(&mut sim, 300.0);
        let t = sim.telemetry();
        assert!(
            (85.0..100.0).contains(&t.coolant_temp_c),
            "coolant {} at {ambient_c} °C",
            t.coolant_temp_c
        );
        assert!(
            t.thermostat_pos > 0.0,
            "thermostat {} at {ambient_c} °C",
            t.thermostat_pos
        );
    }
}

#[test]
fn catalyst_lights_off_at_cold_idle_and_passes_its_monitor() {
    let mut sim =
        EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 48).unwrap();
    start(&mut sim);
    run(&mut sim, 180.0);
    assert!(
        sim.telemetry().catalyst_temp_c > 300.0,
        "catalyst {}",
        sim.telemetry().catalyst_temp_c
    );
    // Warm restart after a short stop (the catalyst cools faster than the coolant).
    sim.update_controls(|c| c.ignition_on = false);
    run(&mut sim, 600.0);
    start(&mut sim);
    run(&mut sim, 400.0);
    assert!(!sim.dtcs().contains(DtcCode::P0420));
}

#[test]
fn stalled_engine_in_gear_carries_no_phantom_load() {
    let mut sim = warm_na(49);
    sim.update_controls(|c| {
        c.load = LoadModel::Vehicle(VehicleParams {
            gear: 1,
            clutch: 1.0,
            brake: 1.0,
            ..VehicleParams::default()
        });
    });
    run(&mut sim, 5.0);
    assert_eq!(sim.condition(), EngineCondition::Stalled);
    let t = sim.telemetry();
    assert!(t.load_torque_nm.abs() < 1.0, "load {}", t.load_torque_nm);
    // Releasing the brake must not launch the car or spin the dead engine.
    sim.update_controls(|c| {
        c.load = LoadModel::Vehicle(VehicleParams {
            gear: 1,
            clutch: 1.0,
            brake: 0.0,
            ..VehicleParams::default()
        });
    });
    run(&mut sim, 2.0);
    assert!(sim.telemetry().vehicle_speed_kph < 0.2);
    assert!(sim.telemetry().rpm < 20.0);
}

#[test]
fn sitting_on_the_rev_limiter_keeps_trouble_codes_in_the_event_log() {
    let mut sim = warm_na(53);
    sim.inject_fault(Fault::TimingChainStretch { retard_deg: 8.0 })
        .unwrap();
    run(&mut sim, 4.0);
    assert!(sim.dtcs().contains(DtcCode::P0016));
    sim.update_controls(|c| c.pedal = 1.0);
    run(&mut sim, 2.0);
    let cursor = sim.events().next_seq();
    run(&mut sim, 5.0);
    let warnings = sim
        .events()
        .since(cursor)
        .filter(|e| matches!(e.kind, EventKind::Warning(_)))
        .count();
    // Limiter cycling must not re-log LeanUnderLoad / InjectorDutyHigh every cut.
    assert!(
        warnings <= 4,
        "{warnings} warning events in 5 s on the limiter"
    );
    assert!(sim
        .events()
        .iter()
        .any(|e| matches!(e.kind, EventKind::Dtc(DtcCode::P0016))));
}

fn vehicle_with(sim: &mut EngineSim, vp: VehicleParams, pedal: f32) {
    sim.update_controls(|c| {
        c.pedal = pedal;
        c.load = LoadModel::Vehicle(vp);
    });
}

/// Pulls away and shifts up to `gear` at 3200 rpm, using the clutch on every change.
fn shift_up_to(sim: &mut EngineSim, gear: u8, base: VehicleParams) {
    for i in 0..=120 {
        let clutch = i as f32 / 120.0;
        vehicle_with(
            sim,
            VehicleParams {
                gear: 1,
                clutch,
                ..base
            },
            0.35,
        );
        sim.tick(FRAME);
    }
    run(sim, 2.0);
    for g in 2..=gear {
        vehicle_with(
            sim,
            VehicleParams {
                gear: g - 1,
                clutch: 1.0,
                ..base
            },
            0.6,
        );
        let mut t = 0.0;
        while sim.telemetry().rpm < 3200.0 && t < 15.0 {
            sim.tick(FRAME);
            t += FRAME;
        }
        vehicle_with(
            sim,
            VehicleParams {
                gear: g - 1,
                clutch: 0.0,
                ..base
            },
            0.0,
        );
        run(sim, 0.3);
        for i in 0..=30 {
            let clutch = i as f32 / 30.0;
            vehicle_with(
                sim,
                VehicleParams {
                    gear: g,
                    clutch,
                    ..base
                },
                0.3,
            );
            sim.tick(FRAME);
        }
    }
}

#[test]
fn declutching_after_a_long_coast_just_above_idle_does_not_stall() {
    let mut sim = warm_na(60);
    // A gentle downhill holds the engine just above its idle target in fourth.
    let base = VehicleParams {
        grade_rad: -0.004,
        ..VehicleParams::default()
    };
    shift_up_to(&mut sim, 4, base);
    vehicle_with(
        &mut sim,
        VehicleParams {
            gear: 4,
            clutch: 1.0,
            ..base
        },
        0.0,
    );
    run(&mut sim, 45.0);
    vehicle_with(
        &mut sim,
        VehicleParams {
            gear: 4,
            clutch: 0.0,
            ..base
        },
        0.0,
    );
    let mut min_rpm = f32::MAX;
    for _ in 0..(6.0 / FRAME) as usize {
        sim.tick(FRAME);
        min_rpm = min_rpm.min(sim.telemetry().rpm);
    }
    assert_eq!(sim.condition(), EngineCondition::Running);
    assert!(min_rpm > 600.0, "dipped to {min_rpm}");
    let codes: Vec<_> = sim.dtcs().iter().collect();
    assert!(codes.is_empty(), "{codes:?}");
}

#[test]
fn declutching_after_lugging_in_top_gear_does_not_flare() {
    let mut sim = warm_na(91);
    shift_up_to(&mut sim, 6, VehicleParams::default());
    // Pedal released in sixth: the wheels drag the engine below its idle target.
    vehicle(&mut sim, 6, 1.0, 0.0);
    run(&mut sim, 90.0);
    // Declutch, select neutral and brake to a stop.
    vehicle_with(
        &mut sim,
        VehicleParams {
            gear: 0,
            clutch: 0.0,
            brake: 0.8,
            ..VehicleParams::default()
        },
        0.0,
    );
    run(&mut sim, 10.0);
    let mut max_rpm = 0.0_f32;
    for _ in 0..(20.0 / FRAME) as usize {
        sim.tick(FRAME);
        max_rpm = max_rpm.max(sim.telemetry().rpm);
    }
    assert_eq!(sim.condition(), EngineCondition::Running);
    assert!(max_rpm < 1100.0, "idle flared to {max_rpm}");
    // Neither idle speed high (P0507) nor a MAP circuit code from a valve hunting shut.
    let codes: Vec<_> = sim.dtcs().iter().collect();
    assert!(codes.is_empty(), "{codes:?}");
}

#[test]
fn lean_engine_on_the_rev_limiter_still_warns() {
    let mut sim = warm_na(62);
    for cylinder in 0..4 {
        sim.inject_fault(Fault::InjectorClogged {
            cylinder,
            flow_fraction: 0.6,
        })
        .unwrap();
    }
    dyno(&mut sim, 8000.0, 1.0);
    run(&mut sim, 8.0);
    let cut = sim.calibration().limiter.cut_rpm;
    let (mut on_limiter, mut lean) = (0, 0);
    for _ in 0..(5.0 / FRAME) as usize {
        sim.tick(FRAME);
        let t = sim.telemetry();
        if t.rpm > cut - 500.0 {
            on_limiter += 1;
            if t.warnings.contains(Warning::LeanUnderLoad) {
                lean += 1;
            }
        }
    }
    assert!(on_limiter > 200, "{on_limiter} frames on the limiter");
    // The rev cut's own lean gas is blanked for a few cycles, not for the whole session.
    assert!(
        lean * 5 > on_limiter,
        "lean in {lean} of {on_limiter} frames"
    );
}

#[test]
fn turbo_survives_sustained_full_load_with_its_oil_cooler() {
    let mut sim = warm_turbo(63);
    dyno(&mut sim, 5500.0, 1.0);
    // The oil settles within two minutes; five cover it with margin.
    for _ in 0..(300.0 / FRAME) as usize {
        sim.tick(FRAME);
        let t = sim.telemetry();
        assert!(
            !t.warnings.contains(Warning::HighOilTemp),
            "oil {}",
            t.oil_temp_c
        );
        assert!(
            !t.warnings.contains(Warning::LowOilPressure),
            "oil {} kPa",
            t.oil_pressure_kpa
        );
    }
    let t = sim.telemetry();
    assert_eq!(sim.condition(), EngineCondition::Running);
    assert!(t.health.bearings > 0.999, "bearings {}", t.health.bearings);
    assert!(
        (100.0..130.0).contains(&t.oil_temp_c),
        "oil {}",
        t.oil_temp_c
    );
}

#[test]
fn car_parked_in_gear_is_held_by_compression() {
    let mut sim =
        EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 64).unwrap();
    sim.prewarm();
    // Ignition off, first gear, clutch up, on an 8.5° downhill; then the brake is released.
    let parked = VehicleParams {
        gear: 1,
        clutch: 1.0,
        brake: 1.0,
        grade_rad: -0.15,
        ..VehicleParams::default()
    };
    vehicle_with(&mut sim, parked, 0.0);
    run(&mut sim, 2.0);
    vehicle_with(
        &mut sim,
        VehicleParams {
            brake: 0.0,
            ..parked
        },
        0.0,
    );
    let mut travelled = 0.0_f32;
    let mut travelled_at_10s = 0.0_f32;
    for i in 0..(30.0 / FRAME) as usize {
        sim.tick(FRAME);
        travelled += sim.telemetry().vehicle_speed_kph / 3.6 * FRAME;
        if i == (10.0 / FRAME) as usize {
            travelled_at_10s = travelled;
        }
    }
    // The car settles against a compression stroke within a few centimetres...
    assert!(travelled < 0.3, "rolled {travelled} m");
    // ...and stays there.
    assert!(
        travelled - travelled_at_10s < 1.0e-3,
        "crept {} m",
        travelled - travelled_at_10s
    );
}

#[test]
fn starter_stalls_against_a_braked_car_in_gear() {
    let mut sim =
        EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 65).unwrap();
    sim.prewarm();
    vehicle_with(
        &mut sim,
        VehicleParams {
            gear: 1,
            clutch: 1.0,
            brake: 1.0,
            ..VehicleParams::default()
        },
        0.0,
    );
    sim.update_controls(|c| {
        c.ignition_on = true;
        c.starter = true;
    });
    run(&mut sim, 2.0);
    let mut revs = 0.0_f32;
    for _ in 0..(8.0 / FRAME) as usize {
        sim.tick(FRAME);
        revs += sim.telemetry().rpm / 60.0 * FRAME;
    }
    assert!(revs < 0.01, "crank crept {revs} rev against the brake");
    assert!(sim.telemetry().vehicle_speed_kph < 0.01);
}

#[test]
fn healthy_engine_does_not_warn_lean_after_fuel_cuts() {
    // On the rev limiter, and on every tip-in after an overrun fuel cut.
    for spec in [
        EngineSpec::naturally_aspirated_2l(),
        EngineSpec::turbocharged_2l(),
    ] {
        let mut sim = EngineSim::with_base_calibration(spec, 62).unwrap();
        sim.prewarm();
        start(&mut sim);
        let cursor = sim.events().next_seq();
        dyno(&mut sim, 9000.0, 1.0);
        run(&mut sim, 10.0);
        vehicle(&mut sim, 0, 0.0, 0.0);
        run(&mut sim, 3.0);
        shift_up_to(&mut sim, 3, VehicleParams::default());
        for _ in 0..5 {
            vehicle(&mut sim, 3, 1.0, 1.0);
            run(&mut sim, 3.0);
            vehicle(&mut sim, 3, 1.0, 0.0);
            run(&mut sim, 2.5);
        }
        let lean: Vec<_> = sim
            .events()
            .since(cursor)
            .filter(|e| {
                matches!(
                    e.kind,
                    EventKind::Warning(Warning::LeanUnderLoad)
                        | EventKind::Warning(Warning::InjectorDutyHigh)
                )
            })
            .map(|e| e.time_s)
            .collect();
        assert!(lean.is_empty(), "mixture warnings at {lean:?}");
    }
}

#[test]
fn turbo_stall_from_boost_sets_no_map_code() {
    // The dyno stalls the engine at full boost with the key on: the still-spinning
    // compressor holds the manifold above ambient for many seconds while it runs down.
    let mut sim = warm_turbo(87);
    dyno(&mut sim, 3500.0, 1.0);
    run(&mut sim, 6.0);
    dyno(&mut sim, 0.0, 1.0);
    run(&mut sim, 45.0);
    assert_ne!(sim.condition(), EngineCondition::Running);
    let codes: Vec<_> = sim.dtcs().iter().collect();
    assert!(codes.is_empty(), "dyno stall: {codes:?}");
    // Key cycled while the manifold is still pressurised.
    let mut sim = warm_turbo(88);
    dyno(&mut sim, 3500.0, 1.0);
    run(&mut sim, 6.0);
    sim.update_controls(|c| c.ignition_on = false);
    run(&mut sim, 2.5);
    sim.update_controls(|c| c.ignition_on = true);
    run(&mut sim, 30.0);
    let codes: Vec<_> = sim.dtcs().iter().collect();
    assert!(codes.is_empty(), "key cycle: {codes:?}");
}

#[test]
fn ambient_step_with_the_key_on_sets_no_map_code() {
    // The filtered BARO sensor lags a full-range ambient change that an open throttle
    // passes straight to the manifold.
    for (from, to) in [(50_000.0, 110_000.0), (110_000.0, 50_000.0)] {
        let mut sim =
            EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 99).unwrap();
        sim.update_controls(|c| c.ambient.pressure_pa = from);
        sim.reset();
        sim.update_controls(|c| {
            c.ignition_on = true;
            c.pedal = 1.0;
        });
        run(&mut sim, 8.0);
        sim.update_controls(|c| c.ambient.pressure_pa = to);
        run(&mut sim, 10.0);
        let codes: Vec<_> = sim.dtcs().iter().collect();
        assert!(codes.is_empty(), "{from} -> {to} Pa: {codes:?}");
    }
}

#[test]
fn declutching_after_a_long_cold_crawl_in_gear_keeps_idling() {
    // Cold start at −20 °C, then ten minutes rolling in third with the pedal released while
    // the idle air schedule warms up; then neutral.
    let mut sim =
        EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 73).unwrap();
    sim.update_controls(|c| c.ambient.temperature_k = 273.15 - 20.0);
    sim.soak_to_ambient();
    start(&mut sim);
    run(&mut sim, 30.0);
    let base = VehicleParams {
        grade_rad: 0.02,
        ..VehicleParams::default()
    };
    shift_up_to(&mut sim, 3, base);
    vehicle_with(
        &mut sim,
        VehicleParams {
            gear: 3,
            clutch: 1.0,
            ..base
        },
        0.0,
    );
    run(&mut sim, 600.0);
    vehicle_with(
        &mut sim,
        VehicleParams {
            gear: 0,
            clutch: 0.0,
            brake: 1.0,
            ..base
        },
        0.0,
    );
    let mut min_rpm = f32::MAX;
    for _ in 0..(10.0 / FRAME) as usize {
        sim.tick(FRAME);
        min_rpm = min_rpm.min(sim.telemetry().rpm);
    }
    assert_eq!(sim.condition(), EngineCondition::Running);
    assert!(min_rpm > 450.0, "idle dipped to {min_rpm}");
    let codes: Vec<_> = sim.dtcs().iter().collect();
    assert!(codes.is_empty(), "{codes:?}");
}

#[test]
fn creeping_off_straight_after_a_start_holds_the_idle_target() {
    // Driving off before the idle integrator has learned anything in neutral.
    let mut sim = warm_na(101);
    drive_away(&mut sim);
    vehicle(&mut sim, 1, 1.0, 0.0);
    run(&mut sim, 20.0);
    let rpm = average(&mut sim, 10.0, |t| t.rpm);
    assert!((750.0..870.0).contains(&rpm), "creep at {rpm} rpm");
}

#[test]
fn stopped_engine_reports_no_indicated_torque() {
    let mut sim = warm_na(1);
    run(&mut sim, 3.0);
    sim.update_controls(|c| c.ignition_on = false);
    run(&mut sim, 30.0);
    let t = sim.telemetry();
    assert_eq!(t.rpm, 0.0);
    assert!(
        t.indicated_torque_nm.abs() < 0.5,
        "indicated {}",
        t.indicated_torque_nm
    );
    // Whatever the trapped charge still pushes, static friction holds.
    assert!(
        t.brake_torque_nm.abs() < 30.0,
        "brake {}",
        t.brake_torque_nm
    );
    // An overnight soak lets the trapped charge leak away.
    sim.soak_to_ambient();
    run(&mut sim, 1.0);
    let t = sim.telemetry();
    assert!(
        t.instantaneous_torque_nm.abs() < 0.5,
        "instantaneous {}",
        t.instantaneous_torque_nm
    );
}

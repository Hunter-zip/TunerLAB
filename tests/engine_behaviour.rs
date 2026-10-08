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
        capacity_fraction: 0.12,
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
    sim.inject_fault(Fault::IntakeAirSensorOpen).unwrap();
    run(&mut sim, 4.0);
    assert!(sim.dtcs().contains(DtcCode::P0113));
    sim.clear_faults();
    sim.clear_dtcs();
    // The snapshot reflects the scan-tool action without waiting for a tick.
    assert!(sim.dtcs().is_empty());
    assert_eq!(sim.telemetry().dtc_count, 0);
    assert!(!sim.telemetry().warnings.contains(Warning::CheckEngine));
    assert_eq!(sim.telemetry().ltft_pct, 0.0);
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

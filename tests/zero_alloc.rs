//! Verifies the real-time contract: `EngineSim::tick` never touches the heap.
//!
//! A counting global allocator wraps the system allocator. Counting is thread-local so the
//! test harness's own threads cannot pollute the measurement.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use tunerlab_core::engine_sim::controls::{DynoParams, LoadModel, VehicleParams};
use tunerlab_core::{EngineSim, EngineSpec, Fault, Language, Localize};

struct CountingAllocator;

thread_local! {
    static ALLOCATIONS: Cell<usize> = const { Cell::new(0) };
}

// SAFETY: forwards every call unchanged to the system allocator; the only addition is a
// thread-local counter increment, which neither allocates nor unwinds.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATIONS.with(|c| c.set(c.get() + 1));
        System.alloc(layout)
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        System.dealloc(ptr, layout)
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        ALLOCATIONS.with(|c| c.set(c.get() + 1));
        System.alloc_zeroed(layout)
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        ALLOCATIONS.with(|c| c.set(c.get() + 1));
        System.realloc(ptr, layout, new_size)
    }
}

#[global_allocator]
static GLOBAL: CountingAllocator = CountingAllocator;

fn allocations() -> usize {
    ALLOCATIONS.with(|c| c.get())
}

/// Runs `seconds` of simulation at a 60 Hz frame rate and returns the number of heap
/// allocations performed meanwhile (including telemetry/event reads a UI would do).
fn run_counted(sim: &mut EngineSim, seconds: f32) -> usize {
    let before = allocations();
    let frames = (seconds * 60.0) as usize;
    let mut sink = 0.0_f64;
    for _ in 0..frames {
        sim.tick(1.0 / 60.0);
        let t = sim.telemetry();
        sink += f64::from(t.rpm) + f64::from(t.brake_torque_nm);
        for e in sim.cylinder_events().since(0) {
            sink += f64::from(e.peak_pressure_bar);
        }
        for e in sim.events().since(0) {
            sink += e.time_s;
        }
        sink += sim.status_text(Language::Pl).len() as f64;
        for w in t.warnings.iter() {
            sink += w.localized(Language::En).len() as f64;
        }
    }
    assert!(sink.is_finite());
    allocations() - before
}

#[test]
fn tick_is_allocation_free_in_every_operating_regime() {
    // Positive control: the counting allocator must observe an ordinary allocation.
    let before = allocations();
    let probe: Vec<u64> = Vec::with_capacity(32);
    assert!(allocations() > before, "allocation counter is not wired up");
    drop(probe);

    for spec in [
        EngineSpec::naturally_aspirated_2l(),
        EngineSpec::turbocharged_2l(),
    ] {
        let mut sim = EngineSim::with_base_calibration(spec, 99).expect("valid preset");

        // Cold crank, start and fast idle.
        sim.update_controls(|c| {
            c.ignition_on = true;
            c.starter = true;
        });
        assert_eq!(run_counted(&mut sim, 1.5), 0, "cranking");
        sim.update_controls(|c| c.starter = false);
        assert_eq!(run_counted(&mut sim, 5.0), 0, "cold idle");

        // Full-load dyno pull with heavy knock (knock control off, +15° spark).
        sim.update_calibration(|cal| {
            cal.knock.enabled = false;
            for row in cal.ignition.values.iter_mut() {
                for v in row.iter_mut() {
                    *v += 15.0;
                }
            }
        })
        .expect("valid calibration");
        sim.update_controls(|c| {
            c.pedal = 1.0;
            c.load = LoadModel::Dyno(DynoParams {
                target_rpm: 3500.0,
                ..DynoParams::default()
            });
        });
        assert_eq!(run_counted(&mut sim, 5.0), 0, "knocking full load");

        // Hidden faults, misfires and DTC storage.
        sim.inject_fault(Fault::IgnitionCoilWeak {
            cylinder: 1,
            strength: 0.2,
        })
        .unwrap();
        sim.inject_fault(Fault::VacuumLeak { diameter_mm: 5.0 })
            .unwrap();
        assert_eq!(run_counted(&mut sim, 5.0), 0, "faults");

        // Vehicle drive-away, over-rev to destruction, failed engine.
        sim.update_controls(|c| {
            c.load = LoadModel::Vehicle(VehicleParams {
                gear: 1,
                ..VehicleParams::default()
            });
        });
        assert_eq!(run_counted(&mut sim, 3.0), 0, "vehicle");
        sim.update_calibration(|cal| cal.limiter.cut_rpm = 14_000.0)
            .unwrap();
        sim.update_controls(|c| c.load = LoadModel::Neutral);
        assert_eq!(run_counted(&mut sim, 10.0), 0, "over-rev / failure");

        // Key off and spin-down.
        sim.update_controls(|c| c.ignition_on = false);
        assert_eq!(run_counted(&mut sim, 3.0), 0, "key off");
    }
}

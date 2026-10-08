//! # TunerLAB core
//!
//! Real-time simulation core of the TunerLAB ECU-calibration training platform.
//!
//! Phase 1 delivers [`engine_sim`]: a physically based, allocation-free internal-combustion
//! engine model consisting of
//!
//! * a mean-value air path (compressible throttle flow, manifold filling/emptying,
//!   optional turbocharger with wastegate, blow-off valve and intercooler),
//! * a crank-angle-resolved closed-cycle combustion model per cylinder (slider-crank
//!   kinematics, Wiebe heat release, single-zone pressure integration, Livengood–Wu knock
//!   integral with the Douaud–Eyzat ignition-delay correlation),
//! * crankshaft, dynamometer and vehicle dynamics,
//! * lumped-capacitance thermal network (metal, coolant, oil, piston crowns, catalyst),
//! * a sensor layer that separates physical truth from what the ECU can measure,
//! * a full ECU emulation driven by user-editable calibration maps,
//! * a mechanical damage / failure model and hidden-fault injection ("Engine Autopsy"),
//! * PL/EN localisation hooks for every status, warning, DTC and failure message.
//!
//! The hot path ([`EngineSim::tick`]) performs no heap allocation, takes no locks and does
//! no I/O, so it can later be driven from a real-time audio thread.
//!
//! ## Quick start
//!
//! ```
//! use tunerlab_core::{EngineSim, EngineSpec, Language};
//!
//! let mut sim = EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 42)?;
//! sim.update_controls(|c| {
//!     c.ignition_on = true;
//!     c.starter = true;
//! });
//! for _ in 0..90 {
//!     sim.tick(1.0 / 60.0); // 1.5 s of cranking at a 60 Hz frame rate
//! }
//! sim.update_controls(|c| c.starter = false);
//! for _ in 0..120 {
//!     sim.tick(1.0 / 60.0);
//! }
//! let t = sim.telemetry();
//! assert!(t.rpm > 600.0);
//! println!("{}: {:.0} rpm, λ {:.2}", sim.status_text(Language::Pl), t.rpm, t.lambda);
//! # Ok::<(), tunerlab_core::SimError>(())
//! ```
#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod engine_sim;

pub use engine_sim::calibration::Calibration;
pub use engine_sim::controls::{Ambient, Controls, DynoParams, LoadModel, VehicleParams};
pub use engine_sim::dtc::DtcCode;
pub use engine_sim::faults::{Fault, FaultId};
pub use engine_sim::i18n::{Language, Localize, MessageKey};
pub use engine_sim::spec::EngineSpec;
pub use engine_sim::status::{EngineCondition, FailureCause, Warning, Warnings};
pub use engine_sim::telemetry::Telemetry;
pub use engine_sim::{EngineSim, SimError};

//! Engine Autopsy: a hidden fault is planted in the running engine and the student has to
//! name it from what the sensors, trims, codes and the event log show.

use std::time::{SystemTime, UNIX_EPOCH};

use tunerlab_core::{EngineSim, Fault, FaultId, Language, Localize};

/// xorshift64* pseudo-random generator; good enough to pick cases.
struct Rng(u64);

impl Rng {
    fn seeded() -> Self {
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0x9E37_79B9_7F4A_7C15);
        Self(nanos | 1)
    }

    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in `[lo, hi)`.
    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        let u = (self.next() >> 40) as f32 / (1u64 << 24) as f32;
        lo + (hi - lo) * u
    }

    fn below(&mut self, n: usize) -> usize {
        (self.next() % n.max(1) as u64) as usize
    }

    fn sign(&mut self) -> f32 {
        if self.next() & 1 == 0 {
            1.0
        } else {
            -1.0
        }
    }
}

/// Faults that can be planted in this engine.
pub fn candidates(turbocharged: bool) -> Vec<FaultId> {
    FaultId::ALL
        .iter()
        .copied()
        .filter(|id| {
            turbocharged || !matches!(id, FaultId::BoostLeak | FaultId::WastegateStuckClosed)
        })
        .collect()
}

/// A fault of kind `id` with a severity that leaves clear symptoms.
fn instance(id: FaultId, cylinders: usize, rng: &mut Rng) -> Fault {
    let cylinder = rng.below(cylinders) as u8;
    match id {
        FaultId::MapSensorBias => Fault::MapSensorBias {
            kpa: rng.sign() * rng.range(12.0, 25.0),
        },
        FaultId::CoolantSensorBias => Fault::CoolantSensorBias {
            kelvin: rng.sign() * rng.range(15.0, 30.0),
        },
        FaultId::IntakeAirSensorOpen => Fault::IntakeAirSensorOpen,
        FaultId::OxygenSensorSlow => Fault::OxygenSensorSlow {
            factor: rng.range(6.0, 15.0),
        },
        FaultId::OxygenSensorBias => Fault::OxygenSensorBias {
            lambda: rng.sign() * rng.range(0.06, 0.12),
        },
        FaultId::KnockSensorDead => Fault::KnockSensorDead,
        FaultId::VacuumLeak => Fault::VacuumLeak {
            diameter_mm: rng.range(5.0, 10.0),
        },
        FaultId::InjectorClogged => Fault::InjectorClogged {
            cylinder,
            flow_fraction: rng.range(0.4, 0.7),
        },
        FaultId::IgnitionCoilWeak => Fault::IgnitionCoilWeak {
            cylinder,
            strength: rng.range(0.25, 0.45),
        },
        FaultId::ThermostatStuckOpen => Fault::ThermostatStuckOpen,
        FaultId::ThermostatStuckClosed => Fault::ThermostatStuckClosed,
        FaultId::TimingChainStretch => Fault::TimingChainStretch {
            retard_deg: rng.range(6.0, 12.0),
        },
        FaultId::FuelPumpWeak => Fault::FuelPumpWeak {
            capacity_fraction: rng.range(0.3, 0.45),
        },
        FaultId::ExhaustRestriction => Fault::ExhaustRestriction {
            factor: rng.range(3.0, 8.0),
        },
        FaultId::LowCompression => Fault::LowCompression {
            cylinder,
            leak_fraction: rng.range(0.5, 0.85),
        },
        FaultId::BoostLeak => Fault::BoostLeak {
            diameter_mm: rng.range(8.0, 15.0),
        },
        FaultId::WastegateStuckClosed => Fault::WastegateStuckClosed,
    }
}

/// Outcome of the last answer.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    Correct,
    Wrong,
}

/// State of the current case.
pub struct Autopsy {
    rng: Rng,
    /// The planted fault; `None` when no case is open.
    pub fault: Option<Fault>,
    pub answer: Option<FaultId>,
    pub verdict: Option<Verdict>,
    pub attempts: u32,
    pub revealed: bool,
}

impl Default for Autopsy {
    fn default() -> Self {
        Self {
            rng: Rng::seeded(),
            fault: None,
            answer: None,
            verdict: None,
            attempts: 0,
            revealed: false,
        }
    }
}

impl Autopsy {
    /// Opens a new case: repairs the engine, clears codes and old faults, plants a new one.
    pub fn new_case(&mut self, sim: &mut EngineSim) {
        let turbo = sim.spec().turbo.is_some();
        let cylinders = sim.spec().geometry.cylinders;
        let pool = candidates(turbo);
        sim.clear_faults();
        sim.repair();
        sim.clear_dtcs();
        // A planted fault must be applicable; retry with a different kind if not.
        for _ in 0..8 {
            let id = pool[self.rng.below(pool.len())];
            let fault = instance(id, cylinders, &mut self.rng);
            if sim.inject_fault(fault).is_ok() {
                self.fault = Some(fault);
                self.answer = None;
                self.verdict = None;
                self.attempts = 0;
                self.revealed = false;
                return;
            }
        }
        self.fault = None;
    }

    /// Checks the selected answer.
    pub fn check(&mut self) {
        let (Some(fault), Some(answer)) = (self.fault, self.answer) else {
            return;
        };
        self.attempts += 1;
        self.verdict = Some(if fault.id() == answer {
            Verdict::Correct
        } else {
            Verdict::Wrong
        });
    }

    /// Forgets the case (the fault stays in the engine until the next case or a reset).
    pub fn close(&mut self) {
        self.fault = None;
        self.answer = None;
        self.verdict = None;
        self.attempts = 0;
        self.revealed = false;
    }
}

/// Name and severity of a planted fault, for the solution screen.
pub fn describe(fault: &Fault, lang: Language) -> String {
    let name = fault.id().localized(lang);
    let cyl = match lang {
        Language::En => "cylinder",
        Language::Pl => "cyl.",
    };
    let detail = match *fault {
        Fault::MapSensorBias { kpa } => format!("{kpa:+.0} kPa"),
        Fault::CoolantSensorBias { kelvin } => format!("{kelvin:+.0} K"),
        Fault::OxygenSensorSlow { factor } => format!("× {factor:.0}"),
        Fault::OxygenSensorBias { lambda } => format!("λ {lambda:+.2}"),
        Fault::VacuumLeak { diameter_mm } | Fault::BoostLeak { diameter_mm } => {
            format!("Ø {diameter_mm:.0} mm")
        }
        Fault::InjectorClogged {
            cylinder,
            flow_fraction,
        } => format!("{cyl} {}, {:.0} %", cylinder + 1, flow_fraction * 100.0),
        Fault::IgnitionCoilWeak { cylinder, strength } => {
            format!("{cyl} {}, {:.0} %", cylinder + 1, strength * 100.0)
        }
        Fault::TimingChainStretch { retard_deg } => format!("{retard_deg:.0}°"),
        Fault::FuelPumpWeak { capacity_fraction } => format!("{:.0} %", capacity_fraction * 100.0),
        Fault::ExhaustRestriction { factor } => format!("× {factor:.1}"),
        Fault::LowCompression {
            cylinder,
            leak_fraction,
        } => format!("{cyl} {}, {:.0} %", cylinder + 1, leak_fraction * 100.0),
        Fault::IntakeAirSensorOpen
        | Fault::KnockSensorDead
        | Fault::ThermostatStuckOpen
        | Fault::ThermostatStuckClosed
        | Fault::WastegateStuckClosed => return name.to_owned(),
    };
    format!("{name} ({detail})")
}

#[cfg(test)]
mod tests {
    use super::*;
    use tunerlab_core::EngineSpec;

    #[test]
    fn every_candidate_can_be_planted_in_both_presets() {
        let mut rng = Rng(12345);
        for spec in [
            EngineSpec::naturally_aspirated_2l(),
            EngineSpec::turbocharged_2l(),
        ] {
            let turbo = spec.turbo.is_some();
            let mut sim = EngineSim::with_base_calibration(spec, 1).unwrap();
            for id in candidates(turbo) {
                for _ in 0..20 {
                    sim.clear_faults();
                    let fault = instance(id, spec.geometry.cylinders, &mut rng);
                    assert_eq!(fault.id(), id);
                    assert!(sim.inject_fault(fault).is_ok(), "{fault:?} rejected");
                    assert!(!describe(&fault, Language::Pl).is_empty());
                }
            }
        }
    }

    #[test]
    fn new_case_plants_a_hidden_fault_and_checks_answers() {
        let mut sim =
            EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 1).unwrap();
        let mut autopsy = Autopsy::default();
        autopsy.new_case(&mut sim);
        let fault = autopsy.fault.expect("a case is open");
        assert!(sim.is_fault_active(fault.id()));
        let wrong = FaultId::ALL
            .iter()
            .copied()
            .find(|id| *id != fault.id())
            .unwrap();
        autopsy.answer = Some(wrong);
        autopsy.check();
        assert!(autopsy.verdict == Some(Verdict::Wrong));
        autopsy.answer = Some(fault.id());
        autopsy.check();
        assert!(autopsy.verdict == Some(Verdict::Correct));
        assert_eq!(autopsy.attempts, 2);
    }
}

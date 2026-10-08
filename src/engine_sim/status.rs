//! Engine condition, failure causes and instructor-level warnings.

/// Catastrophic or critical mechanical damage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FailureCause {
    /// Connecting rod broke in tension (over-rev or fatigue). Fatal.
    ThrownRod {
        /// Zero-based cylinder index.
        cylinder: u8,
    },
    /// Connecting rod buckled under combustion pressure (detonation / hydraulic load). Fatal.
    BentRod {
        /// Zero-based cylinder index.
        cylinder: u8,
    },
    /// Crankshaft bearing lost its oil film and seized. Fatal.
    SpunBearing,
    /// Piston crown or ring land eroded through by detonation.
    HoledPiston {
        /// Zero-based cylinder index.
        cylinder: u8,
    },
    /// Piston crown melted from excessive heat flux (lean / knock thermal runaway).
    MeltedPiston {
        /// Zero-based cylinder index.
        cylinder: u8,
    },
    /// Floating valve struck the piston.
    BentValve {
        /// Zero-based cylinder index.
        cylinder: u8,
    },
    /// Head gasket breached next to a cylinder (combustion gas ↔ coolant).
    HeadGasket {
        /// Zero-based cylinder index.
        cylinder: u8,
    },
    /// Aluminium cylinder head warped by overheating.
    WarpedHead,
    /// Turbocharger shaft or wheel failure.
    TurboFailure,
    /// Catalyst substrate melted (misfire / afterburn exotherm).
    CatalystMeltdown,
}

impl FailureCause {
    /// `true` when the engine cannot continue rotating.
    pub fn is_fatal(&self) -> bool {
        matches!(
            self,
            Self::ThrownRod { .. } | Self::BentRod { .. } | Self::SpunBearing
        )
    }

    /// Affected cylinder, if the failure is cylinder-specific.
    pub fn cylinder(&self) -> Option<u8> {
        match *self {
            Self::ThrownRod { cylinder }
            | Self::BentRod { cylinder }
            | Self::HoledPiston { cylinder }
            | Self::MeltedPiston { cylinder }
            | Self::BentValve { cylinder }
            | Self::HeadGasket { cylinder } => Some(cylinder),
            Self::SpunBearing | Self::WarpedHead | Self::TurboFailure | Self::CatalystMeltdown => {
                None
            }
        }
    }
}

/// High-level operating condition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EngineCondition {
    /// Ignition off, crankshaft at rest.
    Off,
    /// Starter motor turning the engine.
    Cranking,
    /// Self-sustained combustion.
    Running,
    /// Ignition on but the engine is not turning (stalled or not yet started).
    Stalled,
    /// Destroyed; requires [`EngineSim::repair`](super::EngineSim::repair).
    Failed(FailureCause),
}

/// Instructor warnings derived from the *physical truth* (not from sensors).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Warning {
    /// Detonation detected in at least one cylinder.
    Knock = 0,
    /// Engine speed above redline.
    OverRev,
    /// Valve float: springs can no longer control the valves.
    ValveFloat,
    /// Coolant temperature excessive.
    Overheat,
    /// Coolant boiling.
    CoolantBoiling,
    /// Oil pressure below the hydrodynamic bearing requirement.
    LowOilPressure,
    /// Oil temperature excessive.
    HighOilTemp,
    /// Lean mixture at high load.
    LeanUnderLoad,
    /// Excessively rich mixture (bore wash, fouling).
    RichMixture,
    /// Exhaust gas temperature excessive.
    HighEgt,
    /// Piston crown above alloy strength limit.
    PistonOverheat,
    /// Boost pressure above the safe limit.
    Overboost,
    /// Compressor operating left of the surge line.
    CompressorSurge,
    /// Turbocharger shaft speed above its limit.
    TurboOverspeed,
    /// Combustion misfire.
    Misfire,
    /// Injector duty cycle near static flow.
    InjectorDutyHigh,
    /// Catalyst temperature excessive.
    CatalystOverheat,
    /// The ECU has stored at least one diagnostic trouble code.
    CheckEngine,
}

impl Warning {
    /// Every warning, in bit order.
    pub const ALL: [Warning; 18] = [
        Warning::Knock,
        Warning::OverRev,
        Warning::ValveFloat,
        Warning::Overheat,
        Warning::CoolantBoiling,
        Warning::LowOilPressure,
        Warning::HighOilTemp,
        Warning::LeanUnderLoad,
        Warning::RichMixture,
        Warning::HighEgt,
        Warning::PistonOverheat,
        Warning::Overboost,
        Warning::CompressorSurge,
        Warning::TurboOverspeed,
        Warning::Misfire,
        Warning::InjectorDutyHigh,
        Warning::CatalystOverheat,
        Warning::CheckEngine,
    ];

    #[inline]
    fn bit(self) -> u32 {
        1 << (self as u8)
    }
}

/// Fixed-size set of [`Warning`]s (bit field, `Copy`, allocation-free).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Warnings(u32);

impl Warnings {
    /// Empty set.
    pub const NONE: Warnings = Warnings(0);

    /// Adds or removes a warning.
    #[inline]
    pub fn set(&mut self, w: Warning, on: bool) {
        if on {
            self.0 |= w.bit();
        } else {
            self.0 &= !w.bit();
        }
    }

    /// Membership test.
    #[inline]
    pub fn contains(&self, w: Warning) -> bool {
        self.0 & w.bit() != 0
    }

    /// `true` when no warning is active.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.0 == 0
    }

    /// Raw bit field (bit index = `Warning as u8`).
    #[inline]
    pub fn bits(&self) -> u32 {
        self.0
    }

    /// Iterates active warnings without allocating.
    pub fn iter(&self) -> impl Iterator<Item = Warning> + '_ {
        Warning::ALL
            .iter()
            .copied()
            .filter(move |w| self.contains(*w))
    }

    /// Warnings present in `self` but not in `previous` (rising edges).
    #[inline]
    pub fn newly_set(&self, previous: Warnings) -> Warnings {
        Warnings(self.0 & !previous.0)
    }
}

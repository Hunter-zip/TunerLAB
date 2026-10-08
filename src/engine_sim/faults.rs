//! Hidden fault injection for the "Engine Autopsy" diagnostic mode.
//!
//! A fault alters the *physics* or a *sensor*; it is never announced to the ECU. The student
//! must isolate it from logged sensor data, exactly as on a real vehicle.

use core::f32::consts::PI;
use core::fmt;

use super::spec::MAX_CYLINDERS;

/// A fault with its severity parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Fault {
    /// MAP sensor reads high (positive) or low (negative) by a constant offset.
    MapSensorBias {
        /// Offset \[kPa\].
        kpa: f32,
    },
    /// Coolant temperature sensor offset (e.g. corroded connector).
    CoolantSensorBias {
        /// Offset \[K\]; negative reads colder than reality.
        kelvin: f32,
    },
    /// Intake air temperature sensor open circuit (reads −50 °C, below the sensor range).
    IntakeAirSensorOpen,
    /// Ageing wide-band O2 sensor with a slow response.
    OxygenSensorSlow {
        /// Multiplier on the response time constant (> 1).
        factor: f32,
    },
    /// O2 sensor reading offset (exhaust leak upstream of the sensor reads lean).
    OxygenSensorBias {
        /// Offset in λ.
        lambda: f32,
    },
    /// Knock sensor disconnected: no knock feedback.
    KnockSensorDead,
    /// Unmetered air entering the intake manifold (cracked hose, PCV valve).
    VacuumLeak {
        /// Equivalent hole diameter \[mm\].
        diameter_mm: f32,
    },
    /// Partially clogged injector.
    InjectorClogged {
        /// Zero-based cylinder index.
        cylinder: u8,
        /// Remaining flow fraction 0..1.
        flow_fraction: f32,
    },
    /// Ignition coil with reduced secondary output (cracked housing, shorted turns).
    IgnitionCoilWeak {
        /// Zero-based cylinder index.
        cylinder: u8,
        /// Remaining output fraction 0..1.
        strength: f32,
    },
    /// Thermostat stuck open: engine never reaches operating temperature.
    ThermostatStuckOpen,
    /// Thermostat stuck closed: engine overheats under load.
    ThermostatStuckClosed,
    /// Stretched timing chain retarding the camshaft.
    TimingChainStretch {
        /// Cam retard [crank deg].
        retard_deg: f32,
    },
    /// Worn fuel pump that cannot meet high-load demand.
    FuelPumpWeak {
        /// Remaining capacity fraction 0..1.
        capacity_fraction: f32,
    },
    /// Clogged catalyst / crushed exhaust pipe.
    ExhaustRestriction {
        /// Multiplier on exhaust back-pressure (> 1).
        factor: f32,
    },
    /// Low compression in one cylinder (worn rings, leaking valve).
    LowCompression {
        /// Zero-based cylinder index.
        cylinder: u8,
        /// Fraction of trapped charge lost 0..1.
        leak_fraction: f32,
    },
    /// Leaking boost pipe or intercooler (turbo engines).
    BoostLeak {
        /// Equivalent hole diameter \[mm\].
        diameter_mm: f32,
    },
    /// Wastegate seized shut (turbo engines): uncontrolled boost.
    WastegateStuckClosed,
}

/// Fault identity without parameters, used for answer checking in Engine Autopsy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum FaultId {
    /// See [`Fault::MapSensorBias`].
    MapSensorBias = 0,
    /// See [`Fault::CoolantSensorBias`].
    CoolantSensorBias,
    /// See [`Fault::IntakeAirSensorOpen`].
    IntakeAirSensorOpen,
    /// See [`Fault::OxygenSensorSlow`].
    OxygenSensorSlow,
    /// See [`Fault::OxygenSensorBias`].
    OxygenSensorBias,
    /// See [`Fault::KnockSensorDead`].
    KnockSensorDead,
    /// See [`Fault::VacuumLeak`].
    VacuumLeak,
    /// See [`Fault::InjectorClogged`].
    InjectorClogged,
    /// See [`Fault::IgnitionCoilWeak`].
    IgnitionCoilWeak,
    /// See [`Fault::ThermostatStuckOpen`].
    ThermostatStuckOpen,
    /// See [`Fault::ThermostatStuckClosed`].
    ThermostatStuckClosed,
    /// See [`Fault::TimingChainStretch`].
    TimingChainStretch,
    /// See [`Fault::FuelPumpWeak`].
    FuelPumpWeak,
    /// See [`Fault::ExhaustRestriction`].
    ExhaustRestriction,
    /// See [`Fault::LowCompression`].
    LowCompression,
    /// See [`Fault::BoostLeak`].
    BoostLeak,
    /// See [`Fault::WastegateStuckClosed`].
    WastegateStuckClosed,
}

impl FaultId {
    /// Every fault identity.
    pub const ALL: [FaultId; 17] = [
        FaultId::MapSensorBias,
        FaultId::CoolantSensorBias,
        FaultId::IntakeAirSensorOpen,
        FaultId::OxygenSensorSlow,
        FaultId::OxygenSensorBias,
        FaultId::KnockSensorDead,
        FaultId::VacuumLeak,
        FaultId::InjectorClogged,
        FaultId::IgnitionCoilWeak,
        FaultId::ThermostatStuckOpen,
        FaultId::ThermostatStuckClosed,
        FaultId::TimingChainStretch,
        FaultId::FuelPumpWeak,
        FaultId::ExhaustRestriction,
        FaultId::LowCompression,
        FaultId::BoostLeak,
        FaultId::WastegateStuckClosed,
    ];
}

impl Fault {
    /// Parameter-free identity.
    pub fn id(&self) -> FaultId {
        match self {
            Self::MapSensorBias { .. } => FaultId::MapSensorBias,
            Self::CoolantSensorBias { .. } => FaultId::CoolantSensorBias,
            Self::IntakeAirSensorOpen => FaultId::IntakeAirSensorOpen,
            Self::OxygenSensorSlow { .. } => FaultId::OxygenSensorSlow,
            Self::OxygenSensorBias { .. } => FaultId::OxygenSensorBias,
            Self::KnockSensorDead => FaultId::KnockSensorDead,
            Self::VacuumLeak { .. } => FaultId::VacuumLeak,
            Self::InjectorClogged { .. } => FaultId::InjectorClogged,
            Self::IgnitionCoilWeak { .. } => FaultId::IgnitionCoilWeak,
            Self::ThermostatStuckOpen => FaultId::ThermostatStuckOpen,
            Self::ThermostatStuckClosed => FaultId::ThermostatStuckClosed,
            Self::TimingChainStretch { .. } => FaultId::TimingChainStretch,
            Self::FuelPumpWeak { .. } => FaultId::FuelPumpWeak,
            Self::ExhaustRestriction { .. } => FaultId::ExhaustRestriction,
            Self::LowCompression { .. } => FaultId::LowCompression,
            Self::BoostLeak { .. } => FaultId::BoostLeak,
            Self::WastegateStuckClosed => FaultId::WastegateStuckClosed,
        }
    }
}

/// Rejected fault parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultError {
    /// Cylinder index outside the engine.
    InvalidCylinder,
    /// Parameter outside its physical range, not finite, or so mild that nothing would
    /// change (a fault must be detectable).
    InvalidParameter,
    /// The fault needs hardware this engine does not have (turbo faults on an NA engine).
    NotApplicable,
}

impl fmt::Display for FaultError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidCylinder => write!(f, "cylinder index outside the engine"),
            Self::InvalidParameter => write!(f, "fault parameter out of range"),
            Self::NotApplicable => write!(f, "fault does not apply to this engine"),
        }
    }
}

impl std::error::Error for FaultError {}

/// Thermostat failure mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) enum ThermostatFault {
    #[default]
    None,
    StuckOpen,
    StuckClosed,
}

/// Effective fault parameters consumed by plant and sensors (neutral when healthy).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct FaultState {
    pub map_bias_pa: f32,
    pub coolant_sensor_bias_k: f32,
    pub iat_open_circuit: bool,
    pub o2_lag_factor: f32,
    pub o2_bias_lambda: f32,
    pub knock_sensor_dead: bool,
    pub vacuum_leak_area_m2: f32,
    pub injector_flow: [f32; MAX_CYLINDERS],
    pub coil_strength: [f32; MAX_CYLINDERS],
    pub thermostat: ThermostatFault,
    pub cam_retard_deg: f32,
    pub fuel_pump_capacity: f32,
    pub exhaust_restriction: f32,
    pub compression_leak: [f32; MAX_CYLINDERS],
    pub boost_leak_area_m2: f32,
    pub wastegate_stuck_closed: bool,
    active: u32,
}

impl Default for FaultState {
    fn default() -> Self {
        Self {
            map_bias_pa: 0.0,
            coolant_sensor_bias_k: 0.0,
            iat_open_circuit: false,
            o2_lag_factor: 1.0,
            o2_bias_lambda: 0.0,
            knock_sensor_dead: false,
            vacuum_leak_area_m2: 0.0,
            injector_flow: [1.0; MAX_CYLINDERS],
            coil_strength: [1.0; MAX_CYLINDERS],
            thermostat: ThermostatFault::None,
            cam_retard_deg: 0.0,
            fuel_pump_capacity: 1.0,
            exhaust_restriction: 1.0,
            compression_leak: [0.0; MAX_CYLINDERS],
            boost_leak_area_m2: 0.0,
            wastegate_stuck_closed: false,
            active: 0,
        }
    }
}

/// Area of a circular hole of diameter `d_mm` millimetres \[m²\].
fn hole_area(d_mm: f32) -> f32 {
    let d = d_mm * 1.0e-3;
    PI / 4.0 * d * d
}

/// Accepts `v` within `[lo, hi]` but different from the healthy value `neutral`, which
/// would make the fault invisible while still reporting it as active.
fn check(v: f32, lo: f32, hi: f32, neutral: f32) -> Result<f32, FaultError> {
    if v.is_finite() && v >= lo && v <= hi && v != neutral {
        Ok(v)
    } else {
        Err(FaultError::InvalidParameter)
    }
}

impl FaultState {
    /// Applies a fault. `cylinders` bounds cylinder-specific faults; turbo faults require
    /// `turbocharged`. Rejected faults leave the state unchanged.
    pub(crate) fn apply(
        &mut self,
        fault: Fault,
        cylinders: usize,
        turbocharged: bool,
    ) -> Result<(), FaultError> {
        let cyl = |c: u8| -> Result<usize, FaultError> {
            let c = usize::from(c);
            if c < cylinders {
                Ok(c)
            } else {
                Err(FaultError::InvalidCylinder)
            }
        };
        let turbo_only = || {
            if turbocharged {
                Ok(())
            } else {
                Err(FaultError::NotApplicable)
            }
        };
        match fault {
            Fault::MapSensorBias { kpa } => {
                self.map_bias_pa = check(kpa, -100.0, 100.0, 0.0)? * 1000.0;
            }
            Fault::CoolantSensorBias { kelvin } => {
                self.coolant_sensor_bias_k = check(kelvin, -80.0, 80.0, 0.0)?;
            }
            Fault::IntakeAirSensorOpen => self.iat_open_circuit = true,
            Fault::OxygenSensorSlow { factor } => {
                self.o2_lag_factor = check(factor, 1.0, 50.0, 1.0)?;
            }
            Fault::OxygenSensorBias { lambda } => {
                self.o2_bias_lambda = check(lambda, -0.5, 0.5, 0.0)?;
            }
            Fault::KnockSensorDead => self.knock_sensor_dead = true,
            Fault::VacuumLeak { diameter_mm } => {
                self.vacuum_leak_area_m2 = hole_area(check(diameter_mm, 0.0, 30.0, 0.0)?);
            }
            Fault::InjectorClogged {
                cylinder,
                flow_fraction,
            } => {
                let c = cyl(cylinder)?;
                self.injector_flow[c] = check(flow_fraction, 0.0, 1.0, 1.0)?;
            }
            Fault::IgnitionCoilWeak { cylinder, strength } => {
                let c = cyl(cylinder)?;
                self.coil_strength[c] = check(strength, 0.0, 1.0, 1.0)?;
            }
            Fault::ThermostatStuckOpen => {
                self.thermostat = ThermostatFault::StuckOpen;
                self.active &= !(1 << (FaultId::ThermostatStuckClosed as u8));
            }
            Fault::ThermostatStuckClosed => {
                self.thermostat = ThermostatFault::StuckClosed;
                self.active &= !(1 << (FaultId::ThermostatStuckOpen as u8));
            }
            Fault::TimingChainStretch { retard_deg } => {
                self.cam_retard_deg = check(retard_deg, 0.0, 30.0, 0.0)?;
            }
            Fault::FuelPumpWeak { capacity_fraction } => {
                self.fuel_pump_capacity = check(capacity_fraction, 0.05, 1.0, 1.0)?;
            }
            Fault::ExhaustRestriction { factor } => {
                self.exhaust_restriction = check(factor, 1.0, 50.0, 1.0)?;
            }
            Fault::LowCompression {
                cylinder,
                leak_fraction,
            } => {
                let c = cyl(cylinder)?;
                self.compression_leak[c] = check(leak_fraction, 0.0, 1.0, 0.0)?;
            }
            Fault::BoostLeak { diameter_mm } => {
                turbo_only()?;
                self.boost_leak_area_m2 = hole_area(check(diameter_mm, 0.0, 40.0, 0.0)?);
            }
            Fault::WastegateStuckClosed => {
                turbo_only()?;
                self.wastegate_stuck_closed = true;
            }
        }
        self.active |= 1 << (fault.id() as u8);
        Ok(())
    }

    /// Whether a fault of this kind is active.
    pub(crate) fn is_active(&self, id: FaultId) -> bool {
        self.active & (1 << (id as u8)) != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn apply_and_validate() {
        let mut f = FaultState::default();
        assert!(f
            .apply(
                Fault::InjectorClogged {
                    cylinder: 2,
                    flow_fraction: 0.6
                },
                4,
                false
            )
            .is_ok());
        assert_eq!(f.injector_flow[2], 0.6);
        assert!(f.is_active(FaultId::InjectorClogged));
        assert_eq!(
            f.apply(
                Fault::InjectorClogged {
                    cylinder: 5,
                    flow_fraction: 0.6
                },
                4,
                false
            ),
            Err(FaultError::InvalidCylinder)
        );
        assert_eq!(
            f.apply(
                Fault::VacuumLeak {
                    diameter_mm: f32::NAN
                },
                4,
                false
            ),
            Err(FaultError::InvalidParameter)
        );
    }

    #[test]
    fn undetectable_or_inapplicable_faults_are_rejected() {
        let mut f = FaultState::default();
        // Neutral severities would report an active fault that changes nothing.
        assert_eq!(
            f.apply(Fault::VacuumLeak { diameter_mm: 0.0 }, 4, false),
            Err(FaultError::InvalidParameter)
        );
        assert_eq!(
            f.apply(Fault::OxygenSensorSlow { factor: 1.0 }, 4, false),
            Err(FaultError::InvalidParameter)
        );
        // Turbo hardware faults on a naturally aspirated engine.
        assert_eq!(
            f.apply(Fault::BoostLeak { diameter_mm: 20.0 }, 4, false),
            Err(FaultError::NotApplicable)
        );
        assert_eq!(
            f.apply(Fault::WastegateStuckClosed, 4, false),
            Err(FaultError::NotApplicable)
        );
        assert!(f.apply(Fault::WastegateStuckClosed, 4, true).is_ok());
        assert!(!f.is_active(FaultId::BoostLeak));
        // The two thermostat modes are mutually exclusive.
        f.apply(Fault::ThermostatStuckOpen, 4, false).unwrap();
        f.apply(Fault::ThermostatStuckClosed, 4, false).unwrap();
        assert!(f.is_active(FaultId::ThermostatStuckClosed));
        assert!(!f.is_active(FaultId::ThermostatStuckOpen));
        assert_eq!(f.thermostat, ThermostatFault::StuckClosed);
    }
}

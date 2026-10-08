//! Static engine definition.
//!
//! An [`EngineSpec`] is the *physical truth* of an engine: geometry, breathing, fuel
//! hardware, thermal masses, material limits and an optional turbocharger. The emulated ECU
//! never reads it; the ECU only sees sensors and its own
//! [`Calibration`](super::calibration::Calibration). The gap between the two is what the
//! student learns to close.

use core::f32::consts::PI;
use core::fmt;

/// Maximum number of cylinders supported. Every per-cylinder array in the simulation is
/// statically sized with this constant so the hot loop never allocates.
pub const MAX_CYLINDERS: usize = 8;

/// Validation errors returned by [`EngineSpec::validate`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpecError {
    /// Cylinder count is zero or exceeds [`MAX_CYLINDERS`].
    CylinderCount,
    /// Firing order is not a permutation of `0..cylinders`.
    FiringOrder,
    /// A parameter that must be strictly positive and finite is not.
    NonPositive(&'static str),
    /// A parameter lies outside its physically meaningful range.
    OutOfRange(&'static str),
}

impl fmt::Display for SpecError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::CylinderCount => write!(f, "cylinder count must be 1..={MAX_CYLINDERS}"),
            Self::FiringOrder => write!(f, "firing order must be a permutation of 0..cylinders"),
            Self::NonPositive(name) => write!(f, "parameter `{name}` must be positive and finite"),
            Self::OutOfRange(name) => write!(f, "parameter `{name}` is out of range"),
        }
    }
}

impl std::error::Error for SpecError {}

/// Crank-train geometry and inertia.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeometrySpec {
    /// Number of cylinders (1..=[`MAX_CYLINDERS`]).
    pub cylinders: usize,
    /// Cylinder bore B \[m\].
    pub bore_m: f32,
    /// Piston stroke S \[m\] (twice the crank radius).
    pub stroke_m: f32,
    /// Connecting-rod centre-to-centre length l \[m\].
    pub conrod_m: f32,
    /// Geometric compression ratio r_c = (V_d + V_c) / V_c.
    pub compression_ratio: f32,
    /// Firing order as zero-based cylinder indices; the first `cylinders` entries are used.
    pub firing_order: [u8; MAX_CYLINDERS],
    /// Polar moment of inertia of crankshaft, flywheel, clutch and damper \[kg·m²\].
    pub rotating_inertia_kg_m2: f32,
    /// Reciprocating mass per cylinder: piston, pin, rings and ⅓ of the rod \[kg\]
    /// (the classic ⅓ / ⅔ split of rod mass between reciprocating and rotating parts).
    pub reciprocating_mass_kg: f32,
}

impl GeometrySpec {
    /// Swept volume of one cylinder V_d = π/4 · B² · S \[m³\].
    pub fn cylinder_displacement_m3(&self) -> f32 {
        PI / 4.0 * self.bore_m * self.bore_m * self.stroke_m
    }

    /// Total engine displacement \[m³\].
    pub fn displacement_m3(&self) -> f32 {
        self.cylinder_displacement_m3() * self.cylinders as f32
    }

    /// Clearance (combustion-chamber) volume V_c = V_d / (r_c − 1) \[m³\].
    pub fn clearance_volume_m3(&self) -> f32 {
        self.cylinder_displacement_m3() / (self.compression_ratio - 1.0)
    }

    /// Mean piston speed Ū_p = 2·S·n/60 \[m/s\] — the universal similarity parameter for
    /// friction, inlet Mach index and in-cylinder turbulence.
    pub fn mean_piston_speed(&self, rpm: f32) -> f32 {
        2.0 * self.stroke_m * rpm / 60.0
    }
}

/// Valve events and port sizing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ValveSpec {
    /// Intake valve closing, degrees after bottom dead centre. Compression effectively
    /// starts here, so it sets the *effective* compression ratio.
    pub ivc_abdc_deg: f32,
    /// Exhaust valve opening, degrees before bottom dead centre. Expansion work ends here;
    /// what remains in the gas is blow-down energy (lost on NA engines, harvested by a
    /// turbine on turbo engines).
    pub evo_bbdc_deg: f32,
    /// Intake valve head diameter \[m\].
    pub intake_valve_diameter_m: f32,
    /// Number of intake valves per cylinder.
    pub intake_valves_per_cylinder: u8,
}

/// Hidden volumetric-efficiency characteristics (the "true VE" a tuner must discover).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BreathingSpec {
    /// Quasi-steady breathing efficiency of ports and valves without wave tuning.
    pub base_ve: f32,
    /// Peak VE gain of the tuned intake runner (Helmholtz / inertial ram effect).
    pub ram_gain: f32,
    /// Engine speed of the runner's resonance peak \[rpm\].
    pub ram_tuned_rpm: f32,
    /// Gaussian half-width of the ram effect \[rpm\].
    pub ram_width_rpm: f32,
    /// Fractional VE loss from valve-overlap reversion at very low speed.
    pub overlap_loss: f32,
    /// Speed constant over which the overlap loss decays \[rpm\].
    pub overlap_decay_rpm: f32,
    /// Mean inlet-valve flow coefficient C_i used in Taylor's inlet Mach index.
    pub inlet_flow_coefficient: f32,
    /// Inlet Mach index Z at which VE has fallen to 50 % of its unchoked value.
    pub critical_mach_index: f32,
    /// Fraction of the wall-to-charge temperature difference picked up by the fresh charge
    /// at the reference speed of 3000 rpm (charge heating in hot ports).
    pub charge_heating: f32,
}

/// Electronic throttle body and idle-air bypass.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrottleSpec {
    /// Throttle bore diameter \[m\].
    pub bore_m: f32,
    /// Plate angle at the closed stop, measured from the plane normal to the bore \[deg\].
    pub closed_angle_deg: f32,
    /// Discharge coefficient Cd of the plate (vena-contracta contraction, typically 0.7–0.85).
    pub discharge_coefficient: f32,
    /// Fully open area of the idle-air control valve \[m²\].
    pub idle_valve_area_m2: f32,
    /// Leakage area around the closed plate \[m²\].
    pub leak_area_m2: f32,
    /// Time constant of the throttle motor position loop \[s\].
    pub actuator_time_constant_s: f32,
}

/// Intake system volumes and restrictions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IntakeSpec {
    /// Volume of the intake manifold downstream of the throttle \[m³\].
    pub manifold_volume_m3: f32,
    /// Air filter quadratic loss coefficient: Δp = k · ṁ² \[Pa·s²/kg²\] (turbulent
    /// pressure drop scales with dynamic pressure ½ρv² ∝ ṁ²).
    pub filter_restriction: f32,
}

/// Fuel properties (pump gasoline by default).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FuelSpec {
    /// Stoichiometric air-fuel mass ratio (14.7 for E0 gasoline).
    pub stoich_afr: f32,
    /// Lower heating value \[J/kg\] (water leaves as vapour, as it does in an engine).
    pub lower_heating_value: f32,
    /// Latent heat of vaporisation \[J/kg\].
    pub latent_heat: f32,
    /// Fraction of the cylinder fuel that evaporates from the charge after IVC (the rest
    /// evaporates on hot valve/port surfaces). Determines charge cooling.
    pub in_cylinder_evaporation: f32,
}

/// Port-fuel-injection hardware.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FuelSystemSpec {
    /// Static injector flow at the rated differential pressure \[kg/s\].
    pub injector_flow_kg_s: f32,
    /// Differential pressure at which the injector flow is rated \[Pa\].
    pub injector_rated_pressure_pa: f32,
    /// Injector opening dead time at 14 V \[s\].
    pub injector_dead_time_s: f32,
    /// Increase of dead time per volt below 14 V \[s/V\] (weaker solenoid pull-in force).
    pub injector_dead_time_slope_s_per_v: f32,
    /// Manifold-referenced fuel pressure regulator setting \[Pa\].
    pub regulator_pressure_pa: f32,
    /// Maximum fuel delivery of the pump at regulator pressure \[kg/s\].
    pub pump_capacity_kg_s: f32,
}

/// Combustion-chamber characteristics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CombustionSpec {
    /// Wiebe burn duration (0 → 99.3 % mass burned) at the reference speed, λ = 0.9 and
    /// 4 % residual gas [crank deg].
    pub burn_duration_deg: f32,
    /// Reference speed of `burn_duration_deg` \[rpm\].
    pub burn_reference_rpm: f32,
    /// Wiebe efficiency parameter a (a = 5 → 99.3 % burned at the end of the duration).
    pub wiebe_a: f32,
    /// Wiebe form factor m (m = 2 gives the typical slow-fast-slow S-curve of SI engines).
    pub wiebe_m: f32,
    /// Multiplier on the Woschni heat-transfer coefficient (chamber surface-to-volume
    /// ratio, deposits, coating). 1.0 = Woschni's original correlation.
    pub heat_transfer_scale: f32,
    /// Multiplier on the end-gas ignition delay: chamber design quality (squish, spark plug
    /// placement, hot spots). Higher means more knock-resistant.
    pub knock_resistance: f32,
    /// Polytropic index of compression of the unburned mixture.
    pub compression_index: f32,
    /// Secondary voltage a healthy ignition coil can deliver \[kV\].
    pub coil_output_kv: f32,
}

/// Friction and accessory losses (Chen–Flynn FMEP model).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrictionSpec {
    /// Constant FMEP term (valvetrain preload, seal drag) \[Pa\].
    pub fmep_constant_pa: f32,
    /// FMEP proportional to peak cylinder pressure (ring and bearing loading) \[-\].
    pub fmep_peak_pressure_coeff: f32,
    /// FMEP proportional to mean piston speed (hydrodynamic shear) \[Pa·s/m\].
    pub fmep_speed: f32,
    /// FMEP proportional to the square of mean piston speed (windage, pumping of oil) \[Pa·s²/m²\].
    pub fmep_speed_sq: f32,
    /// Base mechanical accessory power (water pump, power steering idling) \[W\].
    pub accessory_power_w: f32,
    /// Alternator efficiency (mechanical → electrical).
    pub alternator_efficiency: f32,
}

/// Lumped-capacitance thermal network.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalSpec {
    /// Heat capacity of head + block metal \[J/K\].
    pub metal_capacity: f32,
    /// Heat capacity of the coolant inventory \[J/K\].
    pub coolant_capacity: f32,
    /// Heat capacity of the oil inventory \[J/K\].
    pub oil_capacity: f32,
    /// Metal→coolant conductance at 3000 rpm pump speed \[W/K\].
    pub metal_coolant_ua: f32,
    /// Metal→oil conductance \[W/K\].
    pub metal_oil_ua: f32,
    /// Metal→ambient conductance in still air \[W/K\].
    pub metal_ambient_ua: f32,
    /// Oil sump→ambient conductance in still air \[W/K\].
    pub oil_ambient_ua: f32,
    /// Thermostat start-to-open temperature \[K\].
    pub thermostat_start_k: f32,
    /// Thermostat fully open temperature \[K\].
    pub thermostat_full_k: f32,
    /// Radiator conductance at 10 m/s core air speed \[W/K\].
    pub radiator_ua: f32,
    /// Radiator core frontal area \[m²\].
    pub radiator_area_m2: f32,
    /// Coolant pump delivery per 1000 rpm \[kg/s\].
    pub pump_flow_per_krpm: f32,
    /// Coolant boiling point under the pressure cap \[K\].
    pub coolant_boil_k: f32,
    /// Heat capacity of one piston crown \[J/K\].
    pub piston_capacity: f32,
    /// Piston crown → oil/ring-pack conductance \[W/K\].
    pub piston_cooling_ua: f32,
    /// Share of the in-cylinder wall heat absorbed by the piston crown.
    pub piston_heat_share: f32,
    /// Heat capacity of the catalytic converter substrate \[J/K\].
    pub catalyst_capacity: f32,
    /// Catalyst light-off temperature \[K\].
    pub catalyst_light_off_k: f32,
}

/// Lubrication system.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LubricationSpec {
    /// Vogel viscosity coefficient a \[Pa·s\].
    pub vogel_a: f32,
    /// Vogel viscosity coefficient b \[K\].
    pub vogel_b: f32,
    /// Vogel viscosity coefficient c \[K\].
    pub vogel_c: f32,
    /// Oil pressure per 1000 rpm at reference viscosity \[Pa\].
    pub pressure_per_krpm: f32,
    /// Exponent of the viscosity dependence of oil pressure.
    pub viscosity_exponent: f32,
    /// Pressure relief valve setting \[Pa\].
    pub relief_pressure_pa: f32,
    /// Oil temperature at which `pressure_per_krpm` is specified \[K\].
    pub reference_temp_k: f32,
}

/// Exhaust system.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExhaustSpec {
    /// Quadratic back-pressure coefficient of catalyst + mufflers: Δp = k·ṁ² \[Pa·s²/kg²\].
    pub backpressure: f32,
    /// Exhaust manifold volume upstream of the turbine/catalyst \[m³\].
    pub manifold_volume_m3: f32,
}

/// Mechanical and thermal design limits used by the damage model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MechanicalLimits {
    /// Manufacturer redline \[rpm\].
    pub redline_rpm: f32,
    /// Speed at which valve-spring force no longer keeps followers on the cam \[rpm\].
    pub valve_float_rpm: f32,
    /// Interference engine: a floating valve can touch the piston.
    pub interference: bool,
    /// Margin above float speed at which valve-to-piston contact becomes possible \[rpm\].
    pub valve_contact_margin_rpm: f32,
    /// Connecting-rod tensile endurance (fatigue) limit \[N\].
    pub rod_endurance_force_n: f32,
    /// Connecting-rod ultimate tensile load \[N\].
    pub rod_ultimate_force_n: f32,
    /// Connecting-rod buckling (compressive) load \[N\].
    pub rod_buckling_force_n: f32,
    /// Basquin exponent k of the rod S–N curve, N_f ∝ F^(−k).
    pub rod_fatigue_exponent: f32,
    /// Cycles to failure at the endurance load \[-\].
    pub rod_endurance_cycles: f32,
    /// Peak cylinder pressure the head gasket clamp load can seal \[Pa\].
    pub head_gasket_pressure_pa: f32,
    /// Piston crown temperature above which the alloy loses strength \[K\].
    pub piston_crown_limit_k: f32,
    /// Piston crown temperature at which the alloy erodes / melts \[K\].
    pub piston_crown_melt_k: f32,
    /// Head metal temperature at which the aluminium head starts to warp \[K\].
    pub head_warp_k: f32,
    /// Minimum oil pressure per 1000 rpm for a full hydrodynamic bearing film \[Pa\].
    pub bearing_oil_per_krpm: f32,
    /// Catalyst substrate melting temperature \[K\].
    pub catalyst_melt_k: f32,
}

/// Starter motor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StarterSpec {
    /// Stall torque referred to the crankshaft \[N·m\].
    pub stall_torque_nm: f32,
    /// No-load speed referred to the crankshaft \[rpm\].
    pub free_speed_rpm: f32,
}

/// Turbocharger, wastegate, blow-off valve and intercooler.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TurboSpec {
    /// Compressor wheel exducer diameter \[m\].
    pub compressor_diameter_m: f32,
    /// Turbine wheel inducer diameter \[m\].
    pub turbine_diameter_m: f32,
    /// Rotor polar moment of inertia \[kg·m²\].
    pub rotor_inertia_kg_m2: f32,
    /// Maximum permissible shaft speed \[rpm\].
    pub max_speed_rpm: f32,
    /// Compressor isentropic head coefficient at zero flow, ψ₀ = Δh_is / U².
    pub head_coefficient: f32,
    /// Compressor flow coefficient φ = ṁ/(ρ·U·D²) at which the speed line reaches PR = 1.
    pub choke_flow_coefficient: f32,
    /// Peak compressor isentropic efficiency.
    pub compressor_peak_efficiency: f32,
    /// Peak turbine total-to-static efficiency.
    pub turbine_peak_efficiency: f32,
    /// Effective nozzle area of the turbine housing \[m²\] (the A/R choice).
    pub turbine_area_m2: f32,
    /// Fully open wastegate valve area \[m²\].
    pub wastegate_area_m2: f32,
    /// Boost (gauge) at which the wastegate actuator spring starts to yield \[Pa\].
    pub wastegate_spring_pa: f32,
    /// Additional actuator pressure for full wastegate opening \[Pa\].
    pub wastegate_span_pa: f32,
    /// Fraction of actuator pressure the boost-control solenoid can bleed at 100 % duty.
    pub wastegate_bleed_authority: f32,
    /// Volume between compressor and throttle (pipes + intercooler) \[m³\].
    pub boost_volume_m3: f32,
    /// Intercooler effectiveness ε = (T_in − T_out)/(T_in − T_ambient).
    pub intercooler_effectiveness: f32,
    /// Fully open blow-off valve area \[m²\].
    pub bov_area_m2: f32,
    /// Boost-minus-manifold pressure at which the blow-off valve cracks \[Pa\].
    pub bov_crack_pa: f32,
    /// Bearing loss coefficient: P = k·ω² \[W·s²\].
    pub bearing_loss: f32,
    /// Maximum continuous turbine inlet temperature \[K\].
    pub turbine_inlet_limit_k: f32,
    /// Highest manifold pressure the hardware (gasket clamp, rods, fuel system) is built
    /// to withstand continuously [Pa abs]; drives the Overboost instructor warning.
    pub max_manifold_pressure_pa: f32,
}

/// Complete physical description of an engine.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EngineSpec {
    /// Crank-train geometry.
    pub geometry: GeometrySpec,
    /// Valve timing and porting.
    pub valves: ValveSpec,
    /// Hidden volumetric efficiency characteristics.
    pub breathing: BreathingSpec,
    /// Throttle body.
    pub throttle: ThrottleSpec,
    /// Intake system.
    pub intake: IntakeSpec,
    /// Fuel properties.
    pub fuel: FuelSpec,
    /// Fuel delivery hardware.
    pub fuel_system: FuelSystemSpec,
    /// Combustion chamber.
    pub combustion: CombustionSpec,
    /// Friction model.
    pub friction: FrictionSpec,
    /// Thermal network.
    pub thermal: ThermalSpec,
    /// Lubrication.
    pub lubrication: LubricationSpec,
    /// Exhaust system.
    pub exhaust: ExhaustSpec,
    /// Damage thresholds.
    pub limits: MechanicalLimits,
    /// Starter motor.
    pub starter: StarterSpec,
    /// Turbocharger (`None` for naturally aspirated engines).
    pub turbo: Option<TurboSpec>,
}

impl EngineSpec {
    /// Generic 2.0 L naturally aspirated DOHC 16V inline-four (the free introductory engine).
    ///
    /// Square 86 × 86 mm bore/stroke (1998 cm³), 10.5:1 compression, tuned intake runners
    /// peaking near 4800 rpm. Comparable to mainstream 2.0 L engines of the 2000s: on the
    /// generated base calibration ≈ 190 N·m at 4500 rpm and ≈ 107 kW at 6500 rpm.
    pub fn naturally_aspirated_2l() -> Self {
        Self {
            geometry: GeometrySpec {
                cylinders: 4,
                bore_m: 0.086,
                stroke_m: 0.086,
                // Rod ratio l/r = 3.34, typical for passenger-car fours (3.0–3.6).
                conrod_m: 0.1435,
                compression_ratio: 10.5,
                // 1-3-4-2, the universal inline-four order (zero-based indices).
                firing_order: [0, 2, 3, 1, 0, 0, 0, 0],
                // Crank ≈ 0.04, dual-mass flywheel + clutch ≈ 0.11, damper/pulleys ≈ 0.01.
                rotating_inertia_kg_m2: 0.16,
                // Cast piston 330 g, pin 90 g, rings 30 g, ⅓ of a 500 g rod ≈ 170 g.
                reciprocating_mass_kg: 0.52,
            },
            valves: ValveSpec {
                ivc_abdc_deg: 45.0,
                evo_bbdc_deg: 45.0,
                intake_valve_diameter_m: 0.033,
                intake_valves_per_cylinder: 2,
            },
            breathing: BreathingSpec {
                base_ve: 0.92,
                ram_gain: 0.14,
                ram_tuned_rpm: 4800.0,
                ram_width_rpm: 2000.0,
                overlap_loss: 0.12,
                overlap_decay_rpm: 900.0,
                // Taylor's mean inlet flow coefficient for poppet valves ≈ 0.35.
                inlet_flow_coefficient: 0.35,
                critical_mach_index: 0.80,
                charge_heating: 0.15,
            },
            throttle: ThrottleSpec {
                bore_m: 0.060,
                closed_angle_deg: 7.0,
                discharge_coefficient: 0.82,
                // ≈ 10.7 mm equivalent bore: enough for cold fast-idle at 1300 rpm.
                idle_valve_area_m2: 9.0e-5,
                leak_area_m2: 1.5e-6,
                actuator_time_constant_s: 0.03,
            },
            intake: IntakeSpec {
                // ≈ 1.75 × displacement: plenum plus runners.
                manifold_volume_m3: 0.0035,
                // 2.5 kPa loss at 0.15 kg/s: k = 2500 / 0.15².
                filter_restriction: 111_000.0,
            },
            fuel: FuelSpec::gasoline(),
            fuel_system: FuelSystemSpec {
                // 3.2 g/s ≈ 260 cm³/min of gasoline at 3 bar: 85 % duty covers 115 kW.
                injector_flow_kg_s: 3.2e-3,
                injector_rated_pressure_pa: 300_000.0,
                injector_dead_time_s: 0.9e-3,
                injector_dead_time_slope_s_per_v: 0.25e-3,
                regulator_pressure_pa: 300_000.0,
                pump_capacity_kg_s: 0.04,
            },
            combustion: CombustionSpec {
                burn_duration_deg: 66.0,
                burn_reference_rpm: 3000.0,
                wiebe_a: 5.0,
                wiebe_m: 2.0,
                heat_transfer_scale: 1.0,
                knock_resistance: 1.5,
                compression_index: 1.32,
                coil_output_kv: 38.0,
            },
            friction: FrictionSpec::passenger_car(),
            thermal: ThermalSpec::two_litre(),
            lubrication: LubricationSpec::sae_5w30(),
            exhaust: ExhaustSpec {
                // ≈ 20 kPa at the 0.116 kg/s full-power exhaust flow.
                backpressure: 1.5e6,
                manifold_volume_m3: 0.0015,
            },
            limits: MechanicalLimits {
                redline_rpm: 6800.0,
                valve_float_rpm: 7400.0,
                interference: true,
                valve_contact_margin_rpm: 500.0,
                // m·r·ω²·(1 + r/l) = 0.52·0.043·(712 rad/s)²·1.30 ≈ 14.7 kN at redline:
                // the rod is designed to survive redline indefinitely.
                rod_endurance_force_n: 15_000.0,
                // Reached at ≈ 9600 rpm.
                rod_ultimate_force_n: 30_000.0,
                rod_buckling_force_n: 85_000.0,
                // Basquin exponent ≈ 9 for quenched & tempered steel rods (b ≈ −0.11).
                rod_fatigue_exponent: 9.0,
                rod_endurance_cycles: 1.0e6,
                head_gasket_pressure_pa: 110.0e5,
                // 350 °C: Al-Si piston alloys lose ~50 % of their strength.
                piston_crown_limit_k: 623.0,
                // 500 °C: approaching the ≈ 520 °C solidus of Al-Si eutectic alloys.
                piston_crown_melt_k: 773.0,
                // 155 °C metal: permanent head distortion of A356-T6 heads.
                head_warp_k: 428.0,
                // The industry "10 psi per 1000 rpm" rule ≈ 69 kPa per 1000 rpm.
                bearing_oil_per_krpm: 69_000.0,
                // Cordierite substrate softens near 1250 K under thermal shock.
                catalyst_melt_k: 1250.0,
            },
            starter: StarterSpec {
                stall_torque_nm: 110.0,
                free_speed_rpm: 450.0,
            },
            turbo: None,
        }
    }

    /// Generic 2.0 L turbocharged inline-four (premium forced-induction module).
    ///
    /// 9.5:1 compression, forged internals, 0.8 bar factory boost: on the generated base
    /// calibration ≈ 295 N·m at 4000 rpm and ≈ 155 kW at 5500 rpm.
    pub fn turbocharged_2l() -> Self {
        let mut spec = Self::naturally_aspirated_2l();
        spec.geometry.compression_ratio = 9.5;
        // Forged piston and rod: heavier but stronger.
        spec.geometry.reciprocating_mass_kg = 0.58;
        spec.breathing = BreathingSpec {
            base_ve: 0.92,
            // Short runners: turbo engines rely on boost, not wave tuning.
            ram_gain: 0.06,
            ram_tuned_rpm: 4000.0,
            ram_width_rpm: 2200.0,
            overlap_loss: 0.10,
            overlap_decay_rpm: 900.0,
            inlet_flow_coefficient: 0.35,
            critical_mach_index: 0.80,
            charge_heating: 0.15,
        };
        spec.throttle.bore_m = 0.062;
        spec.intake.manifold_volume_m3 = 0.0030;
        // Larger air filter for the higher flow.
        spec.intake.filter_restriction = 50_000.0;
        // 5.5 g/s ≈ 450 cm³/min at 3 bar.
        spec.fuel_system.injector_flow_kg_s = 5.5e-3;
        spec.fuel_system.pump_capacity_kg_s = 0.065;
        // Turbine extracts energy, so the post-turbine system flows more freely.
        spec.exhaust.backpressure = 1.0e6;
        spec.exhaust.manifold_volume_m3 = 0.0012;
        spec.combustion.coil_output_kv = 42.0;
        spec.limits = MechanicalLimits {
            redline_rpm: 6500.0,
            valve_float_rpm: 7300.0,
            rod_endurance_force_n: 18_000.0,
            rod_ultimate_force_n: 34_000.0,
            rod_buckling_force_n: 110_000.0,
            // Multi-layer steel gasket and ARP-type studs.
            head_gasket_pressure_pa: 140.0e5,
            ..spec.limits
        };
        spec.thermal.piston_cooling_ua = 24.0; // oil-jet cooled pistons
        spec.turbo = Some(TurboSpec {
            compressor_diameter_m: 0.052,
            turbine_diameter_m: 0.047,
            rotor_inertia_kg_m2: 2.5e-5,
            max_speed_rpm: 210_000.0,
            head_coefficient: 0.62,
            choke_flow_coefficient: 0.27,
            compressor_peak_efficiency: 0.76,
            turbine_peak_efficiency: 0.70,
            // Small (K03-class) turbine housing: high back-pressure at low flow for early
            // spool; the wastegate bypasses the excess at high flow.
            turbine_area_m2: 4.0e-4,
            wastegate_area_m2: 7.0e-4,
            wastegate_spring_pa: 50_000.0,
            wastegate_span_pa: 30_000.0,
            wastegate_bleed_authority: 0.6,
            boost_volume_m3: 0.005,
            intercooler_effectiveness: 0.75,
            bov_area_m2: 3.0e-4,
            bov_crack_pa: 25_000.0,
            // 600 W bearing loss at the 22 000 rad/s speed limit.
            bearing_loss: 1.24e-6,
            // 1050 °C: limit of Inconel 713C turbine wheels.
            turbine_inlet_limit_k: 1323.0,
            // 1.4 bar boost: the forged bottom end and MLS gasket are rated with margin
            // above the 0.8 bar factory target.
            max_manifold_pressure_pa: 240_000.0,
        });
        spec
    }

    /// Checks that every parameter is finite and physically meaningful.
    pub fn validate(&self) -> Result<(), SpecError> {
        let g = &self.geometry;
        if g.cylinders == 0 || g.cylinders > MAX_CYLINDERS {
            return Err(SpecError::CylinderCount);
        }
        let mut seen = [false; MAX_CYLINDERS];
        for &c in &g.firing_order[..g.cylinders] {
            let c = usize::from(c);
            if c >= g.cylinders || seen[c] {
                return Err(SpecError::FiringOrder);
            }
            seen[c] = true;
        }
        let positive: [(&'static str, f32); 53] = [
            ("bore_m", g.bore_m),
            ("stroke_m", g.stroke_m),
            ("conrod_m", g.conrod_m),
            ("rotating_inertia_kg_m2", g.rotating_inertia_kg_m2),
            ("reciprocating_mass_kg", g.reciprocating_mass_kg),
            (
                "intake_valve_diameter_m",
                self.valves.intake_valve_diameter_m,
            ),
            ("base_ve", self.breathing.base_ve),
            ("ram_tuned_rpm", self.breathing.ram_tuned_rpm),
            ("ram_width_rpm", self.breathing.ram_width_rpm),
            ("overlap_decay_rpm", self.breathing.overlap_decay_rpm),
            (
                "inlet_flow_coefficient",
                self.breathing.inlet_flow_coefficient,
            ),
            ("critical_mach_index", self.breathing.critical_mach_index),
            ("throttle.bore_m", self.throttle.bore_m),
            ("discharge_coefficient", self.throttle.discharge_coefficient),
            (
                "actuator_time_constant_s",
                self.throttle.actuator_time_constant_s,
            ),
            ("manifold_volume_m3", self.intake.manifold_volume_m3),
            ("stoich_afr", self.fuel.stoich_afr),
            ("lower_heating_value", self.fuel.lower_heating_value),
            ("injector_flow_kg_s", self.fuel_system.injector_flow_kg_s),
            (
                "injector_rated_pressure_pa",
                self.fuel_system.injector_rated_pressure_pa,
            ),
            (
                "regulator_pressure_pa",
                self.fuel_system.regulator_pressure_pa,
            ),
            ("pump_capacity_kg_s", self.fuel_system.pump_capacity_kg_s),
            ("burn_duration_deg", self.combustion.burn_duration_deg),
            ("burn_reference_rpm", self.combustion.burn_reference_rpm),
            ("wiebe_a", self.combustion.wiebe_a),
            ("heat_transfer_scale", self.combustion.heat_transfer_scale),
            ("knock_resistance", self.combustion.knock_resistance),
            ("coil_output_kv", self.combustion.coil_output_kv),
            ("alternator_efficiency", self.friction.alternator_efficiency),
            ("metal_capacity", self.thermal.metal_capacity),
            ("coolant_capacity", self.thermal.coolant_capacity),
            ("oil_capacity", self.thermal.oil_capacity),
            ("metal_coolant_ua", self.thermal.metal_coolant_ua),
            ("radiator_ua", self.thermal.radiator_ua),
            ("radiator_area_m2", self.thermal.radiator_area_m2),
            ("pump_flow_per_krpm", self.thermal.pump_flow_per_krpm),
            ("piston_capacity", self.thermal.piston_capacity),
            ("piston_cooling_ua", self.thermal.piston_cooling_ua),
            ("catalyst_capacity", self.thermal.catalyst_capacity),
            ("vogel_a", self.lubrication.vogel_a),
            ("pressure_per_krpm", self.lubrication.pressure_per_krpm),
            ("relief_pressure_pa", self.lubrication.relief_pressure_pa),
            (
                "exhaust.manifold_volume_m3",
                self.exhaust.manifold_volume_m3,
            ),
            ("redline_rpm", self.limits.redline_rpm),
            ("valve_float_rpm", self.limits.valve_float_rpm),
            ("rod_endurance_force_n", self.limits.rod_endurance_force_n),
            ("rod_ultimate_force_n", self.limits.rod_ultimate_force_n),
            ("rod_buckling_force_n", self.limits.rod_buckling_force_n),
            ("rod_endurance_cycles", self.limits.rod_endurance_cycles),
            (
                "head_gasket_pressure_pa",
                self.limits.head_gasket_pressure_pa,
            ),
            ("piston_crown_melt_k", self.limits.piston_crown_melt_k),
            ("stall_torque_nm", self.starter.stall_torque_nm),
            ("free_speed_rpm", self.starter.free_speed_rpm),
        ];
        for (name, v) in positive {
            if !(v.is_finite() && v > 0.0) {
                return Err(SpecError::NonPositive(name));
            }
        }
        let b = &self.breathing;
        let fr = &self.friction;
        let th = &self.thermal;
        let lub = &self.lubrication;
        let lim = &self.limits;
        let non_negative: [(&'static str, f32); 20] = [
            ("ram_gain", b.ram_gain),
            ("overlap_loss", b.overlap_loss),
            ("charge_heating", b.charge_heating),
            ("idle_valve_area_m2", self.throttle.idle_valve_area_m2),
            ("leak_area_m2", self.throttle.leak_area_m2),
            ("filter_restriction", self.intake.filter_restriction),
            ("latent_heat", self.fuel.latent_heat),
            (
                "injector_dead_time_s",
                self.fuel_system.injector_dead_time_s,
            ),
            ("fmep_constant_pa", fr.fmep_constant_pa),
            ("fmep_peak_pressure_coeff", fr.fmep_peak_pressure_coeff),
            ("fmep_speed", fr.fmep_speed),
            ("fmep_speed_sq", fr.fmep_speed_sq),
            ("accessory_power_w", fr.accessory_power_w),
            ("metal_oil_ua", th.metal_oil_ua),
            ("metal_ambient_ua", th.metal_ambient_ua),
            ("oil_ambient_ua", th.oil_ambient_ua),
            ("exhaust.backpressure", self.exhaust.backpressure),
            ("valve_contact_margin_rpm", lim.valve_contact_margin_rpm),
            ("bearing_oil_per_krpm", lim.bearing_oil_per_krpm),
            ("vogel_b", lub.vogel_b),
        ];
        for (name, v) in non_negative {
            if !(v.is_finite() && v >= 0.0) {
                return Err(SpecError::OutOfRange(name));
            }
        }
        // (name, value, min, max), inclusive.
        let ranged: [(&'static str, f32, f32, f32); 19] = [
            (
                "injector_dead_time_slope_s_per_v",
                self.fuel_system.injector_dead_time_slope_s_per_v,
                -1.0e-3,
                1.0e-3,
            ),
            (
                "in_cylinder_evaporation",
                self.fuel.in_cylinder_evaporation,
                0.0,
                1.0,
            ),
            ("base_ve", b.base_ve, 0.1, 1.5),
            ("wiebe_m", self.combustion.wiebe_m, -0.9, 10.0),
            (
                "compression_index",
                self.combustion.compression_index,
                1.1,
                1.45,
            ),
            ("alternator_efficiency", fr.alternator_efficiency, 0.1, 1.0),
            ("thermostat_start_k", th.thermostat_start_k, 273.15, 420.0),
            ("thermostat_full_k", th.thermostat_full_k, 273.15, 430.0),
            ("coolant_boil_k", th.coolant_boil_k, 350.0, 500.0),
            (
                "catalyst_light_off_k",
                th.catalyst_light_off_k,
                300.0,
                1000.0,
            ),
            ("vogel_c", lub.vogel_c, 0.0, 250.0),
            ("viscosity_exponent", lub.viscosity_exponent, 0.0, 3.0),
            ("reference_temp_k", lub.reference_temp_k, 273.15, 450.0),
            ("rod_fatigue_exponent", lim.rod_fatigue_exponent, 1.0, 30.0),
            (
                "piston_crown_limit_k",
                lim.piston_crown_limit_k,
                350.0,
                1000.0,
            ),
            ("head_warp_k", lim.head_warp_k, 350.0, 800.0),
            ("catalyst_melt_k", lim.catalyst_melt_k, 600.0, 2500.0),
            ("valve_float_rpm", lim.valve_float_rpm, 1000.0, 30_000.0),
            ("redline_rpm", lim.redline_rpm, 1000.0, 30_000.0),
        ];
        for (name, v, lo, hi) in ranged {
            if !(v.is_finite() && v >= lo && v <= hi) {
                return Err(SpecError::OutOfRange(name));
            }
        }
        if self.valves.intake_valves_per_cylinder == 0 {
            return Err(SpecError::OutOfRange("intake_valves_per_cylinder"));
        }
        if th.coolant_boil_k <= th.thermostat_full_k {
            return Err(SpecError::OutOfRange("coolant_boil_k"));
        }
        if lim.piston_crown_melt_k <= lim.piston_crown_limit_k {
            return Err(SpecError::OutOfRange("piston_crown_melt_k"));
        }
        if lim.rod_ultimate_force_n <= lim.rod_endurance_force_n {
            return Err(SpecError::OutOfRange("rod_ultimate_force_n"));
        }
        if lim.catalyst_melt_k <= th.catalyst_light_off_k + 100.0 {
            return Err(SpecError::OutOfRange("catalyst_melt_k"));
        }
        if !(g.compression_ratio > 4.0 && g.compression_ratio < 25.0) {
            return Err(SpecError::OutOfRange("compression_ratio"));
        }
        // Slider-crank requires l > r, practical engines have l/r > 2.5.
        if g.conrod_m <= 1.5 * g.stroke_m {
            return Err(SpecError::OutOfRange("conrod_m"));
        }
        if !(0.0..90.0).contains(&self.valves.ivc_abdc_deg) {
            return Err(SpecError::OutOfRange("ivc_abdc_deg"));
        }
        if !(0.0..90.0).contains(&self.valves.evo_bbdc_deg) {
            return Err(SpecError::OutOfRange("evo_bbdc_deg"));
        }
        if !(0.0..45.0).contains(&self.throttle.closed_angle_deg) {
            return Err(SpecError::OutOfRange("closed_angle_deg"));
        }
        if self.thermal.thermostat_full_k <= self.thermal.thermostat_start_k {
            return Err(SpecError::OutOfRange("thermostat_full_k"));
        }
        if !(0.0..=1.0).contains(&self.thermal.piston_heat_share) {
            return Err(SpecError::OutOfRange("piston_heat_share"));
        }
        // A healthy engine must be able to meet its own bearing oil requirement up to
        // 500 rpm beyond redline (where the factory limiter sits).
        let oil_needed =
            self.limits.bearing_oil_per_krpm * (self.limits.redline_rpm + 500.0) / 1000.0;
        if self.lubrication.relief_pressure_pa < oil_needed {
            return Err(SpecError::OutOfRange("relief_pressure_pa"));
        }
        if !(0.0..=1.0).contains(&self.fuel.in_cylinder_evaporation) {
            return Err(SpecError::OutOfRange("in_cylinder_evaporation"));
        }
        if !(1.05..1.6).contains(&self.combustion.compression_index) {
            return Err(SpecError::OutOfRange("compression_index"));
        }
        if let Some(t) = &self.turbo {
            let turbo_positive: [(&'static str, f32); 14] = [
                ("compressor_diameter_m", t.compressor_diameter_m),
                ("turbine_diameter_m", t.turbine_diameter_m),
                ("rotor_inertia_kg_m2", t.rotor_inertia_kg_m2),
                ("max_speed_rpm", t.max_speed_rpm),
                ("head_coefficient", t.head_coefficient),
                ("choke_flow_coefficient", t.choke_flow_coefficient),
                ("compressor_peak_efficiency", t.compressor_peak_efficiency),
                ("turbine_peak_efficiency", t.turbine_peak_efficiency),
                ("turbine_area_m2", t.turbine_area_m2),
                ("wastegate_span_pa", t.wastegate_span_pa),
                ("boost_volume_m3", t.boost_volume_m3),
                ("bov_crack_pa", t.bov_crack_pa),
                ("turbine_inlet_limit_k", t.turbine_inlet_limit_k),
                ("max_manifold_pressure_pa", t.max_manifold_pressure_pa),
            ];
            for (name, v) in turbo_positive {
                if !(v.is_finite() && v > 0.0) {
                    return Err(SpecError::NonPositive(name));
                }
            }
            let turbo_non_negative: [(&'static str, f32); 4] = [
                ("wastegate_area_m2", t.wastegate_area_m2),
                ("wastegate_spring_pa", t.wastegate_spring_pa),
                ("bov_area_m2", t.bov_area_m2),
                ("bearing_loss", t.bearing_loss),
            ];
            for (name, v) in turbo_non_negative {
                if !(v.is_finite() && v >= 0.0) {
                    return Err(SpecError::OutOfRange(name));
                }
            }
            if t.compressor_peak_efficiency > 1.0 {
                return Err(SpecError::OutOfRange("compressor_peak_efficiency"));
            }
            if t.turbine_peak_efficiency > 1.0 {
                return Err(SpecError::OutOfRange("turbine_peak_efficiency"));
            }
            if !(0.0..=1.0).contains(&t.intercooler_effectiveness) {
                return Err(SpecError::OutOfRange("intercooler_effectiveness"));
            }
            if !(0.0..=1.0).contains(&t.wastegate_bleed_authority) {
                return Err(SpecError::OutOfRange("wastegate_bleed_authority"));
            }
        }
        Ok(())
    }
}

impl FuelSpec {
    /// European E0/E5 pump gasoline.
    pub fn gasoline() -> Self {
        Self {
            // C₇H₁₃-equivalent surrogate: 14.7 kg air per kg fuel.
            stoich_afr: 14.7,
            // 43.4 MJ/kg lower heating value (Heywood Table D.4: 43–44 MJ/kg).
            lower_heating_value: 43.4e6,
            // ≈ 350 kJ/kg (Heywood Table D.4).
            latent_heat: 350.0e3,
            // PFI: most fuel evaporates on the hot intake valve, ≈ 30 % in the charge.
            in_cylinder_evaporation: 0.3,
        }
    }
}

impl FrictionSpec {
    /// Chen–Flynn coefficients fitted to modern passenger-car SI friction data
    /// (Heywood Fig. 13-11): FMEP ≈ 0.65 bar @ 1000 rpm, 1.15 bar @ 3000 rpm,
    /// 1.6 bar @ 5000 rpm, 2.0 bar @ 6500 rpm for an 86 mm stroke.
    pub fn passenger_car() -> Self {
        Self {
            fmep_constant_pa: 32_000.0,
            fmep_peak_pressure_coeff: 0.004,
            fmep_speed: 5_500.0,
            fmep_speed_sq: 110.0,
            accessory_power_w: 250.0,
            alternator_efficiency: 0.6,
        }
    }
}

impl ThermalSpec {
    /// Thermal network of a 2.0 L aluminium-head / iron-block four.
    pub fn two_litre() -> Self {
        Self {
            // 12 kg Al head × 900 J/(kg·K) + 35 kg iron block × 460 J/(kg·K) ≈ 27 kJ/K.
            metal_capacity: 27_000.0,
            // Engine-side coolant loop (≈ 3.4 kg of 50/50 glycol) that circulates while
            // the thermostat is closed; the radiator side is modelled as the heat exchanger.
            coolant_capacity: 12_000.0,
            // 4.5 L × 0.85 kg/L × 2000 J/(kg·K).
            oil_capacity: 7_600.0,
            metal_coolant_ua: 2_500.0,
            metal_oil_ua: 150.0,
            metal_ambient_ua: 25.0,
            oil_ambient_ua: 30.0,
            // 88 °C wax element, fully open at 98 °C.
            thermostat_start_k: 361.15,
            thermostat_full_k: 371.15,
            radiator_ua: 1_800.0,
            radiator_area_m2: 0.30,
            pump_flow_per_krpm: 0.35,
            // 50/50 glycol under a 1.1 bar cap boils at ≈ 125 °C.
            coolant_boil_k: 398.0,
            // 0.35 kg Al crown region × 900 J/(kg·K).
            piston_capacity: 320.0,
            piston_cooling_ua: 18.0,
            piston_heat_share: 0.22,
            // Close-coupled catalyst: ≈ 1.2 kg cordierite substrate + can × 1000 J/(kg·K)
            // (low thermal mass for fast light-off).
            catalyst_capacity: 1_500.0,
            catalyst_light_off_k: 550.0,
        }
    }
}

impl LubricationSpec {
    /// SAE 5W-30 fitted with the Vogel equation to μ(40 °C) = 55 mPa·s and
    /// μ(100 °C) = 9.5 mPa·s, giving μ(20 °C) ≈ 150 mPa·s and μ(−20 °C) ≈ 5 Pa·s,
    /// consistent with the SAE J300 cold-cranking limits for a 5W grade.
    pub fn sae_5w30() -> Self {
        Self {
            vogel_a: 1.827e-4,
            vogel_b: 770.5,
            // Vogel c = −95 °C.
            vogel_c: 178.15,
            // 1.5 bar hot idle at 800 rpm, typical of gerotor pumps.
            pressure_per_krpm: 190_000.0,
            // Bearing leakage obeys Hagen–Poiseuille (Δp ∝ μ); the relief and the
            // pressure-compensated pump soften that to ≈ μ^0.8.
            viscosity_exponent: 0.8,
            // 5.5 bar: covers the "10 psi per 1000 rpm" bearing requirement up to 8000 rpm.
            relief_pressure_pa: 550_000.0,
            reference_temp_k: 363.15,
        }
    }
}

/// Slider-crank kinematics of one cylinder (crate-internal, precomputed once).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CylinderGeometry {
    pub crank_radius: f32,
    pub conrod: f32,
    pub piston_area: f32,
    pub swept_volume: f32,
    pub clearance_volume: f32,
}

impl CylinderGeometry {
    pub(crate) fn from_spec(g: &GeometrySpec) -> Self {
        Self {
            crank_radius: g.stroke_m * 0.5,
            conrod: g.conrod_m,
            piston_area: PI / 4.0 * g.bore_m * g.bore_m,
            swept_volume: g.cylinder_displacement_m3(),
            clearance_volume: g.clearance_volume_m3(),
        }
    }

    /// Instantaneous cylinder volume at crank angle θ (radians after TDC), from the exact
    /// slider-crank relation (Heywood eq. 2.4–2.6):
    ///
    /// `s(θ) = r·(1 − cos θ) + l − √(l² − r²·sin²θ)`,  `V(θ) = V_c + A_p·s(θ)`.
    ///
    /// The finite rod length makes the piston dwell longer near BDC than TDC; this
    /// asymmetry matters for pressure-trace shape and for the inertia forces.
    #[inline]
    pub(crate) fn volume(&self, theta: f32) -> f32 {
        let (s, c) = theta.sin_cos();
        let r = self.crank_radius;
        let l = self.conrod;
        let disp = r * (1.0 - c) + l - (l * l - r * r * s * s).max(0.0).sqrt();
        self.clearance_volume + self.piston_area * disp
    }

    /// Peak reciprocating inertia force at TDC of the exhaust stroke \[N\]:
    /// `F = m·r·ω²·(1 + r/l)` — first- plus second-order terms of piston acceleration.
    /// At gas-exchange TDC there is no gas pressure to oppose it, so this is pure rod
    /// tension and the governing load case for over-revving.
    #[inline]
    pub(crate) fn inertia_force(&self, reciprocating_mass: f32, omega: f32) -> f32 {
        reciprocating_mass
            * self.crank_radius
            * omega
            * omega
            * (1.0 + self.crank_radius / self.conrod)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_are_valid() {
        EngineSpec::naturally_aspirated_2l().validate().unwrap();
        EngineSpec::turbocharged_2l().validate().unwrap();
    }

    #[test]
    fn displacement_is_two_litres() {
        let d = EngineSpec::naturally_aspirated_2l()
            .geometry
            .displacement_m3();
        assert!((d - 1.998e-3).abs() < 2e-6, "{d}");
    }

    #[test]
    fn slider_crank_volume_limits() {
        let s = EngineSpec::naturally_aspirated_2l();
        let g = CylinderGeometry::from_spec(&s.geometry);
        assert!((g.volume(0.0) - g.clearance_volume).abs() < 1e-9);
        assert!((g.volume(PI) - (g.clearance_volume + g.swept_volume)).abs() < 1e-8);
        let cr = g.volume(PI) / g.volume(0.0);
        assert!((cr - 10.5).abs() < 1e-3);
    }

    #[test]
    fn bad_firing_order_rejected() {
        let mut s = EngineSpec::naturally_aspirated_2l();
        s.geometry.firing_order = [0, 0, 3, 1, 0, 0, 0, 0];
        assert_eq!(s.validate(), Err(SpecError::FiringOrder));
    }

    #[test]
    fn non_finite_parameters_are_rejected_everywhere() {
        let edits: [fn(&mut EngineSpec); 10] = [
            |s| s.friction.fmep_speed = f32::NAN,
            |s| s.friction.accessory_power_w = f32::NAN,
            |s| s.exhaust.backpressure = f32::NAN,
            |s| s.breathing.ram_gain = f32::INFINITY,
            |s| s.combustion.wiebe_m = f32::NAN,
            |s| s.throttle.idle_valve_area_m2 = f32::NAN,
            |s| s.thermal.thermostat_start_k = f32::NAN,
            |s| s.limits.rod_fatigue_exponent = f32::NAN,
            |s| s.lubrication.vogel_c = f32::NAN,
            |s| s.lubrication.relief_pressure_pa = 300_000.0,
        ];
        for edit in edits {
            let mut s = EngineSpec::naturally_aspirated_2l();
            edit(&mut s);
            assert!(s.validate().is_err());
        }
        let mut t = EngineSpec::turbocharged_2l();
        if let Some(turbo) = t.turbo.as_mut() {
            turbo.bearing_loss = f32::NAN;
        }
        assert!(t.validate().is_err());
    }
}

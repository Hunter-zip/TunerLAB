//! Telemetry snapshot published once per [`EngineSim::tick`](super::EngineSim::tick).
//!
//! `Copy` and allocation-free so it can be handed to the UI or audio thread through a
//! lock-free triple buffer. Fields without a "sensor" qualifier are *physical truth*
//! (instructor view); [`Telemetry::sensors`] is exactly what the ECU and a datalogger see.

use super::damage::HealthReport;
use super::ecu::EcuMode;
use super::sensors::SensorReadings;
use super::spec::MAX_CYLINDERS;
use super::status::{EngineCondition, Warnings};

/// Snapshot of the simulation state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Telemetry {
    /// Simulation time \[s\].
    pub time_s: f64,
    /// Operating condition.
    pub condition: EngineCondition,
    /// ECU operating mode.
    pub ecu_mode: EcuMode,
    /// Number of cylinders (valid prefix of the per-cylinder arrays).
    pub cylinders: usize,
    /// Engine speed \[rpm\].
    pub rpm: f32,
    /// Crank angle within the cycle [deg, 0..720).
    pub crank_angle_deg: f32,
    /// Pedal position \[%\].
    pub pedal_pct: f32,
    /// Throttle plate position \[%\].
    pub throttle_pct: f32,
    /// Idle air valve position \[%\].
    pub idle_valve_pct: f32,
    /// Manifold absolute pressure \[kPa\].
    pub manifold_pressure_kpa: f32,
    /// Pre-throttle (boost) absolute pressure \[kPa\].
    pub boost_pressure_kpa: f32,
    /// Exhaust manifold absolute pressure \[kPa\].
    pub exhaust_pressure_kpa: f32,
    /// Intake manifold air temperature \[°C\].
    pub intake_air_temp_c: f32,
    /// Coolant temperature \[°C\].
    pub coolant_temp_c: f32,
    /// Oil temperature \[°C\].
    pub oil_temp_c: f32,
    /// Engine metal temperature \[°C\].
    pub metal_temp_c: f32,
    /// Oil pressure \[kPa\].
    pub oil_pressure_kpa: f32,
    /// Fuel rail differential pressure \[kPa\].
    pub fuel_pressure_kpa: f32,
    /// Battery voltage \[V\].
    pub battery_v: f32,
    /// Coolant fill level 0..1.
    pub coolant_level: f32,
    /// Thermostat opening 0..1.
    pub thermostat_pos: f32,
    /// True exhaust λ (cycle average).
    pub lambda: f32,
    /// ECU λ target.
    pub lambda_target: f32,
    /// Exhaust gas temperature at the manifold \[°C\].
    pub egt_c: f32,
    /// Catalyst temperature \[°C\].
    pub catalyst_temp_c: f32,
    /// Mean brake torque \[N·m\].
    pub brake_torque_nm: f32,
    /// Mean indicated (gas) torque \[N·m\].
    pub indicated_torque_nm: f32,
    /// Mean friction torque \[N·m\].
    pub friction_torque_nm: f32,
    /// Mean pumping torque (signed) \[N·m\].
    pub pumping_torque_nm: f32,
    /// Instantaneous crank gas torque \[N·m\] (firing-pulse resolution, for audio/vibration).
    pub instantaneous_torque_nm: f32,
    /// Torque absorbed by the dynamometer or transmitted to the clutch \[N·m\].
    pub load_torque_nm: f32,
    /// Brake power \[kW\].
    pub power_kw: f32,
    /// True volumetric efficiency.
    pub volumetric_efficiency: f32,
    /// ECU VE table value at the current operating point.
    pub ve_table: f32,
    /// Air mass flow into the cylinders \[g/s\].
    pub air_flow_g_s: f32,
    /// Injected fuel mass flow \[g/s\].
    pub fuel_flow_g_s: f32,
    /// Mean commanded spark advance [deg BTDC].
    pub ignition_advance_deg: f32,
    /// Spark advance for maximum brake torque at this operating point [deg BTDC] (truth).
    pub mbt_advance_deg: f32,
    /// Mean injector pulse width \[ms\].
    pub injector_pulse_ms: f32,
    /// Injector duty cycle \[%\].
    pub injector_duty_pct: f32,
    /// Short-term fuel trim \[%\].
    pub stft_pct: f32,
    /// Long-term fuel trim \[%\].
    pub ltft_pct: f32,
    /// Closed-loop fuelling active.
    pub closed_loop: bool,
    /// Last-cycle knock intensity per cylinder \[bar\].
    pub knock_intensity: [f32; MAX_CYLINDERS],
    /// ECU knock retard per cylinder \[deg\].
    pub knock_retard_deg: [f32; MAX_CYLINDERS],
    /// Last-cycle peak cylinder pressure per cylinder \[bar\].
    pub peak_pressure_bar: [f32; MAX_CYLINDERS],
    /// Piston crown temperature per cylinder \[°C\].
    pub piston_temp_c: [f32; MAX_CYLINDERS],
    /// Last cycle misfired, per cylinder.
    pub cylinder_misfire: [bool; MAX_CYLINDERS],
    /// ECU misfire counters per cylinder.
    pub misfire_count: [u32; MAX_CYLINDERS],
    /// Turbocharger speed \[rpm\].
    pub turbo_rpm: f32,
    /// Wastegate opening \[%\].
    pub wastegate_pct: f32,
    /// ECU boost target [kPa abs].
    pub boost_target_kpa: f32,
    /// Compressor in surge.
    pub compressor_surge: bool,
    /// Vehicle speed \[km/h\].
    pub vehicle_speed_kph: f32,
    /// Active instructor warnings.
    pub warnings: Warnings,
    /// Component health.
    pub health: HealthReport,
    /// Number of stored trouble codes.
    pub dtc_count: usize,
    /// Sensor readings as seen by the ECU.
    pub sensors: SensorReadings,
}

impl Telemetry {
    pub(crate) fn blank(sensors: SensorReadings) -> Self {
        Self {
            time_s: 0.0,
            condition: EngineCondition::Off,
            ecu_mode: EcuMode::Off,
            cylinders: 0,
            rpm: 0.0,
            crank_angle_deg: 0.0,
            pedal_pct: 0.0,
            throttle_pct: 0.0,
            idle_valve_pct: 0.0,
            manifold_pressure_kpa: 0.0,
            boost_pressure_kpa: 0.0,
            exhaust_pressure_kpa: 0.0,
            intake_air_temp_c: 0.0,
            coolant_temp_c: 0.0,
            oil_temp_c: 0.0,
            metal_temp_c: 0.0,
            oil_pressure_kpa: 0.0,
            fuel_pressure_kpa: 0.0,
            battery_v: 0.0,
            coolant_level: 1.0,
            thermostat_pos: 0.0,
            lambda: 0.0,
            lambda_target: 0.0,
            egt_c: 0.0,
            catalyst_temp_c: 0.0,
            brake_torque_nm: 0.0,
            indicated_torque_nm: 0.0,
            friction_torque_nm: 0.0,
            pumping_torque_nm: 0.0,
            instantaneous_torque_nm: 0.0,
            load_torque_nm: 0.0,
            power_kw: 0.0,
            volumetric_efficiency: 0.0,
            ve_table: 0.0,
            air_flow_g_s: 0.0,
            fuel_flow_g_s: 0.0,
            ignition_advance_deg: 0.0,
            mbt_advance_deg: 0.0,
            injector_pulse_ms: 0.0,
            injector_duty_pct: 0.0,
            stft_pct: 0.0,
            ltft_pct: 0.0,
            closed_loop: false,
            knock_intensity: [0.0; MAX_CYLINDERS],
            knock_retard_deg: [0.0; MAX_CYLINDERS],
            peak_pressure_bar: [0.0; MAX_CYLINDERS],
            piston_temp_c: [0.0; MAX_CYLINDERS],
            cylinder_misfire: [false; MAX_CYLINDERS],
            misfire_count: [0; MAX_CYLINDERS],
            turbo_rpm: 0.0,
            wastegate_pct: 0.0,
            boost_target_kpa: 0.0,
            compressor_surge: false,
            vehicle_speed_kph: 0.0,
            warnings: Warnings::NONE,
            health: HealthReport::default(),
            dtc_count: 0,
            sensors,
        }
    }
}

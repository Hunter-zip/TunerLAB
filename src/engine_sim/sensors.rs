//! Sensor layer: converts physical truth into what an ECU (and a datalogger) can measure.
//!
//! Each channel has realistic dynamics (thermistor and thermocouple lags, exhaust transport
//! delay, MAP filtering), noise and fault hooks. Diagnosis in Engine Autopsy is performed
//! exclusively on these readings.

use super::controls::Controls;
use super::faults::FaultState;
use super::math::{approach, clampf, Pcg32};
use super::plant::Plant;
use super::spec::MAX_CYLINDERS;
use super::thermo::ZERO_CELSIUS_K;

/// Wide-band O2 transport-delay buffer length (samples).
const LAMBDA_BUFFER: usize = 128;
/// Sample period of the transport-delay buffer \[s\]; 128 × 2 ms = 256 ms maximum delay.
const LAMBDA_SAMPLE_S: f32 = 0.002;
/// Wide-band sensor cell + controller response time constant \[s\] (LSU 4.9 ≈ 60–80 ms).
const LAMBDA_TAU_S: f32 = 0.06;
/// Heater warm-up time before the Nernst cell reaches its 780 °C operating point \[s\].
const LAMBDA_HEATER_S: f32 = 12.0;
/// Coolant thermistor time constant (brass-sleeved NTC in flowing coolant) \[s\].
const ECT_TAU_S: f32 = 3.0;
/// Intake air thermistor time constant (open-element NTC) \[s\].
const IAT_TAU_S: f32 = 2.0;
/// Oil temperature sender time constant \[s\].
const OIL_TEMP_TAU_S: f32 = 5.0;
/// 3 mm sheathed K-type exhaust thermocouple time constant \[s\].
const EGT_TAU_S: f32 = 0.7;
/// MAP sensor + ECU anti-alias filter time constant \[s\].
const MAP_TAU_S: f32 = 0.003;
/// Open-circuit IAT reading: the ECU's pull-up drives the input to the cold rail.
const IAT_OPEN_READING_C: f32 = -40.0;

/// Everything the ECU can measure, in engineering units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorReadings {
    /// Engine speed from the crank sensor \[rpm\].
    pub rpm: f32,
    /// Manifold absolute pressure \[kPa\].
    pub map_kpa: f32,
    /// Intake air temperature \[°C\].
    pub iat_c: f32,
    /// Engine coolant temperature \[°C\].
    pub ect_c: f32,
    /// Oil temperature \[°C\].
    pub oil_temp_c: f32,
    /// Oil pressure (gauge) \[kPa\].
    pub oil_pressure_kpa: f32,
    /// Fuel rail differential pressure \[kPa\].
    pub fuel_pressure_kpa: f32,
    /// Throttle position \[%\].
    pub throttle_pct: f32,
    /// Accelerator pedal position \[%\].
    pub pedal_pct: f32,
    /// Wide-band λ reading.
    pub lambda: f32,
    /// Wide-band sensor at operating temperature.
    pub lambda_ready: bool,
    /// Exhaust gas temperature probe \[°C\].
    pub egt_c: f32,
    /// Battery voltage \[V\].
    pub battery_v: f32,
    /// Vehicle speed \[km/h\].
    pub vehicle_speed_kph: f32,
    /// Knock sensor signal, windowed per cylinder \[V-equivalent\].
    pub knock_signal: [f32; MAX_CYLINDERS],
    /// Crank segment sequence counter (increments once per firing TDC).
    pub segment_seq: u32,
    /// Cylinder whose power stroke ended at the last firing TDC.
    pub segment_cylinder: u8,
    /// Crank speed measured at the last firing TDC \[rad/s\].
    pub segment_omega: f32,
    /// Camshaft phase deviation from nominal [crank deg].
    pub cam_phase_error_deg: f32,
    /// Rear (post-catalyst) O2 switching activity 0..1 (1 = catalyst not storing oxygen).
    pub rear_o2_activity: f32,
    /// Turbocharger speed sensor \[rpm\] (0 if not fitted).
    pub turbo_rpm: f32,
}

impl SensorReadings {
    fn initial(ambient_k: f32, baro_kpa: f32) -> Self {
        let amb_c = ambient_k - ZERO_CELSIUS_K;
        Self {
            rpm: 0.0,
            map_kpa: baro_kpa,
            iat_c: amb_c,
            ect_c: amb_c,
            oil_temp_c: amb_c,
            oil_pressure_kpa: 0.0,
            fuel_pressure_kpa: 0.0,
            throttle_pct: 0.0,
            pedal_pct: 0.0,
            lambda: 0.0,
            lambda_ready: false,
            egt_c: amb_c,
            battery_v: 12.6,
            vehicle_speed_kph: 0.0,
            knock_signal: [0.0; MAX_CYLINDERS],
            segment_seq: 0,
            segment_cylinder: 0,
            segment_omega: 0.0,
            cam_phase_error_deg: 0.0,
            rear_o2_activity: 1.0,
            turbo_rpm: 0.0,
        }
    }
}

/// Sensor dynamics state.
#[derive(Debug, Clone)]
pub(crate) struct SensorState {
    pub readings: SensorReadings,
    lambda_buf: [f32; LAMBDA_BUFFER],
    buf_head: usize,
    buf_timer: f32,
    lambda_cell: f32,
    heater_s: f32,
    last_events: [u32; MAX_CYLINDERS],
}

impl SensorState {
    pub(crate) fn new(ambient_k: f32, baro_pa: f32) -> Self {
        Self {
            readings: SensorReadings::initial(ambient_k, baro_pa * 1.0e-3),
            lambda_buf: [3.0; LAMBDA_BUFFER],
            buf_head: 0,
            buf_timer: 0.0,
            lambda_cell: 3.0,
            heater_s: 0.0,
            last_events: [0; MAX_CYLINDERS],
        }
    }

    /// Snaps the slow temperature channels to their steady-state readings (used after
    /// [`Plant::prewarm`], when the sensors would long since have equilibrated).
    pub(crate) fn settle(&mut self, plant: &Plant, faults: &FaultState) {
        let r = &mut self.readings;
        if !faults.iat_open_circuit {
            r.iat_c = plant.air.t_man - ZERO_CELSIUS_K;
        }
        r.ect_c = plant.thermal.t_coolant + faults.coolant_sensor_bias_k - ZERO_CELSIUS_K;
        r.oil_temp_c = plant.thermal.t_oil - ZERO_CELSIUS_K;
    }

    /// Samples the plant.
    pub(crate) fn update(
        &mut self,
        plant: &Plant,
        ctl: &Controls,
        faults: &FaultState,
        dt: f32,
        rng: &mut Pcg32,
    ) {
        let r = &mut self.readings;
        let rpm_true = plant.rpm();
        // Crank sensor: 60-2 tooth wheel speed averaged over a few teeth.
        r.rpm = approach(r.rpm, rpm_true, dt, 0.008);

        let map_true = plant.air.p_man + faults.map_bias_pa;
        let map_noisy = map_true * 1.0e-3 + 0.15 * rng.normal();
        // 3-bar sensor transfer function saturates at its rails.
        r.map_kpa = clampf(approach(r.map_kpa, map_noisy, dt, MAP_TAU_S), 10.0, 300.0);

        r.iat_c = if faults.iat_open_circuit {
            IAT_OPEN_READING_C
        } else {
            approach(r.iat_c, plant.air.t_man - ZERO_CELSIUS_K, dt, IAT_TAU_S)
        };
        // With the coolant level below the sensor the thermistor sits in steam/air and
        // reads far too cold — a classic trap when diagnosing overheating.
        let ect_true = if plant.thermal.coolant_level < 0.25 {
            plant.thermal.t_coolant - 40.0
        } else {
            plant.thermal.t_coolant
        };
        r.ect_c = approach(
            r.ect_c,
            ect_true + faults.coolant_sensor_bias_k - ZERO_CELSIUS_K,
            dt,
            ECT_TAU_S,
        );
        r.oil_temp_c = approach(
            r.oil_temp_c,
            plant.thermal.t_oil - ZERO_CELSIUS_K,
            dt,
            OIL_TEMP_TAU_S,
        );
        r.oil_pressure_kpa = approach(r.oil_pressure_kpa, plant.oil_pressure_pa * 1.0e-3, dt, 0.05);
        r.fuel_pressure_kpa = approach(r.fuel_pressure_kpa, plant.fuel_rail_pa * 1.0e-3, dt, 0.05);
        r.throttle_pct = plant.air.throttle_pos * 100.0;
        r.pedal_pct = ctl.pedal * 100.0;
        r.egt_c = approach(r.egt_c, plant.air.t_exh - ZERO_CELSIUS_K, dt, EGT_TAU_S);
        r.battery_v = plant.battery_v + 0.02 * rng.normal();
        r.vehicle_speed_kph = plant.crank.vehicle_speed * 3.6;
        r.turbo_rpm = plant.air.turbo_omega * super::math::RAD_S_TO_RPM;

        // Wide-band λ: gas travels from the exhaust valve to the sensor (transport delay
        // ≈ half a cycle plus pipe volume / flow), then the cell responds with a lag.
        self.buf_timer += dt;
        while self.buf_timer >= LAMBDA_SAMPLE_S {
            self.buf_timer -= LAMBDA_SAMPLE_S;
            self.buf_head = (self.buf_head + 1) % LAMBDA_BUFFER;
            self.lambda_buf[self.buf_head] = plant.exhaust_lambda_apparent;
        }
        let delay_s = clampf(
            60.0 / rpm_true.max(100.0) + 0.015 + 0.0004 / plant.exhaust_mass_flow.max(0.002),
            0.02,
            (LAMBDA_BUFFER - 1) as f32 * LAMBDA_SAMPLE_S,
        );
        let lag_samples = (delay_s / LAMBDA_SAMPLE_S) as usize;
        let idx = (self.buf_head + LAMBDA_BUFFER - lag_samples) % LAMBDA_BUFFER;
        let delayed = self.lambda_buf[idx];
        self.lambda_cell = approach(
            self.lambda_cell,
            delayed,
            dt,
            LAMBDA_TAU_S * faults.o2_lag_factor,
        );
        if ctl.ignition_on {
            self.heater_s += dt;
        } else {
            self.heater_s = 0.0;
        }
        r.lambda_ready = self.heater_s > LAMBDA_HEATER_S;
        r.lambda = if r.lambda_ready {
            clampf(
                self.lambda_cell + faults.o2_bias_lambda + 0.003 * rng.normal(),
                0.5,
                3.0,
            )
        } else {
            0.0
        };

        // Knock sensor: accelerometer band-pass energy in each cylinder's angular window.
        // Mechanical background (valve seating, piston slap) grows with speed squared.
        let background = 0.15 + 0.35 * (rpm_true / 6000.0).powi(2);
        for c in 0..plant.cylinders {
            let ev = plant.cyl[c].events;
            if ev != self.last_events[c] {
                self.last_events[c] = ev;
                r.knock_signal[c] = if faults.knock_sensor_dead {
                    0.02 * rng.normal().abs()
                } else {
                    ((plant.cyl[c].last.knock_intensity + background) * (1.0 + 0.08 * rng.normal())
                        + 0.03 * rng.normal())
                    .max(0.0)
                };
            }
        }

        // Crank segment timing with tooth-edge jitter.
        if plant.segment.seq != r.segment_seq {
            r.segment_seq = plant.segment.seq;
            r.segment_cylinder = plant.segment.cylinder;
            r.segment_omega = plant.segment.omega * (1.0 + 2.0e-4 * rng.normal());
        }

        r.cam_phase_error_deg = approach(
            r.cam_phase_error_deg,
            faults.cam_retard_deg + 0.2 * rng.normal(),
            dt,
            0.1,
        );
        // A healthy, lit-off catalyst stores oxygen and flattens the rear O2 signal.
        r.rear_o2_activity = approach(r.rear_o2_activity, 1.0 - plant.catalyst_efficiency, dt, 2.0);
    }
}

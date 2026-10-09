//! Crankshaft rotational dynamics and external loads (dynamometer, vehicle).
//!
//! `J_eff · dω/dt = T_gas + T_pump + T_starter − T_friction − T_accessory − T_load`
//!
//! integrated with semi-implicit (symplectic) Euler: ω is updated first and the new ω
//! advances the crank angle, which conserves the energy of the periodic gas-torque
//! oscillation instead of slowly pumping energy in as explicit Euler would.
//!
//! A dynamometer absorber is a second inertia joined to the crank by a torsionally soft
//! coupling shaft (two-mass system). At firing frequencies (> 25 Hz) the absorber is
//! dynamically decoupled, exactly as on a real test bed, so crank-speed fluctuations and
//! misfire signatures stay representative.
//!
//! In a vehicle the crank drives the wheels through two compliances in series: the clutch
//! disc's torsional damper springs (crank side) and the half-shafts plus tyres (wheel
//! side, which appears 1/i² softer at the crank). The driveline is a damped link of that
//! combined stiffness whose torque is limited by the clutch's friction capacity
//! (elasto-plastic, Dahl-type friction). It reproduces the 2–4 Hz first-gear "shuffle"
//! mode calibrators tune anti-jerk against, and it isolates the crank's firing-frequency
//! speed ripple from the vehicle inertia, which is what lets an OBD misfire monitor work
//! in gear.

use super::controls::LoadModel;
use super::math::{approach, clampf, wrap_cycle, RAD_S_TO_RPM, RPM_TO_RAD_S};
use super::thermo::GRAVITY;

/// Torsional stiffness of the clutch-disc damper springs plus gearbox input shaft, at the
/// crank \[N·m/rad\] (≈ 10 N·m/deg, typical of passenger-car clutch dampers).
const CLUTCH_DAMPER_STIFFNESS: f32 = 600.0;
/// Torsional stiffness of both half-shafts and the tyres, at the wheels \[N·m/rad\].
const WHEEL_SIDE_STIFFNESS: f32 = 10_000.0;
/// Damping ratio of the driveline mode (damper hysteresis, tyre damping).
const DRIVELINE_DAMPING_RATIO: f32 = 0.2;
/// Relaxation time of driveline wind-up held against a stationary end \[s\]: a stalled
/// crank rocks back, a stopped car settles, instead of storing the spring torque forever.
const DRIVELINE_RELAX_TAU_S: f32 = 0.02;
/// Tyre–road friction coefficient for full brake application.
const BRAKE_MU: f32 = 0.95;
/// Below this speed the engine is treated as stationary for static-friction purposes \[rad/s\].
const STICTION_OMEGA: f32 = 0.5;
/// Torsional stiffness of the dyno coupling shaft \[N·m/rad\]: with 0.16 kg·m² engine and
/// 0.25 kg·m² absorber inertia the first torsional mode sits at ≈ 15 Hz, the usual design
/// target for elastomer test-bed couplings (well below idle firing frequency).
const COUPLING_STIFFNESS: f32 = 900.0;
/// Viscous damping of the dyno coupling \[N·m·s/rad\] (damping ratio ≈ 0.15).
const COUPLING_DAMPING: f32 = 3.0;

/// Crank and load state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CrankState {
    /// Crank angle within the four-stroke cycle [rad, 0..4π).
    pub theta: f32,
    /// Angular velocity \[rad/s\].
    pub omega: f32,
    pub dyno_integral: f32,
    /// Torque absorbed by the dynamometer \[N·m\].
    pub dyno_torque: f32,
    /// Vehicle speed \[m/s\].
    pub vehicle_speed: f32,
    /// Torque transmitted through the clutch \[N·m\].
    pub clutch_torque: f32,
    /// Elastic torque stored in the driveline springs \[N·m\] (positive: engine drives).
    pub driveline_spring_nm: f32,
    /// Measured load torque: dyno load cell or clutch torque \[N·m\].
    pub load_torque: f32,
    /// Absorber rotor speed \[rad/s\].
    pub dyno_omega: f32,
    /// Coupling shaft twist \[rad\].
    pub coupling_twist: f32,
    /// The dyno coupling is engaged (state initialised).
    pub dyno_engaged: bool,
}

impl Default for CrankState {
    fn default() -> Self {
        Self {
            theta: 0.0,
            omega: 0.0,
            dyno_integral: 0.0,
            dyno_torque: 0.0,
            vehicle_speed: 0.0,
            clutch_torque: 0.0,
            driveline_spring_nm: 0.0,
            load_torque: 0.0,
            dyno_omega: 0.0,
            coupling_twist: 0.0,
            dyno_engaged: false,
        }
    }
}

/// Torques acting on the crank this step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CrankInputs {
    /// Sum of instantaneous cylinder gas torques (signed) \[N·m\].
    pub gas_torque: f32,
    /// Gas-exchange (pumping) torque, signed: negative when exhaust > intake pressure.
    pub pumping_torque: f32,
    /// Friction magnitude \[N·m\].
    pub friction_torque: f32,
    /// Accessory drag magnitude \[N·m\].
    pub accessory_torque: f32,
    /// Starter motor torque \[N·m\].
    pub starter_torque: f32,
    /// Rotating inertia of the engine \[kg·m²\].
    pub inertia: f32,
    /// Ambient air density \[kg/m³\].
    pub air_density: f32,
    /// Engine mechanically seized.
    pub seized: bool,
}

/// Advances crank and load by one step.
pub(crate) fn step(st: &mut CrankState, inp: &CrankInputs, load: &LoadModel, dt: f32) {
    let omega = st.omega;
    if !matches!(load, LoadModel::Dyno(_)) {
        st.dyno_engaged = false;
    }
    // (torque acting on the crank, measured load torque)
    let (t_crank, t_measured) = match load {
        LoadModel::Neutral => {
            st.clutch_torque = 0.0;
            st.driveline_spring_nm = 0.0;
            st.dyno_torque = 0.0;
            (0.0, 0.0)
        }
        LoadModel::Dyno(d) => {
            if !st.dyno_engaged {
                st.dyno_engaged = true;
                st.dyno_omega = omega;
                st.coupling_twist = 0.0;
            }
            // Eddy-current absorber with PI speed control on its own rotor speed; it can
            // brake but not motor.
            let err = st.dyno_omega * RAD_S_TO_RPM - d.target_rpm;
            st.dyno_integral = clampf(
                st.dyno_integral + d.ki_nm_per_rpm_s * err * dt,
                0.0,
                d.max_torque_nm,
            );
            st.clutch_torque = 0.0;
            st.driveline_spring_nm = 0.0;
            let (t_coupling, t_abs) = if d.absorber_inertia_kg_m2 > 1.0e-3 {
                // Two-mass system: T_c = k·φ + c·(ω_e − ω_a), J_a·dω_a/dt = T_c − T_abs, with
                // T_abs = k_p·(ω_a − ω_target) + I. Both the coupling damping and the
                // proportional absorber term act on ω_a, so ω_a is advanced implicitly:
                //   ω_a⁺ = (ω_a + h·(k·φ + c·ω_e − I + k_p·ω_t)) / (1 + h·(k_p + c)), h = dt/J_a,
                // stable for any absorber inertia and gain (explicit Euler is not once
                // dt·(k_p + c)/J_a > 2). When T_abs saturates it is a constant instead.
                let h = dt / d.absorber_inertia_kg_m2;
                let kp = d.kp_nm_per_rpm.max(0.0) * RAD_S_TO_RPM;
                let omega_t = d.target_rpm * RPM_TO_RAD_S;
                let spring = COUPLING_STIFFNESS * st.coupling_twist;
                let free = st.dyno_omega + h * (spring + COUPLING_DAMPING * omega);
                let mut w = (free + h * (kp * omega_t - st.dyno_integral))
                    / (1.0 + h * (kp + COUPLING_DAMPING));
                let mut t_abs = kp * (w - omega_t) + st.dyno_integral;
                if !(0.0..=d.max_torque_nm).contains(&t_abs) {
                    t_abs = clampf(t_abs, 0.0, d.max_torque_nm);
                    w = (free - h * t_abs) / (1.0 + h * COUPLING_DAMPING);
                }
                st.dyno_omega = w.max(0.0);
                (spring + COUPLING_DAMPING * (omega - st.dyno_omega), t_abs)
            } else {
                // Massless absorber: rigidly coupled.
                st.dyno_omega = omega;
                let t_abs = clampf(
                    d.kp_nm_per_rpm * err + st.dyno_integral,
                    0.0,
                    d.max_torque_nm,
                );
                (t_abs, t_abs)
            };
            st.dyno_torque = t_abs;
            (t_coupling, t_abs)
        }
        LoadModel::Vehicle(v) => {
            let gear = usize::from(v.gear);
            let ratio = if gear == 0 {
                0.0
            } else {
                v.gear_ratios[gear - 1] * v.final_drive
            };
            let t_cl = if gear == 0 {
                st.driveline_spring_nm = 0.0;
                0.0
            } else {
                // Series stiffness referred to the crank: k = 1/(1/k_clutch + i²/k_wheel).
                // The referred vehicle inertia J_v = m·r²/i² and the crank inertia set the
                // damping for a constant damping ratio, c = 2ζ·√(k·J_red). Both the mode
                // frequency (≤ √(k_wheel/(m·r²)) ≤ 100 rad/s for any sanitised vehicle) and
                // c·dt/J stay far inside the explicit-integration limits.
                let i2 = ratio * ratio;
                let k = 1.0 / (1.0 / CLUTCH_DAMPER_STIFFNESS + i2 / WHEEL_SIDE_STIFFNESS);
                let j_v = v.mass_kg * v.wheel_radius_m * v.wheel_radius_m / i2;
                let j_e = inp.inertia.max(1.0e-3);
                let c = 2.0 * DRIVELINE_DAMPING_RATIO * (k * j_e * j_v / (j_e + j_v)).sqrt();
                // The springs wind up with the speed difference across the driveline until
                // their torque reaches the clutch's friction capacity, beyond which the
                // clutch slips (elasto-plastic friction).
                let omega_trans = st.vehicle_speed * ratio / v.wheel_radius_m;
                let slip = omega - omega_trans;
                let capacity = (v.clutch_capacity_nm * v.clutch).max(0.0);
                st.driveline_spring_nm =
                    clampf(st.driveline_spring_nm + k * slip * dt, -capacity, capacity);
                clampf(st.driveline_spring_nm + c * slip, -capacity, capacity)
            };
            st.clutch_torque = t_cl;
            st.dyno_torque = 0.0;

            // Longitudinal vehicle: m·dv/dt = F_drive − F_aero − F_roll − F_brake − F_grade.
            let eff = if t_cl >= 0.0 {
                v.driveline_efficiency
            } else {
                1.0 / v.driveline_efficiency
            };
            let f_drive = t_cl * ratio * eff / v.wheel_radius_m;
            let speed = st.vehicle_speed;
            let f_aero = 0.5 * inp.air_density * v.drag_area_m2 * speed * speed;
            let normal = v.mass_kg * GRAVITY * v.grade_rad.cos();
            let f_resist = v.rolling_coefficient * normal + v.brake * BRAKE_MU * normal;
            let f_grade = v.mass_kg * GRAVITY * v.grade_rad.sin();
            let net_push = f_drive - f_grade;
            if speed < 0.05 && net_push.abs() <= f_resist {
                st.vehicle_speed = 0.0;
            } else {
                let accel = (net_push - f_resist - f_aero) / v.mass_kg;
                st.vehicle_speed = (speed + accel * dt).max(0.0);
            }
            (t_cl, t_cl)
        }
    };
    st.load_torque = t_measured;

    if inp.seized {
        st.omega = approach(omega, 0.0, dt, 0.02);
    } else {
        let drive = inp.gas_torque + inp.pumping_torque + inp.starter_torque - t_crank;
        let resist = inp.friction_torque.max(0.0) + inp.accessory_torque.max(0.0);
        let j = inp.inertia;
        if omega <= STICTION_OMEGA && drive.abs() <= resist {
            // Static friction holds the crank.
            st.omega = 0.0;
        } else {
            // Kinetic friction always opposes forward rotation; the crank cannot run
            // backwards (in reality the engine rocks back by a fraction of a revolution).
            st.omega = (omega + (drive - resist) / j * dt).max(0.0);
        }
    }
    if st.dyno_engaged {
        st.coupling_twist += (st.omega - st.dyno_omega) * dt;
    }
    // Wind-up that pushes a stationary end the way it cannot move (crank backwards, car
    // backwards) is released: the crank rocks back, the car settles on its tyres.
    let spring = st.driveline_spring_nm;
    if (spring > 0.0 && st.omega <= 0.0) || (spring < 0.0 && st.vehicle_speed <= 0.0) {
        st.driveline_spring_nm = approach(spring, 0.0, dt, DRIVELINE_RELAX_TAU_S);
    }
    st.theta = wrap_cycle(st.theta + st.omega * dt);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine_sim::controls::{DynoParams, VehicleParams};

    fn inputs(gas: f32) -> CrankInputs {
        CrankInputs {
            gas_torque: gas,
            pumping_torque: 0.0,
            friction_torque: 10.0,
            accessory_torque: 2.0,
            starter_torque: 0.0,
            inertia: 0.16,
            air_density: 1.2,
            seized: false,
        }
    }

    #[test]
    fn friction_holds_engine_at_rest() {
        let mut st = CrankState::default();
        step(&mut st, &inputs(5.0), &LoadModel::Neutral, 1e-3);
        assert_eq!(st.omega, 0.0);
    }

    #[test]
    fn dyno_holds_target_speed() {
        let mut st = CrankState {
            omega: 2000.0 / RAD_S_TO_RPM,
            ..CrankState::default()
        };
        let load = LoadModel::Dyno(DynoParams {
            target_rpm: 3000.0,
            ..DynoParams::default()
        });
        for _ in 0..40_000 {
            step(&mut st, &inputs(150.0), &load, 2.5e-4);
        }
        let rpm = st.omega * RAD_S_TO_RPM;
        assert!((rpm - 3000.0).abs() < 20.0, "{rpm}");
        assert!((st.dyno_torque - 138.0).abs() < 5.0, "{}", st.dyno_torque);
    }

    #[test]
    fn light_absorber_with_high_gain_stays_stable() {
        // dt·k_p/J_a ≈ 2.5e-4 · 477 / 0.002 ≈ 60: explicit integration would diverge.
        let mut st = CrankState {
            omega: 3000.0 / RAD_S_TO_RPM,
            ..CrankState::default()
        };
        let load = LoadModel::Dyno(DynoParams {
            target_rpm: 3000.0,
            kp_nm_per_rpm: 50.0,
            absorber_inertia_kg_m2: 0.002,
            ..DynoParams::default()
        });
        let mut min_t = f32::MAX;
        let mut max_t = f32::MIN;
        for i in 0..40_000 {
            step(&mut st, &inputs(150.0), &load, 2.5e-4);
            if i > 30_000 {
                min_t = min_t.min(st.dyno_torque);
                max_t = max_t.max(st.dyno_torque);
            }
        }
        assert!(max_t - min_t < 5.0, "absorber torque {min_t}..{max_t}");
        assert!((st.omega * RAD_S_TO_RPM - 3000.0).abs() < 20.0);
    }

    #[test]
    fn vehicle_accelerates_in_gear() {
        let mut st = CrankState {
            omega: 1500.0 / RAD_S_TO_RPM,
            ..CrankState::default()
        };
        let load = LoadModel::Vehicle(VehicleParams {
            gear: 1,
            ..VehicleParams::default()
        });
        for _ in 0..20_000 {
            step(&mut st, &inputs(150.0), &load, 2.5e-4);
        }
        assert!(st.vehicle_speed > 3.0, "{}", st.vehicle_speed);
        // Clutch locked: engine speed tracks road speed through the gearing.
        let expected = st.vehicle_speed * 3.58 * 4.06 / 0.316;
        assert!((st.omega - expected).abs() < 3.0);
    }

    #[test]
    fn very_strong_clutch_stays_stable() {
        // The damper spring, not the friction capacity, sets the link stiffness, so even a
        // 5000 N·m race clutch neither chatters nor drags the car.
        let mut st = CrankState {
            omega: 1500.0 / RAD_S_TO_RPM,
            vehicle_speed: 1500.0 / RAD_S_TO_RPM * 0.316 / (3.58 * 4.06),
            ..CrankState::default()
        };
        let load = LoadModel::Vehicle(VehicleParams {
            gear: 1,
            clutch_capacity_nm: 5000.0,
            ..VehicleParams::default()
        });
        let mut peak = 0.0_f32;
        for i in 0..20_000 {
            step(&mut st, &inputs(150.0), &load, 2.5e-4);
            if i > 4000 {
                peak = peak.max(st.clutch_torque.abs());
            }
        }
        assert!(peak < 300.0, "clutch torque peak {peak}");
        assert!(st.vehicle_speed > 3.0, "{}", st.vehicle_speed);
    }

    #[test]
    fn stalled_engine_in_gear_does_not_keep_driveline_wind_up() {
        let mut st = CrankState {
            omega: 0.0,
            driveline_spring_nm: 200.0,
            ..CrankState::default()
        };
        let load = LoadModel::Vehicle(VehicleParams {
            gear: 1,
            brake: 1.0,
            ..VehicleParams::default()
        });
        let mut dead = inputs(0.0);
        dead.friction_torque = 30.0;
        for _ in 0..4000 {
            step(&mut st, &dead, &load, 2.5e-4);
        }
        assert!(
            st.driveline_spring_nm.abs() < 1.0,
            "{}",
            st.driveline_spring_nm
        );
        assert_eq!(st.vehicle_speed, 0.0);
    }

    #[test]
    fn first_gear_shuffle_mode_is_a_few_hertz() {
        // Free oscillation of the crank against the referred vehicle after a torque step.
        let speed = 1500.0 / RAD_S_TO_RPM * 0.316 / (3.58 * 4.06);
        let mut st = CrankState {
            omega: 1500.0 / RAD_S_TO_RPM,
            vehicle_speed: speed,
            ..CrankState::default()
        };
        let load = LoadModel::Vehicle(VehicleParams {
            gear: 1,
            ..VehicleParams::default()
        });
        // Period between the first two torque peaks of the (decaying) oscillation.
        let mut peaks = Vec::new();
        let (mut prev2, mut prev1) = (0.0_f32, 0.0_f32);
        for i in 0..8000 {
            step(&mut st, &inputs(150.0), &load, 2.5e-4);
            let x = st.clutch_torque;
            if i > 2 && prev1 > prev2 && prev1 >= x {
                peaks.push(i as f32 * 2.5e-4);
            }
            prev2 = prev1;
            prev1 = x;
        }
        assert!(peaks.len() >= 2, "{peaks:?}");
        let f = 1.0 / (peaks[1] - peaks[0]);
        assert!((2.0..4.5).contains(&f), "shuffle {f} Hz");
    }

    #[test]
    fn light_vehicle_with_tall_gearing_is_stable() {
        let mut st = CrankState {
            omega: 3000.0 / RAD_S_TO_RPM,
            ..CrankState::default()
        };
        let load = LoadModel::Vehicle(VehicleParams {
            mass_kg: 100.0,
            wheel_radius_m: 0.1,
            gear_ratios: [10.0; 6],
            final_drive: 15.0,
            gear: 1,
            ..VehicleParams::default()
        });
        let mut flips = 0;
        let mut prev = 0.0_f32;
        for i in 0..8000 {
            step(&mut st, &inputs(60.0), &load, 2.5e-4);
            if i > 4000 && st.clutch_torque * prev < 0.0 {
                flips += 1;
            }
            prev = st.clutch_torque;
        }
        assert!(flips < 10, "{flips} sign flips");
        assert!(st.vehicle_speed > 0.5, "{}", st.vehicle_speed);
    }
}

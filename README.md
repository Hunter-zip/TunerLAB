# TunerLAB 🚗💻

> **TunerLAB** is a high-fidelity, interactive ECU Tuning Simulator and Educational Platform designed to bridge the gap between virtual engine calibration and real-world calibration engineering.

![Platforms](https://img.shields.io/badge/platform-Windows%2010%2F11%20%7C%20Linux-blue)
![Language](https://img.shields.io/badge/language-Rust-orange)
![GUI](https://img.shields.io/badge/GUI-egui-green)
![i18n](https://img.shields.io/badge/i18n-PL%20%7C%20EN-red)

---

## 💡 Project Vision

Real-world internal combustion engine tuning has a steep learning curve and carries a massive risk of catastrophic engine failure. **TunerLAB** serves as a deterministic, safe sandbox. Built natively in **Rust** using the **`egui` framework**, it combines a real-time math-based thermodynamics engine, procedural audio synthesis, and an interactive structural course to take car enthusiasts from zero knowledge to being real-world ready tuners.

Ultimately, TunerLAB aims to recreate the structural behavior, datalogs, and user interface workflows of the world's most popular tuning systems (such as WinOLS, HP Tuners, and Link/Haltech Standalone suites).

---

## 🛠️ Tech Stack & Key Features

- **Core Engine:** Written in pure, modern **Rust** with an allocation-free, lock-free simulation loop (verified by a counting-allocator test), ready to be driven from a real-time audio thread.
- **Physics Simulation:** Crank-angle-resolved combustion, compressible air path, turbocharging, knock, thermal and damage models running at 4 kHz, ≈ 100× faster than real time.
- **Cross-Platform & Lightweight:** Windows 10/11 and Linux, OpenGL/Vulkan rendering via `egui` (Phase 3).
- **Localization (i18n):** **English (EN)** and **Polish (PL)** for every status, warning, trouble code, failure and fault from day one.
- **Business Model (Freemium):** Entry-level concepts and naturally aspirated tuning are free. Advanced modules (forced induction, launch control, full calibration workflows) are unlocked via a premium course tier.

---

## 📈 Roadmap

- [x] **Phase 1 (Current):** Core Simulation Engine (thermodynamic math loop & fail-safe architecture).
- [ ] **Phase 2:** Procedural Audio Engine (low-latency waveform synthesis tracking live RPM/load changes).
- [ ] **Phase 3:** Core UI Integration & bilingual multi-language support framework.
- [ ] **Phase 4:** ECU Interface Emulation (WinOLS hex-matrix style, OEM tabbed-grid style, motorsport standalone gauge style).
- [ ] **Phase 5:** Interactive Master Course (lessons, diagnostic tasks, Engine Autopsy crash logs, monetization).

---

## ⚙️ Phase 1 — Engine Simulation Core

`tunerlab_core::engine_sim` mirrors a real vehicle in four layers:

```text
Controls ──► Plant (physical truth) ──► Sensors ──► ECU ──► Actuators ──► Plant …
                 │                                    ▲
                 └── Damage model ◄── physical loads   └── user-editable Calibration
Faults (Engine Autopsy) inject into plant and sensors; the ECU is never told.
```

| Layer | Models |
|---|---|
| **Air path** | Compressible isentropic throttle/orifice flow, manifold filling–emptying (linearly-implicit integration), air filter, vacuum/boost leaks |
| **Turbo** | Compressor speed lines with surge, efficiency island, intercooler, turbine with blade-speed-ratio efficiency, pneumatic wastegate + bleed solenoid, blow-off valve, rotor dynamics |
| **Breathing** | Wave ram tuning, overlap reversion, Taylor inlet Mach index choking, residual-gas expansion, charge heating, cam phasing, valve float |
| **Combustion** | Slider-crank kinematics, Wiebe heat release, Brunt γ(T), Woschni heat transfer, Metghalchi–Keck flame speed, Paschen spark breakdown, Livengood–Wu + Douaud–Eyzat knock integral, cycle-to-cycle variation, exhaust energy balance. MBT, knock limits and lean/rich misfire limits are **emergent**, not tables |
| **Dynamics** | Per-cylinder gas-torque pulses, Chen–Flynn friction with oil viscosity, starter, two-mass dynamometer, vehicle with clutch and gearbox |
| **Thermal** | Metal, coolant (thermostat, radiator ε–NTU, boil-over), oil (Vogel viscosity, pressure, fuel dilution), piston crowns, exhaust manifold, catalyst with exotherm |
| **Sensors** | MAP, IAT, ECT, oil, fuel rail, wide-band λ with transport delay, EGT, knock, crank TDC speed, cam phase, rear O2 |
| **ECU** | Speed-density, inverse X–τ transient fuel, warm-up/after-start/cranking fuel, closed-loop STFT/LTFT, idle PI + idle spark, per-cylinder knock control, rev limiter, DFCO, boost PID, OBD-II misfire monitor and 25 SAE J2012 trouble codes |
| **Damage** | Knock erosion, piston crown melting, Basquin/Miner rod fatigue, rod buckling, valve-to-piston contact, head gasket, head warp, bearing wear, turbo overspeed, catalyst meltdown |
| **Faults** | 17 hidden Engine Autopsy faults (sensor bias, slow O2, vacuum leak, clogged injector, weak coil, thermostat, timing chain, fuel pump, exhaust restriction, low compression, boost leak, wastegate) |

Reference results on the generated factory base maps (95 RON, 25 °C):

| Engine | Peak torque | Peak power | Hot idle |
|---|---|---|---|
| 2.0 L NA I4, 10.5:1 | ≈ 200 N·m @ 3500 rpm | ≈ 107 kW @ 6000 rpm | 800 rpm, 25 kPa MAP, λ 1.00 closed loop |
| 2.0 L turbo I4, 9.5:1, 0.8 bar | ≈ 316 N·m @ 4000 rpm | ≈ 170 kW @ 6500 rpm | — |

### Quick start (library)

```rust
use tunerlab_core::{EngineSim, EngineSpec, Language};

let mut sim = EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 42)?;
sim.update_controls(|c| { c.ignition_on = true; c.starter = true; });
for _ in 0..90 { sim.tick(1.0 / 60.0); }          // crank for 1.5 s
sim.update_controls(|c| c.starter = false);
for _ in 0..120 { sim.tick(1.0 / 60.0); }
let t = sim.telemetry();
println!("{}: {:.0} rpm, λ {:.2}", sim.status_text(Language::Pl), t.rpm, t.lambda);
```

---

## 🧑‍💻 Getting Started (For Developers)

### Prerequisites
Latest stable Rust toolchain (1.80+):
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

### Build, test, run
```bash
git clone https://github.com/Hunter-zip/TunerLAB.git
cd TunerLAB
cargo test                                             # unit, behaviour, zero-allocation and doc tests
cargo run --release --example dyno_sweep               # NA dyno power curve
cargo run --release --example dyno_sweep -- turbo pl   # turbo engine, Polish status texts
```

---

## 📄 License
This project is prepared as a commercial-grade MVP software layout. All rights reserved.

# TunerLAB 🚗💻

> **TunerLAB** is a high-fidelity, interactive ECU Tuning Simulator and Educational Platform designed to bridge the gap between virtual engine calibration and real-world calibration engineering.

[![CI](https://github.com/Hunter-zip/TunerLAB/actions/workflows/ci.yml/badge.svg)](https://github.com/Hunter-zip/TunerLAB/actions/workflows/ci.yml)
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
- **Physics Simulation:** Crank-angle-resolved combustion, compressible air path, turbocharging, knock, thermal and damage models running at 4 kHz, ≈ 100–150× faster than real time on one core (≈ 2 µs per 0.25 ms step).
- **Cross-Platform & Lightweight:** Windows 10/11 and Linux, OpenGL/Vulkan rendering via `egui` (Phase 3).
- **Localization (i18n):** **English (EN)** and **Polish (PL)** for every status, warning, trouble code, failure and fault from day one.
- **Business Model (Freemium):** Entry-level concepts and naturally aspirated tuning are free. Advanced modules (forced induction, launch control, full calibration workflows) are unlocked via a premium course tier.

---

## 📈 Roadmap

- [x] **Phase 1 (Current):** Core Simulation Engine (thermodynamic math loop & fail-safe architecture).
- [ ] **Phase 2:** Procedural Audio Engine (low-latency waveform synthesis tracking live RPM/load changes).
- [~] **Phase 3:** Core UI Integration & bilingual multi-language support framework — first working desktop app (see below).
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
| **Dynamics** | Per-cylinder gas-torque pulses, Chen–Flynn friction with oil viscosity, starter, two-mass dynamometer, vehicle with clutch damper springs and gearbox |
| **Thermal** | Metal, coolant (thermostat, cross-flow radiator ε–NTU, boil-over), oil (Vogel viscosity, pressure, fuel dilution with temperature-dependent boil-off, piston cooling, oil/coolant cooler on the turbo), piston crowns, exhaust manifold, catalyst with exotherm; energy-conserving network |
| **Sensors** | MAP, barometric pressure, IAT, ECT, oil, fuel rail, wide-band λ with transport delay, EGT, knock, crank TDC speed, cam phase, rear O2, clutch and neutral switches |
| **ECU** | Speed-density, inverse X–τ transient fuel, warm-up/after-start/cranking fuel, closed-loop STFT/LTFT, idle PI + idle spark + dashpot, per-cylinder knock control, rev limiter, DFCO with decel look-ahead, boost PID with latched overboost cut, OBD-II monitors (misfire, thermostat by airflow model, catalyst by temperature model, O2 response, MAP/IAT circuits) and 26 SAE J2012 trouble codes |
| **Damage** | Knock erosion, piston crown melting, Basquin/Miner rod fatigue, rod buckling, valve-to-piston contact, head gasket, head warp, bearing wear, turbo overspeed, catalyst meltdown |
| **Faults** | 17 hidden Engine Autopsy faults (sensor bias, slow O2, vacuum leak, clogged injector, weak coil, thermostat, timing chain, fuel pump, exhaust restriction, low compression, boost leak, wastegate) |

Reference results on the generated factory base maps (95 RON, 25 °C):

| Engine | Peak torque | Peak power | Hot idle |
|---|---|---|---|
| 2.0 L NA I4, 10.5:1 | ≈ 190 N·m @ 4500 rpm | ≈ 107 kW @ 6500 rpm | 800 rpm, 25 kPa MAP, λ 1.00 closed loop |
| 2.0 L turbo I4, 9.5:1, 0.8 bar | ≈ 297 N·m @ 4000 rpm | ≈ 154 kW @ 5500 rpm | — |

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

## 🖥️ Desktop App — first working version

`tunerlab-app` (crate in `app/`, binary `tunerlab`) wraps the simulation core in a
motorsport-standalone style instrument cluster built with `egui`, fully in Polish and English:

| Tab | What it does |
|---|---|
| **Dashboard / Pulpit** | Tachometer, MAP, λ with target, coolant, oil temperature and pressure, turbo or road speed; torque, power, ignition (vs MBT in the instructor view), knock retard, injector duty, fuel trims, throttle and pedal, boost in bar (turbo), IAT, EGT, per-cylinder table; 30 s charts of speed, MAP, λ and spark. Shows the ECU's sensor readings by default; the *instructor (truth)* view adds physical truth and is locked during an Engine Autopsy case |
| **Course / Kurs** | Six hands-on lessons (first start, VE map and fuel trims, ignition and knock, mixture under load, turbo boost, diagnostics); every step is checked automatically against the live simulation and the current task is shown in a strip at the bottom of every tab |
| **Maps / Mapy** | Live VE, ignition, target-λ and (turbo) boost tables with heat-map colouring, range selection, ± step / set-value editing, restore-to-base, live operating-point marker; every edit is validated by the ECU before it is applied; map files (`.tlmap`, plain text) saved to and loaded from a `mapy` folder next to the program |
| **Dyno / Hamownia** | Wide-open-throttle power sweep on a warmed-up copy of the live engine (calibration, fuel, hidden faults and wear included), on a worker thread; torque/power curves compared with the previous run, each labelled with engine and fuel, full measurement table (λ, spark, MBT, MAP, EGT, knock) |
| **Engine Autopsy** | Plants a random hidden fault (17 kinds, realistic severities); the student diagnoses it from gauges, trims, codes and the event log, then checks the answer |
| **Event log / Dziennik** | Time-stamped status changes, warnings, trouble codes and damage |

Side panels: driver controls (ignition, starter, pedal, free rev / dynamometer / vehicle with
gears, clutch, brake and road grade, ambient temperature and pressure, fuel octane, electrical
load, warm-up / cold-soak / repair / reset) and the live status (condition, warnings, stored
codes with *clear codes*, component health). Simulation speed 1×/2×/5× or pause. Procedural engine
sound from the simulated combustion events (exhaust pulses, misfire gaps, knock ping, overrun pops,
intake, turbo whistle, starter), with an on/off switch and volume in the top bar. The layout
fits windows down to 1200 × 720 (tabs scroll when needed).

Keyboard: **K** ignition · **S** starter (hold) · **W** full throttle (hold) · **↑/↓** pedal ±5 % ·
**0–6** gear · **C** clutch (hold) · **Space** brake (hold) · **+ / −** adjust the selected map cells.

```bash
cargo run --release -p tunerlab-app
```

**Ready-made Windows build:** every CI run on `main` and on pull requests builds `tunerlab.exe`
(static C runtime, no installation). Open
[Actions → CI](https://github.com/Hunter-zip/TunerLAB/actions/workflows/ci.yml), pick the latest
green run and download the `tunerlab-windows-x64` artifact (a zip with the exe and the guides).
A pushed version tag (`v*`) publishes the same build under
[Releases](https://github.com/Hunter-zip/TunerLAB/releases).

Polish user guide with a 10-minute demo scenario: [`docs/INSTRUKCJA.md`](docs/INSTRUKCJA.md).

* **Windows 10/11:** no extra dependencies (OpenGL 2+ driver).
* **Linux:** an OpenGL driver (Mesa) and, on X11, `libxkbcommon-x11`
  (`sudo apt install libxkbcommon-x11-0 libgl1`); Wayland works out of the box.

---

## 🧑‍💻 Getting Started (For Developers)

### Prerequisites
Latest stable Rust toolchain (core 1.80+, desktop app 1.95+):
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

### Build, test, run
```bash
git clone https://github.com/Hunter-zip/TunerLAB.git
cd TunerLAB
cargo test --workspace                                 # core (unit, behaviour, zero-allocation, doc) and app tests
cargo run --release -p tunerlab-app                    # desktop app
cargo run --release --example dyno_sweep               # NA dyno power curve
cargo run --release --example dyno_sweep -- turbo pl   # turbo engine, Polish status texts
```

---

## 📄 License
This project is prepared as a commercial-grade MVP software layout. All rights reserved.

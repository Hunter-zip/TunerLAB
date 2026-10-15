//! Procedural engine sound driven by the simulation's combustion events.
//!
//! Each frame the UI thread hands the new [`CylinderEvent`]s and the engine's operating
//! point to the audio thread: every event becomes an exhaust pulse (none for a misfire),
//! knock rings as a metallic ping and afterburn as a pop, while intake hiss, the turbo
//! whistle and the starter motor follow the telemetry. Without an output device the app
//! simply stays silent.
//!
//! [`CylinderEvent`]: tunerlab_core::engine_sim::events::CylinderEvent

mod output;
mod synth;
mod timeline;

use eframe::egui;
use tunerlab_core::{EngineCondition, EngineSim, Language};

use output::Output;
use synth::{event_sounds, Command, Note, Params};
use timeline::Timeline;

/// Volume the app starts with, 0..1.
const DEFAULT_VOLUME: f32 = 0.7;
/// Width of the volume slider in the top bar \[px\].
const SLIDER_W: f32 = 56.0;

/// Picks the string for the UI language.
fn l(lang: Language, en: &'static str, pl: &'static str) -> &'static str {
    match lang {
        Language::En => en,
        Language::Pl => pl,
    }
}

/// The engine sound: output stream, event cursor and the user's sound settings.
pub struct EngineSound {
    output: Option<Output>,
    timeline: Timeline,
    /// Next combustion event sequence number to consume.
    cursor: u64,
    enabled: bool,
    volume: f32,
}

impl EngineSound {
    /// Opens the default output device; without one the engine sound stays silent.
    pub fn new() -> Self {
        Self::with_output(Output::open())
    }

    fn with_output(output: Option<Output>) -> Self {
        let sample_rate = output.as_ref().map_or(48_000, Output::sample_rate);
        Self {
            output,
            timeline: Timeline::new(f64::from(sample_rate)),
            cursor: 0,
            enabled: true,
            volume: DEFAULT_VOLUME,
        }
    }

    /// An output device is open and running.
    pub fn is_available(&self) -> bool {
        self.output.as_ref().is_some_and(Output::is_alive)
    }

    /// Follows a new or reset simulation: skips its retained events and re-anchors the
    /// timeline (a new engine counts events from zero, a reset restarts the clock).
    pub fn resync(&mut self, sim: &EngineSim) {
        self.cursor = sim.cylinder_events().next_seq();
        self.timeline.reset();
    }

    /// Hands this frame's combustion events and operating point to the audio thread; call
    /// once per frame after the simulation tick.
    pub fn feed(&mut self, sim: &EngineSim, speed: f32, paused: bool) {
        let ring = sim.cylinder_events();
        // A cursor ahead of the ring means the app swapped in a new simulation unnoticed.
        if self.cursor > ring.next_seq() {
            self.cursor = 0;
            self.timeline.reset();
        }
        let from = std::mem::replace(&mut self.cursor, ring.next_seq());
        let Some(output) = self.output.as_ref().filter(|o| o.is_alive()) else {
            return;
        };
        let audible = self.enabled && !paused;
        output.send(Command::Params(params(sim, self.gain(audible))));
        if !audible {
            return;
        }
        // The latency must cover one output period plus a frame.
        self.timeline.set_latency(2.0 * output.period_s() + 0.02);
        self.timeline
            .update(sim.time_s(), output.clock(), f64::from(speed));
        for ev in ring.since(from) {
            for (sim_s, sound) in event_sounds(ev) {
                if let Some(at) = self.timeline.sample_at(sim_s) {
                    output.send(Command::Note(Note { at, sound }));
                }
            }
        }
    }

    /// Perceptual volume curve; zero while muted or paused (the synthesizer fades).
    fn gain(&self, audible: bool) -> f32 {
        if audible {
            self.volume * self.volume
        } else {
            0.0
        }
    }

    /// Compact sound toggle and volume slider for a right-to-left top bar layout.
    pub fn controls(&mut self, ui: &mut egui::Ui, lang: Language) {
        let available = self.is_available();
        ui.add_enabled_ui(available && self.enabled, |ui| {
            ui.spacing_mut().slider_width = SLIDER_W;
            ui.add(egui::Slider::new(&mut self.volume, 0.0..=1.0).show_value(false))
                .on_hover_text(l(lang, "Volume", "Głośność"));
        });
        let toggle = ui
            .add_enabled(
                available,
                egui::Button::selectable(available && self.enabled, l(lang, "Sound", "Dźwięk")),
            )
            .on_hover_text(l(
                lang,
                "Engine sound from the simulated combustion events",
                "Dźwięk silnika z symulowanych zapłonów",
            ))
            .on_disabled_hover_text(l(
                lang,
                "No audio output device: the simulation runs silently",
                "Brak urządzenia wyjściowego audio: symulacja działa bez dźwięku",
            ));
        if toggle.clicked() {
            self.enabled = !self.enabled;
        }
    }
}

/// The operating point behind the continuous sounds.
fn params(sim: &EngineSim, gain: f32) -> Params {
    let t = sim.telemetry();
    let ambient_kpa = sim.controls().ambient.pressure_pa / 1000.0;
    Params {
        rpm: t.rpm,
        throttle: t.throttle_pct / 100.0,
        air_flow_g_s: t.air_flow_g_s,
        turbo_rpm: t.turbo_rpm,
        boost_kpa: t.boost_pressure_kpa - ambient_kpa,
        cylinders: u8::try_from(t.cylinders).unwrap_or(u8::MAX),
        cranking: t.condition == EngineCondition::Cranking,
        gain,
    }
}

#[cfg(test)]
mod tests {
    use super::synth::Synth;
    use super::*;
    use tunerlab_core::engine_sim::controls::{DynoParams, LoadModel};
    use tunerlab_core::engine_sim::MAX_TICK_S;
    use tunerlab_core::EngineSpec;

    const SR: f32 = 48_000.0;
    const FRAME_S: f32 = 1.0 / 60.0;
    const PERIOD: usize = 512;

    /// Runs `seconds` of wall time through the same path as [`EngineSound::feed`], with a
    /// virtual device rendering 512-sample periods in place of the audio thread.
    fn play(sim: &mut EngineSim, seconds: f32, speed: f32) -> Vec<f32> {
        let mut synth = Synth::new(SR);
        let mut timeline = Timeline::new(f64::from(SR));
        let mut cursor = sim.cylinder_events().next_seq();
        let mut out = Vec::new();
        let mut owed = 0.0;
        for _ in 0..(seconds / FRAME_S) as usize {
            let mut remaining = FRAME_S * speed;
            while remaining > 0.0 {
                let step = remaining.min(MAX_TICK_S);
                sim.tick(step);
                remaining -= step;
            }
            synth.set_params(params(sim, 1.0));
            timeline.update(sim.time_s(), synth.clock(), f64::from(speed));
            let ring = sim.cylinder_events();
            for ev in ring.since(cursor) {
                for (sim_s, sound) in event_sounds(ev) {
                    if let Some(at) = timeline.sample_at(sim_s) {
                        synth.schedule(Note { at, sound });
                    }
                }
            }
            cursor = ring.next_seq();
            owed += FRAME_S * SR;
            while owed >= PERIOD as f32 {
                let start = out.len();
                out.resize(start + PERIOD, 0.0);
                synth.render(&mut out[start..]);
                owed -= PERIOD as f32;
            }
        }
        out
    }

    fn rms(x: &[f32]) -> f32 {
        (x.iter().map(|v| v * v).sum::<f32>() / x.len().max(1) as f32).sqrt()
    }

    #[test]
    fn running_engine_sounds_bounded_and_louder_under_load() {
        for spec in [
            EngineSpec::naturally_aspirated_2l(),
            EngineSpec::turbocharged_2l(),
        ] {
            let mut s = match EngineSim::with_base_calibration(spec, 3) {
                Ok(s) => s,
                Err(e) => panic!("built-in preset rejected: {e:?}"),
            };
            s.prewarm();
            s.update_controls(|c| {
                c.ignition_on = true;
                c.starter = true;
            });
            let cranking = play(&mut s, 0.25, 1.0);
            s.update_controls(|c| c.starter = false);
            play(&mut s, 3.0, 1.0);
            let idle = play(&mut s, 1.0, 1.0);
            s.update_controls(|c| {
                c.pedal = 1.0;
                c.load = LoadModel::Dyno(DynoParams {
                    target_rpm: 4500.0,
                    ..DynoParams::default()
                });
            });
            play(&mut s, 3.0, 1.0);
            let wot = play(&mut s, 1.0, 1.0);
            let fast = play(&mut s, 1.0, 5.0);
            for x in [&cranking, &idle, &wot, &fast] {
                assert!(x.iter().all(|v| v.is_finite() && v.abs() <= 1.0));
            }
            assert!(rms(&cranking) > 0.01);
            assert!(rms(&idle) > 0.01);
            assert!(rms(&wot) > 1.5 * rms(&idle));
            assert!(rms(&fast) > 0.01);
        }
    }

    fn sim() -> EngineSim {
        match EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 7) {
            Ok(sim) => sim,
            Err(e) => panic!("built-in preset rejected: {e:?}"),
        }
    }

    fn crank(sim: &mut EngineSim, seconds: f32) {
        sim.update_controls(|c| {
            c.ignition_on = true;
            c.starter = true;
        });
        for _ in 0..(seconds * 60.0) as usize {
            sim.tick(1.0 / 60.0);
        }
    }

    #[test]
    fn cursor_follows_new_and_reset_simulations() {
        let mut sound = EngineSound::with_output(None);
        assert!(!sound.is_available());
        let mut a = sim();
        crank(&mut a, 1.0);
        sound.feed(&a, 1.0, false);
        let consumed = a.cylinder_events().next_seq();
        assert!(consumed > 0);
        assert_eq!(sound.cursor, consumed);

        // A reset keeps the sequence counting.
        a.reset();
        sound.resync(&a);
        assert_eq!(sound.cursor, consumed);

        // A new simulation restarts at zero, also without an explicit resync.
        let b = sim();
        sound.feed(&b, 1.0, false);
        assert_eq!(sound.cursor, 0);
    }

    #[test]
    fn params_follow_the_engine() {
        let mut s = sim();
        crank(&mut s, 0.1);
        let p = params(&s, 0.5);
        assert!(p.cranking);
        assert_eq!(p.cylinders, 4);
        assert!(p.boost_kpa.abs() < 5.0);
        assert_eq!(p.turbo_rpm, 0.0);
        let sound = EngineSound::with_output(None);
        assert_eq!(sound.gain(false), 0.0);
        assert!((sound.gain(true) - DEFAULT_VOLUME * DEFAULT_VOLUME).abs() < 1e-6);
    }
}

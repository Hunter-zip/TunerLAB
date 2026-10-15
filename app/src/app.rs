//! Application state, the simulation loop and the panel layout.

use std::collections::VecDeque;
use std::time::Instant;

use eframe::egui::{self, Color32, Key, RichText, Vec2};
use tunerlab_core::engine_sim::events::{DiagnosticEvent, EventKind};
use tunerlab_core::engine_sim::MAX_TICK_S;
use tunerlab_core::{
    DynoParams, EngineCondition, EngineSim, EngineSpec, FailureCause, Language, LoadModel,
    Localize, VehicleParams, Warning,
};

use crate::audio::EngineSound;
use crate::autopsy::{self, Autopsy, Verdict};
use crate::dyno::DynoBench;
use crate::history::{self, History, Readout, SAMPLES};
use crate::maps::MapEditor;
use crate::text::{tr, T};
use crate::widgets::{self, palette::*, Gauge, Level, Series};

/// Engine presets the student can pick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Preset {
    NaturallyAspirated,
    Turbo,
}

impl Preset {
    fn spec(self) -> EngineSpec {
        match self {
            Preset::NaturallyAspirated => EngineSpec::naturally_aspirated_2l(),
            Preset::Turbo => EngineSpec::turbocharged_2l(),
        }
    }

    fn label(self) -> T {
        match self {
            Preset::NaturallyAspirated => T::PresetNa,
            Preset::Turbo => T::PresetTurbo,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tab {
    Dashboard,
    Maps,
    Dyno,
    Autopsy,
    Log,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LoadKind {
    Neutral,
    Dyno,
    Vehicle,
}

/// Driver inputs as set in the control panel (the keyboard may override some per frame).
struct DriverInputs {
    ignition: bool,
    pedal_pct: f32,
    load: LoadKind,
    dyno_rpm: f32,
    gear: u8,
    clutch_pct: f32,
    brake_pct: f32,
    grade_pct: f32,
    ambient_c: f32,
    baro_kpa: f32,
    octane: f32,
    electrical_w: f32,
}

impl Default for DriverInputs {
    fn default() -> Self {
        Self {
            ignition: false,
            pedal_pct: 0.0,
            load: LoadKind::Neutral,
            dyno_rpm: 3000.0,
            gear: 0,
            clutch_pct: 100.0,
            brake_pct: 0.0,
            grade_pct: 0.0,
            ambient_c: 25.0,
            baro_kpa: 101.3,
            octane: 95.0,
            electrical_w: 300.0,
        }
    }
}

/// Seed of every simulation instance (reproducible behaviour between sessions).
const SEED: u64 = 2026;
/// Diagnostic events kept for the log.
const EVENT_HISTORY: usize = 300;
/// Width the top bar keeps free for the clock, speed, sound and language controls \[px\].
const SIM_CONTROLS_W: f32 = 500.0;
/// Longest wall-clock step fed to the simulation (a stalled window must not jump) \[s\].
const MAX_FRAME_S: f32 = 0.1;

pub struct TunerApp {
    lang: Language,
    preset: Preset,
    sim: EngineSim,
    tab: Tab,
    /// Dashboard shows physical truth instead of the ECU's sensor readings.
    truth_view: bool,
    speed: f32,
    paused: bool,
    last_frame: Option<Instant>,
    inputs: DriverInputs,
    /// Starter button held with the mouse this frame.
    starter_button: bool,
    /// Keys held this frame (the panel shows the inputs they override).
    held: KeyState,
    history: History,
    events: VecDeque<DiagnosticEvent>,
    event_cursor: u64,
    maps: MapEditor,
    dyno: DynoBench,
    autopsy: Autopsy,
    /// Procedural engine sound fed with the combustion events.
    sound: EngineSound,
}

impl TunerApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        configure_style(&cc.egui_ctx);
        let preset = Preset::NaturallyAspirated;
        let sim = EngineSim::with_base_calibration(preset.spec(), SEED)
            .expect("built-in presets are valid");
        let event_cursor = sim.events().next_seq();
        Self {
            lang: Language::Pl,
            preset,
            sim,
            tab: Tab::Dashboard,
            truth_view: false,
            speed: 1.0,
            paused: false,
            last_frame: None,
            inputs: DriverInputs::default(),
            starter_button: false,
            held: KeyState::default(),
            history: History::default(),
            events: VecDeque::new(),
            event_cursor,
            maps: MapEditor::default(),
            dyno: DynoBench::default(),
            autopsy: Autopsy::default(),
            sound: EngineSound::new(),
        }
    }

    fn switch_preset(&mut self, preset: Preset) {
        if preset == self.preset {
            return;
        }
        if let Ok(sim) = EngineSim::with_base_calibration(preset.spec(), SEED) {
            self.preset = preset;
            self.sim = sim;
            // The new engine starts at the ambient set in the panel, not the default 25 °C.
            let (ambient_k, baro_pa) = (
                self.inputs.ambient_c + 273.15,
                self.inputs.baro_kpa * 1000.0,
            );
            self.sim.update_controls(|c| {
                c.ambient.temperature_k = ambient_k;
                c.ambient.pressure_pa = baro_pa;
            });
            self.sim.soak_to_ambient();
            self.clear_log();
            self.history.clear();
            self.autopsy.close();
            self.maps = MapEditor::default();
            self.dyno = DynoBench::default();
            self.inputs.ignition = false;
            self.inputs.pedal_pct = 0.0;
        }
    }

    /// Empties the event log (new engine, reset, new Engine Autopsy case).
    fn clear_log(&mut self) {
        self.events.clear();
        self.event_cursor = self.sim.events().next_seq();
        self.sound.resync(&self.sim);
    }

    /// Writes the driver inputs (panel state plus held keys) into the simulation.
    fn apply_controls(&mut self, ctx: &egui::Context) {
        // Shortcuts are off only while a text field has the keyboard; any other focused
        // widget gives focus back, so Space (brake) never clicks a focused button.
        let typing = ctx.text_edit_focused();
        if !typing {
            ctx.memory_mut(|m| {
                if let Some(id) = m.focused() {
                    m.surrender_focus(id);
                }
            });
        }
        let keys = ctx.input(|i| {
            if typing {
                return KeyState::default();
            }
            // Toggles react to the first press only, not to keyboard auto-repeat.
            let pressed_once = |key: Key| {
                i.events.iter().any(|e| {
                    matches!(e, egui::Event::Key { key: k, pressed: true, repeat: false, .. } if *k == key)
                })
            };
            let mut gear = None;
            for (k, g) in [
                (Key::Num0, 0u8),
                (Key::Num1, 1),
                (Key::Num2, 2),
                (Key::Num3, 3),
                (Key::Num4, 4),
                (Key::Num5, 5),
                (Key::Num6, 6),
            ] {
                if pressed_once(k) {
                    gear = Some(g);
                }
            }
            KeyState {
                ignition_toggle: pressed_once(Key::K),
                starter: i.key_down(Key::S),
                full_throttle: i.key_down(Key::W),
                pedal_up: i.key_pressed(Key::ArrowUp),
                pedal_down: i.key_pressed(Key::ArrowDown),
                clutch: i.key_down(Key::C),
                brake: i.key_down(Key::Space),
                gear,
            }
        });
        self.held = keys;
        let inp = &mut self.inputs;
        if keys.ignition_toggle {
            inp.ignition = !inp.ignition;
        }
        if keys.pedal_up {
            inp.pedal_pct = (inp.pedal_pct + 5.0).min(100.0);
        }
        if keys.pedal_down {
            inp.pedal_pct = (inp.pedal_pct - 5.0).max(0.0);
        }
        if let Some(g) = keys.gear {
            inp.gear = g;
            if inp.load != LoadKind::Vehicle {
                inp.load = LoadKind::Vehicle;
            }
        }
        let pedal = if keys.full_throttle {
            100.0
        } else {
            inp.pedal_pct
        };
        let load = match inp.load {
            LoadKind::Neutral => LoadModel::Neutral,
            LoadKind::Dyno => LoadModel::Dyno(DynoParams {
                target_rpm: inp.dyno_rpm,
                ..DynoParams::default()
            }),
            LoadKind::Vehicle => LoadModel::Vehicle(VehicleParams {
                gear: inp.gear,
                clutch: if keys.clutch {
                    0.0
                } else {
                    inp.clutch_pct / 100.0
                },
                brake: if keys.brake {
                    1.0
                } else {
                    inp.brake_pct / 100.0
                },
                grade_rad: (inp.grade_pct / 100.0).atan(),
                ..VehicleParams::default()
            }),
        };
        let starter = (self.starter_button || keys.starter) && inp.ignition;
        let (ignition, ambient_k, baro_pa, octane, electrical) = (
            inp.ignition,
            inp.ambient_c + 273.15,
            inp.baro_kpa * 1000.0,
            inp.octane,
            inp.electrical_w,
        );
        self.sim.update_controls(|c| {
            c.ignition_on = ignition;
            c.starter = starter;
            c.pedal = pedal / 100.0;
            c.load = load;
            c.ambient.temperature_k = ambient_k;
            c.ambient.pressure_pa = baro_pa;
            c.fuel_octane_ron = octane;
            c.electrical_load_w = electrical;
        });
    }

    /// Whether the dashboard shows physical truth: never while an Engine Autopsy case is
    /// open, where it would give the answer away.
    fn showing_truth(&self) -> bool {
        self.truth_view && self.autopsy.fault.is_none()
    }

    fn readout(&self) -> Readout {
        Readout::new(
            self.sim.telemetry(),
            self.showing_truth(),
            self.inputs.baro_kpa,
        )
    }

    fn poll_events(&mut self) {
        let log = self.sim.events();
        for ev in log.since(self.event_cursor) {
            if self.events.len() == EVENT_HISTORY {
                self.events.pop_front();
            }
            self.events.push_back(*ev);
        }
        self.event_cursor = log.next_seq();
    }
}

#[derive(Default, Clone, Copy)]
struct KeyState {
    ignition_toggle: bool,
    starter: bool,
    full_throttle: bool,
    pedal_up: bool,
    pedal_down: bool,
    clutch: bool,
    brake: bool,
    gear: Option<u8>,
}

fn configure_style(ctx: &egui::Context) {
    let mut visuals = egui::Visuals::dark();
    visuals.panel_fill = PANEL;
    visuals.window_fill = PANEL;
    visuals.extreme_bg_color = BACKGROUND;
    visuals.selection.bg_fill = ACCENT.gamma_multiply(0.6);
    visuals.hyperlink_color = ACCENT;
    ctx.set_visuals(visuals);
}

impl eframe::App for TunerApp {
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let now = Instant::now();
        let dt = self
            .last_frame
            .map_or(0.0, |t| now.duration_since(t).as_secs_f32())
            .min(MAX_FRAME_S);
        self.last_frame = Some(now);
        self.apply_controls(ctx);
        if !self.paused {
            // The core truncates single ticks above MAX_TICK_S; split fast-forward frames.
            let mut remaining = dt * self.speed;
            while remaining > 0.0 {
                let step = remaining.min(MAX_TICK_S);
                self.sim.tick(step);
                remaining -= step;
            }
        }
        let readout = self.readout();
        self.history.record(self.sim.telemetry(), &readout);
        self.poll_events();
        self.sound.feed(&self.sim, self.speed, self.paused);
        self.dyno.poll();
        ctx.request_repaint();
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.top_bar(ui);
        egui::Panel::left("controls")
            .resizable(false)
            .exact_size(280.0)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| self.controls_panel(ui));
            });
        egui::Panel::right("status")
            .resizable(false)
            .exact_size(320.0)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| self.status_panel(ui));
            });
        // Tabs scroll when the window is smaller than their content (minimum 1200×720).
        egui::CentralPanel::default_margins().show(ui, |ui| match self.tab {
            Tab::Dashboard => {
                egui::ScrollArea::vertical()
                    .id_salt("dashboard")
                    .show(ui, |ui| self.dashboard(ui));
            }
            Tab::Maps => {
                let lang = self.lang;
                egui::ScrollArea::both()
                    .id_salt("maps")
                    .show(ui, |ui| self.maps.show(ui, &mut self.sim, lang));
            }
            Tab::Dyno => {
                egui::ScrollArea::vertical()
                    .id_salt("dyno")
                    .show(ui, |ui| self.dyno_tab(ui));
            }
            Tab::Autopsy => {
                egui::ScrollArea::vertical()
                    .id_salt("autopsy")
                    .show(ui, |ui| self.autopsy_tab(ui));
            }
            Tab::Log => self.log_tab(ui),
        });
    }
}

impl TunerApp {
    fn top_bar(&mut self, ui: &mut egui::Ui) {
        let lang = self.lang;
        egui::Panel::top("top").show(ui, |ui| {
            let mut narrow = false;
            ui.horizontal(|ui| {
                ui.label(RichText::new("TunerLAB").strong().size(20.0).color(ACCENT));
                ui.separator();
                ui.label(tr(T::Engine, lang));
                let mut preset = self.preset;
                egui::ComboBox::from_id_salt("preset")
                    .selected_text(tr(preset.label(), lang))
                    .show_ui(ui, |ui| {
                        for p in [Preset::NaturallyAspirated, Preset::Turbo] {
                            ui.selectable_value(&mut preset, p, tr(p.label(), lang));
                        }
                    });
                self.switch_preset(preset);
                ui.separator();
                for (tab, label) in [
                    (Tab::Dashboard, T::TabDashboard),
                    (Tab::Maps, T::TabMaps),
                    (Tab::Dyno, T::TabDyno),
                    (Tab::Autopsy, T::TabAutopsy),
                    (Tab::Log, T::TabLog),
                ] {
                    if ui
                        .selectable_label(
                            self.tab == tab,
                            RichText::new(tr(label, lang)).size(15.0),
                        )
                        .clicked()
                    {
                        self.tab = tab;
                    }
                }
                ui.separator();
                ui.label(RichText::new(tr(T::View, lang)).color(DIM));
                if ui
                    .selectable_label(!self.showing_truth(), tr(T::ViewEcu, lang))
                    .clicked()
                {
                    self.truth_view = false;
                }
                let locked = self.autopsy.fault.is_some();
                let truth = ui
                    .add_enabled(
                        !locked,
                        egui::Button::selectable(self.showing_truth(), tr(T::ViewTruth, lang)),
                    )
                    .on_disabled_hover_text(tr(T::ViewLocked, lang));
                if truth.clicked() {
                    self.truth_view = true;
                }
                if ui.available_width() >= SIM_CONTROLS_W {
                    self.sim_controls(ui);
                } else {
                    narrow = true;
                }
            });
            // Narrow window: the simulation controls get a row of their own.
            if narrow {
                ui.horizontal(|ui| self.sim_controls(ui));
            }
        });
    }

    /// Clock, simulation speed, pause and language, right-aligned.
    fn sim_controls(&mut self, ui: &mut egui::Ui) {
        let lang = self.lang;
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            for l in [Language::En, Language::Pl] {
                if ui.selectable_label(self.lang == l, l.code()).clicked() {
                    self.lang = l;
                }
            }
            ui.separator();
            self.sound.controls(ui, lang);
            ui.separator();
            for s in [5.0_f32, 2.0, 1.0] {
                if ui
                    .selectable_label(!self.paused && self.speed == s, format!("{s:.0}×"))
                    .clicked()
                {
                    self.speed = s;
                    self.paused = false;
                }
            }
            if ui
                .selectable_label(self.paused, tr(T::Pause, lang))
                .clicked()
            {
                self.paused = !self.paused;
            }
            ui.label(RichText::new(tr(T::SimSpeed, lang)).color(DIM));
            ui.separator();
            ui.label(
                RichText::new(format!("{:.1} s", self.sim.time_s()))
                    .monospace()
                    .color(DIM),
            );
        });
    }

    fn controls_panel(&mut self, ui: &mut egui::Ui) {
        let lang = self.lang;
        let inp = &mut self.inputs;
        ui.heading(tr(T::Controls, lang));
        ui.add_space(4.0);
        ui.checkbox(
            &mut inp.ignition,
            RichText::new(tr(T::Ignition, lang)).strong(),
        );
        let starter = ui.add_enabled(
            inp.ignition,
            // Drag sensing keeps the button pressed for as long as it is held (a plain
            // click button gives up after egui's 0.8 s click timeout).
            egui::Button::new(RichText::new(tr(T::Starter, lang)).strong())
                .min_size(Vec2::new(250.0, 34.0))
                .sense(egui::Sense::click_and_drag()),
        );
        self.starter_button = starter.is_pointer_button_down_on();
        let held = self.held;
        let rpm = self.sim.telemetry().rpm;
        let inp = &mut self.inputs;
        ui.add_space(6.0);
        ui.label(tr(T::Pedal, lang));
        held_slider(ui, &mut inp.pedal_pct, held.full_throttle.then_some(100.0));

        ui.add_space(8.0);
        ui.label(RichText::new(tr(T::LoadModel, lang)).strong());
        ui.horizontal(|ui| {
            ui.radio_value(&mut inp.load, LoadKind::Neutral, tr(T::LoadNeutral, lang));
            ui.radio_value(&mut inp.load, LoadKind::Dyno, tr(T::LoadDyno, lang));
        });
        ui.radio_value(&mut inp.load, LoadKind::Vehicle, tr(T::LoadVehicle, lang));
        match inp.load {
            LoadKind::Neutral => {}
            LoadKind::Dyno => {
                ui.label(tr(T::DynoSpeed, lang));
                ui.add(
                    egui::Slider::new(&mut inp.dyno_rpm, 800.0..=8000.0)
                        .step_by(50.0)
                        .suffix(format!(" {}", tr(T::AxisRpm, lang))),
                );
            }
            LoadKind::Vehicle => {
                ui.label(tr(T::Gear, lang));
                ui.horizontal(|ui| {
                    for g in 0..=6u8 {
                        let label = if g == 0 {
                            tr(T::Neutral, lang).to_owned()
                        } else {
                            g.to_string()
                        };
                        ui.selectable_value(
                            &mut inp.gear,
                            g,
                            RichText::new(label).monospace().size(16.0),
                        );
                    }
                });
                ui.label(tr(T::Clutch, lang));
                held_slider(ui, &mut inp.clutch_pct, held.clutch.then_some(0.0));
                ui.label(tr(T::Brake, lang));
                held_slider(ui, &mut inp.brake_pct, held.brake.then_some(100.0));
                ui.label(tr(T::Grade, lang));
                ui.add(egui::Slider::new(&mut inp.grade_pct, -15.0..=15.0).suffix(" %"));
            }
        }

        ui.add_space(8.0);
        ui.label(RichText::new(tr(T::Ambient, lang)).strong());
        ui.label(tr(T::AmbientTemp, lang));
        ui.add(egui::Slider::new(&mut inp.ambient_c, -30.0..=45.0).suffix(" °C"));
        ui.label(tr(T::Barometer, lang));
        ui.add(egui::Slider::new(&mut inp.baro_kpa, 60.0..=105.0).suffix(" kPa"));
        ui.label(tr(T::Octane, lang));
        ui.horizontal(|ui| {
            for ron in [91.0_f32, 95.0, 98.0, 102.0] {
                ui.selectable_value(&mut inp.octane, ron, format!("{ron:.0}"));
            }
        });
        ui.label(tr(T::ElectricalLoad, lang));
        ui.add(
            egui::Slider::new(&mut inp.electrical_w, 0.0..=2500.0)
                .step_by(50.0)
                .suffix(" W"),
        );

        ui.add_space(8.0);
        ui.label(RichText::new(tr(T::Workshop, lang)).strong());
        ui.horizontal_wrapped(|ui| {
            if ui.button(tr(T::Prewarm, lang)).clicked() {
                self.sim.prewarm();
            }
            // The core only soaks a stopped engine.
            if ui
                .add_enabled(rpm <= 0.0, egui::Button::new(tr(T::Soak, lang)))
                .on_disabled_hover_text(tr(T::SoakNeedsStop, lang))
                .clicked()
            {
                self.sim.soak_to_ambient();
            }
            if ui.button(tr(T::Repair, lang)).clicked() {
                self.sim.repair();
            }
            if ui.button(tr(T::Reset, lang)).clicked() {
                self.sim.reset();
                self.inputs.ignition = false;
                self.inputs.pedal_pct = 0.0;
                self.history.clear();
                self.autopsy.close();
                self.clear_log();
            }
        });
        ui.add_space(8.0);
        ui.label(RichText::new(tr(T::KeysHelp, lang)).size(11.0).color(DIM));
    }

    fn status_panel(&mut self, ui: &mut egui::Ui) {
        let lang = self.lang;
        let t = *self.sim.telemetry();
        ui.heading(tr(T::Status, lang));
        let color = match t.condition {
            EngineCondition::Running => OK,
            EngineCondition::Cranking => WARN,
            EngineCondition::Failed(_) => DANGER,
            _ => DIM,
        };
        ui.label(
            RichText::new(self.sim.status_text(lang))
                .color(color)
                .size(16.0)
                .strong(),
        );
        ui.label(
            RichText::new(if t.closed_loop {
                tr(T::ClosedLoop, lang)
            } else {
                tr(T::OpenLoop, lang)
            })
            .color(DIM),
        );

        ui.add_space(8.0);
        ui.label(RichText::new(tr(T::Warnings, lang)).strong());
        if t.warnings.is_empty() {
            ui.label(RichText::new(tr(T::NoWarnings, lang)).color(DIM));
        }
        for w in t.warnings.iter() {
            let color = match w {
                Warning::CheckEngine | Warning::InjectorDutyHigh | Warning::RichMixture => WARN,
                _ => DANGER,
            };
            widgets::lamp(ui, w.localized(lang), color);
        }

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.label(RichText::new(tr(T::Dtcs, lang)).strong());
            if ui.small_button(tr(T::ClearDtcs, lang)).clicked() {
                self.sim.clear_dtcs();
            }
        });
        if self.sim.dtcs().is_empty() {
            ui.label(RichText::new(tr(T::NoDtcs, lang)).color(DIM));
        }
        for code in self.sim.dtcs().iter() {
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new(code.code()).monospace().color(WARN).strong());
                ui.label(code.localized(lang));
            });
        }

        ui.add_space(8.0);
        ui.label(RichText::new(tr(T::Health, lang)).strong());
        let h = t.health;
        let n = t.cylinders.max(1);
        let w = ui.available_width().min(300.0);
        let min = |a: &[f32]| a[..n].iter().copied().fold(1.0_f32, f32::min);
        let max = |a: &[f32]| a[..n].iter().copied().fold(0.0_f32, f32::max);
        // A failed part reads 0 % whatever its wear model says (a rod thrown by over-rev
        // still has fatigue life left on paper).
        let failed = match t.condition {
            EngineCondition::Failed(cause) => Some(cause),
            _ => None,
        };
        let part = |value: f32, broke: fn(FailureCause) -> bool| {
            if failed.is_some_and(broke) {
                0.0
            } else {
                value
            }
        };
        let bars = [
            (
                T::Pistons,
                part(min(&h.piston), |c| {
                    matches!(
                        c,
                        FailureCause::HoledPiston { .. } | FailureCause::MeltedPiston { .. }
                    )
                }),
            ),
            (T::Rings, min(&h.rings)),
            (
                T::Valves,
                part(min(&h.valves), |c| {
                    matches!(c, FailureCause::BentValve { .. })
                }),
            ),
            (
                T::Rods,
                part(1.0 - max(&h.rod_life_used), |c| {
                    matches!(
                        c,
                        FailureCause::ThrownRod { .. } | FailureCause::BentRod { .. }
                    )
                }),
            ),
            (
                T::HeadGasket,
                part(h.head_gasket, |c| {
                    matches!(c, FailureCause::HeadGasket { .. })
                }),
            ),
            (
                T::CylinderHead,
                part(h.cylinder_head, |c| matches!(c, FailureCause::WarpedHead)),
            ),
            (
                T::Bearings,
                part(h.bearings, |c| matches!(c, FailureCause::SpunBearing)),
            ),
        ];
        for (label, value) in bars {
            widgets::health_bar(ui, w, tr(label, lang), value);
        }
        if self.sim.spec().turbo.is_some() {
            let turbo = part(h.turbo, |c| matches!(c, FailureCause::TurboFailure));
            widgets::health_bar(ui, w, tr(T::Turbo, lang), turbo);
        }
        let catalyst = part(h.catalyst, |c| matches!(c, FailureCause::CatalystMeltdown));
        widgets::health_bar(ui, w, tr(T::Catalyst, lang), catalyst);
    }

    fn dashboard(&mut self, ui: &mut egui::Ui) {
        let lang = self.lang;
        let t = *self.sim.telemetry();
        let spec = *self.sim.spec();
        let turbo = spec.turbo.is_some();
        let redline = spec.limits.redline_rpm;
        let running = t.rpm > 1.0;
        let truth = self.showing_truth();
        let r = self.readout();
        let avail = ui.available_width();
        let big = (avail * 0.30).clamp(200.0, 300.0);
        let small = (avail * 0.15).clamp(120.0, 170.0);

        ui.horizontal(|ui| {
            let rpm_max = ((redline / 1000.0).ceil() + 1.0) * 1000.0;
            widgets::round_gauge(
                ui,
                big,
                &Gauge::new(tr(T::Rpm, lang), "×1000", 0.0, rpm_max, r.rpm)
                    .step(1000.0)
                    .divisor(1000.0)
                    .danger(redline, rpm_max),
            );
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    let map_max = if turbo { 250.0 } else { 120.0 };
                    let mut g = Gauge::new(tr(T::Manifold, lang), "kPa", 0.0, map_max, r.map_kpa)
                        .step(if turbo { 50.0 } else { 20.0 });
                    if turbo && t.boost_target_kpa > 0.0 {
                        g = g.marker(t.boost_target_kpa, CYAN);
                    }
                    widgets::round_gauge(ui, small, &g);
                    // No reading during fuel cut (the probe sees air); off-scale values pin
                    // the needle in the end zone.
                    let lambda = if t.injector_pulse_ms > 0.0 {
                        r.lambda
                    } else {
                        f32::NAN
                    };
                    // Lean is what destroys engines (red); rich only costs power (amber).
                    let mut g = Gauge::new(tr(T::Lambda, lang), "λ", 0.6, 1.4, lambda)
                        .decimals(2)
                        .step(0.1)
                        .caution(0.6, 0.75)
                        .danger(1.1, 1.4);
                    if running {
                        g = g.marker(t.lambda_target, CYAN);
                    }
                    widgets::round_gauge(ui, small, &g);
                    widgets::round_gauge(
                        ui,
                        small,
                        &Gauge::new(tr(T::Coolant, lang), "°C", -40.0, 160.0, r.coolant_c)
                            .step(40.0)
                            .caution(105.0, 115.0)
                            .danger(115.0, 160.0),
                    );
                });
                ui.horizontal(|ui| {
                    widgets::round_gauge(
                        ui,
                        small,
                        &Gauge::new(tr(T::OilTemp, lang), "°C", -40.0, 160.0, r.oil_temp_c)
                            .step(40.0)
                            .caution(125.0, 135.0)
                            .danger(135.0, 160.0),
                    );
                    widgets::round_gauge(
                        ui,
                        small,
                        &Gauge::new(tr(T::OilPressure, lang), "bar", 0.0, 7.0, r.oil_kpa / 100.0)
                            .decimals(1)
                            .step(1.0),
                    );
                    if turbo {
                        widgets::round_gauge(
                            ui,
                            small,
                            &Gauge::new(
                                tr(T::TurboSpeed, lang),
                                "krpm",
                                0.0,
                                240.0,
                                r.turbo_rpm / 1000.0,
                            )
                            .step(40.0)
                            .danger(190.0, 240.0),
                        );
                    } else {
                        widgets::round_gauge(
                            ui,
                            small,
                            &Gauge::new(tr(T::VehicleSpeed, lang), "km/h", 0.0, 240.0, r.speed_kph)
                                .step(40.0),
                        );
                    }
                });
            });
        });

        // Digital tiles; engine-output readouts are blank while the crank is at rest.
        let per_row = ((avail + 8.0) / (130.0 + 8.0)).floor().clamp(3.0, 6.0);
        let tile = Vec2::new(((avail + 8.0) / per_row - 8.0).floor(), 52.0);
        let live = |v: String| if running { v } else { "—".to_owned() };
        let knock_max = t.knock_retard_deg[..t.cylinders.max(1)]
            .iter()
            .copied()
            .fold(0.0_f32, f32::max);
        let lvl = |danger: bool, caution: bool| {
            if !running {
                Level::Off
            } else if danger {
                Level::Danger
            } else if caution {
                Level::Caution
            } else {
                Level::Normal
            }
        };
        ui.add_space(6.0);
        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing = Vec2::splat(8.0);
            widgets::tile(
                ui,
                tile,
                tr(T::Torque, lang),
                &live(format!("{:.0}", t.brake_torque_nm)),
                "N·m",
                lvl(false, false),
            );
            widgets::tile(
                ui,
                tile,
                tr(T::Power, lang),
                &live(format!("{:.1}", t.power_kw)),
                "kW",
                lvl(false, false),
            );
            widgets::tile(
                ui,
                tile,
                tr(T::Ignition2, lang),
                &live(format!("{:.1}", t.ignition_advance_deg)),
                &match (truth, history::mbt(&t)) {
                    (true, Some(mbt)) => format!("° {} {mbt:.0}°", tr(T::Mbt, lang)),
                    (true, None) => format!("° {} —", tr(T::Mbt, lang)),
                    (false, _) => "°".to_owned(),
                },
                lvl(false, false),
            );
            widgets::tile(
                ui,
                tile,
                tr(T::KnockRetard, lang),
                &live(format!("{knock_max:.1}")),
                "°",
                lvl(knock_max > 6.0, knock_max > 0.5),
            );
            widgets::tile(
                ui,
                tile,
                tr(T::InjectorDuty, lang),
                &live(format!("{:.0}", t.injector_duty_pct)),
                &format!("% · {:.1} ms", t.injector_pulse_ms),
                lvl(t.injector_duty_pct > 90.0, t.injector_duty_pct > 80.0),
            );
            widgets::tile(
                ui,
                tile,
                tr(T::FuelTrims, lang),
                &live(format!("{:+.0}/{:+.0}", t.stft_pct, t.ltft_pct)),
                "%",
                lvl(
                    t.stft_pct.abs().max(t.ltft_pct.abs()) > 20.0,
                    t.stft_pct.abs().max(t.ltft_pct.abs()) > 10.0,
                ),
            );
            widgets::tile(
                ui,
                tile,
                tr(T::IntakeAir, lang),
                &format!("{:.0}", r.iat_c),
                "°C",
                lvl(r.iat_c > 70.0, r.iat_c > 55.0),
            );
            widgets::tile(
                ui,
                tile,
                tr(T::Egt, lang),
                &format!("{:.0}", r.egt_c),
                "°C",
                lvl(r.egt_c > 950.0, r.egt_c > 880.0),
            );
            widgets::tile(
                ui,
                tile,
                tr(T::Throttle, lang),
                &format!("{:.0}", t.throttle_pct),
                &format!("% · {} {:.0} %", tr(T::PedalShort, lang), t.pedal_pct),
                Level::Normal,
            );
            if turbo {
                // Boost as a boost gauge reads it: manifold pressure above atmosphere.
                let boost_bar = (r.map_kpa - r.baro_kpa) / 100.0;
                widgets::tile(
                    ui,
                    tile,
                    tr(T::Boost, lang),
                    &format!("{boost_bar:+.2}"),
                    "bar",
                    lvl(false, false),
                );
            }
            widgets::tile(
                ui,
                tile,
                tr(T::Baro, lang),
                &format!("{:.1}", r.baro_kpa),
                "kPa",
                Level::Normal,
            );
            widgets::tile(
                ui,
                tile,
                tr(T::FuelPressure, lang),
                &format!("{:.0}", r.fuel_kpa),
                "kPa",
                Level::Normal,
            );
            widgets::tile(
                ui,
                tile,
                tr(T::Battery, lang),
                &format!("{:.1}", r.battery_v),
                "V",
                if r.battery_v < 11.8 {
                    Level::Caution
                } else {
                    Level::Normal
                },
            );
            widgets::tile(
                ui,
                tile,
                tr(T::VehicleSpeed, lang),
                &format!("{:.0}", r.speed_kph),
                "km/h",
                Level::Normal,
            );
        });

        // Per-cylinder data: the ECU's knock retard, knock-sensor window and misfire
        // counters; peak pressure and piston temperature only in the instructor view.
        ui.add_space(6.0);
        let n = t.cylinders.max(1);
        egui::Grid::new("cylinders")
            .striped(true)
            .min_col_width(64.0)
            .show(ui, |ui| {
                ui.label(RichText::new(tr(T::Cylinders, lang)).strong());
                for c in 0..n {
                    ui.label(
                        RichText::new(format!("{} {}", tr(T::Cylinder, lang), c + 1)).color(DIM),
                    );
                }
                ui.end_row();
                let row =
                    |ui: &mut egui::Ui,
                     label: &str,
                     values: &mut dyn Iterator<Item = (String, Color32)>| {
                        ui.label(RichText::new(label).color(DIM));
                        for (v, color) in values {
                            ui.label(RichText::new(v).monospace().color(color));
                        }
                        ui.end_row();
                    };
                row(
                    ui,
                    tr(T::KnockRetard, lang),
                    &mut t.knock_retard_deg[..n]
                        .iter()
                        .map(|v| (format!("{v:.1}°"), if *v > 0.5 { WARN } else { TEXT })),
                );
                row(
                    ui,
                    tr(T::Misfires, lang),
                    &mut t.misfire_count[..n]
                        .iter()
                        .map(|v| (v.to_string(), if *v > 0 { WARN } else { TEXT })),
                );
                if truth {
                    row(
                        ui,
                        tr(T::KnockLevel, lang),
                        &mut t.knock_intensity[..n]
                            .iter()
                            .map(|v| (format!("{v:.2}"), if *v > 0.5 { DANGER } else { TEXT })),
                    );
                    row(
                        ui,
                        tr(T::PeakPressure, lang),
                        &mut t.peak_pressure_bar[..n]
                            .iter()
                            .map(|v| (format!("{v:.0}"), TEXT)),
                    );
                    row(
                        ui,
                        tr(T::PistonTemp, lang),
                        &mut t.piston_temp_c[..n]
                            .iter()
                            .map(|v| (format!("{v:.0}"), if *v > 300.0 { WARN } else { TEXT })),
                    );
                }
            });

        // Charts.
        ui.add_space(6.0);
        let h = self.history_charts_height(ui);
        let chart = Vec2::new(((avail - 8.0) / 2.0).floor().max(200.0), h);
        let hist = &mut self.history;
        let rpm = hist.rpm.make_contiguous().to_vec();
        let map = hist.map_kpa.make_contiguous().to_vec();
        let lam = hist.lambda.make_contiguous().to_vec();
        let lam_t = hist.lambda_target.make_contiguous().to_vec();
        let spark = hist.spark.make_contiguous().to_vec();
        let mbt = hist.mbt.make_contiguous().to_vec();
        let rpm_top = ((redline / 1000.0).ceil() + 1.0) * 1000.0;
        let map_top = if turbo { 250.0 } else { 110.0 };
        ui.horizontal(|ui| {
            widgets::chart(
                ui,
                chart,
                tr(T::ChartSpeed, lang),
                SAMPLES,
                (0.0, rpm_top),
                &[Series {
                    values: &rpm,
                    color: ACCENT,
                    label: "",
                }],
            );
            widgets::chart(
                ui,
                chart,
                tr(T::ChartManifold, lang),
                SAMPLES,
                (0.0, map_top),
                &[Series {
                    values: &map,
                    color: BLUE,
                    label: "",
                }],
            );
        });
        ui.horizontal(|ui| {
            widgets::chart(
                ui,
                chart,
                tr(T::ChartMixture, lang),
                SAMPLES,
                (0.6, 1.4),
                &[
                    Series {
                        values: &lam_t,
                        color: CYAN,
                        label: tr(T::LegendTarget, lang),
                    },
                    Series {
                        values: &lam,
                        color: OK,
                        label: tr(T::LegendMeasured, lang),
                    },
                ],
            );
            widgets::chart(
                ui,
                chart,
                tr(T::ChartSpark, lang),
                SAMPLES,
                (-10.0, 50.0),
                &[
                    Series {
                        values: if truth { &mbt } else { &[] },
                        color: VIOLET,
                        label: if truth { tr(T::Mbt, lang) } else { "" },
                    },
                    Series {
                        values: &spark,
                        color: WARN,
                        label: tr(T::LegendSpark, lang),
                    },
                ],
            );
        });
    }

    fn history_charts_height(&self, ui: &egui::Ui) -> f32 {
        ((ui.available_height() - 16.0) / 2.0).clamp(110.0, 220.0)
    }

    fn dyno_tab(&mut self, ui: &mut egui::Ui) {
        let lang = self.lang;
        ui.label(RichText::new(tr(T::DynoHelp, lang)).color(DIM));
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            let running = self.dyno.running();
            if ui
                .add_enabled(
                    !running,
                    egui::Button::new(RichText::new(tr(T::RunSweep, lang)).strong()),
                )
                .clicked()
            {
                let label = format!(
                    "{} · {:.0} RON",
                    tr(self.preset.label(), lang),
                    self.inputs.octane
                );
                self.dyno.start(self.sim.clone(), label);
            }
            if running {
                ui.add(
                    egui::ProgressBar::new(self.dyno.progress)
                        .text(tr(T::Measuring, lang))
                        .desired_width(240.0),
                );
            }
        });
        ui.add_space(6.0);
        let Some(current) = self.dyno.current.clone() else {
            ui.label(RichText::new(tr(T::NoRunYet, lang)).color(DIM));
            return;
        };
        let previous = self.dyno.previous.clone();
        if let Some(cause) = current.failure {
            ui.label(
                RichText::new(format!(
                    "{}: {}",
                    tr(T::DynoFailed, lang),
                    cause.localized(lang)
                ))
                .color(DANGER)
                .strong(),
            );
        }
        // One line per run, labelled with the engine and fuel it was measured on.
        for (run, label, color) in [
            (Some(&current), T::CurrentRun, ACCENT),
            (previous.as_ref(), T::PreviousRun, DIM),
        ] {
            let Some(run) = run else { continue };
            if let (Some(tq), Some(pw)) = (run.peak_torque(), run.peak_power()) {
                ui.label(
                    RichText::new(format!(
                        "{} ({}): {} {:.0} N·m @ {:.0} · {} {:.1} kW @ {:.0} {}",
                        tr(label, lang),
                        run.label,
                        tr(T::PeakTorque, lang),
                        tq.torque_nm,
                        tq.rpm,
                        tr(T::PeakPower, lang),
                        pw.power_kw,
                        pw.rpm,
                        tr(T::AxisRpm, lang),
                    ))
                    .color(color),
                );
            }
        }
        dyno_chart(
            ui,
            &current,
            previous.as_ref(),
            self.sim.spec().limits.redline_rpm,
        );
        ui.add_space(6.0);
        egui::Grid::new("dyno_table")
            .striped(true)
            .num_columns(9)
            .show(ui, |ui| {
                for h in [
                    tr(T::AxisRpm, lang),
                    "N·m",
                    "kW",
                    "λ",
                    tr(T::ColSpark, lang),
                    "MBT",
                    "MAP kPa",
                    "EGT °C",
                    tr(T::ColKnock, lang),
                ] {
                    ui.label(RichText::new(h).color(DIM).monospace());
                }
                ui.end_row();
                for p in &current.points {
                    ui.monospace(format!("{:.0}", p.rpm));
                    ui.monospace(format!("{:.1}", p.torque_nm));
                    ui.monospace(format!("{:.1}", p.power_kw));
                    ui.monospace(format!("{:.3}", p.lambda));
                    ui.monospace(format!("{:.1}", p.spark_deg));
                    ui.monospace(format!("{:.1}", p.mbt_deg));
                    ui.monospace(format!("{:.1}", p.map_kpa));
                    ui.monospace(format!("{:.0}", p.egt_c));
                    let knock = RichText::new(format!("{:.2}", p.knock_bar)).monospace();
                    ui.label(if p.knock_bar > 0.5 {
                        knock.color(DANGER)
                    } else {
                        knock
                    });
                    ui.end_row();
                }
            });
    }

    fn autopsy_tab(&mut self, ui: &mut egui::Ui) {
        let lang = self.lang;
        ui.label(RichText::new(tr(T::AutopsyHelp, lang)).color(DIM));
        ui.add_space(6.0);
        if ui
            .button(RichText::new(tr(T::NewCase, lang)).strong())
            .clicked()
        {
            self.autopsy.new_case(&mut self.sim);
            self.clear_log();
        }
        ui.add_space(6.0);
        let Some(fault) = self.autopsy.fault else {
            ui.label(tr(T::NoCase, lang));
            return;
        };
        ui.label(RichText::new(tr(T::CaseActive, lang)).color(WARN).strong());
        ui.add_space(6.0);
        ui.label(RichText::new(tr(T::YourDiagnosis, lang)).strong());
        let turbo = self.sim.spec().turbo.is_some();
        // After the reveal the case is over: no more answers to score.
        let open = !self.autopsy.revealed;
        // Two columns of answers only where both fit next to each other.
        let columns = if ui.available_width() >= 840.0 { 2 } else { 1 };
        egui::Grid::new("autopsy_answers")
            .num_columns(columns)
            .show(ui, |ui| {
                for (i, id) in autopsy::candidates(turbo).into_iter().enumerate() {
                    let chosen = self.autopsy.answer == Some(id);
                    if ui
                        .add_enabled(open, egui::RadioButton::new(chosen, id.localized(lang)))
                        .clicked()
                    {
                        self.autopsy.answer = Some(id);
                    }
                    if i % columns == columns - 1 {
                        ui.end_row();
                    }
                }
            });
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    open && self.autopsy.answer.is_some(),
                    egui::Button::new(tr(T::Check, lang)),
                )
                .clicked()
            {
                self.autopsy.check();
            }
            if ui.button(tr(T::Reveal, lang)).clicked() {
                self.autopsy.revealed = true;
            }
            ui.label(format!(
                "{}: {}",
                tr(T::Attempts, lang),
                self.autopsy.attempts
            ));
        });
        match self.autopsy.verdict {
            Some(Verdict::Correct) => {
                ui.label(
                    RichText::new(tr(T::Correct, lang))
                        .color(OK)
                        .size(18.0)
                        .strong(),
                );
            }
            Some(Verdict::Wrong) => {
                ui.label(RichText::new(tr(T::Wrong, lang)).color(DANGER).strong());
            }
            None => {}
        }
        if self.autopsy.revealed || self.autopsy.verdict == Some(Verdict::Correct) {
            ui.label(
                RichText::new(format!(
                    "{}: {}",
                    tr(T::Solution, lang),
                    autopsy::describe(&fault, lang)
                ))
                .color(CYAN),
            );
        }
    }

    fn log_tab(&mut self, ui: &mut egui::Ui) {
        let lang = self.lang;
        ui.heading(tr(T::EventLog, lang));
        if self.events.is_empty() {
            ui.label(RichText::new(tr(T::NoEvents, lang)).color(DIM));
        }
        egui::ScrollArea::vertical()
            .stick_to_bottom(true)
            .show(ui, |ui| {
                egui::Grid::new("events")
                    .striped(true)
                    .num_columns(3)
                    .show(ui, |ui| {
                        for ev in &self.events {
                            let (kind, text, color) = match ev.kind {
                                EventKind::ConditionChanged(c) => {
                                    (T::EventCondition, c.localized(lang).to_owned(), DIM)
                                }
                                EventKind::Warning(w) => {
                                    (T::EventWarning, w.localized(lang).to_owned(), WARN)
                                }
                                EventKind::Dtc(d) => (
                                    T::EventDtc,
                                    format!("{} – {}", d.code(), d.localized(lang)),
                                    WARN,
                                ),
                                EventKind::Damage(f) => {
                                    (T::EventDamage, f.localized(lang).to_owned(), DANGER)
                                }
                            };
                            ui.monospace(format!("{:9.2} s", ev.time_s));
                            ui.label(RichText::new(tr(kind, lang)).color(color));
                            ui.label(text);
                            ui.end_row();
                        }
                    });
            });
    }
}

/// Torque and power curves of the current run, with the previous run dimmed behind.
fn dyno_chart(
    ui: &mut egui::Ui,
    current: &crate::dyno::DynoRun,
    previous: Option<&crate::dyno::DynoRun>,
    redline: f32,
) {
    use egui::{Align2, CornerRadius, FontId, Pos2, Sense, Stroke, StrokeKind};
    // The visible width, so the kW axis never ends up under the side panel.
    let width = ui
        .available_width()
        .min(ui.clip_rect().right() - ui.cursor().min.x);
    let size = Vec2::new(width.max(320.0), 260.0);
    let (rect, _) = ui.allocate_exact_size(size, Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, CornerRadius::same(4), FACE);
    p.rect_stroke(
        rect,
        CornerRadius::same(4),
        Stroke::new(1.0, TRACK),
        StrokeKind::Inside,
    );
    let plot = rect.shrink2(Vec2::new(52.0, 24.0));
    let runs: Vec<&crate::dyno::DynoRun> = std::iter::once(current).chain(previous).collect();
    let tq_max = runs
        .iter()
        .flat_map(|r| r.points.iter().map(|p| p.torque_nm))
        .fold(50.0_f32, f32::max)
        * 1.15;
    let pw_max = runs
        .iter()
        .flat_map(|r| r.points.iter().map(|p| p.power_kw))
        .fold(20.0_f32, f32::max)
        * 1.15;
    let (x0, x1) = (1000.0, redline + 250.0);
    let to_x = |rpm: f32| plot.left() + plot.width() * ((rpm - x0) / (x1 - x0)).clamp(0.0, 1.0);
    for i in 0..=5 {
        let y = plot.bottom() - plot.height() * i as f32 / 5.0;
        p.hline(
            plot.x_range(),
            y,
            Stroke::new(1.0, Color32::from_rgb(38, 43, 53)),
        );
        p.text(
            Pos2::new(plot.left() - 6.0, y),
            Align2::RIGHT_CENTER,
            format!("{:.0}", tq_max * i as f32 / 5.0),
            FontId::monospace(10.0),
            ACCENT,
        );
        p.text(
            Pos2::new(plot.right() + 6.0, y),
            Align2::LEFT_CENTER,
            format!("{:.0}", pw_max * i as f32 / 5.0),
            FontId::monospace(10.0),
            BLUE,
        );
    }
    let mut rpm = 1000.0;
    while rpm <= x1 {
        p.text(
            Pos2::new(to_x(rpm), plot.bottom() + 4.0),
            Align2::CENTER_TOP,
            format!("{rpm:.0}"),
            FontId::monospace(10.0),
            DIM,
        );
        rpm += 1000.0;
    }
    p.text(
        rect.left_top() + Vec2::new(8.0, 4.0),
        Align2::LEFT_TOP,
        "N·m",
        FontId::proportional(12.0),
        ACCENT,
    );
    p.text(
        rect.right_top() + Vec2::new(-8.0, 4.0),
        Align2::RIGHT_TOP,
        "kW",
        FontId::proportional(12.0),
        BLUE,
    );
    for (k, run) in runs.iter().enumerate().rev() {
        let fade = if k == 0 { 1.0 } else { 0.35 };
        let tq: Vec<Pos2> = run
            .points
            .iter()
            .map(|q| {
                Pos2::new(
                    to_x(q.rpm),
                    plot.bottom() - plot.height() * q.torque_nm / tq_max,
                )
            })
            .collect();
        let pw: Vec<Pos2> = run
            .points
            .iter()
            .map(|q| {
                Pos2::new(
                    to_x(q.rpm),
                    plot.bottom() - plot.height() * q.power_kw / pw_max,
                )
            })
            .collect();
        if tq.len() >= 2 {
            p.line(tq, Stroke::new(2.0, ACCENT.gamma_multiply(fade)));
            p.line(pw, Stroke::new(2.0, BLUE.gamma_multiply(fade)));
        }
    }
}

/// Percentage slider that shows (and locks at) the value of a held key while it overrides
/// the panel setting.
fn held_slider(ui: &mut egui::Ui, value: &mut f32, held: Option<f32>) {
    match held {
        Some(v) => {
            let mut shown = v;
            ui.add_enabled(
                false,
                egui::Slider::new(&mut shown, 0.0..=100.0).suffix(" %"),
            );
        }
        None => {
            ui.add(egui::Slider::new(value, 0.0..=100.0).suffix(" %"));
        }
    }
}

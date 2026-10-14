//! Calibration map editor: a WinOLS / HP Tuners style grid with heat-map colouring, range
//! selection, step editing and the live operating point.

use eframe::egui::{
    self, Align2, Color32, CornerRadius, FontId, Key, Pos2, Rect, RichText, Sense, Stroke,
    StrokeKind, Vec2,
};
use tunerlab_core::engine_sim::calibration::Table2D;
use tunerlab_core::{Calibration, EngineSim, Language};

use crate::text::{cells, tr, T};
use crate::widgets::{heat, palette::*};

/// Editable 2-D maps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapKind {
    Ve,
    Ignition,
    Lambda,
    Boost,
}

impl MapKind {
    pub fn title(self) -> T {
        match self {
            MapKind::Ve => T::MapVe,
            MapKind::Ignition => T::MapIgnition,
            MapKind::Lambda => T::MapLambda,
            MapKind::Boost => T::MapBoost,
        }
    }

    /// Display value = stored value × scale (VE is stored as a fraction, shown in %).
    fn scale(self) -> f32 {
        match self {
            MapKind::Ve => 100.0,
            _ => 1.0,
        }
    }

    fn decimals(self) -> usize {
        match self {
            MapKind::Ve | MapKind::Ignition => 1,
            MapKind::Lambda => 2,
            MapKind::Boost => 0,
        }
    }

    /// Default edit step in display units.
    fn default_step(self) -> f32 {
        match self {
            MapKind::Ve => 1.0,
            MapKind::Ignition => 0.5,
            MapKind::Lambda => 0.01,
            MapKind::Boost => 5.0,
        }
    }

    fn y_axis_label(self) -> T {
        match self {
            MapKind::Boost => T::AxisPedal,
            _ => T::AxisMap,
        }
    }

    /// Range the core accepts for a cell, in display units (`Calibration::validate`).
    fn limits(self) -> (f32, f32) {
        match self {
            MapKind::Ve => (0.0, 200.0),
            MapKind::Ignition => (-30.0, 70.0),
            MapKind::Lambda => (0.5, 1.6),
            MapKind::Boost => (50.0, 500.0),
        }
    }

    /// Row-axis coordinate of the operating point, e.g. "MAP 25 kPa" or "pedał 30 %".
    fn y_value_text(self, v: f32, lang: Language) -> String {
        match self {
            MapKind::Boost => format!("{} {v:.0} %", tr(T::PedalShort, lang)),
            _ => format!("MAP {v:.0} kPa"),
        }
    }
}

/// Rectangular cell selection (inclusive, normalised so `x0 <= x1`, `y0 <= y1`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Selection {
    anchor: (usize, usize),
    end: (usize, usize),
}

impl Selection {
    fn bounds(&self) -> (usize, usize, usize, usize) {
        (
            self.anchor.0.min(self.end.0),
            self.anchor.1.min(self.end.1),
            self.anchor.0.max(self.end.0),
            self.anchor.1.max(self.end.1),
        )
    }

    fn count(&self) -> usize {
        let (x0, y0, x1, y1) = self.bounds();
        (x1 - x0 + 1) * (y1 - y0 + 1)
    }
}

/// Edit applied to the selected cells, in stored units.
#[derive(Clone, Copy)]
enum Op {
    Add(f32),
    Set(f32),
}

/// Snapshot of a map for drawing.
struct View {
    x: Vec<f32>,
    y: Vec<f32>,
    /// `v[y][x]`.
    v: Vec<Vec<f32>>,
}

fn view_of<const X: usize, const Y: usize>(t: &Table2D<X, Y>) -> View {
    View {
        x: t.x_axis.to_vec(),
        y: t.y_axis.to_vec(),
        v: t.values.iter().map(|row| row.to_vec()).collect(),
    }
}

fn edit<const X: usize, const Y: usize>(t: &mut Table2D<X, Y>, sel: &Selection, op: Op) {
    let (x0, y0, x1, y1) = sel.bounds();
    for row in t.values.iter_mut().take(y1.min(Y - 1) + 1).skip(y0) {
        for cell in row.iter_mut().take(x1.min(X - 1) + 1).skip(x0) {
            *cell = match op {
                Op::Add(d) => *cell + d,
                Op::Set(v) => v,
            };
        }
    }
}

fn table_view(cal: &Calibration, kind: MapKind) -> View {
    match kind {
        MapKind::Ve => view_of(&cal.ve),
        MapKind::Ignition => view_of(&cal.ignition),
        MapKind::Lambda => view_of(&cal.lambda_target),
        MapKind::Boost => view_of(&cal.boost.target_kpa),
    }
}

fn table_edit(cal: &mut Calibration, kind: MapKind, sel: &Selection, op: Op) {
    match kind {
        MapKind::Ve => edit(&mut cal.ve, sel, op),
        MapKind::Ignition => edit(&mut cal.ignition, sel, op),
        MapKind::Lambda => edit(&mut cal.lambda_target, sel, op),
        MapKind::Boost => edit(&mut cal.boost.target_kpa, sel, op),
    }
}

fn table_restore(cal: &mut Calibration, base: &Calibration, kind: MapKind) {
    match kind {
        MapKind::Ve => cal.ve = base.ve,
        MapKind::Ignition => cal.ignition = base.ignition,
        MapKind::Lambda => cal.lambda_target = base.lambda_target,
        MapKind::Boost => cal.boost.target_kpa = base.boost.target_kpa,
    }
}

/// Fractional index of `v` on a strictly increasing axis (clamped to its ends).
fn axis_position(axis: &[f32], v: f32) -> f32 {
    let n = axis.len();
    if n < 2 || !v.is_finite() || v <= axis[0] {
        return 0.0;
    }
    if v >= axis[n - 1] {
        return (n - 1) as f32;
    }
    let i = axis.windows(2).position(|w| v < w[1]).unwrap_or(n - 2);
    i as f32 + (v - axis[i]) / (axis[i + 1] - axis[i])
}

/// Map editor state.
pub struct MapEditor {
    pub kind: MapKind,
    selection: Option<Selection>,
    step: f32,
    set_value: f32,
    /// The last edit was rejected by the calibration validator.
    rejected: bool,
}

impl Default for MapEditor {
    fn default() -> Self {
        Self {
            kind: MapKind::Ve,
            selection: None,
            step: MapKind::Ve.default_step(),
            set_value: 0.0,
            rejected: false,
        }
    }
}

impl MapEditor {
    fn apply(&mut self, sim: &mut EngineSim, op: Op) {
        let Some(sel) = self.selection else { return };
        let kind = self.kind;
        self.rejected = sim
            .update_calibration(|cal| table_edit(cal, kind, &sel, op))
            .is_err();
    }

    /// Switches to another table with its own default step and no selection.
    fn select_kind(&mut self, kind: MapKind) {
        self.kind = kind;
        self.selection = None;
        self.step = kind.default_step();
        self.rejected = false;
    }

    /// Draws the editor.
    pub fn show(&mut self, ui: &mut egui::Ui, sim: &mut EngineSim, lang: Language) {
        let turbo = sim.calibration().engine.turbocharged;
        if self.kind == MapKind::Boost && !turbo {
            self.select_kind(MapKind::Ve);
        }
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new(tr(T::Table, lang)).color(DIM));
            let mut kinds = vec![MapKind::Ve, MapKind::Ignition, MapKind::Lambda];
            if turbo {
                kinds.push(MapKind::Boost);
            }
            for k in kinds {
                if ui
                    .selectable_label(self.kind == k, tr(k.title(), lang))
                    .clicked()
                    && self.kind != k
                {
                    self.select_kind(k);
                }
            }
        });
        ui.label(RichText::new(tr(T::MapHelp, lang)).color(DIM).size(12.0));
        ui.add_space(4.0);

        let kind = self.kind;
        let view = table_view(sim.calibration(), kind);
        let s = sim.telemetry().sensors;
        let op_x = axis_position(&view.x, s.rpm);
        let op_y = axis_position(
            &view.y,
            if kind == MapKind::Boost {
                s.pedal_pct
            } else {
                s.map_kpa
            },
        );

        // Edit toolbar.
        ui.horizontal_wrapped(|ui| {
            let count = self.selection.map_or(0, |s| s.count());
            ui.label(format!(
                "{}: {}",
                tr(T::Selection, lang),
                cells(count, lang)
            ));
            ui.separator();
            ui.label(tr(T::Step, lang));
            ui.add(
                egui::DragValue::new(&mut self.step)
                    .speed(0.01)
                    .range(0.0..=1000.0),
            );
            let scale = kind.scale();
            if ui.button(format!("− {}", self.step)).clicked() {
                self.apply(sim, Op::Add(-self.step / scale));
            }
            if ui.button(format!("+ {}", self.step)).clicked() {
                self.apply(sim, Op::Add(self.step / scale));
            }
            ui.separator();
            ui.label(tr(T::SetTo, lang));
            ui.add(egui::DragValue::new(&mut self.set_value).speed(0.1));
            if ui.button(tr(T::Apply, lang)).clicked() {
                self.apply(sim, Op::Set(self.set_value / scale));
            }
            ui.separator();
            if ui.button(tr(T::RestoreBase, lang)).clicked() {
                let base = Calibration::base_for(sim.spec());
                self.rejected = sim
                    .update_calibration(|cal| table_restore(cal, &base, kind))
                    .is_err();
            }
        });
        if self.rejected {
            let (lo, hi) = kind.limits();
            let d = kind.decimals();
            ui.label(
                RichText::new(format!(
                    "{}: {} ({lo:.d$} … {hi:.d$})",
                    tr(T::CalibrationRejected, lang),
                    tr(T::ValueOutOfRange, lang),
                ))
                .color(DANGER),
            );
        }
        ui.label(
            RichText::new(format!(
                "{}: {:.0} {} · {}",
                tr(T::OperatingPoint, lang),
                s.rpm,
                tr(T::AxisRpm, lang),
                kind.y_value_text(
                    if kind == MapKind::Boost {
                        s.pedal_pct
                    } else {
                        s.map_kpa
                    },
                    lang
                ),
            ))
            .color(CYAN),
        );

        // Keyboard step editing (not while a number is being typed).
        if !ui.ctx().text_edit_focused() {
            let (plus, minus) = ui.input(|i| {
                (
                    i.key_pressed(Key::Plus) || i.key_pressed(Key::Equals),
                    i.key_pressed(Key::Minus),
                )
            });
            let scale = kind.scale();
            if plus {
                self.apply(sim, Op::Add(self.step / scale));
            }
            if minus {
                self.apply(sim, Op::Add(-self.step / scale));
            }
        }

        let view = table_view(sim.calibration(), kind);
        self.grid(ui, &view, (op_x, op_y), lang);
    }

    fn grid(&mut self, ui: &mut egui::Ui, view: &View, op: (f32, f32), lang: Language) {
        let kind = self.kind;
        let nx = view.x.len();
        let ny = view.y.len();
        let header_w = 64.0;
        let header_h = 22.0;
        let avail = ui.available_size();
        // The visible width: a wide row above must not push the grid under the side panel.
        let width = avail.x.min(ui.clip_rect().right() - ui.cursor().min.x);
        let cell_w = ((width - header_w - 4.0) / nx as f32).clamp(30.0, 72.0);
        let cell_h = ((avail.y - header_h - 24.0) / ny as f32).clamp(16.0, 26.0);
        let size = Vec2::new(header_w + cell_w * nx as f32, header_h + cell_h * ny as f32);
        let (resp, p) = ui.allocate_painter(size, Sense::click_and_drag());
        let origin = resp.rect.min;
        let cell_rect = |x: usize, y: usize| {
            Rect::from_min_size(
                origin + Vec2::new(header_w + cell_w * x as f32, header_h + cell_h * y as f32),
                Vec2::new(cell_w, cell_h),
            )
        };
        let hit = |pos: Pos2| -> Option<(usize, usize)> {
            let local = pos - origin - Vec2::new(header_w, header_h);
            if local.x < 0.0 || local.y < 0.0 {
                return None;
            }
            let x = (local.x / cell_w) as usize;
            let y = (local.y / cell_h) as usize;
            (x < nx && y < ny).then_some((x, y))
        };

        // Selection by click, Shift+click and drag.
        let shift = ui.input(|i| i.modifiers.shift);
        if resp.clicked() || resp.drag_started() {
            if let Some(cell) = resp.interact_pointer_pos().and_then(hit) {
                match (&mut self.selection, shift) {
                    (Some(sel), true) => sel.end = cell,
                    _ => {
                        self.selection = Some(Selection {
                            anchor: cell,
                            end: cell,
                        })
                    }
                }
                let v = view.v[cell.1][cell.0] * kind.scale();
                self.set_value = v;
            }
        } else if resp.dragged() {
            if let (Some(cell), Some(sel)) = (
                resp.interact_pointer_pos().and_then(hit),
                &mut self.selection,
            ) {
                sel.end = cell;
            }
        }

        let (lo, hi) = view
            .v
            .iter()
            .flatten()
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), &v| {
                (lo.min(v), hi.max(v))
            });
        let span = (hi - lo).max(1.0e-6);
        let font = FontId::monospace(if cell_w < 36.0 {
            9.0
        } else if cell_w < 44.0 {
            10.0
        } else {
            11.5
        });
        // Corner cell: column axis top-right, row axis bottom-left.
        p.text(
            origin + Vec2::new(header_w - 3.0, 2.0),
            Align2::RIGHT_TOP,
            tr(T::AxisRpm, lang),
            FontId::proportional(9.0),
            DIM,
        );
        p.text(
            origin + Vec2::new(3.0, header_h - 1.0),
            Align2::LEFT_BOTTOM,
            tr(kind.y_axis_label(), lang),
            FontId::proportional(9.0),
            DIM,
        );
        for (x, v) in view.x.iter().enumerate() {
            let r = Rect::from_min_size(
                origin + Vec2::new(header_w + cell_w * x as f32, 0.0),
                Vec2::new(cell_w, header_h),
            );
            p.rect_filled(r.shrink(0.5), CornerRadius::ZERO, PANEL);
            p.text(
                r.center(),
                Align2::CENTER_CENTER,
                format!("{v:.0}"),
                font.clone(),
                TEXT,
            );
        }
        for (y, v) in view.y.iter().enumerate() {
            let r = Rect::from_min_size(
                origin + Vec2::new(0.0, header_h + cell_h * y as f32),
                Vec2::new(header_w, cell_h),
            );
            p.rect_filled(r.shrink(0.5), CornerRadius::ZERO, PANEL);
            p.text(
                r.center(),
                Align2::CENTER_CENTER,
                format!("{v:.0}"),
                font.clone(),
                TEXT,
            );
        }
        for y in 0..ny {
            for x in 0..nx {
                let v = view.v[y][x];
                let r = cell_rect(x, y);
                p.rect_filled(
                    r.shrink(0.5),
                    CornerRadius::ZERO,
                    heat((v - lo) / span).gamma_multiply(0.85),
                );
                p.text(
                    r.center(),
                    Align2::CENTER_CENTER,
                    format!("{:.*}", kind.decimals(), v * kind.scale()),
                    font.clone(),
                    Color32::BLACK,
                );
            }
        }
        if let Some(sel) = self.selection {
            let (x0, y0, x1, y1) = sel.bounds();
            let r = cell_rect(x0, y0).union(cell_rect(x1, y1));
            p.rect_stroke(
                r,
                CornerRadius::ZERO,
                Stroke::new(2.0, Color32::WHITE),
                StrokeKind::Inside,
            );
        }
        // Live operating point (interpolated position between breakpoints).
        let c = origin
            + Vec2::new(
                header_w + cell_w * (op.0 + 0.5),
                header_h + cell_h * (op.1 + 0.5),
            );
        p.circle_stroke(c, 7.0, Stroke::new(2.5, Color32::WHITE));
        p.circle_filled(c, 3.0, ACCENT);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tunerlab_core::EngineSpec;

    #[test]
    fn axis_position_interpolates_and_clamps() {
        let axis = [500.0, 1000.0, 2000.0];
        assert_eq!(axis_position(&axis, 100.0), 0.0);
        assert_eq!(axis_position(&axis, 750.0), 0.5);
        assert_eq!(axis_position(&axis, 1500.0), 1.5);
        assert_eq!(axis_position(&axis, 9000.0), 2.0);
        assert_eq!(axis_position(&axis, f32::NAN), 0.0);
    }

    #[test]
    fn edits_touch_only_the_selection_and_restore_brings_the_base_back() {
        let spec = EngineSpec::naturally_aspirated_2l();
        let base = Calibration::base_for(&spec);
        let mut cal = base;
        let sel = Selection {
            anchor: (3, 5),
            end: (1, 2),
        };
        table_edit(&mut cal, MapKind::Ignition, &sel, Op::Add(2.0));
        for y in 0..cal.ignition.values.len() {
            for x in 0..cal.ignition.values[0].len() {
                let d = cal.ignition.values[y][x] - base.ignition.values[y][x];
                let inside = (1..=3).contains(&x) && (2..=5).contains(&y);
                assert_eq!(d, if inside { 2.0 } else { 0.0 }, "cell {x},{y}");
            }
        }
        table_edit(&mut cal, MapKind::Ve, &sel, Op::Set(0.5));
        assert_eq!(cal.ve.values[4][2], 0.5);
        assert!(cal.validate().is_ok());
        table_restore(&mut cal, &base, MapKind::Ignition);
        table_restore(&mut cal, &base, MapKind::Ve);
        assert_eq!(cal, base);
    }

    #[test]
    fn invalid_edits_are_rejected_by_the_ecu() {
        let mut sim =
            EngineSim::with_base_calibration(EngineSpec::naturally_aspirated_2l(), 1).unwrap();
        let before = *sim.calibration();
        let mut editor = MapEditor {
            selection: Some(Selection {
                anchor: (0, 0),
                end: (0, 0),
            }),
            kind: MapKind::Lambda,
            ..MapEditor::default()
        };
        editor.apply(&mut sim, Op::Set(-1.0));
        assert!(editor.rejected);
        assert_eq!(*sim.calibration(), before);
    }
}

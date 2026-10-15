//! Instrument-cluster widgets drawn directly with the egui painter: round gauges, value
//! tiles, health bars, scrolling charts and the heat-map colour scale.

use eframe::egui::{
    self, Align2, Color32, CornerRadius, FontId, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2,
};

/// Colour palettes of the skins, switchable at run time.
pub mod palette {
    use std::sync::atomic::{AtomicUsize, Ordering};

    use eframe::egui::Color32;

    /// Colours of one skin.
    pub struct Palette {
        pub background: Color32,
        pub panel: Color32,
        /// Gauge faces, tiles and chart backgrounds.
        pub face: Color32,
        /// Gauge tracks, borders and empty bars.
        pub track: Color32,
        /// Chart grid lines.
        pub grid: Color32,
        pub text: Color32,
        pub dim: Color32,
        pub accent: Color32,
        pub ok: Color32,
        pub warn: Color32,
        pub danger: Color32,
        pub blue: Color32,
        pub cyan: Color32,
        pub violet: Color32,
        /// Built on egui's dark visuals (otherwise the light ones).
        pub dark: bool,
    }

    /// Available skins.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Skin {
        /// Dark motorsport standalone-ECU look, orange accents.
        Motorsport,
        /// Navy tuner-laptop look, blue accents.
        Navy,
        /// Light map-editor look for bright rooms and projectors.
        Light,
    }

    impl Skin {
        pub const ALL: [Skin; 3] = [Skin::Motorsport, Skin::Navy, Skin::Light];
    }

    const fn rgb(r: u8, g: u8, b: u8) -> Color32 {
        Color32::from_rgb(r, g, b)
    }

    static PALETTES: [Palette; 3] = [
        Palette {
            background: rgb(14, 16, 20),
            panel: rgb(22, 25, 31),
            face: rgb(28, 32, 40),
            track: rgb(52, 58, 70),
            grid: rgb(38, 43, 53),
            text: rgb(222, 226, 232),
            dim: rgb(130, 138, 150),
            accent: rgb(255, 122, 26),
            ok: rgb(60, 200, 110),
            warn: rgb(255, 196, 0),
            danger: rgb(235, 56, 56),
            blue: rgb(70, 150, 255),
            cyan: rgb(60, 210, 220),
            violet: rgb(170, 120, 255),
            dark: true,
        },
        Palette {
            background: rgb(12, 18, 30),
            panel: rgb(19, 27, 42),
            face: rgb(25, 36, 55),
            track: rgb(50, 66, 92),
            grid: rgb(34, 47, 68),
            text: rgb(226, 232, 242),
            dim: rgb(134, 150, 175),
            accent: rgb(45, 150, 255),
            ok: rgb(70, 210, 130),
            warn: rgb(255, 190, 50),
            danger: rgb(240, 70, 80),
            blue: rgb(120, 190, 255),
            cyan: rgb(80, 225, 210),
            violet: rgb(190, 140, 255),
            dark: true,
        },
        Palette {
            background: rgb(226, 230, 236),
            panel: rgb(240, 242, 245),
            face: rgb(252, 252, 253),
            track: rgb(188, 195, 206),
            grid: rgb(222, 226, 232),
            text: rgb(24, 30, 40),
            dim: rgb(96, 105, 120),
            accent: rgb(0, 96, 200),
            ok: rgb(16, 140, 70),
            warn: rgb(196, 124, 0),
            danger: rgb(204, 32, 40),
            blue: rgb(30, 100, 220),
            cyan: rgb(0, 138, 160),
            violet: rgb(120, 60, 200),
            dark: false,
        },
    ];

    static CURRENT: AtomicUsize = AtomicUsize::new(0);

    /// Palette of the active skin.
    pub fn pal() -> &'static Palette {
        &PALETTES[CURRENT.load(Ordering::Relaxed).min(PALETTES.len() - 1)]
    }

    pub fn skin() -> Skin {
        Skin::ALL[CURRENT.load(Ordering::Relaxed).min(Skin::ALL.len() - 1)]
    }

    pub fn set_skin(skin: Skin) {
        CURRENT.store(skin as usize, Ordering::Relaxed);
    }
}

use palette::*;

/// Start of the 270° gauge sweep, measured clockwise from +x on screen (y points down).
const SWEEP_START_DEG: f32 = 135.0;
const SWEEP_DEG: f32 = 270.0;

/// Description of a round gauge.
pub struct Gauge<'a> {
    pub label: &'a str,
    pub unit: &'a str,
    pub min: f32,
    pub max: f32,
    pub value: f32,
    pub decimals: usize,
    /// Label spacing on the scale.
    pub major_step: f32,
    /// Scale labels are divided by this (e.g. 1000 for a tachometer in ×1000 rpm).
    pub label_divisor: f32,
    /// Red zone `[from, to]`.
    pub danger: Option<(f32, f32)>,
    /// Amber zone `[from, to]`.
    pub caution: Option<(f32, f32)>,
    /// Secondary marker (e.g. a target value) and its colour.
    pub marker: Option<(f32, Color32)>,
}

impl<'a> Gauge<'a> {
    pub fn new(label: &'a str, unit: &'a str, min: f32, max: f32, value: f32) -> Self {
        Self {
            label,
            unit,
            min,
            max,
            value,
            decimals: 0,
            major_step: (max - min) / 8.0,
            label_divisor: 1.0,
            danger: None,
            caution: None,
            marker: None,
        }
    }

    pub fn decimals(mut self, d: usize) -> Self {
        self.decimals = d;
        self
    }

    pub fn step(mut self, step: f32) -> Self {
        self.major_step = step;
        self
    }

    pub fn divisor(mut self, divisor: f32) -> Self {
        self.label_divisor = divisor;
        self
    }

    pub fn danger(mut self, from: f32, to: f32) -> Self {
        self.danger = Some((from, to));
        self
    }

    pub fn caution(mut self, from: f32, to: f32) -> Self {
        self.caution = Some((from, to));
        self
    }

    pub fn marker(mut self, value: f32, color: Color32) -> Self {
        self.marker = Some((value, color));
        self
    }

    fn fraction(&self, v: f32) -> f32 {
        if !v.is_finite() || self.max <= self.min {
            return 0.0;
        }
        ((v - self.min) / (self.max - self.min)).clamp(0.0, 1.0)
    }

    /// Whether the value lies in `zone`; a value off the scale counts as the end it is
    /// pinned to, so an end-stop reading keeps the colour of the zone it is beyond.
    fn in_zone(&self, zone: Option<(f32, f32)>) -> bool {
        if !self.value.is_finite() {
            return false;
        }
        let v = self.value.clamp(self.min, self.max);
        zone.is_some_and(|(a, b)| v >= a && v <= b)
    }
}

fn polar(center: Pos2, radius: f32, fraction: f32) -> Pos2 {
    let a = (SWEEP_START_DEG + SWEEP_DEG * fraction).to_radians();
    center + Vec2::new(a.cos(), a.sin()) * radius
}

fn arc(center: Pos2, radius: f32, f0: f32, f1: f32) -> Vec<Pos2> {
    let n = (((f1 - f0).abs() * 96.0).ceil() as usize).max(2);
    (0..=n)
        .map(|i| polar(center, radius, f0 + (f1 - f0) * i as f32 / n as f32))
        .collect()
}

/// Draws a round gauge of the given diameter.
pub fn round_gauge(ui: &mut egui::Ui, diameter: f32, g: &Gauge) {
    let (rect, _) = ui.allocate_exact_size(Vec2::splat(diameter), Sense::hover());
    let p = ui.painter_at(rect);
    let c = rect.center();
    let r = diameter * 0.5 - 2.0;
    p.circle_filled(c, r, pal().face);
    p.circle_stroke(c, r, Stroke::new(1.5, pal().track));

    let ring = r * 0.86;
    let width = (r * 0.07).max(3.0);
    p.line(arc(c, ring, 0.0, 1.0), Stroke::new(width, pal().track));
    if let Some((a, b)) = g.caution {
        p.line(
            arc(c, ring, g.fraction(a), g.fraction(b)),
            Stroke::new(width, pal().warn),
        );
    }
    if let Some((a, b)) = g.danger {
        p.line(
            arc(c, ring, g.fraction(a), g.fraction(b)),
            Stroke::new(width, pal().danger),
        );
    }
    let state = if g.in_zone(g.danger) {
        pal().danger
    } else if g.in_zone(g.caution) {
        pal().warn
    } else {
        pal().accent
    };
    let fv = g.fraction(g.value);
    if fv > 0.0 {
        p.line(arc(c, ring, 0.0, fv), Stroke::new(width, state));
    }

    // Scale: major ticks with labels, minor ticks halfway.
    if g.major_step > 0.0 && g.max > g.min {
        // Labels only up to the end of the scale (a partial last step gets no label).
        let n = ((g.max - g.min) / g.major_step + 1.0e-3).floor() as usize;
        let label_font = FontId::proportional((r * 0.13).clamp(9.0, 15.0));
        for i in 0..=n {
            let v = g.min + g.major_step * i as f32;
            let f = g.fraction(v);
            p.line_segment(
                [
                    polar(c, ring - width * 0.5 - r * 0.02, f),
                    polar(c, ring - width * 0.5 - r * 0.11, f),
                ],
                Stroke::new(1.5, pal().text),
            );
            if g.min + g.major_step * (i as f32 + 0.5) <= g.max + 1.0e-3 {
                let fm = g.fraction(v + g.major_step * 0.5);
                p.line_segment(
                    [
                        polar(c, ring - width * 0.5 - r * 0.02, fm),
                        polar(c, ring - width * 0.5 - r * 0.06, fm),
                    ],
                    Stroke::new(1.0, pal().dim),
                );
            }
            let shown = v / g.label_divisor;
            let text = if (shown - shown.round()).abs() < 1.0e-3 {
                format!("{:.0}", shown)
            } else {
                format!("{:.1}", shown)
            };
            p.text(
                polar(c, r * 0.6, f),
                Align2::CENTER_CENTER,
                text,
                label_font.clone(),
                pal().dim,
            );
        }
    }
    if let Some((m, color)) = g.marker {
        let f = g.fraction(m);
        p.line_segment(
            [polar(c, ring - width, f), polar(c, ring + width, f)],
            Stroke::new(3.0, color),
        );
    }

    // Needle.
    let tip = polar(c, ring - width * 0.2, fv);
    p.line_segment([c, tip], Stroke::new((r * 0.035).max(2.0), state));
    p.circle_filled(c, r * 0.07, pal().track);

    let value = if g.value.is_finite() {
        format!("{:.*}", g.decimals, g.value)
    } else {
        "—".to_owned()
    };
    p.text(
        c + Vec2::new(0.0, r * 0.38),
        Align2::CENTER_CENTER,
        value,
        FontId::monospace((r * 0.24).clamp(12.0, 34.0)),
        pal().text,
    );
    p.text(
        c + Vec2::new(0.0, r * 0.62),
        Align2::CENTER_CENTER,
        format!("{} {}", g.label, g.unit),
        FontId::proportional((r * 0.12).clamp(9.0, 14.0)),
        pal().dim,
    );
}

/// Severity colour of a value tile.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Level {
    Normal,
    Caution,
    Danger,
    Off,
}

impl Level {
    fn color(self) -> Color32 {
        match self {
            Level::Normal => pal().text,
            Level::Caution => pal().warn,
            Level::Danger => pal().danger,
            Level::Off => pal().dim,
        }
    }
}

/// Rectangular digital readout: label, value and unit (or a short secondary text).
pub fn tile(ui: &mut egui::Ui, size: Vec2, label: &str, value: &str, unit: &str, level: Level) {
    let (rect, _) = ui.allocate_exact_size(size, Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, CornerRadius::same(4), pal().face);
    let border = match level {
        Level::Danger => pal().danger,
        Level::Caution => pal().warn,
        _ => pal().track,
    };
    p.rect_stroke(
        rect,
        CornerRadius::same(4),
        Stroke::new(1.0, border),
        StrokeKind::Inside,
    );
    p.text(
        rect.left_top() + Vec2::new(8.0, 6.0),
        Align2::LEFT_TOP,
        label,
        FontId::proportional(12.0),
        pal().dim,
    );
    // The unit follows the value on its baseline; when both do not fit, the value font
    // shrinks so the two never overlap.
    let unit_font = FontId::proportional(12.0);
    let unit_w = p
        .layout_no_wrap(unit.to_owned(), unit_font.clone(), pal().dim)
        .size()
        .x;
    let mut value_size = (size.y * 0.42).clamp(12.0, 26.0);
    let value_w = p
        .layout_no_wrap(value.to_owned(), FontId::monospace(value_size), pal().text)
        .size()
        .x;
    let room = size.x - 16.0 - 5.0 - unit_w;
    if value_w > room && value_w > 0.0 {
        value_size = (value_size * room / value_w).max(10.0);
    }
    let value_w = value_w * value_size / (size.y * 0.42).clamp(12.0, 26.0);
    let baseline = rect.bottom() - 6.0;
    p.text(
        Pos2::new(rect.left() + 8.0, baseline),
        Align2::LEFT_BOTTOM,
        value,
        FontId::monospace(value_size),
        level.color(),
    );
    p.text(
        Pos2::new(rect.left() + 8.0 + value_w + 5.0, baseline - 2.0),
        Align2::LEFT_BOTTOM,
        unit,
        unit_font,
        pal().dim,
    );
}

/// Labelled horizontal bar 0..1 (component health).
pub fn health_bar(ui: &mut egui::Ui, width: f32, label: &str, value: f32) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(width, 18.0), Sense::hover());
    let p = ui.painter_at(rect);
    let v = if value.is_finite() {
        value.clamp(0.0, 1.0)
    } else {
        0.0
    };
    let bar = Rect::from_min_size(
        Pos2::new(rect.left() + width * 0.45, rect.top() + 4.0),
        Vec2::new(width * 0.55 - 40.0, 10.0),
    );
    p.text(
        Pos2::new(rect.left(), rect.center().y),
        Align2::LEFT_CENTER,
        label,
        FontId::proportional(12.0),
        pal().dim,
    );
    p.rect_filled(bar, CornerRadius::same(2), pal().track);
    let color = if v > 0.8 {
        pal().ok
    } else if v > 0.4 {
        pal().warn
    } else {
        pal().danger
    };
    p.rect_filled(
        Rect::from_min_size(bar.min, Vec2::new(bar.width() * v, bar.height())),
        CornerRadius::same(2),
        color,
    );
    p.text(
        Pos2::new(rect.right(), rect.center().y),
        Align2::RIGHT_CENTER,
        format!("{:.0} %", v * 100.0),
        FontId::monospace(11.0),
        pal().text,
    );
}

/// One line of a chart; a non-empty `label` adds a legend entry.
pub struct Series<'a> {
    pub values: &'a [f32],
    pub color: Color32,
    pub label: &'a str,
}

/// Scrolling line chart over a fixed number of samples, newest on the right.
pub fn chart(
    ui: &mut egui::Ui,
    size: Vec2,
    title: &str,
    capacity: usize,
    y_range: (f32, f32),
    series: &[Series],
) {
    let (rect, _) = ui.allocate_exact_size(size, Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, CornerRadius::same(4), pal().face);
    p.rect_stroke(
        rect,
        CornerRadius::same(4),
        Stroke::new(1.0, pal().track),
        StrokeKind::Inside,
    );
    let plot = rect
        .shrink2(Vec2::new(42.0, 18.0))
        .translate(Vec2::new(16.0, 6.0));
    let (y0, y1) = y_range;
    let span = (y1 - y0).max(1.0e-6);
    for i in 0..=4 {
        let v = y0 + span * i as f32 / 4.0;
        let y = plot.bottom() - plot.height() * i as f32 / 4.0;
        p.hline(plot.x_range(), y, Stroke::new(1.0, pal().grid));
        let text = if span >= 20.0 {
            format!("{v:.0}")
        } else if span >= 2.0 {
            format!("{v:.1}")
        } else {
            format!("{v:.2}")
        };
        p.text(
            Pos2::new(plot.left() - 6.0, y),
            Align2::RIGHT_CENTER,
            text,
            FontId::monospace(10.0),
            pal().dim,
        );
    }
    p.text(
        rect.left_top() + Vec2::new(8.0, 4.0),
        Align2::LEFT_TOP,
        title,
        FontId::proportional(12.0),
        pal().dim,
    );
    // Legend, right-aligned on the title row.
    let mut x = rect.right() - 8.0;
    for s in series.iter().rev().filter(|s| !s.label.is_empty()) {
        let text = p.text(
            Pos2::new(x, rect.top() + 4.0),
            Align2::RIGHT_TOP,
            s.label,
            FontId::proportional(11.0),
            pal().text,
        );
        let swatch = Rect::from_center_size(
            Pos2::new(text.left() - 9.0, text.center().y),
            Vec2::new(12.0, 3.0),
        );
        p.rect_filled(swatch, CornerRadius::ZERO, s.color);
        x = swatch.left() - 10.0;
    }
    let cap = capacity.max(2);
    for s in series {
        let n = s.values.len();
        // A missing sample (NaN: engine stopped, reading invalid) breaks the line instead of
        // being bridged by a straight segment.
        let mut run: Vec<Pos2> = Vec::new();
        for (i, v) in s.values.iter().enumerate() {
            if v.is_finite() {
                let x = plot.right() - plot.width() * (n - 1 - i) as f32 / (cap - 1) as f32;
                let y = plot.bottom() - plot.height() * ((v - y0) / span).clamp(-0.02, 1.02);
                run.push(Pos2::new(x, y));
            } else if !run.is_empty() {
                draw_run(&p, std::mem::take(&mut run), s.color);
            }
        }
        draw_run(&p, run, s.color);
    }
}

fn draw_run(p: &egui::Painter, run: Vec<Pos2>, color: Color32) {
    match run.len() {
        0 => {}
        1 => {
            p.circle_filled(run[0], 1.2, color);
        }
        _ => {
            p.line(run, Stroke::new(1.6, color));
        }
    }
}

/// Heat-map colour for a normalised value 0..1 (blue → green → yellow → red).
pub fn heat(t: f32) -> Color32 {
    let t = if t.is_finite() {
        t.clamp(0.0, 1.0)
    } else {
        0.0
    };
    let stops = [
        (0.0, [40.0, 80.0, 190.0]),
        (0.33, [40.0, 170.0, 120.0]),
        (0.66, [230.0, 200.0, 40.0]),
        (1.0, [220.0, 60.0, 50.0]),
    ];
    let mut i = 0;
    while i + 2 < stops.len() && t > stops[i + 1].0 {
        i += 1;
    }
    let (t0, c0) = stops[i];
    let (t1, c1) = stops[i + 1];
    let f = ((t - t0) / (t1 - t0)).clamp(0.0, 1.0);
    let ch = |k: usize| (c0[k] + (c1[k] - c0[k]) * f) as u8;
    Color32::from_rgb(ch(0), ch(1), ch(2))
}

/// A warning lamp: coloured dot plus text.
pub fn lamp(ui: &mut egui::Ui, text: &str, color: Color32) {
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(Vec2::splat(12.0), Sense::hover());
        ui.painter().circle_filled(rect.center(), 5.0, color);
        ui.label(egui::RichText::new(text).color(pal().text));
    });
}

//! TunerLAB desktop application.
//!
//! A motorsport-standalone style instrument cluster around the Phase 1 engine simulation:
//! live gauges and charts, driver controls, warnings and trouble codes, a live calibration
//! map editor, a dyno power sweep and the Engine Autopsy diagnostic game, in Polish and
//! English.

#![forbid(unsafe_code)]
// Release builds on Windows start without a console window.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod app;
mod autopsy;
mod course;
mod dyno;
mod history;
mod maps;
mod text;
mod widgets;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_title("TunerLAB")
            .with_inner_size([1600.0, 960.0])
            .with_min_inner_size([1200.0, 720.0]),
        ..Default::default()
    };
    eframe::run_native(
        "TunerLAB",
        options,
        Box::new(|cc| Ok(Box::new(app::TunerApp::new(cc)))),
    )
}

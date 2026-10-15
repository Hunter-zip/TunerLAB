//! Map files: the editable calibration maps (VE, ignition, target λ and, on turbo engines,
//! boost target) saved to and loaded from human-readable text files in a `mapy` folder
//! next to the executable.
//!
//! Format version 1, one item per line; blank lines and lines starting with `#` are
//! ignored:
//!
//! ```text
//! TunerLAB maps 1
//! engine na                    (or `engine turbo`)
//! table ve 16 16               (name, columns, rows)
//! x 500 800 1200 …             (column axis: rpm)
//! y 20 25 30 …                 (row axis: MAP kPa, pedal % for the boost map)
//! v 0.62 0.64 0.66 …           (one line per row, in the core's stored units)
//! table ignition 16 16
//! …
//! ```
//!
//! Numbers are written in Rust's shortest round-trip form, so a saved map loads back
//! bit-for-bit. Parsing is strict: every problem is reported as a [`ParseError`] naming the
//! line and table, and a file for the other engine type is refused.

use std::ffi::OsStr;
use std::fs;
use std::io::{self, Write as _};
use std::path::{Path, PathBuf};

use eframe::egui::{self, Color32, Key, RichText};
use tunerlab_core::engine_sim::calibration::{BoostMap, FuelSparkMap, Table2D};
use tunerlab_core::{Calibration, CalibrationError, EngineSim, Language, SimError};

use crate::widgets::palette::*;

/// First line of every map file.
const HEADER: &str = "TunerLAB maps 1";
/// Folder next to the executable that holds the map files.
const DIR_NAME: &str = "mapy";
/// File extension of map files (without the dot).
pub const EXTENSION: &str = "tlmap";
/// Longest accepted file name (without the extension), in characters.
const MAX_NAME_CHARS: usize = 64;
/// Longest piece of a bad line quoted back in an error message, in characters.
const MAX_QUOTE_CHARS: usize = 24;

/// Table names used in the file (the core's field paths).
const VE: &str = "ve";
const IGNITION: &str = "ignition";
const LAMBDA: &str = "lambda_target";
const BOOST: &str = "boost.target_kpa";

/// `en` or `pl`, whichever `lang` asks for.
fn l(lang: Language, en: &'static str, pl: &'static str) -> &'static str {
    match lang {
        Language::En => en,
        Language::Pl => pl,
    }
}

/// [`l`] for messages with formatted values.
fn lf(lang: Language, en: String, pl: String) -> String {
    match lang {
        Language::En => en,
        Language::Pl => pl,
    }
}

/// The editable maps of one engine, as stored in a map file.
#[derive(Debug, Clone, PartialEq)]
pub struct MapSet {
    pub ve: FuelSparkMap,
    pub ignition: FuelSparkMap,
    pub lambda_target: FuelSparkMap,
    /// Boost target; `None` for a naturally aspirated engine.
    pub boost_target: Option<BoostMap>,
}

impl MapSet {
    /// The maps of `cal` (the boost target only when the engine is turbocharged).
    pub fn from_calibration(cal: &Calibration) -> Self {
        Self {
            ve: cal.ve,
            ignition: cal.ignition,
            lambda_target: cal.lambda_target,
            boost_target: cal.engine.turbocharged.then_some(cal.boost.target_kpa),
        }
    }

    /// The set belongs to a turbocharged engine.
    pub fn turbo(&self) -> bool {
        self.boost_target.is_some()
    }

    /// Writes the maps into `cal`, leaving every other setting untouched.
    pub fn apply_to(&self, cal: &mut Calibration) {
        cal.ve = self.ve;
        cal.ignition = self.ignition;
        cal.lambda_target = self.lambda_target;
        if let Some(boost) = self.boost_target {
            cal.boost.target_kpa = boost;
        }
    }
}

/// Map-file text of `set`.
pub fn serialize(set: &MapSet) -> String {
    let mut s = String::with_capacity(8 * 1024);
    s.push_str(HEADER);
    s.push('\n');
    s.push_str(if set.turbo() {
        "engine turbo\n"
    } else {
        "engine na\n"
    });
    write_table(&mut s, VE, &set.ve);
    write_table(&mut s, IGNITION, &set.ignition);
    write_table(&mut s, LAMBDA, &set.lambda_target);
    if let Some(boost) = &set.boost_target {
        write_table(&mut s, BOOST, boost);
    }
    s
}

fn write_table<const X: usize, const Y: usize>(s: &mut String, name: &str, t: &Table2D<X, Y>) {
    s.push_str(&format!("\ntable {name} {X} {Y}\n"));
    write_row(s, RowTag::X, &t.x_axis);
    write_row(s, RowTag::Y, &t.y_axis);
    for row in &t.values {
        write_row(s, RowTag::V, row);
    }
}

fn write_row(s: &mut String, tag: RowTag, values: &[f32]) {
    s.push_str(tag.word());
    for v in values {
        s.push(' ');
        // `Display` of f32 is the shortest text that parses back to the same bits.
        s.push_str(&v.to_string());
    }
    s.push('\n');
}

/// Kind of a numeric line inside a table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RowTag {
    /// Column axis.
    X,
    /// Row axis.
    Y,
    /// One row of cell values.
    V,
}

impl RowTag {
    fn word(self) -> &'static str {
        match self {
            RowTag::X => "x",
            RowTag::Y => "y",
            RowTag::V => "v",
        }
    }

    /// What the line holds, as the object of "expected …".
    fn describe(self, lang: Language) -> &'static str {
        match self {
            RowTag::X => l(lang, "the column axis (x …)", "osi kolumn (x …)"),
            RowTag::Y => l(lang, "the row axis (y …)", "osi wierszy (y …)"),
            RowTag::V => l(lang, "a row of values (v …)", "wiersza wartości (v …)"),
        }
    }
}

/// Why a map file could not be read. Line numbers are 1-based and count every line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParseError {
    /// The file is empty or its first line is not a TunerLAB map header.
    BadHeader,
    /// The header names a format version this build cannot read.
    UnsupportedVersion(String),
    /// The line after the header is not `engine na` / `engine turbo`.
    BadEngine { line: usize },
    /// The file holds maps of the other engine type.
    WrongEngine { file_turbo: bool },
    /// A line that should start a table is not `table <name> <columns> <rows>`.
    BadTableHeader { line: usize },
    /// The table name is unknown or not used by this engine.
    UnknownTable { line: usize, name: String },
    /// The table appears a second time.
    DuplicateTable { line: usize, table: &'static str },
    /// The table header declares other dimensions than the ECU's map (columns, rows).
    WrongSize {
        line: usize,
        table: &'static str,
        expected: (usize, usize),
        found: (usize, usize),
    },
    /// A line inside a table does not start with the expected tag.
    ExpectedRow {
        line: usize,
        table: &'static str,
        tag: RowTag,
    },
    /// A numeric line has the wrong number of values.
    WrongCount {
        line: usize,
        table: &'static str,
        expected: usize,
        found: usize,
    },
    /// A value is not a finite decimal number.
    NotANumber {
        line: usize,
        table: &'static str,
        token: String,
    },
    /// The file ends inside a table.
    UnexpectedEnd { table: &'static str },
    /// A table the engine needs is not in the file.
    MissingTable(&'static str),
}

impl ParseError {
    /// Localized description of the problem.
    pub fn message(&self, lang: Language) -> String {
        match self {
            ParseError::BadHeader => lf(
                lang,
                format!("Not a TunerLAB map file: the first line must be “{HEADER}”."),
                format!("To nie jest plik map TunerLAB: pierwszy wiersz musi brzmieć „{HEADER}”."),
            ),
            ParseError::UnsupportedVersion(v) => lf(
                lang,
                format!("Unsupported map file version {v} (this program reads version 1)."),
                format!("Nieobsługiwana wersja pliku map: {v} (ten program odczytuje wersję 1)."),
            ),
            ParseError::BadEngine { line } => lf(
                lang,
                format!("Line {line}: expected “engine na” or “engine turbo”."),
                format!("Wiersz {line}: oczekiwano „engine na” lub „engine turbo”."),
            ),
            ParseError::WrongEngine { file_turbo: true } => l(
                lang,
                "This file holds maps for the turbo engine, but the current engine is naturally aspirated. Switch the engine in the top bar first.",
                "Ten plik zawiera mapy silnika turbo, a bieżący silnik jest wolnossący. Najpierw przełącz silnik na górnym pasku.",
            )
            .to_owned(),
            ParseError::WrongEngine { file_turbo: false } => l(
                lang,
                "This file holds maps for the naturally aspirated engine, but the current engine is turbocharged. Switch the engine in the top bar first.",
                "Ten plik zawiera mapy silnika wolnossącego, a bieżący silnik ma turbodoładowanie. Najpierw przełącz silnik na górnym pasku.",
            )
            .to_owned(),
            ParseError::BadTableHeader { line } => lf(
                lang,
                format!("Line {line}: expected a table header “table <name> <columns> <rows>”."),
                format!("Wiersz {line}: oczekiwano nagłówka tablicy „table <nazwa> <kolumny> <wiersze>”."),
            ),
            ParseError::UnknownTable { line, name } => lf(
                lang,
                format!("Line {line}: table “{name}” is unknown or not used by this engine."),
                format!("Wiersz {line}: tablica „{name}” jest nieznana lub nieużywana przez ten silnik."),
            ),
            ParseError::DuplicateTable { line, table } => lf(
                lang,
                format!("Line {line}: table “{table}” appears twice."),
                format!("Wiersz {line}: tablica „{table}” występuje dwukrotnie."),
            ),
            ParseError::WrongSize {
                line,
                table,
                expected: (ex, ey),
                found: (fx, fy),
            } => lf(
                lang,
                format!("Line {line}: table “{table}” must be {ex} × {ey} (columns × rows), the file says {fx} × {fy}."),
                format!("Wiersz {line}: tablica „{table}” musi mieć wymiar {ex} × {ey} (kolumny × wiersze), a w pliku jest {fx} × {fy}."),
            ),
            ParseError::ExpectedRow { line, table, tag } => lf(
                lang,
                format!("Line {line}, table “{table}”: expected {}.", tag.describe(lang)),
                format!("Wiersz {line}, tablica „{table}”: oczekiwano {}.", tag.describe(lang)),
            ),
            ParseError::WrongCount {
                line,
                table,
                expected,
                found,
            } => lf(
                lang,
                format!("Line {line}, table “{table}”: wrong number of values – expected {expected}, found {found}."),
                format!("Wiersz {line}, tablica „{table}”: zła liczba wartości – oczekiwano {expected}, znaleziono {found}."),
            ),
            ParseError::NotANumber { line, table, token } => {
                let comma = token.contains(',');
                lf(
                    lang,
                    format!(
                        "Line {line}, table “{table}”: “{token}” is not a valid number{}.",
                        if comma { " (use a dot as the decimal separator)" } else { "" }
                    ),
                    format!(
                        "Wiersz {line}, tablica „{table}”: „{token}” nie jest poprawną liczbą{}.",
                        if comma { " (separatorem dziesiętnym jest kropka)" } else { "" }
                    ),
                )
            }
            ParseError::UnexpectedEnd { table } => lf(
                lang,
                format!("The file ends in the middle of table “{table}”."),
                format!("Plik kończy się w środku tablicy „{table}”."),
            ),
            ParseError::MissingTable(table) => lf(
                lang,
                format!("Table “{table}” is missing from the file."),
                format!("W pliku brakuje tablicy „{table}”."),
            ),
        }
    }
}

/// `s` shortened for quoting in a message.
fn quote(s: &str) -> String {
    if s.chars().count() > MAX_QUOTE_CHARS {
        let head: String = s.chars().take(MAX_QUOTE_CHARS).collect();
        format!("{head}…")
    } else {
        s.to_owned()
    }
}

/// Parses map-file text for an engine that is (`turbo`) or is not turbocharged.
pub fn parse(text: &str, turbo: bool) -> Result<MapSet, ParseError> {
    // Editors on Windows may prepend a byte-order mark.
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    let mut lines = text
        .lines()
        .enumerate()
        .map(|(i, l)| (i + 1, l.trim()))
        .filter(|(_, l)| !l.is_empty() && !l.starts_with('#'));

    let (header_line, header) = lines.next().ok_or(ParseError::BadHeader)?;
    match header.split_whitespace().collect::<Vec<_>>().as_slice() {
        ["TunerLAB", "maps", "1"] => {}
        ["TunerLAB", "maps", v] => return Err(ParseError::UnsupportedVersion(quote(v))),
        _ => return Err(ParseError::BadHeader),
    }
    let (line, engine) = lines.next().ok_or(ParseError::BadEngine {
        line: header_line + 1,
    })?;
    let file_turbo = match engine.split_whitespace().collect::<Vec<_>>().as_slice() {
        ["engine", "na"] => false,
        ["engine", "turbo"] => true,
        _ => return Err(ParseError::BadEngine { line }),
    };
    if file_turbo != turbo {
        return Err(ParseError::WrongEngine { file_turbo });
    }

    let mut ve = None;
    let mut ignition = None;
    let mut lambda_target = None;
    let mut boost_target = None;
    while let Some((line, text)) = lines.next() {
        let words: Vec<&str> = text.split_whitespace().collect();
        let &["table", name, cols, rows] = words.as_slice() else {
            return Err(ParseError::BadTableHeader { line });
        };
        let (Ok(cols), Ok(rows)) = (cols.parse::<usize>(), rows.parse::<usize>()) else {
            return Err(ParseError::BadTableHeader { line });
        };
        let at = (line, (cols, rows));
        match name {
            VE => read_table(&mut ve, &mut lines, VE, at)?,
            IGNITION => read_table(&mut ignition, &mut lines, IGNITION, at)?,
            LAMBDA => read_table(&mut lambda_target, &mut lines, LAMBDA, at)?,
            BOOST if turbo => read_table(&mut boost_target, &mut lines, BOOST, at)?,
            _ => {
                return Err(ParseError::UnknownTable {
                    line,
                    name: quote(name),
                })
            }
        }
    }
    Ok(MapSet {
        ve: ve.ok_or(ParseError::MissingTable(VE))?,
        ignition: ignition.ok_or(ParseError::MissingTable(IGNITION))?,
        lambda_target: lambda_target.ok_or(ParseError::MissingTable(LAMBDA))?,
        boost_target: if turbo {
            Some(boost_target.ok_or(ParseError::MissingTable(BOOST))?)
        } else {
            None
        },
    })
}

/// Reads the body of `table` (declared at `at = (line, (columns, rows))`) into `slot`.
fn read_table<'a, const X: usize, const Y: usize>(
    slot: &mut Option<Table2D<X, Y>>,
    lines: &mut impl Iterator<Item = (usize, &'a str)>,
    table: &'static str,
    (line, size): (usize, (usize, usize)),
) -> Result<(), ParseError> {
    if slot.is_some() {
        return Err(ParseError::DuplicateTable { line, table });
    }
    if size != (X, Y) {
        return Err(ParseError::WrongSize {
            line,
            table,
            expected: (X, Y),
            found: size,
        });
    }
    let mut t = Table2D {
        x_axis: [0.0; X],
        y_axis: [0.0; Y],
        values: [[0.0; X]; Y],
    };
    read_row(lines, table, RowTag::X, &mut t.x_axis)?;
    read_row(lines, table, RowTag::Y, &mut t.y_axis)?;
    for row in t.values.iter_mut() {
        read_row(lines, table, RowTag::V, row)?;
    }
    *slot = Some(t);
    Ok(())
}

fn read_row<'a>(
    lines: &mut impl Iterator<Item = (usize, &'a str)>,
    table: &'static str,
    tag: RowTag,
    out: &mut [f32],
) -> Result<(), ParseError> {
    let (line, text) = lines.next().ok_or(ParseError::UnexpectedEnd { table })?;
    let mut words = text.split_whitespace();
    if words.next() != Some(tag.word()) {
        return Err(ParseError::ExpectedRow { line, table, tag });
    }
    let numbers: Vec<&str> = words.collect();
    if numbers.len() != out.len() {
        return Err(ParseError::WrongCount {
            line,
            table,
            expected: out.len(),
            found: numbers.len(),
        });
    }
    for (slot, word) in out.iter_mut().zip(numbers) {
        *slot = word
            .parse::<f32>()
            .ok()
            .filter(|v| v.is_finite())
            .ok_or_else(|| ParseError::NotANumber {
                line,
                table,
                token: quote(word),
            })?;
    }
    Ok(())
}

/// File name (without extension) made from what the user typed: letters, digits, `-` and
/// `_` are kept, whitespace becomes `_`, anything else is dropped. `None` when nothing is
/// left.
pub fn sanitize_name(raw: &str) -> Option<String> {
    let raw = raw.trim();
    // A typed extension is not part of the name.
    let suffix = ".tlmap";
    let raw = match raw.split_at_checked(raw.len().saturating_sub(suffix.len())) {
        Some((head, tail)) if tail.eq_ignore_ascii_case(suffix) => head,
        _ => raw,
    };
    let mut name: String = raw
        .chars()
        .filter_map(|c| {
            if c.is_alphanumeric() || c == '-' || c == '_' {
                Some(c)
            } else if c.is_whitespace() {
                Some('_')
            } else {
                None
            }
        })
        .take(MAX_NAME_CHARS)
        .collect();
    if name.is_empty() {
        return None;
    }
    if reserved_on_windows(&name) {
        name.push('_');
    }
    Some(name)
}

/// Device names Windows refuses as file names, whatever the extension.
fn reserved_on_windows(name: &str) -> bool {
    let upper = name.to_ascii_uppercase();
    let numbered = |prefix: &str| {
        upper
            .strip_prefix(prefix)
            .is_some_and(|rest| rest.len() == 1 && rest.as_bytes()[0].is_ascii_digit())
    };
    matches!(upper.as_str(), "CON" | "PRN" | "AUX" | "NUL") || numbered("COM") || numbered("LPT")
}

/// Folder holding the map files: `mapy` next to the executable, or in the current
/// directory when the executable's location is unknown.
pub fn maps_dir() -> PathBuf {
    std::env::current_exe()
        .ok()
        .and_then(|exe| exe.parent().map(Path::to_path_buf))
        .or_else(|| std::env::current_dir().ok())
        .unwrap_or_default()
        .join(DIR_NAME)
}

fn file_path(dir: &Path, name: &str) -> PathBuf {
    dir.join(format!("{name}.{EXTENSION}"))
}

/// File operation that failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    Save,
    Load,
    List,
}

/// Why saving, listing or loading map files failed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FileError {
    /// Nothing usable was typed as the file name.
    EmptyName,
    /// The file already exists and overwriting was not confirmed.
    Exists { file: String },
    /// The operating system refused the operation.
    Io {
        action: Action,
        kind: io::ErrorKind,
        detail: String,
    },
    /// The file is not a valid map file for this engine.
    Parse { file: String, error: ParseError },
    /// The maps parsed but the ECU's calibration check refused them.
    Rejected { file: String, error: SimError },
}

fn io_error(action: Action, e: &io::Error) -> FileError {
    FileError::Io {
        action,
        kind: e.kind(),
        detail: e.to_string(),
    }
}

impl FileError {
    /// Localized description of the problem.
    pub fn message(&self, lang: Language) -> String {
        match self {
            FileError::EmptyName => l(
                lang,
                "Enter a file name (letters, digits, - and _).",
                "Podaj nazwę pliku (litery, cyfry, - i _).",
            )
            .to_owned(),
            FileError::Exists { file } => lf(
                lang,
                format!("{file}.{EXTENSION} already exists – click “Overwrite” to replace it."),
                format!(
                    "Plik {file}.{EXTENSION} już istnieje – kliknij „Nadpisz”, aby go zastąpić."
                ),
            ),
            FileError::Io {
                action,
                kind,
                detail,
            } => {
                let what = match action {
                    Action::Save => l(
                        lang,
                        "Could not save the file",
                        "Nie udało się zapisać pliku",
                    ),
                    Action::Load => l(
                        lang,
                        "Could not read the file",
                        "Nie udało się odczytać pliku",
                    ),
                    Action::List => l(
                        lang,
                        "Could not read the map folder",
                        "Nie udało się odczytać folderu map",
                    ),
                };
                let why = match kind {
                    io::ErrorKind::NotFound => l(lang, "not found", "nie znaleziono").to_owned(),
                    io::ErrorKind::PermissionDenied => {
                        l(lang, "access denied", "brak dostępu").to_owned()
                    }
                    io::ErrorKind::InvalidData => l(
                        lang,
                        "the file is not UTF-8 text",
                        "plik nie jest tekstem UTF-8",
                    )
                    .to_owned(),
                    io::ErrorKind::StorageFull => {
                        l(lang, "the disk is full", "dysk jest pełny").to_owned()
                    }
                    io::ErrorKind::ReadOnlyFilesystem => {
                        l(lang, "the disk is read-only", "dysk jest tylko do odczytu").to_owned()
                    }
                    _ => format!(
                        "{} ({detail})",
                        l(lang, "input/output error", "błąd wejścia/wyjścia")
                    ),
                };
                format!("{what}: {why}.")
            }
            FileError::Parse { file, error } => format!("{file}: {}", error.message(lang)),
            FileError::Rejected { file, error } => {
                let why = match error {
                    SimError::Calibration(CalibrationError::InvalidAxis(name)) => lf(
                        lang,
                        format!(
                            "the axes of table “{name}” must be finite and strictly increasing"
                        ),
                        format!("osie tablicy „{name}” muszą być skończone i ściśle rosnące"),
                    ),
                    SimError::Calibration(CalibrationError::InvalidValue(name)) => lf(
                        lang,
                        format!("table “{name}” contains a value outside the allowed range"),
                        format!("tablica „{name}” zawiera wartość spoza dopuszczalnego zakresu"),
                    ),
                    _ => l(
                        lang,
                        "the maps do not fit this engine",
                        "mapy nie pasują do tego silnika",
                    )
                    .to_owned(),
                };
                lf(
                    lang,
                    format!("{file}: calibration rejected – {why}. Nothing was changed."),
                    format!("{file}: kalibracja odrzucona – {why}. Nic nie zostało zmienione."),
                )
            }
        }
    }
}

/// Names (without extension) of the map files in `dir`, sorted; empty if `dir` does not
/// exist yet.
pub fn list_maps(dir: &Path) -> Result<Vec<String>, FileError> {
    let entries = match fs::read_dir(dir) {
        Ok(entries) => entries,
        Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(e) => return Err(io_error(Action::List, &e)),
    };
    let mut names: Vec<String> = entries
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|p| p.extension() == Some(OsStr::new(EXTENSION)) && p.is_file())
        .filter_map(|p| p.file_stem().and_then(OsStr::to_str).map(str::to_owned))
        .collect();
    names.sort_by_key(|n| n.to_lowercase());
    Ok(names)
}

/// Writes `set` to `dir/name.tlmap` (`name` already sanitized), creating `dir` if needed.
/// An existing file is replaced only with `overwrite`, otherwise [`FileError::Exists`].
pub fn save_maps(
    dir: &Path,
    name: &str,
    set: &MapSet,
    overwrite: bool,
) -> Result<PathBuf, FileError> {
    fs::create_dir_all(dir).map_err(|e| io_error(Action::Save, &e))?;
    let path = file_path(dir, name);
    let mut options = fs::OpenOptions::new();
    options.write(true);
    if overwrite {
        options.create(true).truncate(true);
    } else {
        options.create_new(true);
    }
    let mut file = match options.open(&path) {
        Ok(file) => file,
        Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {
            return Err(FileError::Exists {
                file: name.to_owned(),
            })
        }
        Err(e) => return Err(io_error(Action::Save, &e)),
    };
    file.write_all(serialize(set).as_bytes())
        .map_err(|e| io_error(Action::Save, &e))?;
    Ok(path)
}

/// Reads and parses `dir/name.tlmap` for an engine that is (`turbo`) or is not
/// turbocharged.
pub fn load_maps(dir: &Path, name: &str, turbo: bool) -> Result<MapSet, FileError> {
    let text = fs::read_to_string(file_path(dir, name)).map_err(|e| io_error(Action::Load, &e))?;
    parse(&text, turbo).map_err(|error| FileError::Parse {
        file: name.to_owned(),
        error,
    })
}

/// Loads `dir/name.tlmap` into the ECU, all tables at once: if the calibration check
/// refuses them, nothing changes.
pub fn load_into(sim: &mut EngineSim, dir: &Path, name: &str) -> Result<(), FileError> {
    let set = load_maps(dir, name, sim.calibration().engine.turbocharged)?;
    sim.update_calibration(|cal| set.apply_to(cal))
        .map_err(|error| FileError::Rejected {
            file: name.to_owned(),
            error,
        })
}

/// Outcome of the last file action, shown under the controls.
enum Status {
    Saved(PathBuf),
    Loaded(String),
    Failed(FileError),
}

impl Status {
    fn text(&self, lang: Language) -> (String, Color32) {
        match self {
            Status::Saved(path) => (
                format!("{} {}", l(lang, "Saved:", "Zapisano:"), path.display()),
                pal().ok,
            ),
            Status::Loaded(name) => (
                lf(
                    lang,
                    format!("Loaded “{name}” into the ECU."),
                    format!("Wczytano „{name}” do ECU."),
                ),
                pal().ok,
            ),
            Status::Failed(e @ FileError::Exists { .. }) => (e.message(lang), pal().warn),
            Status::Failed(e) => (e.message(lang), pal().danger),
        }
    }
}

/// "Map files" section of the map editor: save, list and load map files.
pub struct MapFiles {
    dir: PathBuf,
    /// File name being typed.
    name: String,
    /// Map files found in `dir`.
    files: Vec<String>,
    /// File picked for loading.
    chosen: Option<String>,
    /// `files` has been read from the folder (first done when the section is opened).
    scanned: bool,
    /// Name whose existing file the next save click replaces.
    confirm: Option<String>,
    status: Option<Status>,
}

impl Default for MapFiles {
    fn default() -> Self {
        Self {
            dir: maps_dir(),
            name: String::new(),
            files: Vec::new(),
            chosen: None,
            scanned: false,
            confirm: None,
            status: None,
        }
    }
}

impl MapFiles {
    /// Draws the collapsible section; returns `true` when a file was loaded into the ECU.
    pub fn show(&mut self, ui: &mut egui::Ui, sim: &mut EngineSim, lang: Language) -> bool {
        let mut loaded = false;
        egui::CollapsingHeader::new(l(lang, "Map files", "Pliki map"))
            .id_salt("map_files")
            .default_open(false)
            .show(ui, |ui| loaded = self.contents(ui, sim, lang));
        loaded
    }

    fn contents(&mut self, ui: &mut egui::Ui, sim: &mut EngineSim, lang: Language) -> bool {
        if !self.scanned {
            self.rescan();
        }
        let mut loaded = false;
        ui.horizontal_wrapped(|ui| {
            ui.label(l(lang, "Name", "Nazwa"));
            let edit = ui.add(
                egui::TextEdit::singleline(&mut self.name)
                    .hint_text(l(lang, "map name", "nazwa mapy"))
                    .char_limit(MAX_NAME_CHARS)
                    .desired_width(150.0),
            );
            let target = sanitize_name(&self.name);
            let confirmed = target.is_some() && self.confirm == target;
            let button = if confirmed {
                egui::Button::new(RichText::new(l(lang, "Overwrite", "Nadpisz")).color(pal().warn))
            } else {
                egui::Button::new(l(lang, "Save", "Zapisz"))
            };
            let enter = edit.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter));
            if ui.add(button).clicked() || enter {
                self.save(sim, target, confirmed);
            }
        });
        ui.horizontal_wrapped(|ui| {
            ui.label(l(lang, "File", "Plik"));
            let shown = match &self.chosen {
                Some(name) => name.as_str(),
                None if self.files.is_empty() => l(lang, "no files yet", "brak plików"),
                None => l(lang, "choose a file…", "wybierz plik…"),
            };
            egui::ComboBox::from_id_salt("map_file_list")
                .selected_text(shown)
                .width(170.0)
                .show_ui(ui, |ui| {
                    for name in &self.files {
                        ui.selectable_value(&mut self.chosen, Some(name.clone()), name.as_str());
                    }
                });
            let load = egui::Button::new(l(lang, "Load", "Wczytaj"));
            if ui.add_enabled(self.chosen.is_some(), load).clicked() {
                loaded = self.load(sim);
            }
            if ui.button(l(lang, "Refresh", "Odśwież")).clicked() {
                self.rescan();
            }
        });
        ui.label(
            RichText::new(format!(
                "{} {}",
                l(lang, "Folder:", "Folder:"),
                self.dir.display()
            ))
            .color(pal().dim)
            .size(12.0),
        );
        ui.label(
            RichText::new(l(
                lang,
                "Loading replaces the VE, ignition and target-lambda maps (and the boost target on a turbo engine) in the ECU.",
                "Wczytanie zastępuje w ECU mapy VE, zapłonu i docelowej lambdy (a w silniku turbo także docelowe doładowanie).",
            ))
            .color(pal().dim)
            .size(12.0),
        );
        if let Some(status) = &self.status {
            let (text, color) = status.text(lang);
            ui.label(RichText::new(text).color(color));
        }
        loaded
    }

    /// Reads the folder again, keeping the picked file if it is still there.
    fn rescan(&mut self) {
        self.scanned = true;
        match list_maps(&self.dir) {
            Ok(files) => self.files = files,
            Err(e) => {
                self.files.clear();
                self.status = Some(Status::Failed(e));
            }
        }
        if self
            .chosen
            .as_ref()
            .is_some_and(|c| !self.files.contains(c))
        {
            self.chosen = None;
        }
    }

    fn save(&mut self, sim: &EngineSim, target: Option<String>, overwrite: bool) {
        let Some(name) = target else {
            self.status = Some(Status::Failed(FileError::EmptyName));
            return;
        };
        let set = MapSet::from_calibration(sim.calibration());
        match save_maps(&self.dir, &name, &set, overwrite) {
            Ok(path) => {
                self.confirm = None;
                self.rescan();
                self.name = name.clone();
                self.chosen = Some(name);
                self.status = Some(Status::Saved(path));
            }
            Err(e @ FileError::Exists { .. }) => {
                self.confirm = Some(name);
                self.status = Some(Status::Failed(e));
            }
            Err(e) => {
                self.confirm = None;
                self.status = Some(Status::Failed(e));
            }
        }
    }

    fn load(&mut self, sim: &mut EngineSim) -> bool {
        let Some(name) = self.chosen.clone() else {
            return false;
        };
        self.confirm = None;
        match load_into(sim, &self.dir, &name) {
            Ok(()) => {
                // Saving again goes to the same file (after the overwrite confirmation).
                self.name = name.clone();
                self.status = Some(Status::Loaded(name));
                true
            }
            Err(e) => {
                self.status = Some(Status::Failed(e));
                false
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tunerlab_core::EngineSpec;

    fn sim(turbo: bool) -> EngineSim {
        let spec = if turbo {
            EngineSpec::turbocharged_2l()
        } else {
            EngineSpec::naturally_aspirated_2l()
        };
        EngineSim::with_base_calibration(spec, 1).unwrap()
    }

    fn base_text(turbo: bool) -> String {
        serialize(&MapSet::from_calibration(sim(turbo).calibration()))
    }

    /// Bit pattern of every number in the set, for exact comparison.
    fn bits(set: &MapSet) -> Vec<u32> {
        let mut out = Vec::new();
        let mut table = |x: &[f32], y: &[f32], v: &mut dyn Iterator<Item = f32>| {
            out.extend(x.iter().chain(y).map(|f| f.to_bits()));
            out.extend(v.map(f32::to_bits));
        };
        for t in [&set.ve, &set.ignition, &set.lambda_target] {
            table(
                &t.x_axis,
                &t.y_axis,
                &mut t.values.iter().flatten().copied(),
            );
        }
        if let Some(b) = &set.boost_target {
            table(
                &b.x_axis,
                &b.y_axis,
                &mut b.values.iter().flatten().copied(),
            );
        }
        out
    }

    /// Unique scratch folder under the system temp directory.
    fn scratch_dir(tag: &str) -> PathBuf {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_nanos());
        std::env::temp_dir().join(format!(
            "tunerlab-mapfile-{tag}-{}-{nanos}",
            std::process::id()
        ))
    }

    #[test]
    fn presets_round_trip_exactly() {
        for turbo in [false, true] {
            let sim = sim(turbo);
            let set = MapSet::from_calibration(sim.calibration());
            assert_eq!(set.turbo(), turbo);
            let text = serialize(&set);
            assert!(text.starts_with("TunerLAB maps 1\n"));
            assert!(text.contains(if turbo { "engine turbo" } else { "engine na" }));
            assert_eq!(text.contains("table boost.target_kpa 16 8"), turbo);
            let back = parse(&text, turbo).unwrap();
            assert_eq!(bits(&back), bits(&set));
            let mut cal = *sim.calibration();
            back.apply_to(&mut cal);
            assert_eq!(cal, *sim.calibration());
        }
    }

    #[test]
    fn edited_values_survive_the_round_trip() {
        let mut cal = *sim(true).calibration();
        cal.ve.values[3][4] = 1.0 / 3.0;
        cal.ve.values[15][15] = 0.1 + f32::EPSILON;
        cal.ignition.values[0][0] = -12.0 - 1.0 / 3.0;
        cal.ignition.values[7][2] = 1.0e-7;
        cal.lambda_target.values[10][11] = 0.8 + 1.0 / 13.0;
        cal.boost.target_kpa.values[7][15] = 212.125;
        cal.boost.target_kpa.x_axis[0] = 500.0_f32.next_down();
        let set = MapSet::from_calibration(&cal);
        let back = parse(&serialize(&set), true).unwrap();
        assert_eq!(bits(&back), bits(&set));
        assert_eq!(back.ve.values[3][4], 1.0 / 3.0);
        assert_eq!(back.ignition.values[0][0], -12.0 - 1.0 / 3.0);
    }

    #[test]
    fn comments_blank_lines_crlf_and_bom_are_accepted() {
        let text = base_text(false).replace('\n', "\r\n");
        let text = format!(
            "\u{feff}# my map\r\n\r\n{}\r\n# end\r\n",
            text.replacen("engine na", "engine   na", 1)
        );
        let set = parse(&text, false).unwrap();
        assert_eq!(set, MapSet::from_calibration(sim(false).calibration()));
    }

    /// `text` with 1-based line `n` replaced by `with`.
    fn replace_line(text: &str, n: usize, with: &str) -> String {
        text.lines()
            .enumerate()
            .map(|(i, l)| if i + 1 == n { with } else { l })
            .collect::<Vec<_>>()
            .join("\n")
    }

    #[test]
    fn malformed_files_are_rejected_with_the_right_error() {
        use ParseError as E;
        let na = base_text(false);
        // Layout: 1 header, 2 engine, 3 blank, 4 `table ve`, 5 x, 6 y, 7–22 rows, 23 blank,
        // 24 `table ignition`, …
        assert_eq!(na.lines().nth(3), Some("table ve 16 16"));
        assert_eq!(na.lines().nth(23), Some("table ignition 16 16"));
        let cases: Vec<(String, ParseError)> = vec![
            (String::new(), E::BadHeader),
            ("# only a comment\n".into(), E::BadHeader),
            (replace_line(&na, 1, "TunerLAB mapy 1"), E::BadHeader),
            (
                replace_line(&na, 1, "TunerLAB maps 2"),
                E::UnsupportedVersion("2".into()),
            ),
            ("TunerLAB maps 1\n".into(), E::BadEngine { line: 2 }),
            (
                replace_line(&na, 2, "engine diesel"),
                E::BadEngine { line: 2 },
            ),
            (
                replace_line(&na, 4, "table ve 16"),
                E::BadTableHeader { line: 4 },
            ),
            (
                replace_line(&na, 4, "table ve sixteen 16"),
                E::BadTableHeader { line: 4 },
            ),
            (
                replace_line(&na, 5, "garbage"),
                E::ExpectedRow {
                    line: 5,
                    table: VE,
                    tag: RowTag::X,
                },
            ),
            (
                replace_line(&na, 4, "table maf 16 16"),
                E::UnknownTable {
                    line: 4,
                    name: "maf".into(),
                },
            ),
            (
                replace_line(&na, 4, "table ve 15 16"),
                E::WrongSize {
                    line: 4,
                    table: VE,
                    expected: (16, 16),
                    found: (15, 16),
                },
            ),
            (
                replace_line(&na, 6, "x 1 2 3"),
                E::ExpectedRow {
                    line: 6,
                    table: VE,
                    tag: RowTag::Y,
                },
            ),
            (
                replace_line(&na, 9, "v 1 2 3"),
                E::WrongCount {
                    line: 9,
                    table: VE,
                    expected: 16,
                    found: 3,
                },
            ),
            (
                na.replacen("table ignition 16 16", "table ve 16 16", 1),
                E::DuplicateTable {
                    line: 24,
                    table: VE,
                },
            ),
            (
                na.lines().take(15).collect::<Vec<_>>().join("\n"),
                E::UnexpectedEnd { table: VE },
            ),
            (
                na.lines().take(43).collect::<Vec<_>>().join("\n"),
                E::MissingTable(LAMBDA),
            ),
        ];
        for (text, expected) in cases {
            assert_eq!(parse(&text, false), Err(expected.clone()), "{expected:?}");
        }
        // Non-numbers, including values that parse but are not finite.
        for bad in ["abc", "0,85", "NaN", "inf", "1e60", "--1"] {
            let row = format!("v {bad}{}", " 0.5".repeat(15));
            assert_eq!(
                parse(&replace_line(&na, 12, &row), false),
                Err(E::NotANumber {
                    line: 12,
                    table: VE,
                    token: bad.into()
                }),
            );
        }
    }

    #[test]
    fn a_file_for_the_other_engine_is_refused() {
        let na = base_text(false);
        let turbo = base_text(true);
        assert_eq!(
            parse(&turbo, false),
            Err(ParseError::WrongEngine { file_turbo: true })
        );
        assert_eq!(
            parse(&na, true),
            Err(ParseError::WrongEngine { file_turbo: false })
        );
        // A boost table forced into a naturally aspirated file is not accepted either.
        let boost_start = turbo.find("\ntable boost.target_kpa").unwrap();
        let smuggled = format!("{na}{}", &turbo[boost_start..]);
        assert!(matches!(
            parse(&smuggled, false),
            Err(ParseError::UnknownTable { name, .. }) if name == BOOST
        ));
        // A turbo file without its boost table is incomplete.
        assert_eq!(
            parse(&turbo[..boost_start], true),
            Err(ParseError::MissingTable(BOOST))
        );
    }

    #[test]
    fn every_message_is_translated() {
        let errors = [
            FileError::EmptyName,
            FileError::Exists { file: "a".into() },
            FileError::Io {
                action: Action::Save,
                kind: io::ErrorKind::PermissionDenied,
                detail: "x".into(),
            },
            FileError::Io {
                action: Action::List,
                kind: io::ErrorKind::Other,
                detail: "x".into(),
            },
            FileError::Parse {
                file: "a".into(),
                error: ParseError::WrongEngine { file_turbo: true },
            },
            FileError::Parse {
                file: "a".into(),
                error: ParseError::NotANumber {
                    line: 3,
                    table: VE,
                    token: "1,5".into(),
                },
            },
            FileError::Rejected {
                file: "a".into(),
                error: SimError::Calibration(CalibrationError::InvalidAxis(VE)),
            },
            FileError::Rejected {
                file: "a".into(),
                error: SimError::CylinderMismatch,
            },
        ];
        for e in errors {
            let (en, pl) = (e.message(Language::En), e.message(Language::Pl));
            assert!(!en.is_empty() && !pl.is_empty());
            assert_ne!(en, pl, "{e:?}");
        }
        let comma = ParseError::NotANumber {
            line: 3,
            table: VE,
            token: "1,5".into(),
        };
        assert!(comma.message(Language::Pl).contains("kropka"));
    }

    #[test]
    fn names_are_sanitized() {
        assert_eq!(sanitize_name("  my map 2 ").as_deref(), Some("my_map_2"));
        assert_eq!(sanitize_name("../a/b\\c:d*?").as_deref(), Some("abcd"));
        assert_eq!(
            sanitize_name("Mapa-ŁÓDŹ_ąę").as_deref(),
            Some("Mapa-ŁÓDŹ_ąę")
        );
        assert_eq!(sanitize_name("stage1.tlmap").as_deref(), Some("stage1"));
        assert_eq!(sanitize_name("stage1.TLMAP").as_deref(), Some("stage1"));
        assert_eq!(sanitize_name("v1.2").as_deref(), Some("v12"));
        assert_eq!(sanitize_name("con").as_deref(), Some("con_"));
        assert_eq!(sanitize_name("LPT1").as_deref(), Some("LPT1_"));
        assert_eq!(sanitize_name("console").as_deref(), Some("console"));
        assert_eq!(sanitize_name(""), None);
        assert_eq!(sanitize_name("   "), None);
        assert_eq!(sanitize_name("!!!/..."), None);
        assert_eq!(sanitize_name(".tlmap"), None);
        assert_eq!(
            sanitize_name(&"x".repeat(200)).map(|n| n.len()),
            Some(MAX_NAME_CHARS)
        );
    }

    #[test]
    fn save_list_and_load_in_a_folder() {
        let dir = scratch_dir("cycle");
        // A folder that does not exist yet simply has no maps.
        assert_eq!(list_maps(&dir), Ok(Vec::new()));

        let mut sim = sim(false);
        let original = *sim.calibration();
        let path = save_maps(&dir, "base", &MapSet::from_calibration(&original), false).unwrap();
        assert_eq!(path, dir.join("base.tlmap"));

        // A richer map under another name, plus files that are not maps.
        let mut tuned = original;
        tuned.ignition.values[5][5] += 2.5;
        tuned.ve.values[9][9] = 0.913;
        save_maps(&dir, "Stage_1", &MapSet::from_calibration(&tuned), false).unwrap();
        fs::write(dir.join("notes.txt"), "not a map").unwrap();
        fs::create_dir_all(dir.join("folder.tlmap")).unwrap();
        assert_eq!(
            list_maps(&dir),
            Ok(vec!["base".to_owned(), "Stage_1".to_owned()])
        );

        // Existing files are replaced only on request.
        assert_eq!(
            save_maps(&dir, "base", &MapSet::from_calibration(&tuned), false),
            Err(FileError::Exists {
                file: "base".into()
            })
        );
        assert_eq!(
            load_maps(&dir, "base", false),
            Ok(MapSet::from_calibration(&original))
        );

        load_into(&mut sim, &dir, "Stage_1").unwrap();
        assert_eq!(*sim.calibration(), tuned);
        load_into(&mut sim, &dir, "base").unwrap();
        assert_eq!(*sim.calibration(), original);

        save_maps(&dir, "base", &MapSet::from_calibration(&tuned), true).unwrap();
        assert_eq!(
            load_maps(&dir, "base", false),
            Ok(MapSet::from_calibration(&tuned))
        );

        // Missing file, wrong engine and a map the ECU refuses: nothing changes.
        assert!(matches!(
            load_maps(&dir, "nope", false),
            Err(FileError::Io {
                action: Action::Load,
                kind: io::ErrorKind::NotFound,
                ..
            })
        ));
        let mut turbo = self::sim(true);
        let before = *turbo.calibration();
        assert!(matches!(
            load_into(&mut turbo, &dir, "base"),
            Err(FileError::Parse {
                error: ParseError::WrongEngine { file_turbo: false },
                ..
            })
        ));
        assert_eq!(*turbo.calibration(), before);

        let mut broken = MapSet::from_calibration(&original);
        broken.lambda_target.values[0][0] = 3.0;
        broken.ve.x_axis[1] = broken.ve.x_axis[0];
        save_maps(&dir, "broken", &broken, false).unwrap();
        let before = *sim.calibration();
        assert_eq!(
            load_into(&mut sim, &dir, "broken"),
            Err(FileError::Rejected {
                file: "broken".into(),
                error: SimError::Calibration(CalibrationError::InvalidAxis(VE)),
            })
        );
        assert_eq!(*sim.calibration(), before);

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn overwriting_takes_a_second_click_and_the_section_draws_headless() {
        let dir = scratch_dir("panel");
        let mut files = MapFiles {
            dir: dir.clone(),
            ..MapFiles::default()
        };
        let mut sim = sim(false);
        files.save(&sim, None, false);
        assert!(matches!(
            files.status,
            Some(Status::Failed(FileError::EmptyName))
        ));

        files.name = " my map ".into();
        let target = sanitize_name(&files.name);
        files.save(&sim, target.clone(), false);
        assert!(matches!(files.status, Some(Status::Saved(_))));
        assert_eq!(files.name, "my_map");
        assert_eq!(files.chosen.as_deref(), Some("my_map"));
        assert_eq!(files.files, vec!["my_map".to_owned()]);

        // The first click on an existing name only asks; the second one writes.
        sim.update_calibration(|c| c.ignition.values[0][0] += 1.5)
            .unwrap();
        files.save(&sim, target.clone(), false);
        assert_eq!(files.confirm, target);
        assert!(matches!(
            files.status,
            Some(Status::Failed(FileError::Exists { .. }))
        ));
        let mut fresh = self::sim(false);
        assert!(files.load(&mut fresh));
        assert_ne!(*fresh.calibration(), *sim.calibration());
        files.save(&sim, target.clone(), true);
        assert_eq!(files.confirm, None);
        assert!(files.load(&mut fresh));
        assert_eq!(*fresh.calibration(), *sim.calibration());
        assert!(matches!(&files.status, Some(Status::Loaded(n)) if n == "my_map"));

        // One frame of the section in each language, with a file listed and picked.
        let ctx = egui::Context::default();
        for lang in [Language::En, Language::Pl] {
            ctx.run_ui(egui::RawInput::default(), |ui| {
                files.contents(ui, &mut fresh, lang);
            })
            .drop_without_applying_deltas();
        }
        fs::remove_dir_all(&dir).unwrap();
    }
}

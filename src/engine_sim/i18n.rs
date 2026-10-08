//! Localisation hooks (Polish and English from day one).
//!
//! Every user-facing status, warning, trouble code, failure and fault has a compile-time
//! translation table: exhaustive `match`es make a missing translation a build error, and
//! returning `&'static str` keeps lookups allocation-free (safe on the audio thread).
//! Phase 3 can swap [`StaticLocalizer`] for a Fluent-backed implementation of
//! [`Localizer`] without touching the simulation.

use super::dtc::DtcCode;
use super::faults::FaultId;
use super::status::{EngineCondition, FailureCause, Warning};

/// Supported UI languages.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Language {
    /// English.
    #[default]
    En,
    /// Polish.
    Pl,
}

impl Language {
    /// Every supported language.
    pub const ALL: [Language; 2] = [Language::En, Language::Pl];

    /// ISO-style two-letter code ("EN", "PL").
    pub fn code(&self) -> &'static str {
        match self {
            Self::En => "EN",
            Self::Pl => "PL",
        }
    }

    /// Parses a two-letter code, case-insensitively.
    pub fn from_code(code: &str) -> Option<Self> {
        if code.eq_ignore_ascii_case("EN") {
            Some(Self::En)
        } else if code.eq_ignore_ascii_case("PL") {
            Some(Self::Pl)
        } else {
            None
        }
    }
}

/// Any localisable message.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MessageKey {
    /// Engine condition.
    Condition(EngineCondition),
    /// Mechanical failure.
    Failure(FailureCause),
    /// Instructor warning.
    Warning(Warning),
    /// Diagnostic trouble code description.
    Dtc(DtcCode),
    /// Hidden-fault name (revealed after an Engine Autopsy).
    Fault(FaultId),
}

/// Types with a localised description.
pub trait Localize {
    /// Description in `lang`.
    fn localized(&self, lang: Language) -> &'static str;
}

/// Source of localised strings.
pub trait Localizer {
    /// Active language.
    fn language(&self) -> Language;
    /// Text for `key` in the active language.
    fn text(&self, key: MessageKey) -> &'static str {
        text(key, self.language())
    }
}

/// Built-in compile-time translation tables.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StaticLocalizer {
    /// Active language.
    pub language: Language,
}

impl Localizer for StaticLocalizer {
    fn language(&self) -> Language {
        self.language
    }
}

/// Text for `key` in `lang`.
pub fn text(key: MessageKey, lang: Language) -> &'static str {
    match key {
        MessageKey::Condition(c) => c.localized(lang),
        MessageKey::Failure(f) => f.localized(lang),
        MessageKey::Warning(w) => w.localized(lang),
        MessageKey::Dtc(d) => d.localized(lang),
        MessageKey::Fault(f) => f.localized(lang),
    }
}

impl Localize for MessageKey {
    fn localized(&self, lang: Language) -> &'static str {
        text(*self, lang)
    }
}

#[inline]
fn pick(lang: Language, en: &'static str, pl: &'static str) -> &'static str {
    match lang {
        Language::En => en,
        Language::Pl => pl,
    }
}

impl Localize for EngineCondition {
    fn localized(&self, lang: Language) -> &'static str {
        match self {
            Self::Off => pick(lang, "Engine off", "Silnik wyłączony"),
            Self::Cranking => pick(lang, "Cranking", "Rozruch"),
            Self::Running => pick(lang, "Running", "Silnik pracuje"),
            Self::Stalled => pick(
                lang,
                "Ignition on – engine stopped",
                "Zapłon włączony – silnik stoi",
            ),
            Self::Failed(_) => pick(lang, "Engine failure", "Awaria silnika"),
        }
    }
}

impl Localize for FailureCause {
    fn localized(&self, lang: Language) -> &'static str {
        match self {
            Self::ThrownRod { .. } => pick(
                lang,
                "Connecting rod broke and was thrown through the block",
                "Zerwany korbowód przebił blok silnika",
            ),
            Self::BentRod { .. } => pick(
                lang,
                "Bent connecting rod (excessive cylinder pressure)",
                "Wygięty korbowód (nadmierne ciśnienie w cylindrze)",
            ),
            Self::SpunBearing => pick(
                lang,
                "Spun bearing – engine seized from oil starvation",
                "Obrócona panewka – zatarcie silnika z braku smarowania",
            ),
            Self::HoledPiston { .. } => pick(
                lang,
                "Holed piston from detonation",
                "Przepalony tłok wskutek spalania stukowego",
            ),
            Self::MeltedPiston { .. } => pick(
                lang,
                "Melted piston crown (thermal overload)",
                "Stopione denko tłoka (przeciążenie cieplne)",
            ),
            Self::BentValve { .. } => pick(
                lang,
                "Bent valve – valve struck the piston",
                "Wygięty zawór – kolizja zaworu z tłokiem",
            ),
            Self::HeadGasket { .. } => pick(
                lang,
                "Blown head gasket",
                "Przepalona uszczelka pod głowicą",
            ),
            Self::WarpedHead => pick(
                lang,
                "Cylinder head warped by overheating",
                "Głowica odkształcona wskutek przegrzania",
            ),
            Self::TurboFailure => pick(lang, "Turbocharger failure", "Awaria turbosprężarki"),
            Self::CatalystMeltdown => pick(
                lang,
                "Catalytic converter meltdown",
                "Stopiony wkład katalizatora",
            ),
        }
    }
}

impl Localize for Warning {
    fn localized(&self, lang: Language) -> &'static str {
        match self {
            Self::Knock => pick(lang, "Knock detected", "Wykryto spalanie stukowe"),
            Self::OverRev => pick(
                lang,
                "Over-rev: above redline",
                "Przekroczone obroty maksymalne",
            ),
            Self::ValveFloat => pick(
                lang,
                "Valve float",
                "Pływanie zaworów (utrata kontroli sprężyn)",
            ),
            Self::Overheat => pick(lang, "Engine overheating", "Przegrzanie silnika"),
            Self::CoolantBoiling => pick(lang, "Coolant boiling", "Wrzenie płynu chłodzącego"),
            Self::LowOilPressure => pick(lang, "Low oil pressure", "Niskie ciśnienie oleju"),
            Self::HighOilTemp => pick(lang, "High oil temperature", "Wysoka temperatura oleju"),
            Self::LeanUnderLoad => pick(
                lang,
                "Lean mixture under load",
                "Uboga mieszanka pod obciążeniem",
            ),
            Self::RichMixture => pick(lang, "Excessively rich mixture", "Zbyt bogata mieszanka"),
            Self::HighEgt => pick(
                lang,
                "Exhaust gas temperature too high",
                "Zbyt wysoka temperatura spalin",
            ),
            Self::PistonOverheat => {
                pick(lang, "Piston crown overheating", "Przegrzanie denka tłoka")
            }
            Self::Overboost => pick(lang, "Overboost", "Przekroczone ciśnienie doładowania"),
            Self::CompressorSurge => pick(lang, "Compressor surge", "Pompowanie sprężarki"),
            Self::TurboOverspeed => pick(
                lang,
                "Turbocharger overspeed",
                "Przekroczone obroty turbosprężarki",
            ),
            Self::Misfire => pick(lang, "Misfire detected", "Wykryto wypadanie zapłonów"),
            Self::InjectorDutyHigh => pick(
                lang,
                "Injector duty cycle near limit",
                "Wypełnienie wtryskiwaczy bliskie limitu",
            ),
            Self::CatalystOverheat => pick(
                lang,
                "Catalytic converter overheating",
                "Przegrzanie katalizatora",
            ),
            Self::CheckEngine => pick(
                lang,
                "Check engine – trouble code stored",
                "Kontrolka silnika – zapisano kod usterki",
            ),
        }
    }
}

impl Localize for DtcCode {
    fn localized(&self, lang: Language) -> &'static str {
        match self {
            Self::P0016 => pick(
                lang,
                "Crankshaft/camshaft position correlation",
                "Korelacja położenia wału korbowego i wałka rozrządu",
            ),
            Self::P0087 => pick(
                lang,
                "Fuel rail pressure too low",
                "Zbyt niskie ciśnienie paliwa w listwie",
            ),
            Self::P0106 => pick(
                lang,
                "MAP sensor range/performance",
                "Czujnik MAP – zakres/wiarygodność sygnału",
            ),
            Self::P0113 => pick(
                lang,
                "Intake air temperature sensor high input",
                "Czujnik temperatury powietrza – wysoki sygnał wejściowy",
            ),
            Self::P0128 => pick(
                lang,
                "Coolant temperature below thermostat regulating temperature",
                "Temperatura płynu poniżej temperatury regulacji termostatu",
            ),
            Self::P0133 => pick(
                lang,
                "O2 sensor slow response",
                "Wolna reakcja sondy lambda",
            ),
            Self::P0171 => pick(lang, "System too lean", "Mieszanka zbyt uboga"),
            Self::P0172 => pick(lang, "System too rich", "Mieszanka zbyt bogata"),
            Self::P0217 => pick(
                lang,
                "Engine coolant over-temperature",
                "Przegrzanie płynu chłodzącego",
            ),
            Self::P0219 => pick(
                lang,
                "Engine overspeed condition",
                "Przekroczenie dopuszczalnych obrotów silnika",
            ),
            Self::P0234 => pick(
                lang,
                "Turbocharger overboost",
                "Nadmierne ciśnienie doładowania",
            ),
            Self::P0299 => pick(
                lang,
                "Turbocharger underboost",
                "Zbyt niskie ciśnienie doładowania",
            ),
            Self::P0300 => pick(
                lang,
                "Random/multiple cylinder misfire",
                "Losowe/wielokrotne wypadanie zapłonów",
            ),
            Self::P0301 => pick(
                lang,
                "Cylinder 1 misfire",
                "Wypadanie zapłonów – cylinder 1",
            ),
            Self::P0302 => pick(
                lang,
                "Cylinder 2 misfire",
                "Wypadanie zapłonów – cylinder 2",
            ),
            Self::P0303 => pick(
                lang,
                "Cylinder 3 misfire",
                "Wypadanie zapłonów – cylinder 3",
            ),
            Self::P0304 => pick(
                lang,
                "Cylinder 4 misfire",
                "Wypadanie zapłonów – cylinder 4",
            ),
            Self::P0305 => pick(
                lang,
                "Cylinder 5 misfire",
                "Wypadanie zapłonów – cylinder 5",
            ),
            Self::P0306 => pick(
                lang,
                "Cylinder 6 misfire",
                "Wypadanie zapłonów – cylinder 6",
            ),
            Self::P0307 => pick(
                lang,
                "Cylinder 7 misfire",
                "Wypadanie zapłonów – cylinder 7",
            ),
            Self::P0308 => pick(
                lang,
                "Cylinder 8 misfire",
                "Wypadanie zapłonów – cylinder 8",
            ),
            Self::P0325 => pick(
                lang,
                "Knock sensor circuit malfunction",
                "Usterka obwodu czujnika spalania stukowego",
            ),
            Self::P0420 => pick(
                lang,
                "Catalyst efficiency below threshold",
                "Sprawność katalizatora poniżej progu",
            ),
            Self::P0507 => pick(
                lang,
                "Idle speed higher than expected",
                "Obroty biegu jałowego wyższe od oczekiwanych",
            ),
            Self::P0524 => pick(
                lang,
                "Engine oil pressure too low",
                "Zbyt niskie ciśnienie oleju silnikowego",
            ),
        }
    }
}

impl Localize for FaultId {
    fn localized(&self, lang: Language) -> &'static str {
        match self {
            Self::MapSensorBias => pick(
                lang,
                "MAP sensor offset",
                "Przesunięcie wskazań czujnika MAP",
            ),
            Self::CoolantSensorBias => pick(
                lang,
                "Coolant temperature sensor offset",
                "Przesunięcie wskazań czujnika temperatury płynu",
            ),
            Self::IntakeAirSensorOpen => pick(
                lang,
                "Intake air temperature sensor open circuit",
                "Przerwa w obwodzie czujnika temperatury powietrza",
            ),
            Self::OxygenSensorSlow => pick(
                lang,
                "Slow (aged) oxygen sensor",
                "Wolna (zużyta) sonda lambda",
            ),
            Self::OxygenSensorBias => pick(
                lang,
                "Oxygen sensor offset / exhaust leak",
                "Przesunięcie wskazań sondy lambda / nieszczelny wydech",
            ),
            Self::KnockSensorDead => pick(
                lang,
                "Knock sensor disconnected",
                "Odłączony czujnik spalania stukowego",
            ),
            Self::VacuumLeak => pick(lang, "Vacuum leak", "Nieszczelność dolotu (lewe powietrze)"),
            Self::InjectorClogged => pick(lang, "Clogged injector", "Zapchany wtryskiwacz"),
            Self::IgnitionCoilWeak => pick(lang, "Weak ignition coil", "Słaba cewka zapłonowa"),
            Self::ThermostatStuckOpen => pick(
                lang,
                "Thermostat stuck open",
                "Termostat zablokowany w pozycji otwartej",
            ),
            Self::ThermostatStuckClosed => pick(
                lang,
                "Thermostat stuck closed",
                "Termostat zablokowany w pozycji zamkniętej",
            ),
            Self::TimingChainStretch => pick(
                lang,
                "Stretched timing chain",
                "Wyciągnięty łańcuch rozrządu",
            ),
            Self::FuelPumpWeak => pick(lang, "Weak fuel pump", "Słaba pompa paliwa"),
            Self::ExhaustRestriction => pick(lang, "Restricted exhaust", "Zatkany układ wydechowy"),
            Self::LowCompression => pick(lang, "Low compression", "Niska kompresja"),
            Self::BoostLeak => pick(lang, "Boost leak", "Nieszczelność układu doładowania"),
            Self::WastegateStuckClosed => pick(
                lang,
                "Wastegate stuck closed",
                "Zawór upustowy (wastegate) zablokowany w pozycji zamkniętej",
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_message_is_translated_and_distinct() {
        let mut keys: Vec<MessageKey> = Vec::new();
        keys.extend(Warning::ALL.iter().map(|w| MessageKey::Warning(*w)));
        keys.extend(DtcCode::ALL.iter().map(|d| MessageKey::Dtc(*d)));
        keys.extend(FaultId::ALL.iter().map(|f| MessageKey::Fault(*f)));
        keys.push(MessageKey::Condition(EngineCondition::Running));
        keys.push(MessageKey::Failure(FailureCause::SpunBearing));
        for k in keys {
            let en = text(k, Language::En);
            let pl = text(k, Language::Pl);
            assert!(!en.is_empty() && !pl.is_empty());
            assert_ne!(en, pl, "{k:?} untranslated");
        }
    }

    #[test]
    fn language_codes_round_trip() {
        for l in Language::ALL {
            assert_eq!(Language::from_code(l.code()), Some(l));
        }
        assert_eq!(Language::from_code("pl"), Some(Language::Pl));
        assert_eq!(Language::from_code("de"), None);
        let loc = StaticLocalizer {
            language: Language::Pl,
        };
        assert_eq!(
            loc.text(MessageKey::Warning(Warning::Knock)),
            "Wykryto spalanie stukowe"
        );
    }
}

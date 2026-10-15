//! Guided course: short lessons whose steps are checked automatically against the live
//! simulation, so the student learns on the real dashboard, maps and dyno instead of reading
//! about them.
//!
//! A step is complete when its check has held continuously for [`HOLD_S`] (a momentary
//! blip does not count); steps without a check are confirmed with the "Next" button.

use eframe::egui::{self, RichText};
use tunerlab_core::engine_sim::calibration::Table2D;
use tunerlab_core::{Calibration, EngineCondition, Language, Telemetry};

use crate::app::{LoadKind, Tab};
use crate::maps::MapKind;
use crate::widgets::palette::*;

/// Time a check must hold before its step counts as done \[s\].
const HOLD_S: f32 = 0.6;
/// How long the "step done" confirmation stays visible \[s\].
const FLASH_S: f32 = 2.0;

/// Live state the steps are checked against.
pub struct Ctx<'a> {
    pub t: &'a Telemetry,
    pub ignition: bool,
    pub cal: &'a Calibration,
    /// Base calibration of the current engine (what "restore base map" brings back).
    pub base: &'a Calibration,
    pub turbo: bool,
    pub tab: Tab,
    pub map_kind: MapKind,
    pub load: LoadKind,
    pub octane: f32,
    /// Dyno sweeps finished since the current step started.
    pub new_runs: u32,
    /// An Engine Autopsy case is open.
    pub case_open: bool,
    /// The open case was diagnosed correctly or its solution revealed.
    pub case_closed: bool,
}

impl Ctx<'_> {
    fn running(&self) -> bool {
        self.t.condition == EngineCondition::Running
    }

    fn warm(&self) -> bool {
        self.running() && self.t.sensors.ect_c > 80.0
    }

    /// Boost seen by the ECU: manifold pressure above the barometric reading \[kPa\].
    fn boost_kpa(&self) -> f32 {
        self.t.sensors.map_kpa - self.t.sensors.baro_kpa
    }
}

/// One instruction with its automatic check.
pub struct Step {
    /// Instruction, `[English, Polish]`.
    pub text: [&'static str; 2],
    /// Why it matters or how to do it, `[English, Polish]`.
    pub hint: [&'static str; 2],
    /// Completion test; `None` means the student confirms with "Next".
    pub done: Option<fn(&Ctx) -> bool>,
}

pub struct Lesson {
    pub title: [&'static str; 2],
    pub intro: [&'static str; 2],
    pub steps: &'static [Step],
}

fn pick(s: &[&'static str; 2], lang: Language) -> &'static str {
    match lang {
        Language::En => s[0],
        Language::Pl => s[1],
    }
}

fn l(lang: Language, en: &'static str, pl: &'static str) -> &'static str {
    pick(&[en, pl], lang)
}

/// Mean cell value over the cells whose axis values fall in the given ranges (0 when the
/// region is empty).
fn region_mean<const X: usize, const Y: usize>(
    t: &Table2D<X, Y>,
    x: (f32, f32),
    y: (f32, f32),
) -> f32 {
    let (mut sum, mut n) = (0.0, 0u32);
    for (iy, &yv) in t.y_axis.iter().enumerate() {
        for (ix, &xv) in t.x_axis.iter().enumerate() {
            if (x.0..=x.1).contains(&xv) && (y.0..=y.1).contains(&yv) {
                sum += t.values[iy][ix];
                n += 1;
            }
        }
    }
    if n == 0 {
        0.0
    } else {
        sum / n as f32
    }
}

/// Mean change of `cal` against `base` over a region.
fn region_delta<const X: usize, const Y: usize>(
    cal: &Table2D<X, Y>,
    base: &Table2D<X, Y>,
    x: (f32, f32),
    y: (f32, f32),
) -> f32 {
    region_mean(cal, x, y) - region_mean(base, x, y)
}

/// Idle area of the VE map used in the VE lesson.
const VE_IDLE: ((f32, f32), (f32, f32)) = ((500.0, 1600.0), (20.0, 40.0));
/// Full-load area of the spark map used in the ignition lesson.
const SPARK_WOT: ((f32, f32), (f32, f32)) = ((2400.0, 5000.0), (88.0, 105.0));
/// Full-load, high-speed area of the target-lambda map.
const LAMBDA_WOT: ((f32, f32), (f32, f32)) = ((3200.0, 7500.0), (88.0, 105.0));
/// Large-pedal area of the boost-target map.
const BOOST_WOT: ((f32, f32), (f32, f32)) = ((2000.0, 6800.0), (75.0, 100.0));

fn ve_delta_points(c: &Ctx) -> f32 {
    100.0 * region_delta(&c.cal.ve, &c.base.ve, VE_IDLE.0, VE_IDLE.1)
}

fn spark_delta(c: &Ctx) -> f32 {
    region_delta(&c.cal.ignition, &c.base.ignition, SPARK_WOT.0, SPARK_WOT.1)
}

fn lambda_delta(c: &Ctx) -> f32 {
    region_delta(
        &c.cal.lambda_target,
        &c.base.lambda_target,
        LAMBDA_WOT.0,
        LAMBDA_WOT.1,
    )
}

/// Mean target lambda over the full-load area.
fn lambda_target(c: &Ctx) -> f32 {
    region_mean(&c.cal.lambda_target, LAMBDA_WOT.0, LAMBDA_WOT.1)
}

fn boost_delta(c: &Ctx) -> f32 {
    region_delta(
        &c.cal.boost.target_kpa,
        &c.base.boost.target_kpa,
        BOOST_WOT.0,
        BOOST_WOT.1,
    )
}

/// The course content.
pub static LESSONS: &[Lesson] = &[
    Lesson {
        title: ["First start", "Pierwsze uruchomienie"],
        intro: [
            "Get to know the dashboard and start a cold engine. Watch the ECU take it from cranking to closed-loop running.",
            "Poznasz pulpit i uruchomisz zimny silnik. Zobaczysz, jak ECU prowadzi go od rozruchu do pracy w pętli zamkniętej.",
        ],
        steps: &[
            Step {
                text: [
                    "Switch the ignition on: tick “Ignition (key)” in the left panel or press K.",
                    "Włącz zapłon: zaznacz „Zapłon (stacyjka)” w lewym panelu albo naciśnij K.",
                ],
                hint: [
                    "With the key on, the ECU wakes its sensors and primes the fuel pump — fuel pressure appears on the dashboard.",
                    "Po włączeniu zapłonu ECU budzi czujniki i pompę paliwa — na pulpicie pojawia się ciśnienie paliwa.",
                ],
                done: Some(|c| c.ignition),
            },
            Step {
                text: [
                    "Start the engine: hold S (or the “Starter” button) until it fires.",
                    "Uruchom silnik: przytrzymaj S (albo przycisk „Rozrusznik”), aż silnik zaskoczy.",
                ],
                hint: [
                    "A cold engine gets a cranking enrichment — the colder the coolant, the more fuel.",
                    "Zimny silnik dostaje dawkę rozruchową — im zimniejszy płyn chłodzący, tym więcej paliwa.",
                ],
                done: Some(|c| c.running()),
            },
            Step {
                text: [
                    "Watch the tachometer: a cold engine idles fast.",
                    "Obserwuj obrotomierz: zimny silnik pracuje na podwyższonych obrotach jałowych.",
                ],
                hint: [
                    "Fast idle warms the engine and the catalyst sooner; the ECU brings it down as the coolant warms.",
                    "Podwyższone obroty szybciej rozgrzewają silnik i katalizator; ECU obniża je, gdy płyn się nagrzewa.",
                ],
                done: Some(|c| c.running() && (650.0..1600.0).contains(&c.t.rpm)),
            },
            Step {
                text: [
                    "Warm the engine up: click “Warm up” in the left panel, or pick speed 5× and wait until the coolant passes 80 °C.",
                    "Rozgrzej silnik: kliknij „Rozgrzej” w lewym panelu albo wybierz tempo 5× i poczekaj, aż płyn chłodzący przekroczy 80 °C.",
                ],
                hint: [
                    "Coolant temperature is one of the ECU's key inputs: it drives enrichment, idle speed and spark.",
                    "Temperatura płynu to jeden z najważniejszych sygnałów ECU: steruje wzbogaceniem, obrotami jałowymi i zapłonem.",
                ],
                done: Some(|c| c.warm()),
            },
            Step {
                text: [
                    "Wait for closed loop: “Open loop” in the right panel turns into “Closed loop”.",
                    "Poczekaj na pętlę zamkniętą: w prawym panelu „Pętla otwarta” zmieni się na „Pętla zamknięta”.",
                ],
                hint: [
                    "Closed loop means the ECU trims the fuel from the oxygen sensor. It needs a warm engine and a hot sensor.",
                    "Pętla zamknięta oznacza, że ECU koryguje dawkę paliwa według sondy lambda. Wymaga ciepłego silnika i rozgrzanej sondy.",
                ],
                done: Some(|c| c.running() && c.t.closed_loop),
            },
            Step {
                text: [
                    "Open the throttle: hold W for a moment and watch the “Manifold” gauge.",
                    "Dodaj gazu: przytrzymaj W przez chwilę i obserwuj wskaźnik „Kolektor”.",
                ],
                hint: [
                    "MAP (manifold pressure) tells the ECU how much air the cylinders take in: about 30 kPa (vacuum) at idle, almost atmospheric at full throttle.",
                    "MAP (ciśnienie w kolektorze) mówi ECU, ile powietrza wpada do cylindrów: ok. 30 kPa (podciśnienie) na biegu jałowym, prawie atmosferyczne przy pełnym gazie.",
                ],
                done: Some(|c| c.running() && c.t.rpm > 2000.0 && c.t.sensors.map_kpa > 85.0),
            },
        ],
    },
    Lesson {
        title: ["VE map and fuel trims", "Mapa VE i korekty paliwa"],
        intro: [
            "The ECU computes the air mass from MAP, temperature and the VE (volumetric efficiency) map, and the fuel dose from that. A VE error is a mixture error that closed loop tries to patch with fuel trims. A tuner corrects VE until the trims sit near zero.",
            "ECU liczy masę powietrza z MAP, temperatury i mapy VE (sprawności napełniania), a z niej dawkę paliwa. Błąd w VE to błąd mieszanki, który pętla zamknięta łata korektami paliwa. Tuner poprawia VE tak, by korekty były bliskie zera.",
        ],
        steps: &[
            Step {
                text: [
                    "Run the warm engine at idle and wait for closed loop (“Warm up” speeds this up).",
                    "Uruchom ciepły silnik na biegu jałowym i poczekaj na pętlę zamkniętą („Rozgrzej” przyspiesza).",
                ],
                hint: [
                    "Fuel trims only work in closed loop.",
                    "Korekty paliwa działają tylko w pętli zamkniętej.",
                ],
                done: Some(|c| c.running() && c.t.closed_loop),
            },
            Step {
                text: [
                    "Look at the “Trims ST/LT” tile: STFT is the fast trim (seconds), LTFT the learned long-term one. On a healthy engine both sit near 0 %.",
                    "Spójrz na kafelek „Korekty ST/LT”: STFT to szybka korekta (sekundy), LTFT — wyuczona, długoterminowa. Na sprawnym silniku obie są bliskie 0 %.",
                ],
                hint: [
                    "Up to ±10 % is normal; beyond 20 % points to a fault (leak, injector, sensor) or a bad map.",
                    "Do ±10 % to norma; powyżej 20 % to sygnał usterki (nieszczelność, wtryskiwacz, czujnik) albo złej mapy.",
                ],
                done: None,
            },
            Step {
                text: [
                    "Open the “Maps” tab → VE table. The dot marks the current operating point.",
                    "Otwórz zakładkę „Mapy” → tablica VE. Kropka pokazuje bieżący punkt pracy.",
                ],
                hint: [
                    "Columns are engine speed, rows are MAP. At idle the point sits around 800 rpm and 25–35 kPa.",
                    "Kolumny to obroty, wiersze to MAP. Na biegu jałowym punkt leży ok. 800 obr/min i 25–35 kPa.",
                ],
                done: Some(|c| c.tab == Tab::Maps && c.map_kind == MapKind::Ve),
            },
            Step {
                text: [
                    "Select the cells 500–1600 rpm × 20–40 kPa with the mouse and lower VE by about 10 points (step 1, press “−” ten times).",
                    "Zaznacz myszą komórki 500–1600 obr/min × 20–40 kPa i zmniejsz VE o ok. 10 punktów (krok 1, dziesięć razy „−”).",
                ],
                hint: [
                    "The ECU now believes less air enters the cylinders, so it injects less fuel and the mixture goes lean.",
                    "ECU myśli teraz, że do cylindrów wpada mniej powietrza, więc podaje mniej paliwa — mieszanka zubaża się.",
                ],
                done: Some(|c| ve_delta_points(c) <= -7.0),
            },
            Step {
                text: [
                    "Watch “Trims ST/LT” on the dashboard: the ECU sees a lean mixture on the sensor and adds fuel. Wait until the trims add up to more than +6 %.",
                    "Obserwuj „Korekty ST/LT” na pulpicie: ECU widzi na sondzie ubogą mieszankę i dodaje paliwa. Poczekaj, aż suma korekt przekroczy +6 %.",
                ],
                hint: [
                    "This is what a mismatched VE map looks like in ECU data: trims stuck positive (or negative) in one area of the map.",
                    "Tak wygląda źle dopasowana mapa VE w danych z ECU: korekty stale na plus (albo minus) w jednym obszarze mapy.",
                ],
                done: Some(|c| c.running() && c.t.stft_pct + c.t.ltft_pct > 6.0),
            },
            Step {
                text: [
                    "Fix the map: in “Maps”, click “Restore base map”. The trims return towards zero (LTFT unlearns more slowly).",
                    "Napraw mapę: w „Mapach” kliknij „Przywróć mapę bazową”. Korekty wrócą w stronę zera (LTFT oducza się wolniej).",
                ],
                hint: [
                    "A real tuner works the other way round: adjust VE until the trims sit near zero in every area — then open loop at full load hits its target too.",
                    "Prawdziwy tuner robi odwrotnie: zmienia VE, aż korekty w każdym obszarze są bliskie zera — wtedy pętla otwarta przy pełnym obciążeniu też trafia w cel.",
                ],
                done: Some(|c| ve_delta_points(c).abs() < 0.5),
            },
        ],
    },
    Lesson {
        title: ["Ignition and knock", "Zapłon i stuk"],
        intro: [
            "More spark advance adds torque only up to MBT. Beyond it torque stops rising, and at full load knock appears — abnormal combustion that destroys pistons. You will see it on the dyno.",
            "Wcześniejszy zapłon zwiększa moment tylko do punktu MBT. Dalej moment już nie rośnie, a przy pełnym obciążeniu pojawia się stuk — nieprawidłowe spalanie, które niszczy tłoki. Zobaczysz to na hamowni.",
        ],
        steps: &[
            Step {
                text: [
                    "“Dyno” tab → “Run WOT power sweep”. This is your baseline.",
                    "Zakładka „Hamownia” → „Pomiar mocy (pełny gaz)”. To twój pomiar bazowy.",
                ],
                hint: [
                    "The dyno measures a copy of your engine with its current calibration and fuel. Note peak torque — the previous run stays on the chart, dimmed.",
                    "Hamownia mierzy kopię twojego silnika z bieżącą kalibracją i paliwem. Zapamiętaj maks. moment — poprzedni pomiar zostaje na wykresie, przygaszony.",
                ],
                done: Some(|c| c.new_runs >= 1),
            },
            Step {
                text: [
                    "“Maps” → “Ignition advance”: select 2400–5000 rpm × 88–105 kPa and add +5° (step 0.5 pressed ten times, or set the step to 5).",
                    "„Mapy” → „Wyprzedzenie zapłonu”: zaznacz 2400–5000 obr/min × 88–105 kPa i dodaj +5° (krok 0.5 dziesięć razy albo ustaw krok 5).",
                ],
                hint: [
                    "This is the full-load area, where the base map already sits at the knock limit.",
                    "To obszar pełnego obciążenia, w którym mapa bazowa jest już na granicy stuku.",
                ],
                done: Some(|c| spark_delta(c) >= 4.0),
            },
            Step {
                text: [
                    "Run the sweep again and compare.",
                    "Zmierz moc ponownie i porównaj.",
                ],
                hint: [
                    "Torque barely changed, while red values appear in the “knock [bar]” column — the ECU pulls timing (knock retard).",
                    "Moment praktycznie się nie zmienił, a w kolumnie „stuk [bar]” pojawiły się czerwone wartości — ECU cofa zapłon (korekta stukowa).",
                ],
                done: Some(|c| c.new_runs >= 1),
            },
            Step {
                text: [
                    "Pick 91 RON fuel in the left panel.",
                    "Wybierz w lewym panelu paliwo 91 RON.",
                ],
                hint: [
                    "Lower-octane fuel resists knock less.",
                    "Paliwo o niższej liczbie oktanowej gorzej opiera się spalaniu stukowemu.",
                ],
                done: Some(|c| c.octane <= 91.5),
            },
            Step {
                text: [
                    "Run the sweep on 91 RON.",
                    "Zmierz moc na paliwie 91 RON.",
                ],
                hint: [
                    "Knock grows, the ECU retards harder and torque drops — the same map is not safe on every fuel.",
                    "Stuk rośnie, ECU mocniej cofa zapłon i moment spada — ta sama mapa nie jest bezpieczna na każdym paliwie.",
                ],
                done: Some(|c| c.new_runs >= 1 && c.octane <= 91.5),
            },
            Step {
                text: [
                    "Clean up: restore the base ignition map and select 95 RON.",
                    "Posprzątaj: przywróć bazową mapę zapłonu i wybierz paliwo 95 RON.",
                ],
                hint: [
                    "The takeaway: more spark ≠ more power. Spark is set at MBT or just short of the knock limit, with margin for poor fuel and hot days.",
                    "Wniosek: więcej zapłonu ≠ więcej mocy. Zapłon ustawia się na MBT albo tuż przed granicą stuku, z zapasem na gorsze paliwo i upał.",
                ],
                done: Some(|c| spark_delta(c).abs() < 0.25 && c.octane >= 94.5),
            },
        ],
    },
    Lesson {
        title: ["Mixture under load", "Mieszanka pod obciążeniem"],
        intro: [
            "At full throttle the ECU runs open loop and aims for the target lambda from its map. A rich mixture cools the chamber and the exhaust; a lean one gives a little power but heats pistons and valves.",
            "Przy pełnym gazie ECU pracuje w pętli otwartej i celuje w docelową lambdę z mapy. Bogata mieszanka chłodzi komorę i spaliny; uboga daje trochę mocy, ale grzeje tłoki i zawory.",
        ],
        steps: &[
            Step {
                text: [
                    "Dyno → baseline sweep. Note the λ and EGT °C columns.",
                    "Hamownia → pomiar bazowy. Zwróć uwagę na kolumny λ i EGT °C.",
                ],
                hint: [
                    "The base map targets about λ 0.84 at full load.",
                    "Mapa bazowa celuje w λ ok. 0.84 przy pełnym obciążeniu.",
                ],
                done: Some(|c| c.new_runs >= 1),
            },
            Step {
                text: [
                    "“Maps” → “Target lambda”: select 3200–7500 rpm × 88–105 kPa, type 0.95 in “Set to” and click “Apply”.",
                    "„Mapy” → „Docelowa lambda”: zaznacz 3200–7500 obr/min × 88–105 kPa, wpisz 0.95 w „Ustaw na” i kliknij „Zastosuj”.",
                ],
                hint: [
                    "This leans the mixture out at full throttle — a typical temptation when tuning for power.",
                    "To zubożenie mieszanki przy pełnym gazie — typowa pokusa przy strojeniu na moc.",
                ],
                done: Some(|c| lambda_target(c) >= 0.93),
            },
            Step {
                text: [
                    "Run the sweep again and compare torque and EGT.",
                    "Zmierz moc ponownie i porównaj moment oraz EGT.",
                ],
                hint: [
                    "Torque rose by a few N·m, but EGT went up — under sustained load that is the road to burnt pistons and valves.",
                    "Moment wzrósł o kilka N·m, ale EGT poszła w górę — przy długim obciążeniu to droga do przepalonych tłoków i zaworów.",
                ],
                done: Some(|c| c.new_runs >= 1),
            },
            Step {
                text: [
                    "Now go rich: set 0.80 in the same area.",
                    "Teraz wzbogać: ustaw 0.80 w tym samym obszarze.",
                ],
                hint: [
                    "Extra fuel absorbs heat as it evaporates — the classic way to protect an engine under load.",
                    "Dodatkowe paliwo odbiera ciepło, gdy odparowuje — klasyczny sposób ochrony silnika pod obciążeniem.",
                ],
                done: Some(|c| lambda_target(c) <= 0.82),
            },
            Step {
                text: [
                    "Run the sweep on the rich map.",
                    "Zmierz moc na bogatej mapie.",
                ],
                hint: [
                    "Lower EGT, but also less torque and more fuel burnt. Tuners look for the compromise — usually λ 0.80–0.88 at full load, richer on turbo engines.",
                    "Niższa EGT, ale też mniej momentu i więcej spalonego paliwa. Tuner szuka kompromisu — zwykle λ 0.80–0.88 przy pełnym obciążeniu, w turbo bogaciej.",
                ],
                done: Some(|c| c.new_runs >= 1),
            },
            Step {
                text: [
                    "Restore the base target-lambda map.",
                    "Przywróć bazową mapę docelowej lambdy.",
                ],
                hint: [
                    "Always finish an experiment with a known-good map.",
                    "Każdy eksperyment kończ na sprawdzonej mapie.",
                ],
                done: Some(|c| lambda_delta(c).abs() < 0.005),
            },
        ],
    },
    Lesson {
        title: ["Turbo and boost", "Turbo i doładowanie"],
        intro: [
            "On the turbo engine the ECU drives the wastegate to hold the boost target from its map (engine speed × pedal). Boost is manifold pressure above atmospheric.",
            "W silniku turbo ECU steruje zaworem upustowym (wastegate), żeby utrzymać docelowe doładowanie z mapy (obroty × pedał gazu). Doładowanie to ciśnienie w kolektorze powyżej atmosferycznego.",
        ],
        steps: &[
            Step {
                text: [
                    "Pick the “2.0 L turbo” engine in the top bar.",
                    "Wybierz silnik „2.0 L turbo” na górnym pasku.",
                ],
                hint: [
                    "Switching engines creates a fresh, cold engine with its base map.",
                    "Zmiana silnika tworzy nowy, zimny silnik z mapą bazową.",
                ],
                done: Some(|c| c.turbo),
            },
            Step {
                text: [
                    "Start the engine (K, S) and click “Warm up”.",
                    "Uruchom silnik (K, S) i kliknij „Rozgrzej”.",
                ],
                hint: [
                    "The turbo engine idles like the naturally aspirated one — the turbo only matters under load.",
                    "Silnik turbo na biegu jałowym pracuje jak wolnossący — turbo liczy się dopiero pod obciążeniem.",
                ],
                done: Some(|c| c.turbo && c.warm()),
            },
            Step {
                text: [
                    "Load → “Dynamometer”, dyno speed 3500 rpm, then hold W and watch the “Boost” tile.",
                    "Obciążenie → „Hamownia”, obroty hamowni 3500 obr/min, potem przytrzymaj W i obserwuj kafelek „Doładowanie”.",
                ],
                hint: [
                    "The turbo needs a moment to spool up (turbo lag) — the pressure reaches the target after a fraction of a second.",
                    "Turbosprężarka potrzebuje chwili, żeby się rozpędzić (turbo lag) — ciśnienie dochodzi do celu po ułamku sekundy.",
                ],
                done: Some(|c| {
                    c.turbo && c.running() && c.load == LoadKind::Dyno && c.boost_kpa() >= 50.0
                }),
            },
            Step {
                text: [
                    "“Maps” → “Boost target”: select 2000–6800 rpm for pedal 75–100 % and lower the target by 30 kPa (step 5, press “−” six times).",
                    "„Mapy” → „Docelowe doładowanie”: zaznacz 2000–6800 obr/min dla pedału 75–100 % i obniż cel o 30 kPa (krok 5, sześć razy „−”).",
                ],
                hint: [
                    "The target is absolute pressure: 180 kPa is about 0.8 bar of boost at 101 kPa atmospheric pressure.",
                    "Cel to ciśnienie absolutne: 180 kPa to ok. 0.8 bar doładowania przy ciśnieniu atmosferycznym 101 kPa.",
                ],
                done: Some(|c| c.turbo && boost_delta(c) <= -25.0),
            },
            Step {
                text: [
                    "Dyno tab → run the power sweep and compare with the turbo base figures (about 297 N·m / 154 kW).",
                    "Hamownia → pomiar mocy; porównaj z wynikiem bazowym turbo (ok. 297 N·m / 154 kW).",
                ],
                hint: [
                    "Less boost = less air = less fuel and power. More boost raises power, and with it the knock risk and the load on the turbo.",
                    "Mniej doładowania = mniej powietrza = mniej paliwa i mocy. Więcej doładowania podnosi moc, a razem z nią ryzyko stuku i obciążenie turbosprężarki.",
                ],
                done: Some(|c| c.turbo && c.new_runs >= 1),
            },
            Step {
                text: [
                    "Restore the base boost map.",
                    "Przywróć bazową mapę doładowania.",
                ],
                hint: [
                    "Boost changes always go together with fuel and spark checks on the dyno.",
                    "Zmiana doładowania zawsze idzie w parze ze sprawdzeniem paliwa i zapłonu na hamowni.",
                ],
                done: Some(|c| c.turbo && boost_delta(c).abs() < 1.0),
            },
        ],
    },
    Lesson {
        title: ["Diagnostics: Engine Autopsy", "Diagnostyka: Engine Autopsy"],
        intro: [
            "A diagnostician works like a detective: symptoms first (engine behaviour, trims, codes), then a hypothesis and a test. The program hides a fault — find it.",
            "Diagnosta pracuje jak detektyw: najpierw objawy (zachowanie silnika, korekty, kody), potem hipoteza i test. Program ukryje usterkę — znajdź ją.",
        ],
        steps: &[
            Step {
                text: [
                    "“Engine Autopsy” tab → “New case”.",
                    "Zakładka „Engine Autopsy” → „Nowy przypadek”.",
                ],
                hint: [
                    "The instructor view is now locked — you only have what the ECU sees, as in a real workshop.",
                    "Widok instruktora jest teraz zablokowany — masz tylko to, co widzi ECU, jak w prawdziwym warsztacie.",
                ],
                done: Some(|c| c.case_open),
            },
            Step {
                text: [
                    "Start the engine and warm it up. Watch the idle, the fuel trims and the trouble codes in the right panel.",
                    "Uruchom silnik i rozgrzej go. Obserwuj bieg jałowy, korekty paliwa i kody usterek w prawym panelu.",
                ],
                hint: [
                    "A rough idle or trims far from zero are the first clues.",
                    "Nierówne obroty albo korekty daleko od zera to pierwsze tropy.",
                ],
                done: Some(|c| c.case_open && c.warm()),
            },
            Step {
                text: [
                    "Load it: Load → “Dynamometer” at 2500 rpm, pedal 30–50 % (Up/Down arrows), then full throttle (W).",
                    "Obciąż silnik: Obciążenie → „Hamownia” 2500 obr/min, pedał 30–50 % (strzałki góra/dół), potem pełny gaz (W).",
                ],
                hint: [
                    "Many faults only show under load: a weak pump, a weak coil, a blocked exhaust.",
                    "Wiele usterek ujawnia się dopiero pod obciążeniem: słaba pompa, słaba cewka, zatkany wydech.",
                ],
                done: Some(|c| {
                    c.case_open && c.running() && c.load == LoadKind::Dyno && c.t.pedal_pct >= 25.0
                }),
            },
            Step {
                text: [
                    "Check the “Event log” and the cylinder table on the dashboard (misfires, knock retard). A dyno sweep helps too.",
                    "Sprawdź „Dziennik” i tabelę cylindrów na pulpicie (wypadanie zapłonu, korekta stukowa). Pomoże też pomiar na hamowni.",
                ],
                hint: [
                    "Look for a pattern: one cylinder = coil, injector or compression; all cylinders and lean = pump or leak; a sensor = odd readings while the engine runs normally.",
                    "Szukaj wzorca: jeden cylinder = cewka, wtryskiwacz albo kompresja; wszystkie i ubogo = pompa albo nieszczelność; czujnik = dziwne odczyty przy normalnej pracy.",
                ],
                done: None,
            },
            Step {
                text: [
                    "Pick your diagnosis in “Engine Autopsy” and click “Check”.",
                    "Wybierz diagnozę w „Engine Autopsy” i kliknij „Sprawdź”.",
                ],
                hint: [
                    "A wrong guess costs nothing but an attempt. “Reveal” ends the case.",
                    "Pomyłka kosztuje tylko próbę. „Pokaż rozwiązanie” kończy przypadek.",
                ],
                done: Some(|c| c.case_closed),
            },
        ],
    },
];

/// Lesson in progress.
#[derive(Debug, Clone, Copy)]
struct Active {
    lesson: usize,
    step: usize,
    /// Time the current check has held continuously \[s\].
    held_s: f32,
    /// Finished dyno sweeps when the current step started.
    runs_at_start: u32,
}

/// Course progress (kept for the session).
pub struct Course {
    /// Lesson shown in the Course tab.
    pub selected: usize,
    active: Option<Active>,
    completed: Vec<bool>,
    /// Remaining display time of the "step done" / "lesson done" message \[s\].
    flash_s: f32,
    flash_lesson_done: bool,
}

impl Default for Course {
    fn default() -> Self {
        Self {
            selected: 0,
            active: None,
            completed: vec![false; LESSONS.len()],
            flash_s: 0.0,
            flash_lesson_done: false,
        }
    }
}

impl Course {
    pub fn is_active(&self) -> bool {
        self.active.is_some() || self.flash_s > 0.0
    }

    /// Dyno sweeps the current step has seen (`total` = sweeps finished so far).
    pub fn new_runs(&self, total: u32) -> u32 {
        self.active
            .map_or(0, |a| total.saturating_sub(a.runs_at_start))
    }

    fn start(&mut self, lesson: usize, total_runs: u32) {
        self.selected = lesson;
        self.flash_s = 0.0;
        self.active = Some(Active {
            lesson,
            step: 0,
            held_s: 0.0,
            runs_at_start: total_runs,
        });
    }

    fn advance(&mut self, total_runs: u32) {
        let Some(a) = &mut self.active else { return };
        a.step += 1;
        a.held_s = 0.0;
        a.runs_at_start = total_runs;
        self.flash_s = FLASH_S;
        self.flash_lesson_done = false;
        if a.step >= LESSONS[a.lesson].steps.len() {
            self.completed[a.lesson] = true;
            self.flash_lesson_done = true;
            self.active = None;
        }
    }

    /// Checks the current step against the live state; `dt` is wall-clock time \[s\].
    pub fn update(&mut self, ctx: &Ctx, dt: f32, total_runs: u32) {
        self.flash_s = (self.flash_s - dt).max(0.0);
        let Some(a) = &mut self.active else { return };
        let step = &LESSONS[a.lesson].steps[a.step];
        let Some(done) = step.done else { return };
        if done(ctx) {
            a.held_s += dt;
            if a.held_s >= HOLD_S {
                self.advance(total_runs);
            }
        } else {
            a.held_s = 0.0;
        }
    }

    /// Bottom strip with the current step, shown on every tab while a lesson runs.
    pub fn strip_ui(&mut self, ui: &mut egui::Ui, lang: Language, total_runs: u32) {
        let Some(a) = self.active else {
            if self.flash_lesson_done {
                ui.label(
                    RichText::new(l(
                        lang,
                        "✔ Lesson complete! Pick the next one in the “Course” tab.",
                        "✔ Lekcja ukończona! Następną wybierzesz w zakładce „Kurs”.",
                    ))
                    .color(OK)
                    .strong(),
                );
            }
            return;
        };
        let lesson = &LESSONS[a.lesson];
        let step = &lesson.steps[a.step];
        ui.horizontal_wrapped(|ui| {
            ui.label(
                RichText::new(format!(
                    "{} {}: {}",
                    l(lang, "Lesson", "Lekcja"),
                    a.lesson + 1,
                    pick(&lesson.title, lang)
                ))
                .color(ACCENT)
                .strong(),
            );
            ui.label(
                RichText::new(format!(
                    "{} {}/{}",
                    l(lang, "step", "krok"),
                    a.step + 1,
                    lesson.steps.len()
                ))
                .color(DIM),
            );
            ui.add(
                egui::ProgressBar::new(a.step as f32 / lesson.steps.len() as f32)
                    .desired_width(120.0),
            );
            if self.flash_s > 0.0 {
                ui.label(
                    RichText::new(l(lang, "✔ Step done", "✔ Krok zaliczony"))
                        .color(OK)
                        .strong(),
                );
            }
        });
        ui.label(RichText::new(pick(&step.text, lang)).size(15.0).strong());
        ui.label(RichText::new(pick(&step.hint, lang)).color(DIM).size(12.5));
        ui.horizontal_wrapped(|ui| {
            if step.done.is_none() {
                if ui
                    .button(RichText::new(l(lang, "Next", "Dalej")).strong())
                    .clicked()
                {
                    self.advance(total_runs);
                }
            } else {
                ui.label(
                    RichText::new(l(
                        lang,
                        "The step completes itself when you do it.",
                        "Krok zaliczy się sam, gdy go wykonasz.",
                    ))
                    .color(DIM)
                    .size(12.0),
                );
            }
            if ui
                .small_button(l(lang, "Skip step", "Pomiń krok"))
                .clicked()
            {
                self.advance(total_runs);
            }
            if ui
                .small_button(l(lang, "End lesson", "Zakończ lekcję"))
                .clicked()
            {
                self.active = None;
                self.flash_s = 0.0;
            }
        });
    }

    /// The Course tab: lesson list, lesson outline and start buttons.
    pub fn tab_ui(&mut self, ui: &mut egui::Ui, lang: Language, total_runs: u32) {
        ui.label(
            RichText::new(l(
                lang,
                "Each lesson is a series of short tasks on the live engine. A step is ticked off automatically as soon as you do it; the current step is shown at the bottom of every tab.",
                "Każda lekcja to seria krótkich zadań na żywym silniku. Krok zalicza się automatycznie, gdy go wykonasz; bieżący krok widać na dole każdej zakładki.",
            ))
            .color(DIM),
        );
        ui.add_space(6.0);
        ui.horizontal_wrapped(|ui| {
            for (i, lesson) in LESSONS.iter().enumerate() {
                let mark = if self.completed[i] { "✔ " } else { "" };
                let running = self.active.is_some_and(|a| a.lesson == i);
                let text = format!("{mark}{}. {}", i + 1, pick(&lesson.title, lang));
                let text = if running {
                    RichText::new(text).color(ACCENT)
                } else if self.completed[i] {
                    RichText::new(text).color(OK)
                } else {
                    RichText::new(text)
                };
                if ui.selectable_label(self.selected == i, text).clicked() {
                    self.selected = i;
                }
            }
        });
        ui.separator();
        let i = self.selected.min(LESSONS.len() - 1);
        let lesson = &LESSONS[i];
        ui.heading(format!("{}. {}", i + 1, pick(&lesson.title, lang)));
        ui.label(pick(&lesson.intro, lang));
        ui.add_space(6.0);
        let current = self.active.filter(|a| a.lesson == i).map(|a| a.step);
        for (k, step) in lesson.steps.iter().enumerate() {
            let (mark, color) = match current {
                Some(c) if k < c => ("✔", OK),
                Some(c) if k == c => ("▶", ACCENT),
                None if self.completed[i] => ("✔", OK),
                _ => ("•", DIM),
            };
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new(mark).color(color).monospace());
                ui.label(
                    RichText::new(format!("{}. {}", k + 1, pick(&step.text, lang)))
                        .color(if Some(k) == current { TEXT } else { DIM }),
                );
            });
        }
        ui.add_space(8.0);
        ui.horizontal(|ui| {
            let label = match current {
                Some(_) => l(lang, "Restart lesson", "Zacznij od nowa"),
                None => l(lang, "Start lesson", "Rozpocznij lekcję"),
            };
            if ui
                .button(RichText::new(label).strong().size(15.0))
                .clicked()
            {
                self.start(i, total_runs);
            }
            if let Some(a) = self.active.filter(|a| a.lesson != i) {
                ui.label(
                    RichText::new(format!(
                        "{} {}",
                        l(lang, "In progress: lesson", "W toku: lekcja"),
                        a.lesson + 1
                    ))
                    .color(DIM),
                );
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tunerlab_core::{DynoParams, EngineSim, EngineSpec, LoadModel};

    /// Drives `sim` for `seconds` in 1/60 s frames.
    fn run(sim: &mut EngineSim, seconds: f32) {
        for _ in 0..(seconds * 60.0) as usize {
            sim.tick(1.0 / 60.0);
        }
    }

    /// Applies `f` to every cell of `t` inside the region.
    fn edit<const X: usize, const Y: usize>(
        t: &mut Table2D<X, Y>,
        x: (f32, f32),
        y: (f32, f32),
        f: impl Fn(f32) -> f32,
    ) {
        let (xs, ys) = (t.x_axis, t.y_axis);
        for (iy, &yv) in ys.iter().enumerate() {
            for (ix, &xv) in xs.iter().enumerate() {
                if (x.0..=x.1).contains(&xv) && (y.0..=y.1).contains(&yv) {
                    t.values[iy][ix] = f(t.values[iy][ix]);
                }
            }
        }
    }

    struct Scene {
        sim: EngineSim,
        base: Calibration,
        tab: Tab,
        map_kind: MapKind,
        load: LoadKind,
        octane: f32,
        new_runs: u32,
    }

    impl Scene {
        fn new(spec: EngineSpec) -> Self {
            let sim = EngineSim::with_base_calibration(spec, 3).unwrap();
            let base = *sim.calibration();
            Self {
                sim,
                base,
                tab: Tab::Dashboard,
                map_kind: MapKind::Ve,
                load: LoadKind::Neutral,
                octane: 95.0,
                new_runs: 0,
            }
        }

        fn check(&self, lesson: usize, step: usize) -> bool {
            let ctx = Ctx {
                t: self.sim.telemetry(),
                ignition: self.sim.controls().ignition_on,
                cal: self.sim.calibration(),
                base: &self.base,
                turbo: self.sim.spec().turbo.is_some(),
                tab: self.tab,
                map_kind: self.map_kind,
                load: self.load,
                octane: self.octane,
                new_runs: self.new_runs,
                case_open: false,
                case_closed: false,
            };
            (LESSONS[lesson].steps[step].done.unwrap())(&ctx)
        }

        fn start_warm(&mut self) {
            self.sim.prewarm();
            self.sim.update_controls(|c| {
                c.ignition_on = true;
                c.starter = true;
            });
            run(&mut self.sim, 1.5);
            self.sim.update_controls(|c| c.starter = false);
            run(&mut self.sim, 20.0);
        }
    }

    #[test]
    fn every_lesson_has_bilingual_text() {
        for lesson in LESSONS {
            assert!(lesson.title.iter().all(|s| !s.is_empty()));
            assert!(lesson.intro.iter().all(|s| !s.is_empty()));
            for step in lesson.steps {
                assert!(step.text.iter().all(|s| !s.is_empty()));
                assert!(step.hint.iter().all(|s| !s.is_empty()));
            }
        }
    }

    #[test]
    fn first_start_lesson_completes_on_the_simulation() {
        let mut s = Scene::new(EngineSpec::naturally_aspirated_2l());
        assert!(!s.check(0, 0));
        s.sim.update_controls(|c| {
            c.ignition_on = true;
            c.starter = true;
        });
        run(&mut s.sim, 0.1);
        assert!(s.check(0, 0));
        run(&mut s.sim, 1.5);
        s.sim.update_controls(|c| c.starter = false);
        run(&mut s.sim, 2.0);
        assert!(s.check(0, 1), "engine should run");
        assert!(s.check(0, 2), "cold idle {:.0} rpm", s.sim.telemetry().rpm);
        s.sim.prewarm();
        run(&mut s.sim, 1.0);
        assert!(s.check(0, 3), "warm-up");
        run(&mut s.sim, 40.0);
        assert!(s.check(0, 4), "closed loop");
        s.sim.update_controls(|c| c.pedal = 1.0);
        run(&mut s.sim, 1.5);
        assert!(
            s.check(0, 5),
            "MAP {:.0}",
            s.sim.telemetry().sensors.map_kpa
        );
    }

    #[test]
    fn lean_ve_drives_the_trims_positive_and_restore_clears_the_check() {
        let mut s = Scene::new(EngineSpec::naturally_aspirated_2l());
        s.start_warm();
        run(&mut s.sim, 30.0);
        assert!(s.check(1, 0), "closed loop at warm idle");
        s.tab = Tab::Maps;
        assert!(s.check(1, 2));
        s.sim
            .update_calibration(|cal| edit(&mut cal.ve, VE_IDLE.0, VE_IDLE.1, |v| v - 0.10))
            .unwrap();
        assert!(s.check(1, 3));
        let mut reached = false;
        for _ in 0..60 {
            run(&mut s.sim, 1.0);
            if s.check(1, 4) {
                reached = true;
                break;
            }
        }
        let t = s.sim.telemetry();
        assert!(reached, "trims {:+.1}/{:+.1}", t.stft_pct, t.ltft_pct);
        let base = s.base;
        s.sim.update_calibration(|cal| cal.ve = base.ve).unwrap();
        assert!(s.check(1, 5));
    }

    #[test]
    fn map_edit_checks_follow_the_calibration() {
        let mut s = Scene::new(EngineSpec::naturally_aspirated_2l());
        assert!(!s.check(2, 1));
        s.sim
            .update_calibration(|cal| {
                edit(&mut cal.ignition, SPARK_WOT.0, SPARK_WOT.1, |v| v + 5.0)
            })
            .unwrap();
        assert!(s.check(2, 1));
        assert!(!s.check(2, 5), "not restored yet");
        let base = s.base;
        s.sim
            .update_calibration(|cal| cal.ignition = base.ignition)
            .unwrap();
        assert!(s.check(2, 5));

        assert!(!s.check(3, 1));
        let set = |s: &mut Scene, v: f32| {
            s.sim
                .update_calibration(|cal| {
                    edit(&mut cal.lambda_target, LAMBDA_WOT.0, LAMBDA_WOT.1, |_| v)
                })
                .unwrap();
        };
        set(&mut s, 0.95);
        assert!(s.check(3, 1));
        set(&mut s, 0.80);
        assert!(s.check(3, 3));
        assert!(!s.check(3, 5));
    }

    #[test]
    fn turbo_lesson_sees_boost_on_the_dyno_and_the_boost_map_edit() {
        let mut s = Scene::new(EngineSpec::turbocharged_2l());
        assert!(s.check(4, 0));
        s.start_warm();
        assert!(s.check(4, 1), "turbo warm and running");
        s.load = LoadKind::Dyno;
        s.sim.update_controls(|c| {
            c.load = LoadModel::Dyno(DynoParams {
                target_rpm: 3500.0,
                ..DynoParams::default()
            });
            c.pedal = 1.0;
        });
        run(&mut s.sim, 5.0);
        let t = s.sim.telemetry();
        assert!(
            s.check(4, 2),
            "boost {:.0} kPa at {:.0} rpm",
            t.sensors.map_kpa - t.sensors.baro_kpa,
            t.rpm
        );
        s.sim
            .update_calibration(|cal| {
                edit(&mut cal.boost.target_kpa, BOOST_WOT.0, BOOST_WOT.1, |v| {
                    v - 30.0
                })
            })
            .unwrap();
        assert!(s.check(4, 3));
        s.new_runs = 1;
        assert!(s.check(4, 4));
        assert!(!s.check(4, 5));
    }

    #[test]
    fn manual_steps_advance_with_next_and_lessons_complete() {
        let mut c = Course::default();
        c.start(1, 0);
        c.advance(0);
        assert_eq!(c.active.unwrap().step, 1);
        assert!(LESSONS[1].steps[1].done.is_none());
        for _ in 1..LESSONS[1].steps.len() {
            c.advance(0);
        }
        assert!(c.active.is_none());
        assert!(c.completed[1]);
    }

    #[test]
    fn a_check_must_hold_before_the_step_counts() {
        let s = Scene::new(EngineSpec::naturally_aspirated_2l());
        let mut c = Course::default();
        c.start(0, 0);
        let ctx_ignition = |on: bool, dt: f32, c: &mut Course| {
            let ctx = Ctx {
                t: s.sim.telemetry(),
                ignition: on,
                cal: s.sim.calibration(),
                base: &s.base,
                turbo: false,
                tab: Tab::Dashboard,
                map_kind: MapKind::Ve,
                load: LoadKind::Neutral,
                octane: 95.0,
                new_runs: 0,
                case_open: false,
                case_closed: false,
            };
            c.update(&ctx, dt, 0);
        };
        ctx_ignition(true, 0.3, &mut c);
        ctx_ignition(false, 0.3, &mut c);
        ctx_ignition(true, 0.3, &mut c);
        assert_eq!(c.active.unwrap().step, 0, "interrupted hold must restart");
        ctx_ignition(true, 0.4, &mut c);
        assert_eq!(c.active.unwrap().step, 1);
    }
}

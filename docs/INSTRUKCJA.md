# TunerLAB — instrukcja pierwszej wersji

## Uruchomienie

**Gotowy plik dla Windows:** GitHub → zakładka
[Actions → CI](https://github.com/Hunter-zip/TunerLAB/actions/workflows/ci.yml) → ostatnie
zielone uruchomienie → sekcja *Artifacts* → `tunerlab-windows-x64` (zip z `tunerlab.exe`
i instrukcjami). Po rozpakowaniu wystarczy uruchomić `tunerlab.exe`. Wersje oznaczone tagiem
`v*` trafiają też do [Releases](https://github.com/Hunter-zip/TunerLAB/releases).

**Ze źródeł** (Rust 1.95 lub nowszy):

```bash
git clone https://github.com/Hunter-zip/TunerLAB.git
cd TunerLAB
cargo run --release -p tunerlab-app
```

* **Windows 10/11:** gotowy plik `tunerlab.exe` nie wymaga instalacji ani dodatkowych bibliotek
  (potrzebny sterownik OpenGL 2+, obecny w każdym systemie z kartą graficzną).
* **Linux:** sterownik OpenGL (Mesa) oraz w X11 `libxkbcommon-x11`
  (`sudo apt install libxkbcommon-x11-0 libgl1`).

## Ekran

| Obszar | Zawartość |
|---|---|
| Górny pasek | wybór silnika (2.0 wolnossący / 2.0 turbo), zakładki, **Widok**: *Czujniki ECU* (to, co widzi datalogger) lub *Instruktor (prawda)* (rzeczywiste wartości fizyczne, MBT, ciśnienie w cylindrach), tempo symulacji 1×/2×/5×, pauza, język PL/EN |
| Lewy panel | stacyjka, rozrusznik, pedał gazu, obciążenie: bez obciążenia / hamownia / samochód (bieg, sprzęgło, hamulec, nachylenie), otoczenie (temperatura, ciśnienie, paliwo RON, obciążenie elektryczne), warsztat (rozgrzej, wystudź, napraw, reset) |
| Prawy panel | stan silnika, ostrzeżenia instruktora, zapisane kody usterek (DTC) z kasowaniem, stan podzespołów |
| **Kurs** | 6 lekcji krok po kroku (pierwsze uruchomienie, mapa VE i korekty paliwa, zapłon i stuk, mieszanka pod obciążeniem, turbo i doładowanie, diagnostyka); każdy krok zalicza się automatycznie, gdy go wykonasz, a bieżące zadanie widać na pasku na dole każdej zakładki |
| **Pulpit** | obrotomierz, MAP, lambda z wartością docelową, temperatury i ciśnienie oleju, kafelki (moment, moc, zapłon, korekta stukowa, wtrysk, korekty paliwa, przepustnica, doładowanie w barach…), tabela cylindrów, wykresy z 30 s |
| **Mapy** | tablice VE, zapłonu, docelowej lambdy i (turbo) doładowania; zaznaczanie myszą, zmiana o krok (+/−), ustawianie wartości, przywracanie mapy bazowej; kropka pokazuje bieżący punkt pracy |
| **Hamownia** | pomiar mocy przy pełnym gazie na kopii bieżącego silnika (kalibracja, paliwo, ukryte usterki, zużycie); krzywe momentu i mocy porównane z poprzednim pomiarem, każdy opisany silnikiem i paliwem |
| **Engine Autopsy** | ukryta usterka do zdiagnozowania (17 rodzajów) |
| **Dziennik** | zdarzenia z czasem: zmiany stanu, ostrzeżenia, kody, uszkodzenia |

## Klawisze

| Klawisz | Akcja |
|---|---|
| K | stacyjka |
| S (przytrzymaj) | rozrusznik |
| W (przytrzymaj) | gaz do dechy |
| strzałki góra/dół | pedał ±5 % |
| 0–6 | bieg (przełącza na tryb samochodu) |
| C (przytrzymaj) | sprzęgło wciśnięte |
| Spacja (przytrzymaj) | hamulec |
| + / − | zmiana zaznaczonych komórek mapy o krok |

## Kurs

Zakładka **Kurs** → wybierz lekcję → *Rozpocznij lekcję*. Na dole okna pojawi się pasek z bieżącym
zadaniem i podpowiedzią; krok zalicza się sam, gdy warunek jest spełniony przez chwilę (np. silnik
pracuje, korekty paliwa przekroczyły +6 %, zrobiłeś nowy pomiar na hamowni). Kroki do przeczytania
potwierdzasz przyciskiem *Dalej*; każdy krok można pominąć.

## Scenariusz demo (ok. 10 minut)

1. **Zimny rozruch.** K, przytrzymaj S ok. 1 s. Obroty biegu jałowego ok. 1100/min, sonda
   lambda gotowa po kilkunastu sekundach, *Pętla otwarta* → *Pętla zamknięta* po rozgrzaniu.
   Przycisk **Rozgrzej** skraca czekanie.
2. **Gaz.** Przytrzymaj W — silnik dochodzi do ogranicznika obrotów, pojawia się ostrzeżenie
   o przekroczonych obrotach; po puszczeniu odcięcie paliwa (lambda skacze, wtrysk 0 ms).
3. **Hamownia.** Zakładka *Hamownia* → *Pomiar mocy*: ok. 190 N·m / 107 kW (wolnossący).
4. **Strojenie zapłonu.** *Mapy* → *Wyprzedzenie zapłonu*: zaznacz obszar 3200–5000 obr/min
   przy pełnym obciążeniu (88–105 kPa) i dodaj 4–6° (+). Ponowny pomiar na hamowni: moment
   praktycznie nie rośnie (mapa bazowa jest już przy MBT i granicy stuku), w tabeli pomiaru
   pojawia się stuk (czerwony, ok. 1 bar), a ECU cofa zapłon (kafelek *Korekta stukowa*,
   tabela cylindrów). Na paliwie 91 RON stuk i cofanie są wyraźnie większe — przełącz
   *Paliwo (RON)*. Wniosek dla kursanta: więcej zapłonu ≠ więcej mocy.
5. **Mieszanka.** *Docelowa lambda* przy pełnym obciążeniu: zubożenie do 0.95 (z 0.84) daje
   kilka N·m więcej, ale podnosi temperaturę spalin i tłoków; wzbogacenie do 0.78 obniża EGT
   kosztem momentu. W widoku *Instruktor* widać, że rzeczywista lambda przy pełnym gazie
   jest o kilka procent bogatsza od docelowej — bazowa mapa VE nie jest idealna i jej
   dopasowanie to zadanie dla tunera (pętla zamknięta koryguje tylko obciążenia częściowe).
6. **Turbo.** Wybierz *2.0 turbo*: doładowanie ok. 0.8 bar, ok. 297 N·m / 154 kW; mapa
   *Docelowe doładowanie*.
7. **Engine Autopsy.** *Nowy przypadek*, jazda (np. hamownia 2500 obr/min, pedał 30 %),
   obserwacja korekt paliwa, kodów, dziennika i tabeli cylindrów, wybór diagnozy → *Sprawdź*.
8. **Uszkodzenie.** Skrajne wyprzedzenie zapłonu (+20° przy pełnym obciążeniu) na paliwie
   91 RON utrzymywane na hamowni: mimo cofania zapłonu przez ECU stan tłoków spada o ok.
   25 % w 2 minuty (prawy panel *Stan podzespołów*); *Napraw* przywraca silnik.

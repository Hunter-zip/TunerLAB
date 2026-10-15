//! Procedural engine synthesizer: turns scheduled combustion sounds and the engine's
//! operating point into a mono signal.
//!
//! Pure DSP without audio-device types: the output callback drives it in the app and the
//! unit tests drive it directly. Everything is fixed-size, so rendering never allocates and
//! its cost per sample is bounded by [`MAX_VOICES`].

use std::f32::consts::TAU;

use tunerlab_core::engine_sim::events::CylinderEvent;

/// Exhaust pulses and pops that can sound at once; when full, the quietest is replaced.
pub const MAX_VOICES: usize = 64;
/// Knock quieter than this is inaudible over the running engine \[bar\].
pub const KNOCK_AUDIBLE_BAR: f32 = 0.3;
/// Samples rendered per internal block.
const BLOCK: usize = 256;
/// Notes later than this are dropped instead of played at once, so a stalled frame does not
/// release a burst \[s\].
const MAX_LATE_S: f32 = 0.1;
/// Notes further ahead than this are not plausible and are dropped \[s\].
const MAX_AHEAD_S: f32 = 2.0;
/// Blow-down pressure of a full-load exhaust pulse on the naturally aspirated engine \[bar\].
const FULL_PULSE_BAR: f32 = 6.0;
/// Smoothing time of the continuous parameters (no zipper noise) \[s\].
const PARAM_TAU_S: f32 = 0.03;
/// Fade time of the master gain (pause, mute, volume) \[s\].
const GAIN_TAU_S: f32 = 0.04;
/// Level of the summed buses before the soft clipper.
const MASTER_LEVEL: f32 = 1.6;
/// Knock ping partials: chamber resonance modes of an ≈ 86 mm bore \[Hz\], decay \[s\], weight.
const PING_MODES: [(f32, f32, f32); 3] = [
    (6300.0, 0.007, 1.0),
    (7150.0, 0.005, 0.7),
    (8050.0, 0.004, 0.5),
];

/// A sound triggered by one combustion event.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Sound {
    /// Exhaust blow-down pulse; `amp` is about 0..1.6, `rpm` sets its length.
    Pulse { amp: f32, rpm: f32 },
    /// Knock: metallic ring of the combustion chamber, `amp` 0..1.
    Ping { amp: f32 },
    /// Afterburn in the exhaust manifold ("pops & bangs"), `amp` 0..1.
    Pop { amp: f32 },
}

/// A sound scheduled at an absolute output sample index.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Note {
    pub at: u64,
    pub sound: Sound,
}

/// Engine operating point behind the continuous sounds; smoothed inside the synthesizer.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Params {
    /// Engine speed \[rpm\].
    pub rpm: f32,
    /// Throttle opening 0..1.
    pub throttle: f32,
    /// Air mass flow into the engine \[g/s\].
    pub air_flow_g_s: f32,
    /// Turbocharger speed \[rpm\] (zero on the naturally aspirated engine).
    pub turbo_rpm: f32,
    /// Boost above ambient pressure \[kPa\].
    pub boost_kpa: f32,
    /// Number of cylinders.
    pub cylinders: u8,
    /// The starter motor is turning the engine.
    pub cranking: bool,
    /// Master gain 0..1 (volume; zero while muted or paused).
    pub gain: f32,
}

/// Message from the UI thread to the synthesizer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    Note(Note),
    Params(Params),
}

/// The sounds of one combustion event with their simulation times \[s\].
///
/// The exhaust pulse and an afterburn pop start at exhaust valve opening, a knock ping at
/// firing TDC. A misfire has no pulse: the gap in the firing rhythm is what the student
/// should hear.
pub fn event_sounds(ev: &CylinderEvent) -> impl Iterator<Item = (f64, Sound)> {
    let evo = ev.time_s + f64::from(finite(ev.time_to_evo_s).max(0.0));
    let tdc = ev.time_s + f64::from(finite(ev.time_to_tdc_s).max(0.0));
    let pulse = (!ev.misfire).then(|| {
        let amp = (finite(ev.blowdown_pressure_bar).max(0.0) / FULL_PULSE_BAR)
            .powf(0.6)
            .min(1.6);
        (
            evo,
            Sound::Pulse {
                amp,
                rpm: finite(ev.rpm),
            },
        )
    });
    let knock = finite(ev.knock_intensity);
    let ping = (!ev.misfire && knock > KNOCK_AUDIBLE_BAR).then(|| {
        let amp = ((knock - KNOCK_AUDIBLE_BAR) / 3.0).clamp(0.1, 1.0);
        (tdc, Sound::Ping { amp })
    });
    // Unburnt mixture can also ignite in the manifold after a misfire.
    let afterburn = finite(ev.afterburn_j);
    let pop = (afterburn > 0.0).then(|| {
        let amp = (afterburn / 40.0).sqrt().clamp(0.3, 1.0);
        (evo + 0.002, Sound::Pop { amp })
    });
    [pulse, ping, pop].into_iter().flatten()
}

fn finite(x: f32) -> f32 {
    if x.is_finite() {
        x
    } else {
        0.0
    }
}

/// Which bus a voice feeds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Bus {
    #[default]
    Exhaust,
    Pop,
    Ping,
}

/// Envelope `amp · (1 − att) · dec` with exponentially decaying `att` and `dec`.
#[derive(Debug, Clone, Copy, Default)]
struct Voice {
    active: bool,
    start: u64,
    bus: Bus,
    amp: f32,
    att: f32,
    att_k: f32,
    dec: f32,
    dec_k: f32,
}

impl Voice {
    fn loudness(&self) -> f32 {
        self.amp * self.dec
    }
}

/// White noise from a 32-bit xorshift generator, in [-1, 1).
#[derive(Debug, Clone, Copy)]
struct Noise(u32);

impl Noise {
    fn next(&mut self) -> f32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        (x >> 8) as f32 / (1u32 << 23) as f32 - 1.0
    }
}

/// RBJ biquad, transposed direct form II.
#[derive(Debug, Clone, Copy, Default)]
struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
}

impl Biquad {
    fn lowpass(sr: f32, hz: f32, q: f32) -> Self {
        let mut f = Self::default();
        f.set_lowpass(sr, hz, q);
        f
    }

    /// Band-pass with 0 dB gain at the centre frequency.
    fn bandpass(sr: f32, hz: f32, q: f32) -> Self {
        let (cos, alpha) = Self::prewarp(sr, hz, q);
        let a0 = 1.0 + alpha;
        Self {
            b0: alpha / a0,
            b1: 0.0,
            b2: -alpha / a0,
            a1: -2.0 * cos / a0,
            a2: (1.0 - alpha) / a0,
            ..Self::default()
        }
    }

    /// Changes the cut-off and keeps the filter state.
    fn set_lowpass(&mut self, sr: f32, hz: f32, q: f32) {
        let (cos, alpha) = Self::prewarp(sr, hz, q);
        let a0 = 1.0 + alpha;
        self.b0 = (1.0 - cos) * 0.5 / a0;
        self.b1 = (1.0 - cos) / a0;
        self.b2 = self.b0;
        self.a1 = -2.0 * cos / a0;
        self.a2 = (1.0 - alpha) / a0;
    }

    fn prewarp(sr: f32, hz: f32, q: f32) -> (f32, f32) {
        let w = TAU * hz.clamp(10.0, 0.45 * sr) / sr;
        (w.cos(), w.sin() / (2.0 * q))
    }

    fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        y
    }

    fn clear(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
    }
}

/// Two-pole resonator ringing at `hz` with decay time constant `tau_s`; an impulse of
/// amplitude 1 rings with a peak of about 1.
#[derive(Debug, Clone, Copy, Default)]
struct Resonator {
    b0: f32,
    c1: f32,
    c2: f32,
    y1: f32,
    y2: f32,
}

impl Resonator {
    fn new(sr: f32, hz: f32, tau_s: f32) -> Self {
        let w = TAU * hz.min(0.45 * sr) / sr;
        let r = (-1.0 / (tau_s * sr)).exp();
        Self {
            b0: w.sin(),
            c1: 2.0 * r * w.cos(),
            c2: -r * r,
            ..Self::default()
        }
    }

    fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.c1 * self.y1 + self.c2 * self.y2;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }

    fn clear(&mut self) {
        self.y1 = 0.0;
        self.y2 = 0.0;
    }
}

/// One-pole DC blocker / high-pass.
#[derive(Debug, Clone, Copy, Default)]
struct HighPass {
    r: f32,
    x1: f32,
    y1: f32,
}

impl HighPass {
    fn new(sr: f32, hz: f32) -> Self {
        Self {
            r: (-TAU * hz / sr).exp(),
            ..Self::default()
        }
    }

    fn process(&mut self, x: f32) -> f32 {
        let y = x - self.x1 + self.r * self.y1;
        self.x1 = x;
        self.y1 = y;
        y
    }

    fn clear(&mut self) {
        self.x1 = 0.0;
        self.y1 = 0.0;
    }
}

/// Smoothed copies of the continuous parameters.
#[derive(Debug, Clone, Copy, Default)]
struct Smoothed {
    gain: f32,
    throttle: f32,
    hiss: f32,
    whistle_amp: f32,
    whistle_hz: f32,
    starter_amp: f32,
    starter_hz: f32,
    compression_hz: f32,
}

/// The synthesizer. Its clock counts rendered samples from 0.
pub struct Synth {
    sr: f32,
    clock: u64,
    voices: [Voice; MAX_VOICES],
    target: Params,
    s: Smoothed,
    param_k: f32,
    gain_k: f32,
    noise: Noise,
    whistle_phase: f32,
    starter_phase: f32,
    gear_phase: f32,
    compression_phase: f32,
    /// Removes the DC of the unipolar pulse train.
    exhaust_dc: HighPass,
    /// Muffler: low-pass that opens up with the throttle.
    muffler: Biquad,
    /// Exhaust pipe resonances.
    pipe: [Biquad; 2],
    pop_hp: HighPass,
    pop_lp: Biquad,
    ping_modes: [Resonator; 3],
    hiss_bp: Biquad,
}

impl Synth {
    pub fn new(sample_rate: f32) -> Self {
        let sr = if sample_rate.is_finite() {
            sample_rate.clamp(8000.0, 384_000.0)
        } else {
            48_000.0
        };
        let smoothing = |tau: f32| 1.0 - (-1.0 / (tau * sr)).exp();
        Self {
            sr,
            clock: 0,
            voices: [Voice::default(); MAX_VOICES],
            target: Params::default(),
            s: Smoothed::default(),
            param_k: smoothing(PARAM_TAU_S),
            gain_k: smoothing(GAIN_TAU_S),
            noise: Noise(0x2545_f491),
            whistle_phase: 0.0,
            starter_phase: 0.0,
            gear_phase: 0.0,
            compression_phase: 0.0,
            exhaust_dc: HighPass::new(sr, 25.0),
            muffler: Biquad::lowpass(sr, 600.0, 0.7),
            pipe: [
                Biquad::bandpass(sr, 85.0, 2.5),
                Biquad::bandpass(sr, 260.0, 4.0),
            ],
            pop_hp: HighPass::new(sr, 400.0),
            pop_lp: Biquad::lowpass(sr, 6000.0, 0.7),
            ping_modes: PING_MODES.map(|(hz, tau, _)| Resonator::new(sr, hz, tau)),
            hiss_bp: Biquad::bandpass(sr, 2500.0, 0.8),
        }
    }

    /// Samples rendered so far: the index of the next sample [`render`](Self::render) writes.
    pub fn clock(&self) -> u64 {
        self.clock
    }

    pub fn apply(&mut self, command: Command) {
        match command {
            Command::Note(note) => self.schedule(note),
            Command::Params(p) => self.set_params(p),
        }
    }

    pub fn set_params(&mut self, p: Params) {
        let clean = |x: f32| finite(x).max(0.0);
        self.target = Params {
            rpm: clean(p.rpm),
            throttle: clean(p.throttle).min(1.0),
            air_flow_g_s: clean(p.air_flow_g_s),
            turbo_rpm: clean(p.turbo_rpm),
            boost_kpa: clean(p.boost_kpa),
            cylinders: p.cylinders,
            cranking: p.cranking,
            gain: clean(p.gain).min(1.0),
        };
    }

    /// Queues a note. A note that is slightly late starts at once, a very late or
    /// implausibly early one is dropped.
    pub fn schedule(&mut self, note: Note) {
        let late = (MAX_LATE_S * self.sr) as u64;
        let ahead = (MAX_AHEAD_S * self.sr) as u64;
        if note.at.saturating_add(late) < self.clock || note.at > self.clock.saturating_add(ahead) {
            return;
        }
        let ms = |t: f32| (-1.0 / (t * 0.001 * self.sr)).exp();
        let (bus, amp, att, att_k, dec_k) = match note.sound {
            Sound::Pulse { amp, rpm } => {
                // Blow-down lasts ≈ 60° of crank: shorter pulses at higher speed.
                let tau_ms = (3000.0 / rpm.max(1.0)).clamp(0.7, 4.0);
                (Bus::Exhaust, amp, 1.0, ms(0.3), ms(tau_ms))
            }
            Sound::Pop { amp } => (Bus::Pop, amp, 1.0, ms(0.1), ms(4.0)),
            // A single impulse into the ringing chamber modes.
            Sound::Ping { amp } => (Bus::Ping, amp, 0.0, 0.0, 0.0),
        };
        let amp = finite(amp).clamp(0.0, 2.0);
        if amp <= 0.0 {
            return;
        }
        let voice = Voice {
            active: true,
            start: note.at.max(self.clock),
            bus,
            amp,
            att,
            att_k,
            dec: 1.0,
            dec_k,
        };
        let slot = match self.voices.iter().position(|v| !v.active) {
            Some(free) => free,
            None => {
                let quietest = self
                    .voices
                    .iter()
                    .enumerate()
                    .min_by(|a, b| a.1.loudness().total_cmp(&b.1.loudness()));
                match quietest {
                    Some((i, _)) => i,
                    None => return,
                }
            }
        };
        self.voices[slot] = voice;
    }

    /// Renders the next `out.len()` samples, each within [-1, 1].
    pub fn render(&mut self, out: &mut [f32]) {
        for chunk in out.chunks_mut(BLOCK) {
            self.render_block(chunk);
        }
    }

    fn render_block(&mut self, out: &mut [f32]) {
        let n = out.len();
        let mut exhaust = [0.0_f32; BLOCK];
        let mut pop = [0.0_f32; BLOCK];
        let mut ping = [0.0_f32; BLOCK];
        let end = self.clock + n as u64;
        let noise = &mut self.noise;
        for v in self.voices.iter_mut().filter(|v| v.active && v.start < end) {
            let first = (v.start.saturating_sub(self.clock)) as usize;
            for i in first..n {
                let env = v.amp * (1.0 - v.att) * v.dec;
                v.att *= v.att_k;
                v.dec *= v.dec_k;
                match v.bus {
                    // Blow-down pulse with turbulent noise riding on it.
                    Bus::Exhaust => exhaust[i] += env * (0.7 + 0.6 * noise.next()),
                    Bus::Pop => pop[i] += env * noise.next(),
                    Bus::Ping => ping[i] += env,
                }
                if v.dec < 1e-4 {
                    v.active = false;
                    break;
                }
            }
        }

        let t = self.target;
        let s = &mut self.s;
        // Per-block targets of the continuous sounds.
        let hiss = 0.10 * (t.air_flow_g_s / 170.0).min(1.5).sqrt() * (0.25 + 0.75 * t.throttle);
        let boost = (t.boost_kpa / 80.0).min(1.2);
        let spin = (t.turbo_rpm / 200_000.0).min(1.0);
        let whistle_amp = 0.03 * (0.8 * boost + 0.2 * spin) * spin.min(0.3) / 0.3;
        let whistle_hz = 400.0 + 0.04 * t.turbo_rpm.min(220_000.0);
        let crank_rpm = t.rpm.min(600.0);
        let starter_amp = if t.cranking { 0.12 } else { 0.0 };
        let starter_hz = 40.0 + 0.8 * crank_rpm;
        let compression_hz = f32::from(t.cylinders.max(1)) * crank_rpm / 120.0;
        self.muffler
            .set_lowpass(self.sr, 500.0 + 1000.0 * s.throttle, 0.7);

        let (k, kg, dt) = (self.param_k, self.gain_k, 1.0 / self.sr);
        let mut broken = false;
        let wrap = |phase: &mut f32, hz: f32| {
            *phase += hz * dt;
            *phase -= phase.floor();
            *phase * TAU
        };
        for (i, y) in out.iter_mut().enumerate() {
            s.gain += (t.gain - s.gain) * kg;
            s.throttle += (t.throttle - s.throttle) * k;
            s.hiss += (hiss - s.hiss) * k;
            s.whistle_amp += (whistle_amp - s.whistle_amp) * k;
            s.whistle_hz += (whistle_hz - s.whistle_hz) * k;
            s.starter_amp += (starter_amp - s.starter_amp) * k;
            s.starter_hz += (starter_hz - s.starter_hz) * k;
            s.compression_hz += (compression_hz - s.compression_hz) * k;

            let e = self.exhaust_dc.process(exhaust[i] + 0.4 * pop[i]);
            let body = 1.2 * self.muffler.process(e)
                + 1.0 * self.pipe[0].process(e)
                + 0.4 * self.pipe[1].process(e);
            let crack = self.pop_lp.process(self.pop_hp.process(pop[i]));
            let mut knock = 0.0;
            for (m, &(_, _, w)) in self.ping_modes.iter_mut().zip(&PING_MODES) {
                knock += w * m.process(ping[i]);
            }
            let hiss = s.hiss * self.hiss_bp.process(self.noise.next());
            let whistle = s.whistle_amp * wrap(&mut self.whistle_phase, s.whistle_hz).sin();
            let mut starter = 0.0;
            if s.starter_amp > 1e-5 {
                let p = wrap(&mut self.starter_phase, s.starter_hz);
                let g = wrap(&mut self.gear_phase, 7.5 * s.starter_hz);
                // Each compression stroke loads the starter: the whine pulses with it.
                let c = wrap(&mut self.compression_phase, s.compression_hz);
                let tone =
                    p.sin() + 0.45 * (2.0 * p).sin() + 0.25 * (3.0 * p).sin() + 0.2 * g.sin();
                starter = s.starter_amp * (0.6 + 0.4 * c.sin()) * tone;
            }
            let mix = (body + 0.8 * crack + 0.5 * knock + hiss + whistle + starter)
                * MASTER_LEVEL
                * s.gain;
            let v = mix.tanh().clamp(-1.0, 1.0);
            *y = if v.is_finite() {
                v
            } else {
                broken = true;
                0.0
            };
        }
        if broken {
            self.recover();
        }
        self.clock = end;
    }

    /// Clears every filter and voice after a non-finite sample, so one bad value cannot
    /// keep the output silent or noisy.
    fn recover(&mut self) {
        self.exhaust_dc.clear();
        self.muffler.clear();
        for f in &mut self.pipe {
            f.clear();
        }
        self.pop_hp.clear();
        self.pop_lp.clear();
        for m in &mut self.ping_modes {
            m.clear();
        }
        self.hiss_bp.clear();
        for v in &mut self.voices {
            v.active = false;
        }
        self.s = Smoothed {
            gain: self.s.gain,
            ..Smoothed::default()
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f32 = 48_000.0;

    fn loud() -> Params {
        Params {
            gain: 1.0,
            cylinders: 4,
            ..Params::default()
        }
    }

    fn event(misfire: bool) -> CylinderEvent {
        CylinderEvent {
            time_s: 1.0,
            time_to_tdc_s: 0.010,
            time_to_evo_s: 0.0175,
            cylinder: 0,
            rpm: 3000.0,
            peak_pressure_bar: 55.0,
            misfire,
            indicated_work_j: 620.0,
            exhaust_temp_k: 1360.0,
            blowdown_pressure_bar: 5.7,
            ..CylinderEvent::default()
        }
    }

    fn render(synth: &mut Synth, samples: usize) -> Vec<f32> {
        let mut out = vec![0.0; samples];
        synth.render(&mut out);
        out
    }

    fn energy(x: &[f32]) -> f32 {
        x.iter().map(|v| v * v).sum()
    }

    #[test]
    fn silence_in_silence_out() {
        let mut synth = Synth::new(SR);
        synth.set_params(loud());
        let out = render(&mut synth, 9600);
        assert!(out.iter().all(|v| *v == 0.0));
        assert_eq!(synth.clock(), 9600);
    }

    #[test]
    fn one_pulse_sounds_at_its_sample() {
        let mut synth = Synth::new(SR);
        synth.set_params(loud());
        // Let the master gain settle first.
        render(&mut synth, 4800);
        let at = 4800 + 2400;
        for (_, sound) in event_sounds(&event(false)) {
            synth.schedule(Note { at, sound });
        }
        let out = render(&mut synth, 9600);
        let before = &out[..2400];
        let window = &out[2400..2400 + (0.04 * SR) as usize];
        assert_eq!(energy(before), 0.0);
        assert!(energy(window) > 1e-3, "pulse energy {}", energy(window));
        assert!(energy(window) > 0.8 * energy(&out));
        assert!(out.iter().all(|v| v.is_finite() && v.abs() <= 1.0));
    }

    #[test]
    fn misfire_has_no_pulse() {
        let sounds: Vec<_> = event_sounds(&event(true)).collect();
        assert!(sounds
            .iter()
            .all(|(_, s)| !matches!(s, Sound::Pulse { .. })));
        let mut synth = Synth::new(SR);
        synth.set_params(loud());
        for (_, sound) in sounds {
            synth.schedule(Note { at: 100, sound });
        }
        assert_eq!(energy(&render(&mut synth, 4800)), 0.0);
    }

    #[test]
    fn knock_and_afterburn_add_their_sounds() {
        let ev = CylinderEvent {
            knock_intensity: 2.5,
            afterburn_j: 30.0,
            ..event(false)
        };
        let sounds: Vec<_> = event_sounds(&ev).collect();
        assert_eq!(sounds.len(), 3);
        let ping_time = sounds
            .iter()
            .find_map(|(t, s)| matches!(s, Sound::Ping { .. }).then_some(*t));
        assert_eq!(ping_time, Some(1.0 + f64::from(0.010_f32)));
        assert!(sounds.iter().any(|(_, s)| matches!(s, Sound::Pop { .. })));
        // Light knock below the audible threshold rings nothing.
        let quiet = CylinderEvent {
            knock_intensity: 0.1,
            ..event(false)
        };
        assert_eq!(event_sounds(&quiet).count(), 1);
    }

    #[test]
    fn dense_stream_stays_bounded() {
        let mut synth = Synth::new(SR);
        synth.set_params(Params {
            rpm: 7000.0,
            throttle: 1.0,
            air_flow_g_s: 400.0,
            turbo_rpm: 210_000.0,
            boost_kpa: 150.0,
            cylinders: 8,
            cranking: true,
            gain: 1.0,
        });
        let mut out = vec![0.0; 512];
        let mut peak = 0.0_f32;
        for block in 0..400u64 {
            for j in 0..40u64 {
                let at = 2048 + block * 512 + j * 13;
                synth.schedule(Note {
                    at,
                    sound: Sound::Pulse {
                        amp: 1.6,
                        rpm: 7000.0,
                    },
                });
                synth.schedule(Note {
                    at,
                    sound: Sound::Pop { amp: 1.0 },
                });
                synth.schedule(Note {
                    at,
                    sound: Sound::Ping { amp: 1.0 },
                });
            }
            synth.render(&mut out);
            for v in &out {
                assert!(v.is_finite() && v.abs() <= 1.0, "sample {v}");
                peak = peak.max(v.abs());
            }
        }
        assert!(peak > 0.5);
    }

    #[test]
    fn garbage_input_never_yields_nan() {
        let mut synth = Synth::new(f32::NAN);
        synth.set_params(Params {
            rpm: f32::NAN,
            throttle: f32::INFINITY,
            air_flow_g_s: -5.0,
            turbo_rpm: f32::NAN,
            boost_kpa: f32::INFINITY,
            cylinders: 0,
            cranking: true,
            gain: f32::NAN,
        });
        synth.schedule(Note {
            at: 0,
            sound: Sound::Pulse {
                amp: f32::NAN,
                rpm: f32::NAN,
            },
        });
        let ev = CylinderEvent {
            time_to_evo_s: f32::NAN,
            blowdown_pressure_bar: f32::INFINITY,
            rpm: f32::NAN,
            ..event(false)
        };
        for (t, sound) in event_sounds(&ev) {
            assert!(t.is_finite());
            synth.schedule(Note { at: 10, sound });
        }
        assert!(render(&mut synth, 4800).iter().all(|v| v.is_finite()));
    }

    #[test]
    fn late_notes_play_at_once_and_stale_ones_are_dropped() {
        let mut synth = Synth::new(SR);
        synth.set_params(loud());
        render(&mut synth, 9600);
        let pulse = Sound::Pulse {
            amp: 1.0,
            rpm: 3000.0,
        };
        // 20 ms late: still played, from the next sample on.
        synth.schedule(Note {
            at: 9600 - 960,
            sound: pulse,
        });
        assert!(energy(&render(&mut synth, 960)) > 1e-4);
        render(&mut synth, 9600);
        // 1 s late: dropped (only the filter tail of the first pulse remains).
        synth.schedule(Note {
            at: 100,
            sound: pulse,
        });
        assert!(energy(&render(&mut synth, 4800)) < 1e-12);
    }

    #[test]
    fn zero_gain_fades_to_silence() {
        let mut synth = Synth::new(SR);
        synth.set_params(Params {
            air_flow_g_s: 100.0,
            throttle: 1.0,
            ..loud()
        });
        assert!(energy(&render(&mut synth, 4800)) > 0.0);
        synth.set_params(Params {
            gain: 0.0,
            air_flow_g_s: 100.0,
            throttle: 1.0,
            ..loud()
        });
        let out = render(&mut synth, 24_000);
        assert!(out[20_000..].iter().all(|v| v.abs() < 1e-4));
    }
}

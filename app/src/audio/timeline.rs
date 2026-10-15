//! Mapping from simulation time to the output's sample clock.
//!
//! The UI thread ticks the simulation once per frame, so combustion events arrive in
//! frame-sized bursts. Each event carries its own simulation time; the timeline turns it
//! into an absolute sample index a fixed latency ahead of the audio clock, so the firing
//! rhythm survives frame jitter. The mapping follows the clocks slowly and only jumps on a
//! real discontinuity (pause, reset, a stalled window).

/// Default distance of "now" ahead of the rendered audio \[s\]; must cover one frame plus
/// one output buffer.
pub const LATENCY_S: f64 = 0.1;
/// A mapping error larger than this re-anchors the timeline instead of slewing it \[s\].
const RESYNC_S: f64 = 0.15;
/// Fraction of the mapping error corrected per frame (≈ 0.3 s time constant at 60 fps).
const SLEW: f64 = 0.05;

/// Linear map anchored at one (simulation time, sample) pair.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Anchor {
    sim_s: f64,
    sample: f64,
    speed: f64,
}

impl Anchor {
    fn map(&self, sim_s: f64, sample_rate: f64) -> f64 {
        self.sample + (sim_s - self.sim_s) / self.speed * sample_rate
    }
}

#[derive(Debug, Clone)]
pub struct Timeline {
    sample_rate: f64,
    latency_s: f64,
    anchor: Option<Anchor>,
}

impl Timeline {
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate: sample_rate.max(1.0),
            latency_s: LATENCY_S,
            anchor: None,
        }
    }

    /// Sets the latency (at least [`LATENCY_S`]); the mapping follows on the next updates.
    pub fn set_latency(&mut self, latency_s: f64) {
        if latency_s.is_finite() {
            self.latency_s = latency_s.max(LATENCY_S);
        }
    }

    /// Forgets the mapping (new simulation, reset); the next update re-anchors.
    pub fn reset(&mut self) {
        self.anchor = None;
    }

    /// Aligns the mapping with this frame: `sim_now` is the simulation time after the tick,
    /// `audio_now` the output's sample clock and `speed` the simulation speed (> 0).
    pub fn update(&mut self, sim_now: f64, audio_now: u64, speed: f64) {
        let speed = if speed.is_finite() && speed > 0.0 {
            speed
        } else {
            1.0
        };
        let sr = self.sample_rate;
        let target = audio_now as f64 + self.latency_s * sr;
        let fresh = Anchor {
            sim_s: sim_now,
            sample: target,
            speed,
        };
        self.anchor = Some(match self.anchor {
            Some(a) => {
                // A speed change pivots the mapping at the current time to stay continuous.
                let a = if a.speed == speed {
                    a
                } else {
                    Anchor {
                        sim_s: sim_now,
                        sample: a.map(sim_now, sr),
                        speed,
                    }
                };
                let error = target - a.map(sim_now, sr);
                // Re-anchor before slewing could map notes behind the audio clock: the
                // window stays within half the latency.
                let window = RESYNC_S.min(0.5 * self.latency_s) * sr;
                if error.is_finite() && error.abs() <= window {
                    Anchor {
                        sample: a.sample + SLEW * error,
                        ..a
                    }
                } else {
                    fresh
                }
            }
            None => fresh,
        });
    }

    /// Output sample at which simulation time `sim_s` should sound, once anchored.
    pub fn sample_at(&self, sim_s: f64) -> Option<u64> {
        let s = self.anchor?.map(sim_s, self.sample_rate);
        s.is_finite().then(|| s.max(0.0).round() as u64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f64 = 48_000.0;
    const LATENCY: u64 = (LATENCY_S * SR) as u64;

    #[test]
    fn maps_sim_time_at_normal_speed() {
        let mut tl = Timeline::new(SR);
        assert_eq!(tl.sample_at(1.0), None);
        tl.update(10.0, 48_000, 1.0);
        assert_eq!(tl.sample_at(10.0), Some(48_000 + LATENCY));
        // 10 ms of simulation = 480 samples.
        assert_eq!(tl.sample_at(10.01), Some(48_000 + LATENCY + 480));
        assert_eq!(tl.sample_at(9.99), Some(48_000 + LATENCY - 480));
    }

    #[test]
    fn fast_forward_compresses_sim_time() {
        let mut tl = Timeline::new(SR);
        tl.update(10.0, 48_000, 5.0);
        // At 5× speed 50 ms of simulation play in 10 ms.
        assert_eq!(tl.sample_at(10.05), Some(48_000 + LATENCY + 480));
        // Changing speed keeps the current instant in place.
        tl.update(10.5, 48_000 + 4800, 1.0);
        let now = tl.sample_at(10.5).unwrap_or(0);
        assert_eq!(tl.sample_at(10.51), Some(now + 480));
    }

    #[test]
    fn frame_jitter_is_smoothed() {
        let mut tl = Timeline::new(SR);
        let mut sim = 0.0;
        tl.update(sim, 0, 1.0);
        let mut previous = 0.0;
        for frame in 0..600u64 {
            // 60 fps frames whose audio clock moves in 512-sample buffer steps.
            sim += 1.0 / 60.0;
            let ideal = (sim * SR) as u64;
            tl.update(sim, ideal - ideal % 512, 1.0);
            let expected = sim * SR + LATENCY_S * SR;
            let offset = tl.sample_at(sim).unwrap_or(0) as f64 - expected;
            // Within one buffer of the ideal, and no frame moves the mapping by more than
            // a small fraction of the buffer step.
            assert!(offset.abs() < 512.0, "frame {frame}: offset {offset}");
            assert!((offset - previous).abs() < 40.0, "frame {frame}: jump");
            previous = offset;
        }
    }

    /// Statistics of a simulated session: notes that landed behind the audio clock and the
    /// worst deviation of the pulse spacing from the ideal \[samples\].
    #[derive(Debug, Default)]
    struct Stats {
        notes: usize,
        late: usize,
        worst_late: f64,
        worst_spacing_error: f64,
    }

    /// Plays 20 s of 100 Hz pulses through the timeline the way `EngineSound::feed` does:
    /// UI frames of `frame_s` with `frame_jitter_s` of jitter and a `spike_s` frame every
    /// `spike_every` frames; audio callbacks rendering `period` samples every `period`
    /// (± `callback_jitter_s`).
    fn session(
        frame_s: f64,
        frame_jitter_s: f64,
        spike_s: f64,
        spike_every: u64,
        period: u64,
        callback_jitter_s: f64,
    ) -> Stats {
        let mut rng = 0x2545_f491_4f6c_dd1d_u64;
        let mut rand = move || {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            (rng >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
        };
        let period_s = period as f64 / SR;
        let mut tl = Timeline::new(SR);
        let (mut wall, mut sim, mut clock) = (0.0_f64, 0.0_f64, 0u64);
        let mut next_callback = period_s;
        let mut next_event = 0.0_f64;
        let mut peak = 0.0_f64;
        let mut last_at: Option<u64> = None;
        let mut stats = Stats::default();
        let mut frame = 0u64;
        while wall < 20.0 {
            frame += 1;
            let dt = if frame.is_multiple_of(spike_every) {
                spike_s
            } else {
                frame_s + frame_jitter_s * rand()
            };
            wall += dt;
            sim += dt.min(0.1);
            while next_callback <= wall {
                clock += period;
                next_callback += period_s + callback_jitter_s * rand();
            }
            peak = dt.min(0.15).max(peak * 0.99);
            tl.set_latency(2.0 * period_s + peak + 0.01);
            tl.update(sim, clock, 1.0);
            while next_event <= sim {
                if let Some(at) = tl.sample_at(next_event) {
                    stats.notes += 1;
                    if at < clock {
                        stats.late += 1;
                        stats.worst_late = stats.worst_late.max((clock - at) as f64);
                    }
                    if let Some(prev) = last_at {
                        let spacing = at as f64 - prev as f64;
                        stats.worst_spacing_error =
                            stats.worst_spacing_error.max((spacing - 480.0).abs());
                    }
                    last_at = Some(at);
                }
                next_event += 0.01;
            }
        }
        stats
    }

    #[test]
    fn jittery_frames_and_callbacks_keep_the_firing_rhythm() {
        let cases = [
            // 60 fps, WASAPI-like 10 ms periods.
            (
                "60 fps, 10 ms",
                session(1.0 / 60.0, 0.004, 0.035, 90, 480, 0.002),
            ),
            // Busy GUI: 30 fps with 60 ms hiccups.
            (
                "30 fps, 10 ms",
                session(1.0 / 30.0, 0.008, 0.06, 45, 480, 0.002),
            ),
            // Bluetooth-like: 20 ms periods arriving irregularly.
            (
                "60 fps, 20 ms",
                session(1.0 / 60.0, 0.004, 0.035, 90, 960, 0.006),
            ),
        ];
        for (name, s) in cases {
            eprintln!("{name}: {s:?}");
            assert_eq!(s.late, 0, "{name}: {s:?}");
            assert!(s.worst_spacing_error < 0.25 * 480.0, "{name}: {s:?}");
        }
    }

    #[test]
    fn discontinuities_reanchor() {
        let mut tl = Timeline::new(SR);
        tl.update(50.0, 480_000, 1.0);
        // Reset: the simulation clock restarts at zero.
        tl.update(0.0, 481_000, 1.0);
        assert_eq!(tl.sample_at(0.0), Some(481_000 + LATENCY));
        // Resume after a long pause: simulation time stood still, audio ran on.
        tl.update(0.0, 1_000_000, 1.0);
        assert_eq!(tl.sample_at(0.0), Some(1_000_000 + LATENCY));
        tl.reset();
        assert_eq!(tl.sample_at(0.0), None);
    }
}

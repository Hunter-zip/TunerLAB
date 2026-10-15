//! Mapping from simulation time to the output's sample clock.
//!
//! The UI thread ticks the simulation once per frame, so combustion events arrive in
//! frame-sized bursts. Each event carries its own simulation time; the timeline turns it
//! into an absolute sample index a fixed latency ahead of the audio clock, so the firing
//! rhythm survives frame jitter. The mapping follows the clocks slowly and only jumps on a
//! real discontinuity (pause, reset, a stalled window).

/// Default distance of "now" ahead of the rendered audio \[s\]; must cover one frame plus
/// one output buffer.
pub const LATENCY_S: f64 = 0.07;
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
                if error.is_finite() && error.abs() <= RESYNC_S * sr {
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

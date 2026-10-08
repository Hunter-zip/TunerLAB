//! Allocation-free numerical primitives shared by every simulation module.

use core::f32::consts::{PI, TAU};

/// One complete four-stroke cycle spans two crankshaft revolutions: 720° = 4π rad.
pub(crate) const CYCLE_RAD: f32 = 2.0 * TAU;
/// Degrees → radians.
pub(crate) const DEG_TO_RAD: f32 = PI / 180.0;
/// rad/s → rev/min: n = ω · 60 / 2π.
pub(crate) const RAD_S_TO_RPM: f32 = 60.0 / TAU;
/// rev/min → rad/s: ω = n · 2π / 60.
pub(crate) const RPM_TO_RAD_S: f32 = TAU / 60.0;

/// Returns `x` when finite, otherwise `fallback`. Used as a last line of defence so that a
/// single NaN (e.g. from a user-edited calibration cell) can never poison the state vector.
#[inline]
pub(crate) fn finite_or(x: f32, fallback: f32) -> f32 {
    if x.is_finite() {
        x
    } else {
        fallback
    }
}

/// Clamp that maps NaN to the lower bound (`f32::max` discards a NaN operand).
#[inline]
pub(crate) fn clampf(x: f32, lo: f32, hi: f32) -> f32 {
    x.max(lo).min(hi)
}

/// Exact discrete-time gain of the first-order lag `τ·dy/dt = u − y` over one step:
/// `α = 1 − e^(−dt/τ)`. Unlike forward Euler (`α = dt/τ`) it is unconditionally stable
/// and exact for piecewise-constant input, so sensor and actuator lags stay correct even
/// when τ is shorter than the integration step.
#[inline]
pub(crate) fn lag_alpha(dt: f32, tau: f32) -> f32 {
    if tau <= 1.0e-9 {
        1.0
    } else {
        1.0 - (-dt / tau).exp()
    }
}

/// Magnitude below which a decaying state is snapped to its target / to zero. Without it
/// exponential decays end in subnormal numbers, whose arithmetic takes a microcode slow
/// path on x86 and would slow every later step of a stopped engine several-fold.
const SNAP_EPSILON: f32 = 1.0e-30;

/// Advances a first-order lag towards `target` (see [`lag_alpha`]), landing exactly on the
/// target once the remaining gap is negligible.
#[inline]
pub(crate) fn approach(current: f32, target: f32, dt: f32, tau: f32) -> f32 {
    let next = current + (target - current) * lag_alpha(dt, tau);
    if (next - target).abs() <= SNAP_EPSILON * (1.0 + target.abs()) {
        target
    } else {
        next
    }
}

/// Flushes a negligible magnitude to exactly zero (see [`SNAP_EPSILON`]).
#[inline]
pub(crate) fn flush_tiny(x: f32) -> f32 {
    if x.abs() < SNAP_EPSILON {
        0.0
    } else {
        x
    }
}

/// Linear interpolation in the weighted form `a·(1 − t) + b·t`, which (unlike
/// `a + (b − a)·t`) cannot overflow for finite `a`, `b` and `t ∈ [0, 1]`.
#[inline]
pub(crate) fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a * (1.0 - t) + b * t
}

/// Hermite smoothstep: C¹-continuous 0→1 transition between `e0` and `e1`. Used wherever a
/// physical switch (thermostat wax, valve opening) must not inject derivative
/// discontinuities into the stiff parts of the model.
#[inline]
pub(crate) fn smoothstep(e0: f32, e1: f32, x: f32) -> f32 {
    let t = clampf((x - e0) / (e1 - e0), 0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Wraps a crank angle into one four-stroke cycle `[0, 4π)`.
#[inline]
pub(crate) fn wrap_cycle(theta: f32) -> f32 {
    let w = theta.rem_euclid(CYCLE_RAD);
    if w >= CYCLE_RAD {
        0.0
    } else {
        w
    }
}

/// PCG-XSH-RR 32-bit pseudo-random generator (O'Neill, 2014).
///
/// Chosen over OS entropy because Engine-Autopsy scenarios, regression tests and replays
/// must be bit-reproducible from a seed, and over `xorshift` because PCG passes TestU01
/// BigCrush while costing one 64-bit multiply per draw. 16 bytes of state, no allocation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Pcg32 {
    state: u64,
    inc: u64,
}

impl Pcg32 {
    /// 64-bit LCG multiplier from Knuth's MMIX, the reference constant of PCG.
    const MULTIPLIER: u64 = 6_364_136_223_846_793_005;

    /// Seeds the generator. `stream` selects one of 2⁶³ independent sequences.
    pub(crate) fn new(seed: u64, stream: u64) -> Self {
        let mut rng = Self {
            state: 0,
            inc: (stream << 1) | 1,
        };
        rng.next_u32();
        rng.state = rng.state.wrapping_add(seed);
        rng.next_u32();
        rng
    }

    /// Next 32 uniformly distributed bits.
    #[inline]
    pub(crate) fn next_u32(&mut self) -> u32 {
        let old = self.state;
        self.state = old.wrapping_mul(Self::MULTIPLIER).wrapping_add(self.inc);
        let xorshifted = (((old >> 18) ^ old) >> 27) as u32;
        let rot = (old >> 59) as u32;
        xorshifted.rotate_right(rot)
    }

    /// Uniform sample in `[0, 1)` with 24-bit resolution (the full f32 mantissa).
    #[inline]
    pub(crate) fn uniform(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 * (1.0 / 16_777_216.0)
    }

    /// Approximately standard-normal sample (mean 0, σ 1).
    ///
    /// Irwin–Hall construction: the sum of four U(0,1) variables has mean 2 and variance
    /// 4/12 = 1/3, so `(Σ − 2)·√3` has unit variance. Bounded to ±3.46σ, which is exactly
    /// what we want for cycle-to-cycle combustion variation: physically plausible spread
    /// without the unbounded tails of Box–Muller that could produce a 6σ "super-knock"
    /// out of pure numerical luck.
    #[inline]
    pub(crate) fn normal(&mut self) -> f32 {
        let s = self.uniform() + self.uniform() + self.uniform() + self.uniform();
        (s - 2.0) * 1.732_050_8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pcg_is_deterministic_and_uniform() {
        let mut a = Pcg32::new(42, 7);
        let mut b = Pcg32::new(42, 7);
        let mut sum = 0.0_f64;
        for _ in 0..10_000 {
            let x = a.uniform();
            assert_eq!(x, b.uniform());
            assert!((0.0..1.0).contains(&x));
            sum += f64::from(x);
        }
        let mean = sum / 10_000.0;
        assert!((mean - 0.5).abs() < 0.02, "mean {mean}");
    }

    #[test]
    fn normal_has_unit_variance() {
        let mut r = Pcg32::new(1, 1);
        let n = 20_000;
        let (mut s, mut s2) = (0.0_f64, 0.0_f64);
        for _ in 0..n {
            let x = f64::from(r.normal());
            s += x;
            s2 += x * x;
        }
        let mean = s / f64::from(n);
        let var = s2 / f64::from(n) - mean * mean;
        assert!(mean.abs() < 0.03);
        assert!((var - 1.0).abs() < 0.05, "var {var}");
    }

    #[test]
    fn decays_end_at_exact_zero() {
        let mut x = 1.0_f32;
        for _ in 0..200_000 {
            x = approach(x, 0.0, 2.5e-4, 0.01);
        }
        assert_eq!(x, 0.0);
        assert_eq!(flush_tiny(1.0e-35), 0.0);
        assert_eq!(flush_tiny(-2.0), -2.0);
    }

    #[test]
    fn wrap_cycle_stays_in_range() {
        assert_eq!(wrap_cycle(0.0), 0.0);
        assert!((wrap_cycle(-0.1) - (CYCLE_RAD - 0.1)).abs() < 1e-5);
        assert!(wrap_cycle(CYCLE_RAD * 3.0 + 0.5) < CYCLE_RAD);
    }
}

//! 3-band parallel crossover EQ + SVF sweep filter.
//!
//! Uses a Linkwitz-Riley 4th-order (LR4) crossover to split the signal into
//! three independent frequency bands (low / mid / hi), apply per-band gain,
//! then sum back together.  Killing one band (EQ = -1.0) produces true silence
//! for only that band's frequency range with zero bleed.
//!
//! ## Crossover frequencies (Pioneer DJM-900NXS2)
//!   - Low/Mid boundary:  250 Hz
//!   - Mid/Hi  boundary: 2500 Hz
//!
//! ## Gain mapping
//!   -1.0 → 0.0  (full kill)
//!    0.0 → 1.0  (unity / flat)
//!   +1.0 → +12 dB  (~3.98×)
//!
//! ## Sweep filter
//! TPT SVF (see svf.rs). State shared between HP/LP — no crossover click.

use super::biquad::{BiquadState, SosChain, linkwitz_riley_4th};
use super::svf::{SvfBand, sweep_filter_svf};
use crate::CHANNELS;

const XOVER_LOW_MID: f64 = 250.0;
const XOVER_MID_HI: f64 = 2500.0;

/// Per-channel state for one crossover filter path.
struct CrossoverPath {
    chain: SosChain,
    states: [BiquadState; CHANNELS],
}

impl CrossoverPath {
    fn new(wn: f64, highpass: bool) -> Self {
        let chain = linkwitz_riley_4th(wn, highpass);
        let num_sections = chain.num_sections();
        Self {
            chain,
            states: std::array::from_fn(|_| BiquadState::new(num_sections)),
        }
    }

    /// Process one sample for a single channel.
    #[inline]
    fn process_sample(&mut self, x: f64, ch: usize) -> f64 {
        self.chain.process(x, &mut self.states[ch])
    }

    fn reset(&mut self) {
        for s in &mut self.states {
            s.reset();
        }
    }
}

/// Map EQ knob value [-1, +1] → linear gain.
#[inline]
fn eq_gain(v: f64) -> f64 {
    if v <= -1.0 + 1e-4 {
        return 0.0; // full kill
    }
    if v.abs() < 1e-4 {
        return 1.0; // unity
    }
    let db = if v < 0.0 { v * 26.0 } else { v * 12.0 };
    10.0_f64.powf(db / 20.0)
}

/// Complete 3-band crossover EQ + SVF sweep filter for one deck.
pub struct EqChain {
    // LR4 crossover filter paths
    // Topology: input → lp1(low) + hp1(upper) → lp2(mid) + hp2(hi)
    lp1: CrossoverPath,  // input → low band
    hp1: CrossoverPath,  // input → upper (mid+hi)
    lp2: CrossoverPath,  // upper → mid band
    hp2: CrossoverPath,  // upper → hi band

    // Per-band gain (linear, with smoothing)
    gain_low:  f64,
    gain_mid:  f64,
    gain_hi:   f64,
    target_low:  f64,
    target_mid:  f64,
    target_hi:   f64,

    sweep: SvfBand,

    // Public parameter values (for easing reads)
    pub eq_low:     f64,
    pub eq_mid:     f64,
    pub eq_hi:      f64,
    pub filter_val: f64,
    /// Sample rate the crossover coefficients were designed for.
    sr: f64,
    dirty: bool,
}

impl EqChain {
    pub fn new(sample_rate: f64) -> Self {
        let sr = sample_rate;
        let nyq = sr / 2.0;
        let wn1 = XOVER_LOW_MID / nyq;
        let wn2 = XOVER_MID_HI / nyq;

        Self {
            lp1: CrossoverPath::new(wn1, false),
            hp1: CrossoverPath::new(wn1, true),
            lp2: CrossoverPath::new(wn2, false),
            hp2: CrossoverPath::new(wn2, true),

            gain_low:  1.0,
            gain_mid:  1.0,
            gain_hi:   1.0,
            target_low:  1.0,
            target_mid:  1.0,
            target_hi:   1.0,

            sweep: SvfBand::new(),

            eq_low:     0.0,
            eq_mid:     0.0,
            eq_hi:      0.0,
            filter_val: 0.5,
            sr,
            dirty:      true,
        }
    }

    /// Update EQ parameters. Marks dirty if any value changed.
    pub fn set_eq(&mut self, hi: f64, mid: f64, low: f64) {
        if (self.eq_hi  - hi ).abs() > 1e-6
            || (self.eq_mid - mid).abs() > 1e-6
            || (self.eq_low - low).abs() > 1e-6
        {
            self.eq_hi  = hi;
            self.eq_mid = mid;
            self.eq_low = low;
            self.dirty = true;
        }
    }

    /// Update filter sweep value. Marks dirty if changed.
    pub fn set_filter(&mut self, val: f64) {
        if (self.filter_val - val).abs() > 1e-6 {
            self.filter_val = val;
            self.dirty = true;
        }
    }

    /// Rebuild target gains and sweep filter if dirty. Called from audio thread.
    pub fn rebuild_if_dirty(&mut self) {
        if !self.dirty {
            return;
        }

        self.target_low = eq_gain(self.eq_low);
        self.target_mid = eq_gain(self.eq_mid);
        self.target_hi  = eq_gain(self.eq_hi);

        // SVF sweep: no crackling, no state reset at HP/LP crossover
        let sr = self.sr;
        self.sweep.set(sweep_filter_svf(self.filter_val, sr));

        self.dirty = false;
    }

    /// Process a stereo frame in-place through the parallel crossover EQ + sweep.
    #[inline]
    pub fn process(&mut self, frame: &mut [f64; CHANNELS]) {
        // Smooth gains toward targets (one-pole, ~1ms at 44.1kHz)
        const SMOOTH: f64 = 0.02;
        self.gain_low += SMOOTH * (self.target_low - self.gain_low);
        self.gain_mid += SMOOTH * (self.target_mid - self.gain_mid);
        self.gain_hi  += SMOOTH * (self.target_hi  - self.gain_hi);

        // Snap to target when close enough to avoid denormal drift
        if (self.gain_low - self.target_low).abs() < 1e-6 { self.gain_low = self.target_low; }
        if (self.gain_mid - self.target_mid).abs() < 1e-6 { self.gain_mid = self.target_mid; }
        if (self.gain_hi  - self.target_hi ).abs() < 1e-6 { self.gain_hi  = self.target_hi;  }

        for ch in 0..CHANNELS {
            let x = frame[ch];

            // Split: input → low (LP at 250Hz) + upper (HP at 250Hz)
            let low   = self.lp1.process_sample(x, ch);
            let upper = self.hp1.process_sample(x, ch);

            // Split upper → mid (LP at 2.5kHz) + hi (HP at 2.5kHz)
            let mid = self.lp2.process_sample(upper, ch);
            let hi  = self.hp2.process_sample(upper, ch);

            // Apply per-band gain and sum
            frame[ch] = low * self.gain_low + mid * self.gain_mid + hi * self.gain_hi;
        }

        // Sweep filter on summed output
        self.sweep.process(frame);
    }

    /// Reset all filter states (e.g., on track load).
    pub fn reset(&mut self) {
        self.lp1.reset();
        self.hp1.reset();
        self.lp2.reset();
        self.hp2.reset();
        self.sweep.reset();
        self.gain_low = 1.0;
        self.gain_mid = 1.0;
        self.gain_hi  = 1.0;
        self.target_low = 1.0;
        self.target_mid = 1.0;
        self.target_hi  = 1.0;
        self.dirty = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f64 = 44_100.0;

    /// Steady-state magnitude flatness: a flat EQ must pass every frequency at
    /// unity, including the crossover frequencies (250 Hz, 2.5 kHz). A true
    /// 4th-order Butterworth crossover would show a +3 dB peak there; a correct
    /// Linkwitz-Riley crossover sums voltage-flat. Tolerance ±1 dB.
    #[test]
    fn flat_eq_is_flat_across_spectrum() {
        let sr = SR;
        // Include the two crossover frequencies where a Butterworth sum peaks.
        for &freq in &[60.0, 250.0, 1000.0, 2500.0, 8000.0, 14000.0] {
            let mut eq = EqChain::new(SR);
            eq.rebuild_if_dirty();

            // Skip a settling window so filter state and gain smoother converge.
            let warmup = 8192;
            let measure = 8192;
            let mut sig_energy = 0.0_f64;
            let mut out_energy = 0.0_f64;
            for i in 0..(warmup + measure) {
                let t = i as f64 / sr;
                let x = (2.0 * std::f64::consts::PI * freq * t).sin();
                let mut frame = [x; CHANNELS];
                eq.process(&mut frame);
                if i >= warmup {
                    sig_energy += x * x;
                    out_energy += frame[0] * frame[0];
                }
            }
            let gain_db = 10.0 * (out_energy / sig_energy).log10();
            assert!(
                gain_db.abs() < 1.0,
                "Flat EQ at {freq} Hz should be ~0 dB, got {gain_db:.2} dB"
            );
        }
    }

    #[test]
    fn kill_low_does_not_silence_highs() {
        let mut eq = EqChain::new(SR);
        eq.set_eq(0.0, 0.0, -1.0); // kill low only
        eq.rebuild_if_dirty();

        // Converge the gain smoother
        for _ in 0..500 {
            let mut f = [0.0; CHANNELS];
            eq.process(&mut f);
        }

        // Feed a high-frequency tone (~5kHz) — should pass through
        let sr = SR;
        let mut energy = 0.0_f64;
        let mut ref_energy = 0.0_f64;
        for i in 0..1024 {
            let t = i as f64 / sr;
            let x = (2.0 * std::f64::consts::PI * 5000.0 * t).sin();
            ref_energy += x * x;
            let mut frame = [x; CHANNELS];
            eq.process(&mut frame);
            energy += frame[0] * frame[0];
        }
        let ratio = energy / ref_energy;
        assert!(
            ratio > 0.7,
            "Killing low should not affect 5kHz tone, got ratio {ratio}"
        );
    }

    #[test]
    fn kill_low_silences_bass() {
        let mut eq = EqChain::new(SR);
        eq.set_eq(0.0, 0.0, -1.0); // kill low only
        eq.rebuild_if_dirty();

        // Converge the gain smoother
        for _ in 0..500 {
            let mut f = [0.0; CHANNELS];
            eq.process(&mut f);
        }

        // Feed a 100Hz tone — should be silenced
        let sr = SR;
        let mut energy = 0.0_f64;
        let mut ref_energy = 0.0_f64;
        for i in 0..1024 {
            let t = i as f64 / sr;
            let x = (2.0 * std::f64::consts::PI * 100.0 * t).sin();
            ref_energy += x * x;
            let mut frame = [x; CHANNELS];
            eq.process(&mut frame);
            energy += frame[0] * frame[0];
        }
        let ratio = energy / ref_energy;
        assert!(
            ratio < 0.05,
            "Killing low should silence 100Hz, got ratio {ratio}"
        );
    }

    #[test]
    fn kill_hi_preserves_bass() {
        let mut eq = EqChain::new(SR);
        eq.set_eq(-1.0, 0.0, 0.0); // kill hi only
        eq.rebuild_if_dirty();

        // Converge
        for _ in 0..500 {
            let mut f = [0.0; CHANNELS];
            eq.process(&mut f);
        }

        // 100Hz tone should pass through
        let sr = SR;
        let mut energy = 0.0_f64;
        let mut ref_energy = 0.0_f64;
        for i in 0..1024 {
            let t = i as f64 / sr;
            let x = (2.0 * std::f64::consts::PI * 100.0 * t).sin();
            ref_energy += x * x;
            let mut frame = [x; CHANNELS];
            eq.process(&mut frame);
            energy += frame[0] * frame[0];
        }
        let ratio = energy / ref_energy;
        assert!(
            ratio > 0.7,
            "Killing hi should not affect 100Hz, got ratio {ratio}"
        );
    }

    #[test]
    fn kill_mid_silences_midrange() {
        let mut eq = EqChain::new(SR);
        eq.set_eq(0.0, -1.0, 0.0); // kill mid only
        eq.rebuild_if_dirty();

        // Converge the gain smoother
        for _ in 0..500 {
            let mut f = [0.0; CHANNELS];
            eq.process(&mut f);
        }

        // A 1kHz tone sits squarely in the mid band (250Hz–2.5kHz) → silenced.
        let sr = SR;
        let mut energy = 0.0_f64;
        let mut ref_energy = 0.0_f64;
        for i in 0..2048 {
            let t = i as f64 / sr;
            let x = (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
            ref_energy += x * x;
            let mut frame = [x; CHANNELS];
            eq.process(&mut frame);
            energy += frame[0] * frame[0];
        }
        let ratio = energy / ref_energy;
        assert!(
            ratio < 0.05,
            "Killing mid should silence 1kHz, got ratio {ratio}"
        );
    }

    #[test]
    fn kill_mid_preserves_bass_and_highs() {
        let mut eq = EqChain::new(SR);
        eq.set_eq(0.0, -1.0, 0.0); // kill mid only
        eq.rebuild_if_dirty();

        for _ in 0..500 {
            let mut f = [0.0; CHANNELS];
            eq.process(&mut f);
        }

        // Both a 100Hz (low) and 8kHz (hi) tone should pass through.
        let sr = SR;
        for &freq in &[100.0, 8000.0] {
            let mut energy = 0.0_f64;
            let mut ref_energy = 0.0_f64;
            for i in 0..2048 {
                let t = i as f64 / sr;
                let x = (2.0 * std::f64::consts::PI * freq * t).sin();
                ref_energy += x * x;
                let mut frame = [x; CHANNELS];
                eq.process(&mut frame);
                energy += frame[0] * frame[0];
            }
            let ratio = energy / ref_energy;
            assert!(
                ratio > 0.7,
                "Killing mid should not affect {freq}Hz, got ratio {ratio}"
            );
        }
    }
}

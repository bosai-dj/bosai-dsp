//! Beat FX: Phaser.
//!
//! A chain of 8 first-order all-pass filters whose break frequencies are swept
//! by a sine LFO. Where the all-pass chain produces 180° of phase shift, the
//! 50/50 mix of dry and AP output cancels — creating notches. With 8 stages
//! there are 4 notches that sweep together as the LFO moves.
//!
//! Unlike a flanger (which uses a delay line and produces evenly-spaced comb
//! filter teeth), the all-pass topology produces non-uniformly spaced notches
//! and a smoother, less aggressive "whoosh" sweep.
//!
//! Beat value → LFO period: `lfo_rate_hz = bpm / (60 × beat_value)`.
//! Depth controls wet/dry mix AND feedback level:
//!   depth = 0 → dry only
//!   depth = 1 → 50/50 dry+AP mix (maximum notch depth) with full feedback
//!
//! Feedback (from last stage output back into first stage input) deepens the
//! notches and adds resonance; capped at 0.7 for stability.
//!
//! All-pass coefficient: `g = (tan(π×fc/sr) − 1) / (tan(π×fc/sr) + 1)`.
//! Break frequency fc is swept logarithmically from 100 Hz to 8 kHz.

use super::smoother::Smoother;
use std::f64::consts::{PI, TAU};

const STAGES: usize = 8;
const MIN_FC: f64 = 100.0;   // Hz — bottom of LFO sweep
const MAX_FC: f64 = 8000.0;  // Hz — top of LFO sweep
const MAX_FEEDBACK: f64 = 0.70;
const SMOOTH_S: f64 = 0.015;

pub struct Phaser {
    pub enabled: bool,
    sample_rate: f64,

    /// All-pass integrator states: [channel][stage] = previous output.
    /// 1st-order AP only needs one state per stage (y[n-1]).
    ap_state: [[f64; STAGES]; 2],
    /// Feedback sample from the last AP stage, per channel.
    fb_state: [f64; 2],

    lfo_phase: f64,
    lfo_rate_hz: f64,
    bpm: f64,
    beat_value: f64,

    depth_target: f64,
    depth_smooth: Smoother,
}

impl Phaser {
    pub fn new(sample_rate: f64) -> Self {
        let bpm = 120.0;
        let beat_value = 1.0;
        Self {
            enabled: false,
            sample_rate,
            ap_state: [[0.0; STAGES]; 2],
            fb_state: [0.0; 2],
            lfo_phase: 0.0,
            lfo_rate_hz: bpm / (60.0 * beat_value),
            bpm,
            beat_value,
            depth_target: 0.5,
            depth_smooth: Smoother::new(sample_rate, SMOOTH_S),
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        if enabled && !self.enabled {
            self.ap_state = [[0.0; STAGES]; 2];
            self.fb_state = [0.0; 2];
            self.lfo_phase = 0.0;
            // Reset to 0 so depth fades in on arm rather than snapping to target.
            self.depth_smooth.reset(0.0);
        }
        self.enabled = enabled;
    }

    pub fn set_depth(&mut self, depth: f64) {
        self.depth_target = depth.clamp(0.0, 1.0);
    }

    pub fn set_bpm(&mut self, bpm: f64) {
        if bpm > 0.0 && (bpm - self.bpm).abs() > 0.01 {
            self.bpm = bpm;
            self.update_lfo_rate();
        }
    }

    pub fn set_beat_value(&mut self, beat_value: f64) {
        if beat_value > 0.0 && (beat_value - self.beat_value).abs() > 1e-9 {
            self.beat_value = beat_value;
            self.update_lfo_rate();
        }
    }

    fn update_lfo_rate(&mut self) {
        self.lfo_rate_hz = self.bpm / (60.0 * self.beat_value.max(1e-6));
    }

    pub fn process_sample(&mut self, sample: &mut [f64; 2]) {
        if !self.enabled {
            return;
        }

        let depth = self.depth_smooth.process(self.depth_target);
        let feedback_level = depth * MAX_FEEDBACK;

        // Sine LFO → log sweep of all-pass break frequency.
        self.lfo_phase += self.lfo_rate_hz / self.sample_rate;
        if self.lfo_phase >= 1.0 {
            self.lfo_phase -= 1.0;
        }
        let lfo = (TAU * self.lfo_phase).sin(); // -1..1
        let t = 0.5 * (1.0 + lfo); // 0..1
        let fc = MIN_FC * (MAX_FC / MIN_FC).powf(t);
        let g = self.ap_coeff(fc);

        for ch in 0..2 {
            // Feed forward path with feedback from last stage.
            let input = sample[ch] + feedback_level * self.fb_state[ch];

            // Cascade 8 first-order all-pass stages.
            // y[n] = g * (x[n] - y[n-1]) + x[n-1]  where we store y[n-1] per stage.
            let mut x = input;
            for stage in 0..STAGES {
                // 1-state all-pass (Regalia & Mitra):
                //   w[n] = x[n] - g * w[n-1]
                //   y[n] = g * w[n] + w[n-1]
                // State stored: w[n-1].
                let w_prev = self.ap_state[ch][stage];
                let w = x - g * w_prev;
                let y = g * w + w_prev;
                self.ap_state[ch][stage] = w;
                x = y;
            }

            self.fb_state[ch] = x; // last stage output → feedback next sample

            // Mix: notches appear where AP output is 180° from dry.
            // At depth=1: 50/50 mix maximises notch depth.
            // lerp(dry, 0.5*(dry+ap), depth) = dry*(1 - 0.5*depth) + ap*0.5*depth
            sample[ch] = sample[ch] * (1.0 - 0.5 * depth) + x * 0.5 * depth;
        }
    }

    /// First-order all-pass coefficient for break frequency `fc`.
    /// g = (tan(π·fc/sr) − 1) / (tan(π·fc/sr) + 1)
    fn ap_coeff(&self, fc: f64) -> f64 {
        let fc = fc.clamp(1.0, self.sample_rate * 0.499);
        let t = (PI * fc / self.sample_rate).tan();
        (t - 1.0) / (t + 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f64 = 44100.0;

    fn armed(beat_value: f64, depth: f64) -> Phaser {
        let mut p = Phaser::new(SR);
        p.set_bpm(120.0);
        p.set_beat_value(beat_value);
        p.set_depth(depth);
        p.set_enabled(true);
        p
    }

    #[test]
    fn disabled_passes_through() {
        let mut p = Phaser::new(SR);
        p.set_depth(1.0);
        let mut s = [0.7_f64, 0.4_f64];
        p.process_sample(&mut s);
        assert_eq!(s, [0.7, 0.4]);
    }

    #[test]
    fn depth_zero_is_dry() {
        // At depth=0 the AP chain runs but the output should be pure dry.
        let mut p = armed(1.0, 0.0);
        // Let smoother settle to 0.
        for _ in 0..4000 { let mut s = [0.5_f64; 2]; p.process_sample(&mut s); }
        let mut s = [0.6_f64, 0.3_f64];
        p.process_sample(&mut s);
        assert!((s[0] - 0.6).abs() < 1e-4, "depth=0 should be dry; got {}", s[0]);
        assert!((s[1] - 0.3).abs() < 1e-4, "depth=0 should be dry; got {}", s[1]);
    }

    #[test]
    fn creates_notches_at_full_depth() {
        // At depth=1 and a fixed (non-modulated) LFO, feed a tone at a notch
        // frequency — energy should be lower than input.
        // We can't know exact notch frequencies without computing AP coeffs,
        // so instead: measure that output RMS at full depth differs from input RMS,
        // proving the AP chain is doing something.
        let mut p = armed(8.0, 1.0); // very slow LFO ≈ static filter
        // Settle smoother
        for _ in 0..4000 { let mut s = [0.5_f64; 2]; p.process_sample(&mut s); }

        let mut in_energy = 0.0_f64;
        let mut out_energy = 0.0_f64;
        for i in 0..512 {
            let x = (i as f64 * 0.01).sin(); // arbitrary tone
            in_energy += x * x;
            let mut s = [x, x];
            p.process_sample(&mut s);
            out_energy += s[0] * s[0];
        }
        assert_ne!((in_energy * 1000.0) as i64, (out_energy * 1000.0) as i64,
            "phaser should modify energy; in={in_energy:.4} out={out_energy:.4}");
    }

    #[test]
    fn lfo_modulates_output() {
        let mut p = armed(0.125, 1.0); // fast LFO: 120/(60*0.125)=16 Hz
        for _ in 0..4000 { let mut s = [1.0_f64; 2]; p.process_sample(&mut s); }

        let rms = |ph: &mut Phaser, n: usize| {
            let mut e = 0.0_f64;
            for _ in 0..n { let mut s = [1.0_f64; 2]; ph.process_sample(&mut s); e += s[0]*s[0]; }
            (e / n as f64).sqrt()
        };
        let a = rms(&mut p, 256);
        let half_period = (SR / (16.0 * 2.0)) as usize;
        for _ in 0..half_period { let mut s = [1.0_f64; 2]; p.process_sample(&mut s); }
        let b = rms(&mut p, 256);
        assert!((a - b).abs() > 0.001, "LFO should modulate phaser; a={a:.4} b={b:.4}");
    }

    #[test]
    fn enable_resets_state() {
        let mut p = armed(1.0, 0.8);
        for _ in 0..10000 { let mut s = [0.9_f64; 2]; p.process_sample(&mut s); }
        p.set_enabled(false);
        p.set_enabled(true);
        // After re-arm, AP states and feedback should be zeroed.
        for ch in 0..2 {
            for stage in 0..STAGES {
                assert_eq!(p.ap_state[ch][stage], 0.0,
                    "ap_state[{ch}][{stage}] should be 0 after re-arm");
            }
            assert_eq!(p.fb_state[ch], 0.0, "fb_state[{ch}] should be 0 after re-arm");
        }
    }
}

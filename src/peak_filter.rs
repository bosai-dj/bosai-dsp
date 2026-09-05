//! Beat FX: Peak Filter.
//!
//! A narrow resonant bandpass filter with a sine LFO sweeping the center
//! frequency through the audible spectrum. Unlike the sweep Filter FX (which
//! moves a low- or high-pass edge), Peak Filter cuts everything *except* a
//! narrow moving band — useful for isolating a specific element (hi-hat, synth
//! note, kick transient) during a transition or build.
//!
//! Center frequency sweeps in **log space** from 200 Hz to 8 kHz so every
//! octave gets equal LFO time (sounds musical vs. a linear sweep).
//!
//! Parameters:
//!   `depth`      — wet/dry mix. 0 = dry pass-through, 1 = full bandpass isolation.
//!   `shape`      — resonance / Q: 0 → Q≈4 (wide), 0.5 → Q≈12 (default, hi-hat
//!                  isolation), 1 → Q≈24 (very narrow, near-sinusoidal band).
//!   `beat_value` — LFO period in beats. 4 = one full sweep every 4 beats
//!                  (default). 1 = fast sweep, 8 = very slow.
//!
//! Insertion point: post-fader, alongside Flanger / Phaser / Gater.
//!
//! Implementation: TPT State Variable Filter (Simper 2012). Integrator state
//! (ic1eq, ic2eq) is independent of coefficient values, so per-sample cutoff
//! changes from the LFO produce no zipper noise even at Q=24. BP output is
//! v1*k (normalized 0 dB at the center frequency). Contrast with the old DF1
//! biquad approach — DF1 recomputes coefficients per sample and feeds them back
//! through accumulated y1/y2 history, causing audible fizz at high Q.

use super::smoother::Smoother;
use std::f64::consts::{PI, TAU};

const F_MIN: f64 = 200.0;    // Hz — lower sweep bound (below this = sub content)
const F_MAX: f64 = 8_000.0;  // Hz — upper sweep bound (above this = air, less musical)
const Q_MIN: f64 = 4.0;      // shape=0  → wider band, gentler resonance
const Q_MAX: f64 = 24.0;     // shape=1  → very narrow, near-sine isolation
const SMOOTH_S: f64 = 0.015; // 15 ms param smoother — click-free depth/shape changes

pub struct PeakFilter {
    pub enabled: bool,
    sample_rate: f64,

    // LFO state
    lfo_phase: f64,
    lfo_rate_hz: f64,
    bpm: f64,
    beat_value: f64,

    // TPT SVF integrator states — one pair per channel.
    // Preserved across per-sample coefficient changes → no zipper noise.
    ic1eq: [f64; 2],
    ic2eq: [f64; 2],

    depth_target: f64,
    depth_smooth: Smoother,

    shape_target: f64,
    shape_smooth: Smoother,
}

impl PeakFilter {
    pub fn new(sample_rate: f64) -> Self {
        let bpm = 120.0;
        let beat_value = 4.0; // one sweep every 4 beats
        Self {
            enabled: false,
            sample_rate,
            lfo_phase: 0.0,
            lfo_rate_hz: bpm / (60.0 * beat_value),
            bpm,
            beat_value,
            ic1eq: [0.0; 2],
            ic2eq: [0.0; 2],
            depth_target: 0.75,
            // Reset to 0.0 so depth fades in on arm — no snap to target.
            depth_smooth: Smoother::new(sample_rate, SMOOTH_S),
            shape_target: 0.5,
            shape_smooth: Smoother::new(sample_rate, SMOOTH_S),
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        if enabled && !self.enabled {
            // Reset SVF state and LFO on arm so the effect always starts cleanly.
            self.lfo_phase = 0.0;
            self.ic1eq = [0.0; 2];
            self.ic2eq = [0.0; 2];
            // Start smoother at 0 so depth fades in rather than snapping to target.
            self.depth_smooth.reset(0.0);
            self.shape_smooth.reset(self.shape_target);
        }
        self.enabled = enabled;
    }

    pub fn set_depth(&mut self, depth: f64) {
        self.depth_target = depth.clamp(0.0, 1.0);
    }

    pub fn set_shape(&mut self, shape: f64) {
        self.shape_target = shape.clamp(0.0, 1.0);
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
        let shape = self.shape_smooth.process(self.shape_target);

        // Advance LFO phase.
        self.lfo_phase += self.lfo_rate_hz / self.sample_rate;
        if self.lfo_phase >= 1.0 {
            self.lfo_phase -= 1.0;
        }

        // Sine LFO → log-space center frequency.
        // lfo_norm ∈ [0, 1]; log interpolation → equal time per octave.
        let lfo_norm = 0.5 * (1.0 + (TAU * self.lfo_phase).sin());
        let f0 = (F_MIN * (F_MAX / F_MIN).powf(lfo_norm)).clamp(F_MIN, F_MAX);

        // Q from shape: linear mapping Q_MIN..Q_MAX.
        let q = (Q_MIN + shape * (Q_MAX - Q_MIN)).max(0.5);

        // TPT SVF coefficients (Simper 2012).
        //   g  = tan(π f0 / sr)   — integrator gain
        //   k  = 1/Q              — damping
        //   a1 = 1 / (1 + g(g+k))
        //   a2 = g·a1
        //   a3 = g·a2
        let g  = (PI * f0 / self.sample_rate).tan();
        let k  = 1.0 / q;
        let a1 = 1.0 / (1.0 + g * (g + k));
        let a2 = g * a1;
        let a3 = g * a2;

        // Per-channel SVF — integrator state preserved across coefficient changes.
        // BP output: v1 (unnormalized). Normalized 0 dB at center: v1 * k.
        for ch in 0..2 {
            let x = sample[ch];
            let v3 = x - self.ic2eq[ch];
            let v1 = a1 * self.ic1eq[ch] + a2 * v3;
            let v2 = self.ic2eq[ch] + a2 * self.ic1eq[ch] + a3 * v3;
            self.ic1eq[ch] = 2.0 * v1 - self.ic1eq[ch];
            self.ic2eq[ch] = 2.0 * v2 - self.ic2eq[ch];

            // Normalized bandpass (0 dB at center frequency) = v1 * k.
            let filtered = v1 * k;

            // Wet/dry blend: depth=0 → dry, depth=1 → bandpass only.
            sample[ch] = x * (1.0 - depth) + filtered * depth;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f64 = 44100.0;

    fn armed(beat_value: f64, depth: f64, shape: f64) -> PeakFilter {
        let mut pf = PeakFilter::new(SR);
        pf.set_bpm(120.0);
        pf.set_beat_value(beat_value);
        pf.set_depth(depth);
        pf.set_shape(shape);
        pf.set_enabled(true);
        pf
    }

    #[test]
    fn disabled_passes_through() {
        let mut pf = PeakFilter::new(SR);
        pf.set_depth(1.0);
        let mut s = [0.7_f64, 0.3_f64];
        pf.process_sample(&mut s);
        assert_eq!(s, [0.7, 0.3]);
    }

    #[test]
    fn depth_zero_is_dry() {
        let mut pf = armed(4.0, 0.0, 0.5);
        // Settle smoothers to 0.
        for _ in 0..4_000 {
            let mut s = [0.5_f64; 2];
            pf.process_sample(&mut s);
        }
        let mut s = [0.6_f64, 0.4_f64];
        pf.process_sample(&mut s);
        assert!((s[0] - 0.6).abs() < 1e-3, "depth=0 must be dry; got {}", s[0]);
        assert!((s[1] - 0.4).abs() < 1e-3, "depth=0 must be dry; got {}", s[1]);
    }

    #[test]
    fn dc_attenuated_at_full_depth() {
        // A bandpass filter has zero gain at DC (w=0). With depth=1, feeding
        // constant 1.0 should yield near-zero output after SVF state settles.
        let mut pf = armed(4.0, 1.0, 0.5);
        // Freeze LFO at a mid-band frequency.
        pf.lfo_rate_hz = 0.0;
        pf.lfo_phase   = 0.25; // sin(TAU*0.25) = 1 → lfo_norm=1 → f0=F_MAX
        // Settle depth smoother.
        for _ in 0..8_000 {
            let mut s = [1.0_f64; 2];
            pf.process_sample(&mut s);
        }
        // Now drive DC and measure output.
        let mut power = 0.0_f64;
        for _ in 0..1_024 {
            let mut s = [1.0_f64; 2];
            pf.process_sample(&mut s);
            power += s[0] * s[0];
        }
        let rms = (power / 1_024.0).sqrt();
        assert!(rms < 0.05, "DC should be attenuated by bandpass; rms={rms:.4}");
    }

    #[test]
    fn lfo_shifts_filter_frequency() {
        // Freeze the LFO at two different phases and drive a 1 kHz sine (within
        // the 200–8 kHz sweep range). The phase that puts f0 near 1 kHz should
        // pass significantly more energy than the phase that puts f0 at 8 kHz.
        //
        // phase=0.0 → sin=0 → lfo_norm=0.5 → f0 = F_MIN*(F_MAX/F_MIN)^0.5 ≈ 1265 Hz
        //                  → 1 kHz is close to the passband, passes more energy.
        // phase=0.25 → sin=1 → lfo_norm=1.0 → f0 = F_MAX = 8000 Hz
        //                  → 1 kHz is 3 octaves below passband, heavily attenuated.
        let measure_1khz_power = |phase: f64| -> f64 {
            let mut pf = PeakFilter::new(SR);
            pf.set_bpm(120.0);
            pf.set_beat_value(4.0);
            pf.set_depth(1.0);
            pf.set_shape(0.5);
            pf.lfo_rate_hz = 0.0; // freeze LFO
            pf.set_enabled(true);
            pf.lfo_phase = phase; // set after enable (enable resets to 0)
            // Settle SVF transient.
            for i in 0..8_000_usize {
                let v = (TAU * 1000.0 * i as f64 / SR).sin();
                let mut s = [v; 2]; pf.process_sample(&mut s);
            }
            // Measure output power of 1 kHz sine.
            let mut e = 0.0_f64;
            for i in 8_000_usize..9_024 {
                let v = (TAU * 1000.0 * i as f64 / SR).sin();
                let mut s = [v; 2]; pf.process_sample(&mut s);
                e += s[0] * s[0];
            }
            e / 1_024.0
        };

        let power_near = measure_1khz_power(0.0);   // f0 ≈ 1265 Hz — 1 kHz near passband
        let power_far  = measure_1khz_power(0.25);  // f0 = 8000 Hz — 1 kHz 3 octaves away
        assert!(power_near > power_far * 3.0,
            "LFO should shift filter freq; near_1k={power_near:.5} far_1k={power_far:.5}");
    }

    #[test]
    fn enable_resets_svf_state() {
        let mut pf = armed(1.0, 0.8, 0.5);
        // Flood with high-amplitude signal to fill SVF integrators.
        for _ in 0..2_000 {
            let mut s = [1.0_f64; 2];
            pf.process_sample(&mut s);
        }
        pf.set_enabled(false);
        pf.set_enabled(true);
        assert_eq!(pf.ic1eq, [0.0; 2]);
        assert_eq!(pf.ic2eq, [0.0; 2]);
        assert!(pf.lfo_phase < 1e-9, "LFO phase should reset on re-arm");
    }

    #[test]
    fn depth_fades_in_on_arm() {
        // depth_smooth resets to 0.0 on arm, not depth_target.
        // The first output sample should be much quieter than steady-state.
        let mut pf = PeakFilter::new(SR);
        pf.set_depth(1.0);
        pf.set_shape(0.5);
        pf.lfo_rate_hz = 0.0;
        pf.lfo_phase   = 0.0;
        pf.set_enabled(true);

        // First sample: smoother starts at 0 → depth ≈ 0 → near-dry output.
        let mut s_first = [0.5_f64; 2];
        pf.process_sample(&mut s_first);

        // After settling (~300 samples at 15 ms TC), smoother reaches depth_target.
        for _ in 0..4_000 {
            let mut s = [0.5_f64; 2];
            pf.process_sample(&mut s);
        }
        let mut s_settled = [0.5_f64; 2];
        pf.process_sample(&mut s_settled);

        // First sample should be closer to dry (0.5) than settled bandpass output.
        assert!((s_first[0] - 0.5).abs() < (s_settled[0] - 0.5).abs() + 1e-2,
            "first sample should be near-dry on arm; first={} settled={}", s_first[0], s_settled[0]);
    }

    #[test]
    fn output_bounded() {
        // Full depth, narrowest Q — check no blow-up over 1 second.
        let mut pf = armed(0.5, 1.0, 1.0);
        for i in 0..44_100_usize {
            // Alternating ±1 (Nyquist tone) — worst case for a narrow bandpass.
            let v = if i % 2 == 0 { 0.9 } else { -0.9 };
            let mut s = [v; 2];
            pf.process_sample(&mut s);
            assert!(s[0].abs() <= 2.0 && s[1].abs() <= 2.0,
                "output out of range at sample {i}: {} {}", s[0], s[1]);
        }
    }

    #[test]
    fn shape_affects_q() {
        // Higher shape → higher Q → narrower band → less energy on broadband input.
        // Both run with LFO frozen at the same phase so only Q differs.
        let run = |shape: f64| -> f64 {
            let mut pf = PeakFilter::new(SR);
            pf.set_bpm(120.0);
            pf.set_beat_value(4.0);
            pf.set_depth(1.0);
            pf.set_shape(shape);
            pf.lfo_rate_hz = 0.0;
            pf.lfo_phase   = 0.0; // f0 at F_MIN
            pf.set_enabled(true);
            // Settle.
            for _ in 0..8_000 { let mut s = [0.5_f64; 2]; pf.process_sample(&mut s); }
            // Measure broadband (alternating sign) output power.
            let mut e = 0.0_f64;
            let mut sign = 1.0_f64;
            for _ in 0..1_024 {
                let mut s = [sign; 2]; pf.process_sample(&mut s);
                e += s[0] * s[0]; sign = -sign;
            }
            e / 1_024.0
        };

        let power_wide   = run(0.0); // Q=4
        let power_narrow = run(1.0); // Q=24
        assert!(power_narrow < power_wide,
            "higher shape/Q should pass less broadband energy; wide={power_wide:.4} narrow={power_narrow:.4}");
    }
}

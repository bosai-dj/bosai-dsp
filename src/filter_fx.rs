//! Beat FX: resonant filter with LFO-modulated cutoff.
//!
//! The depth knob doubles as the filter position knob (Pioneer-style):
//!   0.0 → maximum HP (20 kHz HPF that sweeps down toward 20 Hz)
//!   0.5 → bypass / flat
//!   1.0 → maximum LP (20 Hz LPF that sweeps up toward 20 kHz)
//!
//! `beat_value` sets the LFO period in beats — same unit as Echo/Reverb.
//! The LFO modulates the cutoff between the set position and bypass using a
//! triangle wave: filter sweeps to the target and back once per beat_value beats.
//! At beat_value = 0 or lfo_rate < 0.001 Hz, the filter is static at `depth`.
//!
//! Q is fixed at 0.707 (Butterworth / maximally-flat). Pioneer hardware does
//! not expose a user-adjustable resonance on Beat FX Filter.
//!
//! Implementation: TPT SVF (Simper 2012) — same as the channel sweep filter.
//! State is preserved across parameter changes so there are no click artefacts
//! when the LFO changes the cutoff continuously.

use super::smoother::Smoother;
use super::svf::{SvfFilter, SvfState};

const SMOOTH_DEPTH_S: f64 = 0.020; // 20 ms depth smoothing (prevents zipper noise)
const Q: f64 = 0.707;              // Butterworth — flat passband, no resonance peak

pub struct FilterFx {
    pub enabled: bool,
    /// Filter position: 0.0=max HP, 0.5=bypass, 1.0=max LP.
    depth_target: f64,
    depth_smooth: Smoother,
    /// LFO triangle wave phase, 0..1.
    lfo_phase: f64,
    /// LFO rate in Hz, derived from beat_value + BPM. 0 = static filter.
    lfo_rate_hz: f64,
    /// Current BPM (updated from deck render loop, same as echo).
    bpm: f64,
    /// LFO period in beats (user-facing; converted to lfo_rate_hz via BPM).
    beat_value: f64,
    /// Per-channel SVF integrator state — intentionally NOT reset on parameter
    /// changes so there are no discontinuity clicks as the LFO sweeps.
    states: [SvfState; 2],
    sample_rate: f64,
}

impl FilterFx {
    pub fn new(sample_rate: f64) -> Self {
        Self {
            enabled: false,
            depth_target: 0.5,
            depth_smooth: Smoother::new(sample_rate, SMOOTH_DEPTH_S),
            lfo_phase: 0.0,
            lfo_rate_hz: 0.0,
            bpm: 120.0,
            beat_value: 1.0,
            states: [SvfState::new(), SvfState::new()],
            sample_rate,
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        if enabled && !self.enabled {
            self.lfo_phase = 0.0;
            // Snap smoother to current target so there's no initial filter sweep
            // artifact (smoother initialises at 0.0 which would briefly apply
            // a strong HP filter before settling at the real depth).
            self.depth_smooth.reset(self.depth_target);
        }
        self.enabled = enabled;
    }

    pub fn set_depth(&mut self, depth: f64) {
        self.depth_target = depth.clamp(0.0, 1.0);
    }

    /// Update live BPM (called from the deck render loop, mirrors Echo::set_bpm).
    pub fn set_bpm(&mut self, bpm: f64) {
        if bpm > 0.0 && (bpm - self.bpm).abs() > 0.01 {
            self.bpm = bpm;
            self.update_lfo_rate();
        }
    }

    /// Set LFO period in beats (same unit as Echo beat_value).
    /// At bpm=120, beat_value=1: one sweep per beat = 2 Hz LFO.
    pub fn set_beat_value(&mut self, beat_value: f64) {
        if beat_value > 0.0 && (beat_value - self.beat_value).abs() > 1e-9 {
            self.beat_value = beat_value;
            self.update_lfo_rate();
        }
    }

    fn update_lfo_rate(&mut self) {
        // LFO completes one cycle every beat_value beats.
        // lfo_rate_hz = beats_per_second / beat_value = (bpm/60) / beat_value
        self.lfo_rate_hz = self.bpm / (60.0 * self.beat_value.max(1e-6));
    }

    pub fn process_sample(&mut self, sample: &mut [f64; 2]) {
        if !self.enabled {
            return;
        }

        let depth = self.depth_smooth.process(self.depth_target);

        // Triangle LFO: sweeps 0→1→0 over one period.
        // At phase<0.5: rising  (0..1), at phase≥0.5: falling (1..0).
        // LFO = 1 → filter fully at `depth`. LFO = 0 → filter at bypass (0.5).
        let lfo = if self.lfo_rate_hz > 0.001 {
            self.lfo_phase += self.lfo_rate_hz / self.sample_rate;
            if self.lfo_phase >= 1.0 {
                self.lfo_phase -= 1.0;
            }
            let t = self.lfo_phase;
            if t < 0.5 { t * 2.0 } else { 2.0 - t * 2.0 }
        } else {
            1.0 // no LFO: filter stays at static `depth` position
        };

        // Modulate between bypass (0.5) and the set depth position.
        let effective_val = 0.5 + (depth - 0.5) * lfo;

        // Build SVF filter. Returns None inside the bypass zone — pass through.
        if let Some(filter) = make_filter(effective_val, self.sample_rate) {
            for ch in 0..2 {
                sample[ch] = filter.process(sample[ch], &mut self.states[ch]);
            }
        }
    }
}

/// Build an SVF filter from a normalised val [0, 1].
/// Matches the sweep_filter_svf convention in svf.rs:
///   val < 0.5 → HP (fc sweeps 20 kHz → 20 Hz)
///   val > 0.5 → LP (fc sweeps 20 kHz → 20 Hz)
///   |val - 0.5| < 0.005 → bypass (None)
fn make_filter(val: f64, sr: f64) -> Option<SvfFilter> {
    if (val - 0.5).abs() < 0.005 {
        return None;
    }
    let (fc, is_hp) = if val < 0.5 {
        let t = 1.0 - val * 2.0;               // 1.0 at val=0, 0.0 at val=0.5
        let f = 20.0 * 1000.0_f64.powf(t);     // 20 Hz × 1000^t: 20 Hz → 20 kHz
        (f.clamp(20.0, sr * 0.499), true)
    } else {
        let t = (val - 0.5) * 2.0;             // 0.0 at val=0.5, 1.0 at val=1.0
        let f = 20.0 * 1000.0_f64.powf(1.0 - t); // 20 kHz → 20 Hz
        (f.clamp(20.0, sr * 0.499), false)
    };
    Some(SvfFilter::new(fc, Q, sr, is_hp))
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f64 = 44100.0;

    fn armed_filter(depth: f64, beat_value: f64) -> FilterFx {
        let mut f = FilterFx::new(SR);
        f.set_depth(depth);
        f.set_bpm(120.0);
        f.set_beat_value(beat_value);
        f.set_enabled(true);
        f
    }

    fn fill(fx: &mut FilterFx, frames: usize) {
        for _ in 0..frames {
            let mut s = [1.0_f64, 1.0_f64];
            fx.process_sample(&mut s);
        }
    }

    #[test]
    fn bypass_at_center_depth() {
        let mut fx = FilterFx::new(SR);
        fx.set_depth(0.5);
        fx.set_enabled(true);
        let mut s = [1.0_f64, 1.0_f64];
        // Let depth smoother settle
        for _ in 0..2000 { fx.process_sample(&mut s); s = [1.0, 1.0]; }
        fx.process_sample(&mut s);
        // At depth=0.5, filter is in bypass zone — output ≈ input.
        assert!((s[0] - 1.0).abs() < 1e-6, "expected bypass at depth=0.5, got {}", s[0]);
    }

    #[test]
    fn lp_attenuates_highs() {
        // LP at depth=0.9 (strong LP) should heavily attenuate a high-frequency tone.
        let mut fx = armed_filter(0.9, 8.0); // slow LFO so filter stays near depth
        fill(&mut fx, 4410); // settle smoother + LFO starts at 0

        // Feed alternating ±1 (Nyquist) for 512 frames, measure RMS after filter
        let mut energy = 0.0_f64;
        for i in 0..512 {
            let mut s = [if i % 2 == 0 { 1.0_f64 } else { -1.0_f64 }; 2];
            fx.process_sample(&mut s);
            energy += s[0] * s[0];
        }
        let rms = (energy / 512.0).sqrt();
        assert!(rms < 0.2, "LP at depth=0.9 should attenuate Nyquist, got rms={rms}");
    }

    #[test]
    fn hp_attenuates_dc() {
        // HP at depth=0.1 (strong HP) should attenuate DC.
        let mut fx = armed_filter(0.1, 8.0);
        fill(&mut fx, 4410);
        let mut energy = 0.0_f64;
        for _ in 0..512 {
            let mut s = [1.0_f64; 2];
            fx.process_sample(&mut s);
            energy += s[0] * s[0];
        }
        let rms = (energy / 512.0).sqrt();
        assert!(rms < 0.2, "HP at depth=0.1 should attenuate DC, got rms={rms}");
    }

    #[test]
    fn lfo_modulates_output() {
        // With a fast LFO, output energy should vary over time (not constant).
        let mut fx = armed_filter(0.9, 0.25); // fast LFO: 120/(60*0.25) = 8 Hz
        fill(&mut fx, 4410); // settle

        // Collect 512-frame RMS snapshots at two points in the LFO cycle.
        let rms_snapshot = |fx: &mut FilterFx, n: usize| -> f64 {
            let mut e = 0.0;
            for i in 0..n {
                let mut s = [if i % 2 == 0 { 1.0_f64 } else { -1.0_f64 }; 2];
                fx.process_sample(&mut s);
                e += s[0] * s[0];
            }
            (e / n as f64).sqrt()
        };

        let rms_a = rms_snapshot(&mut fx, 512);
        // Advance half an LFO cycle: at SR=44100, 8 Hz LFO → half period ≈ 2756 frames
        fill(&mut fx, 2756);
        let rms_b = rms_snapshot(&mut fx, 512);
        // The RMS should differ by more than a tiny amount — LFO is working.
        assert!(
            (rms_a - rms_b).abs() > 0.05,
            "LFO should modulate output; rms_a={rms_a:.4} rms_b={rms_b:.4}",
        );
    }

    #[test]
    fn disabled_passes_through() {
        let mut fx = FilterFx::new(SR);
        fx.set_depth(0.0); // would be strong HP if enabled
        // Not enabled — signal must pass through untouched.
        let mut s = [1.0_f64, 0.5_f64];
        fx.process_sample(&mut s);
        assert_eq!(s, [1.0, 0.5]);
    }

    #[test]
    fn enable_resets_lfo_phase() {
        let mut fx = armed_filter(0.8, 1.0);
        fill(&mut fx, 10000);
        let phase_before = fx.lfo_phase;
        fx.set_enabled(false);
        fx.set_enabled(true);
        assert_eq!(fx.lfo_phase, 0.0, "lfo_phase should reset on re-arm; was {phase_before}");
    }
}

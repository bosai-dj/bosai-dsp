//! Reverb Beat FX: Freeverb algorithm wrapped with DJM-style depth + RT60 control.
//!
//! Algorithm: Schroeder/Moorer — 8 parallel comb filters into 4 series all-pass
//! filters (classic Freeverb). Identical to what Rekordbox uses internally.
//!
//! User-facing params:
//!   - `depth`: wet/dry mix (asymmetric DJM curve — same as Echo)
//!   - `decay_secs`: RT60 decay time in seconds (0.5 s → 8 s range)
//!     The beat-value knob drives this when Reverb is armed — NOT a beat
//!     multiplier. 1.0 = 1-second decay, 4.0 = 4-second decay (large hall).
//!
//! RT60 → Freeverb room_size conversion uses the longest comb (L=1617 @ 44100):
//!   g   = 10 ^ ( −4851 / (44100 * rt60) )
//!   room_size = ( g − 0.7 ) / 0.28            (Freeverb OFFSET_ROOM / SCALE_ROOM)
//!
//! Kill behaviour: always tail-mode. Setting enabled=false enters drain mode —
//! the comb filters ring out naturally. The input goes silent but the decaying
//! tail is mixed into the output until it drops below −80 dB.

use super::freeverb::Freeverb;
use super::smoother::Smoother;

const SMOOTH_DEPTH_S: f64 = 0.020;
const SMOOTH_DECAY_S: f64 = 0.080; // RT60 crossfade takes a bit longer

// Freeverb tuning constants (from freeverb/tuning.rs — must match).
const FREEVERB_OFFSET_ROOM: f64 = 0.7;
const FREEVERB_SCALE_ROOM: f64 = 0.28;
// Longest comb at 44100 Hz (samples).
const LONGEST_COMB_SAMPLES: f64 = 1617.0;
const SAMPLERATE_TUNING: f64 = 44100.0;

/// Convert RT60 (seconds) to Freeverb's normalised room_size [0, 1].
fn rt60_to_room_size(rt60_secs: f64) -> f64 {
    let rt60 = rt60_secs.max(0.01);
    let g = 10f64.powf(-3.0 * LONGEST_COMB_SAMPLES / (SAMPLERATE_TUNING * rt60));
    ((g - FREEVERB_OFFSET_ROOM) / FREEVERB_SCALE_ROOM).clamp(0.0, 1.0)
}

pub struct Reverb {
    pub enabled: bool,
    /// True while comb filters are draining after disarm. Never hard-cut.
    draining: bool,
    inner: Freeverb<f64>,
    depth_target: f64,
    depth_smooth: Smoother,
    decay_target: f64,
    decay_smooth: Smoother,
    /// Last applied room_size — skip set_room_size when delta is negligible.
    last_room_size: f64,
}

impl Reverb {
    pub fn new(sample_rate: f64) -> Self {
        let default_decay = 2.0; // 2-second RT60 on init
        let mut inner = Freeverb::<f64>::new(sample_rate as usize);
        inner.set_width(1.0);
        inner.set_dampening(0.5);
        inner.set_room_size(rt60_to_room_size(default_decay));
        inner.set_dry(1.0);
        inner.set_wet(0.0);
        let initial_room_size = rt60_to_room_size(default_decay);
        Self {
            enabled: false,
            draining: false,
            inner,
            depth_target: 0.0,
            depth_smooth: Smoother::new(sample_rate, SMOOTH_DEPTH_S),
            decay_target: default_decay,
            decay_smooth: Smoother::new(sample_rate, SMOOTH_DECAY_S),
            last_room_size: initial_room_size,
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        match (self.enabled, enabled) {
            (false, true) => {
                self.draining = false;
                self.enabled = true;
            }
            (true, false) => {
                // Always tail drain — never hard-cut reverb.
                self.draining = true;
                self.enabled = false;
            }
            _ => {}
        }
    }

    pub fn set_depth(&mut self, depth: f64) {
        self.depth_target = depth.clamp(0.0, 1.0);
    }

    /// Set RT60 decay time in seconds. The beat-value arrows drive this when
    /// Reverb is armed — 1.0 = 1 s, 2.0 = 2 s (large hall), 4.0 = cathedral.
    pub fn set_decay_secs(&mut self, secs: f64) {
        self.decay_target = secs.clamp(0.1, 10.0);
    }

    pub fn process_sample(&mut self, sample: &mut [f64; 2]) {
        if !self.enabled && !self.draining {
            return;
        }

        // Smooth RT60 → room_size and apply only when meaningfully changed.
        let decay = self.decay_smooth.process(self.decay_target);
        let room_size = rt60_to_room_size(decay);
        if (room_size - self.last_room_size).abs() > 1e-5 {
            self.inner.set_room_size(room_size);
            self.last_room_size = room_size;
        }

        if self.draining {
            // Advance depth smoother toward 0 so re-arm starts clean (no snap).
            self.depth_smooth.process(0.0);
            // Feed silence so the comb filters decay naturally into the output.
            self.inner.set_dry(0.0);
            self.inner.set_wet(1.0);
            let (l, r) = self.inner.tick((0.0, 0.0));
            if l.abs() < 1e-4 && r.abs() < 1e-4 {
                self.draining = false;
                return;
            }
            sample[0] += l;
            sample[1] += r;
            return;
        }

        let depth = self.depth_smooth.process(self.depth_target);

        // Asymmetric DJM wet/dry curve — matches Echo.
        // At depth 0.5: dry=1, wet=1 (wet stacks, feels louder — same feel as hardware).
        let (dry_gain, wet_gain) = if depth < 0.5 {
            (1.0, 2.0 * depth)
        } else {
            (2.0 * (1.0 - depth), 1.0)
        };
        self.inner.set_dry(dry_gain);
        self.inner.set_wet(wet_gain);

        let (l, r) = self.inner.tick((sample[0], sample[1]));
        sample[0] = l;
        sample[1] = r;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f64 = 44100.0;

    fn armed_reverb(depth: f64, decay_secs: f64) -> Reverb {
        let mut r = Reverb::new(SR);
        r.set_depth(depth);
        r.set_decay_secs(decay_secs);
        r.set_enabled(true);
        r
    }

    fn fill(reverb: &mut Reverb, frames: usize) {
        for _ in 0..frames {
            let mut s = [1.0_f64, 1.0_f64];
            reverb.process_sample(&mut s);
        }
    }

    #[test]
    fn rt60_to_room_size_range() {
        // Short decay → low room_size; long decay → high room_size.
        let short = rt60_to_room_size(0.5);
        let medium = rt60_to_room_size(2.0);
        let long_ = rt60_to_room_size(6.0);
        assert!(short < medium, "short={short} medium={medium}");
        assert!(medium < long_, "medium={medium} long={long_}");
        assert!(long_ <= 1.0, "room_size must not exceed 1.0: {long_}");
        // 2 s decay should map to a reasonable mid-range room_size.
        assert!(medium > 0.4 && medium < 0.9, "2 s RT60 out of range: {medium}");
    }

    #[test]
    fn disarm_enters_drain_not_hard_bypass() {
        let mut r = armed_reverb(0.7, 2.0);
        fill(&mut r, 4410);
        r.set_enabled(false);
        assert!(!r.enabled);
        assert!(r.draining, "expected draining=true after disarm");
    }

    #[test]
    fn tail_drains_to_silence() {
        // Short decay (1 s) should drain within 10 s of silence.
        let mut r = armed_reverb(0.7, 1.0);
        fill(&mut r, 4410);
        r.set_enabled(false);
        let limit = SR as usize * 10;
        for _ in 0..limit {
            let mut s = [0.0_f64, 0.0_f64];
            r.process_sample(&mut s);
            if !r.draining {
                return;
            }
        }
        panic!("reverb tail never drained within 10 s");
    }

    #[test]
    fn dry_passes_through_during_drain() {
        let mut r = armed_reverb(0.7, 2.0);
        fill(&mut r, 4410);
        r.set_enabled(false);
        let mut s = [1.0_f64, 1.0_f64];
        r.process_sample(&mut s);
        assert!(s[0] > 0.1, "dry signal muted during drain: {}", s[0]);
        assert!((s[0] - 1.0).abs() > 1e-6, "no reverb tail during drain: {}", s[0]);
    }
}

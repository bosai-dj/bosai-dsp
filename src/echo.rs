//! BPM-synced stereo echo with optional pre-filter variants.
//!
//! Post-fader channel send (inserted after the volume/xfade multiply in the
//! deck render path) so the tail persists through a channel fade-out — the
//! "echo out" move. Delay time = (60 / live_bpm) * beat_value * sample_rate,
//! where live_bpm tracks the track BPM scaled by current playback speed.
//!
//! Changing `beat_value` or BPM triggers a 20 ms 2-tap crossfade between the
//! old and new read positions so the transition doesn't click. Per-sample
//! one-pole smoothing on depth/feedback prevents zipper noise. Soft tanh
//! saturation inside the feedback loop prevents runaway.
//!
//! ## Filter variants (Pioneer DJM-900NXS2 Beat FX)
//!
//! The pre-filter sits between the live input and the delay buffer write.
//! Only the filtered signal enters the buffer; the dry output is always unfiltered.
//! This lets kick/bass stay solid while only filtered frequencies echo (HPF),
//! or creates a warm subby trail from low-frequency content only (LPF).
//!
//! - HPF / Low Cut Echo: HPF at ~1 kHz — highs and mids echo, lows stay dry.
//! - LPF Echo: LPF at ~1 kHz — lows and mids echo, highs stay dry.
//! - BPF Echo: cascade HPF 500 Hz → LPF 2 kHz — mid band echoes only.

use super::smoother::Smoother;
use super::svf::{SvfFilter, SvfState};

const MAX_DELAY_S: f64 = 4.0;
const CROSSFADE_S: f64 = 0.020;
const SMOOTH_S: f64 = 0.020;

const HPF_CUTOFF: f64 = 1000.0; // Hz — HPF Echo / Low Cut Echo
const LPF_CUTOFF: f64 = 1000.0; // Hz — LPF Echo
const BPF_HP_FC: f64  =  500.0; // Hz — BPF high-pass stage
const BPF_LP_FC: f64  = 2000.0; // Hz — BPF low-pass stage
const PRE_Q: f64 = 0.707;       // Butterworth — no resonance peak

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum EchoFilter {
    None,
    HPF,
    LPF,
    BPF,
}

pub struct Echo {
    pub enabled: bool,
    /// True while the delay buffer is draining after disarm.
    /// The buffer decays via the feedback loop with zero input until it falls
    /// silent, then draining resets to false automatically.
    draining: bool,
    sample_rate: f64,

    buffer: Vec<[f64; 2]>,
    buf_frames: usize,
    write_idx: usize,

    current_delay: f64,
    prev_delay: f64,
    crossfade_remaining: usize,
    crossfade_total: usize,

    depth_target: f64,
    depth_smooth: Smoother,
    feedback_target: f64,
    feedback_smooth: Smoother,

    bpm: f64,
    beat_value: f64,

    /// Pre-filter variant — filters input before it enters the delay buffer.
    pub filter_mode: EchoFilter,
    pre_hp: Option<SvfFilter>,
    pre_lp: Option<SvfFilter>,
    pre_hp_states: [SvfState; 2],
    pre_lp_states: [SvfState; 2],
}

impl Echo {
    pub fn new(sample_rate: f64) -> Self {
        let buf_frames = (MAX_DELAY_S * sample_rate) as usize;
        let crossfade_total = (CROSSFADE_S * sample_rate) as usize;
        let bpm = 120.0;
        let beat_value = 1.0;
        let initial_delay = (60.0 / bpm) * beat_value * sample_rate;
        Self {
            enabled: false,
            draining: false,
            sample_rate,
            buffer: vec![[0.0; 2]; buf_frames],
            buf_frames,
            write_idx: 0,
            current_delay: initial_delay,
            prev_delay: initial_delay,
            crossfade_remaining: 0,
            crossfade_total,
            depth_target: 0.0,
            depth_smooth: Smoother::new(sample_rate, SMOOTH_S),
            feedback_target: 0.5,
            feedback_smooth: Smoother::new(sample_rate, SMOOTH_S),
            bpm,
            beat_value,
            filter_mode: EchoFilter::None,
            pre_hp: None,
            pre_lp: None,
            pre_hp_states: [SvfState::new(), SvfState::new()],
            pre_lp_states: [SvfState::new(), SvfState::new()],
        }
    }

    pub fn set_filter_mode(&mut self, mode: EchoFilter) {
        if mode == self.filter_mode {
            return;
        }
        self.filter_mode = mode;
        let sr = self.sample_rate;
        match mode {
            EchoFilter::None => {
                self.pre_hp = None;
                self.pre_lp = None;
            }
            EchoFilter::HPF => {
                self.pre_hp = Some(SvfFilter::new(HPF_CUTOFF, PRE_Q, sr, true));
                self.pre_lp = None;
            }
            EchoFilter::LPF => {
                self.pre_hp = None;
                self.pre_lp = Some(SvfFilter::new(LPF_CUTOFF, PRE_Q, sr, false));
            }
            EchoFilter::BPF => {
                self.pre_hp = Some(SvfFilter::new(BPF_HP_FC, PRE_Q, sr, true));
                self.pre_lp = Some(SvfFilter::new(BPF_LP_FC, PRE_Q, sr, false));
            }
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        match (self.enabled, enabled) {
            (false, true) => {
                // Arm: flush stale buffer so the first repeat is clean.
                self.buffer.fill([0.0; 2]);
                self.write_idx = 0;
                self.crossfade_remaining = 0;
                self.draining = false;
                // Reset pre-filter integrator states to avoid initial transient.
                self.pre_hp_states = [SvfState::new(), SvfState::new()];
                self.pre_lp_states = [SvfState::new(), SvfState::new()];
                self.enabled = true;
            }
            (true, false) => {
                // Disarm: enter tail drain instead of hard bypass.
                // The feedback loop continues with zero input until the buffer
                // falls below the silence threshold.
                self.draining = true;
                self.enabled = false;
            }
            _ => {} // already in the requested state
        }
    }

    pub fn set_bpm(&mut self, bpm: f64) {
        if bpm > 0.0 && (bpm - self.bpm).abs() > 0.01 {
            self.bpm = bpm;
            self.retune_delay();
        }
    }

    pub fn set_beat_value(&mut self, beat_value: f64) {
        if beat_value > 0.0 && (beat_value - self.beat_value).abs() > 1e-9 {
            self.beat_value = beat_value;
            self.retune_delay();
        }
    }

    pub fn set_depth(&mut self, depth: f64) {
        self.depth_target = depth.clamp(0.0, 1.0);
    }

    pub fn set_feedback(&mut self, feedback: f64) {
        self.feedback_target = feedback.clamp(0.0, 0.9);
    }

    fn retune_delay(&mut self) {
        let raw = (60.0 / self.bpm) * self.beat_value * self.sample_rate;
        let clamped = raw.clamp(1.0, (self.buf_frames - 1) as f64);
        if (clamped - self.current_delay).abs() < 1.0 {
            return;
        }
        self.prev_delay = self.current_delay;
        self.current_delay = clamped;
        self.crossfade_remaining = self.crossfade_total;
    }

    pub fn process_sample(&mut self, sample: &mut [f64; 2]) {
        if !self.enabled && !self.draining {
            return;
        }

        let feedback = self.feedback_smooth.process(self.feedback_target);
        let wet = self.read_delayed();

        if self.draining {
            // Feed silence into the buffer — feedback loop decays each repeat by
            // the feedback coefficient until the tail falls below -80 dB.
            let fb_l = (feedback * wet[0]) * 0.8;
            let fb_r = (feedback * wet[1]) * 0.8;
            self.buffer[self.write_idx] = [fb_l.tanh() / 0.8, fb_r.tanh() / 0.8];
            self.write_idx = (self.write_idx + 1) % self.buf_frames;
            if wet[0].abs() < 1e-4 && wet[1].abs() < 1e-4 {
                self.draining = false;
                return;
            }
            // Add decaying tail to the dry signal; dry passes through unchanged.
            sample[0] += wet[0];
            sample[1] += wet[1];
            return;
        }

        let depth = self.depth_smooth.process(self.depth_target);

        // Apply pre-filter to the signal going into the delay buffer.
        // Dry output (sample) is always unfiltered — only the echoed repeats
        // carry the filtered content.
        let mut write_sig = *sample;
        if self.filter_mode != EchoFilter::None {
            if let Some(ref hp) = self.pre_hp {
                write_sig[0] = hp.process(write_sig[0], &mut self.pre_hp_states[0]);
                write_sig[1] = hp.process(write_sig[1], &mut self.pre_hp_states[1]);
            }
            if let Some(ref lp) = self.pre_lp {
                write_sig[0] = lp.process(write_sig[0], &mut self.pre_lp_states[0]);
                write_sig[1] = lp.process(write_sig[1], &mut self.pre_lp_states[1]);
            }
        }

        // Feedback: write (filtered_input + feedback * wet) with soft saturation.
        // tanh(x * 0.8) / 0.8 has unity gain near 0 and saturates at ±1.25.
        let fb_l = (write_sig[0] + feedback * wet[0]) * 0.8;
        let fb_r = (write_sig[1] + feedback * wet[1]) * 0.8;
        self.buffer[self.write_idx] = [fb_l.tanh() / 0.8, fb_r.tanh() / 0.8];
        self.write_idx = (self.write_idx + 1) % self.buf_frames;

        // Asymmetric wet/dry: dry flat until 0.5 then drops; wet ramps to 1.0 by 0.5 then pins.
        // At 0.5 both are 1.0 (wet+dry = 2.0) — past-center "wet louder than dry" feel.
        let (dry_gain, wet_gain) = if depth < 0.5 {
            (1.0, 2.0 * depth)
        } else {
            (2.0 * (1.0 - depth), 1.0)
        };
        sample[0] = sample[0] * dry_gain + wet[0] * wet_gain;
        sample[1] = sample[1] * dry_gain + wet[1] * wet_gain;
    }

    fn read_delayed(&mut self) -> [f64; 2] {
        let current = self.read_tap(self.current_delay);
        if self.crossfade_remaining > 0 {
            let prev = self.read_tap(self.prev_delay);
            let t = 1.0 - (self.crossfade_remaining as f64 / self.crossfade_total as f64);
            self.crossfade_remaining -= 1;
            [
                prev[0] * (1.0 - t) + current[0] * t,
                prev[1] * (1.0 - t) + current[1] * t,
            ]
        } else {
            current
        }
    }

    fn read_tap(&self, delay_frames: f64) -> [f64; 2] {
        let buf_len_f = self.buf_frames as f64;
        let mut read_pos = self.write_idx as f64 - delay_frames;
        while read_pos < 0.0 {
            read_pos += buf_len_f;
        }
        let idx_floor = (read_pos.floor() as usize) % self.buf_frames;
        let idx_ceil = (idx_floor + 1) % self.buf_frames;
        let frac = read_pos - read_pos.floor();
        let a = self.buffer[idx_floor];
        let b = self.buffer[idx_ceil];
        [a[0] * (1.0 - frac) + b[0] * frac, a[1] * (1.0 - frac) + b[1] * frac]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f64 = 48_000.0;

    fn armed_echo(beat_value: f64, feedback: f64) -> Echo {
        let mut e = Echo::new(SR);
        e.set_beat_value(beat_value);
        e.set_feedback(feedback);
        e.set_depth(0.8);
        e.set_enabled(true);
        e
    }

    fn fill(echo: &mut Echo, frames: usize) {
        for _ in 0..frames {
            let mut s = [1.0_f64, 1.0_f64];
            echo.process_sample(&mut s);
        }
    }

    #[test]
    fn disarm_enters_drain_not_hard_bypass() {
        let mut e = armed_echo(0.25, 0.5);
        fill(&mut e, 2400);
        e.set_enabled(false);
        assert!(!e.enabled);
        assert!(e.draining, "expected draining=true after disarm");
    }

    #[test]
    fn rearm_clears_buffer() {
        // Fill buffer with loud audio, arm off, arm on — first output must be silent.
        let mut e = armed_echo(0.25, 0.5);
        fill(&mut e, 4800);
        e.set_enabled(false);
        e.set_enabled(true);
        assert!(!e.draining);
        let mut s = [0.0_f64, 0.0_f64];
        e.process_sample(&mut s);
        assert!(
            s[0].abs() < 1e-9 && s[1].abs() < 1e-9,
            "stale buffer content after re-arm: {s:?}"
        );
    }

    #[test]
    fn tail_drains_to_silence() {
        // 0.1-beat delay at 120 BPM = 0.05 s = 2400 frames. With feedback=0.5,
        // amplitude halves every repeat → below 1e-4 in ~13 repeats (~31 200 frames).
        let mut e = armed_echo(0.1, 0.5);
        fill(&mut e, 2400);
        e.set_enabled(false);

        let limit = SR as usize * 5; // 5 seconds worst-case
        for _ in 0..limit {
            let mut s = [0.0_f64, 0.0_f64];
            e.process_sample(&mut s);
            if !e.draining {
                return; // drain completed cleanly
            }
        }
        panic!("echo tail never drained within 5 s");
    }

    #[test]
    fn dry_passes_through_during_drain() {
        // While draining the dry signal should be audible (tail adds to it).
        let mut e = armed_echo(0.1, 0.5);
        fill(&mut e, 2400);
        e.set_enabled(false);

        // Immediately after disarm: draining=true, process one frame of loud dry audio.
        let mut s = [1.0_f64, 1.0_f64];
        e.process_sample(&mut s);
        // Dry (1.0) + tail → output > 1.0, never zero.
        assert!(s[0] > 0.5, "dry signal blocked during drain: {}", s[0]);
    }
}

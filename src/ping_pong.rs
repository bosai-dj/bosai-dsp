//! Beat FX: Ping Pong delay (Pioneer DJM-900NXS2 style, locked variant).
//!
//! A mono delay buffer is read at two offsets — left channel at `beat_value`
//! (one bounce), right channel at `beat_value × 2` (two bounces). Cross-feed
//! from the right tap back into the write path creates successive L→R→L→R echoes
//! that decay via the feedback coefficient.
//!
//! The dry signal retains its original stereo image — only the wet taps are
//! hard-panned. At depth < 0.5 the dry is louder; at depth > 0.5 the wet
//! overtakes it (same asymmetric curve as the standard echo).
//!
//! Beat value changes trigger a 20 ms crossfade on both taps simultaneously so
//! the bounce timing updates cleanly without a click.

use super::smoother::Smoother;

const MAX_DELAY_S: f64 = 4.0;
const CROSSFADE_S: f64 = 0.020;
const SMOOTH_S: f64 = 0.020;

pub struct PingPong {
    pub enabled: bool,
    draining: bool,
    sample_rate: f64,

    /// Mono delay buffer — input is summed to mono before writing.
    buffer: Vec<f64>,
    buf_frames: usize,
    write_idx: usize,

    /// Left tap delay in frames (= beat_value * samples_per_beat).
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
}

impl PingPong {
    pub fn new(sample_rate: f64) -> Self {
        let buf_frames = (MAX_DELAY_S * 2.0 * sample_rate) as usize; // 2× for right tap headroom
        let crossfade_total = (CROSSFADE_S * sample_rate) as usize;
        let bpm = 120.0;
        let beat_value = 1.0;
        let initial_delay = (60.0 / bpm) * beat_value * sample_rate;
        Self {
            enabled: false,
            draining: false,
            sample_rate,
            buffer: vec![0.0; buf_frames],
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
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        match (self.enabled, enabled) {
            (false, true) => {
                self.buffer.fill(0.0);
                self.write_idx = 0;
                self.crossfade_remaining = 0;
                self.draining = false;
                self.enabled = true;
            }
            (true, false) => {
                self.draining = true;
                self.enabled = false;
            }
            _ => {}
        }
    }

    pub fn set_bpm(&mut self, bpm: f64) {
        if bpm > 0.0 && (bpm - self.bpm).abs() > 0.01 {
            self.bpm = bpm;
            self.retune();
        }
    }

    pub fn set_beat_value(&mut self, beat_value: f64) {
        if beat_value > 0.0 && (beat_value - self.beat_value).abs() > 1e-9 {
            self.beat_value = beat_value;
            self.retune();
        }
    }

    pub fn set_depth(&mut self, depth: f64) {
        self.depth_target = depth.clamp(0.0, 1.0);
    }

    pub fn set_feedback(&mut self, feedback: f64) {
        self.feedback_target = feedback.clamp(0.0, 0.9);
    }

    fn retune(&mut self) {
        let raw = (60.0 / self.bpm) * self.beat_value * self.sample_rate;
        // Right tap is at 2× — clamp so it fits in the buffer.
        let clamped = raw.clamp(1.0, (self.buf_frames / 2 - 1) as f64);
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
        let [left_wet, right_wet] = self.read_taps();

        if self.draining {
            // Advance depth smoother toward 0 so re-arm starts clean (no snap).
            self.depth_smooth.process(0.0);
            let fb = (feedback * right_wet) * 0.8;
            self.buffer[self.write_idx] = fb.tanh() / 0.8;
            self.write_idx = (self.write_idx + 1) % self.buf_frames;
            if left_wet.abs() < 1e-4 && right_wet.abs() < 1e-4 {
                self.draining = false;
                return;
            }
            sample[0] += left_wet;
            sample[1] += right_wet;
            return;
        }

        let depth = self.depth_smooth.process(self.depth_target);

        // Sum to mono — only the wet path is stereo (hard-panned taps).
        let mono_in = (sample[0] + sample[1]) * 0.5;

        // Write mono + cross-feed from right tap. The right tap feeds back so
        // the signal bounces: write→left(T)→right(2T)→write→left(3T)→…
        let fb_in = (mono_in + feedback * right_wet) * 0.8;
        self.buffer[self.write_idx] = fb_in.tanh() / 0.8;
        self.write_idx = (self.write_idx + 1) % self.buf_frames;

        // Asymmetric wet/dry: same curve as Echo.
        let (dry_gain, wet_gain) = if depth < 0.5 {
            (1.0, 2.0 * depth)
        } else {
            (2.0 * (1.0 - depth), 1.0)
        };

        // Dry retains stereo; wet is hard-panned L/R.
        sample[0] = sample[0] * dry_gain + left_wet * wet_gain;
        sample[1] = sample[1] * dry_gain + right_wet * wet_gain;
    }

    /// Returns [left_wet, right_wet] with crossfade applied if retuning.
    fn read_taps(&mut self) -> [f64; 2] {
        let left_cur  = self.read_mono(self.current_delay);
        let right_cur = self.read_mono(self.current_delay * 2.0);

        if self.crossfade_remaining > 0 {
            let left_prev  = self.read_mono(self.prev_delay);
            let right_prev = self.read_mono(self.prev_delay * 2.0);
            let t = 1.0 - (self.crossfade_remaining as f64 / self.crossfade_total as f64);
            self.crossfade_remaining -= 1;
            [
                left_prev  * (1.0 - t) + left_cur  * t,
                right_prev * (1.0 - t) + right_cur * t,
            ]
        } else {
            [left_cur, right_cur]
        }
    }

    fn read_mono(&self, delay_frames: f64) -> f64 {
        let buf_len_f = self.buf_frames as f64;
        let mut pos = self.write_idx as f64 - delay_frames;
        while pos < 0.0 {
            pos += buf_len_f;
        }
        let idx_lo = (pos.floor() as usize) % self.buf_frames;
        let idx_hi = (idx_lo + 1) % self.buf_frames;
        let frac = pos - pos.floor();
        self.buffer[idx_lo] * (1.0 - frac) + self.buffer[idx_hi] * frac
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f64 = 44100.0;

    fn armed(beat_value: f64, feedback: f64) -> PingPong {
        let mut p = PingPong::new(SR);
        p.set_bpm(120.0);
        p.set_beat_value(beat_value);
        p.set_feedback(feedback);
        p.set_depth(1.0);
        p.set_enabled(true);
        p
    }

    fn fill(pp: &mut PingPong, frames: usize, val: f64) {
        for _ in 0..frames {
            let mut s = [val, val];
            pp.process_sample(&mut s);
        }
    }

    /// Frames of history needed to cover one `beat_value` delay.
    ///
    /// Rounds **up**: the implementation keeps delay as a fractional frame count
    /// (5512.5 at 120 BPM / 0.25 beats), so truncating here would under-fill the
    /// buffer — and the right tap, at 2x, would under-fill by a whole frame and
    /// read unwritten silence.
    fn beat_frames(bpm: f64, beat_value: f64, sr: f64) -> usize {
        ((60.0 / bpm) * beat_value * sr).ceil() as usize
    }

    #[test]
    fn disabled_passes_through() {
        let mut p = PingPong::new(SR);
        p.set_depth(1.0);
        let mut s = [0.7_f64, 0.3_f64];
        p.process_sample(&mut s);
        assert_eq!(s, [0.7, 0.3]);
    }

    #[test]
    fn left_tap_fires_at_beat_value() {
        let bv = 0.25;
        let n = beat_frames(120.0, bv, SR);
        let mut p = armed(bv, 0.0); // no feedback so only first taps
        // Fill with signal for exactly n frames (= left tap delay)
        fill(&mut p, n, 0.5);
        // Next sample: left tap should read the content written n frames ago.
        let mut s = [0.0_f64, 0.0_f64];
        p.process_sample(&mut s);
        assert!(s[0].abs() > 0.1, "left tap should fire at beat_value; got {}", s[0]);
    }

    #[test]
    fn right_tap_fires_at_double_beat_value() {
        let bv = 0.25;
        let n = beat_frames(120.0, bv, SR);
        let mut p = armed(bv, 0.0);
        // Fill for 2n frames (= right tap delay)
        fill(&mut p, 2 * n, 0.5);
        let mut s = [0.0_f64, 0.0_f64];
        p.process_sample(&mut s);
        assert!(s[1].abs() > 0.1, "right tap should fire at 2×beat_value; got {}", s[1]);
    }

    #[test]
    fn left_fires_before_right() {
        let bv = 0.25;
        let n = beat_frames(120.0, bv, SR);
        let mut p = armed(bv, 0.0);

        // At exactly n frames: left should have content, right should not yet.
        fill(&mut p, n, 0.5);
        let mut s = [0.0_f64, 0.0_f64];
        p.process_sample(&mut s);
        let left_at_n = s[0];
        let right_at_n = s[1];

        assert!(left_at_n.abs() > 0.05, "left should fire at T; got {}", left_at_n);
        assert!(right_at_n.abs() < left_at_n.abs(),
            "right should not yet match left at T; left={left_at_n} right={right_at_n}");
    }

    #[test]
    fn disarm_enters_drain() {
        let bv = 0.25;
        let n = beat_frames(120.0, bv, SR);
        let mut p = armed(bv, 0.5);
        fill(&mut p, n * 2, 0.5);
        p.set_enabled(false);
        assert!(!p.enabled);
        assert!(p.draining, "disarm must enter drain mode");
    }

    #[test]
    fn rearm_clears_buffer() {
        let bv = 0.25;
        let n = beat_frames(120.0, bv, SR);
        let mut p = armed(bv, 0.5);
        fill(&mut p, n * 3, 0.9);
        p.set_enabled(false);
        p.set_enabled(true);
        let mut s = [0.0_f64, 0.0_f64];
        p.process_sample(&mut s);
        assert!(s[0].abs() < 1e-9 && s[1].abs() < 1e-9,
            "buffer should be cleared on re-arm: {s:?}");
    }

    #[test]
    fn dry_retains_stereo() {
        // With depth=0 the output should be the dry signal unchanged.
        let mut p = armed(1.0, 0.0);
        p.set_depth(0.0);
        let mut s = [0.8_f64, 0.2_f64];
        p.process_sample(&mut s);
        assert!((s[0] - 0.8).abs() < 1e-6, "dry L should be unchanged; got {}", s[0]);
        assert!((s[1] - 0.2).abs() < 1e-6, "dry R should be unchanged; got {}", s[1]);
    }
}

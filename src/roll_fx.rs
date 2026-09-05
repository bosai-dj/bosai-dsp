//! Beat FX: Roll, Slip Roll, Rev Roll, Helix.
//!
//! All four variants capture audio into a buffer of `beat_value` length and
//! loop it while the effect is active. The slip mechanism is inherent in the
//! architecture: the deck's read_pos always advances in the background, so
//! on deactivation the output instantly reverts to where the track would have
//! been — no seek required.
//!
//! ## Variants
//!
//! **Roll** — Buffer is locked at the moment of activation. Changing beat_value
//! moves the loop window within the already-captured audio (tighter/wider loop).
//! The captured content never changes mid-effect.
//!
//! **Slip Roll** — Re-captures from the current playback position whenever
//! `beat_value` changes. The buffer is always fresh from the current moment,
//! not the activation point.
//!
//! **Rev Roll** — Identical to Roll except the read head traverses the buffer
//! backwards: frame N of the loop plays buffer[buf_len - 1 - N].
//!
//! **Helix** — Layers the looped buffer additively on top of the live signal
//! rather than replacing it. Output = live + (loop × depth). Creates a
//! phase-offset doubling that builds with each repeat.
//!
//! ## Capture → Loop transition
//!
//! While the buffer is being filled (`captured = false`) the live signal passes
//! through unchanged. Once `buf_len_frames` frames are written, the loop begins.
//! The read head starts at frame 0, so the first loop repeat is identical to
//! what was just heard — no discontinuity at the transition.
//!
//! ## Beat value changes
//!
//! Roll/RevRoll/Helix: `buf_len_frames` is updated. The loop window shrinks or
//! grows within the existing captured audio; `read_frame` is clamped. Cannot
//! grow beyond `captured_frames` (no stale content read).
//!
//! Slip Roll: resets write_pos and restarts capture from the current position.

use super::smoother::Smoother;

const MAX_ROLL_SECS: f64 = 16.0; // 8 beats at 30 BPM, 4 beats at 60 BPM
const SMOOTH_DEPTH_S: f64 = 0.010; // 10 ms — fast crossfade on arm/disarm

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum RollMode {
    Roll,
    SlipRoll,
    RevRoll,
    Helix,
}

pub struct RollFx {
    pub enabled: bool,
    pub mode: RollMode,
    bpm: f64,
    beat_value: f64,
    /// Current loop window in frames. May be < captured_frames if beat_value shrank.
    buf_len_frames: usize,
    /// How many frames are valid in `buffer`. Equals buf_len_frames once captured.
    captured_frames: usize,
    buffer: Vec<[f32; 2]>,
    max_frames: usize,
    write_pos: usize,
    /// True once the initial capture fill is complete.
    captured: bool,
    /// Current read position within the loop (wraps at buf_len_frames).
    read_frame: usize,
    depth_target: f64,
    depth_smooth: Smoother,
    sample_rate: f64,
}

impl RollFx {
    pub fn new(sample_rate: f64) -> Self {
        let max_frames = (MAX_ROLL_SECS * sample_rate) as usize;
        let initial_buf_len = ((60.0 / 120.0) * sample_rate) as usize; // 1 beat at 120 BPM
        Self {
            enabled: false,
            mode: RollMode::Roll,
            bpm: 120.0,
            beat_value: 1.0,
            buf_len_frames: initial_buf_len,
            captured_frames: 0,
            buffer: vec![[0.0; 2]; max_frames],
            max_frames,
            write_pos: 0,
            captured: false,
            read_frame: 0,
            depth_target: 1.0,
            depth_smooth: Smoother::new(sample_rate, SMOOTH_DEPTH_S),
            sample_rate,
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        if enabled && !self.enabled {
            // Fresh capture on every arm.
            self.write_pos = 0;
            self.captured = false;
            self.captured_frames = 0;
            self.read_frame = 0;
            self.recompute_buf_len();
            // Snap smoother to target — avoids a ramp-from-0 artifact on arm.
            self.depth_smooth.reset(self.depth_target);
        }
        self.enabled = enabled;
    }

    pub fn set_mode(&mut self, mode: RollMode) {
        self.mode = mode;
    }

    pub fn set_depth(&mut self, depth: f64) {
        self.depth_target = depth.clamp(0.0, 1.0);
    }

    /// Called from the deck render loop (same as Echo / FilterFx).
    pub fn set_bpm(&mut self, bpm: f64) {
        if bpm > 0.0 && (bpm - self.bpm).abs() > 0.01 {
            self.bpm = bpm;
            // Don't resize the buffer mid-loop — it would disrupt the current capture.
        }
    }

    pub fn set_beat_value(&mut self, beat_value: f64) {
        if beat_value <= 0.0 || (beat_value - self.beat_value).abs() < 1e-9 {
            return;
        }
        self.beat_value = beat_value;

        match self.mode {
            RollMode::SlipRoll => {
                // Re-capture from the current live position.
                self.write_pos = 0;
                self.captured = false;
                self.captured_frames = 0;
                self.read_frame = 0;
                self.recompute_buf_len();
            }
            _ => {
                // Roll / RevRoll / Helix: adjust the loop window within the
                // existing captured buffer. Cannot grow beyond captured_frames.
                self.recompute_buf_len();
                if self.captured {
                    // Clamp loop window to what actually exists in the buffer.
                    self.buf_len_frames = self.buf_len_frames.min(self.captured_frames);
                    // Wrap read_frame into the new window.
                    if self.buf_len_frames > 0 {
                        self.read_frame %= self.buf_len_frames;
                    }
                }
            }
        }
    }

    fn recompute_buf_len(&mut self) {
        let frames = ((60.0 / self.bpm) * self.beat_value * self.sample_rate) as usize;
        self.buf_len_frames = frames.clamp(1, self.max_frames);
    }

    /// Process one output frame.
    ///
    /// `live` is the deck's fully-processed audio for this frame (post-fader,
    /// post-EQ, post-echo). Returns the frame that should go to speakers.
    pub fn process_frame(&mut self, live: [f32; 2]) -> [f32; 2] {
        if !self.enabled {
            return live;
        }

        let depth = self.depth_smooth.process(self.depth_target) as f32;

        if !self.captured {
            // Capture phase: fill the buffer with live audio.
            if self.write_pos < self.buf_len_frames {
                self.buffer[self.write_pos] = live;
                self.write_pos += 1;
                self.captured_frames = self.write_pos;
            }
            if self.write_pos >= self.buf_len_frames {
                self.captured = true;
                self.read_frame = 0;
            }
            // Pass live audio through during capture (user still hears the track).
            return live;
        }

        // Loop phase: read from buffer.
        let loop_frame = if self.buf_len_frames == 0 {
            live
        } else {
            let buf_pos = self.read_frame % self.buf_len_frames;
            match self.mode {
                RollMode::RevRoll => {
                    // Read backward: frame 0 = last captured, frame N-1 = first.
                    let rev_pos = self.buf_len_frames.saturating_sub(1) - buf_pos;
                    self.buffer[rev_pos]
                }
                _ => self.buffer[buf_pos],
            }
        };

        // Advance read head, wrap at loop boundary.
        if self.buf_len_frames > 0 {
            self.read_frame = (self.read_frame + 1) % self.buf_len_frames;
        }

        match self.mode {
            RollMode::Helix => {
                // Additive: live + loop × depth. Both heard simultaneously.
                // Clamp to prevent clipping on accumulation.
                [
                    (live[0] + loop_frame[0] * depth).clamp(-1.0, 1.0),
                    (live[1] + loop_frame[1] * depth).clamp(-1.0, 1.0),
                ]
            }
            _ => {
                // Roll / SlipRoll / RevRoll: crossfade between loop and live.
                let dry = 1.0 - depth;
                [
                    loop_frame[0] * depth + live[0] * dry,
                    loop_frame[1] * depth + live[1] * dry,
                ]
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f64 = 44100.0;

    fn armed_roll(mode: RollMode, beat_value: f64, depth: f64) -> RollFx {
        let mut r = RollFx::new(SR);
        r.set_mode(mode);
        r.set_bpm(120.0);
        r.set_beat_value(beat_value);
        r.set_depth(depth);
        r.set_enabled(true);
        r
    }

    fn buf_len_frames(bpm: f64, beat_value: f64, sr: f64) -> usize {
        ((60.0 / bpm) * beat_value * sr) as usize
    }

    fn fill(roll: &mut RollFx, frames: usize, value: f32) {
        for _ in 0..frames {
            roll.process_frame([value, value]);
        }
    }

    #[test]
    fn passes_through_during_capture() {
        let mut r = armed_roll(RollMode::Roll, 0.25, 1.0);
        let out = r.process_frame([0.5, 0.5]);
        // Still in capture phase — must pass through.
        assert_eq!(out, [0.5, 0.5], "should pass through during capture");
    }

    #[test]
    fn loops_after_capture() {
        let n = buf_len_frames(120.0, 0.25, SR);
        let mut r = armed_roll(RollMode::Roll, 0.25, 1.0);
        // Fill buffer with a recognisable value.
        fill(&mut r, n, 0.8);
        // Now in loop mode. Read frame 0 should return buffer[0] = 0.8.
        let out = r.process_frame([0.0, 0.0]); // live is 0.0; depth=1.0 → pure loop
        assert!(
            (out[0] - 0.8).abs() < 1e-5,
            "expected loop frame 0.8, got {}", out[0]
        );
    }

    #[test]
    fn disabled_passes_through() {
        let mut r = RollFx::new(SR);
        r.set_depth(1.0);
        // Not enabled — must pass through regardless.
        let out = r.process_frame([0.7, 0.3]);
        assert_eq!(out, [0.7, 0.3]);
    }

    #[test]
    fn slip_roll_recaptures_on_beat_value_change() {
        let n = buf_len_frames(120.0, 0.25, SR);
        let mut r = armed_roll(RollMode::SlipRoll, 0.25, 1.0);
        fill(&mut r, n, 0.5);
        assert!(r.captured, "should be captured after filling buffer");
        // Change beat_value — slip roll must reset.
        r.set_beat_value(0.5);
        assert!(!r.captured, "slip roll must re-capture on beat_value change");
        assert_eq!(r.write_pos, 0);
    }

    #[test]
    fn roll_does_not_recapture_on_beat_value_change() {
        let n = buf_len_frames(120.0, 0.5, SR);
        let mut r = armed_roll(RollMode::Roll, 0.5, 1.0);
        fill(&mut r, n, 0.5);
        assert!(r.captured);
        // Change beat_value — Roll must NOT re-capture.
        r.set_beat_value(0.25);
        assert!(r.captured, "roll must not recapture on beat_value change");
    }

    #[test]
    fn rev_roll_reads_backward() {
        let n = buf_len_frames(120.0, 0.25, SR);
        let mut r = armed_roll(RollMode::RevRoll, 0.25, 1.0);
        // Fill buffer: frame 0 = 0.1, frame 1 = 0.2, ..., frame n-1 = 0.1 * n.
        // (Use a sinusoid to distinguish forward from backward.)
        for i in 0..n {
            let v = ((i as f32 + 1.0) * 0.001).min(1.0);
            r.process_frame([v, v]);
        }
        // Now in loop mode. First output should be buffer[n-1] (last captured), not buffer[0].
        let last_captured_val = ((n as f32) * 0.001).min(1.0);
        let out = r.process_frame([0.0, 0.0]);
        assert!(
            (out[0] - last_captured_val).abs() < 1e-4,
            "rev roll frame 0 should be buffer[n-1]={last_captured_val:.4}, got {:.4}", out[0]
        );
    }

    #[test]
    fn helix_adds_loop_to_live() {
        let n = buf_len_frames(120.0, 0.25, SR);
        let mut r = armed_roll(RollMode::Helix, 0.25, 1.0);
        fill(&mut r, n, 0.3);
        // In loop mode with depth=1.0: output = live + loop_frame.
        let out = r.process_frame([0.1, 0.1]); // live=0.1, loop≈0.3
        // Helix depth_smooth needs a few samples to settle; just check > live.
        assert!(
            out[0] > 0.1,
            "helix should add loop on top of live; live=0.1, got {}", out[0]
        );
    }

    #[test]
    fn roll_shrinks_loop_window_on_beat_value_change() {
        let n = buf_len_frames(120.0, 1.0, SR);
        let mut r = armed_roll(RollMode::Roll, 1.0, 1.0);
        fill(&mut r, n, 0.5);
        assert!(r.captured);
        let original_captured = r.captured_frames;
        // Halve beat_value — loop window should shrink; captured content unchanged.
        r.set_beat_value(0.5);
        let half_n = buf_len_frames(120.0, 0.5, SR);
        assert_eq!(r.buf_len_frames, half_n.min(original_captured));
        assert!(r.captured, "captured flag must stay true");
    }

    #[test]
    fn re_arm_resets_capture() {
        let n = buf_len_frames(120.0, 0.25, SR);
        let mut r = armed_roll(RollMode::Roll, 0.25, 1.0);
        fill(&mut r, n, 0.5);
        assert!(r.captured);
        r.set_enabled(false);
        r.set_enabled(true);
        assert!(!r.captured, "re-arm must reset capture");
        assert_eq!(r.write_pos, 0);
    }
}

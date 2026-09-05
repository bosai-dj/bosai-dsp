//! Beat FX: Flanger.
//!
//! A very short variable delay line (0.5–5 ms) modulated by a sine LFO, summed
//! with the dry signal. The phase relationship between dry and delayed creates a
//! comb filter whose notches sweep up and down as the delay varies — the classic
//! "jet engine" sweep.
//!
//! Beat value → LFO period: `lfo_rate_hz = bpm / (60 × beat_value)`.
//! One full LFO cycle = beat_value beats.
//!
//! Depth doubles as wet/dry AND feedback amount:
//!   depth = 0 → dry pass-through (delay line still running but not mixed in)
//!   depth = 1 → full wet mix + max feedback (≤ 0.75, always stable)
//!
//! Implementation uses a stereo circular buffer with linear interpolation so
//! the non-integer delay lengths the LFO produces don't alias.

use super::smoother::Smoother;
use std::f64::consts::TAU;

const MIN_DELAY_MS: f64 = 0.5;
const MAX_DELAY_MS: f64 = 5.0;
const MAX_FEEDBACK: f64 = 0.75;
const SMOOTH_S: f64 = 0.015;

pub struct Flanger {
    pub enabled: bool,
    sample_rate: f64,

    buffer: Vec<[f64; 2]>,
    buf_frames: usize,
    write_idx: usize,

    lfo_phase: f64,
    lfo_rate_hz: f64,
    bpm: f64,
    beat_value: f64,

    depth_target: f64,
    depth_smooth: Smoother,

    min_delay: f64, // frames
    max_delay: f64, // frames
}

impl Flanger {
    pub fn new(sample_rate: f64) -> Self {
        let min_delay = MIN_DELAY_MS * 0.001 * sample_rate;
        let max_delay = MAX_DELAY_MS * 0.001 * sample_rate;
        // Buffer needs to hold max_delay + 2 guard frames for interpolation.
        let buf_frames = (max_delay as usize + 4).next_power_of_two();
        let bpm = 120.0;
        let beat_value = 1.0;
        Self {
            enabled: false,
            sample_rate,
            buffer: vec![[0.0; 2]; buf_frames],
            buf_frames,
            write_idx: 0,
            lfo_phase: 0.0,
            lfo_rate_hz: bpm / (60.0 * beat_value),
            bpm,
            beat_value,
            depth_target: 0.5,
            depth_smooth: Smoother::new(sample_rate, SMOOTH_S),
            min_delay,
            max_delay,
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        if enabled && !self.enabled {
            self.buffer.fill([0.0; 2]);
            self.write_idx = 0;
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
        let feedback = depth * MAX_FEEDBACK;

        // Sine LFO: maps to delay range [min, max].
        self.lfo_phase += self.lfo_rate_hz / self.sample_rate;
        if self.lfo_phase >= 1.0 {
            self.lfo_phase -= 1.0;
        }
        let lfo = (TAU * self.lfo_phase).sin(); // -1..1
        let delay = self.min_delay + (self.max_delay - self.min_delay) * 0.5 * (1.0 + lfo);

        let wet = self.read_interp(delay);

        // Write: input + feedback × delayed (feedback comb).
        let write = [
            (sample[0] + feedback * wet[0]).tanh(),
            (sample[1] + feedback * wet[1]).tanh(),
        ];
        self.buffer[self.write_idx] = write;
        self.write_idx = (self.write_idx + 1) % self.buf_frames;

        // Output: feedforward comb — dry + wet × depth.
        sample[0] += wet[0] * depth;
        sample[1] += wet[1] * depth;
    }

    fn read_interp(&self, delay: f64) -> [f64; 2] {
        let buf_len = self.buf_frames as f64;
        let mut pos = self.write_idx as f64 - delay;
        if pos < 0.0 {
            pos += buf_len;
        }
        let idx_lo = pos.floor() as usize % self.buf_frames;
        let idx_hi = (idx_lo + 1) % self.buf_frames;
        let frac = pos - pos.floor();
        [
            self.buffer[idx_lo][0] * (1.0 - frac) + self.buffer[idx_hi][0] * frac,
            self.buffer[idx_lo][1] * (1.0 - frac) + self.buffer[idx_hi][1] * frac,
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f64 = 44100.0;

    fn armed(beat_value: f64, depth: f64) -> Flanger {
        let mut f = Flanger::new(SR);
        f.set_bpm(120.0);
        f.set_beat_value(beat_value);
        f.set_depth(depth);
        f.set_enabled(true);
        f
    }

    #[test]
    fn disabled_passes_through() {
        let mut f = Flanger::new(SR);
        f.set_depth(1.0);
        let mut s = [0.5_f64, 0.3_f64];
        f.process_sample(&mut s);
        assert_eq!(s, [0.5, 0.3]);
    }

    #[test]
    fn enable_clears_buffer() {
        let mut f = armed(1.0, 0.8);
        // Flood with signal
        for _ in 0..500 { let mut s = [1.0_f64; 2]; f.process_sample(&mut s); }
        f.set_enabled(false);
        f.set_enabled(true);
        // Buffer should be zeroed — immediately after arm, wet tap is silent.
        // With depth=0 we can't measure wet directly; check write position reset.
        assert_eq!(f.write_idx, 0);
        for samp in &f.buffer {
            assert!(samp[0].abs() < 1e-12 && samp[1].abs() < 1e-12);
        }
    }

    #[test]
    fn lfo_modulates_output_over_time() {
        let mut f = armed(0.125, 1.0); // fast LFO: 120/(60*0.125) = 16 Hz
        // Let smoother settle
        for _ in 0..2000 { let mut s = [1.0_f64; 2]; f.process_sample(&mut s); }

        // Collect RMS at two LFO half-cycles.
        let rms = |fx: &mut Flanger, n: usize| {
            let mut e = 0.0;
            for _ in 0..n { let mut s = [1.0_f64; 2]; fx.process_sample(&mut s); e += s[0]*s[0]; }
            (e / n as f64).sqrt()
        };
        let half = (SR / (16.0 * 2.0)) as usize; // half LFO period
        let a = rms(&mut f, 256);
        for _ in 0..half { let mut s = [1.0_f64; 2]; f.process_sample(&mut s); }
        let b = rms(&mut f, 256);
        assert!((a - b).abs() > 0.01, "LFO should modulate; rms_a={a:.4} rms_b={b:.4}");
    }

    #[test]
    fn delay_stays_in_range() {
        // Verify the delay read position never falls outside [min, max].
        let mut f = armed(1.0, 0.5);
        let min = f.min_delay;
        let max = f.max_delay;
        // Step LFO through many phases and check delay range.
        for _ in 0..44100 {
            f.lfo_phase += f.lfo_rate_hz / SR;
            if f.lfo_phase >= 1.0 { f.lfo_phase -= 1.0; }
            let lfo = (std::f64::consts::TAU * f.lfo_phase).sin();
            let delay = f.min_delay + (f.max_delay - f.min_delay) * 0.5 * (1.0 + lfo);
            assert!(delay >= min - 1e-9 && delay <= max + 1e-9,
                "delay {delay:.4} outside [{min:.4}, {max:.4}]");
        }
    }
}

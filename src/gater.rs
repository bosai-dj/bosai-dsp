//! Beat FX: Gater.
//!
//! Rhythmically mutes the audio at a rate locked to the master clock.
//!
//! Each gate cycle is divided into two phases:
//!   Hold  [0 .. shape]: gain = 1.0 (audio passes through fully)
//!   Decay [shape .. 1]: gain decays exponentially to near 0
//!
//! `shape` = 0.5 → equal hold and decay (classic gate).
//! `shape` = 1.0 → gate never closes (dry pass-through).
//! `shape` = 0.0 → instant decay, maximum chop.
//!
//! A low-level white-noise fill (constant NOISE_LEVEL) blends in
//! inversely to the gate gain so the gaps are never silent — giving
//! a textured "breath" that keeps the spectral energy alive during
//! gate-off, especially useful for techno / D&B build sections.
//!
//! `depth` controls the overall effect amount:
//!   depth = 0 → dry (no gating, no noise)
//!   depth = 1 → full gating + full noise fill
//!
//! Gate rate: `gate_hz = bpm / (60 × beat_value)`.
//! Typical beat_value steps: 1.0 (1/4 bar), 0.5 (1/8), 0.25 (1/16), 0.125 (1/32).

use super::smoother::Smoother;

const NOISE_LEVEL: f64 = 0.12;   // noise amplitude during fully-closed gate
const DECAY_K: f64    = 5.0;     // exponential decay steepness
const SMOOTH_S: f64   = 0.008;   // 8 ms param smoother — snappy but click-free

pub struct Gater {
    pub enabled: bool,
    sample_rate: f64,

    phase: f64,       // gate phase 0..1
    gate_hz: f64,     // gate cycles per second
    bpm: f64,
    beat_value: f64,

    shape_target: f64,
    shape_smooth: Smoother,

    depth_target: f64,
    depth_smooth: Smoother,

    noise_seed: u64,  // xorshift64 state
}

impl Gater {
    pub fn new(sample_rate: f64) -> Self {
        let bpm = 120.0;
        let beat_value = 0.5; // default 1/8 bar — tight gating for build sections
        Self {
            enabled: false,
            sample_rate,
            phase: 0.0,
            gate_hz: bpm / (60.0 * beat_value),
            bpm,
            beat_value,
            shape_target: 0.5,
            shape_smooth: Smoother::new(sample_rate, SMOOTH_S),
            depth_target: 0.75,
            depth_smooth: Smoother::new(sample_rate, SMOOTH_S),
            noise_seed: 0xDEAD_BEEF_1234_5678,
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        if enabled && !self.enabled {
            self.phase = 0.0;
            self.shape_smooth.reset(self.shape_target);
            self.depth_smooth.reset(self.depth_target);
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
            self.update_gate_hz();
        }
    }

    pub fn set_beat_value(&mut self, beat_value: f64) {
        if beat_value > 0.0 && (beat_value - self.beat_value).abs() > 1e-9 {
            self.beat_value = beat_value;
            self.update_gate_hz();
        }
    }

    fn update_gate_hz(&mut self) {
        self.gate_hz = self.bpm / (60.0 * self.beat_value.max(1e-6));
    }

    /// xorshift64 — cheap, decent spectral flatness for noise fill.
    #[inline]
    fn next_noise(&mut self) -> f64 {
        let s = &mut self.noise_seed;
        *s ^= *s << 13;
        *s ^= *s >> 7;
        *s ^= *s << 17;
        // map u64 to -1..1
        (*s as i64 as f64) * (1.0 / i64::MAX as f64)
    }

    pub fn process_sample(&mut self, sample: &mut [f64; 2]) {
        if !self.enabled {
            return;
        }

        let depth = self.depth_smooth.process(self.depth_target);
        let shape = self.shape_smooth.process(self.shape_target);

        // Advance gate phase.
        self.phase += self.gate_hz / self.sample_rate;
        if self.phase >= 1.0 {
            self.phase -= 1.0;
        }

        // Gate envelope.
        // Snap to fully-open when shape is very close to 1.0 — avoids
        // the near-zero denominator in the decay formula as the smoother
        // approaches 1.0 from below.
        let gate_gain = if shape > 1.0 - 1e-3 || self.phase < shape {
            1.0
        } else {
            let t = (self.phase - shape) / (1.0 - shape);
            (-t * DECAY_K).exp()
        };

        // Noise fill — same sample for both channels (mono noise, sounds cleaner).
        let noise = self.next_noise() * NOISE_LEVEL;

        // Mix: at depth=0 pass through dry; at depth=1 fully gate + noise.
        // effective_gain = lerp(1.0, gate_gain, depth)
        //               = 1.0 - depth * (1.0 - gate_gain)
        let effective_gain = 1.0 - depth * (1.0 - gate_gain);
        let noise_add = noise * depth * (1.0 - gate_gain);

        sample[0] = sample[0] * effective_gain + noise_add;
        sample[1] = sample[1] * effective_gain + noise_add;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f64 = 44100.0;

    fn armed(beat_value: f64, shape: f64, depth: f64) -> Gater {
        let mut g = Gater::new(SR);
        g.set_bpm(120.0);
        g.set_beat_value(beat_value);
        g.set_shape(shape);
        g.set_depth(depth);
        g.set_enabled(true);
        g
    }

    #[test]
    fn disabled_passes_through() {
        let mut g = Gater::new(SR);
        g.set_depth(1.0);
        let mut s = [0.8_f64, 0.4_f64];
        g.process_sample(&mut s);
        assert_eq!(s, [0.8, 0.4]);
    }

    #[test]
    fn depth_zero_is_dry() {
        let mut g = armed(0.5, 0.5, 0.0);
        // Settle smoother to 0.
        for _ in 0..4000 { let mut s = [0.5_f64; 2]; g.process_sample(&mut s); }
        let mut s = [0.6_f64, 0.3_f64];
        g.process_sample(&mut s);
        assert!((s[0] - 0.6).abs() < 1e-3, "depth=0 should be dry; got {}", s[0]);
        assert!((s[1] - 0.3).abs() < 1e-3, "depth=0 should be dry; got {}", s[1]);
    }

    #[test]
    fn gate_cycles_at_correct_rate() {
        // At 120 BPM and beat_value=0.5, gate_hz = 120/(60*0.5) = 4 Hz.
        // Gate period = SR/4 = 11025 samples. Measure power over two windows
        // separated by half a period — they should differ when gating is active.
        let mut g = armed(0.5, 0.5, 1.0);
        // Let smoother settle.
        for _ in 0..4000 { let mut s = [1.0_f64; 2]; g.process_sample(&mut s); }

        let rms = |gtr: &mut Gater, n: usize| {
            let mut e = 0.0_f64;
            for _ in 0..n { let mut s = [1.0_f64; 2]; gtr.process_sample(&mut s); e += s[0]*s[0]; }
            (e / n as f64).sqrt()
        };

        let half_period = (SR / (4.0 * 2.0)) as usize; // half gate period in samples
        let a = rms(&mut g, 256);
        for _ in 0..half_period { let mut s = [1.0_f64; 2]; g.process_sample(&mut s); }
        let b = rms(&mut g, 256);
        assert!((a - b).abs() > 0.05,
            "gate should modulate power; rms_a={a:.4} rms_b={b:.4}");
    }

    #[test]
    fn shape_one_is_always_open() {
        // shape=1.0 → gate never closes → gate_gain always 1.0 → no attenuation.
        let mut g = armed(0.5, 1.0, 1.0);
        for _ in 0..4000 { let mut s = [0.5_f64; 2]; g.process_sample(&mut s); }
        let mut total_diff = 0.0_f64;
        for _ in 0..1024 {
            let mut s = [1.0_f64; 2];
            g.process_sample(&mut s);
            // With shape=1 there's no gating but noise is added at depth=1 on "closed" phase.
            // Since closed phase is 0 length, (1 - gate_gain) = 0 → noise_add = 0.
            // Output should equal dry (1.0) exactly.
            total_diff += (s[0] - 1.0).abs();
        }
        assert!(total_diff < 1e-6, "shape=1 should pass dry; total_diff={total_diff:.6}");
    }

    #[test]
    fn enable_resets_phase() {
        let mut g = armed(1.0, 0.5, 0.8);
        for _ in 0..5000 { let mut s = [0.5_f64; 2]; g.process_sample(&mut s); }
        g.set_enabled(false);
        g.set_enabled(true);
        assert!(g.phase < 1e-6, "phase should reset to 0 on re-arm");
    }

    #[test]
    fn noise_seed_advances() {
        let mut g = armed(0.25, 0.0, 1.0); // shape=0, instant close → noise fills always
        let n1 = g.next_noise();
        let n2 = g.next_noise();
        assert_ne!(n1, n2, "noise generator must advance each call");
    }

    #[test]
    fn output_bounded() {
        // With full depth and instant close (shape=0), check output stays in [-1.2, 1.2].
        let mut g = armed(0.25, 0.0, 1.0);
        for _ in 0..44100 {
            let mut s = [0.9_f64; 2];
            g.process_sample(&mut s);
            assert!(s[0].abs() <= 1.2 && s[1].abs() <= 1.2,
                "output out of range: {} {}", s[0], s[1]);
        }
    }
}

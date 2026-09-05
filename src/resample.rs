//! Resamplers for variable-speed playback and vinyl scratching.
//!
//! - `resample_linear`: fast linear interpolation for normal playback.
//! - `scratch_sample_at`: single-stage Hermite cubic sampler at an arbitrary
//!   fractional position for scratch mode — reads directly from source data.

use crate::CHANNELS;

/// Resample `src` (interleaved stereo f32) from `src_use` source frames
/// into `out` with `frames` output frames via linear interpolation.
///
/// `src` must have at least `src_use` frames. `out` must have `frames` frames.
/// Each frame is `CHANNELS` f32 values.
#[inline]
pub fn resample_linear(
    src: &[[f32; CHANNELS]],
    src_use: usize,
    out: &mut [[f32; CHANNELS]],
) {
    let frames = out.len();
    if frames == 0 || src_use < 2 {
        for frame in out.iter_mut() {
            *frame = [0.0; CHANNELS];
        }
        return;
    }

    let ratio = (src_use - 1) as f64 / (frames - 1).max(1) as f64;

    for i in 0..frames {
        let pos = i as f64 * ratio;
        let lo = pos as usize;
        let hi = (lo + 1).min(src_use - 1);
        let frac = (pos - lo as f64) as f32;

        for ch in 0..CHANNELS {
            out[i][ch] = src[lo][ch] * (1.0 - frac) + src[hi][ch] * frac;
        }
    }
}

/// 4-point Hermite interpolation coefficient.
/// Given samples y[-1], y[0], y[1], y[2] and fractional position t ∈ [0,1),
/// returns the interpolated value.
#[inline]
fn hermite(ym1: f32, y0: f32, y1: f32, y2: f32, t: f32) -> f32 {
    let c0 = y0;
    let c1 = 0.5 * (y1 - ym1);
    let c2 = ym1 - 2.5 * y0 + 2.0 * y1 - 0.5 * y2;
    let c3 = 0.5 * (y2 - ym1) + 1.5 * (y0 - y1);
    ((c3 * t + c2) * t + c1) * t + c0
}

/// Read a single stereo frame from `data` at fractional position `pos` using
/// 4-point Hermite interpolation. For scratch mode: samples directly from the
/// original decoded audio in a single stage (no extract→resample pipeline).
///
/// `len` is the total number of frames in `data`.
#[inline]
pub fn scratch_sample_at(data: &[[f32; CHANNELS]], len: usize, pos: f64) -> [f32; CHANNELS] {
    if len < 2 {
        return [0.0; CHANNELS];
    }

    let idx = pos as usize;
    let frac = (pos - idx as f64) as f32;

    let i_m1 = idx.saturating_sub(1);
    let i_0  = idx.min(len - 1);
    let i_1  = (idx + 1).min(len - 1);
    let i_2  = (idx + 2).min(len - 1);

    let mut out = [0.0_f32; CHANNELS];
    for ch in 0..CHANNELS {
        out[ch] = hermite(
            data[i_m1][ch], data[i_0][ch],
            data[i_1][ch],  data[i_2][ch],
            frac,
        );
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unity_passthrough() {
        // If src_use == frames, output should be a copy
        let src = vec![[0.5_f32, -0.5]; 10];
        let mut out = vec![[0.0_f32; CHANNELS]; 10];
        resample_linear(&src, 10, &mut out);
        for i in 0..10 {
            assert!((out[i][0] - src[i][0]).abs() < 1e-5);
            assert!((out[i][1] - src[i][1]).abs() < 1e-5);
        }
    }

    #[test]
    fn double_speed() {
        // 20 source frames → 10 output frames (2x speed)
        let src: Vec<[f32; CHANNELS]> = (0..20).map(|i| [i as f32 / 19.0; CHANNELS]).collect();
        let mut out = vec![[0.0; CHANNELS]; 10];
        resample_linear(&src, 20, &mut out);
        // First and last should match
        assert!((out[0][0] - 0.0).abs() < 1e-5);
        assert!((out[9][0] - 1.0).abs() < 1e-5);
    }

    #[test]
    fn half_speed() {
        // 5 source frames → 10 output frames (0.5x speed)
        let src: Vec<[f32; CHANNELS]> = (0..5).map(|i| [i as f32; CHANNELS]).collect();
        let mut out = vec![[0.0; CHANNELS]; 10];
        resample_linear(&src, 5, &mut out);
        assert!((out[0][0] - 0.0).abs() < 1e-5);
        assert!((out[9][0] - 4.0).abs() < 1e-5);
    }

    // ── Hermite / scratch_sample_at tests ───────────────────────────────

    #[test]
    fn hermite_exact_at_knots() {
        // Hermite should pass through source samples exactly at integer positions
        assert!((hermite(1.0, 2.0, 3.0, 4.0, 0.0) - 2.0).abs() < 1e-6);
        assert!((hermite(1.0, 2.0, 3.0, 4.0, 1.0) - 3.0).abs() < 1e-6);
    }

    #[test]
    fn hermite_linear_ramp() {
        // For a perfectly linear ramp, Hermite should reproduce it exactly
        let val = hermite(0.0, 1.0, 2.0, 3.0, 0.5);
        assert!((val - 1.5).abs() < 1e-5);
    }

    // ── scratch_sample_at tests ──────────────────────────────────────────

    #[test]
    fn scratch_sample_integer_position() {
        let data: Vec<[f32; CHANNELS]> = (0..100).map(|i| [i as f32; CHANNELS]).collect();
        let s = scratch_sample_at(&data, 100, 50.0);
        assert!((s[0] - 50.0).abs() < 1e-4);
    }

    #[test]
    fn scratch_sample_fractional() {
        // Linear ramp — Hermite should produce exact midpoint
        let data: Vec<[f32; CHANNELS]> = (0..100).map(|i| [i as f32; CHANNELS]).collect();
        let s = scratch_sample_at(&data, 100, 50.5);
        assert!((s[0] - 50.5).abs() < 1e-3);
    }

    #[test]
    fn scratch_sample_at_boundaries() {
        let data: Vec<[f32; CHANNELS]> = (0..10).map(|i| [i as f32; CHANNELS]).collect();
        // At position 0 — should not panic
        let s0 = scratch_sample_at(&data, 10, 0.0);
        assert!((s0[0] - 0.0).abs() < 1e-4);
        // Near end
        let s9 = scratch_sample_at(&data, 10, 9.0);
        assert!((s9[0] - 9.0).abs() < 1e-4);
    }
}

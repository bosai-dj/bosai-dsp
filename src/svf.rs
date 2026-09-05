//! Topology Preserving Transform (TPT) State Variable Filter.
//!
//! Both HP and LP outputs are derived from the same two integrator states
//! (ic1eq, ic2eq), so the state is preserved when switching between modes.
//! This eliminates the coefficient-discontinuity click that plagued the old
//! butterworth topology-switch approach.
//!
//! Reference: Andy Simper, "Solving the continuous SVF equations using
//! trapezoidal integration and equivalent currents" (Cytomic, 2012).
use std::f64::consts::PI;
use crate::CHANNELS;

/// Per-channel integrator state. Shared between HP and LP — never reset on mode change.
#[derive(Clone, Debug, Default)]
pub struct SvfState {
    pub ic1eq: f64,
    pub ic2eq: f64,
}

impl SvfState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset(&mut self) {
        self.ic1eq = 0.0;
        self.ic2eq = 0.0;
    }
}

/// Pre-computed TPT SVF coefficients for one cutoff frequency and Q.
pub struct SvfFilter {
    a1: f64,
    a2: f64,
    a3: f64,
    k: f64,
    pub is_hp: bool,
}

impl SvfFilter {
    /// Build SVF coefficients for `fc` Hz, quality factor `q`, at sample rate `sr`.
    pub fn new(fc: f64, q: f64, sr: f64, is_hp: bool) -> Self {
        let fc_clamped = fc.clamp(1.0, sr * 0.499);
        let g = (PI * fc_clamped / sr).tan();
        let k = 1.0 / q.max(0.001);
        let a1 = 1.0 / (1.0 + g * (g + k));
        let a2 = g * a1;
        let a3 = g * a2;
        Self { a1, a2, a3, k, is_hp }
    }

    /// Process one sample, updating integrator state in-place.
    /// Returns LP or HP output depending on `is_hp`.
    #[inline]
    pub fn process(&self, x: f64, state: &mut SvfState) -> f64 {
        let v3 = x - state.ic2eq;
        let v1 = self.a1 * state.ic1eq + self.a2 * v3;
        let v2 = state.ic2eq + self.a2 * state.ic1eq + self.a3 * v3;
        state.ic1eq = 2.0 * v1 - state.ic1eq;
        state.ic2eq = 2.0 * v2 - state.ic2eq;
        if self.is_hp {
            x - self.k * v1 - v2  // HP output
        } else {
            v2                     // LP output
        }
    }
}

/// Per-deck sweep filter band: persistent SVF state across parameter changes.
pub struct SvfBand {
    pub filter: Option<SvfFilter>,
    pub states: [SvfState; CHANNELS],
}

impl Default for SvfBand {
    fn default() -> Self {
        Self::new()
    }
}

impl SvfBand {
    pub fn new() -> Self {
        Self {
            filter: None,
            states: std::array::from_fn(|_| SvfState::new()),
        }
    }

    pub fn set(&mut self, new_filter: Option<SvfFilter>) {
        // States are intentionally NOT reset — preserving ic1eq/ic2eq across
        // parameter changes is what eliminates the click at the HP/LP crossover.
        self.filter = new_filter;
    }

    /// Process a stereo frame in-place.
    #[inline]
    pub fn process(&mut self, frame: &mut [f64; CHANNELS]) {
        if let Some(ref f) = self.filter {
            for ch in 0..CHANNELS {
                frame[ch] = f.process(frame[ch], &mut self.states[ch]);
            }
        }
    }

    pub fn reset(&mut self) {
        for s in &mut self.states {
            s.reset();
        }
        // Keep `filter` — coefficients are still valid after a track load reset.
    }
}

/// DJ sweep filter using the TPT SVF.
///
/// `val` ∈ [0, 1]: 0.0 = full HP, 0.5 = flat bypass, 1.0 = full LP.
/// Frequency follows a logarithmic curve (20 Hz ↔ 20 kHz), matching Pioneer CFX.
/// Returns `None` for the bypass zone (|val - 0.5| < 0.005) — no filter needed.
pub fn sweep_filter_svf(val: f64, sr: f64) -> Option<SvfFilter> {
    if (val - 0.5).abs() < 0.005 {
        return None;
    }
    let (fc, is_hp) = if val < 0.5 {
        // HP: val 0.0 → 0.5 sweeps fc 20 kHz → 20 Hz (log)
        let t = 1.0 - val * 2.0;          // 1.0 at val=0, 0.0 at val=0.5
        let f = 20.0 * 1000.0_f64.powf(t); // 20 Hz × 1000^t
        (f.clamp(20.0, sr * 0.499), true)
    } else {
        // LP: val 0.5 → 1.0 sweeps fc 20 kHz → 20 Hz (log)
        let t = (val - 0.5) * 2.0;              // 0.0 at val=0.5, 1.0 at val=1.0
        let f = 20.0 * 1000.0_f64.powf(1.0 - t); // 20 kHz → 20 Hz
        (f.clamp(20.0, sr * 0.499), false)
    };
    // Q = 0.707 (Butterworth) — flat passband, no resonance peak
    Some(SvfFilter::new(fc, 0.707, sr, is_hp))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bypass_zone_returns_none() {
        assert!(sweep_filter_svf(0.5, 44100.0).is_none());
        assert!(sweep_filter_svf(0.503, 44100.0).is_none());
        assert!(sweep_filter_svf(0.497, 44100.0).is_none());
        assert!(sweep_filter_svf(0.494, 44100.0).is_some()); // just outside
        assert!(sweep_filter_svf(0.506, 44100.0).is_some());
    }

    #[test]
    fn hp_at_zero_val() {
        let f = sweep_filter_svf(0.0, 44100.0).unwrap();
        assert!(f.is_hp);
    }

    #[test]
    fn lp_at_one_val() {
        let f = sweep_filter_svf(1.0, 44100.0).unwrap();
        assert!(!f.is_hp);
    }

    #[test]
    fn no_click_at_crossover() {
        // Run a signal through SVF band and change mode at the crossover.
        // State should be preserved — ic1eq and ic2eq should not jump to 0.
        let mut band = SvfBand::new();

        // Fill state with LP filter running
        band.set(sweep_filter_svf(0.8, 44100.0));
        let mut frame = [0.5_f64; 2];
        for _ in 0..100 {
            band.process(&mut frame);
        }
        let ic1_before = band.states[0].ic1eq;
        let ic2_before = band.states[0].ic2eq;

        // Switch to HP mode — state must be preserved (not zeroed)
        band.set(sweep_filter_svf(0.2, 44100.0));
        assert!(
            (band.states[0].ic1eq - ic1_before).abs() < 1e-15,
            "ic1eq was reset on mode switch"
        );
        assert!(
            (band.states[0].ic2eq - ic2_before).abs() < 1e-15,
            "ic2eq was reset on mode switch"
        );
    }

    #[test]
    fn lp_attenuates_high_frequencies() {
        // LP at val=0.9 should significantly attenuate a near-Nyquist tone
        let f = sweep_filter_svf(0.9, 44100.0).unwrap();
        let mut state = SvfState::new();
        // Nyquist test tone: alternating +1/-1 (22050 Hz)
        let mut energy = 0.0_f64;
        for i in 0..512 {
            let x = if i % 2 == 0 { 1.0 } else { -1.0 };
            let y = f.process(x, &mut state);
            energy += y * y;
        }
        let rms = (energy / 512.0).sqrt();
        assert!(rms < 0.1, "LP at val=0.9 should heavily attenuate Nyquist tone, got rms={rms}");
    }

    #[test]
    fn hp_attenuates_dc() {
        // HP at val=0.1 should significantly attenuate DC
        let f = sweep_filter_svf(0.1, 44100.0).unwrap();
        let mut state = SvfState::new();
        let mut energy = 0.0_f64;
        for _ in 0..512 {
            let y = f.process(1.0, &mut state); // DC input
            energy += y * y;
        }
        let rms = (energy / 512.0).sqrt();
        assert!(rms < 0.1, "HP at val=0.1 should heavily attenuate DC, got rms={rms}");
    }
}

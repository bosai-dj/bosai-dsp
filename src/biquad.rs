//! Second-order IIR biquad filter.
//!
//! Coefficients stored as [b0, b1, b2, a0, a1, a2] (a0 is always 1.0 after normalisation).
//! Filter state (zi) is per-channel: two delay elements per section.
use std::f64::consts::PI;

/// Per-channel filter state (2 delay elements per second-order section).
#[derive(Clone, Debug)]
pub struct BiquadState {
    pub z: Vec<[f64; 2]>, // one [z1, z2] per SOS section
}

impl BiquadState {
    pub fn new(num_sections: usize) -> Self {
        Self {
            z: vec![[0.0; 2]; num_sections],
        }
    }

    pub fn reset(&mut self) {
        for z in &mut self.z {
            *z = [0.0; 2];
        }
    }
}

/// A single second-order section: b0, b1, b2, a1, a2 (a0 normalised to 1.0).
#[derive(Clone, Debug)]
pub struct Sos {
    pub b0: f64,
    pub b1: f64,
    pub b2: f64,
    pub a1: f64,
    pub a2: f64,
}

impl Sos {
    /// Apply this section to a single sample, updating state in-place (direct form II transposed).
    #[inline]
    pub fn process(&self, x: f64, z: &mut [f64; 2]) -> f64 {
        let y = self.b0 * x + z[0];
        z[0] = self.b1 * x - self.a1 * y + z[1];
        z[1] = self.b2 * x - self.a2 * y;
        y
    }
}

/// A chain of second-order sections (cascaded biquads).
#[derive(Clone, Debug)]
pub struct SosChain {
    pub sections: Vec<Sos>,
}

impl SosChain {
    pub fn num_sections(&self) -> usize {
        self.sections.len()
    }

    /// Apply the full chain to one sample, updating per-section state.
    #[inline]
    pub fn process(&self, mut x: f64, state: &mut BiquadState) -> f64 {
        for (i, sec) in self.sections.iter().enumerate() {
            x = sec.process(x, &mut state.z[i]);
        }
        x
    }
}

/// 4th-order Butterworth highpass or lowpass.
/// Returns None if bypass (val ≈ 0.5).
///
/// `val` ∈ [0.0, 1.0]: 0.0 = full HP, 0.5 = bypass, 1.0 = full LP.
/// Frequency follows a cubic curve from 20 Hz to 20 kHz.
///
/// **Superseded** by `dsp::svf::sweep_filter_svf` which uses a TPT SVF and
/// eliminates the HP/LP topology-switch click. Kept here for unit tests only.
#[allow(dead_code)]
pub fn sweep_filter(val: f64, sr: f64) -> Option<SosChain> {
    let nyquist = sr / 2.0;
    if (val - 0.5).abs() < 0.01 {
        return None; // bypass zone
    }

    let (freq, is_highpass) = if val < 0.5 {
        // HP: val 0.0→0.5 maps to freq 20kHz→20Hz (cubic)
        let t = 1.0 - val / 0.5; // 1.0 at val=0, 0.0 at val=0.5
        let f = (20000.0 * t * t * t).max(20.0);
        (f, true)
    } else {
        // LP: val 0.5→1.0 maps to freq 20kHz→20Hz (cubic) — matches Python
        let t = (val - 0.5) * 2.0; // 0.0 at val=0.5, 1.0 at val=1.0
        let f = (20000.0 * (1.0 - t).powi(3)).max(20.0);
        (f, false)
    };

    // NOT `.clamp()`: f64::min/max ignore NaN, so a NaN frequency collapses to
    // 0.99 and the filter stays well-formed. `clamp` propagates NaN into the
    // coefficients, which poisons the filter state permanently.
    #[allow(clippy::manual_clamp)]
    let wn = (freq / nyquist).min(0.99).max(0.001);
    Some(butterworth_4th(wn, is_highpass))
}

/// Bilinear-transform one s-domain conjugate pole pair (at `warped * e^{±j·angle}`)
/// into a z-domain second-order section. `highpass` selects HP vs LP numerator.
#[inline]
fn pole_pair_section(warped: f64, angle: f64, highpass: bool) -> Sos {
    let fs = 2.0;
    let re = angle.cos() * warped;
    let im = angle.sin() * warped;

    // Bilinear transform: s → 2*fs*(z-1)/(z+1)
    let a0s = 4.0 * fs * fs - 4.0 * fs * re + re * re + im * im;

    let (b0, b1, b2) = if highpass {
        (4.0 * fs * fs / a0s, -8.0 * fs * fs / a0s, 4.0 * fs * fs / a0s)
    } else {
        let ww = re * re + im * im;
        (ww / a0s, 2.0 * ww / a0s, ww / a0s)
    };

    let a1 = (2.0 * (re * re + im * im) - 8.0 * fs * fs) / a0s;
    let a2 = (4.0 * fs * fs + 4.0 * fs * re + re * re + im * im) / a0s;

    Sos { b0, b1, b2, a1, a2 }
}

/// 4th-order Butterworth highpass or lowpass as two cascaded second-order
/// sections (maximally-flat, poles at angles 5π/8 and 7π/8).
///
/// Use this for a *standalone* filter (e.g. the sweep/colour filter). Do NOT
/// use it to build a summed crossover: a Butterworth LP+HP pair is only
/// power-complementary, so summing the bands produces a +3 dB peak at the
/// crossover frequency. For a flat-summing crossover use [`linkwitz_riley_4th`].
pub(super) fn butterworth_4th(wn: f64, highpass: bool) -> SosChain {
    let fs = 2.0;
    let warped = 2.0 * fs * (PI * wn / fs).tan();

    // 4th-order Butterworth pole-pair angles (s-domain, upper half).
    let angles = [5.0 * PI / 8.0, 7.0 * PI / 8.0];
    let sections = angles
        .iter()
        .map(|&angle| pole_pair_section(warped, angle, highpass))
        .collect();

    SosChain { sections }
}

/// Linkwitz-Riley 4th-order (LR4) highpass or lowpass: two cascaded *identical*
/// 2nd-order Butterworth sections (a doubled pole pair at Q = 1/√2, angle 3π/4).
///
/// Unlike a true 4th-order Butterworth, an LR4 LP+HP pair is voltage-
/// complementary: at the crossover frequency each band is −6 dB and in phase,
/// so they sum to unity magnitude. This is the correct building block for the
/// 3-band crossover EQ, which sums its bands back together.
pub(super) fn linkwitz_riley_4th(wn: f64, highpass: bool) -> SosChain {
    let fs = 2.0;
    let warped = 2.0 * fs * (PI * wn / fs).tan();

    // LR4 = 2nd-order Butterworth (single pole pair at 3π/4) cascaded with itself.
    let angle = 3.0 * PI / 4.0;
    let section = pole_pair_section(warped, angle, highpass);

    SosChain { sections: vec![section.clone(), section] }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sweep_filter_bypass_at_center() {
        assert!(sweep_filter(0.5, 44100.0).is_none());
        assert!(sweep_filter(0.505, 44100.0).is_none());
        assert!(sweep_filter(0.495, 44100.0).is_none());
        // Just outside bypass zone — should return a filter
        assert!(sweep_filter(0.49, 44100.0).is_some());
        assert!(sweep_filter(0.51, 44100.0).is_some());
    }

    #[test]
    fn sweep_filter_hp_at_zero() {
        let chain = sweep_filter(0.0, 44100.0).unwrap();
        assert_eq!(chain.num_sections(), 2); // 4th order = 2 sections
    }

    #[test]
    fn sweep_filter_lp_at_one() {
        let chain = sweep_filter(1.0, 44100.0).unwrap();
        assert_eq!(chain.num_sections(), 2);
    }

    #[test]
    fn biquad_state_process() {
        let chain = butterworth_4th(0.1, false); // LP at wn=0.1
        let mut state = BiquadState::new(chain.num_sections());
        let y0 = chain.process(1.0, &mut state);
        let y1 = chain.process(0.0, &mut state);
        assert!(y0.abs() > 0.0);
        assert!(y1.abs() > 0.0);
    }

    // --- shared measurement helpers ---------------------------------------

    /// Steady-state gain (dB) of a single chain at `freq`, after a warmup window.
    fn gain_db(chain: &SosChain, freq: f64, sr: f64) -> f64 {
        let mut st = BiquadState::new(chain.num_sections());
        let (warmup, measure) = (16384, 16384);
        let (mut sig, mut out) = (0.0_f64, 0.0_f64);
        for i in 0..(warmup + measure) {
            let x = (2.0 * PI * freq * (i as f64 / sr)).sin();
            let y = chain.process(x, &mut st);
            if i >= warmup {
                sig += x * x;
                out += y * y;
            }
        }
        10.0 * (out / sig).log10()
    }

    /// Steady-state gain (dB) of `lp(x) + hp(x)` summed — the crossover sum.
    fn sum_gain_db(lp: &SosChain, hp: &SosChain, freq: f64, sr: f64) -> f64 {
        let mut s_lp = BiquadState::new(lp.num_sections());
        let mut s_hp = BiquadState::new(hp.num_sections());
        let (warmup, measure) = (16384, 16384);
        let (mut sig, mut out) = (0.0_f64, 0.0_f64);
        for i in 0..(warmup + measure) {
            let x = (2.0 * PI * freq * (i as f64 / sr)).sin();
            let y = lp.process(x, &mut s_lp) + hp.process(x, &mut s_hp);
            if i >= warmup {
                sig += x * x;
                out += y * y;
            }
        }
        10.0 * (out / sig).log10()
    }

    fn sos_eq(a: &Sos, b: &Sos) -> bool {
        let close = |x: f64, y: f64| (x - y).abs() < 1e-12;
        close(a.b0, b.b0) && close(a.b1, b.b1) && close(a.b2, b.b2)
            && close(a.a1, b.a1) && close(a.a2, b.a2)
    }

    // --- butterworth_4th --------------------------------------------------

    #[test]
    fn butterworth_4th_has_two_distinct_sections() {
        let lp = butterworth_4th(0.1, false);
        assert_eq!(lp.num_sections(), 2, "4th order = 2 biquads");
        // Butterworth-4 poles sit at two different angles (5π/8, 7π/8) → the
        // two sections must NOT be identical (this is what distinguishes it
        // from a Linkwitz-Riley filter, whose two sections are identical).
        assert!(
            !sos_eq(&lp.sections[0], &lp.sections[1]),
            "Butterworth-4 sections should differ (different pole pairs)"
        );
    }

    #[test]
    fn butterworth_4th_lp_passband_stopband() {
        let sr = 44100.0;
        let wn = 0.1;
        let fc = wn * sr / 2.0; // cutoff in Hz
        let lp = butterworth_4th(wn, false);
        // Passband (well below cutoff) ≈ 0 dB
        assert!(gain_db(&lp, fc / 10.0, sr).abs() < 0.5);
        // Maximally-flat Butterworth is −3 dB at its cutoff (any order).
        let at_cut = gain_db(&lp, fc, sr);
        assert!(
            (at_cut + 3.0).abs() < 0.7,
            "Butterworth-4 LP should be ~-3 dB at cutoff, got {at_cut:.2} dB"
        );
        // Stopband (well above cutoff) strongly attenuated (4th order = 24 dB/oct)
        assert!(gain_db(&lp, fc * 4.0, sr) < -30.0);
    }

    #[test]
    fn butterworth_4th_hp_is_mirror() {
        let sr = 44100.0;
        let wn = 0.1;
        let fc = wn * sr / 2.0;
        let hp = butterworth_4th(wn, true);
        // Passband (well above cutoff) ≈ 0 dB
        assert!(gain_db(&hp, fc * 8.0, sr).abs() < 0.5);
        // −3 dB at cutoff
        assert!((gain_db(&hp, fc, sr) + 3.0).abs() < 0.7);
        // Stopband (well below cutoff) strongly attenuated
        assert!(gain_db(&hp, fc / 4.0, sr) < -30.0);
    }

    #[test]
    fn butterworth_4th_sum_peaks_3db_at_crossover() {
        // Power-complementary: a Butterworth-4 LP+HP pair sums to +3 dB at the
        // crossover frequency. This is exactly why it must NOT be used for the
        // summed crossover EQ — documents the bug that motivated LR4.
        let sr = 44100.0;
        let wn = 0.1;
        let fc = wn * sr / 2.0;
        let lp = butterworth_4th(wn, false);
        let hp = butterworth_4th(wn, true);
        let sum = sum_gain_db(&lp, &hp, fc, sr);
        assert!(
            (sum - 3.0).abs() < 0.7,
            "Butterworth-4 LP+HP should peak ~+3 dB at crossover, got {sum:.2} dB"
        );
    }

    // --- linkwitz_riley_4th -----------------------------------------------

    #[test]
    fn lr4_has_two_identical_sections() {
        let lp = linkwitz_riley_4th(0.1, false);
        assert_eq!(lp.num_sections(), 2, "LR4 = 2 cascaded biquads");
        // Defining property: LR4 is a 2nd-order Butterworth applied twice, so
        // the two sections are identical.
        assert!(
            sos_eq(&lp.sections[0], &lp.sections[1]),
            "LR4 sections must be identical (doubled 2nd-order Butterworth)"
        );
    }

    #[test]
    fn lr4_is_minus_6db_at_crossover() {
        let sr = 44100.0;
        let wn = 0.1;
        let fc = wn * sr / 2.0;
        // LR4 = (2nd-order Butterworth)², so it is −6 dB at cutoff (not −3 dB
        // like a maximally-flat Butterworth). This is what lets LP+HP sum flat.
        let lp = linkwitz_riley_4th(wn, false);
        let hp = linkwitz_riley_4th(wn, true);
        let lp_db = gain_db(&lp, fc, sr);
        let hp_db = gain_db(&hp, fc, sr);
        assert!(
            (lp_db + 6.0).abs() < 0.7,
            "LR4 LP should be ~-6 dB at crossover, got {lp_db:.2} dB"
        );
        assert!(
            (hp_db + 6.0).abs() < 0.7,
            "LR4 HP should be ~-6 dB at crossover, got {hp_db:.2} dB"
        );
    }

    #[test]
    fn lr4_sum_is_flat_at_crossover() {
        // Voltage-complementary: LR4 LP+HP sum to unity (0 dB) at the crossover.
        let sr = 44100.0;
        let wn = 0.1;
        let fc = wn * sr / 2.0;
        let lp = linkwitz_riley_4th(wn, false);
        let hp = linkwitz_riley_4th(wn, true);
        for &f in &[fc / 4.0, fc, fc * 4.0] {
            let sum = sum_gain_db(&lp, &hp, f, sr);
            assert!(
                sum.abs() < 0.5,
                "LR4 LP+HP should sum ~0 dB at {f:.0} Hz, got {sum:.2} dB"
            );
        }
    }

    #[test]
    fn lr4_lp_passband_stopband() {
        let sr = 44100.0;
        let wn = 0.1;
        let fc = wn * sr / 2.0;
        let lp = linkwitz_riley_4th(wn, false);
        assert!(gain_db(&lp, fc / 10.0, sr).abs() < 0.5); // passband flat
        assert!(gain_db(&lp, fc * 4.0, sr) < -30.0); // 24 dB/oct stopband
    }
}

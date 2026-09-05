//! One-pole lowpass parameter smoother.
//!
//! Prevents zipper noise when a parameter (wet/dry, feedback, filter cutoff)
//! changes in discrete steps — as it does with 14-bit MIDI. Call `process`
//! once per sample with the current target; the returned value eases toward
//! it with the configured time constant.

pub struct Smoother {
    current: f64,
    coef: f64,
}

impl Smoother {
    pub fn new(sample_rate: f64, time_constant_s: f64) -> Self {
        // Clamp to 1 µs minimum so the coefficient stays in [0, 1) even when
        // the caller passes 0.0 (instant snap) or a negative value.
        let tc = time_constant_s.max(1e-6);
        let coef = (-1.0_f64 / (tc * sample_rate)).exp();
        Self { current: 0.0, coef }
    }

    pub fn process(&mut self, target: f64) -> f64 {
        self.current = self.coef * self.current + (1.0 - self.coef) * target;
        self.current
    }

    #[allow(dead_code)]
    pub fn reset(&mut self, value: f64) {
        self.current = value;
    }
}

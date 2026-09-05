//! Beat-synced DJ effects, in pure Rust.
//!
//! A library of DSP building blocks modelled on the Beat FX section of a
//! professional DJ mixer: effects whose time base is *musical* rather than
//! absolute. Every time-based effect here takes a `beat_value` (in beats) plus
//! a live BPM, and derives its own delay time, loop length, or LFO rate from
//! them — so a 1/2-beat echo stays a 1/2-beat echo when the deck's tempo fader
//! moves.
//!
//! That is the distinction from general-purpose audio DSP crates, which
//! overwhelmingly express time in seconds and leave tempo mapping to the caller.
//!
//! # What's here
//!
//! Beat-synced effects — all take `set_bpm()` + `beat_value`:
//!
//! - [`echo`] — stereo echo with DJ-mixer pre-filter variants (HPF / LPF / BPF)
//! - [`ping_pong`] — cross-fed L/R bouncing delay
//! - [`roll_fx`] — Roll, Slip Roll, Rev Roll and Helix buffer loopers
//! - [`gater`] — rhythmic gate with noise fill
//! - [`filter_fx`] — resonant HP/LP filter with beat-rate LFO sweep
//! - [`flanger`], [`phaser`], [`peak_filter`] — beat-rate modulated
//!
//! Free-running blocks:
//!
//! - [`reverb`] — Freeverb, RT60-parameterised
//! - [`eq_chain`] — 3-band Linkwitz-Riley crossover EQ + sweep filter
//! - [`biquad`], [`svf`] — filter primitives
//! - [`smoother`] — one-pole parameter smoothing
//! - [`resample`] — linear/cubic interpolation helpers
//!
//! # Conventions
//!
//! **Stereo only.** [`CHANNELS`] is fixed at 2. Every DJ mixer signal path is
//! stereo, and fixing it keeps frames on the stack as `[f64; 2]` with no
//! allocation on the audio path.
//!
//! **`f64` internally.** Effects process in `f64` and are intended to sit in an
//! `f64` mix bus; convert at the device boundary.
//!
//! **Click-free by construction.** Parameter changes are smoothed with one-pole
//! filters, and changes that would jump a read pointer (delay time, beat value)
//! are handled with short equal-power crossfades rather than discontinuities.
//! Filter state is deliberately *not* reset on parameter change.
//!
//! **No allocation on the audio path.** Buffers are sized once at construction
//! from a documented maximum; `process()` never allocates.

#![forbid(unsafe_code)]
// Audio frames are indexed by channel throughout; `for ch in 0..CHANNELS` keeps
// the channel index visible and matches how the DSP is written on paper.
#![allow(clippy::needless_range_loop)]

/// Channel count for every frame in this crate. Stereo, fixed — see the
/// crate-level docs for why.
pub const CHANNELS: usize = 2;

pub mod biquad;
pub mod echo;
pub mod eq_chain;
pub mod filter_fx;
pub mod flanger;
pub mod freeverb;
pub mod gater;
pub mod peak_filter;
pub mod phaser;
pub mod ping_pong;
pub mod resample;
pub mod reverb;
pub mod roll_fx;
pub mod smoother;
pub mod svf;

//! Freeverb — a Rust implementation of the public-domain Freeverb reverb algorithm.
//!
//! Vendored from https://github.com/irh/freeverb-rs (MIT License, © 2018 Ian Hobson).
//! The original Freeverb C++ implementation was written by "Jezar at Dreampoint"
//! and released into the public domain in June 2000.
//!
//! See https://ccrma.stanford.edu/~jos/pasp/Freeverb.html for algorithm analysis.
//!
//! ============================================================================
//! MIT License
//!
//! Copyright (c) 2018 Ian Hobson
//!
//! Permission is hereby granted, free of charge, to any person obtaining a copy
//! of this software and associated documentation files (the "Software"), to deal
//! in the Software without restriction, including without limitation the rights
//! to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//! copies of the Software, and to permit persons to whom the Software is
//! furnished to do so, subject to the following conditions:
//!
//! The above copyright notice and this permission notice shall be included in all
//! copies or substantial portions of the Software.
//!
//! THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//! IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//! FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//! AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//! LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//! OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//! SOFTWARE.
//! ============================================================================

mod all_pass;
mod comb;
mod delay_line;
mod float;
#[allow(clippy::module_inception)]
mod freeverb;
mod tuning;

pub use self::freeverb::Freeverb;

# bosai-dsp

Beat-synced DJ effects in pure Rust. No dependencies, no allocation on the audio path.

Effects modelled on the Beat FX section of a professional DJ mixer — where time is
measured in **beats, not seconds**. Every time-based effect takes a `beat_value`
plus a live BPM and derives its own delay time, loop length or LFO rate from them,
so a 1/2-beat echo stays a 1/2-beat echo when the tempo fader moves.

```toml
[dependencies]
bosai-dsp = "0.1"
```

```rust
use bosai_dsp::echo::{Echo, EchoFilter};

let mut echo = Echo::new(44_100.0);
echo.set_bpm(128.0);
echo.set_beat_value(0.5);        // 1/2-beat delay, tracks BPM
echo.set_filter_mode(EchoFilter::HPF); // only highs echo; the kick stays dry
echo.set_depth(0.4);
echo.set_enabled(true);

for frame in buffer.iter_mut() {
    echo.process_sample(frame);   // &mut [f64; 2]
}
```

## Why this exists

Rust's audio ecosystem has excellent general-purpose DSP, and a small but growing
set of DJ-specific crates — [`timestretch`][ts] for keylock and varispeed,
[`stratum-dsp`][st] for BPM/key analysis. Neither does effects, and the
general-purpose libraries express delay time in seconds with no notion of beats.

The one mature open-source DJ effects implementation is [Mixxx][mx], which is
GPL-2.0-or-later — unusable if you are building anything permissively licensed.

This crate fills that gap: **MIT-licensed, BPM-synced DJ effects.**

## Effects

| Module | Beat-synced | What it does |
|---|:--:|---|
| `echo` | ✓ | Stereo echo, with HPF / LPF / BPF pre-filter variants |
| `ping_pong` | ✓ | Cross-fed L→R→L bouncing delay |
| `roll_fx` | ✓ | Roll, Slip Roll, Rev Roll, Helix buffer loopers |
| `gater` | ✓ | Rhythmic gate with noise fill |
| `filter_fx` | ✓ | Resonant HP/LP with beat-rate LFO sweep |
| `flanger` `phaser` `peak_filter` | ✓ | Beat-rate modulated |
| `reverb` | | Freeverb, parameterised by RT60 |
| `eq_chain` | | 3-band Linkwitz-Riley crossover EQ + sweep filter |
| `biquad` `svf` `smoother` `resample` | | Primitives |

The pre-filter variants matter more than they sound: in `echo`, only the filtered
signal enters the delay buffer while the dry path stays untouched — so an HPF echo
leaves the kick and bass solid while the tail rings out on top of it.

## Design notes

**Click-free by construction.** Parameter changes are one-pole smoothed. Changes
that would jump a read pointer — delay time, beat value — use a 20 ms equal-power
crossfade between the old and new read positions rather than a discontinuity.
Filter state is deliberately *not* reset on parameter change.

**Real-time safe.** Buffers are sized once at construction from a documented
maximum. `process_sample` never allocates, never locks, and `#![forbid(unsafe_code)]`.

**Stereo, `f64`.** `CHANNELS` is fixed at 2 — every DJ mixer signal path is stereo,
and fixing it keeps frames on the stack as `[f64; 2]`.

**Linkwitz-Riley, not Butterworth.** `eq_chain` uses a true LR4 crossover, which
sums voltage-flat. A cascaded-Butterworth crossover — the common shortcut — leaves
a +3 dB bump at each crossover frequency. There is a test asserting flatness at
250 Hz and 2.5 kHz specifically to catch that regression.

## Status

`0.1.0`. 93 tests, zero clippy warnings.

The API is not yet stable — `0.1` means the DSP is tested and working, not that
the surface is settled. Feedback on the interface is very welcome before `1.0`.

## Credits

`freeverb/` is vendored from [freeverb-rs][fv] by Ian Hobson (MIT).

## License

MIT

[ts]: https://crates.io/crates/timestretch
[st]: https://crates.io/crates/stratum-dsp
[mx]: https://github.com/mixxxdj/mixxx
[fv]: https://github.com/irh/freeverb-rs

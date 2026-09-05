# Contributing

<br>

## Running it

```bash
git clone https://github.com/bosai-dj/bosai-dsp
cd bosai-dsp
cargo test
```

There is nothing to install and no feature flags to pick. The crate has no
dependencies — not runtime, not dev — so `cargo test` compiles only this crate, and CI
asserts the dependency tree stays empty. If `cargo test` is not enough to run the suite,
that is a bug in this project, not in your setup.

Minimum supported Rust version is **1.74**, tested in CI alongside stable. Raising it is
a breaking change for someone, so it needs a reason in the PR.

<br>

## What a good change looks like

**A new effect** needs a `beat_value` and a live BPM if it is time-based, and must derive
its own timing from them rather than taking seconds. That is the line the whole crate is
organised around: musical time in, absolute time never. An effect that takes milliseconds
belongs in a general-purpose DSP crate, of which Rust has several good ones.

**Every effect must be click-free under parameter change.** This is the invariant that is
easiest to break and hardest to notice, because a click does not fail a test that only
checks amplitudes. Parameter changes get one-pole smoothing; changes that move a read
pointer — delay time, beat value — get a short crossfade between the old and new read
positions. Filter state is deliberately *not* reset on parameter change. If your change
touches any of that, say in the PR how you convinced yourself it stayed clean.

**No allocation on the audio path.** Buffers are sized once in the constructor from a
documented maximum. `process_sample` must not allocate, lock, or panic. `unsafe` is
forbidden at the crate root and that is not up for negotiation.

**Tests are synthetic, never device-backed.** Every module tests against a buffer it
fills itself, which is why the suite runs in under a tenth of a second and needs no audio
hardware. Copy the shape of an existing test in the same file.

<br>

## Timing bugs live in the fractional frame

Delay times are fractional frame counts — 5512.5 frames at 120 BPM and a 1/4-beat value,
not 5512. Nearly every timing bug in this crate's history has been something truncating
that half-frame, and the damage doubles anywhere a tap sits at 2x. If a test fills a
buffer to cover a delay, it must round **up**. There is a test helper that does this and
a comment explaining why; read it before writing a new one.

<br>

## Formatting

The code is not `rustfmt`-clean, deliberately. Coefficient tables are column-aligned so
the values read down the page, and rustfmt would break that alignment for no gain. There
is no `cargo fmt --check` gate. Match the surrounding style; `cargo clippy -- -D warnings`
is enforced and is the standard that does apply.

<br>

## Before you open a PR

- `cargo test` passes, and `cargo clippy --all-targets -- -D warnings` is clean
- New behaviour has a test, including the case it must *not* affect
- A comment explains *why the code is the way it is*, where that is not obvious. Not what
  it does, and not the story of how it was found
- The PR title reads as a release note, because it becomes one verbatim: release notes are
  generated from merged PRs at tag time

Issues and PRs are welcome. There is no response SLA.

<br>

## Releasing

1. Merge everything you want in the release, with PR titles that read as release notes.
   They become the notes verbatim, categorised by label via `.github/release.yml`.
2. Bump `version` in `Cargo.toml` via a PR like any other change.
3. Tag it: `git tag v0.1.0 && git push origin v0.1.0`. A tag alone publishes nothing.
4. Create the GitHub Release for that tag with generated notes. **Publishing the Release
   is what triggers the crates.io upload.** A tag is easy to push by accident and
   impossible to retract once it has reached crates.io, so the deliberate act is the gate.

The workflow re-runs the tests, checks the tag matches the manifest version, publishes via
[Trusted Publishing][tp] (no API token exists in repo secrets), and attaches the `.crate`
back to the Release.

**One-time setup:** Trusted Publishing must be configured on the crates.io side before the
first automated release — on the crate's Settings page, add a GitHub publisher for
`bosai-dj/bosai-dsp`, workflow `release.yml`, environment `crates-io`. Until that exists
the publish job will fail to mint a token.

[tp]: https://crates.io/docs/trusted-publishing

# Security policy

## Supported versions

Only the latest released version receives fixes. This is a 0.x library; there are no
maintenance branches.

## Reporting a vulnerability

Report privately via
[GitHub Security Advisories](https://github.com/bosai-dj/bosai-dsp/security/advisories/new).
Please do not open a public issue.

Expect an acknowledgement within 7 days. This is a small project with no dedicated
security staffing, so that window is what can actually be met rather than an aspiration.

## Scope

bosai-dsp transforms audio buffers it is handed. It has no dependencies, makes no network
calls, reads no files, and contains no `unsafe` — `#![forbid(unsafe_code)]` is enforced at
the crate root, so memory-safety bugs would have to come from the compiler rather than
from here. The realistic surface is therefore narrow:

- **Denial of service through allocation.** Constructors size their buffers from the
  sample rate you pass. `Echo::new(f64::MAX)` will try to allocate accordingly. Sample
  rates are expected to come from your audio device, not from untrusted input.
- **Non-finite input.** NaN or infinite samples, BPM values, or beat values can propagate
  into filter state and persist across calls, since filter state is deliberately not
  reset on parameter change. Coefficient computation guards against NaN frequencies, but
  this is not comprehensive. Sanitise at your input boundary.

Neither is a memory-safety issue, and both are bounded by the caller. If you find
something that escapes those bounds, it is worth reporting.

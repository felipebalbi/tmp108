# Closing the datasheet test gaps

Design for [issue #66][issue-66]. Covers the four test gaps that remain
open (Part 1 items 3, 4, 5 and 7), the one genuine documentation
overpromise (Part 3), and three adjacent coverage gaps found while
surveying for this work.

Baseline: `854a0c722754`, the tip of `config-read-acknowledgement`.

[issue-66]: https://github.com/OpenDevicePartnership/tmp108/issues/66

---

## What is already done

Issue #66 is an umbrella raised against `5e7b8d2`. Three of its seven
Part 1 items have since been closed by other work, and this design does
not revisit them:

| Item | Claim | Status |
|---|---|---|
| 1 | Mode encoding `11` is actively mis-pinned | Closed by #62. `RESERVED_MODE` is gone; `the_mode_getter_decodes_every_register_word` (`src/lib.rs:4345`) walks all 65,536 words and maps both `10` and `11` to `Mode::Continuous`. |
| 2 | Limit register reset values unpinned | Closed by #63. `tmp108.ddsl:84` and `:96` declare them; `limit_registers_reset_to_the_documented_window` (`src/lib.rs:4845`) asserts them through a no-op `write()`, not against `Fieldset::ZERO`. |
| 6 | `one_shot()` has no unit test | Closed by #60. `one_shot` is now a supervised acquisition returning `Celsius`, with 14 unit tests plus a timeline ordering test. |

Part 2 of the issue is a do-not-touch list. Nothing in this design
changes any behaviour it protects.

## What remains

Four test gaps, one doc correction, three adjacent gaps.

---

## Gap 3 — the decoder has no independent oracle

`decoding_a_register_is_total` (`src/lib.rs:3898`) walks all 65,536
register words but asserts only that the result *falls inside* the
representable range:

```rust
assert!((MIN_SIXTEENTHS..=MAX_SIXTEENTHS).contains(&c.sixteenths()));
```

Every permutation of the 4096 values passes that. The companion
round-trip `every_temperature_roundtrips_through_a_register`
(`src/lib.rs:3910`) compares `to_register` against `from_register` —
the encoder against its own inverse — so a mutually-consistent wrong
permutation passes that too. The only genuine oracle is
`known_datasheet_values_decode` (`src/lib.rs:3985`), eleven hand-listed
points.

### Change

Rewrite `decoding_a_register_is_total` into a value oracle over the
same 65,536 words. For word `w`, the 12-bit code is `n = w >> 4` and
the expected reading is `n < 2048 ? n : n - 4096` sixteenths, computed
without reference to the implementation.

This strictly subsumes the range check, and additionally pins
unused-low-bit handling across *every* bit pattern rather than the
three cases currently at `src/lib.rs:4007`. The test is renamed to say
what it now checks.

Add a companion over the 4096 canonical codes asserting the encode
direction independently: `to_register()` equals `[n >> 4, (n & 15) << 4]`.

Add ±0.0625 °C and ±0.125 °C as explicit named cases so the individual
fractional-bit weights are visible rather than implied by an exhaustive
loop. Both verified: `±0.0625 → ±1` sixteenth, `±0.125 → ±2`.

## Gap 4 — the application note's worked values are unasserted

SBAA588A works three conversions by hand. None appears in the test
suite. All three were checked against the oracle before being written
into this design, and all three re-encode exactly:

| Word | Code | Sixteenths | Degrees | Re-encodes |
|---|---|---|---|---|
| `0x2090` | 521 | 521 | 32.5625 | yes |
| `0xFAE0` | 4014 | −82 | −5.125 | yes |
| `0x1880` | 392 | 392 | 24.5 | yes |

These are worth having separately from the exhaustive oracle because
they exercise mixed integer/fractional data and negative fractional
decoding at points a human independently computed.

### Change

A new test, kept separate from `known_datasheet_values_decode` because
the source document differs — SBAA588A, not SBOS663A Table 7. Asserts
both directions, since all three have a zero low nibble.

## Gap 5 — conversion-rate delays are entirely unpinned

`conversion_period_us` (`src/lib.rs:3158`) has no assertion anywhere.
`wait_for_temperature` has no unit test at all in either flavor; its
only exercise is two doctests (`src/lib.rs:2329`, `src/lib.rs:3011`),
both using `NoopDelay`, which asserts nothing.

Deleting the `delay.delay_us(...)` call from either body would not fail
a single test.

### Change, in three layers

**1. Move `conversion_period_us` into `mod ops`.** It is crate-level
private today, but it is precisely what AGENTS.md says belongs in
`ops`: sync/async-agnostic logic shared by both `wait_for_temperature`
bodies. Two call sites update (`src/lib.rs:2346`, `src/lib.rs:3032`).
No behaviour change and no public API change, so this is a `refactor:`.
It also puts the function's test in the module AGENTS.md designates for
pure-function tests.

**2. A table test over all four `ConversionRate` variants.** The
function is total over a four-inhabitant enum; covering all four costs
nothing and removes the possibility that three of them are wrong.
Per SBOS663A §7.5.3.5: 0.25 Hz → 4 000 000 µs, 1 Hz → 1 000 000 µs,
4 Hz → 250 000 µs, 16 Hz → 62 500 µs.

**3. Ordering tests via `tests::timeline`.** The duration alone is not
the property at risk — the *sequence* is. A test that only checks "a
delay of 250 000 µs happened" still passes if the delay is moved after
`temperature()`, which is exactly the bug that would return a stale
conversion. `tests::timeline` (`src/lib.rs:4531`) already records
interleaved bus and clock events into one ordered log and is already
used this way by the `one_shot` tests.

New `blocking::wait_for_temperature` and
`asynchronous::wait_for_temperature` modules assert the log is exactly
`[Read(0x01), Delay(period), Read(0x00)]`, for each of the four rates.

### Prerequisite: the timeline needs microsecond resolution

`Event::Delay` stores milliseconds (`src/lib.rs:4538`) because the
`one_shot` path calls `delay_ms`. `wait_for_temperature` calls
`delay_us`, which neither `Clock` impl overrides, so it falls through
`embedded-hal`'s default to `delay_ns` and then to
`record(Event::Delay(ns / 1_000_000))` (`src/lib.rs:4652`,
`src/lib.rs:4717`).

At 16 Hz that is 62 500 µs → 62 500 000 ns → **62 ms**. The half
millisecond is silently truncated, and the test would be pinning a
value the driver never requested.

`Event::Delay` therefore changes to hold microseconds, both `Clock`
impls gain a `delay_us` arm, `delay_ms` records `ms * 1_000`, and
`delay_ns` records `ns / 1_000`. The existing `one_shot` expectations
update from `5` to `5_000`. This is confined to the test module and
lands as its own commit ahead of the tests that need it.

## Gap 7 — hysteresis snapping and float rounding are untested policy

The ±0.05 °C acceptance band in `ops::snap_hysteresis`
(`src/lib.rs:494`) and the half-away-from-zero quantisation in
`Celsius::try_from_degrees` (`src/lib.rs:420`) are driver policy. The
sources specify discrete encodings only (SBOS663A §7.5.3.1 Table 9).
They are legal choices; they are simply unpinned at the points where
floating point makes them non-obvious.

### Evidence

Both functions were transcribed verbatim into a scratch binary and run
under `rustc -O`, rather than reasoned about on paper. The scratch
reproduction of `snap_hysteresis` and `try_from_degrees` is
byte-for-byte the source at the baseline commit.

**`try_from_degrees` is clean.** The rejection boundaries are exactly
representable (`-2048.5 / 16 = -128.03125`, `2047.5 / 16 = 127.96875`)
and behave symmetrically, because the comparisons are `<=` and `>=`:

```
low  next_down -128.03127 -> Err(TooLow)
low  exact     -128.03125 -> Err(TooLow)
low  next_up   -128.03123 -> Ok(-2048)     <- Celsius::MIN
high next_down  127.96874 -> Ok(2047)      <- Celsius::MAX
high exact      127.96875 -> Err(TooHigh)
high next_up    127.96876 -> Err(TooHigh)
```

Rounding ties go away from zero, symmetrically: `±0.03125 → ±1`,
`±0.09375 → ±2`, `±1.53125 → ±25`.

**`snap_hysteresis` is not symmetric.** `HYSTERESIS_TOLERANCE` is
`0.05f32`, which is `0.05000000074505805969`. The rejection test is
`(input - closest).abs() > HYSTERESIS_TOLERANCE`, and whether the
subtraction lands above or below that constant depends on which side of
its own value each `f32` literal rounded to:

| Setting | −0.05 edge | diff | + 0.05 edge | diff |
|---|---|---|---|---|
| 0 °C | accepted | `0.05` | accepted | `0.05` |
| 1 °C | **rejected** | `0.050000012` | accepted | `0.049999952` |
| 2 °C | accepted | `0.049999952` | accepted | `0.049999952` |
| 4 °C | accepted | `0.049999952` | **rejected** | `0.05000019` |

Six of eight edges are accepted. The two rejections fall on *opposite*
sides — the low edge at 1 °C, the high edge at 4 °C — so the band is
neither uniformly inclusive nor uniformly exclusive, and no single
description of it is correct for all four settings.

The literal forms the tests will use (`0.95f32`, `1.05f32`, …) were
checked to agree with the computed forms (`base - 0.05f32`), so the
tests can be written with readable literals.

### Change

Pin all eight edges as measured, with a comment naming the cause. This
is characterisation, not correction: the issue classes the band as a
legal driver choice, and changing it would alter runtime behaviour for
callers currently on the accepted side of an edge.

The asymmetry is a real wart and gets its own follow-up issue so it can
be judged on its merits, separately from this branch.

Also pin the rounding ties and the boundary ulps shown above.

The two hysteresis tests currently sitting loose in `ops_tests`
(`src/lib.rs:4024`, `src/lib.rs:4046`) gather into a new
`ops_tests::hysteresis` alongside the new ones, matching how every
other subject in that module is already organised.

## Part 3 — `continuous()` promises more than it delivers

`src/lib.rs:2918`:

> Switches the chip into `Mode::Continuous`, runs the user-supplied
> closure, and **unconditionally returns** the chip to `Mode::Shutdown`
> before returning, **regardless of whether the closure succeeded or
> failed**.

Three paths contradict "unconditionally":

- An entry-phase error returns before the closure runs, so no cleanup
  is attempted at all.
- A panic inside the closure unwinds past the cleanup.
- A failing cleanup write means the chip is not returned to shutdown,
  and when the closure also failed, `user_result.and(cleanup_result)`
  (`src/lib.rs:2994`) discards the cleanup error — so a caller cannot
  distinguish confirmed cleanup from a chip still converting.

A fourth fact is missing rather than wrong: even an accepted shutdown
request lets the conversion already in flight run to completion
(SBOS663A §7.4.1).

Cancel-safety (`src/lib.rs:2922`) is already documented correctly and
is not touched.

### Change

Wording: "unconditionally returns" becomes "attempts to return".
Document the entry-error path, the panic path, and the shutdown
latency.

Test: the entry-phase failure path is currently untested — the existing
tests cover closure-error (`src/lib.rs:5732`) and cleanup-also-fails
(`src/lib.rs:5770`) but not entry. A new test asserts the closure is
never invoked and no cleanup transaction is issued.

The panic path is documented but **not** tested.
`embedded_hal_mock::Mock` cannot survive an unwind, so proving "cleanup
did not run" would need the shared-log timeline `Bus` plus
`catch_unwind(AssertUnwindSafe(..))` around a hand-driven executor.
That is the most machinery in this design for its least reachable path,
and `no_std` consumers build `panic = "abort"`, where the question does
not arise.

## Adjacent gaps

Three found while surveying, not named in #66:

- **No async mirror of `limit_registers_reset_to_the_documented_window`**
  (`src/lib.rs:4845`). AGENTS.md requires parallel coverage across both
  flavors; this is the same concern as the issue's item 2.
- **`Celsius`'s `Display` impl (`src/lib.rs:327`) is unasserted**,
  including its precision handling.
- **`Celsius`'s derived `Ord` is unasserted.** It orders correctly
  today because the private representation is signed sixteenths, but
  nothing pins that, and a representation change could break it
  silently.

---

## Structure

Tests go where their subject already lives. `ops_tests` organises by
subject (`celsius`, `config_bits`, `one_shot_decisions`), and this work
follows that rather than grouping by provenance: a module named for
this audit would say nothing about its contents once the issue closes,
and the next audit would start a second one.

| Test | Home |
|---|---|
| Decode/encode oracles, ±0.0625, ±0.125, app-note values, rounding ties, boundary ulps, `Display`, `Ord` | `ops_tests::celsius` |
| Eight band edges, plus the two relocated tests | `ops_tests::hysteresis` (new) |
| Four-variant period table | `ops_tests::conversion_rate` (new) |
| Read → delay → read ordering | `blocking::wait_for_temperature`, `asynchronous::wait_for_temperature` (new) |
| `continuous()` entry failure | `tests::asynchronous` |
| Async reset mirror | `tests::asynchronous` |

## Addendum — the layout tests assert a host-dependent word

Found while executing this plan, not during the original survey.

Eighteen assertions at `src/lib.rs:3692-3754` compare the
configuration fieldset against a packed word using **native**-endian
decoding:

```rust
assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1026);
```

`from_ne_bytes` resolves to `from_le_bytes` on x86, so they pass here
and would fail on a big-endian host. `default_configuration`
(`src/lib.rs:3685`), four lines above them, correctly uses
`from_le_bytes` — so the module contradicts itself.

This is the cleanup issue #66 Part 2 asked for in the same breath as
recording the byte order as correct:

> Prefer explicit byte-array assertions over `from_ne_bytes` in
> layout tests so this stops looking suspicious.

Replacing the packed word with the wire bytes it stands for removes
the host dependency and states the thing the test actually cares
about. `Configuration` is `Copy` and already converts to `[u8; 2]`,
so `assert_eq!(<[u8; 2]>::from(cfg), [0x26, 0x10]);` is a direct
substitution.

### Why the DDSL keeps `default-byte-order: LE`

The `from_ne_bytes` confusion prompted a second look at the DDSL,
which declares `LE` for a device the datasheet describes as MSB-first
(SBOS663A §7.3.4, §7.5.3, §7.5.4). The alternative was measured
rather than argued: `default-byte-order` switched to `BE`, all eight
configuration fields renumbered (`m` 1:0 → 9:8, `cr` 6:5 → 14:13,
`hys` 13:12 → 5:4, `pol` 15 → 7, and so on), the three reset values
rewritten, and `src/inner.rs` regenerated with `ddc`.

Result: 12 DDSL lines plus a regenerate, **no driver changes, no test
changes**, and the full matrix green at 60/6/24, 83/6/44 and
189/7/56. The generated reset byte arrays were byte-identical —
`[34, 16]`, `[128, 0]`, `[127, 248]`. The two encodings are the same
wire format under different labels.

Since it is a pure relabel, it was judged on readability, and neither
option wins outright:

- `BE` would let the limit registers declare the datasheet's literal
  `0x7FF8` and `0x8000` instead of the byte-swapped `0xF87F` and
  `0x0080` that today need eight lines of comment in `tmp108.ddsl`.
- `LE` keeps the field offsets matching Table 8's per-byte bit
  numbering: `m 1:0` reads as "BYTE 1, D1:D0", where `m 9:8` does not.

`BE` would also leave `ops::POR_CONFIG` (`0x1022`) and `probe()`'s
`from_le_bytes` as the inconsistent remainder, which nothing would
catch. The status quo stays; this section exists so the next reader
does not re-derive the equivalence from scratch.

Note that issue #66's warning — "changing the serialisation to 'fix'
the endianness would introduce a defect" — is about a *different*
change: flipping the byte order without renumbering the fields. A
coordinated relabel does not touch serialisation at all.

## Non-goals

- No change to `snap_hysteresis` behaviour. The asymmetry is recorded
  and referred out.
- No change to `tmp108.ddsl`'s byte order, and therefore no
  regeneration of `src/inner.rs`. See the addendum above.
- No change to `Error<E, P>`, to `Celsius`'s domain, to `Config`'s
  inhabitants, or to the `THIGH >= TLOW` relationship. Part 2 of the
  issue addresses each of these directly.
- No change to `continuous()`'s error semantics. Surfacing the cleanup
  error when the closure also failed is a behavioural change the issue
  does not ask for.
- No public API change anywhere in this design.

## Commits

Twelve, one concern each, each building and passing clippy on its own.

1. `refactor:` move `conversion_period_us` into `ops`
2. `test:` record timeline delays in microseconds
3. `test:` pin the conversion period for every conversion rate
4. `test:` pin the read-delay-read ordering of `wait_for_temperature`
5. `test:` give the temperature decoder an independent oracle
6. `test:` assert the application note's worked temperature values
7. `test:` pin the hysteresis band edges
8. `test:` pin the rounding ties and rejection boundaries
9. `test:` pin `continuous()`'s entry-failure path
10. `docs:` stop promising `continuous()` unconditionally shuts down
11. `test:` mirror the limit-register reset assertions on the async driver
12. `test:` pin `Celsius`'s `Display` and ordering

A thirteenth was added mid-execution: `test:` assert configuration
layout against wire bytes, covering the addendum above.

## Verification

The full local matrix from AGENTS.md: nightly `fmt`, pedantic clippy
across all features, `cargo doc`, unit and doc tests in all four
feature combinations, examples in all four, `cargo hack
--feature-powerset`, the README snippet script, `cargo vet`.

No hardware run is required. Nothing here touches alert-pin behaviour,
and the delay assertions are against fakes by construction — a real
clock would make the ordering test slower without making it stronger.

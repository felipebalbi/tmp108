# TMP108 One-Shot Acquisition — Design Spec

**Date:** 2026-09-24
**Status:** approved and implemented (`fb847e9`, `fc50dff`)
**Branch:** `fixes`
**Issues:** #60, #61; non-destructive acquisition follow-up #65
**Audience:** Maintainers familiar with the driver and I²C register
transactions. This is an explanation of accepted decisions, not a
proposal or a caller tutorial.

---

## Problem statement

Two defects exposed the same mistaken model: treating the TMP108's
`M` field as ordinary configuration state instead of a command with a
completion indication. The maintainer confirmed both defects on
silicon using a Pico de Gallo v1.1 rev2 and a TMP108 at `0x48`. That
hardware confirmation is the evidence supplied for this record; this
documentation pass did not repeat the bench experiments.

### #61. Reconfiguration reissued an in-flight command

`configure()` performed a read-modify-write. When the sampled register
still contained `M = 0b01`, writing it back reissued a one-shot trigger
while changing an unrelated setting. The old `ops::apply_config`
comment called `M` an "untouched bit". That was the wrong invariant,
not merely an imprecise description.

The [SBOS663A datasheet][datasheet], One-Shot Mode, describes writing
`01` from shutdown to start a conversion, reading `01` during it, and
reading `00` after completion. A sampled command was therefore not
safe to echo as though it were a persistent user setting.

### #60. A trigger was mistaken for a complete acquisition

`one_shot()` only wrote the trigger. It neither established shutdown
beforehand nor observed completion afterwards. A fixed delay followed
by `temperature()` could return the previous conversion when the delay
was too short. The Temperature Register section specifies that the
register holds the most recent conversion; reading it does not start
or wait for a new one. The old doctests also started from the
continuous-mode power-on word, modelling the wrong entry state.

The accepted solution paired safe reconfiguration (#61) with a
supervised acquisition (#60). The common invariant was that only an
explicit trigger requested a one-shot, and completion was attributed
to the chip only while no software write had stood that trigger down.
The helper did not call `configure()` between trigger and completion.
The reconfiguration fix was not permission to interleave the two.

## Goals

1. Prevent unrelated settings writes from replaying `M = 0b01`.
2. Provide the same complete, bounded acquisition on both driver shells.
3. Make caller-owned settling time and interrupt-evidence loss explicit.
4. Keep existing signatures and `Error<E, P>` unchanged.

## Non-goals

- Inventing a conversion-time bound for every board and operating point.
- Preserving interrupt evidence during this sample-only protocol (#65).
- Restoring the entry mode or introducing error-path cleanup writes.
- Replacing the low-level `one_shot()` trigger or redesigning `Mode`.
- Bumping the release version in this documentation pass; the maintainer
  deferred that work explicitly.

---

## Accepted acquisition protocol

`fc50dff` added `acquire_one_shot(&mut self, delay,
shutdown_settle_ms: u32)` to `Tmp108` and `AsyncTmp108`. Both returned
`Result<Celsius, OneShotError<I2C::Error>>`. The blocking method used
`embedded_hal::delay::DelayNs`; the async method used
`embedded_hal_async::delay::DelayNs` and awaited each I/O and delay.
The async API retained the existing `async` feature gate.

The accepted transaction order was:

1. Read configuration and write it back with `M = 0b00` (shutdown).
2. Delay the caller-supplied `shutdown_settle_ms`.
3. Re-read configuration and require raw `M == 0b00`. Any other value
   returned `PreparationNotShutdown(Mode)` without a trigger or a
   temperature read. This was a register consistency check, not proof
   of physical quiescence.
4. Trigger through another read-modify-write: read configuration, then
   write `M = 0b01`. This extra read was part of the landed protocol.
5. Poll at most eight times. Each iteration delayed 5 ms **first**,
   then read configuration. Raw `0b01` continued polling; raw `0b00`
   completed; raw `0b10` or `0b11` returned `UnexpectedMode(Mode)`.
6. After observed completion, read temperature exactly once and return
   it. Completion on the eighth poll still succeeded. Eight reads of
   `0b01` returned `Timeout` without reading temperature.

Any I²C failure returned `Bus(E)` immediately, including failure of
the final temperature read. The polling budget was eight requested
5 ms delays, not a 40 ms wall-clock deadline: bus and scheduling time
were additional. Preparation had its own, separate settling delay.

Success deliberately left the part in shutdown, as the chip cleared
the completed one-shot itself. The previous mode was not recorded or
restored. **The implementation made no cleanup write on failure.**
An unexpected mode, a timeout, a bus error, or async cancellation did
not establish a shutdown postcondition.

## Design decisions

### D1. Settling time belonged to the caller

The required `shutdown_settle_ms` argument was chosen instead of a
driver-selected constant. SBOS663A §7.4.1 says the device "shuts down
when current conversion is completed". Shutdown was therefore
deferred: acknowledgment of an `M = 0b00` write did not prove that an
active conversion had stopped. Re-reading `00` only read back what
software had written; it did not supply the missing timing evidence.

The Electrical Characteristics table gives one-shot conversion times
of 21 / 27 / 33 ms (minimum / typical / maximum) at +25 °C and
V+ = +1.8 V. Those conditions did not establish a bound over a
caller's board, supply and temperature range. The driver could not
know that bound and did not invent or validate one.

The rustdoc's 40 ms recommendation was accepted only as a **starting
point, not a guarantee**, with those conditions attached. The caller
remained responsible for validating sufficient settling time. The
post-settle read did not make an insufficient delay safe.

### D2. Interrupt-evidence loss was accepted and documented

**This operation was accepted as interrupt-destructive.** The full
polling budget performed 11 configuration reads: the initial
read-modify-write, the preparation check, the trigger's
read-modify-write, and eight polls. Earlier completion or failure
reduced that count, not the destructiveness of any read performed.

SBOS663A §7.5.3.4 states: "Reading the configuration register clears
both the flags and the pin." In interrupt thermostat mode each read
acknowledged FL/FH and released ALERT. The helper discarded those
flags; neither `Celsius` nor `OneShotError` reported the evidence lost.
Collecting pending evidence beforehand did not preserve new evidence
that might latch during acquisition.

The maintainer accepted this cost knowingly. #60 was field-visible;
blocking its remedy on an unscheduled redesign was the worse trade.
The warning was part of the design, not an incidental caveat. A
non-destructive variant was left as future work in #65, not promised
by this API.

### D3. Decoded error modes were deliberately lossy

`UnexpectedMode(Mode)` reported the functional mode, not the raw two
bits. `Mode` had three variants; since `bbcfbe5` (#62), `From<u8>`
mapped both `0b10` and `0b11` to `Continuous`. SBOS663A's Continuous
Conversion Mode (M1 = 1) section defined that mode by M1 alone.

Both encodings meant the same functional mode and required the same
recovery from an unexpected completion poll. Adding raw bits to the
public error would not have changed recovery, so the information loss
was accepted. `PreparationNotShutdown(Mode)` used the same decoded
representation. Internally, `ops::raw_mode`, `check_prepared` and
`classify_poll` still classified raw bits before building any error.

### D4. Normalisation belonged at the write-back boundary

`fb847e9` put the policy in `ops::apply_config`, shared by blocking and
async `configure()`. Only sampled raw `0b01` became `0b00`. Raw
`0b00`, `0b10` and `0b11` remained bit-for-bit unchanged. Decoding and
re-encoding every mode through `Mode` would have silently changed
`0b11` to canonical `0b10`, so the implementation called `set_m` only
for the encoding that needed to change.

Normalising the settings projection (`ops::decode_config`) would
have missed the offending sample. The hysteresis path was
**read + read + write**: `read_configuration()` projected the first
read into `Config`, then `configure()` performed a fresh
read-modify-write. The stale `M` that would be echoed came from the
**second** read. The policy had to run on that write-back snapshot,
not on the earlier settings value. `Config` did not acquire a mode
field to compensate.

Inspection also confirmed that `AsyncTmp108::continuous()` did
**not** route through `configure()`: it wrote `Mode::Continuous`
directly, ran the closure, then called `shutdown()`. Its own mode
transitions therefore did not pass through this normalisation policy.

Preservation of FL/FH in the outgoing snapshot was a codec property,
not preservation of interrupt evidence on silicon. The preceding
configuration read still had the side effect described in D2.

### D5. Exhaustive tests needed a new oracle, not less coverage

`apply_config_preserves_every_unmodelled_bit` had asserted that every
bit outside `MODELLED_MASK`, including `M`, survived unchanged. That
was exactly the false model exposed by #61. Merely excluding `M`
would have let arbitrary mode corruption pass; widening
`MODELLED_MASK` would have misrepresented what `Config` contained.

The test was reworked rather than relaxed:

- Preservation was asserted over `!MODELLED_MASK & !MODE_FIELD`.
  The full 65,536-word sweep remained, for both extreme `Config`s.
- A separate M-mapping test explicitly pinned `00 -> 00`, `01 -> 00`,
  `10 -> 10`, and `11 -> 11`, then swept all 65,536 words for both
  extreme `Config`s. Direct tests covered standing down an in-flight
  trigger and preserving raw `11` without canonicalisation.
- The two-sided bracket that pinned `MODELLED_MASK` itself stayed
  intact: `0xffff` had `M = 11` and `0x0000` had `M = 00`, neither
  subject to normalisation. Both driver shells gained wire-level
  tests for all four sampled encodings.

The precedent mattered: this repository had already shipped an
exhaustive test confidently asserting a false fact in #62, namely
that raw `M = 0b11` was reserved and must fail decoding. Exhaustive
coverage proved agreement with an oracle, not that the oracle agreed
with the chip. The corrected policy needed equally exhaustive tests
grounded in the command semantics.

---

## Public-API impact and migration

The #60 public API change was purely additive: two acquisition methods
and `OneShotError<E>` with `Bus`, `PreparationNotShutdown`,
`UnexpectedMode` and `Timeout`. `Error<E, P>` was unchanged; the new
error described acquisition failures without an unused GPIO parameter
or unrelated invalid-input variants.

The maintainer ruled #61 non-breaking. Busy-indicator preservation
across reconfiguration had never been a documented or supported
contract. `configure()` retained its signature, but a caller
hand-rolling a one-shot must not call it between trigger and completion
check. Writing `00` would destroy the evidence that a later `00` came
from chip completion. Configuration, including the delegating
hysteresis setter, belonged before acquisition.

`one_shot()` retained its signature and bare-trigger semantics. Its
documentation was corrected, and each doctest's initial reply changed
from continuous POR `0x1022` (`[0x22, 0x10]`) to shutdown `0x1020`
(`[0x20, 0x10]`). The expected trigger write stayed `0x1021`
(`[0x21, 0x10]`). The [Unreleased changelog][changelog] contains the
caller-facing before/after example and migration caveats; runnable
mock examples remain on the methods in `src/lib.rs`.

## Verification evidence

The landed tests separated pure decisions from transaction ordering.
`tests::ops_tests::one_shot_decisions` covered all
four raw modes against unrelated-bit patterns and pinned the 8 × 5 ms
budget. Blocking and async wire tests covered preparation rejection,
unexpected continuous modes, timeout, completion on the last poll,
and bus failure at each transaction stage. Their scripts asserted no
temperature read before completion and no trigger after failed
preparation.

A shared I²C-and-delay timeline additionally pinned the interleaving.
Independent bus and delay mocks alone could not detect a poll read
moved before its delay. These tests checked software ordering, not
the physical settling bound. Hardware confirmation of the original
defects did not establish a universal timing guarantee either.

## Open questions and follow-ups

- **Missing empirical number:** at bench conditions, how far can
  `shutdown_settle_ms` be swept downward from an active one-shot entry
  before preparation starts failing? No failure threshold was supplied
  for this record. The experiment must distinguish physical preparation
  failure from `PreparationNotShutdown`: reading software-written `00`
  can pass even if the delay was insufficient. A bench threshold would
  not become an operating-range guarantee.
- **#65:** a non-destructive acquisition variant remained future work.
- **Rustdoc drift for the Coder:** `fc50dff` claimed shutdown on every
  error path past the initial write, but neither shell performed error
  cleanup. It also described clearing the trigger in `configure()` as
  preventing completion from ever being reported; in fact software-written
  `00` could be mistaken for completion. This record described the
  implemented sequence without repeating either unsupported claim.
  Correcting `src/lib.rs` was outside this documentation pass.

## Source material

- `fb847e9` and `fc50dff`: implementation, rustdoc and test diffs.
- `bbcfbe5`: total `Mode` decoding and replacement of the #62 oracle.
- `src/lib.rs`: `ops::apply_config`, `raw_mode`, `check_prepared`,
  `classify_poll`; both `configure` and `acquire_one_shot` methods;
  `continuous`; `TemperatureHysteresis`; `tests::ops_tests::config_bits`,
  `tests::ops_tests::one_shot_decisions`, and `tests::timeline`.
- `src/inner.rs`: `Mode` and its conversions.
- [SBOS663A][datasheet] (revised September 2019): §7.4.1, §7.5.3.4,
  Electrical Characteristics, One-Shot Mode, Continuous Conversion Mode
  (M1 = 1), Temperature Register, and Configuration Register.
- [Type-split design][type-split], D3: shared codec convention. Its old
  "untouched bits (M, FL, FH, ID)" description was superseded by D4 here.
- [Reliability design][reliability], H4: its fixed-delay freshness advice
  was superseded by the supervised protocol and its caller-owned settle.

[datasheet]: https://www.ti.com/lit/gpn/tmp108
[changelog]: ../../../CHANGELOG.md#unreleased
[type-split]: 2026-06-04-tmp108-type-split-design.md
[reliability]: 2026-06-03-tmp108-reliability-fixes-design.md

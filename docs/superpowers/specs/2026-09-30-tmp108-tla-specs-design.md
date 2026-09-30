# TMP108 TLA+ Formal Specifications — Design Spec

**Date:** 2026-09-30
**Status:** approved; implementation in progress
**Branch:** `tla-specs` (from `code-review-skill`)
**Audience:** Maintainers familiar with the driver, the SBOS663A
datasheet, and at least the shape of TLA+. This records accepted
decisions and the evidence behind them. It is not a TLA+ tutorial.

---

## Problem statement

The driver's contract is currently expressed in three places that
cannot be checked against one another: the datasheet prose, the `#
Examples` doctests, and a ~5,800-line `mod tests` that pins wire-level
transaction sequences. Each of those pins *one* trace. None of them
answers a question of the form "over every interleaving of a drifting
temperature, a failing bus, and a cancelled future, can the driver ever
report a stale reading, lose an acknowledged alert, or leave the part
converting?"

Those are the questions a model checker answers and a unit test cannot,
because the property quantifies over behaviours rather than examples.

This work adds a formal specification of the part (`Tmp108Hw.tla`), a
formal specification of the driver composed with it
(`Tmp108Driver.tla`), a pure-arithmetic codec specification
(`Tmp108Codec.tla`), and a runner that executes all of them as a quick
local verification gate.

## Goals

1. Encode the chip's observable behaviour from the vendor
   documentation, with every fact carrying a `docs/vendor/datasheet.txt`
   line anchor or a recorded bench measurement.
2. Encode the driver's behaviour faithfully enough that a counterexample
   is a real defect rather than a modelling artefact.
3. Check properties the test suite structurally cannot express:
   freshness of a one-shot reading, cleanup of `continuous()`, and
   non-loss of an acknowledged alert obligation.
4. Run the whole suite in well under a minute so it is usable as a gate.
5. Record the three defects found during this work as *checked* artefacts
   rather than prose.

## Non-goals

- Proving the Rust implementation correct. These specs model the
  driver's documented and tested behaviour; they do not extract from
  source. A divergence between `src/lib.rs` and `Tmp108Driver.tla` is a
  maintenance hazard and is called out as such below.
- Modelling a second I²C master. Explicitly excluded by the maintainer.
  Every `modify()` is a non-atomic read-modify-write (`src/lib.rs:13-23`)
  and a second master would find that, but every counterexample would be
  "working as documented" and the state space cost is severe.
- Modelling real time. TLA+ here is untimed. `one_shot`'s budget is
  modelled as a poll *count*, so the gate cannot tell you whether 8 × 5 ms
  is physically sufficient.
- Wiring into CI. Needs a JDK and a `tla2tools.jar` that is neither
  vendored nor `cargo-vet`-able. Recorded as a follow-up.
- Changing any driver behaviour. Fixes for the defects below are
  separate work.

---

## Evidence gathered on silicon

A Pico de Gallo (firmware 0.9.0) with a TMP108 at `0x48`, ALERT on
`GPIO0`, was used to settle questions the datasheet leaves open or
states ambiguously. The part was restored to its as-found state
afterwards via a general-call reset.

| # | Question | Result | Method |
|---|---|---|---|
| E1 | Are `FL`/`FH` software-writable? | **No.** Writes ignored. | Shutdown the part so no conversion can race, write `0x34`/`0x2c`/`0x3c`, read back `0x24` each time. Positive control: a legitimately latched `FH` reads `0x34`, then `0x24`. |
| E2 | Does a non-config read clear the flags? | **No.** Only `0x01` clears. | Latch `FH`, read `0x00`, `0x02`, `0x03`, then `0x01` — still `0x34`. |
| E3 | Is raw `M = 0b11` a real held state that converts? | **Yes.** | Write `0x27`, reads back `0x27`; with `THIGH` below ambient, `FH` re-latches, so conversions are running. |
| E4 | Interrupt-mode ALERT semantics | Config read clears flags **and** releases the pin. | GPIO0 HIGH after read, LOW after re-latch, HIGH after next read. |
| E5 | Comparator-mode ALERT semantics | Pin survives a config read; deasserts on re-entering the band. | `TM=0`, `THIGH=20 °C`, ambient ~26 °C: pin LOW across a config read; raising `THIGH` to 40 °C deasserts. |
| E6 | Is the limit low nibble writable? | **No.** | Write `0x7ff8` → reads `0x7ff0`; write `0x1408` → reads `0x1400`. |
| E7 | What is the reset config value? | **`0x1026`** on this part. | `0x22 0x10` written, general-call reset, reads `0x26 0x10`. Reproduced three times. |
| E8 | Do the watchdog flags latch or track? | **Interrupt latches; comparator tracks.** | Drive an excursion, return in range, read *once* with no intervening config read. Interrupt reads `FH=1`, comparator reads `FH=0`. |
| E9 | In comparator mode, do the flags follow the raw limit or the hysteresis band? | **The band**, exactly as the pin does. | `HYS=4`, `THIGH=28`, ambient ~26: below the limit but inside the band, and `FH` stayed set. It cleared only at `THIGH=31`, when `T <= THIGH-HYS`. |

E1–E5 confirm the datasheet. E6 and E7 contradict repository
assumptions and are the basis of Findings 2 and 1 respectively.

E8 and E9 are **not stated in the datasheet at all**. `:919-927`
describes the flags generically and then discusses only interrupt mode.
The measured comparator behaviour — flags that track a hysteresis band
rather than latching an event — is what justifies the driver refusing to
treat entry flags as an event in comparator mode
(`src/lib.rs:6504`). The driver was already right; E9 records why, and
the specification now encodes it.

E9 also collapsed two planned variables out of the model. The design
originally carried `intAlert` and `cmpAlert` as separate latches; the
measurement showed the pin is simply `fl \/ fh` in **both** modes, with
the entire mode difference living in how the flags evolve.

---

## Defects found

### Finding 1 — `probe()` returns `Ok(false)` on a pristine part

`Tmp108::probe` is `Ok(u16::from_le_bytes(raw) == 0x1022)`
(`src/lib.rs:1769`). Per E7, this part resets to `0x1026`. Its
documentation (`src/lib.rs:1736-1744`) attributes a `false` result only
to the part having been "already reconfigured", which is not the case
here.

The root cause is not a wrong constant. It is that the reset value of
the configuration register is **not a device-identity constant at all**:

> Other options for the default values are available by request.
> — `docs/vendor/datasheet.txt:880`

The same caveat appears for the address bits at `:628`. So
`por_config_matches_default_configuration` (`src/lib.rs:3969`) pins
`ops::POR_CONFIG` against the DDSL reset value, which is
self-consistent and could never have caught this.

`probe()` additionally compares all 16 bits, including the volatile
`FL`/`FH` flags and `ID`, without justification.

**Encoded as:** `Tmp108Probe.cfg`, an expected-violation check. See
"Falsification tests" below.

### Finding 2 — `tmp108.ddsl` states the wrong resolution of a datasheet conflict

`tmp108.ddsl:98-106` says of the `THIGH` reset value:

> The datasheet contradicts itself about the encoding: 7.5.4 prose says
> 0x7FF8, while Table 11 shows the low nibble as fixed zero, implying
> 0x7FF0. Measured on a real TMP108: the part resets to 0x7FF8 […]
> The prose is correct; Table 11 is not.

Per E6 and E7 both statements are correct, about different things.
The prose (`docs/vendor/datasheet.txt:994`) describes the **reset
value**, which is `0x7FF8`. Table 11 (`:1004`) describes
**writability**, and the low nibble is genuinely read-only. There is no
contradiction to resolve.

Consequences:

- The part holds a `THIGH` value that no write can reproduce. Only a
  reset restores bit 3.
- `limit_registers_reset_to_the_documented_window` (`src/lib.rs:4427-4463`)
  asserts the driver emits `[0x03, 0x7f, 0xf8]`. That is the correct
  DDSL constant but a write the hardware will not honour, and the test's
  framing obscures this.
- Functional impact is nil: `0x7FF8 >> 4 == 0x7FF0 >> 4`, so the
  effective 12-bit threshold is identical. The defect is in the recorded
  rationale, not in behaviour.

**Encoded as:** `Tmp108Codec.tla`'s `LimitNibbleReadOnly`, which models
the write-side mask and checks that `RegToCelsius` is insensitive to it.

### Finding 3 — the flag-preservation comment overstates what is preserved

`ops::apply_config` echoes the sampled `FL`/`FH` back into the register,
commented (`src/lib.rs:541-543`) as "a read-modify-write must hand them
back exactly as sampled". Per E1 the write is ignored, so the echo is a
no-op; and per E4 the read that began the read-modify-write already
cleared the latch on the chip.

This is not a behavioural defect — the resulting register contents are
correct — but it means `configure`, `shutdown`, `probe`,
`read_configuration` and `wait_for_temperature` are all silent alert
acknowledgments. Only `one_shot` documents this
(`src/lib.rs:1899-1910`).

**Encoded as:** `Tmp108Hw.tla`'s `FlagsNotWritable` invariant, and
`Tmp108Driver.tla`'s `AcknowledgingOps` set, which makes the
acknowledging operations enumerable rather than implicit.

---

## Architecture

### Module layout

```
docs/tla/
  README.md            abstractions, measured facts, how to run
  Tmp108Hw.tla         the chip
  Tmp108Hw.cfg         chip invariants, no driver
  Tmp108Driver.tla     EXTENDS Tmp108Hw; the driver composed with the chip
  Tmp108Driver.cfg     driver properties, quick profile
  Tmp108DriverDeep.cfg wider constants, deep profile
  Tmp108Codec.tla      pure arithmetic, no state exploration
  Tmp108Codec.cfg
  Tmp108Probe.cfg      falsification test for Finding 1
scripts/check-tla.py   the gate
```

`Tmp108Hw.cfg` exists so that a chip-invariant failure is
distinguishable from a driver defect. Without it every counterexample is
ambiguous about which side is wrong.

### Composition: transaction-atomic, shared variables

`Tmp108Driver.tla` EXTENDS `Tmp108Hw.tla` and the two `Next` relations
are disjoined. Each driver step performs **at most one complete I²C
transaction**, applied atomically.

Rejected alternatives:

- *Bus-protocol-level model* (START / address / pointer / data / ACK per
  `datasheet.txt:661-691`, including the persistent pointer register).
  Faithful, but `Interface` (`src/lib.rs:2840`) writes the pointer
  before every single read, so pointer persistence is never exercised.
  Large cost, no reachable defect.
- *Two-phase request/response bus*, so cancellation can land
  mid-transaction. `write_read` is atomic from `embedded-hal`'s
  perspective; a cancellation inside it is unobservable to the driver.
  Doubles the state space to model something no caller can detect.

Reads are **actions, not pure functions**, because a configuration read
has a side effect (`datasheet.txt:926`, confirmed by E4). This is the
single most important structural decision in `Tmp108Hw.tla`.

### The units abstraction

The state machine models temperature in **whole degrees Celsius**.

Hysteresis is 0/1/2/4 °C (`datasheet.txt:902-907`) — that is 0/16/32/64
sixteenths. A sixteenths-denominated domain small enough to
model-check exhaustively cannot represent even a 1 °C hysteresis band,
which would make the comparator-mode logic vacuous.

Sub-degree resolution affects no mode, flag or ALERT decision. It lives
entirely in the encoding, which `Tmp108Codec.tla` checks at full
4096-value fidelity as constant expressions — no state exploration, so
it costs milliseconds.

The seam between the two is an explicit assumption, recorded in
`docs/tla/README.md`: that `RegToCelsius` is monotone, so that ordering
comparisons in degrees agree with ordering comparisons in raw counts.
`Tmp108Codec.tla` checks that monotonicity over the whole domain, so the
seam is verified rather than asserted.

### What `Tmp108Hw.tla` models

Variables, restricted to the behaviourally relevant:

| Variable | Domain | Source |
|---|---|---|
| `ambient` | `MinTemp..MaxTemp` | drifts ±1 per step |
| `mode` | `0..3` — four values | `:753`, E3 |
| `tm`, `pol`, `hys` | enumerations | `:914`, `:910`, `:902` |
| `fl`, `fh` | BOOLEAN, read-only status | `:919-927`, E1 |
| `tlow`, `thigh`, `tempReg` | `MinTemp..MaxTemp` | `:979`, `:815` |
| `converting` | BOOLEAN | `:741` |
| `intAlert` | BOOLEAN | `:926`, E4 |
| `cmpAlert` | BOOLEAN | `:980-985`, E5 |
| `convEpoch` | Nat, history | needed for freshness |

`cr` and `id` are **omitted** from the state machine. They affect
nothing observable in an untimed model, and multiplying the state space
by eight to carry inert bits is not defensible. The full 16-bit
`Config` codec, including `cr` and `id`, is checked exhaustively in
`Tmp108Codec.tla` instead — the same division of labour the Rust tests
already use.

`intAlert` is a separate variable from `fl`/`fh` because
`datasheet.txt:926` says the SMBus alert response "only clears the pin
and not the flags", so the pin latch and flag latches are physically
distinct. The driver never uses ARA, so they move together here — which
becomes the checkable invariant `IntAlertTracksFlags` rather than a
silent assumption.

`mode` is 4-valued, not 3-valued. `0b11` is a real state the part holds
and converts in (E3, and `src/lib.rs:3947` on silicon per issue #62).

`PorConfig` is a **`CONSTANT`**, not a literal. This is what allows the
same specification to express both the datasheet's `0x1022` and the
measured `0x1026`, and is the mechanism by which Finding 1 becomes a
checked result.

### What `Tmp108Driver.tla` models

Sequential by construction — `&mut self` means exactly one operation is
in flight.

| Variable | Purpose |
|---|---|
| `pc` | `[op, step, polls]`, the transaction boundary |
| `scratch` | values held across steps |
| `pending` | `interrupt_sample_pending`, 5 values (`src/lib.rs:1233-1239`) |
| `ret` | result of the last completed call |

Operations: `probe`, `read_configuration`, `temperature`, `configure`,
`shutdown`, `set_low_limit`, `set_high_limit`, `wait_for_temperature`,
`one_shot`, `continuous`, `wait_for_alert`. A `CONSTANT EnabledOps`
lets a profile narrow the set.

**Bus errors.** Every I²C step branches: succeed, applying the
`Tmp108Hw` action; or fail, leaving the chip unchanged and taking the
driver's error path. This alone makes each `modify()` visibly
non-atomic — a failed write after a successful read leaves a half-applied
update — without needing a second master.

**Cancellation.** A `Cancel` action is enabled at any `pc` corresponding
to an `.await`, returning to `Idle` with no cleanup. Async only.

### Properties

Chip (`Tmp108Hw.cfg`):

| Property | Statement | Evidence |
|---|---|---|
| `HwTypeOK` | type correctness | — |
| `IntAlertTracksFlags` | `tm = Interrupt => intAlert = (fl \/ fh)` | `:926`, E4 |
| `ComparatorQuiescent` | `~cmpAlert => tlow <= tempReg <= thigh` | `:980-985`, E5 |
| `FlagsNotWritable` | a config write never changes `fl`/`fh` | E1 |
| `OnlyConfigReadClears` | reads of `0x00`/`0x02`/`0x03` preserve flags | `:926`, E2 |
| `OneShotSelfClears` | `mode = 1 ~> mode = 0` | `:746` |

Driver (`Tmp108Driver.cfg`):

| Property | Statement |
|---|---|
| `DriverTypeOK` | type correctness |
| `NoStaleOneShot` | `one_shot` returning `Ok(t)` implies `t` came from a conversion with `epoch > triggerEpoch` |
| `ContinuousOkImpliesShutdown` | `Ok` from `continuous` implies `mode = 0` |
| `CancelledContinuousLeaks` | cancelling inside `continuous` implies `mode = 2` |
| `NoLostAcknowledgedAlert` | past the arming point (`src/lib.rs:1680`), a debt is never silently dropped |
| `AtMostOneObligation` | `pending` holds at most one obligation |

`NoStaleOneShot` requires the `convEpoch` history variable. "This
reading is fresh" is a claim about *which* conversion produced the
value, and is inexpressible without it.

`CancelledContinuousLeaks` asserts a documented defect as a *truth*.
`continuous()` has no `Drop` guard (`src/lib.rs:2600-2608`), so
cancellation leaves the part converting, always. Asserting it means the
day someone adds a guard, this check turns red and forces the
documentation to be updated in the same change.

### Falsification tests

`Tmp108Probe.cfg` sets `PorConfig = 0x1026` — the measured value — and
checks `ProbeDetectsPristineChip`, which says a `probe()` issued against
a freshly reset part returns `TRUE`. It **fails**, and that failure is
the deliverable.

The runner therefore supports a per-entry `expect` of `OK` or
`VIOLATION`. A `VIOLATION` entry passes only when TLC reports a
violation of the **named** invariant, so it cannot be satisfied by an
unrelated error or by a spec that fails to parse.

This keeps the gate green today while encoding Finding 1 as an
executable artefact. When `probe()` is fixed, the check turns red and
the fixer must update the spec — which is the correct workflow. The
alternative, a commented-out check or a prose note, decays silently.

### The runner

`scripts/check-tla.py`, Python 3, **standard library only** — no pip
dependencies, therefore no new `cargo-vet` or CI install surface.

- Jar resolution: `--jar`, then `$TLA2TOOLS_JAR`, then
  `~/Downloads/tla2tools.jar`, then `PATH`. A missing jar produces one
  clear line, not a stack trace.
- Invocation: `java -XX:+UseParallelGC -cp <jar> tlc2.TLC -workers auto
  -cleanup -config <cfg> <module>`, `cwd=docs/tla`, no `shell=True`,
  `pathlib` throughout. Works on Windows and POSIX.
- Deadlock checking stays **on**. Every driver operation returns to
  `Idle` and `Drift` is always enabled, so a deadlock would indicate a
  modelling error.
- Output: one line per spec with status, wall time and distinct state
  count; full TLC trace printed verbatim on an unexpected result.
- Flags: `--deep`, `--only`, `--jar`, `--timeout` (default 120 s),
  `--verbose`, `--list`.
- Exit 0 iff every entry matched its expectation.

Budget: the quick profile targets under ~30 s total. The tuning knobs
are the temperature domain and `EnabledOps`. If `Tmp108Driver` overruns,
the domain shrinks first — operation coverage is what makes the
properties meaningful, so it is the last thing to cut.

---

## Risks and limitations

**A TLC arithmetic trap, found during implementation.** TLC's `\div`
truncates toward zero for negative operands (`-1 \div 16 = 0`) while its
`%` floors (`-1 % 16 = 15`); the two are mutually inconsistent. Rust's
`>>` on a signed integer is an arithmetic shift, i.e. floor division.
Using `\div` directly would have made every negative temperature in the
model disagree silently with the driver. `Tmp108Codec` defines `FloorDiv`
and asserts both halves of the reasoning in `DivIsFloorDivision`, so a
future TLC release that fixes the inconsistency fails loudly instead of
quietly changing what the model means.

This is worth recording because the guard that caught it —
`DivIsFloorDivision` — was written speculatively, before there was any
reason to suspect a problem. It failed on the first run.

**The specs can drift from `src/lib.rs`.** Nothing mechanically ties
them together. A behavioural change to the driver will not fail the
gate. Mitigation: `docs/tla/README.md` states which `src/lib.rs` line
ranges each operation models, and `AGENTS.md` gains a "Where to put new
things" row directing new driver methods to update the spec. This is a
documentation-grade guarantee, not a mechanical one, and is stated
plainly rather than glossed.

**The abstraction could hide a defect.** Omitting `cr`/`id`, modelling
degrees instead of sixteenths, and excluding a second master all shrink
the reachable state space. Each omission is recorded in
`docs/tla/README.md` with its justification so a future reader can
challenge it.

**E7 rests on a single part.** The conclusion for `probe()` does not,
because `datasheet.txt:880` makes the reset value non-identifying
regardless. But the specific value `0x1026` is one sample; the spec
carries it as a `CONSTANT` precisely so it is not load-bearing.

**Model checking is not proof.** The gate explores a bounded domain.
A property holding at `MinTemp..MaxTemp = -4..8` is evidence, not a
theorem.

---

## Follow-up work

1. Fix `probe()` — mask the volatile bits at minimum; more likely,
   reconsider whether a reset-value comparison can identify the part at
   all given `datasheet.txt:880`.
2. Correct the `tmp108.ddsl:98-106` rationale, and reframe
   `limit_registers_reset_to_the_documented_window`.
3. Document the acknowledging reads on `configure`, `shutdown`, `probe`,
   `read_configuration` and `wait_for_temperature`, as `one_shot`
   already does.
4. Consider a CI job once a jar-provisioning story exists.

Each is separate work and none is in scope here.

[datasheet]: https://www.ti.com/lit/gpn/tmp108

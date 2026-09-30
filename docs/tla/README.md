# TLA+ specifications

Formal specifications of the TMP108 part and of this crate's driver,
plus a runner that executes them as a quick verification gate.

They answer questions the Rust test suite structurally cannot. `mod
tests` pins one transaction trace per test; a model checker quantifies
over *every* interleaving of a drifting temperature, a failing bus and a
cancelled future. "Can `one_shot` ever report a stale reading?" is that
kind of question.

## Running them

```bash
python scripts/check-tla.py            # quick profile, about a minute
python scripts/check-tla.py --deep     # adds the wide-domain profile
python scripts/check-tla.py --list     # show the manifest
python scripts/check-tla.py --only Hw  # one entry
```

Needs a JRE 11+ and `tla2tools.jar`, found via `--jar`, then
`$TLA2TOOLS_JAR`, then `~/Downloads/tla2tools.jar`, then `PATH`. Get the
jar from <https://github.com/tlaplus/tlaplus/releases>.

Measured on a 20-core machine:

| Check | Distinct states | Time |
|---|---|---|
| `Tmp108Codec` | 1 (constant expressions over the full domain) | ~9 s |
| `Tmp108Hw` | 29,040 | ~9 s |
| `Tmp108Driver` | 1,815,240 | ~30 s |
| `Tmp108Probe` | 86 | ~8 s |
| `Tmp108DriverDeep` (opt-in) | 3,907,176 | ~56 s |

**`Tmp108Probe` is expected to FAIL.** See "Falsification tests" below.
A green gate means every check matched its declared expectation, not that
every check passed.

## The modules

| File | Covers |
|---|---|
| `Tmp108Codec.tla` | Pure arithmetic: `Celsius` ↔ register word, `Config` ↔ 16-bit word, `apply_config`. No state. Checked over **all** 65,536 words and 4,096 temperatures. |
| `Tmp108CodecCheck.tla` | A one-state machine so TLC names a failing codec property instead of reporting an anonymous false assumption. `Tmp108Codec` declares no `VARIABLE` so `Tmp108Hw` can extend it. |
| `Tmp108Hw.tla` | The part, from the datasheet and the bench. Registers, modes, conversion, watchdog flags, ALERT. Knows nothing about the driver. |
| `Tmp108Driver.tla` | The driver, composed with the chip by shared variables. |

`Tmp108Hw.cfg` checks the chip alone. It exists so that a chip-model
defect is distinguishable from a driver defect; without it every
counterexample is ambiguous about which side is wrong.

## What the driver model covers

Each operation is a step machine whose steps are complete I²C
transactions.

| Operation | `src/lib.rs` | Transactions modelled |
|---|---|---|
| `probe` | 1767 | `R(1)` |
| `read_configuration` | 1792 | `R(1)` |
| `temperature` | 1869 | `R(0)` |
| `configure` | 1845 | `R(1) W(1)` |
| `shutdown` | 2056 | `R(1) W(1)` |
| `set_low_limit` / `set_high_limit` | 2185, 2238 | `W(2)` / `W(3)`, no preceding read |
| `one_shot` | 2003 | `R(1) W(1) R(1) R(1) W(1) [R(1)]* R(0)` |
| `continuous` | 2645 | `R(1) W(1)` · closure · `R(1) W(1)` |
| `wait_for_alert` | 1594 | retained `R(0)`; comparator `R(1) R(0)`; interrupt fast `R(1) R(0)`; interrupt slow `R(1) R(1) R(0)` |

`one_shot`'s shape mirrors `expected_one_shot_timeline`
(`src/lib.rs:4313-4335`), including the redundant back-to-back read of
register 1 at steps 3 and 4.

## Properties

**Chip** — `Tmp108Hw.cfg`

- `HwTypeOK`
- `AlertMatchesFlags` — the pin is exactly `fl \/ fh`. True only because
  SMBus ARA, the one operation that separates them
  (`datasheet.txt:926`), is not modelled; the driver never issues one.
- `ReservedBitsAreZero` — the premise for leaving `ID` and the reserved
  bits out of the state.
- `ComparatorQuiescent` — in comparator mode, clear flags mean the die is
  inside the limit window.
- `OneShotReturnsToShutdown` — an action property, not a leads-to. See
  the comment in the module for why the leads-to is unfalsifiable here.

**Driver** — `Tmp108Driver.cfg`

- `DriverTypeOK`
- `NoStaleOneShot` — **the prize.** A successful `one_shot` never reports
  a conversion that predates its own trigger. This is the defect behind
  issue #60. Expressible only because of the `tempFresh` marker:
  freshness is a claim about *which* conversion produced the value.
- `ContinuousOkImpliesShutdown`
- `CancelledContinuousLeaks` — a documented defect asserted as a truth;
  see below.
- `ObligationClearedOnlyBySettlement` — an acknowledged alert obligation
  is cleared only by the step that successfully reports it. In
  particular a cancellation must not clear it, which is why
  `src/lib.rs:1597` copies the retained cause rather than taking it.
- `ObligationOnlyFromInterrupt` — comparator alerts are levels, not
  events (E9), so they carry no debt.

## Falsification tests

`Tmp108Probe.cfg` runs `Tmp108Driver` with `PorConfigWord = 0x1026` — the
value measured on a real part (E7) — and checks
`ProbeDetectsPristineChip`. It fails, and **that failure is the
deliverable**: `probe()` hardcodes `0x1022` (`src/lib.rs:1769`) while
`datasheet.txt:880` says "other options for the default values are
available by request".

The runner requires an expected-violation entry to violate the *named*
invariant, so it cannot be satisfied by an unrelated error or by a spec
that no longer parses.

`CancelledContinuousLeaks` works the same way in the opposite direction.
`continuous()` has no `Drop` guard (`src/lib.rs:2600-2608`), so
cancellation leaves the part converting. Asserting the leak — rather than
asserting the opposite and tolerating a red check — means the day someone
adds a guard, this turns red and forces the docs and this spec to be
corrected together. A commented-out check decays silently.

## Bench measurements

Questions the datasheet leaves open or states ambiguously, settled on a
TMP108 at `0x48` on a Pico de Gallo (firmware 0.9.0), ALERT on `GPIO0`.
The part was restored afterwards with a general-call reset.

| # | Question | Result |
|---|---|---|
| E1 | Are `FL`/`FH` software-writable? | **No**, writes ignored |
| E2 | Does a non-config read clear the flags? | **No**, only register `0x01` |
| E3 | Is raw `M = 0b11` a real held state that converts? | **Yes** |
| E4 | Interrupt-mode ALERT | config read clears flags *and* releases the pin |
| E5 | Comparator-mode ALERT | pin survives a config read; deasserts on re-entering the band |
| E6 | Is the limit low nibble writable? | **No** |
| E7 | Reset value of the configuration register | **`0x1026`** on this part, not `0x1022` |
| E8 | Do the flags latch or track? | **Interrupt latches, comparator tracks** |
| E9 | In comparator mode, do flags use the raw limit or the band? | **The hysteresis band**, same as the pin |

E1, E8 and E9 are the ones the datasheet does not state. E6 and E7
contradict repository assumptions; see Findings 1 and 2 in
[the design spec](../superpowers/specs/2026-09-30-tmp108-tla-specs-design.md).

Reproducing E1, the sharpest one — note the part must be in **shutdown**
first, or a conversion races in and rewrites the flags, and note the
positive control, without which a read of `24` proves nothing:

```bash
gallo i2c write      -a 0x48 -b 0x01 0x24 0x10   # shutdown, TM=1
gallo i2c write-read -a 0x48 -b 0x01 -c 2        # baseline: 24 10
gallo i2c write      -a 0x48 -b 0x01 0x34 0x10   # try to set FH
gallo i2c write-read -a 0x48 -b 0x01 -c 2        # 24 10  => ignored

# positive control: a genuinely latched FH does read back
gallo i2c write      -a 0x48 -b 0x03 0x14 0x00   # THIGH = 20 C, below ambient
gallo i2c write      -a 0x48 -b 0x01 0x25 0x10   # one-shot
gallo i2c write-read -a 0x48 -b 0x01 -c 2        # 34 10  => FH set, M back to 00
gallo i2c write-read -a 0x48 -b 0x01 -c 2        # 24 10  => the read cleared it
```

E8/E9 need care: any config read clears the flags, so an intervening
read destroys the evidence. Drive the excursion, return the temperature
in range, and read **once**.

## Abstractions, and why each is defensible

Every one of these shrinks the reachable state space. They are listed so
a future reader can challenge them.

**Temperature is whole degrees, on a non-negative abstract scale.**
Hysteresis is 0/1/2/4 °C (`datasheet.txt:902-907`) — 0/16/32/64
sixteenths. A sixteenths domain small enough to model-check could not
represent even a 1 °C band, making the comparator logic vacuous.
Sub-degree resolution affects no mode, flag or ALERT decision. The zero
point is arbitrary because every comparison the part makes is a
*difference*. The real signed range is covered exhaustively by
`Tmp108Codec`, where sign handling actually lives, and the seam between
the two — that decoding is monotone, so ordering in degrees agrees with
ordering in raw counts — is *checked* there as `DecodeIsMonotone`, not
assumed.

**`CR` and `ID` are not chip state.** Neither influences anything
observable in an untimed model. `CR` only sets a delay; `ID` and the
reserved bits are zero at reset and no modelled write sets them
(`ReservedBitsAreZero` checks the premise). Their encodings are checked
over all 65,536 words in `Tmp108Codec`. This is the same division of
labour the Rust suite already uses.

**Transactions are atomic.** `write_read` is atomic from
`embedded-hal`'s perspective, so a cancellation inside one is
unobservable to the driver. A two-phase bus would double the state space
to model something no caller can detect.

**No second bus master.** Every `modify()` is a non-atomic
read-modify-write (`src/lib.rs:13-23`) and a second master would find
that, but each counterexample would be a documented hazard rather than a
defect, at severe state-space cost. Excluded deliberately.

**No real time.** The one-shot budget is a *poll count* here, so this
spec cannot tell you whether 8 × 5 ms is physically sufficient. Nothing
in `src/lib.rs` argues that it is either; `src/lib.rs:4093` pins the
constants without justifying them.

**`flagsCurrent` is auxiliary.** It carries no hardware meaning and the
driver cannot observe it. It exists so `ComparatorQuiescent` can be
stated at all, because the flags are only re-evaluated at the end of a
conversion — a limit written afterwards leaves them judging the old
window.

## A TLC trap worth knowing

**TLC's `\div` truncates toward zero for negative operands while its `%`
floors.** `-1 \div 16` is `0`, but `-1 % 16` is `15`; the two are
mutually inconsistent. Rust's `>>` on a signed integer is an *arithmetic*
shift, i.e. floor division. Using `\div` directly would have made every
negative temperature in the model disagree silently with the driver —
`0xFFFF` decodes to `-1` in Rust (`src/lib.rs:3589`) and would have
decoded to `0` here.

`Tmp108Codec` therefore defines `FloorDiv` and asserts both halves of the
reasoning in `DivIsFloorDivision`, so that a future TLC release which
fixes the inconsistency fails loudly rather than quietly changing what
the model means.

## Known limitation: these specs can drift

Nothing mechanically ties `Tmp108Driver.tla` to `src/lib.rs`. A
behavioural change to the driver will **not** fail this gate.

The mitigation is the operation table above, which records what each step
machine models and where, plus a row in `AGENTS.md` directing new driver
methods here. That is a documentation-grade guarantee, not a mechanical
one, and it is stated plainly rather than glossed.

Model checking is also not proof. These checks explore a bounded domain;
a property holding at `MaxTemp = 4` is evidence, not a theorem.

## Not wired into CI

A CI job would need a JDK plus a jar that is neither vendored nor
`cargo-vet`-able. Left as a follow-up rather than bolted onto
`check.yml`.

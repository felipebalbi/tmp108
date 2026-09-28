# Configuration-read acknowledgement: documentation and a flags-preserving read

Design for [issue #65][issue-65]. Covers both the issue's **Short**
action (document the interrupt-mode acknowledgement effect on every
entry point that reads the configuration register) and its **Medium**
action (add an explicitly-named operation that returns FL/FH alongside
the settings).

Baseline: `c007930b0c76`.

[issue-65]: https://github.com/OpenDevicePartnership/tmp108/issues/65

---

## The problem

In `Thermostat::Interrupt` mode, reading the TMP108 configuration
register **is** acknowledging an alert: the read clears FL and FH and
releases the ALERT pin (datasheet SBOS663A §7.5.3.4). Thirteen public
method instances across the two driver flavors read it directly, and
two further trait methods reach it indirectly (A4). Two of the thirteen
say so. The other eleven do not, and the flags the read consumed are
discarded by `ops::decode_config` (`src/lib.rs:476`) before any caller
can observe them.

A caller in interrupt mode therefore has no way to know that `probe()`
— or `wait_for_temperature()`, which reads configuration *only* to
compute a sleep duration — destroys a pending alert. The information
is not merely undocumented but unrecoverable: `Config` is a settings
projection (2 × 2 × 4 × 4 = 64 inhabitants, exactly the documented
field combinations) being returned from an operation that also
sampled, and destroyed, three bits of hardware status.

## Evidence

The datasheet claim was re-verified on hardware for this design rather
than inherited from the issue. Rig: Pico de Gallo `5256657D8A5D7F03`
(hw rev 2, fw 0.11.0, schema 0.7) with a TMP108 at `0x48`, ALERT on
GPIO0. Raw I²C, no driver involved.

Setup: interrupt mode (`TM=1`), `T_high` = 10 °C against an ambient of
22.125 °C, so the over-temperature condition is true and stays true
throughout. After FH latched, the part was written into **shutdown**
(`M=0b00`) so that no further conversion could re-assert the flag —
this removes the re-latch race that makes the naive version of this
experiment ambiguous.

```
write config [04,10]   (shutdown)    -> a write does NOT clear FH
gpio0 = LOW                          -> ALERT asserted
read #1  [14,10]   FH=1              -> flag present; this read acknowledges
gpio0 = HIGH                         -> ALERT released, by the read alone
read #2  [04,10]   FH=0
read #3  [04,10]   FH=0
gpio0 = HIGH                         -> stays released
```

Nothing about the physical condition changed between read #1 and read
#2, and no conversion could occur. The only intervening event was read
#1 itself.

Two facts fall out that the issue does not state, and that this design
relies on:

1. **A configuration *write* does not acknowledge.** FH survived the
   shutdown write and was still set on the next read. This matters for
   `configure()`, which is a read-modify-write: it is the **read** half
   that acknowledges, not the write-back.
2. **The pin releases on the read**, not only the flags. The issue's
   own hardware comment measured the flags only.

## Non-goals

- Changing `read_configuration`'s return type. `Config` is exported and
  v0.6.0 is published; that would be a breaking migration for
  downstream users. The issue says so explicitly and this design
  honors it.
- Changing any driver's I²C traffic. No existing method gains, loses,
  or reorders a transaction.
- Exposing `M`. See "Rejected alternatives".
- Preventing acknowledgement. There is no wrapper type that can stop a
  configuration read from clearing the flags; the chip does that. What
  a richer return type *can* do is preserve the flags the read already
  consumed, so the event survives to the caller instead of being
  dropped on the floor.

---

## Part A — Documentation

### A1. One canonical section

Add `## Interrupt-mode acknowledgement` to the crate-level `//!` docs,
under the existing `# Operational notes` (`src/lib.rs:11`) and as a
sibling of `## I²C bus ownership` (13) and `## Driver lifecycle on
drop` (25). Those lines currently contain no alert content at all —
the words "alert", "FL", "FH", "interrupt" and "7.5.3.4" do not appear
anywhere in `src/lib.rs:1-41`.

The section states:

- The rule, cited to SBOS663A §7.5.3.4.
- The read/write asymmetry established above: reads acknowledge,
  writes do not.
- That the effect applies only in `Thermostat::Interrupt`; in
  `Thermostat::Comparator` the pin tracks the temperature and a
  configuration read does not release it.
- That acknowledgement lands when the read lands, so a method that
  fails or is cancelled *after* its configuration read has already
  acknowledged. No error variant reports this.
- A table of every acknowledging entry point and its config-read count
  per call.
- A pointer to `read_configuration_and_acknowledge` (Part B) as the
  only operation that hands the flags back.

This is the single authoritative statement. Everything else links here.

### A2. Eleven per-method sections

Each gets a `# Interrupt-mode acknowledgement` section of three to five
lines: what this specific method does, how many configuration reads it
performs, and an intra-doc link to A1 for the full rule. Deliberately
short — the file already demonstrates what happens when the full text
is duplicated (`one_shot`'s twelve-line block appears byte-identically
at 1899 and 2410).

| Method | Blocking | Async | Reads | Per-method emphasis |
|---|---|---|---|---|
| `probe` | 1767 | 2269 | 1 | Looks like a pure liveness check; is not. |
| `read_configuration` | 1792 | 2296 | 1 | Returns *configurable parameters*, not complete hardware state. FL, FH and M are sampled and discarded. |
| `configure` | 1845 | 2351 | 1 | Read-modify-write: the **read** acknowledges. The preserved-flags note at 1799–1804 is about the write-back and does not cover this. |
| `shutdown` | 2056 | 2585 | 1 | Read-modify-write, same as above. |
| `wait_for_temperature` | 2130 | 2695 | 1 | Reads configuration *only* to pick a delay. The existing `# I²C cost per call` section (2098) names the read as bus cost, not as an acknowledgement. |
| `continuous` | — | 2645 | 2 | Entry `modify_async` (2649) and the cleanup `shutdown()` (2660). Twice per call, including on the cleanup path after the closure fails. |

The async `wait_for_temperature` (2695) currently defers to the
blocking one (2666–2669). It keeps deferring, and the blocking one
gains the section.

### A3. Trim the two existing `one_shot` blocks

`Tmp108::one_shot` (1899–1910) and `AsyncTmp108::one_shot` (2410–2421)
already carry `# This operation is interrupt-destructive`. They shrink
to the A2 shape and link to A1, keeping the two facts specific to them:
roughly ten configuration reads per call, and that the loss is not
reported in the return value.

### A4. The two undocumented trait paths

Not in the issue's table, found while surveying:

- `TemperatureHysteresis::set_temperature_threshold_hysteresis` for
  `AsyncTmp108` (`src/lib.rs:3190`)
- the same for `AlertTmp108` (`src/lib.rs:3211`), which delegates to it

Each performs **two** configuration reads per call — `read_configuration()`
at 3201 followed by `configure()` at 3203, which is itself a
read-modify-write. Neither has any rustdoc; they carry only inline `//`
comments about hysteresis snapping. Both get real rustdoc including the
two-read count.

This is the worst case in the crate: the method most likely to be
called while an alert is pending is the one configuring alert
hysteresis, and it acknowledges twice.

### A5. `AlertTmp108::sensor_mut`

Its doctest (`src/lib.rs:1412`) demonstrates
`tmp.sensor_mut().read_configuration().await` on a wrapper whose entire
purpose is alert handling, with no warning. The existing doc (1395–1398)
covers the retained-delivery obligation but not acknowledgement.

The doctest changes to `read_configuration_and_acknowledge()`, which is
the correct call on that type, and the prose gains a sentence pointing
at A1. This is the one place Part A depends on Part B, which is why the
`feat:` commit lands first.

### A6. README

`## Gotchas` (README.md:115–190) covers the waiter path only. The bullet
at 126–130 is actively reassuring — "the pin clears as soon as the
configuration register is read (the driver does this for you inside
`wait_for_temperature_threshold`)" — which is true and, read alone,
implies the driver handles it wherever it matters.

Add one bullet for the general case: any configuration read
acknowledges, name the surprising entry points, and point at
`read_configuration_and_acknowledge`. The existing 126–130 bullet stays
as-is; it is not wrong.

---

## Part B — `read_configuration_and_acknowledge`

### B1. Promote `AlertSnapshot`

`ops::AlertSnapshot` (`src/lib.rs:500`) already exists with exactly the
right shape and a doc comment that already says the right things
(485–497). It is `pub(crate)` and gated on
`embedded-sensors-hal-async`.

Promote it to a public crate-root type, **ungated**:

```rust
pub struct AlertSnapshot {
    pub config: Config,
    pub low: bool,
    pub high: bool,
}
```

Ungating is the point. FL and FH exist regardless of which cargo
features are on, and the blocking `Tmp108` has no `AlertTmp108` and no
waiter at all — a blocking user in interrupt mode has, today, no way
whatsoever to observe the flags. `AlertCause` is already ungated public
surface for the same reason (its doc at 130–132 notes the type is
always available even though its only current producer is not).

Derives: `Clone, Copy, Debug, PartialEq, Eq, Hash`, matching
`AlertCause` (147) and `AlertEvent` (196). The current `pub(crate)`
version omits `Hash`; public status is the reason to add it.

Not `#[non_exhaustive]`. See "Rejected alternatives".

`ops::decode_alert_snapshot` (511) is ungated to match, as is its
existing exhaustive test module (3338). `ops::interrupt_alert_cause`
(530) stays gated — it is waiter-internal and its contract ("converting
an arbitrary empty snapshot through it must never create an event",
527–528) is specific to an already-qualified notification. Its gate is
also what keeps the `AlertCause` import in `mod ops` (225–226) gated.

### B2. The method

On both `Tmp108` and `AsyncTmp108`:

```rust
pub fn read_configuration_and_acknowledge(&mut self)
    -> Result<AlertSnapshot, I2C::Error>
```

Body is a thin shell over the existing pure codec, per AGENTS.md:

```rust
let c = self.inner.configuration().read()?;
Ok(ops::decode_alert_snapshot(c))
```

Identical I²C traffic to `read_configuration` — one write-read of
register `0x01`. The only difference is how much of the result survives
decoding.

**Naming.** `read_configuration` parallelism gives discoverability; the
`_and_acknowledge` suffix puts the effect in the name, which is what
the issue asked for ("an explicitly-named snapshot/acknowledge
operation"). It is long, and that is deliberate: the shorter candidates
(`read_alert_snapshot`, `snapshot`) all read as non-destructive status
queries, which is precisely the misconception this issue exists to
correct.

**Return shape.** Raw `low`/`high` bools rather than `AlertCause`.
`AlertCause` cannot express "no alert": its `Unknown` variant is
documented as "An alert was observed, but its direction cannot be
established … It is not an error and does not mean 'no alert'" (157–162).
A speculative snapshot with both flags clear has no honest `AlertCause`,
and manufacturing one would violate the contract
`ops::interrupt_alert_cause` states at 519–528. The two flags are
independent and all four combinations occur; bools say that and nothing
more.

### B3. Collapse the duplicate implementation

`AlertTmp108::read_alert_snapshot` (`src/lib.rs:1718`) is private and
does exactly this already:

```rust
let c = self.sensor_mut().inner.configuration().read_async().await?;
Ok(ops::decode_alert_snapshot(c))
```

It becomes a delegation to `AsyncTmp108::read_configuration_and_acknowledge`,
leaving one implementation. Its existing doc comment (1701–1717) — the
strongest acknowledgement statement in the crate — stays where it is,
because it documents the *waiter protocol's* use of the read (when to
call it, how evidence transfers into the delivery obligation), which is
`AlertTmp108`-specific and does not belong on the general method.

### B4. Semver

Purely additive: one new public type, one new method on each of two
existing types, no signature or behavior change to anything that
exists. Minor bump. release-plz owns the version and `CHANGELOG.md`
(AGENTS.md gotcha #9) — neither is touched here. `cargo semver-checks`
cannot pass on a feature branch, as AGENTS.md notes, because
`Cargo.toml` still carries the published version.

---

## Rejected alternatives

**`#[non_exhaustive]` on `AlertSnapshot`.** Considered on the theory
that issue #60 wanted `M` exposed here and a `mode` field was coming.
Checked: **#60 is closed**, completed 2026-09-25 by PR #71 and its
follow-on. Its completion-readback need was met internally by
`ops::raw_mode` (623), `ops::classify_poll` (644) and `ops::PollOutcome`
(630); nothing about it wanted `M` on a public type. There is no known
pending field. Against adding it: `AlertEvent` and `OneShotError` are
both exhaustive, and `tests/reexports.rs:166-171` pins the absence of
`#[non_exhaustive]` on `OneShotError` as a deliberate contract that
"nothing inside the crate can pin for us". Matching house style wins
over pre-paying for a field nobody has asked for.

**Adding `mode: Mode` to `AlertSnapshot`.** `AlertSnapshot`'s own doc
(495–497) deliberately excludes a decoded `Mode`, and `ops::raw_mode`'s
comment (619–622) explains why the decode is lossy: `Mode` has no
inhabitant for raw `0b11`, so `0b10` and `0b11` become
indistinguishable. A faithful field would have to be a raw `u8`, which
is a different design discussion.

**Returning `AlertCause` or `Option<AlertCause>`.** Covered in B2. Both
require inventing a meaning for the both-flags-clear case that the
existing type contract forbids.

**A full self-contained section at all thirteen sites.** Most
consistent with the file's dominant idiom, but adds roughly 150 lines
of near-duplicate prose across twin methods that must then be kept in
sync by hand. The crate already has one instance of this duplication
(1899 / 2410) and it is exactly the thing that drifts.

**Per-method sections only where the effect is surprising.** Tempting
for `read_configuration`, where a configuration read is arguably
obvious. Rejected because `read_configuration` is the single method a
caller is most likely to reach for *as a status query*, which is the
misconception at the center of this issue.

---

## Verification

**Unit tests.** Coverage of `decode_alert_snapshot` already exists and
is strong: `mod tests::ops_tests::alert_snapshot` (`src/lib.rs:3339`)
walks all 65,536 `[u8; 2]` patterns asserting the decoder is total and
that its settings projection agrees with `decode_config`, plus a
four-case independence check on the flag pair. It needs no new
assertions — it needs its `#[cfg(feature =
"embedded-sensors-hal-async")]` gate at 3338 removed, so the coverage
runs on the default build where the newly-ungated function now lives.
That gate removal is the test-side half of B1 and is easy to forget.

New: `mod tests::blocking` (4421) and `mod tests::asynchronous` (4967)
each gain a mock-transaction test that the new method issues exactly
one `write_read(0x48, [0x01], …)` and surfaces flags the corresponding
`read_configuration` call drops.

**Doctests.** Both shapes per AGENTS.md — blocking plain, async wrapped
in `tokio::runtime::Runtime::new().unwrap().block_on(async { … })`.
Byte values come from the measured register contents above: `[0x14,
0x10]` is a real acknowledging read with FH set, `[0x04, 0x10]` the
same register after acknowledgement.

**`tests/reexports.rs`.** Pins `AlertSnapshot` as reachable,
constructible and destructurable from outside the crate with all three
fields nameable, its derive set, and exhaustive-match/construction
without `..` (pinning the deliberate absence of `#[non_exhaustive]`, as
that file already does for `OneShotError`). Pins both method signatures.
The `AlertSnapshot` pin is **ungated**, which is what would catch a
regression that re-introduces the feature gate.

**Local matrix.** The full AGENTS.md list: nightly `cargo fmt --check`,
pedantic clippy, `cargo doc` (every per-method section added by A2–A5
links into the crate-level section, and rustdoc is what checks those
intra-doc links resolve), `cargo test` and `cargo test --doc` across the
three feature combinations, `cargo build --examples` across four, `cargo
hack --feature-powerset check`, `scripts/check-readme-snippets.sh`, and
`cargo vet`. No new dependencies, so no new audit entries.

**Feature-powerset is load-bearing here.** Ungating a type and a pure
function is exactly the change that compiles under
`--all-features` and fails on the default build. The `AlertCause` import
in `mod ops` (225–226) is currently `#[cfg(feature =
"embedded-sensors-hal-async")]`, and gates are being selectively
removed at 498, 510 and 3338 while the one at 529 stays; the powerset
run is what proves the result. Note the four gates are not
interchangeable — removing 529 as well would drag `AlertCause` into the
default build's `ops` import and is not part of this change.

**Hardware.** The change adds a flags-returning read, so AGENTS.md's
mandatory-verification rule for alert-pin behavior applies. Re-run the
experiment above *through the driver* on the attached rig: drive FH,
park in shutdown, call `read_configuration_and_acknowledge()`, assert
`high == true`, observe GPIO0 release, call again, assert `high ==
false`. Then the example matrix, including the two alert examples that
need human warming.

---

## Commits

Conventional Commits v1.0.0, each building clean and passing clippy and
fmt independently (AGENTS.md gotcha #6). Commit subjects are the
changelog (gotcha #9), so they are written for users.

1. `feat: add read_configuration_and_acknowledge for FL/FH-preserving reads`
   — Part B in full: type promotion and ungating, the two methods,
   `AlertTmp108::read_alert_snapshot` delegation, unit tests, doctests,
   reexport pins.
2. `docs: document the interrupt-mode acknowledgement effect of configuration reads`
   — A1 through A5.
3. `docs(readme): note that any configuration read acknowledges an alert`
   — A6, separate because `scripts/check-readme-snippets.sh` gates the
   README and the bullet is independently revertible.

Each carries an `Assisted-by:` trailer. No `Signed-off-by:`.

No PR will be opened; the branch is handed back for review.

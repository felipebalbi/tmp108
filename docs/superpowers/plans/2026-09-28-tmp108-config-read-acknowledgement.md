# Configuration-read acknowledgement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Document that reading the TMP108 configuration register acknowledges an interrupt-mode alert on every entry point that does so, and add `read_configuration_and_acknowledge` so the FL/FH flags such a read consumes survive to the caller.

**Architecture:** Two halves. Part B promotes the existing `pub(crate) ops::AlertSnapshot` to ungated public surface and adds one thin method per driver flavor that decodes more of the same single register read. Part A adds one canonical `## Interrupt-mode acknowledgement` section to the crate docs and short per-method sections linking to it. No existing method changes its I²C traffic.

**Tech Stack:** Rust 1.98.1, `no_std`, edition 2024, `embedded-hal` 1.0 / `embedded-hal-async` 1.0, `device-driver` 2.1.0 generated registers, `embedded_hal_mock` for unit tests and doctests.

Spec: [`docs/superpowers/specs/2026-09-28-tmp108-config-read-acknowledgement-design.md`](../specs/2026-09-28-tmp108-config-read-acknowledgement-design.md)

---

## Baseline — measured, not assumed

Recorded on branch `config-read-acknowledgement` at spec commit `1c22935f6011`. **Every gate below must still produce exactly this after each commit.** "No new warnings" is meaningless without it.

| Gate | Command | Baseline result |
|---|---|---|
| Build | `cargo build --all-features --all-targets --locked` | 0 warnings, 0 errors |
| Clippy | `cargo clippy --all-features --all-targets --locked -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style` | exit 0, 0 warnings |
| Format | `cargo +nightly fmt --check` | exit 0 |
| Docs (every feature combination) | `cargo doc --no-deps --locked` ×7, see gate | exit 0, **0 warnings in all seven** |
| Tests (default) | `cargo test --locked` | 53 unit + 5 reexports + 22 doc pass, 3 ignored |
| Tests (all) | `cargo test --locked -F async,embedded-sensors-hal-async` | 164 unit + 6 reexports + 53 doc pass, 3 ignored |
| Doctests | `cargo test --doc --locked -F async,embedded-sensors-hal-async` | 53 pass, 3 ignored |
| Powerset | `cargo hack --feature-powerset check --locked` | 6/6 combinations clean |
| README | `bash ./scripts/check-readme-snippets.sh` | "README snippets match (3 checked)." |

Test counts **will rise** as tasks add tests. That is expected. What must not change: zero warnings, zero failures, and the ignored count staying at 3.

### The lint configuration is stricter than AGENTS.md's command line

`Cargo.toml:42-51` denies crate-wide, via the lints table, so these apply to *every* `cargo build` and `cargo clippy` regardless of flags:

```toml
[lints.rust]
unsafe_code = "deny"
missing_docs = "deny"

[lints.clippy]
correctness = "deny"
suspicious  = "deny"
perf        = "deny"
style       = "deny"
pedantic    = "deny"
```

`pedantic = "deny"` is the one that will bite. Three specific lints:

- **`missing_docs`** — a public struct needs a doc comment *and so does every public field*. Three fields, three doc comments.
- **`clippy::missing_errors_doc`** (pedantic) — every public function returning `Result` needs an `# Errors` section. Both new methods.
- **`clippy::doc_markdown`** (pedantic) — identifiers in prose must be backticked. This change adds ~15 prose sections. Write `` `read_configuration_and_acknowledge` ``, `` `AlertTmp108` ``, `` `wait_for_temperature` `` — never bare. Bare `ALERT`, `FL`, `FH` are fine (not identifier-shaped); `AlertTmp108` is not.

---

## The verification gate

Run this **after every commit**, not just at the end. It is referenced below as "**run the gate**".

```bash
cargo +nightly fmt --check
cargo clippy --all-features --all-targets --locked -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
cargo doc --no-deps --locked
cargo doc --no-deps --locked -F async
cargo doc --no-deps --locked -F embedded-sensors-hal
cargo doc --no-deps --locked -F embedded-sensors-hal-async
cargo doc --no-deps --locked -F async,embedded-sensors-hal
cargo doc --no-deps --locked -F async,embedded-sensors-hal-async
cargo doc --no-deps --all-features --locked
cargo test --locked
cargo test --locked -F async
cargo test --locked -F async,embedded-sensors-hal-async
cargo test --doc --locked
cargo test --doc --locked -F async
cargo test --doc --locked -F async,embedded-sensors-hal-async
cargo build --examples --locked
cargo build --examples --locked -F async
cargo build --examples --locked -F embedded-sensors-hal
cargo build --examples --locked -F async,embedded-sensors-hal-async
cargo hack --feature-powerset check --locked
bash ./scripts/check-readme-snippets.sh
```

Expected from all twenty-one: exit 0, no `warning:` lines, no failures.

If any command fails, **fix it before the next commit** by amending the commit under test (`git commit --amend`), not by adding a follow-up fix commit. AGENTS.md gotcha #6: each commit must be clean independently, so history stays bisectable.

### Why the doc build is run under seven feature combinations

Because two bookend builds cannot see a whole class of broken link.

An intra-doc link breaks when the documented item exists but the link target does not. A link on an `async`-gated item pointing at an `embedded-sensors-hal-async`-gated item is invisible to **both** ends of the matrix: under default features the enclosing item does not exist so nothing is documented, and under `--all-features` the target does exist so the link resolves. It warns only in the middle.

That is not hypothetical — `src/lib.rs:732` was exactly this, and was missed until the intermediate combinations were checked. The crate had accumulated ten such warnings because CI's doc job only builds one configuration.

This matters most for **Task 2**, which adds a crate-level section full of intra-doc links to items across several feature gates. The current baseline is **0 warnings in every combination**; anything above 0 anywhere is a regression introduced by that task.

`cargo hack --feature-powerset check` does not help here — it runs `check`, not `doc`, so it never evaluates a link.

### Why the powerset step is not optional here

Task 1 removes `#[cfg(feature = "embedded-sensors-hal-async")]` from three places. That is precisely the class of change that compiles under `--all-features` and breaks the default build. `cargo hack --feature-powerset check` is the only gate that exercises all six combinations.

### Why Task 1 cannot be split

A tempting decomposition is "first ungate the codec, then add the method". **Do not do this.** Ungating `AlertSnapshot` and `decode_alert_snapshot` while their only caller (`AlertTmp108::read_alert_snapshot`) is still feature-gated leaves both items unused in the default build, which produces `dead_code` warnings and fails the gate. The ungating and the first ungated caller must land in the same commit.

---

## File structure

| File | Change | Responsibility |
|---|---|---|
| `src/lib.rs` | Modify | Everything. Single-file crate by design; see AGENTS.md "Where to put new things". |
| `src/lib.rs:63` | Modify | Crate-root re-export list gains `AlertSnapshot` |
| `src/lib.rs:485-517` | Modify | `ops::AlertSnapshot` + `ops::decode_alert_snapshot` ungated and made public |
| `src/lib.rs:1795` | Insert after | Blocking `Tmp108::read_configuration_and_acknowledge` |
| `src/lib.rs:2299` | Insert after | Async `AsyncTmp108::read_configuration_and_acknowledge` |
| `src/lib.rs:1718` | Modify | `AlertTmp108::read_alert_snapshot` delegates |
| `src/lib.rs:3338` | Modify | Remove the test module's feature gate |
| `src/lib.rs:11-38` | Modify | Crate-level `## Interrupt-mode acknowledgement` section |
| `tests/reexports.rs` | Modify | Pin the new type and both method signatures |
| `README.md:115-190` | Modify | One new `## Gotchas` bullet |

`src/inner.rs` is generated and is **not** touched. `tmp108.ddsl` is not touched — no register-layout change. `Cargo.toml` and `CHANGELOG.md` are **not** touched (AGENTS.md gotcha #9: release-plz owns version and changelog).

---

## Task 1: `read_configuration_and_acknowledge`

Implements spec Part B in full. One commit.

**Files:**
- Modify: `src/lib.rs:63` (re-export), `485-517` (type + decoder), `1718` (delegation), `1795` (blocking method), `2299` (async method), `3338` (test gate)
- Test: `src/lib.rs` `mod tests::blocking`, `mod tests::asynchronous`; `tests/reexports.rs`

---

- [ ] **Step 1: Write the failing reexport test**

Append to `tests/reexports.rs`:

```rust
/// `AlertSnapshot` is ungated public surface: it must be reachable and
/// destructurable with no optional features enabled. The gate-free
/// `#[test]` is the point — it is what catches a regression that
/// re-introduces the `embedded-sensors-hal-async` gate the type
/// carried while it was `pub(crate)`.
///
/// Constructing it by literal, with no `..`, pins the deliberate
/// absence of `#[non_exhaustive]`, exactly as
/// `one_shot_error_is_reachable_and_exhaustive` does for `OneShotError`.
// Pinning the `Clone` impl is the point of the assertion, so the
// redundant clone on a `Copy` type is deliberate.
#[allow(clippy::clone_on_copy)]
#[test]
fn alert_snapshot_is_reachable_and_exhaustive() {
    use tmp108::{AlertSnapshot, Config};

    fn eq_bound<T: Eq>(_: T) {}

    let snapshot = AlertSnapshot {
        config: Config::default(),
        low: false,
        high: true,
    };

    let AlertSnapshot { config, low, high } = snapshot;
    assert_eq!(config, Config::default());
    assert!(!low);
    assert!(high);

    // Clone, Copy, Debug, PartialEq, Eq, Hash.
    let copied = snapshot;
    assert_eq!(copied, snapshot.clone());
    assert_ne!(
        snapshot,
        AlertSnapshot {
            config: Config::default(),
            low: true,
            high: true,
        }
    );
    assert_ne!(format!("{snapshot:?}"), "");
    let mut set = std::collections::HashSet::new();
    assert!(set.insert(snapshot));
    assert!(!set.insert(copied));
    eq_bound(copied);
}

/// `Tmp108::read_configuration_and_acknowledge` is public surface, and
/// returns the snapshot type rather than `Config`.
#[allow(dead_code)]
fn pin_blocking_read_configuration_and_acknowledge() {
    fn _type_check<I2C: embedded_hal::i2c::I2c>(
        tmp: &mut tmp108::Tmp108<I2C>,
    ) -> Result<tmp108::AlertSnapshot, I2C::Error> {
        tmp.read_configuration_and_acknowledge()
    }
}

/// The async twin carries the same signature over the async I2C trait.
#[cfg(feature = "async")]
#[allow(dead_code)]
fn pin_async_read_configuration_and_acknowledge() {
    async fn _type_check<I2C: embedded_hal_async::i2c::I2c>(
        tmp: &mut tmp108::AsyncTmp108<I2C>,
    ) -> Result<tmp108::AlertSnapshot, I2C::Error> {
        tmp.read_configuration_and_acknowledge().await
    }
}
```

- [ ] **Step 2: Run it to make sure it fails**

Run: `cargo test --locked --test reexports`

Expected: FAIL to compile, `unresolved import tmp108::AlertSnapshot` / `no method named read_configuration_and_acknowledge`.

If it *compiles*, stop — something is already present and this plan's baseline is wrong.

- [ ] **Step 3: Promote and ungate `AlertSnapshot`**

In `src/lib.rs`, replace lines 485-517 (the `AlertSnapshot` doc comment, struct, and `decode_alert_snapshot`) with:

```rust
    /// One configuration-register read, decoded into the settings it
    /// carries *and* the ALERT status flags it simultaneously consumed.
    ///
    /// Reading the configuration register is destructive: per TMP108
    /// datasheet SBOS663A §7.5.3.4, it clears both FL/FH and the ALERT
    /// pin. `low` and `high` therefore describe the snapshot that was
    /// returned, not the chip's state once the transaction completed.
    /// Whoever holds this value holds the only remaining evidence of a
    /// latched interrupt.
    ///
    /// The two flags are independent; all four combinations occur. The
    /// type deliberately carries no temperature, timestamp, event
    /// count, or decoded `Mode`.
    ///
    /// Produced by
    /// [`read_configuration_and_acknowledge`][crate::Tmp108::read_configuration_and_acknowledge]
    /// and its async twin. See the crate-level "Interrupt-mode
    /// acknowledgement" section for which other calls consume these
    /// flags without returning them.
    ///
    /// # Examples
    ///
    /// ```
    /// use tmp108::{AlertSnapshot, Config};
    ///
    /// let snapshot = AlertSnapshot {
    ///     config: Config::default(),
    ///     low: false,
    ///     high: true,
    /// };
    /// // The flags are historical evidence, not a live comparison.
    /// assert!(snapshot.high);
    /// ```
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    pub struct AlertSnapshot {
        /// Settings carried by the configuration read that produced this.
        pub config: Config,
        /// FL as that read returned it: a low-limit excursion was latched.
        ///
        /// Meaningful in [`Thermostat::Interrupt`][crate::Thermostat::Interrupt]
        /// mode, where it records an excursion since the flags were last
        /// observed and cleared.
        pub low: bool,
        /// FH as that read returned it: a high-limit excursion was latched.
        ///
        /// Meaningful in [`Thermostat::Interrupt`][crate::Thermostat::Interrupt]
        /// mode, where it records an excursion since the flags were last
        /// observed and cleared.
        pub high: bool,
    }

    /// Decode a configuration-register snapshot into its settings and
    /// the ALERT status flags it returned.
    ///
    /// Pure, total, and allocation-free.
    pub(crate) fn decode_alert_snapshot(c: Configuration) -> AlertSnapshot {
        AlertSnapshot {
            config: decode_config(c),
            low: c.fl(),
            high: c.fh(),
        }
    }
```

Both `#[cfg(feature = "embedded-sensors-hal-async")]` attributes (previously at 498 and 510) are gone. `ops::interrupt_alert_cause` and its gate at 529 are **unchanged** — leaving that gate is what keeps the `AlertCause` import at 225-226 correctly gated.

- [ ] **Step 4: Re-export at the crate root**

In `src/lib.rs`, change line 63 from:

```rust
pub use crate::ops::{Celsius, OutOfRange};
```

to:

```rust
pub use crate::ops::{AlertSnapshot, Celsius, OutOfRange};
```

- [ ] **Step 5: Add the blocking method**

In `src/lib.rs`, insert immediately after `read_configuration`'s closing brace (currently line 1795, before the `/// Configure device parameters.` at 1797):

```rust

    /// Read the configuration register, keeping the ALERT status flags
    /// that read consumed.
    ///
    /// Same single I²C transaction as
    /// [`read_configuration`][Self::read_configuration] — the
    /// difference is how much of the result survives decoding.
    /// `read_configuration` returns a [`Config`], which models the 64
    /// configurable settings and drops FL, FH and M. This returns an
    /// [`AlertSnapshot`], which keeps FL and FH.
    ///
    /// # This call acknowledges
    ///
    /// In [`Thermostat::Interrupt`] mode the read clears FL and FH and
    /// releases the ALERT pin. That is true of every configuration
    /// read; see the crate-level "Interrupt-mode acknowledgement"
    /// section. What is specific to this method is that it is the only
    /// one that hands the consumed flags back, so the returned value is
    /// the sole surviving evidence of the alert.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails. On failure the
    /// flags may or may not have been consumed — the error does not
    /// distinguish a transaction that never reached the chip from one
    /// whose response was lost.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{ConversionRate, Hysteresis, Polarity, Thermostat, Tmp108};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x14, 0x10]),
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x04, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    ///
    /// // A latched high-limit excursion, in interrupt mode.
    /// let first = tmp.read_configuration_and_acknowledge().unwrap();
    /// assert!(first.high);
    /// assert!(!first.low);
    /// assert_eq!(first.config.thermostat_mode, Thermostat::Interrupt);
    /// assert_eq!(first.config.alert_polarity, Polarity::ActiveLow);
    /// assert_eq!(first.config.conversion_rate, ConversionRate::QuarterHz);
    /// assert_eq!(first.config.hysteresis, Hysteresis::OneC);
    ///
    /// // The first read consumed it. The second sees nothing, even
    /// // though the temperature never changed.
    /// let second = tmp.read_configuration_and_acknowledge().unwrap();
    /// assert!(!second.high);
    /// assert_eq!(second.config, first.config);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn read_configuration_and_acknowledge(&mut self) -> Result<AlertSnapshot, I2C::Error> {
        let c = self.inner.configuration().read()?;
        Ok(ops::decode_alert_snapshot(c))
    }
```

The two register values are real: `[0x14, 0x10]` and `[0x04, 0x10]` were read back-to-back off a TMP108 at `0x48` while the over-temperature condition held constant. See the spec's "Evidence" section.

- [ ] **Step 6: Add the async method**

In `src/lib.rs`, insert immediately after the async `read_configuration`'s closing brace (currently line 2299, before `/// Configure device parameters.` at 2301). Identical prose, async body, async doctest shape:

```rust

    /// Read the configuration register, keeping the ALERT status flags
    /// that read consumed.
    ///
    /// Same single I²C transaction as
    /// [`read_configuration`][Self::read_configuration] — the
    /// difference is how much of the result survives decoding.
    /// `read_configuration` returns a [`Config`], which models the 64
    /// configurable settings and drops FL, FH and M. This returns an
    /// [`AlertSnapshot`], which keeps FL and FH.
    ///
    /// # This call acknowledges
    ///
    /// In [`Thermostat::Interrupt`] mode the read clears FL and FH and
    /// releases the ALERT pin. That is true of every configuration
    /// read; see the crate-level "Interrupt-mode acknowledgement"
    /// section. What is specific to this method is that it is the only
    /// one that hands the consumed flags back, so the returned value is
    /// the sole surviving evidence of the alert.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails. On failure the
    /// flags may or may not have been consumed — the error does not
    /// distinguish a transaction that never reached the chip from one
    /// whose response was lost.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{AsyncTmp108, ConversionRate, Hysteresis, Polarity, Thermostat};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x14, 0x10]),
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x04, 0x10]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    ///
    /// // A latched high-limit excursion, in interrupt mode.
    /// let first = tmp.read_configuration_and_acknowledge().await.unwrap();
    /// assert!(first.high);
    /// assert!(!first.low);
    /// assert_eq!(first.config.thermostat_mode, Thermostat::Interrupt);
    /// assert_eq!(first.config.alert_polarity, Polarity::ActiveLow);
    /// assert_eq!(first.config.conversion_rate, ConversionRate::QuarterHz);
    /// assert_eq!(first.config.hysteresis, Hysteresis::OneC);
    ///
    /// // The first read consumed it. The second sees nothing, even
    /// // though the temperature never changed.
    /// let second = tmp.read_configuration_and_acknowledge().await.unwrap();
    /// assert!(!second.high);
    /// assert_eq!(second.config, first.config);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn read_configuration_and_acknowledge(&mut self) -> Result<AlertSnapshot, I2C::Error> {
        let c = self.inner.configuration().read_async().await?;
        Ok(ops::decode_alert_snapshot(c))
    }
```

- [ ] **Step 7: Collapse the duplicate implementation**

In `src/lib.rs`, replace the body of `AlertTmp108::read_alert_snapshot` (currently line 1718-1721). Keep its doc comment at 1701-1717 exactly as it is — it documents the waiter protocol's use of the read, which is `AlertTmp108`-specific.

```rust
    async fn read_alert_snapshot(&mut self) -> Result<ops::AlertSnapshot, I2C::Error> {
        self.sensor_mut().read_configuration_and_acknowledge().await
    }
```

Note the return type still names `ops::AlertSnapshot`. That is the same type as the re-exported `crate::AlertSnapshot`; leave it as-is to keep the diff minimal.

- [ ] **Step 8: Ungate the existing exhaustive test module**

In `src/lib.rs`, delete line 3338:

```rust
        #[cfg(feature = "embedded-sensors-hal-async")]
```

so that `mod alert_snapshot` at 3339 runs on the default build. Its two tests need **no** changes — they already walk all 65,536 bit patterns and check flag independence.

- [ ] **Step 9: Add the driver-level unit tests**

In `src/lib.rs`, append inside `mod tests::blocking` (module opens at 4421 — add at the end of the module, before its closing brace):

```rust
        /// Issue #65: the acknowledging read returns the flags that
        /// `read_configuration` drops, over identical bus traffic.
        #[test]
        fn read_configuration_and_acknowledge_surfaces_flags() {
            let i2c = Mock::new(&[
                Transaction::write_read(0x48, vec![0x01], vec![0x14, 0x10]),
                Transaction::write_read(0x48, vec![0x01], vec![0x14, 0x10]),
            ]);
            let mut tmp = Tmp108::new_with_a0_gnd(i2c);

            let snapshot = tmp.read_configuration_and_acknowledge().unwrap();
            assert!(snapshot.high, "FH was set in the register value");
            assert!(!snapshot.low, "FL was clear in the register value");

            // Same bytes through the settings-only decoder: the flags
            // are gone, and the settings agree.
            let config = tmp.read_configuration().unwrap();
            assert_eq!(config, snapshot.config);

            let mut i2c = tmp.destroy();
            i2c.done();
        }

        /// All four flag combinations reach the caller intact.
        #[test]
        fn read_configuration_and_acknowledge_reports_every_flag_pair() {
            for (byte0, low, high) in [
                (0x04_u8, false, false),
                (0x0c_u8, true, false),
                (0x14_u8, false, true),
                (0x1c_u8, true, true),
            ] {
                let i2c = Mock::new(&[Transaction::write_read(0x48, vec![0x01], vec![byte0, 0x10])]);
                let mut tmp = Tmp108::new_with_a0_gnd(i2c);

                let snapshot = tmp.read_configuration_and_acknowledge().unwrap();
                assert_eq!((snapshot.low, snapshot.high), (low, high), "byte0 {byte0:#04x}");

                let mut i2c = tmp.destroy();
                i2c.done();
            }
        }
```

And the async twins inside `mod tests::asynchronous` (module opens at 4967 — add at the end, before its closing brace):

```rust
        /// Issue #65: the acknowledging read returns the flags that
        /// `read_configuration` drops, over identical bus traffic.
        #[tokio::test]
        async fn read_configuration_and_acknowledge_surfaces_flags() {
            let i2c = Mock::new(&[
                Transaction::write_read(0x48, vec![0x01], vec![0x14, 0x10]),
                Transaction::write_read(0x48, vec![0x01], vec![0x14, 0x10]),
            ]);
            let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);

            let snapshot = tmp.read_configuration_and_acknowledge().await.unwrap();
            assert!(snapshot.high, "FH was set in the register value");
            assert!(!snapshot.low, "FL was clear in the register value");

            let config = tmp.read_configuration().await.unwrap();
            assert_eq!(config, snapshot.config);

            let mut i2c = tmp.destroy();
            i2c.done();
        }

        /// All four flag combinations reach the caller intact.
        #[tokio::test]
        async fn read_configuration_and_acknowledge_reports_every_flag_pair() {
            for (byte0, low, high) in [
                (0x04_u8, false, false),
                (0x0c_u8, true, false),
                (0x14_u8, false, true),
                (0x1c_u8, true, true),
            ] {
                let i2c = Mock::new(&[Transaction::write_read(0x48, vec![0x01], vec![byte0, 0x10])]);
                let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);

                let snapshot = tmp.read_configuration_and_acknowledge().await.unwrap();
                assert_eq!((snapshot.low, snapshot.high), (low, high), "byte0 {byte0:#04x}");

                let mut i2c = tmp.destroy();
                i2c.done();
            }
        }
```

**Before writing these, note the local convention**: this file does **not** define an `ADDR` constant — all 222 mock transactions spell the address as the literal `0x48`. The snippets above use `0x48` accordingly. `Mock` and `Transaction` are already imported at the top of `mod blocking` (`src/lib.rs:4423`); confirm the same holds for `mod asynchronous` and that its async tests use `#[tokio::test]`, matching the neighbours.

- [ ] **Step 10: Run the tests**

Run: `cargo test --locked --test reexports`
Expected: PASS, including `alert_snapshot_is_reachable_and_exhaustive`.

Run: `cargo test --locked`
Expected: PASS. Unit count rises from 53; the previously-gated exhaustive `alert_snapshot` module now runs on the default build, so expect a jump of more than the two tests just added.

Run: `cargo test --locked -F async,embedded-sensors-hal-async`
Expected: PASS, unit count rises from 164.

- [ ] **Step 11: Run the gate**

Run every command in "The verification gate" above.

Expected: all twenty-one exit 0 with no `warning:` lines.

Pay attention to `cargo hack --feature-powerset check --locked`. If it fails on a combination that does not include `embedded-sensors-hal-async`, a `#[cfg]` removal in Step 3 was incomplete or `interrupt_alert_cause`'s gate at 529 was removed by mistake.

- [ ] **Step 12: Commit**

On Windows/PowerShell there is no heredoc, so write the message to a file first. Save this as `msg.txt` outside the repo (for example `$env:TEMP\msg.txt`):

```
feat: add read_configuration_and_acknowledge for FL/FH-preserving reads

Reading the configuration register clears FL, FH and the ALERT pin in
interrupt mode, but read_configuration decodes only the 64 configurable
settings and drops the flags it just consumed. Callers had no way to
recover an alert their own status query had destroyed.

read_configuration_and_acknowledge issues the same single register read
and returns an AlertSnapshot carrying the flags alongside the settings.
AlertSnapshot and its decoder were already present but private and
gated on embedded-sensors-hal-async; both are now public and ungated,
because the flags exist regardless of features and the blocking driver
has no alert waiter at all.

Assisted-by: opencode:claude-opus-5
```

Then:

```bash
git add src/lib.rs tests/reexports.rs
git commit -F $env:TEMP\msg.txt
git log --oneline -1
```

Expected: one new commit. Verify the trailer survived with `git log -1 --format=%B | Select-String Assisted-by` — it must be present, and there must be **no** `Signed-off-by` (AGENTS.md: only humans certify the DCO).

---

## Task 2: Document the acknowledgement effect

Implements spec Part A, sections A1 through A5. One commit. Pure documentation plus one doctest change; no behavior change.

**Files:**
- Modify: `src/lib.rs:11-38` (A1), and the method sites listed below (A2-A5)

---

- [ ] **Step 1: Add the canonical crate-level section (A1)**

In `src/lib.rs`, add a new subsection under `# Operational notes`, after `## Driver lifecycle on drop` (which ends at line 38) and before `#![doc(html_root_url = ...)]` at line 40:

```rust
//!
//! ## Interrupt-mode acknowledgement
//!
//! In [`Thermostat::Interrupt`] mode, **reading the configuration
//! register is acknowledging an alert**. The read clears the FL and FH
//! flags and releases the ALERT pin (TMP108 datasheet SBOS663A
//! §7.5.3.4). A configuration *write* does not: flags latched before a
//! write survive it and are still present on the next read.
//!
//! This matters because the driver reads that register far more often
//! than callers expect, including from methods whose names suggest
//! nothing of the sort:
//!
//! | Call | Configuration reads | Why it reads |
//! |---|---|---|
//! | [`probe`][Tmp108::probe] | 1 | compares against the power-on reset value |
//! | [`read_configuration`][Tmp108::read_configuration] | 1 | returns the settings |
//! | [`read_configuration_and_acknowledge`][Tmp108::read_configuration_and_acknowledge] | 1 | returns the settings **and the flags** |
//! | [`configure`][Tmp108::configure] | 1 | read-modify-write |
//! | [`shutdown`][Tmp108::shutdown] | 1 | read-modify-write |
//! | [`wait_for_temperature`][Tmp108::wait_for_temperature] | 1 | to pick a delay, nothing more |
//! | [`one_shot`][Tmp108::one_shot] | ~10 | preparation, trigger, completion polling |
//! | `AsyncTmp108::continuous` | 2 | entry read-modify-write, then cleanup |
//! | `set_temperature_threshold_hysteresis` | 2 | a read followed by a read-modify-write |
//!
//! The async driver mirrors every row. Only
//! `read_configuration_and_acknowledge` returns the flags it consumed;
//! every other row discards them, because [`Config`] models the 64
//! configurable settings and has no home for FL, FH or M.
//!
//! Three consequences worth stating plainly:
//!
//! - A call that fails or is cancelled *after* its configuration read
//!   has already acknowledged. No error variant reports this, and
//!   nothing undoes it.
//! - In [`Thermostat::Comparator`] mode none of this applies. The pin
//!   tracks the temperature against the limits and hysteresis band, and
//!   a configuration read does not release it.
//! - If you rely on latched interrupt evidence, collect it with
//!   [`read_configuration_and_acknowledge`][Tmp108::read_configuration_and_acknowledge]
//!   before calling anything else in the table.
```

Watch `clippy::doc_markdown`: every identifier above is either an intra-doc link or backticked. Bare `ALERT`, `FL`, `FH`, `SBOS663A` are not identifier-shaped and are fine.

- [ ] **Step 2: Verify the links resolve**

Run: `cargo doc --no-deps --all-features --locked`

Expected: exit 0, no warnings. Broken intra-doc links surface here and nowhere else.

Note `AsyncTmp108::continuous` and `set_temperature_threshold_hysteresis` are written as plain code spans, not links — they are feature-gated, and an intra-doc link to a gated item warns on builds where the feature is off.

Run: `cargo doc --no-deps --locked`

Expected: exit 0, no warnings. This is the build where gated links would break.

- [ ] **Step 3: Add the eleven per-method sections (A2)**

Each is the same shape. Insert immediately before the `# Errors` section of each method's rustdoc, so the order stays prose → custom sections → `# Errors` → `# Examples`, matching the file's convention.

For `Tmp108::probe` (1767) and `AsyncTmp108::probe` (2269):

```rust
    /// # Interrupt-mode acknowledgement
    ///
    /// This performs one configuration read, so in
    /// [`Thermostat::Interrupt`] mode it clears FL/FH and releases the
    /// ALERT pin. A liveness check is not free: calling it with an
    /// alert pending destroys the evidence. See the crate-level
    /// "Interrupt-mode acknowledgement" section.
```

For `Tmp108::read_configuration` (1792) and `AsyncTmp108::read_configuration` (2296):

```rust
    /// Returns the *configurable parameters*, not complete hardware
    /// state: FL, FH and M are present in the register this reads and
    /// are discarded by the decode.
    ///
    /// # Interrupt-mode acknowledgement
    ///
    /// This performs one configuration read, so in
    /// [`Thermostat::Interrupt`] mode it clears FL/FH and releases the
    /// ALERT pin — and returns nothing about them. Use
    /// [`read_configuration_and_acknowledge`][Self::read_configuration_and_acknowledge]
    /// to keep the flags. See the crate-level "Interrupt-mode
    /// acknowledgement" section.
```

The first paragraph goes at the *top* of the doc, right after the existing `/// Read configuration register` summary line.

For `Tmp108::configure` (1845) and `AsyncTmp108::configure` (2351):

```rust
    /// # Interrupt-mode acknowledgement
    ///
    /// The read half of the read-modify-write acknowledges: in
    /// [`Thermostat::Interrupt`] mode it clears FL/FH and releases the
    /// ALERT pin. The write-back preserves the sampled flag bits, but
    /// that is not the same as preserving the alert — the chip cleared
    /// it when the read landed. See the crate-level "Interrupt-mode
    /// acknowledgement" section.
```

This one matters. The existing text at 1799-1804 says the flags "are read back and preserved", which is true of the write-back and easy to misread as meaning the alert survives.

For `Tmp108::shutdown` (2056) and `AsyncTmp108::shutdown` (2585):

```rust
    /// # Interrupt-mode acknowledgement
    ///
    /// This is a read-modify-write, and the read half acknowledges: in
    /// [`Thermostat::Interrupt`] mode it clears FL/FH and releases the
    /// ALERT pin. See the crate-level "Interrupt-mode acknowledgement"
    /// section.
```

For `Tmp108::wait_for_temperature` (2130):

```rust
    /// # Interrupt-mode acknowledgement
    ///
    /// The configuration read named above exists only to discover the
    /// conversion rate, but in [`Thermostat::Interrupt`] mode it still
    /// clears FL/FH and releases the ALERT pin. Picking a delay costs
    /// you a pending alert. See the crate-level "Interrupt-mode
    /// acknowledgement" section.
```

The async `wait_for_temperature` (2695) defers to the blocking one and keeps deferring — add nothing there.

For `AsyncTmp108::continuous` (2645):

```rust
    /// # Interrupt-mode acknowledgement
    ///
    /// Two configuration reads per call: the entry read-modify-write
    /// that selects continuous mode, and the cleanup shutdown on the
    /// way out. In [`Thermostat::Interrupt`] mode each clears FL/FH and
    /// releases the ALERT pin, and the cleanup read happens even when
    /// the closure returned an error. See the crate-level
    /// "Interrupt-mode acknowledgement" section.
```

- [ ] **Step 4: Trim the two `one_shot` blocks (A3)**

Replace the twelve-line `# This operation is interrupt-destructive` section in `Tmp108::one_shot` (1899-1910) and the byte-identical one in `AsyncTmp108::one_shot` (2410-2421) with:

```rust
    /// # Interrupt-mode acknowledgement
    ///
    /// The sequence performs roughly ten configuration reads, so in
    /// [`Thermostat::Interrupt`] mode it is thoroughly destructive:
    /// each read clears FL/FH and releases the ALERT pin, and the loss
    /// is not reported anywhere in the return value. This is a
    /// sample-only helper — collect any latched interrupt evidence
    /// first, or do not use this method. See the crate-level
    /// "Interrupt-mode acknowledgement" section.
```

Keep both copies identical to each other, as the originals were.

- [ ] **Step 5: Document the two trait paths (A4)**

`TemperatureHysteresis::set_temperature_threshold_hysteresis` for `AsyncTmp108` (3190) currently has no rustdoc at all — only inline `//` comments at 3194-3198, which stay. Add above the method:

```rust
    /// Snap a requested hysteresis to one of the chip's four settings
    /// and write it.
    ///
    /// # Interrupt-mode acknowledgement
    ///
    /// **Two** configuration reads per call: one to read the current
    /// settings, and one inside the read-modify-write that writes them
    /// back. In [`Thermostat::Interrupt`] mode each clears FL/FH and
    /// releases the ALERT pin. Configuring alert hysteresis while an
    /// alert is pending therefore discards it. See the crate-level
    /// "Interrupt-mode acknowledgement" section.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when any of the transactions fail.
```

And for `AlertTmp108` (3211), which delegates to it:

```rust
    /// Snap a requested hysteresis to one of the chip's four settings
    /// and write it.
    ///
    /// Delegates to the inner [`AsyncTmp108`], and inherits its cost:
    /// **two** configuration reads, each of which clears FL/FH and
    /// releases the ALERT pin in [`Thermostat::Interrupt`] mode. This
    /// does not clear a retained interrupt delivery obligation, which
    /// is wrapper-local. See the crate-level "Interrupt-mode
    /// acknowledgement" section.
    ///
    /// # Errors
    ///
    /// [`Error::Bus`] when any of the transactions fail.
```

Check the actual error type on the `AlertTmp108` impl before writing that `# Errors` line — the wrapper uses `Error<E, P>`, so confirm whether the associated error is `Error::Bus` or the bare I²C error, and match it.

- [ ] **Step 6: Fix the `sensor_mut` doctest (A5)**

In `src/lib.rs`, `AlertTmp108::sensor_mut`'s doc (1395-1418). Add after the existing line at 1398:

```rust
    /// Reads reached this way still acknowledge: a `read_configuration`
    /// through this borrow clears FL/FH and releases the ALERT pin in
    /// [`Thermostat::Interrupt`] mode, without returning the flags. On
    /// this type, prefer
    /// [`read_configuration_and_acknowledge`][AsyncTmp108::read_configuration_and_acknowledge].
```

Then change the doctest call at 1412 from `read_configuration()` to `read_configuration_and_acknowledge()`, and adjust the assertion that follows it to read `.config` — the returned value is now an `AlertSnapshot`, not a `Config`.

**Read the surrounding doctest before editing**: the mock transaction list and the assertion shape must still line up. The transaction itself does not change, because the bus traffic is identical.

- [ ] **Step 7: Run the doctests and the gate**

Run: `cargo test --doc --locked -F async,embedded-sensors-hal-async`
Expected: PASS. The `sensor_mut` doctest is the one at risk.

Run every command in "The verification gate".

Expected: all twenty-one exit 0, no `warning:` lines. The **seven `cargo doc` runs** are the ones that matter for this task: it adds eleven new intra-doc links, and a link to a feature-gated item only warns in the combinations where the documented item exists but the target does not. The baseline is 0 warnings in all seven — any nonzero result is this task's regression.

- [ ] **Step 8: Commit**

Write the message to `$env:TEMP\msg.txt`:

```
docs: document the interrupt-mode acknowledgement effect of configuration reads

In interrupt mode, reading the configuration register clears FL, FH and
the ALERT pin. Eleven public methods did that without saying so,
including probe(), which looks like a liveness check, and
wait_for_temperature(), which reads configuration only to pick a delay.

Adds a canonical crate-level section with a table of every acknowledging
call and its read count, and a short section on each method linking to
it. Also documents the two TemperatureHysteresis impls, which had no
rustdoc at all despite costing two configuration reads per call, and
corrects sensor_mut's example, which demonstrated an unacknowledged
read on a wrapper built for alert handling.

Assisted-by: opencode:claude-opus-5
```

Then:

```bash
git add src/lib.rs
git commit -F $env:TEMP\msg.txt
```

---

## Task 3: README gotcha

Implements spec A6. One commit, separate because `scripts/check-readme-snippets.sh` gates this file and the bullet is independently revertible.

**Files:**
- Modify: `README.md:115-190`

---

- [ ] **Step 1: Add the bullet**

In `README.md`, insert immediately after the existing comparator-vs-interrupt bullet (which ends at line 130) and before the "Retained delivery precedes fresh acquisition" bullet at 131:

```markdown
- **Any configuration read acknowledges, not just the waiter's.** In
  interrupt mode every read of the configuration register clears FL/FH and
  releases the ALERT pin — including `probe()`, which looks like a liveness
  check, `wait_for_temperature()`, which reads it only to pick a delay,
  `configure()` and `shutdown()`, whose read-modify-write acknowledges on
  the read half, and `set_temperature_threshold_hysteresis()`, which does it
  twice. A configuration *write* does not acknowledge. Only
  `read_configuration_and_acknowledge()` returns the flags it consumed;
  everything else discards them. See the
  [crate documentation](https://docs.rs/tmp108/latest/tmp108/#interrupt-mode-acknowledgement)
  for the full table.
```

The existing bullet at 126-130 stays unchanged — it is correct, just narrower than a reader will assume.

- [ ] **Step 2: Check the snippet script still passes**

Run: `bash ./scripts/check-readme-snippets.sh`

Expected: `README snippets match (3 checked).`

The new bullet is outside all three marker regions, so this should pass untouched. If it fails, the bullet landed inside a `<!-- snippet: NAME -->` block — move it.

- [ ] **Step 3: Check the anchor resolves**

The link targets `#interrupt-mode-acknowledgement`, which rustdoc generates from the `## Interrupt-mode acknowledgement` heading added in Task 2 Step 1. Confirm the heading text matches exactly, including case, or the fragment will 404 silently.

Run: `cargo doc --no-deps --all-features --locked` then grep the generated HTML:

```bash
Select-String -Path target/doc/tmp108/index.html -Pattern 'id="interrupt-mode-acknowledgement"'
```

Expected: one match.

- [ ] **Step 4: Run the gate**

Run every command in "The verification gate".

Expected: all twenty-one exit 0, no `warning:` lines. `README.md` is included into the crate docs via `#![doc = include_str!("../README.md")]` at `src/lib.rs:41`, so a malformed bullet shows up as a `cargo doc` warning, not just a cosmetic issue.

- [ ] **Step 5: Commit**

Write the message to `$env:TEMP\msg.txt`:

```
docs(readme): note that any configuration read acknowledges an alert

The Gotchas section described the effect only for the threshold waiter,
where the driver handles it, which reads as reassurance that the driver
handles it everywhere. It does not: probe, wait_for_temperature,
configure, shutdown and set_temperature_threshold_hysteresis all
acknowledge too.

Assisted-by: opencode:claude-opus-5
```

Then:

```bash
git add README.md
git commit -F $env:TEMP\msg.txt
```

---

## Task 4: Per-commit regression check

The gate in Tasks 1-3 proves each commit is clean *when written*. This proves each is clean *in sequence*, which is what bisect and `git rebase` actually depend on.

**Files:** none modified.

---

- [ ] **Step 1: Replay the gate over every commit**

From the tip of the branch:

```bash
git rebase --exec "cargo +nightly fmt --check && cargo clippy --all-features --all-targets --locked -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style && cargo test --locked && cargo test --locked -F async,embedded-sensors-hal-async && cargo hack --feature-powerset check --locked" upstream/main
```

This checks out each commit in turn and runs the command. Expected: the rebase completes without stopping.

If it stops, the named commit is not independently clean. Fix it in place — the rebase is already paused at that commit, so `git commit --amend` then `git rebase --continue`. Do **not** add a fix-up commit on top.

The spec commit `1c22935f6011` is documentation-only and will pass trivially; it is included because a cheap pass is better than a special case.

- [ ] **Step 2: Confirm the full gate at the tip**

Run every command in "The verification gate" one final time at the branch tip.

Expected: all twenty-one exit 0. Compare the test counts against the baseline table — they should be *higher*, with warnings still zero, failures zero, and ignored still 3.

- [ ] **Step 3: Supply chain**

Run: `cargo vet --locked`

Expected: exit 0. No dependency was added, so this should pass unchanged. If it reports anything, a dependency crept in that should not have.

---

## Task 5: Hardware verification

AGENTS.md makes live-hardware verification mandatory for changes that touch alert-pin behavior. This change adds a flags-returning read, so it qualifies.

Rig: Pico de Gallo `5256657D8A5D7F03` with a TMP108 at `0x48`, ALERT on GPIO0.

**Files:** none modified. If this task finds a discrepancy, it is a bug in Task 1, not something to paper over here.

---

- [ ] **Step 1: Reproduce the spec's measurement through the driver**

Create `examples/scratch_ack.rs`. **This file is deliberately not committed** — it exists to prove the method behaves on silicon, then it is deleted in Step 2.

```rust
//! Scratch verification for issue #65. NOT COMMITTED.
//!
//! Hardware: Pico de Gallo + TMP108 at 0x48, ALERT on GPIO0.
//! Proves that `read_configuration_and_acknowledge` returns a latched
//! FH and that the same read releases the ALERT pin.
//!
//! Run with: cargo run --example scratch_ack -F async,embedded-sensors-hal-async

#[cfg(not(feature = "async"))]
fn main() {
    eprintln!("examples/scratch_ack.rs requires --features async");
}

#[cfg(feature = "async")]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use anyhow::anyhow;
    use embedded_hal::digital::InputPin;
    use pico_de_gallo_hal::Hal;
    use pico_de_gallo_lib::{GpioDirection, GpioPull};
    use tmp108::{AsyncTmp108, Celsius, Config, ConversionRate, Hysteresis, Polarity, Thermostat};

    let hal = Hal::new();
    let i2c = hal.i2c();
    let mut alert = hal.gpio(0);
    alert
        .set_config(GpioDirection::Input, GpioPull::Up)
        .map_err(|_| anyhow!("Failed to configure GPIO0 as input"))?;

    let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);

    // 1-2. Interrupt mode at 0.25 Hz, and a high limit well below
    // ambient so the over-temperature condition is true and stays true.
    tmp.configure(Config {
        thermostat_mode: Thermostat::Interrupt,
        alert_polarity: Polarity::ActiveLow,
        conversion_rate: ConversionRate::QuarterHz,
        hysteresis: Hysteresis::OneC,
    })
    .await
    .map_err(|_| anyhow!("configure failed"))?;
    tmp.set_low_limit(Celsius::MIN)
        .await
        .map_err(|_| anyhow!("set_low_limit failed"))?;
    tmp.set_high_limit(Celsius::try_from_degrees(10.0).unwrap())
        .await
        .map_err(|_| anyhow!("set_high_limit failed"))?;

    // 3. One conversion period at 0.25 Hz, plus margin, to latch FH.
    tokio::time::sleep(std::time::Duration::from_secs(6)).await;

    // 4. Park in shutdown so no further conversion can re-assert FH.
    tmp.shutdown().await.map_err(|_| anyhow!("shutdown failed"))?;

    // 5. ALERT asserted. POL = ActiveLow, so asserted reads low.
    assert!(alert.is_low().unwrap(), "expected ALERT asserted before the ack");

    // 6. The acknowledging read hands the flag back.
    let first = tmp
        .read_configuration_and_acknowledge()
        .await
        .map_err(|_| anyhow!("first ack read failed"))?;
    println!("first:  low={} high={}", first.low, first.high);
    assert!(first.high, "FH should be latched");
    assert!(!first.low, "FL should be clear");

    // 7. That read released the pin.
    assert!(alert.is_high().unwrap(), "expected ALERT released by the ack");

    // 8. The evidence is gone, though nothing physical changed.
    let second = tmp
        .read_configuration_and_acknowledge()
        .await
        .map_err(|_| anyhow!("second ack read failed"))?;
    println!("second: low={} high={}", second.low, second.high);
    assert!(!second.high, "FH should have been consumed by the first read");
    assert_eq!(second.config, first.config);

    println!("OK: acknowledgement confirmed on hardware");
    Ok(())
}
```

The preamble (`Hal::new()`, `hal.i2c()`, `hal.gpio(0)`, `set_config`) is copied from `examples/alert_interrupt.rs:60-65`, which is the version that works against the pinned `pico-de-gallo-hal` 0.7.0. Note this uses `GpioPull::Up` where `alert_interrupt.rs` uses `GpioPull::None` — the internal pull-up makes the released state unambiguous if the rig lacks the external 2 kΩ pull-up AGENTS.md describes.

Because the file lives in `examples/`, the `#[cfg]`-guarded `main` is required: `cargo build --examples` compiles it under every feature combination, including ones without `async`, and the gate in Task 4 will fail otherwise.

Run: `cargo run --example scratch_ack -F async,embedded-sensors-hal-async`

Expected output:

```
first:  low=false high=true
second: low=false high=false
OK: acknowledgement confirmed on hardware
```

Diagnosis if it fails:

- `first.high == false` — the decode is wrong; check `decode_alert_snapshot` reads `c.fh()` into `high`, and that FH is bit 4.
- Panic at step 5 (`ALERT asserted`) — the part never latched. The ambient may be below the 10 °C limit, or the sleep was too short for the configured rate.
- Panic at step 7 (`ALERT released`) — the shutdown in step 4 did not take, so a conversion re-asserted between the read and the pin check. This is the exact race the spec's Evidence section avoids.

- [ ] **Step 2: Delete the scratch example and restore the chip**

```bash
git status --short    # must show only examples/scratch_ack.rs as untracked
Remove-Item examples/scratch_ack.rs
```

Then restore the chip: limits wide open and the configuration register back to its original value. The T_high reset value is `0x7FF8` per `src/lib.rs:4429`, but note that bit 3 is not writable — a write of `0x7FF8` reads back `0x7FF0`, and both mean +127.9375 °C. Either is an acceptable restore.

```
write reg 0x03 <- 0x7F,0xF8   (T_high, wide open)
write reg 0x02 <- 0x80,0x00   (T_low, -128 °C)
write reg 0x01 <- 0x26,0x10   (interrupt, 1 Hz, ActiveLow, 1 °C hysteresis)
```

Leaving a 10 °C high limit programmed will make the next person's alert examples behave strangely.

- [ ] **Step 3: Run the example matrix**

```bash
cargo run --example oneshot
cargo run --example oneshot -F async
cargo run --example continuous -F async
cargo run --example sensor_trait -F embedded-sensors-hal
cargo run --example sensor_trait -F async,embedded-sensors-hal-async
cargo run --example alert_interrupt -F async,embedded-sensors-hal-async
cargo run --example alert_comparator -F async,embedded-sensors-hal-async
```

Expected: each prints plausible temperatures and exits 0. The last two need a human to warm the part (and, for `alert_comparator`, to let it cool again) — they will block waiting for a threshold crossing.

If an example hangs with no output at all, check AGENTS.md's "Hardware setup" troubleshooting list before assuming this change broke it.

---

## Done

Branch `config-read-acknowledgement`, four commits on top of `upstream/main`:

1. `docs:` the design spec
2. `feat:` `read_configuration_and_acknowledge`
3. `docs:` the acknowledgement documentation
4. `docs(readme):` the Gotchas bullet

**Do not open a pull request.** The branch is handed back for review first.

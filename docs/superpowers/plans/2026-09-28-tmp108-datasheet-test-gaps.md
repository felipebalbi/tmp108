# Datasheet Test Gaps Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the four open test gaps in [issue #66][issue-66], correct the `continuous()` documentation overpromise, and pin three adjacent gaps found while surveying.

**Architecture:** Test-only work plus one documentation correction and one mechanical move. `conversion_period_us` moves into the private `mod ops` where the crate's other sync/async-agnostic logic lives. The in-tree `tests::timeline` fake gains microsecond resolution so it can pin conversion-period delays. Everything else is new assertions in existing test modules. No public API changes.

**Tech Stack:** Rust 2024, `no_std`, `embedded-hal` 1.0 / `embedded-hal-async` 1.0, `embedded-hal-mock` 0.11.1 (`Mock`, `CheckedDelay`), `tokio` for async tests, `device-driver` 2.x generated registers.

**Spec:** `docs/superpowers/specs/2026-09-28-tmp108-datasheet-test-gaps-design.md`

[issue-66]: https://github.com/OpenDevicePartnership/tmp108/issues/66

---

## Before you start

Read `AGENTS.md`. Three rules bite in this plan:

1. **`cargo fmt` must run on nightly.** Stable silently ignores `rustfmt.toml`'s unstable options and CI does not. Run `cargo +nightly fmt` before every commit.
2. **Each commit builds clean and passes clippy on its own.** Do not batch a formatting fix into a later commit; amend the originating one.
3. **Never edit `Cargo.toml`'s version or `CHANGELOG.md`.** release-plz owns both. The commit subject is the changelog entry.

Commit messages use Conventional Commits and end with an AI-attribution trailer. **Verify your own model identity** rather than copying the one below:

```
Assisted-by: opencode:<your-model-id>
```

Keep commit subjects and bodies short and factual. No padding.

Because PowerShell has no heredoc, write commit messages to a file and use `git commit -F`:

```powershell
# Use the Write tool to create C:\Users\febalbi\AppData\Local\Temp\opencode\commitmsg.txt
git commit -F "C:\Users\febalbi\AppData\Local\Temp\opencode\commitmsg.txt"
```

### Feature combinations

Tests live behind different feature gates. The three test commands that matter:

```bash
cargo test --locked                                       # blocking only
cargo test --locked -F async                              # + AsyncTmp108
cargo test --locked -F async,embedded-sensors-hal-async   # + AlertTmp108, snap_hysteresis
```

`snap_hysteresis` and everything in `ops_tests::hysteresis` **only compile under the third command**. If a hysteresis test appears to vanish, you ran the wrong one.

### Register encoding cheat sheet

The configuration register is a little-endian word; wire byte 0 is the LE **low** byte. Power-on reset is word `0x1022`, wire bytes `[0x22, 0x10]`.

Wire byte 0 bit layout: `M` = bits 1:0, `TM` = bit 2, `FL` = bit 3, `FH` = bit 4, `CR` = bits 6:5, `ID` = bit 7.

Conversion rate is therefore selected by bits 6:5 of wire byte 0:

| Rate | CR | Wire byte 0 | Period |
|---|---|---|---|
| `QuarterHz` | `0b00` | `0x02` | 4 000 000 µs |
| `OneHz` | `0b01` | `0x22` (POR) | 1 000 000 µs |
| `FourHz` | `0b10` | `0x42` | 250 000 µs |
| `SixteenHz` | `0b11` | `0x62` | 62 500 µs |

Wire byte 1 stays `0x10` throughout (`HYS` = `OneC`, `POL` = `ActiveLow`).

---

## File Structure

One production file is touched, and only in two places.

| File | Responsibility | Change |
|---|---|---|
| `src/lib.rs` — `mod ops` (`src/lib.rs:269`) | Sync/async-agnostic codec and decision logic | Gains `conversion_period_us` (Task 1) |
| `src/lib.rs` — `AsyncTmp108::continuous` doc (`src/lib.rs:2915`) | Public documentation | Wording correction (Task 10) |
| `src/lib.rs` — `mod tests::timeline` (`src/lib.rs:4531`) | Ordered cross-peripheral event log fake | `Event::Delay` becomes microseconds (Task 2) |
| `src/lib.rs` — `mod tests::ops_tests::celsius` (`src/lib.rs:3881`) | Pure temperature-codec tests | Tasks 5, 6, 8, 12 |
| `src/lib.rs` — `mod tests::ops_tests::hysteresis` | Hysteresis snapping tests | Created in Task 7 |
| `src/lib.rs` — `mod tests::ops_tests::conversion_rate` | Conversion-period tests | Created in Task 3 |
| `src/lib.rs` — `mod tests::blocking::wait_for_temperature` | Blocking ordering test | Created in Task 4 |
| `src/lib.rs` — `mod tests::asynchronous::wait_for_temperature` | Async ordering test | Created in Task 4 |
| `src/lib.rs` — `mod tests::asynchronous` (`src/lib.rs:5427`) | Async driver tests | Tasks 9, 11 |

No new files. The crate is deliberately single-file plus generated `src/inner.rs`.

**Do not touch `src/inner.rs`.** It is generated from `tmp108.ddsl`. Nothing in this plan requires regenerating it.

---

## Task 1: Move `conversion_period_us` into `mod ops`

`conversion_period_us` is a crate-level private `const fn` at `src/lib.rs:3158`, but it is sync/async-agnostic logic shared by both `wait_for_temperature` bodies — exactly what AGENTS.md says belongs in `ops`. Moving it also puts its test in the module AGENTS.md designates for pure-function tests.

Mechanical move. No behaviour change, no public API change.

**Files:**
- Modify: `src/lib.rs:269-276` (add the `ConversionRate` import to `ops`)
- Modify: `src/lib.rs:755-763` (insert the function at the end of `ops`)
- Modify: `src/lib.rs:3158-3166` (delete the original)
- Modify: `src/lib.rs:2346`, `src/lib.rs:3033` (call sites)

- [ ] **Step 1: Add the `ConversionRate` import to `mod ops`**

`ops` currently imports `Config` but not `ConversionRate`. Replace `src/lib.rs:272`:

```rust
    use crate::Config;
```

with:

```rust
    use crate::{Config, ConversionRate};
```

- [ ] **Step 2: Insert the function at the end of `mod ops`**

`mod ops` closes at `src/lib.rs:763`. Insert immediately **before** that closing brace, after `check_prepared` ends at `src/lib.rs:762`:

```rust

    /// The chip's conversion period (1/CR) in microseconds.
    ///
    /// Total over the four rates SBOS663A §7.5.3.5 defines. Callers
    /// use it to size the sleep between requesting a conversion and
    /// reading the result, so an understated value here returns a
    /// stale sample rather than failing.
    pub(crate) const fn conversion_period_us(rate: ConversionRate) -> u32 {
        match rate {
            ConversionRate::QuarterHz => 4_000_000,
            ConversionRate::OneHz => 1_000_000,
            ConversionRate::FourHz => 250_000,
            ConversionRate::SixteenHz => 62_500,
        }
    }
```

- [ ] **Step 3: Delete the original**

Delete `src/lib.rs:3158-3166` in full, including the blank line that follows it, leaving `impl` block end at `src/lib.rs:3156` followed directly by the `/// Blocking-side I²C wire interface` doc comment:

```rust
/// Compute the chip's conversion period (1/CR) in microseconds.
const fn conversion_period_us(rate: ConversionRate) -> u32 {
    match rate {
        ConversionRate::QuarterHz => 4_000_000,
        ConversionRate::OneHz => 1_000_000,
        ConversionRate::FourHz => 250_000,
        ConversionRate::SixteenHz => 62_500,
    }
}
```

- [ ] **Step 4: Update the blocking call site**

At `src/lib.rs:2346`, change:

```rust
        delay.delay_us(conversion_period_us(config.conversion_rate));
```

to:

```rust
        delay.delay_us(ops::conversion_period_us(config.conversion_rate));
```

- [ ] **Step 5: Update the async call site**

At `src/lib.rs:3033`, change:

```rust
        delay.delay_us(conversion_period_us(config.conversion_rate)).await;
```

to:

```rust
        delay.delay_us(ops::conversion_period_us(config.conversion_rate)).await;
```

- [ ] **Step 6: Verify the move compiles in every feature combination**

```bash
cargo hack --feature-powerset check --locked
```

Expected: all 6 combinations succeed, no warnings.

A failure naming `ConversionRate` as unused means the default-feature build does not reach the function. It does — `Tmp108::wait_for_temperature` is unconditional — so investigate rather than adding `#[allow]`.

- [ ] **Step 7: Verify nothing else changed**

```bash
cargo test --locked
cargo test --locked -F async,embedded-sensors-hal-async
```

Expected: PASS, identical test counts to before the move.

- [ ] **Step 8: Format and commit**

```bash
cargo +nightly fmt
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
```

Expected: no output from either.

Commit message file contents:

```
refactor: move conversion_period_us into the shared ops module

It is sync/async-agnostic logic used by both wait_for_temperature
bodies, which is what mod ops is for. No behaviour change.

Assisted-by: opencode:<your-model-id>
```

```bash
git add src/lib.rs
git commit -F "C:\Users\febalbi\AppData\Local\Temp\opencode\commitmsg.txt"
```

---

## Task 2: Give the timeline fake microsecond resolution

`tests::timeline::Event::Delay` stores **milliseconds** (`src/lib.rs:4538`) because `one_shot` calls `delay_ms`. `wait_for_temperature` calls `delay_us`, which neither `Clock` impl overrides, so it falls through `embedded-hal`'s default to `delay_ns` and then to `record(Event::Delay(ns / 1_000_000))`.

At 16 Hz that is 62 500 µs → 62 500 000 ns → **62 ms**. The half millisecond is silently truncated and Task 4 would pin a value the driver never requested.

This task changes the unit to microseconds. It is a prerequisite for Task 4 and lands separately so that task's diff is only new tests.

**Files:**
- Modify: `src/lib.rs:4536-4543` (`Event` doc)
- Modify: `src/lib.rs:4650-4661` (blocking `Clock`)
- Modify: `src/lib.rs:4676-4691` (`Pause`)
- Modify: `src/lib.rs:4693-4723` (async `Pause`/`Clock`)
- Modify: `src/lib.rs:4731-4753` (`expected_one_shot_timeline`)
- Modify: `src/lib.rs:4789-4835` (`timeline::fakes` self-tests)

- [ ] **Step 1: Change the `Event::Delay` doc to say microseconds**

At `src/lib.rs:4536-4538`, change:

```rust
        pub enum Event {
            /// A delay of this many milliseconds was requested.
            Delay(u32),
```

to:

```rust
        pub enum Event {
            /// A delay of this many **microseconds** was requested.
            ///
            /// Microseconds rather than milliseconds because
            /// `wait_for_temperature` requests a 16 Hz conversion
            /// period of 62 500 µs, which has no exact millisecond
            /// representation. Recording milliseconds truncated it to
            /// 62 and pinned a value the driver never asked for.
            Delay(u32),
```

- [ ] **Step 2: Rewrite the blocking `Clock` impl**

Replace `src/lib.rs:4650-4661` in full:

```rust
        impl embedded_hal::delay::DelayNs for Clock {
            fn delay_ns(&mut self, ns: u32) {
                record(&self.log, Event::Delay(ns / 1_000_000));
            }

            // Overridden so a millisecond request lands in the log as
            // one event rather than as the default implementation's
            // loop of microsecond waits.
            fn delay_ms(&mut self, ms: u32) {
                record(&self.log, Event::Delay(ms));
            }
        }
```

with:

```rust
        impl embedded_hal::delay::DelayNs for Clock {
            fn delay_ns(&mut self, ns: u32) {
                record(&self.log, Event::Delay(ns / 1_000));
            }

            // Both overridden so a request lands in the log as one
            // event rather than as the default implementation's loop,
            // and so no unit conversion rounds on the way in.
            fn delay_us(&mut self, us: u32) {
                record(&self.log, Event::Delay(us));
            }

            fn delay_ms(&mut self, ms: u32) {
                record(&self.log, Event::Delay(ms * 1_000));
            }
        }
```

- [ ] **Step 3: Rename `Pause`'s field to its new unit**

Replace `src/lib.rs:4676-4691`:

```rust
        pub struct Pause {
            log: Log,
            ms: u32,
            suspended: bool,
        }

        #[cfg(feature = "async")]
        impl Pause {
            fn new(log: &Log, ms: u32) -> Self {
                Self {
                    log: log.clone(),
                    ms,
                    suspended: false,
                }
            }
        }
```

with:

```rust
        pub struct Pause {
            log: Log,
            us: u32,
            suspended: bool,
        }

        #[cfg(feature = "async")]
        impl Pause {
            fn new(log: &Log, us: u32) -> Self {
                Self {
                    log: log.clone(),
                    us,
                    suspended: false,
                }
            }
        }
```

Leave the `#[cfg(feature = "async")]` attribute at `src/lib.rs:4675` in place above the struct.

- [ ] **Step 4: Update `Pause::poll`**

At `src/lib.rs:4702`, change:

```rust
                    record(&self.log, Event::Delay(self.ms));
```

to:

```rust
                    record(&self.log, Event::Delay(self.us));
```

- [ ] **Step 5: Rewrite the async `Clock` impl**

Replace `src/lib.rs:4714-4723`:

```rust
        #[cfg(feature = "async")]
        impl embedded_hal_async::delay::DelayNs for Clock {
            fn delay_ns(&mut self, ns: u32) -> impl core::future::Future<Output = ()> {
                Pause::new(&self.log, ns / 1_000_000)
            }

            fn delay_ms(&mut self, ms: u32) -> impl core::future::Future<Output = ()> {
                Pause::new(&self.log, ms)
            }
        }
```

with:

```rust
        #[cfg(feature = "async")]
        impl embedded_hal_async::delay::DelayNs for Clock {
            fn delay_ns(&mut self, ns: u32) -> impl core::future::Future<Output = ()> {
                Pause::new(&self.log, ns / 1_000)
            }

            fn delay_us(&mut self, us: u32) -> impl core::future::Future<Output = ()> {
                Pause::new(&self.log, us)
            }

            fn delay_ms(&mut self, ms: u32) -> impl core::future::Future<Output = ()> {
                Pause::new(&self.log, ms * 1_000)
            }
        }
```

- [ ] **Step 6: Update `expected_one_shot_timeline`**

Three `Event::Delay` values at `src/lib.rs:4738`, `:4746` and `:4748` are in milliseconds. Replace the body of `expected_one_shot_timeline` (`src/lib.rs:4731-4753`) with:

```rust
        pub fn expected_one_shot_timeline(settle_ms: u32) -> Vec<Event> {
            vec![
                // Preparation: read-modify-write M = 0b00.
                Event::Read(0x01),
                Event::Write(0x01),
                // The caller's settling delay, before anything is
                // concluded about the part being quiescent. Logged in
                // microseconds; the caller states it in milliseconds.
                Event::Delay(settle_ms * 1_000),
                // The re-read that gates the trigger.
                Event::Read(0x01),
                // Trigger: read-modify-write M = 0b01.
                Event::Read(0x01),
                Event::Write(0x01),
                // Each poll delays *first*, then reads. A read here
                // before its delay would be sampling M at 0 ms.
                Event::Delay(ops::ONE_SHOT_POLL_INTERVAL_MS * 1_000),
                Event::Read(0x01),
                Event::Delay(ops::ONE_SHOT_POLL_INTERVAL_MS * 1_000),
                Event::Read(0x01),
                // And only now the temperature register.
                Event::Read(0x00),
            ]
        }
```

Using `ops::ONE_SHOT_POLL_INTERVAL_MS` rather than a literal `5_000` keeps the expectation pinned to the constant. `ops` resolves here because `mod tests` has `use super::*` at `src/lib.rs:3659`.

- [ ] **Step 7: Update the two `timeline::fakes` self-tests**

These test the fakes themselves and assert `Event::Delay(5)` after calling `delay_ms(5)`. At `src/lib.rs:4802`, change:

```rust
                    vec![Event::Read(0x01), Event::Delay(5)],
```

to:

```rust
                    vec![Event::Read(0x01), Event::Delay(5_000)],
```

At `src/lib.rs:4817`, change:

```rust
                assert_eq!(events(&log), vec![Event::Delay(5), Event::Read(0x01)]);
```

to:

```rust
                assert_eq!(events(&log), vec![Event::Delay(5_000), Event::Read(0x01)]);
```

Leave the `clock.delay_ms(5)` calls at `src/lib.rs:4795` and `src/lib.rs:4813` alone — the point is that a 5 ms request now logs as 5 000 µs.

- [ ] **Step 8: Run the tests that depend on the timeline**

```bash
cargo test --locked one_shot
cargo test --locked -F async one_shot
cargo test --locked -F async fakes
```

Expected: PASS. Specifically `every_step_happens_in_the_documented_order` must pass in both the blocking and async `one_shot` modules, and all three `fakes` tests must pass.

If `every_step_happens_in_the_documented_order` fails with a diff showing `Delay(40)` against `Delay(40000)`, Step 6 was not applied.

- [ ] **Step 9: Run the full suite**

```bash
cargo test --locked
cargo test --locked -F async
cargo test --locked -F async,embedded-sensors-hal-async
```

Expected: PASS in all three.

- [ ] **Step 10: Format, lint and commit**

```bash
cargo +nightly fmt
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
```

Commit message file contents:

```
test: record timeline delays in microseconds

The Clock fake recorded milliseconds, and neither impl overrode
delay_us. A 62 500 us conversion period reached the log through
delay_ns as 62 ms, losing the half millisecond. Nothing needed
microsecond resolution until now, so nothing caught it.

Assisted-by: opencode:<your-model-id>
```

```bash
git add src/lib.rs
git commit -F "C:\Users\febalbi\AppData\Local\Temp\opencode\commitmsg.txt"
```

---

## Task 3: Pin the conversion period for every rate

`ops::conversion_period_us` has no assertion anywhere. It is total over a four-inhabitant enum, so covering all four costs nothing and removes the possibility that three of them are wrong.

**Files:**
- Modify: `src/lib.rs` — insert a new `mod conversion_rate` inside `mod tests::ops_tests`

- [ ] **Step 1: Write the failing test**

Insert immediately after the `mod celsius { ... }` block closes (currently `src/lib.rs:4020`) and before the `#[cfg(all(feature = "embedded-sensors-hal-async", ...))]` attribute on `snap_hysteresis_accepts_within_tolerance`:

```rust

        /// The conversion period the driver sleeps for between
        /// requesting a sample and reading it.
        mod conversion_rate {
            use super::*;

            /// SBOS663A §7.5.3.5 defines four conversion rates. The
            /// period is 1/CR, and it is what `wait_for_temperature`
            /// sleeps for — understate it and the caller reads the
            /// previous conversion.
            ///
            /// All four, not a sample: the function is total over a
            /// four-inhabitant enum, so there is no reason to leave
            /// three of them unchecked.
            #[test]
            fn every_rate_has_its_documented_period() {
                for (rate, expected_us) in [
                    (ConversionRate::QuarterHz, 4_000_000_u32),
                    (ConversionRate::OneHz, 1_000_000),
                    (ConversionRate::FourHz, 250_000),
                    (ConversionRate::SixteenHz, 62_500),
                ] {
                    assert_eq!(
                        ops::conversion_period_us(rate),
                        expected_us,
                        "{rate:?} must sleep for 1/CR"
                    );
                }
            }

            /// 16 Hz is the one rate whose period is not a whole
            /// number of milliseconds. A driver that worked in
            /// milliseconds would sleep 62 ms and read early.
            #[test]
            fn the_fastest_rate_is_not_a_whole_millisecond() {
                assert_eq!(ops::conversion_period_us(ConversionRate::SixteenHz), 62_500);
                assert_ne!(ops::conversion_period_us(ConversionRate::SixteenHz) % 1_000, 0);
            }
        }
```

- [ ] **Step 2: Run the test**

```bash
cargo test --locked conversion_rate
```

Expected: PASS, 2 tests. These characterise existing correct behaviour, so they pass immediately — that is expected for a coverage-gap task, not a TDD failure to chase.

To confirm the tests actually bind to the implementation rather than passing vacuously, temporarily change `ConversionRate::FourHz => 250_000` to `=> 250_001` in `mod ops`, re-run, and confirm `every_rate_has_its_documented_period` FAILS. Revert before committing.

- [ ] **Step 3: Run the full suite**

```bash
cargo test --locked
cargo test --locked -F async,embedded-sensors-hal-async
```

Expected: PASS.

- [ ] **Step 4: Format, lint and commit**

```bash
cargo +nightly fmt
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
```

Commit message file contents:

```
test: pin the conversion period for every conversion rate

conversion_period_us had no assertion at all. Four inhabitants, so
all four are checked rather than sampled.

Assisted-by: opencode:<your-model-id>
```

```bash
git add src/lib.rs
git commit -F "C:\Users\febalbi\AppData\Local\Temp\opencode\commitmsg.txt"
```

---

## Task 4: Pin `wait_for_temperature`'s read-delay-read ordering

`wait_for_temperature` has no unit test in either flavor. Its only exercise is two doctests (`src/lib.rs:2329`, `src/lib.rs:3011`), both using `NoopDelay`, which asserts nothing. Deleting the `delay.delay_us(...)` call from either body fails no test today.

Duration alone is not the property at risk — the *sequence* is. A test that only checks "a delay of 250 000 µs happened" still passes if the delay moves after `temperature()`, which is exactly the bug that returns a stale conversion. `tests::timeline` puts both peripherals on one ordered log.

**Files:**
- Modify: `src/lib.rs` — new `mod wait_for_temperature` inside `mod tests::blocking`
- Modify: `src/lib.rs` — new `mod wait_for_temperature` inside `mod tests::asynchronous`

- [ ] **Step 1: Write the blocking ordering test**

Insert inside `mod tests::blocking`, immediately after the `mod one_shot { ... }` block closes (currently `src/lib.rs:5380`) and before `read_configuration_and_acknowledge_surfaces_flags`:

```rust

        /// `wait_for_temperature` reads the configuration only to size
        /// its sleep, then reads the temperature. Both peripherals go
        /// on one timeline, because the duration is not the property
        /// at risk — the ordering is. A delay moved after the
        /// temperature read still sleeps for the right length and
        /// still returns the previous conversion.
        mod wait_for_temperature {
            use super::super::timeline;
            use super::*;

            /// Wire byte 0 selecting `rate`, with every other field at
            /// its reset value. CR is bits 6:5 of the low byte.
            fn configuration_byte(rate: ConversionRate) -> u8 {
                match rate {
                    ConversionRate::QuarterHz => 0x02,
                    ConversionRate::OneHz => 0x22,
                    ConversionRate::FourHz => 0x42,
                    ConversionRate::SixteenHz => 0x62,
                }
            }

            #[test]
            fn reads_the_configuration_then_sleeps_then_reads_the_temperature() {
                for rate in [
                    ConversionRate::QuarterHz,
                    ConversionRate::OneHz,
                    ConversionRate::FourHz,
                    ConversionRate::SixteenHz,
                ] {
                    let log = timeline::log();
                    let bus = timeline::Bus::new(
                        &log,
                        &[[configuration_byte(rate), 0x10], [0x32, 0x00]],
                    );
                    let mut sensor = Tmp108::new_with_a0_gnd(bus);
                    let mut clock = timeline::Clock::new(&log);

                    let temp = sensor.wait_for_temperature(&mut clock).unwrap();

                    assert_approx_eq!(temp.to_degrees(), 50.0);
                    assert_eq!(
                        timeline::events(&log),
                        vec![
                            timeline::Event::Read(0x01),
                            timeline::Event::Delay(ops::conversion_period_us(rate)),
                            timeline::Event::Read(0x00),
                        ],
                        "{rate:?}"
                    );
                }
            }
        }
```

- [ ] **Step 2: Run the blocking test**

```bash
cargo test --locked wait_for_temperature
```

Expected: PASS, 1 test.

Now prove it bites. Temporarily reorder the body of `Tmp108::wait_for_temperature` (`src/lib.rs:2344-2348`) so the delay comes last:

```rust
    pub fn wait_for_temperature<DELAY: DelayNs>(&mut self, delay: &mut DELAY) -> Result<Celsius, I2C::Error> {
        let config = self.read_configuration()?;
        let temperature = self.temperature();
        delay.delay_us(ops::conversion_period_us(config.conversion_rate));
        temperature
    }
```

Re-run. Expected: FAIL, with the log showing `Read(0x00)` before `Delay(..)`. **Revert the production change** before continuing.

- [ ] **Step 3: Write the async ordering test**

Insert inside `mod tests::asynchronous`, immediately after the `mod one_shot { ... }` block closes (at the end of the module, currently around `src/lib.rs:9568`) and before `read_configuration_and_acknowledge_surfaces_flags`:

```rust

        /// The async mirror of the blocking ordering test.
        ///
        /// Sharper than its blocking twin: `timeline::Pause` records
        /// when it is *polled to completion*, not when it is created,
        /// and returns `Pending` once before completing. A body that
        /// built its delay future early and awaited it late would be
        /// caught here.
        mod wait_for_temperature {
            use super::super::timeline;
            use super::*;

            /// Wire byte 0 selecting `rate`, with every other field at
            /// its reset value. CR is bits 6:5 of the low byte.
            fn configuration_byte(rate: ConversionRate) -> u8 {
                match rate {
                    ConversionRate::QuarterHz => 0x02,
                    ConversionRate::OneHz => 0x22,
                    ConversionRate::FourHz => 0x42,
                    ConversionRate::SixteenHz => 0x62,
                }
            }

            #[tokio::test]
            async fn reads_the_configuration_then_sleeps_then_reads_the_temperature() {
                for rate in [
                    ConversionRate::QuarterHz,
                    ConversionRate::OneHz,
                    ConversionRate::FourHz,
                    ConversionRate::SixteenHz,
                ] {
                    let log = timeline::log();
                    let bus = timeline::Bus::new(
                        &log,
                        &[[configuration_byte(rate), 0x10], [0x32, 0x00]],
                    );
                    let mut sensor = AsyncTmp108::new_with_a0_gnd(bus);
                    let mut clock = timeline::Clock::new(&log);

                    let temp = sensor.wait_for_temperature(&mut clock).await.unwrap();

                    assert_approx_eq!(temp.to_degrees(), 50.0);
                    assert_eq!(
                        timeline::events(&log),
                        vec![
                            timeline::Event::Read(0x01),
                            timeline::Event::Delay(ops::conversion_period_us(rate)),
                            timeline::Event::Read(0x00),
                        ],
                        "{rate:?}"
                    );
                }
            }
        }
```

- [ ] **Step 4: Run the async test**

```bash
cargo test --locked -F async wait_for_temperature
```

Expected: PASS, 2 tests (blocking + async).

- [ ] **Step 5: Run the full suite**

```bash
cargo test --locked
cargo test --locked -F async
cargo test --locked -F async,embedded-sensors-hal-async
```

Expected: PASS in all three.

- [ ] **Step 6: Format, lint and commit**

```bash
cargo +nightly fmt
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
```

Commit message file contents:

```
test: pin wait_for_temperature's read-delay-read ordering

Neither flavor had a unit test; the only coverage was two doctests
using NoopDelay. Deleting the delay call, or moving it after the
temperature read, failed nothing. Both peripherals go on one
timeline so the interleaving is asserted, not just the duration.

Assisted-by: opencode:<your-model-id>
```

```bash
git add src/lib.rs
git commit -F "C:\Users\febalbi\AppData\Local\Temp\opencode\commitmsg.txt"
```

---

## Task 5: Give the temperature decoder an independent oracle

`decoding_a_register_is_total` (`src/lib.rs:3897-3907`) walks all 65,536 words but asserts only that the result falls *inside* the representable range. Every permutation of the 4096 values passes that. The companion round-trip at `src/lib.rs:3910` compares the encoder against its own inverse, so a mutually-consistent wrong permutation passes that too.

This task replaces the range check with a value oracle computed independently of the implementation, and adds the matching encode-direction test.

**Files:**
- Modify: `src/lib.rs:3891-3907` (helper region and the range test) inside `mod tests::ops_tests::celsius`

- [ ] **Step 1: Add the oracle helper**

Insert into `mod celsius` immediately after the `all()` helper closes at `src/lib.rs:3895`:

```rust

            /// Sign-extend a 12-bit two's complement code, computed
            /// without reference to the driver.
            ///
            /// This is the oracle. It must not call anything in
            /// `ops`, or the tests below become the implementation
            /// checked against itself — which is exactly the hole
            /// they exist to close.
            fn sign_extend_12(code: u16) -> i16 {
                assert!(code < 4096, "not a 12-bit code: {code:#06x}");
                let widened = i32::from(code);
                let signed = if widened < 2048 { widened } else { widened - 4096 };
                i16::try_from(signed).expect("-2048 ..= 2047 fits an i16")
            }
```

- [ ] **Step 2: Replace the range-membership test with the oracle**

Replace `src/lib.rs:3897-3907` in full:

```rust
            #[test]
            fn decoding_a_register_is_total() {
                // All 65,536 bit patterns, not a sample of them.
                for word in 0..=u16::MAX {
                    let c = Celsius::from_register(word.to_be_bytes());
                    assert!(
                        (MIN_SIXTEENTHS..=MAX_SIXTEENTHS).contains(&c.sixteenths()),
                        "word {word:#06x} decoded out of range: {c:?}"
                    );
                }
            }
```

with:

```rust
            #[test]
            fn decoding_a_register_matches_an_independent_oracle() {
                // All 65,536 bit patterns, and the *value* of each,
                // not merely that it landed in range. Range membership
                // alone is satisfied by every permutation of the 4096
                // readings, and the round-trip tests below compare the
                // encoder against its own inverse — so neither can see
                // a mutually-consistent wrong mapping.
                //
                // The part left-justifies a 12-bit two's complement
                // field in 16 bits, so the code is the top 12 bits and
                // the low nibble is discarded.
                for word in 0..=u16::MAX {
                    let c = Celsius::from_register(word.to_be_bytes());
                    assert_eq!(
                        c.sixteenths(),
                        sign_extend_12(word >> 4),
                        "word {word:#06x}"
                    );
                    // Still total, and now for a stated reason.
                    assert!((MIN_SIXTEENTHS..=MAX_SIXTEENTHS).contains(&c.sixteenths()));
                }
            }

            #[test]
            fn encoding_matches_an_independent_oracle() {
                // The other direction, over the 4096 codes that have a
                // canonical encoding. Byte 0 is the code's top eight
                // bits; byte 1 is its low nibble left-justified, with
                // the four hardwired-zero bits below it.
                for code in 0_u16..4096 {
                    let c = Celsius::from_sixteenths(sign_extend_12(code))
                        .expect("every 12-bit code names a representable temperature");
                    let expected = [
                        u8::try_from(code >> 4).expect("12 bits less 4 is 8"),
                        u8::try_from((code & 0xf) << 4).expect("a nibble shifted up by 4 is a byte"),
                    ];
                    assert_eq!(c.to_register(), expected, "code {code:#05x}");
                }
            }

            #[test]
            fn the_fractional_bit_weights_are_visible() {
                // One LSB is 0.0625 °C and two are 0.125 °C. Spelled
                // out rather than left implicit in an exhaustive loop,
                // so a reader can see the individual bit weights.
                assert_eq!(Celsius::try_from_degrees(0.0625).unwrap().sixteenths(), 1);
                assert_eq!(Celsius::try_from_degrees(-0.0625).unwrap().sixteenths(), -1);
                assert_eq!(Celsius::try_from_degrees(0.125).unwrap().sixteenths(), 2);
                assert_eq!(Celsius::try_from_degrees(-0.125).unwrap().sixteenths(), -2);

                assert_eq!(Celsius::from_sixteenths(1).unwrap().to_degrees(), 0.0625);
                assert_eq!(Celsius::from_sixteenths(-1).unwrap().to_degrees(), -0.0625);
                assert_eq!(Celsius::from_sixteenths(2).unwrap().to_degrees(), 0.125);
                assert_eq!(Celsius::from_sixteenths(-2).unwrap().to_degrees(), -0.125);
            }
```

- [ ] **Step 3: Run the tests**

```bash
cargo test --locked celsius
```

Expected: PASS. The 65 536-iteration loop is not instant in a debug build; allow a few seconds.

- [ ] **Step 4: Prove the oracle bites**

Temporarily change `ops::Celsius::from_register` (`src/lib.rs:471-476`) to use a logical shift on an unsigned value, which breaks sign extension:

```rust
        pub(crate) const fn from_register(raw: [u8; 2]) -> Self {
            Self((u16::from_be_bytes(raw) >> REGISTER_SHIFT) as i16)
        }
```

```bash
cargo test --locked decoding_a_register_matches_an_independent_oracle
```

Expected: FAIL. The old range-membership test would have failed here too, but only because the result left the range — it could not have caught a wrong mapping that stayed inside it.

**Revert the production change.**

- [ ] **Step 5: Run the full suite**

```bash
cargo test --locked
cargo test --locked -F async,embedded-sensors-hal-async
```

Expected: PASS.

- [ ] **Step 6: Format, lint and commit**

```bash
cargo +nightly fmt
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
```

Commit message file contents:

```
test: give the temperature decoder an independent oracle

The 65,536-input test asserted only range membership, which every
permutation of the 4096 readings satisfies, and the round-trip test
compares the encoder against its own inverse. Neither could see a
mutually-consistent wrong mapping. Both directions now check against
a sign-extension oracle that does not call into ops.

Assisted-by: opencode:<your-model-id>
```

```bash
git add src/lib.rs
git commit -F "C:\Users\febalbi\AppData\Local\Temp\opencode\commitmsg.txt"
```

---

## Task 6: Assert the application note's worked values

SBAA588A works three conversions by hand. None appears in the test suite. All three were verified against the oracle while writing the spec, and all three re-encode exactly:

| Word | Code | Sixteenths | Degrees |
|---|---|---|---|
| `0x2090` | 521 | 521 | 32.5625 |
| `0xFAE0` | 4014 | −82 | −5.125 |
| `0x1880` | 392 | 392 | 24.5 |

Kept separate from `known_datasheet_values_decode` because the source document differs — SBAA588A, not SBOS663A Table 7.

**Files:**
- Modify: `src/lib.rs` — insert after `known_datasheet_values_decode` inside `mod tests::ops_tests::celsius`

- [ ] **Step 1: Write the test**

Insert immediately after `known_datasheet_values_decode` closes (currently `src/lib.rs:4004`):

```rust

            #[test]
            fn application_note_worked_values_decode() {
                // SBAA588A works these three by hand. They are worth
                // having alongside the exhaustive oracle because a
                // human computed them independently, and because they
                // exercise mixed integer/fractional data and negative
                // fractional decoding rather than the round numbers of
                // Table 7.
                //
                // All three have a zero low nibble, so the encode
                // direction round-trips exactly and is asserted too.
                for (word, degrees) in
                    [(0x2090_u16, 32.5625_f32), (0xfae0, -5.125), (0x1880, 24.5)]
                {
                    let c = Celsius::from_register(word.to_be_bytes());
                    assert_eq!(c.to_degrees(), degrees, "word {word:#06x}");
                    assert_eq!(c.to_register(), word.to_be_bytes(), "word {word:#06x}");
                }
            }
```

- [ ] **Step 2: Run the test**

```bash
cargo test --locked application_note_worked_values_decode
```

Expected: PASS, 1 test.

- [ ] **Step 3: Run the full suite**

```bash
cargo test --locked
cargo test --locked -F async,embedded-sensors-hal-async
```

Expected: PASS.

- [ ] **Step 4: Format, lint and commit**

```bash
cargo +nightly fmt
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
```

Commit message file contents:

```
test: assert the application note's worked temperature values

SBAA588A works 0x2090, 0xFAE0 and 0x1880 by hand. None was asserted.
They exercise mixed integer/fractional data and negative fractional
decoding, which Table 7's round numbers do not.

Assisted-by: opencode:<your-model-id>
```

```bash
git add src/lib.rs
git commit -F "C:\Users\febalbi\AppData\Local\Temp\opencode\commitmsg.txt"
```

---

## Task 7: Pin the hysteresis band edges

The ±0.05 °C acceptance band in `ops::snap_hysteresis` (`src/lib.rs:494`) is driver policy — the sources specify discrete encodings only (SBOS663A §7.5.3.1 Table 9). It is a legal choice, simply unpinned where floating point makes it non-obvious.

**It is not symmetric.** `HYSTERESIS_TOLERANCE` is `0.05f32`, which is `0.05000000074505805969`. The rejection test is `(input - closest).abs() > HYSTERESIS_TOLERANCE`, and whether the subtraction lands above or below that constant depends on which side of its own value each literal rounded to. Measured by running the function verbatim under `rustc -O`:

| Setting | −0.05 edge | diff | +0.05 edge | diff |
|---|---|---|---|---|
| 0 °C | accepted | `0.05` | accepted | `0.05` |
| 1 °C | **rejected** | `0.050000012` | accepted | `0.049999952` |
| 2 °C | accepted | `0.049999952` | accepted | `0.049999952` |
| 4 °C | accepted | `0.049999952` | **rejected** | `0.05000019` |

Six of eight accepted; the two rejections fall on opposite sides. This task pins that as measured. **Do not "fix" it** — the follow-up issue in Task 12's step list covers that decision separately.

This task also gathers the two hysteresis tests currently sitting loose in `ops_tests` (`src/lib.rs:4022-4053`) into a module, matching how every other subject there is organised.

**Files:**
- Modify: `src/lib.rs:4022-4053` (relocate the two existing tests into a new module, add one)

- [ ] **Step 1: Replace the two loose tests with a gated module**

Replace `src/lib.rs:4022-4053` in full — that is both existing tests including their individual `#[cfg(all(...))]` attributes:

```rust
        #[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
        #[test]
        fn snap_hysteresis_accepts_within_tolerance() {
            let cases: &[(f32, Hysteresis)] = &[
                (0.0, Hysteresis::ZeroC),
                (1.0, Hysteresis::OneC),
                (2.0, Hysteresis::TwoC),
                (4.0, Hysteresis::FourC),
                (0.04, Hysteresis::ZeroC),
                (0.1_f32 + 0.9_f32, Hysteresis::OneC),
                (1.95, Hysteresis::TwoC),
                (3.97, Hysteresis::FourC),
            ];
            for (input, expected) in cases {
                assert_eq!(
                    ops::snap_hysteresis(*input),
                    Some(*expected),
                    "input {input} should snap to {expected:?}"
                );
            }
        }

        #[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
        #[test]
        fn snap_hysteresis_rejects_out_of_tolerance_and_non_finite() {
            for bad in [-0.5_f32, 0.5, 3.0, 5.0, -1.0, 10.0] {
                assert_eq!(ops::snap_hysteresis(bad), None);
            }
            for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                assert_eq!(ops::snap_hysteresis(bad), None);
            }
        }
```

with:

```rust
        /// Snapping a continuous `f32` hysteresis request onto the
        /// four settings the chip actually has.
        ///
        /// The ±0.05 °C acceptance band is driver policy, not a
        /// datasheet tolerance — SBOS663A §7.5.3.1 Table 9 specifies
        /// the four discrete encodings and says nothing about how a
        /// caller's arbitrary float should reach one.
        #[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
        mod hysteresis {
            use super::*;

            #[test]
            fn snap_hysteresis_accepts_within_tolerance() {
                let cases: &[(f32, Hysteresis)] = &[
                    (0.0, Hysteresis::ZeroC),
                    (1.0, Hysteresis::OneC),
                    (2.0, Hysteresis::TwoC),
                    (4.0, Hysteresis::FourC),
                    (0.04, Hysteresis::ZeroC),
                    (0.1_f32 + 0.9_f32, Hysteresis::OneC),
                    (1.95, Hysteresis::TwoC),
                    (3.97, Hysteresis::FourC),
                ];
                for (input, expected) in cases {
                    assert_eq!(
                        ops::snap_hysteresis(*input),
                        Some(*expected),
                        "input {input} should snap to {expected:?}"
                    );
                }
            }

            #[test]
            fn snap_hysteresis_rejects_out_of_tolerance_and_non_finite() {
                for bad in [-0.5_f32, 0.5, 3.0, 5.0, -1.0, 10.0] {
                    assert_eq!(ops::snap_hysteresis(bad), None);
                }
                for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                    assert_eq!(ops::snap_hysteresis(bad), None);
                }
            }

            /// The band is **not** ±0.05 °C at every setting, and this
            /// test records that rather than asserting the symmetry
            /// the constant implies.
            ///
            /// `HYSTERESIS_TOLERANCE` is `0.05f32`, whose exact value
            /// is 0.05000000074505805969. The rejection test is
            /// `(input - closest).abs() > HYSTERESIS_TOLERANCE`, so
            /// whether an edge is inside or outside depends on which
            /// side of its own decimal value each literal rounded to:
            ///
            /// - `1.0 - 0.95f32` is 0.050000012, above the constant.
            /// - `1.05f32 - 1.0` is 0.049999952, below it.
            /// - `4.05f32 - 4.0` is 0.050000191, above it.
            ///
            /// Six of the eight edges are accepted; the two
            /// rejections fall on opposite sides. That asymmetry is a
            /// wart, and it is tracked separately — this test exists
            /// so a change to it is a deliberate one rather than a
            /// silent one.
            #[test]
            fn the_band_edges_are_asymmetric() {
                let edges: &[(f32, Option<Hysteresis>)] = &[
                    (-0.05, Some(Hysteresis::ZeroC)),
                    (0.05, Some(Hysteresis::ZeroC)),
                    (0.95, None),
                    (1.05, Some(Hysteresis::OneC)),
                    (1.95, Some(Hysteresis::TwoC)),
                    (2.05, Some(Hysteresis::TwoC)),
                    (3.95, Some(Hysteresis::FourC)),
                    (4.05, None),
                ];
                for (input, expected) in edges {
                    assert_eq!(
                        ops::snap_hysteresis(*input),
                        *expected,
                        "edge {input} is measured behaviour, not a symmetry claim"
                    );
                }
            }

            /// Halfway between two settings snaps to the lower one.
            ///
            /// `min_by` returns the *first* minimum, and the table is
            /// in ascending order, so a tie resolves downward. The
            /// result is rejected anyway — 0.5 is far outside any
            /// band — but the tie-break is what decides *which*
            /// setting it is measured against.
            #[test]
            fn a_tie_resolves_to_the_lower_setting() {
                assert_eq!(ops::snap_hysteresis(0.5), None);
                assert_eq!(ops::snap_hysteresis(1.5), None);
                assert_eq!(ops::snap_hysteresis(3.0), None);
            }
        }
```

- [ ] **Step 2: Run the tests**

```bash
cargo test --locked -F async,embedded-sensors-hal-async hysteresis
```

Expected: PASS, 4 tests in `ops_tests::hysteresis`.

If `the_band_edges_are_asymmetric` FAILS, **do not adjust the expectations to match**. Report the actual output — it means the toolchain's float behaviour differs from the measurement this plan was built on, which is a finding in its own right.

- [ ] **Step 3: Confirm the gate still works**

```bash
cargo test --locked
cargo test --locked -F async
```

Expected: PASS. `ops_tests::hysteresis` must not appear in either run — it is gated on `embedded-sensors-hal-async`. Confirm with:

```bash
cargo test --locked -- --list | rg hysteresis
```

Expected: no output.

- [ ] **Step 4: Full feature powerset**

```bash
cargo hack --feature-powerset check --locked
```

Expected: all 6 combinations succeed. This catches a module gated on one feature but using a name gated on another.

- [ ] **Step 5: Format, lint and commit**

```bash
cargo +nightly fmt
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
```

Commit message file contents:

```
test: pin the hysteresis band edges as measured

The +/-0.05 C band is not symmetric: 0.95 is rejected while 1.05 is
accepted, and 3.95 is accepted while 4.05 is rejected, because
0.05f32 is 0.05000000074505805969 and the literals round to either
side of it. Pinned as measured so a change to it is deliberate.

The two existing hysteresis tests move into the new module.

Assisted-by: opencode:<your-model-id>
```

```bash
git add src/lib.rs
git commit -F "C:\Users\febalbi\AppData\Local\Temp\opencode\commitmsg.txt"
```

- [ ] **Step 6: File the follow-up issue**

The asymmetry is recorded but not resolved. Open an issue so it can be judged on its merits:

Title: `snap_hysteresis's +/-0.05 C band is asymmetric at two of its eight edges`

Body:

```markdown
`ops::snap_hysteresis` (`src/lib.rs:494`) rejects an input further than
`HYSTERESIS_TOLERANCE` from the nearest legal setting. The constant is
`0.05f32`, whose exact value is `0.05000000074505805969`, and the
comparison is against a subtraction whose result depends on which side
of its decimal value each `f32` literal rounded to.

Measured:

| Setting | −0.05 edge | diff | +0.05 edge | diff |
|---|---|---|---|---|
| 0 °C | accepted | `0.05` | accepted | `0.05` |
| 1 °C | **rejected** | `0.050000012` | accepted | `0.049999952` |
| 2 °C | accepted | `0.049999952` | accepted | `0.049999952` |
| 4 °C | accepted | `0.049999952` | **rejected** | `0.05000019` |

Six of eight edges accepted, and the two rejections fall on opposite
sides — the low edge at 1 °C, the high edge at 4 °C. No single
description of the band is correct for all four settings.

Pinned as measured by `ops_tests::hysteresis::the_band_edges_are_asymmetric`,
so any change is deliberate. Whether it *should* change is this issue.

Options, roughly in increasing order of disruption:

1. Leave it. Document the band as approximate. Callers near an edge
   are already asking for a setting they did not name.
2. Compare against a tolerance scaled to the setting, or against the
   midpoint between adjacent settings, so the decision does not turn
   on one constant's representation.
3. Widen the comparison to `>=` with a slack term, making all eight
   edges inclusive.

Behavioural either way for callers currently sitting on an accepted
edge, so it needs a changelog-visible commit.

Raised while closing #66.
```

```bash
gh issue create --repo OpenDevicePartnership/tmp108 --title "..." --body-file <path>
```

Record the issue number; it is referenced nowhere in code, only here.

---

## Task 8: Pin the rounding ties and rejection boundaries

`Celsius::try_from_degrees` (`src/lib.rs:420`) rounds half away from zero and rejects outside an open interval half an LSB wider than the representable range. Both are driver policy. The existing tests check `0.03`/`0.04` and `-128.03`/`-128.04` — comfortably inside and outside, never at a tie or at an adjacent representable float.

Measured by running the function verbatim under `rustc -O`:

```
low  next_down -128.03127 -> Err(TooLow)
low  exact     -128.03125 -> Err(TooLow)
low  next_up   -128.03123 -> Ok(-2048)     <- Celsius::MIN
high next_down  127.96874 -> Ok(2047)      <- Celsius::MAX
high exact      127.96875 -> Err(TooHigh)
high next_up    127.96876 -> Err(TooHigh)
```

Unlike the hysteresis band, this is clean and symmetric — the boundaries are exactly representable (`-2048.5 / 16` and `2047.5 / 16`) and the comparisons are `<=` and `>=`.

**Files:**
- Modify: `src/lib.rs` — insert after `degrees_round_to_the_nearest_sixteenth` inside `mod tests::ops_tests::celsius`

- [ ] **Step 1: Write the tests**

Insert immediately after `degrees_round_to_the_nearest_sixteenth` closes (currently `src/lib.rs:3982`):

```rust

            #[test]
            fn ties_round_half_away_from_zero() {
                // `f32::round` lives in `std` and this crate is
                // `no_std`, so the conversion adds a half and
                // truncates toward zero. That rounds ties away from
                // zero, and it is policy rather than anything the
                // datasheet asks for — the part only ever reports
                // exact sixteenths.
                //
                // A tie is a value exactly half an LSB above a
                // representable one, i.e. an odd multiple of 1/32.
                for (degrees, expected) in [
                    (0.03125_f32, 1_i16),
                    (-0.03125, -1),
                    (0.09375, 2),
                    (-0.09375, -2),
                    (1.53125, 25),
                    (-1.53125, -25),
                ] {
                    assert_eq!(
                        Celsius::try_from_degrees(degrees).unwrap().sixteenths(),
                        expected,
                        "{degrees} is a tie and must round away from zero"
                    );
                }
            }

            #[test]
            fn the_rejection_boundaries_are_exact_and_closed() {
                // The boundaries are exactly representable: -2048.5/16
                // is -128.03125 and 2047.5/16 is 127.96875. The
                // comparisons are `<=` and `>=`, so the boundary value
                // itself is rejected and the interval is open.
                //
                // Checked one `f32` ulp either side, not at a rounded
                // decimal, because a decimal approximation cannot
                // distinguish "the boundary is closed" from "the
                // boundary is a little further out than I thought".
                // `LOWEST_ACCEPTED` and `HIGHEST_ACCEPTED` are in
                // sixteenths (-2048.5 and 2047.5); dividing by 16
                // gives degrees. Both divisions are exact — the
                // divisor is a power of two.
                let low = -128.03125_f32;
                let high = 127.96875_f32;

                assert_eq!(low, LOWEST_ACCEPTED / 16.0);
                assert_eq!(high, HIGHEST_ACCEPTED / 16.0);

                assert_eq!(Celsius::try_from_degrees(low.next_down()), Err(OutOfRange::TooLow));
                assert_eq!(Celsius::try_from_degrees(low), Err(OutOfRange::TooLow));
                assert_eq!(Celsius::try_from_degrees(low.next_up()), Ok(Celsius::MIN));

                assert_eq!(Celsius::try_from_degrees(high.next_down()), Ok(Celsius::MAX));
                assert_eq!(Celsius::try_from_degrees(high), Err(OutOfRange::TooHigh));
                assert_eq!(Celsius::try_from_degrees(high.next_up()), Err(OutOfRange::TooHigh));
            }
```

`LOWEST_ACCEPTED` and `HIGHEST_ACCEPTED` are already imported into `mod celsius` at `src/lib.rs:3889`. `f32::next_up` and `f32::next_down` are stable since 1.86; the crate's MSRV is 1.94.

- [ ] **Step 2: Run the tests**

```bash
cargo test --locked ties_round_half_away_from_zero
cargo test --locked the_rejection_boundaries_are_exact_and_closed
```

Expected: PASS, 1 test each.

- [ ] **Step 3: Prove the tie test bites**

Temporarily change `ops::Celsius::try_from_degrees` (`src/lib.rs:438-442`) to round half *toward* zero by dropping the half:

```rust
            let rounded = scaled;
```

```bash
cargo test --locked ties_round_half_away_from_zero
```

Expected: FAIL — `0.03125` yields 0 rather than 1.

**Revert the production change.**

- [ ] **Step 4: Run the full suite**

```bash
cargo test --locked
cargo test --locked -F async,embedded-sensors-hal-async
```

Expected: PASS.

- [ ] **Step 5: Format, lint and commit**

```bash
cargo +nightly fmt
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
```

Commit message file contents:

```
test: pin the rounding ties and rejection boundaries

try_from_degrees rounds half away from zero and rejects on a closed
boundary. Both are driver policy and neither was checked at a tie or
at an adjacent representable float; the existing tests used rounded
decimals comfortably inside and outside.

Assisted-by: opencode:<your-model-id>
```

```bash
git add src/lib.rs
git commit -F "C:\Users\febalbi\AppData\Local\Temp\opencode\commitmsg.txt"
```

---

## Task 9: Pin `continuous()`'s entry-failure path

`continuous()` documents unconditional cleanup. An entry-phase error returns before the closure runs, so no cleanup is attempted at all — and nothing tests that. The existing tests cover closure-error (`src/lib.rs:5732`) and cleanup-also-fails (`src/lib.rs:5770`), not entry.

The test lands before the documentation correction in Task 10, so the corrected wording describes behaviour that is already pinned.

**Files:**
- Modify: `src/lib.rs` — insert after `continuous_returns_closure_error_when_shutdown_also_fails` inside `mod tests::asynchronous`

- [ ] **Step 1: Write the test**

Insert immediately after `continuous_returns_closure_error_when_shutdown_also_fails` closes (currently `src/lib.rs:5801`):

```rust

        #[tokio::test]
        async fn continuous_skips_the_closure_and_cleanup_when_entry_fails() {
            use embedded_hal_async::i2c::Error as _;

            // The entry read-modify-write is the first thing
            // `continuous` does. If it fails, the closure never runs
            // and there is nothing to clean up — the chip was never
            // put into continuous mode.
            //
            // The single scripted transaction is the assertion:
            // `done()` fails if a closure body, or a cleanup
            // shutdown, issued anything further.
            let entry_err = embedded_hal::i2c::ErrorKind::Bus;

            let expectations = vec![
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]).with_error(entry_err),
            ];
            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            let mut closure_ran = false;
            let result = tmp108
                .continuous(async |_| {
                    closure_ran = true;
                    Ok(())
                })
                .await;

            assert_eq!(result.err().map(|e| e.kind()), Some(entry_err));
            assert!(!closure_ran, "the closure must not run when entry failed");

            let mut mock = tmp108.destroy();
            mock.done();
        }
```

- [ ] **Step 2: Run the test**

```bash
cargo test --locked -F async continuous
```

Expected: PASS, 3 tests (the two existing plus this one).

- [ ] **Step 3: Prove it bites**

Temporarily change `AsyncTmp108::continuous` (`src/lib.rs:2982-2995`) so the entry error does not short-circuit:

```rust
        let _ = self
            .inner
            .configuration()
            .modify_async(|r| r.set_m(Mode::Continuous))
            .await;
```

```bash
cargo test --locked -F async continuous_skips_the_closure_and_cleanup_when_entry_fails
```

Expected: FAIL — the closure runs, and `done()` reports unsatisfied expectations from the cleanup shutdown.

**Revert the production change.**

- [ ] **Step 4: Run the full suite**

```bash
cargo test --locked
cargo test --locked -F async
cargo test --locked -F async,embedded-sensors-hal-async
```

Expected: PASS.

- [ ] **Step 5: Format, lint and commit**

```bash
cargo +nightly fmt
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
```

Commit message file contents:

```
test: pin continuous()'s entry-failure path

An entry-phase error returns before the closure runs, so no cleanup
is attempted. The existing tests covered closure-error and
cleanup-also-fails but not entry.

Assisted-by: opencode:<your-model-id>
```

```bash
git add src/lib.rs
git commit -F "C:\Users\febalbi\AppData\Local\Temp\opencode\commitmsg.txt"
```

---

## Task 10: Correct the `continuous()` documentation

`src/lib.rs:2918` claims `continuous()` "unconditionally returns the chip to `Mode::Shutdown` ... regardless of whether the closure succeeded or failed". Three paths contradict that, and a fourth fact is missing.

Cancel-safety (`src/lib.rs:2922-2932`) is already documented correctly. **Leave it alone.**

**Files:**
- Modify: `src/lib.rs:2915-2921` (the opening paragraph)
- Modify: `src/lib.rs:2943-2951` (the `# Errors` section)

- [ ] **Step 1: Correct the opening paragraph**

Replace `src/lib.rs:2915-2921`:

```rust
    /// Initiate continuous conversions.
    ///
    /// Switches the chip into [`Mode::Continuous`], runs the user-supplied
    /// closure, and unconditionally returns the chip to [`Mode::Shutdown`]
    /// before returning, **regardless of whether the closure succeeded or
    /// failed**. This ensures the chip is not left burning current after
    /// a transient bus failure inside the closure.
```

with:

```rust
    /// Initiate continuous conversions.
    ///
    /// Switches the chip into [`Mode::Continuous`], runs the user-supplied
    /// closure, and then **attempts** to return the chip to
    /// [`Mode::Shutdown`], whether the closure succeeded or failed. This
    /// keeps the chip from burning current after a transient bus failure
    /// inside the closure.
    ///
    /// Attempts, not guarantees. Three paths skip or lose the cleanup:
    ///
    /// - If the *entry* transition into [`Mode::Continuous`] fails, the
    ///   closure never runs and no cleanup is attempted. The chip was
    ///   never switched, so there is nothing to undo.
    /// - If the closure panics, the unwind carries past the cleanup and
    ///   the chip is left converting. `no_std` builds using
    ///   `panic = "abort"` never reach this case.
    /// - If the cleanup write itself fails while the closure also
    ///   failed, the closure's error is what you get and the cleanup
    ///   error is dropped — so a caller cannot tell a confirmed
    ///   shutdown apart from a chip still converting.
    ///
    /// Even an accepted shutdown is not immediate: the conversion
    /// already in flight runs to completion before the part goes
    /// quiescent (SBOS663A §7.4.1).
```

- [ ] **Step 2: Sharpen the `# Errors` section**

Replace `src/lib.rs:2943-2951` — the current text describes the same three outcomes but calls the discarded cleanup result merely "discarded":

```rust
    /// # Errors
    ///
    /// - If the closure returns `Err(e)`, the cleanup `shutdown()` still
    ///   runs but its result is discarded; the closure's error is
    ///   returned.
    /// - If the closure returns `Ok(())` and the cleanup `shutdown()`
    ///   fails, that I2C error is returned.
    /// - If the initial transition into `Mode::Continuous` fails, the
    ///   closure is not invoked and the I2C error is returned.
```

with:

```rust
    /// # Errors
    ///
    /// - If the initial transition into [`Mode::Continuous`] fails, the
    ///   closure is not invoked and that I2C error is returned.
    /// - If the closure returns `Err(e)`, the cleanup `shutdown()` still
    ///   runs, but `e` is returned and any cleanup error is dropped.
    ///   A returned closure error therefore says nothing about whether
    ///   the chip reached shutdown.
    /// - If the closure returns `Ok(())` and the cleanup `shutdown()`
    ///   fails, that I2C error is returned. `Ok(())` is the only
    ///   result that confirms the shutdown write was accepted.
```

- [ ] **Step 3: Verify the docs build**

```bash
cargo doc --no-deps --all-features --locked
```

Expected: no warnings. A broken intra-doc link to `Mode::Continuous` or `Mode::Shutdown` fails here.

- [ ] **Step 4: Verify the doctest still runs**

The `# Examples` block below the edited sections is untouched, but confirm it still compiles and passes:

```bash
cargo test --doc --locked -F async continuous
```

Expected: PASS.

- [ ] **Step 5: Run the documentation build under every feature combination**

`continuous` is `async`-gated, and the repo builds docs under all combinations:

```bash
cargo doc --no-deps --locked
cargo doc --no-deps --locked -F async
cargo doc --no-deps --locked -F embedded-sensors-hal
cargo doc --no-deps --locked -F async,embedded-sensors-hal-async
```

Expected: no warnings from any.

- [ ] **Step 6: Format, lint and commit**

```bash
cargo +nightly fmt
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
```

Commit message file contents:

```
docs: stop promising continuous() unconditionally shuts the chip down

An entry-phase error skips cleanup entirely, a panic in the closure
unwinds past it, and a cleanup error is dropped when the closure also
failed. Documents all three, and that an accepted shutdown still lets
the in-flight conversion finish.

Assisted-by: opencode:<your-model-id>
```

```bash
git add src/lib.rs
git commit -F "C:\Users\febalbi\AppData\Local\Temp\opencode\commitmsg.txt"
```

---

## Task 11: Mirror the limit-register reset assertions on the async driver

`limit_registers_reset_to_the_documented_window` (`src/lib.rs:4844-4880`) exists only for the blocking driver. AGENTS.md requires parallel coverage across both flavors, and this is the same concern issue #66 raised as its item 2.

**Files:**
- Modify: `src/lib.rs` — insert at the top of `mod tests::asynchronous`, before `handle_a0_pin_accordingly` (currently `src/lib.rs:5434`)

- [ ] **Step 1: Write the test**

Insert immediately after the `use super::*;` at the top of `mod asynchronous` (currently `src/lib.rs:5431`) and before the `#[tokio::test]` attribute on `handle_a0_pin_accordingly`:

```rust

        /// The async mirror of
        /// `blocking::limit_registers_reset_to_the_documented_window`.
        ///
        /// The reset value is declared once in the DDSL and both
        /// drivers read it through the same generated register
        /// operation, so this cannot drift from its blocking twin
        /// independently — but AGENTS.md asks for both flavors to be
        /// covered, and a future change to `write_async` could.
        #[tokio::test]
        async fn limit_registers_reset_to_the_documented_window() {
            // Datasheet §7.5.4: THIGH = +127.9375 °C (0x7FF8) and
            // TLOW = -128 °C (0x8000), sent MSB first.
            //
            // Asserted against explicit wire bytes rather than against
            // `Fieldset::ZERO`, because comparing against ZERO is
            // precisely what let the wrong defaults through. The only
            // path where the reset value is observable is a `write()`
            // whose closure changes nothing.
            let expectations = vec![
                Transaction::write(0x48, vec![0x02, 0x80, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x7f, 0xf8]),
            ];
            let mock = Mock::new(&expectations);
            let mut tmp = AsyncTmp108::new_with_a0_gnd(mock);

            tmp.inner.t_low().write_async(|_| {}).await.unwrap();
            tmp.inner.t_high().write_async(|_| {}).await.unwrap();

            let mut mock = tmp.destroy();
            mock.done();
        }
```

- [ ] **Step 2: Run the test**

```bash
cargo test --locked -F async limit_registers_reset
```

Expected: PASS, 2 tests (blocking + async).

If the async one fails to compile on `write_async`, check the generated signature in `src/inner.rs` — it may need `.await` placed differently. Do not edit `src/inner.rs`.

- [ ] **Step 3: Run the full suite**

```bash
cargo test --locked
cargo test --locked -F async
cargo test --locked -F async,embedded-sensors-hal-async
```

Expected: PASS.

- [ ] **Step 4: Format, lint and commit**

```bash
cargo +nightly fmt
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
```

Commit message file contents:

```
test: mirror the limit-register reset assertions on the async driver

The reset-value test existed only for the blocking driver.

Assisted-by: opencode:<your-model-id>
```

```bash
git add src/lib.rs
git commit -F "C:\Users\febalbi\AppData\Local\Temp\opencode\commitmsg.txt"
```

---

## Task 12: Pin `Celsius`'s `Display` and ordering

`Celsius`'s `Display` impl (`src/lib.rs:327-335`) and its derived `Ord` are both public surface and both unasserted. `Ord` works today because the private representation is signed sixteenths, but nothing pins that.

The `Display` expectations below were produced by running `f32::from(sixteenths) * 0.0625` through the same format specifiers, not inferred.

**Files:**
- Modify: `src/lib.rs` — insert at the end of `mod tests::ops_tests::celsius`, after `unused_low_bits_are_discarded_not_truncated_toward_zero`

- [ ] **Step 1: Write the tests**

Insert immediately after `unused_low_bits_are_discarded_not_truncated_toward_zero` closes (currently `src/lib.rs:4019`), still inside `mod celsius`:

```rust

            #[test]
            fn display_renders_degrees_and_honours_precision() {
                // `Display` is where the value is finally allowed to
                // become a float, so it is worth pinning that it
                // renders degrees rather than sixteenths, and that a
                // precision specifier reaches the underlying `f32`.
                let c = Celsius::from_sixteenths(521).unwrap();
                assert_eq!(std::format!("{c}"), "32.5625");
                assert_eq!(std::format!("{c:.1}"), "32.6");
                assert_eq!(std::format!("{c:.2}"), "32.56");
                assert_eq!(std::format!("{c:.4}"), "32.5625");

                // Negative, and a fraction the decimal representation
                // holds exactly.
                let c = Celsius::from_sixteenths(-82).unwrap();
                assert_eq!(std::format!("{c}"), "-5.125");
                assert_eq!(std::format!("{c:.2}"), "-5.12");

                // The endpoints.
                assert_eq!(std::format!("{}", Celsius::MIN), "-128");
                assert_eq!(std::format!("{}", Celsius::MAX), "127.9375");
                assert_eq!(std::format!("{}", Celsius::ZERO), "0");

                // Sixteenths would render 521 here, not 32.5625.
                assert_ne!(std::format!("{}", Celsius::from_sixteenths(521).unwrap()), "521");
            }

            #[test]
            fn ordering_matches_temperature() {
                // `Ord` is derived on the private representation. It
                // agrees with temperature only because that
                // representation is signed sixteenths — nothing else
                // pins that, and a change to it would reorder every
                // `BTreeMap<Celsius, _>` in every downstream crate
                // without a compile error.
                assert!(Celsius::MIN < Celsius::ZERO);
                assert!(Celsius::ZERO < Celsius::MAX);
                assert!(Celsius::MIN < Celsius::MAX);

                // Across zero, where an unsigned representation would
                // disagree.
                let below = Celsius::try_from_degrees(-0.0625).unwrap();
                let above = Celsius::try_from_degrees(0.0625).unwrap();
                assert!(below < Celsius::ZERO);
                assert!(Celsius::ZERO < above);

                // Monotonic across the whole domain.
                let mut previous = Celsius::MIN;
                for c in all().skip(1) {
                    assert!(previous < c, "{previous:?} should sort below {c:?}");
                    assert!(
                        previous.to_degrees() < c.to_degrees(),
                        "{previous:?} should be colder than {c:?}"
                    );
                    previous = c;
                }
                assert_eq!(previous, Celsius::MAX);
            }
```

- [ ] **Step 2: Run the tests**

The crate is `#![no_std]`, but `mod tests` is compiled with `std` linked and reachable by path — `mod timeline` already writes `use std::sync::{Arc, Mutex};` at `src/lib.rs:4532`. `std::format!` therefore resolves without an `extern crate std;`, and the fully-qualified form matches the existing convention in the file. Do not add a bare `use std::format;`.

```bash
cargo test --locked celsius
```

Expected: PASS. `ordering_matches_temperature` walks all 4096 values; that is fast.

- [ ] **Step 3: Prove the Display test bites**

Temporarily change `Display for Celsius` (`src/lib.rs:332-334`) to render sixteenths:

```rust
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            core::fmt::Display::fmt(&self.0, f)
        }
```

```bash
cargo test --locked display_renders_degrees_and_honours_precision
```

Expected: FAIL.

**Revert the production change.**

- [ ] **Step 4: Run the full suite**

```bash
cargo test --locked
cargo test --locked -F async
cargo test --locked -F async,embedded-sensors-hal-async
```

Expected: PASS.

- [ ] **Step 5: Format, lint and commit**

```bash
cargo +nightly fmt
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
```

Commit message file contents:

```
test: pin Celsius's Display and ordering

Both are public surface and neither was asserted. Ord agrees with
temperature only because the representation is signed sixteenths;
a change to it would reorder downstream collections silently.

Assisted-by: opencode:<your-model-id>
```

```bash
git add src/lib.rs
git commit -F "C:\Users\febalbi\AppData\Local\Temp\opencode\commitmsg.txt"
```

---

## Task 13: Full verification

Everything is committed. Run the complete local matrix from AGENTS.md before handing the branch back.

- [ ] **Step 1: Format check on nightly**

```bash
cargo +nightly fmt --check
```

Expected: no output. Stable `fmt` silently ignores `rustfmt.toml`'s unstable options; CI runs nightly.

- [ ] **Step 2: Clippy**

```bash
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
```

Expected: no output.

- [ ] **Step 3: Documentation**

```bash
cargo doc --no-deps --locked
cargo doc --no-deps --locked -F async
cargo doc --no-deps --locked -F embedded-sensors-hal
cargo doc --no-deps --locked -F async,embedded-sensors-hal-async
```

Expected: no warnings from any.

- [ ] **Step 4: Unit tests**

```bash
cargo test --locked
cargo test --locked -F async
cargo test --locked -F async,embedded-sensors-hal-async
```

Expected: PASS.

- [ ] **Step 5: Doctests**

```bash
cargo test --doc --locked
cargo test --doc --locked -F async
cargo test --doc --locked -F async,embedded-sensors-hal-async
```

Expected: PASS.

- [ ] **Step 6: Examples**

```bash
cargo build --examples --locked
cargo build --examples --locked -F async
cargo build --examples --locked -F embedded-sensors-hal
cargo build --examples --locked -F async,embedded-sensors-hal-async
```

Expected: success, no warnings.

- [ ] **Step 7: Feature powerset**

```bash
cargo hack --feature-powerset check --locked
```

Expected: all 6 combinations succeed.

- [ ] **Step 8: README snippets**

```bash
./scripts/check-readme-snippets.sh
```

Expected: no drift. Nothing in this plan edits an example, so this should be untouched — a failure means something unrelated drifted.

- [ ] **Step 9: Supply chain**

```bash
cargo vet --locked
```

Expected: success. No dependencies were added, so this should pass unchanged.

- [ ] **Step 10: Confirm each commit builds independently**

Bisectability is a stated requirement. Check the last twelve commits each build:

```bash
git rebase --exec "cargo check --all-features --locked" HEAD~12
```

Expected: no failures. If the rebase stops, fix the offending commit with `git rebase --continue` after amending — do not add a follow-up fix commit.

- [ ] **Step 11: Review the branch**

```bash
git log --oneline origin/config-read-acknowledgement..HEAD
git diff --stat origin/config-read-acknowledgement..HEAD
```

Expected: 13 commits (the spec plus twelve implementation commits), touching only `src/lib.rs` and `docs/superpowers/specs/`.

**No hardware run is required.** Nothing here touches alert-pin behaviour, and the delay assertions are against fakes by construction — a real clock would make the ordering test slower without making it stronger.

---

## Notes for the implementer

**Line numbers drift.** Every `src/lib.rs:NNNN` in this plan is from the baseline `8fbde659d724`. After Task 1 removes nine lines and Task 2 adds some, later references shift. Locate code by the quoted text, not by the number.

**Expect tests to pass on first run.** This is a coverage-gap plan, not a bugfix plan. Almost every test characterises behaviour that is already correct. That is why several tasks include an explicit "prove it bites" step where you temporarily break the implementation and confirm the new test fails — without that, a vacuous test looks identical to a real one.

**Always revert the temporary breakage.** Steps that say "revert the production change" mean it. Run `git diff` before committing and confirm only the intended files and hunks are staged.

**Do not fix the hysteresis asymmetry.** Task 7 pins it as measured and files a follow-up. Changing `snap_hysteresis` in this branch would be a behavioural change the issue does not ask for, and Part 2 of #66 warns specifically against reshaping things that merely look wrong.

**Do not touch `src/inner.rs`, `Cargo.toml`'s version, or `CHANGELOG.md`.**

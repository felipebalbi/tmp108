# TMP108 issue #59: alert-wait ordering design

**Date:** 2026-09-24  
**Status:** proposed  
**Related issues:** #59 (in scope), #58 (partial), #65 / #62 (out of scope)  
**Baseline:** branch `fixes`, HEAD `74745f6`

This document specifies a fix, not an implementation. It does not authorize changes to public API, dependencies, feature definitions, or release versions. The maintainer owns version and release decisions.

## 1. Problem statement

`AlertTmp108::wait_for_temperature_threshold()` starts by reading configuration to select its thermostat-mode and ALERT-polarity branch (`src/lib.rs:2010`). That read is not side-effect-free observation: in interrupt mode it acknowledges the very event the caller wants to receive.

The authoritative hardware source for this design is the caller-supplied, previously audited quotation from **TI TMP108 datasheet SBOS663A §7.5.3.4**:

> "the SMBus ALERT Response only clears the pin and not the flags. Reading the configuration register clears both the flags and the pin."

The entry read therefore obtains useful evidence and simultaneously clears the chip's latched evidence. However, `ops::decode_config` (`src/lib.rs:379-386`) projects the returned register onto the 64 combinations of the four public `Config` settings. It discards FL, FH, and M. The interrupt branches subsequently wait for a falling or rising edge without either the original pin assertion or the flags returned by the entry read.

When an interrupt was already latched at entry, this ordering loses the notification. If the excursion has ended, no further assertion is required to occur, so the call can remain pending indefinitely.

The supplied Pico de Gallo hardware observations establish this failure:

- With ALERT already asserted LOW and the excursion ongoing, the call returned after 34.9181 ms.
- With ALERT already asserted LOW but the excursion ended, the call timed out after 6.0152275 s.
- The apparent success in the ongoing case is a subsequent conversion relatching the alert, not delivery of the original event.
- Issue #65's consecutive configuration reads returning `FH=1`, `FH=0`, `FH=0` establish why another read cannot recover the evidence consumed by the first.

The current post-edge configuration read also discards flags, and the final temperature comment incorrectly describes the returned value as the temperature "at time of trigger." Those observations inform the #58 scope decision in section 8.

The hardware facts supplied with the task are accepted as authoritative. No datasheet download or independent full datasheet audit was performed. There is no `sources/datasheet.txt` artifact or source hash to claim.

## 2. Chosen design

### 2.1 Overview

**Read and consume once, retain the evidence, then decide whether waiting is necessary.**

One configuration-register transaction supplies both the settings and FL/FH. In interrupt mode:

- If either flag is set in the entry snapshot, the call has already observed an event. Skip every GPIO operation and skip any further configuration acknowledgment. Read the temperature register and return.
- If neither flag is set, await the asserted pin **level**, acknowledge that subsequent assertion with one configuration read, then read the temperature register and return.

This is not the insufficient fix "replace edge waits with level waits." Preserving and checking the flags consumed by the entry read is essential for the reported transient. The level wait additionally closes the separate race in which a new alert asserts after the entry read but before GPIO waiting is armed.

"Return immediately" on the pending-event path means no GPIO wait and no conversion-period delay. The external trait still requires a temperature value, so that path includes one fallible temperature-register transaction.

Comparator mode retains its existing level-based behavior:

- Read configuration.
- Await the asserted level.
- Read temperature.
- Do not perform a post-wait configuration acknowledgment.

Comparator flags must not activate the interrupt-mode fast path.

### 2.2 Settings freshness and ownership

There is no configuration cache, event queue, background task, or new persistent wrapper state.

Constructors and `AsyncTmp108::into_alert()` remain synchronous, infallible, and I/O-free.

Every wait call obtains TM and POL from its own entry configuration snapshot. Reconfiguration between calls through `sensor_mut()`, direct `configure()`, or a delegated hysteresis operation cannot make a cache stale because no cache exists.

Existing serialized-device assumptions continue to apply:

- No independent owner may change thermostat mode or polarity during a pending wait.
- No independent owner may acknowledge the same device during a pending wait.
- Holding `&mut self` prevents simultaneous operations through this wrapper but does not prevent access through an independent I2C handle, another master, or an SMBus Alert Response actor.

A configuration read performed before this call by another driver operation may already have cleared a transient. This fix cannot recover evidence destroyed before its entry snapshot. That is part of the remaining #65 boundary.

### 2.3 Transaction definitions and wire bytes

Let `addr` be the existing sensor address. The examples below use A0 tied to GND, so `addr = 0x48`.

Define these operations:

**C0 — entry configuration snapshot**

```text
I2c::write_read(addr, &[0x01], &mut configuration_bytes)
```

The receive buffer is exactly two bytes. Preserve the returned settings and flags in one `ops::AlertSnapshot`. In interrupt mode, this transaction also acknowledges the chip's flags and pin.

**C1 — post-wait interrupt acknowledgment**

```text
I2c::write_read(addr, &[0x01], &mut configuration_bytes)
```

This is the same transaction shape as C0, but it occurs only after a successful interrupt-mode pin wait. It acknowledges a subsequent assertion. It is not a write, read-modify-write, or SMBus Alert Response operation.

**T — latest temperature**

```text
I2c::write_read(addr, &[0x00], &mut temperature_bytes)
```

The receive buffer is exactly two bytes. Use the existing temperature conversion path, `Celsius::from_register`, and return `to_degrees()` through the external trait.

The configuration fieldset uses little-endian bit numbering over the received bytes. Temperature bytes use the existing MSB-first temperature codec.

These representative configuration responses use `Mode::Continuous`, `ConversionRate::OneHz`, and `Hysteresis::OneC`:

| Thermostat | Polarity | No flags | FL only | FH only | FL and FH |
|---|---|---|---|---|---|
| Interrupt | ActiveLow | `[0x26, 0x10]` | `[0x2e, 0x10]` | `[0x36, 0x10]` | `[0x3e, 0x10]` |
| Interrupt | ActiveHigh | `[0x26, 0x90]` | `[0x2e, 0x90]` | `[0x36, 0x90]` | `[0x3e, 0x90]` |
| Comparator | ActiveLow | `[0x22, 0x10]` | `[0x2a, 0x10]` | `[0x32, 0x10]` | `[0x3a, 0x10]` |
| Comparator | ActiveHigh | `[0x22, 0x90]` | `[0x2a, 0x90]` | `[0x32, 0x90]` | `[0x3a, 0x90]` |

Representative temperature responses:

- `[0x19, 0x00]` represents 25 °C.
- `[0x50, 0x00]` represents 80 °C.

The table defines mock fixtures, not a requirement to reconfigure the chip to those settings. The implementation accepts every existing legal setting combination.

No branch adds a delay, reads a limit register, writes a register, or performs any configuration read beyond the explicitly listed C0 and optional C1.

Map I2C failures to `Error::Bus` and pin failures to `Error::Pin`. Stop at the first failure without executing later operations.

### 2.4 Branch: `(Thermostat::Interrupt, Polarity::ActiveLow)`

1. Perform C0: write `[0x01]` and read two configuration bytes.
2. Decode TM, POL, FL, and FH from that one result.
3. If `snapshot.low || snapshot.high`:
   1. Perform no pin operation.
   2. Do not perform C1.
   3. Perform T: write `[0x00]` and read two temperature bytes.
   4. Return the decoded latest temperature.
4. Otherwise call `self.alert.wait_for_low().await` exactly once.
   - If ALERT became LOW between C0 and arming the wait, accept the already-active level.
   - If ALERT remains HIGH, wait until it becomes LOW.
   - Do not call `wait_for_falling_edge`.
5. After a successful level wait, perform C1 exactly once.
6. Perform T and return the decoded latest temperature.

Representative pending path:

```text
write_read(0x48, [0x01]) -> [0x36, 0x10]
write_read(0x48, [0x00]) -> [0x19, 0x00]
return Ok(25.0)
```

Representative no-pending path:

```text
write_read(0x48, [0x01]) -> [0x26, 0x10]
pin.wait_for_low() -> Ok(())
write_read(0x48, [0x01]) -> [0x36, 0x10]
write_read(0x48, [0x00]) -> [0x50, 0x00]
return Ok(80.0)
```

### 2.5 Branch: `(Thermostat::Interrupt, Polarity::ActiveHigh)`

1. Perform C0: write `[0x01]` and read two configuration bytes.
2. Decode TM, POL, FL, and FH from that one result.
3. If `snapshot.low || snapshot.high`:
   1. Perform no pin operation.
   2. Do not perform C1.
   3. Perform T: write `[0x00]` and read two temperature bytes.
   4. Return the decoded latest temperature.
4. Otherwise call `self.alert.wait_for_high().await` exactly once.
   - If ALERT became HIGH between C0 and arming the wait, accept the already-active level.
   - If ALERT remains LOW, wait until it becomes HIGH.
   - Do not call `wait_for_rising_edge`.
5. After a successful level wait, perform C1 exactly once.
6. Perform T and return the decoded latest temperature.

Representative pending path:

```text
write_read(0x48, [0x01]) -> [0x36, 0x90]
write_read(0x48, [0x00]) -> [0x19, 0x00]
return Ok(25.0)
```

Representative no-pending path:

```text
write_read(0x48, [0x01]) -> [0x26, 0x90]
pin.wait_for_high() -> Ok(())
write_read(0x48, [0x01]) -> [0x36, 0x90]
write_read(0x48, [0x00]) -> [0x50, 0x00]
return Ok(80.0)
```

### 2.6 Branch: `(Thermostat::Comparator, Polarity::ActiveLow)`

1. Perform C0: write `[0x01]` and read two configuration bytes.
2. Decode settings. Do not use FL/FH to short-circuit this branch.
3. Call `self.alert.wait_for_low().await` exactly once.
   - Return from the pin wait immediately if the pin is already LOW.
   - Otherwise wait for LOW.
   - Do not add a deassertion wait or change to an edge wait.
4. Perform T: write `[0x00]` and read two temperature bytes.
5. Return the decoded latest temperature.
6. Do not perform C1.

Representative sequence:

```text
write_read(0x48, [0x01]) -> [0x22, 0x10]
pin.wait_for_low() -> Ok(())
write_read(0x48, [0x00]) -> [0x50, 0x00]
return Ok(80.0)
```

### 2.7 Branch: `(Thermostat::Comparator, Polarity::ActiveHigh)`

1. Perform C0: write `[0x01]` and read two configuration bytes.
2. Decode settings. Do not use FL/FH to short-circuit this branch.
3. Call `self.alert.wait_for_high().await` exactly once.
   - Return from the pin wait immediately if the pin is already HIGH.
   - Otherwise wait for HIGH.
   - Do not add a deassertion wait or change to an edge wait.
4. Perform T: write `[0x00]` and read two temperature bytes.
5. Return the decoded latest temperature.
6. Do not perform C1.

Representative sequence:

```text
write_read(0x48, [0x01]) -> [0x22, 0x90]
pin.wait_for_high() -> Ok(())
write_read(0x48, [0x00]) -> [0x50, 0x00]
return Ok(80.0)
```

### 2.8 Interpretation of C1

C1 is an acknowledgment after a successful level wait. It must never overwrite the C0 snapshot before the entry pending test.

A successful asserted-level wait is sufficient evidence for the post-wait path under this proposal. Do not add a loop that requires C1's flags to be nonzero before completing. Such a loop would introduce a new qualification policy beyond #59.

The existing external trait cannot return flag identity. C1's flags are therefore intentionally not exposed in this scope. They must not be represented as proof that the returned temperature identifies a particular threshold crossing.

If both entry flags are set, complete once. Do not prioritize one flag, emit two results, or infer event ordering.

### 2.9 Successful-operation invariants

1. A nonzero interrupt flag in successful C0 prevents all GPIO waiting in that invocation.
2. Current temperature and current pin level do not invalidate a pending interrupt captured in C0.
3. The pending interrupt path performs exactly C0 and T.
4. The no-pending interrupt path performs exactly C0, one asserted-level wait, C1, and T.
5. No configuration reads occur while waiting for GPIO.
6. Comparator paths perform exactly C0, one asserted-level wait, and T.
7. A new interrupt that relatches after C0 on the pending fast path is left for a subsequent call; there is no trailing acknowledgment to consume it.
8. The result is a latest conversion value, not a trigger-time measurement.
9. The implementation does not promise an event count, event ordering, exactly-once delivery, or durable receipt.

## 3. New and changed `mod ops` functions

### 3.1 Preserve the existing settings-only decoder

Keep this existing signature and behavior unchanged:

```rust
pub(crate) fn decode_config(c: Configuration) -> Config;
```

It continues to return:

- `thermostat_mode` from `c.tm()`;
- `alert_polarity` from `c.pol()`;
- `conversion_rate` from `c.cr()`;
- `hysteresis` from `c.hys()`.

Do not add fields to public `Config`. In particular, status flags are not configurable settings.

### 3.2 Add `ops::AlertSnapshot`

Add this crate-private type in the existing private codec module:

```rust
#[cfg(feature = "embedded-sensors-hal-async")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct AlertSnapshot {
    pub(crate) config: Config,
    pub(crate) low: bool,
    pub(crate) high: bool,
}
```

Semantics:

- `config` is the settings projection from one configuration-register read.
- `low` is the FL value returned by that same read.
- `high` is the FH value returned by that same read.
- The flags describe the returned snapshot, not the chip's flag values after the destructive transaction completes.
- The two booleans are independent. All four combinations are representable.
- The type does not contain a temperature, timestamp, event count, or decoded `Mode`.
- It is not reexported or made part of public API.

### 3.3 Add the pure snapshot decoder

Exact signature:

```rust
#[cfg(feature = "embedded-sensors-hal-async")]
pub(crate) fn decode_alert_snapshot(c: Configuration) -> AlertSnapshot;
```

Exact semantic definition:

```text
AlertSnapshot {
    config: decode_config(c),
    low: c.fl(),
    high: c.fh(),
}
```

`Configuration` is `Copy`, so extracting the flags and passing the fieldset to `decode_config` requires no further I2C access.

The function must be pure, total, allocation-free, and independent of sync/async HAL traits.

Use the generated accessors rather than introducing duplicated bit-decoding logic in the driver shell.

The pending predicate is simply:

```rust
snapshot.low || snapshot.high
```

No separate predicate helper is needed for this expression.

### 3.4 Register-bit provenance

At baseline `74745f6`:

| Field | DDSL declaration | Generated accessor | Meaning in received bytes |
|---|---|---|---|
| M | `tmp108.ddsl:23-30`, bits `1:0` | `src/inner.rs:277-286`, `m()` | `bytes[0] & 0x03` |
| TM | `tmp108.ddsl:31-37`, bit `2` | `src/inner.rs:287-296`, `tm()` | `bytes[0] & 0x04` |
| FL | `tmp108.ddsl:38-39`, bit `3` | `src/inner.rs:297-306`, `fl()` | `bytes[0] & 0x08 != 0` |
| FH | `tmp108.ddsl:40-41`, bit `4` | `src/inner.rs:307-316`, `fh()` | `bytes[0] & 0x10 != 0` |
| POL | `tmp108.ddsl:62-68`, bit `15` | `src/inner.rs:347-356`, `pol()` | `bytes[1] & 0x80` |

`tmp108.ddsl:2` declares the default byte order as LE. The generated `Configuration` metadata at `src/inner.rs:271-274` carries that byte order, and `Configuration::from([u8; 2])` stores the supplied bytes directly at `src/inner.rs:435-439`.

The new decoder does not call `m()`. It must accept a snapshot with M=`0b11` without entering the generated fallible `Mode` decoder. This avoids coupling #59 to the separate #62 defect.

### 3.5 Private I/O helper on `AlertTmp108`

Keep I/O outside `mod ops`.

Add a private helper on the existing feature-gated `AlertTmp108` impl:

```rust
async fn read_alert_snapshot(
    &mut self,
) -> Result<ops::AlertSnapshot, I2C::Error>;
```

Its operation is:

1. Access the inner sensor through `self.sensor_mut()`.
2. Read `inner.configuration().read_async().await` exactly once.
3. Pass the resulting `Configuration` to `ops::decode_alert_snapshot`.
4. Return that snapshot or the original I2C error.

The existing code is in one module, so this can use the inner sensor's private register machinery without publishing `inner` or adding a method to either bare driver.

The helper must document internally that it is a destructive read and acknowledgment. It is not a nondestructive status query.

Use the helper for C0 and C1. The waiter maps its errors to `Error::Bus`. Use `self.sensor_mut().temperature()` for T.

### 3.6 Generated code and feature gating

No register-layout change is needed. TM, POL, FL, and FH already have generated accessors.

Do not modify `tmp108.ddsl` or `src/inner.rs` for this fix. Never hand-edit generated code. A future register-layout correction must change DDSL and regenerate with:

```text
ddc build rust -s tmp108.ddsl -o src/inner.rs
```

The installed generation tool is ddc 2.1.0.

Gate the new snapshot type, decoder, relevant imports, wrapper helper, and associated tests consistently with `embedded-sensors-hal-async`, which already implies `async`.

The existing settings decoder remains available for the blocking driver and every legal feature combination.

## 4. Alternatives considered and rejected

### 4.1 Consume flags, check them, then use an edge wait

This is the issue author's suggested direction and correctly identifies the entry-read defect.

Accept the consume-and-check portion: retain FL/FH from the same transaction that acknowledges them, and complete without GPIO waiting when either was set.

Reject a bare edge wait for the no-pending tail because it leaves a second race:

1. C0 returns with FL/FH clear.
2. A conversion asserts ALERT.
3. The driver arms an edge wait after that transition.
4. ALERT remains asserted, but the edge wait requires another transition.

A level wait after the flags test handles an assertion that occurs in this interval.

### 4.2 Only replace edge waits with level waits

Reject.

By the time the current code arms its wait, the entry read has already cleared the interrupt pin and discarded the flags. If the excursion ended, a level wait then blocks just as reliably as an edge wait.

The flag-preserving snapshot and pending test are mandatory. Level waiting alone is not the fix.

### 4.3 Cache TM and POL at `into_alert()`

Reject for this fix.

`AsyncTmp108::into_alert()` is currently synchronous, infallible, and I/O-free. Reading the actual chip configuration during that operation would require changing its contract or introducing a separate constructor.

Using defaults is not a valid cache initialization strategy: the chip can already be configured differently before wrapping.

Even a correctly initialized cache can become stale through:

- `sensor_mut()` followed by direct `configure()`;
- delegated sensor operations;
- configuration changes through other device access paths;
- failed writes whose hardware effects are uncertain.

Caching also does not eliminate the need to preserve pending flags if a configuration read is used to initialize or refresh it.

### 4.4 Cache with invalidation on `sensor_mut()`

Reject.

Invalidating on every mutable escape could avoid trusting some stale values, but the next wait would still need a flag-preserving configuration refresh.

The design would add invalidation state, constructor initialization policy, failed-write policy, and lifecycle bookkeeping without improving correctness over a single fresh snapshot per call.

The hysteresis setter does not inherently change TM or POL, but it illustrates the delegated configuration paths that a cache policy must consider. Its destructive reads remain part of #65.

### 4.5 Add a constructor that accepts caller-supplied settings

Reject for #59.

A separate constructor could be additive, but it would make cache correctness the caller's obligation and would not fix existing constructors and existing wait calls.

There is no need to enlarge public API to repair this internal ordering defect.

### 4.6 Sample `InputPin` before an edge wait

Reject as the chosen implementation.

Before C0, the driver does not yet have trustworthy polarity. After C0, sampling cannot reconstruct a transient whose pin and flags have just been cleared.

Even after the pending-flags test, `is_low()` or `is_high()` followed by an edge wait creates a sample-to-arm race. It merely moves the lost-wakeup window.

The level-wait contract already includes returning for an already-active level. It is the appropriate primitive.

Keep the existing public `InputPin` bound unchanged, but do not call its methods in the new waiter.

### 4.7 Arm an edge future before reading configuration

Reject.

Constructing an async future does not necessarily arm hardware. A correct implementation would require a polling/selection protocol, additional cancellation reasoning, and careful ownership of both bus and pin.

It would still need to preserve FL/FH when acknowledging an already-latched event.

That complexity is unnecessary when a snapshot followed by a level wait provides the required behavior.

### 4.8 Poll configuration until flags appear

Reject.

Repeated configuration polling adds destructive bus traffic, conversion-rate or polling-delay policy, and potential interference with other users of status. It worsens the class of side effects tracked by #65.

The wrapper already owns a GPIO intended to support waiting.

### 4.9 Read temperature before acknowledging

Reject as a complete solution.

The entry configuration read is itself an acknowledgment, so moving T ahead of C1 cannot prevent acknowledgment before T on the pending-entry path.

Reading T before C0 does not provide a trigger-time sample either, and would add unnecessary temperature reads when no event is pending.

### 4.10 Retain acknowledged events in persistent wrapper state

Defer to a separate delivery-policy design.

Persistent pending state could help with retries after T errors or some cancellation points, but it introduces:

- replay versus consumption policy;
- coalescing rules for additional flags;
- interactions with `sensor_mut()`;
- lifecycle semantics for `into_inner()` and `destroy()`;
- cancellation during the destructive read itself;
- decisions about whether subsequent calls report an old event or await a new one.

These are not private implementation details of the successful-operation ordering fix.

### 4.11 Add a public status-bearing API now

Defer to #58.

A receipt containing flags and a separately fallible temperature sample is useful, but it requires an explicit public error and event-consumption contract. It is not necessary to fix #59 behind the existing trait.

## 5. Breaking-change verdict

**BREAKING: no.**

The selected fix fits behind the existing external trait signature:

```rust
async fn wait_for_temperature_threshold(
    &mut self,
) -> Result<embedded_sensors_hal_async::temperature::DegreesCelsius, Self::Error>;
```

The associated error remains:

```rust
Error<I2C::Error, <ALERT as embedded_hal::digital::ErrorType>::Error>
```

The proposal does not change:

- public method signatures;
- public type names;
- public generic bounds;
- constructors or their synchronous/infallible behavior;
- the external trait implementation's associated types;
- fields of public `Config`;
- variants of public `Error`;
- feature definitions or implications;
- public reexports.

The new snapshot type and decoder are crate-private. The new I/O helper is private to the wrapper implementation.

`cargo semver-checks check-release` should therefore see **no additional public-API incompatibility caused by this fix**. This is an assessment of the proposed delta, not a claim that the command has been run or that unrelated changes since the last published release will pass.

The behavior is observably corrected: an already-latched interrupt now completes, and a new assertion occurring before GPIO arming is no longer required to produce another edge. This needs release documentation, but not a caller signature migration.

No public additive API is proposed in this scope. A later inherent method returning a new status-bearing receipt could be additive-only, a minor API addition, provided it preserves existing public signatures, associated types, bounds, and exhaustive enums. That later API needs its own assessment.

The maintainer owns version changes. This document does not propose a `Cargo.toml` version bump.

## 6. Mirroring matrix

| Surface | Proposed change | Why mirroring is or is not required |
|---|---|---|
| `Tmp108` | None | No new bare-driver operation is introduced. There is no blocking ALERT wrapper or blocking implementation of this async GPIO wait protocol. |
| `AsyncTmp108` | None | Existing temperature and register machinery suffice. No new public or private bare-driver method, cache, or constructor behavior is needed. |
| `AlertTmp108` | Add private `read_alert_snapshot`; change trait wait ordering; correct behavior and failure documentation | This is an async-only wrapper-specific acknowledgment protocol. A blocking mirror would be a new feature, not maintenance of the existing mirrored driver shells. |
| `mod ops` | Add crate-private `AlertSnapshot` and pure `decode_alert_snapshot`; preserve `decode_config` | Chip-specific register projection stays in the shared codec. The wrapper contains I/O and sequencing, not duplicated bit decoding. |
| `tmp108.ddsl` | None | Existing fields already describe the needed bits. |
| `src/inner.rs` | None | Existing generated accessors suffice. Generated code remains read-only. |
| Public `Config`, `Error`, and reexports | None | Existing external construction, matching, and trait usage remain valid. |

The omission of bare-driver mirrors is deliberate. The new helper belongs to `AlertTmp108`, not to only one of the two parallel bare-driver shells.

If a later status-reading method is added to a bare driver as public functionality, provide blocking and async mirrors sharing the pure decoder. That is not part of this fix.

All six legal feature combinations must remain green:

1. No features.
2. `embedded-sensors-hal`.
3. `async`.
4. `async,embedded-sensors-hal`.
5. `embedded-sensors-hal-async`, which implies `async`.
6. `embedded-sensors-hal,embedded-sensors-hal-async`, which also implies `async`.

Use the existing async-sensors feature gates for the wrapper-specific code and new codec items. Introduce no dependency, allocation, `std` requirement, or new feature relationship.

Preserve every public name pinned by `tests/reexports.rs`, including `Tmp108`, feature-gated `AsyncTmp108`, `Mode`, `Thermostat`, `ConversionRate`, `Hysteresis`, and `Polarity`. The baseline test also pins `Celsius` and `OutOfRange`; preserve those as well.

The design maintains the type-split spec's private pure codec and thin I/O shell boundary. It does not reintroduce macros, a shared bus abstraction, or a configuration-state framework.

## 7. Testability

### 7.1 Test organization and mock requirements

The automated suite must run without hardware.

Place wrapper regression tests in the existing `tests::asynchronous` module under the async-sensors feature. Place pure decoder tests in `tests::ops_tests` with matching feature gating.

Use existing I2C transaction-list mocks and a pin fake implementing:

- `embedded_hal::digital::ErrorType`;
- `embedded_hal::digital::InputPin`;
- `embedded_hal_async::digital::Wait`.

No new test dependency is required.

For ordering-sensitive cases, separate I2C and pin expectation lists are not enough by themselves: they cannot prove that C1 happened after pin completion. Use a shared operation log or a gated pin fake to assert the cross-interface sequence.

The fake's InputPin methods should fail on unexpected calls. The selected implementation never samples InputPin.

### 7.2 Transient regression: already latched, excursion ended

Initial modeled state:

- Thermostat: Interrupt.
- Polarity: ActiveLow.
- Mode: Continuous.
- Conversion rate: OneHz.
- Hysteresis: OneC.
- FH is latched.
- ALERT is asserted LOW.
- The temperature excursion has already ended.
- No future alert assertion or falling edge will ever occur.
- The latest temperature is now 25 °C.

Construction performs no I2C traffic.

The exact successful I2C transaction list is:

```rust
[
    Transaction::write_read(
        0x48,
        vec![0x01],
        vec![0x36, 0x10],
    ),
    Transaction::write_read(
        0x48,
        vec![0x00],
        vec![0x19, 0x00],
    ),
]
```

The first response has Interrupt mode and FH set. The second response is the latest 25 °C sample.

C0's hardware effect releases ALERT from LOW to HIGH and clears FL/FH.

The expected pin-operation list is empty:

- no `wait_for_low`;
- no `wait_for_high`;
- no falling-edge wait;
- no rising-edge wait;
- no any-edge wait;
- no InputPin reads.

A strict fake that panics on every pin call makes the regression deterministic without waiting for a timeout.

Assertions:

1. The call returns `Ok(25.0)`.
2. Exactly C0 and T are consumed.
3. There is no C1.
4. No pin method is called.
5. No threshold read, register write, or delay is performed.
6. Returning an in-band temperature does not cause the event to be discarded or the waiter to start waiting again.

**Why this fails against today's code:** today's C0 decodes only settings and discards FH. The code then invokes `wait_for_falling_edge`. The strict fake fails immediately at that unexpected operation, before T is reached.

For an additional literal liveness test, wrap the transaction-list I2C mock with a small test adapter that updates shared chip/pin state when C0 completes:

- The pin starts LOW with FH latched.
- C0 returns FH=1 and changes the modeled pin to HIGH.
- Every later wait remains Pending because there will be no further assertion.
- I2C transactions themselves are immediately ready.

Manually poll the overall future. The corrected waiter must produce `Ready(Ok(25.0))` without entering a pin wait. Today's waiter produces Pending at the falling-edge wait. This variant models the destructive side effect directly; the strict no-pin-call variant is sufficient as the primary regression gate.

Do not schedule a later falling edge in the transient fixture. Doing so recreates the persistent-condition masking behavior.

Repeat the strict test for:

- FL only: C0 `[0x2e, 0x10]`.
- FL and FH: C0 `[0x3e, 0x10]`.
- ActiveHigh FH: C0 `[0x36, 0x90]`.
- ActiveHigh FL: C0 `[0x2e, 0x90]`.
- ActiveHigh both: C0 `[0x3e, 0x90]`.

For ActiveHigh, the pin begins HIGH and C0 releases it to LOW. The expected pin-operation list remains empty. Both flags still produce one successful result, not two.

### 7.3 Persistent interrupt: already latched

Use the same initially pending active-low interrupt, but keep the temperature condition true and return 80 °C.

Exact I2C list:

```rust
[
    Transaction::write_read(
        0x48,
        vec![0x01],
        vec![0x36, 0x10],
    ),
    Transaction::write_read(
        0x48,
        vec![0x00],
        vec![0x50, 0x00],
    ),
]
```

Expected pin operations: none.

Assertions:

1. Return `Ok(80.0)`.
2. Complete using the entry FH evidence.
3. Do not wait for a new conversion or another assertion.
4. Do not perform C1.

This ensures the persistent condition is not allowed to mask the original ordering bug.

Mirror the case with ActiveHigh C0 `[0x36, 0x90]`.

### 7.4 Interrupt: no pending entry flag, later assertion

ActiveLow exact I2C list:

```rust
[
    Transaction::write_read(
        0x48,
        vec![0x01],
        vec![0x26, 0x10],
    ),
    Transaction::write_read(
        0x48,
        vec![0x01],
        vec![0x36, 0x10],
    ),
    Transaction::write_read(
        0x48,
        vec![0x00],
        vec![0x50, 0x00],
    ),
]
```

Required combined operation order:

```text
C0
wait_for_low starts
wait_for_low remains pending while pin is HIGH
test asserts pin LOW and wakes waiter
wait_for_low completes
C1
T
return Ok(80.0)
```

Assertions:

- Exactly one level wait occurs.
- No edge wait occurs.
- C1 cannot occur before the pin wait succeeds.
- Exactly one post-wait acknowledgment occurs.
- T follows C1.
- The result is `Ok(80.0)`.

For ActiveHigh, replace both configuration response second bytes with `0x90`, use `wait_for_high`, start the pin LOW, and assert it HIGH.

Add a case in which C1 returns clear flags after the successful pin wait. Under the chosen contract, the waiter still performs T and returns. It does not loop or require a nonzero C1 flag to requalify the pin notification.

### 7.5 Interrupt: assertion between C0 and GPIO arming

Use the same I2C list as section 7.4.

The pin fake models:

1. C0 returns no flags.
2. An interrupt asserts the pin before the first poll of the chosen level wait.
3. No further transition ever occurs.

For ActiveLow, `wait_for_low` must complete for the already-LOW pin. For ActiveHigh, `wait_for_high` must complete for the already-HIGH pin.

Assertions:

- Completion does not require another edge.
- C1 and T occur after the level wait completes.
- The result is `Ok(80.0)`.

A fake edge wait should remain Pending in this state. This test rejects the consume-then-edge-wait variant even though that variant passes the already-pending transient test.

### 7.6 Comparator ActiveLow

Exact I2C list:

```rust
[
    Transaction::write_read(
        0x48,
        vec![0x01],
        vec![0x22, 0x10],
    ),
    Transaction::write_read(
        0x48,
        vec![0x00],
        vec![0x50, 0x00],
    ),
]
```

Required pin operation: exactly one `wait_for_low`.

Test two pin behaviors:

1. Already LOW: the level wait completes immediately.
2. Initially HIGH: the level wait remains Pending until the test drives LOW.

Assertions:

- Return `Ok(80.0)` after the level wait completes.
- No edge wait.
- No C1.
- No InputPin call.
- No added wait for deassertion.

Repeat two calls while the pin stays LOW. Both must complete after their own C0 and T transactions, preserving current comparator behavior.

### 7.7 Comparator ActiveHigh

Exact I2C list:

```rust
[
    Transaction::write_read(
        0x48,
        vec![0x01],
        vec![0x22, 0x90],
    ),
    Transaction::write_read(
        0x48,
        vec![0x00],
        vec![0x50, 0x00],
    ),
]
```

Required pin operation: exactly one `wait_for_high`.

Test:

1. Already HIGH: immediate level-wait completion.
2. Initially LOW: Pending until the test drives HIGH.

Assertions:

- Return `Ok(80.0)`.
- No edge wait.
- No C1.
- No InputPin call.
- No added wait for deassertion.

Repeat calls while the pin remains HIGH to preserve immediate-level semantics.

For both comparator polarities, additionally supply entry flags with an inactive modeled pin. Assert that the driver still invokes the level wait rather than taking the interrupt pending-flags fast path. This is a branch-semantics test: comparator flags do not substitute for the chosen pin predicate.

### 7.8 Freshness and consecutive-call tests

Test reconfiguration between calls through `sensor_mut().configure(...)`, with the existing exact RMW transaction expectations.

For a change from the representative comparator active-low configuration to interrupt active-high:

```rust
[
    Transaction::write_read(
        0x48,
        vec![0x01],
        vec![0x22, 0x10],
    ),
    Transaction::write(
        0x48,
        vec![0x01, 0x26, 0x90],
    ),
]
```

The next waiter C0 must use the newly returned TM/POL, not remembered constructor settings. Supply `[0x36, 0x90]` to that C0 and assert completion through the pending interrupt path with no pin call.

Also test two successive pending interrupt calls:

- First C0 reports FH.
- First call performs T without C1.
- A later C0 reports FH again for a new latched condition.
- Second call also completes without GPIO waiting.

This protects the invariant that no trailing acknowledgment on the first call consumes an event intended for the next call.

### 7.9 Pure decoder tests

Exhaust all 65,536 possible `[u8; 2]` fieldset patterns.

For each input:

- `snapshot.config` equals `decode_config(Configuration::from(bytes))`.
- `snapshot.low` equals `bytes[0] & 0x08 != 0`.
- `snapshot.high` equals `bytes[0] & 0x10 != 0`.

Include M=`0b11` patterns explicitly in test intent. The decoder must not call or unwrap `m()`.

This pins the new decoder's totality and independence from #62. It does not claim to fix the generated `Mode` decoder.

### 7.10 Error and cancellation tests

Inject failures separately at:

- C0;
- ActiveLow level wait;
- ActiveHigh level wait;
- C1;
- T on the pending-entry path;
- T on the post-wait interrupt path;
- T on comparator paths.

Assert exact error payload preservation and no subsequent operations:

- I2C error becomes `Error::Bus`.
- Pin error becomes `Error::Pin`.
- Pin error does not trigger cleanup acknowledgment.
- C1 error prevents T.
- T error does not trigger retry or an additional configuration read.

For pending interrupt followed by T failure:

1. C0 returns FH=1.
2. T returns an I2C error.
3. The call returns `Error::Bus`.
4. A second call sees clear flags and an inactive pin.
5. That second call remains Pending; no replay is promised.

For cancellation:

1. Let C0 return pending flags.
2. Hold T Pending.
3. Drop the waiter future.
4. Assert that no cleanup configuration read is performed.
5. Treat the original event as potentially lost, consistent with section 9.

Update the existing edge-oriented mocks in:

- `handle_threshold_alerts_properly`;
- `alert_pin_error_is_propagated_as_error_pin`.

Preserve their temperature and error assertions, but do not treat these updated tests as substitutes for the dedicated transient regression.

### 7.11 Verification expectations

Implementation must run the canonical AGENTS.md verification matrix. In particular:

```text
cargo hack --feature-powerset check --locked
```

must remain green across all six legal feature combinations.

Also run the required formatting, clippy, unit-test, doctest, example-build, documentation, snippet-synchronization, supply-chain, and semver checks.

The automated suite remains mock-only. Separately, AGENTS.md requires human hardware verification before shipping an alert-behavior change. Rerun:

- the reported transient/persistent A/B;
- interrupt behavior on the actual Pico de Gallo HAL;
- comparator warm/cool behavior.

No implementation test execution or hardware verification is claimed by this design document.

## 8. Scope decision on #58

**Decision: partially address #58 alongside #59. Include the short documentation fix; defer the medium status-bearing API and durable delivery policy.**

### 8.1 Included short fix

Replace the false trigger-time comment at `src/lib.rs:2042`.

The documented contract must state:

- The method returns the latest temperature conversion read after observing an alert.
- That value need not equal the triggering temperature.
- The value can be back inside the configured band when a historical interrupt is delivered.
- The returned temperature cannot reliably identify whether FL, FH, or both caused the event.
- Reading configuration acknowledges the interrupt before the temperature transaction completes.

Make this visible in wrapper/waiter documentation, not only a private implementation comment.

The README's alert snippet also prints "Temperature at trigger." Correct that wording in the example's source marker region, then synchronize the README snippet according to the repository's snippet contract. These are requirements for the later implementation, not permission to edit files during the design-only task.

### 8.2 Internal flags are not a public status API

This fix gives the waiter access to FL/FH before its wait decision. That is necessary to repair #59.

It does not expose flag identity to callers:

- The existing trait returns only `DegreesCelsius` on success.
- C1's flags remain unreported.
- Both flags may be set.
- Hardware can coalesce multiple excursions.
- A later temperature read cannot reconstruct the original threshold direction.

Keep #58 open. Internally preserving the entry evidence is not equivalent to implementing its medium API proposal.

### 8.3 Temperature-read failure after acknowledgment

If T fails after C0 or C1 has acknowledged an interrupt, the existing trait returns `Err(Error::Bus(...))` and the caller does not receive the event evidence.

The selected fix does not solve this residual failure.

A second configuration read cannot recover the acknowledged flags. Retrying the trait method may wait for a completely different event. An error must not be interpreted as proof that no alert occurred.

This failure already exists in today's post-edge acknowledgment path. The corrected pending-entry path has the same limitation: a successful acknowledgment followed by a failed temperature read cannot be represented as both event success and temperature failure by the existing scalar success result.

Document and test the limitation rather than silently inventing:

- a trigger temperature;
- a substitute threshold value;
- an automatic retry policy;
- hidden event replay;
- a new public error variant.

### 8.4 Deferred medium fix

A separate design should consider an additive inherent API returning a receipt that preserves low/high evidence independently of the temperature-read result.

That design must settle:

- acknowledgment and receipt ownership;
- whether temperature is optional or separately fallible;
- retry and replay behavior;
- cancellation before and after acknowledgment;
- coalescing of later flags;
- behavior through `sensor_mut()`, `into_inner()`, and destruction.

Such an API can be additive-only if it adds a new method and receipt type without changing the existing trait or existing public types. Changing the existing trait-associated error, adding variants to an exhaustive public enum, or adding required fields to public `Config` would need a separate breaking-change assessment.

Deferring that API keeps #59 a bounded ordering repair with no public migration.

## 9. Open questions and risks

### 9.1 Prior pending-edge assumption is explicitly superseded

The prior reliability design, `docs/superpowers/specs/2026-06-03-tmp108-reliability-fixes-design.md`, section H1 at lines 115-127, attributes an inter-call pending-edge guarantee to `embedded_hal_async::digital::Wait`.

The current wrapper documentation at `src/lib.rs:731-737` repeats that claim.

The installed dependency's actual contract does not support relying on that guarantee.

The inspected source is:

```text
C:\Users\febalbi\.cargo\registry\src\index.crates.io-1949cf8c6b5b557f\embedded-hal-async-1.0.0\src\digital.rs
```

For `wait_for_rising_edge`, lines 35-39 state:

> "Wait for the pin to undergo a transition from low to high."

> "If the pin is already high, this does *not* return immediately, it'll wait for the pin to go low and then high again."

For `wait_for_falling_edge`, lines 41-45 state:

> "Wait for the pin to undergo a transition from high to low."

> "If the pin is already low, this does *not* return immediately, it'll wait for the pin to go high and then low again."

By contrast, lines 21-33 specify:

> "Wait until the pin is high. If it is already high, return immediately."

> "Wait until the pin is low. If it is already low, return immediately."

Those level-wait methods also require a completed observation to remain reportable if the pin changes back before the awakened task runs.

This proposal explicitly supersedes the prior H1 conclusion. It does not silently assume that an edge wait queues every transition occurring before it is armed.

Later implementation must replace the incorrect wrapper rustdoc and mark the prior H1 decision as superseded with a reference to this design. Preserve unrelated prior decisions, including the type split, pure codec boundary, error mapping, and comparator repeated-level behavior.

### 9.2 Read-to-arm correctness depends on the level-wait contract

The driver relies on `wait_for_low` and `wait_for_high` correctly handling an already-active pin and safely arranging continued observation when it is inactive.

A faulty HAL could implement its own check-then-arm race. An extra driver-side InputPin sample would not repair that.

Mock tests establish the driver's ordering and primitive choice. Human verification must exercise the actual Pico de Gallo HAL implementation.

### 9.3 Configuration-read sampling versus hardware clearing

The supplied facts establish that a read returns already-latched flags and clears the flags and pin. They do not fully specify a conversion racing the chip's internal flag-sampling and clearing point during the transaction.

Do not claim atomic snapshot-and-clear relative to conversion completion merely because the bus operation is one I2C transaction.

The design addresses:

- events pending when captured in C0;
- new assertions after C0 and before GPIO arming;
- subsequent assertions while awaiting the level.

If an event were both absent from the returned snapshot and cleared internally before leaving any observable pin state, software could not reconstruct it from this interface. Additional hardware evidence would be required to make a stronger guarantee about that boundary.

This is a primary reliability-review target, not a reason to discard the confirmed entry-ordering fix.

### 9.4 Failure and cancellation contract

The waiter is **not event-delivery cancel-safe**.

| Failure point | Required result | Residual uncertainty |
|---|---|---|
| C0 fails | `Error::Bus`; no GPIO or T | The bus error may occur after the chip has already acknowledged. Software cannot assume flags survived. |
| GPIO wait fails | `Error::Pin`; no C1 or T | A later event may still be pending; no cleanup read may consume it. |
| C1 fails | `Error::Bus`; no T | The pin was observed active, but acknowledgment completion is uncertain. |
| T fails after interrupt acknowledgment | `Error::Bus`; no retry | Event evidence may be irrecoverable to the caller. |
| Future is dropped | No asynchronous cleanup | A completed destructive read may have consumed an event whose local snapshot is then lost. In-flight I2C cancellation inherits the bus HAL's contract. |

Applications requiring durable event delivery need a separate receipt/replay design. Awaiting the future to completion avoids intentional mid-operation cancellation but does not eliminate bus failures.

**Scope approval needed:** confirm that this documented limitation is acceptable while the #58 medium API remains deferred. If durable receipt is required now, the work is larger than the selected ordering fix.

### 9.5 Comparator preservation

Comparator behavior is intentionally level-driven, not a historical interrupt queue.

Reviewers should reject:

- a comparator fast path based only on FL/FH;
- a post-wait comparator acknowledgment;
- a new wait for deassertion;
- changing comparator waits to edge waits;
- filtering a completed notification by comparing the latest temperature with freshly read limits.

A comparator level wait can complete before T is sampled. An in-band temperature at T is not grounds to pretend the preceding pin observation never occurred.

### 9.6 Competing acknowledgments and shared ALERT wiring

The design retains the dedicated ALERT and serialized-device assumptions.

An independent configuration reader or SMBus Alert Response participant can clear evidence outside the waiter's control. An unrelated device sharing a wired interrupt line can assert that line without a TMP108 threshold event.

This proposal does not implement shared-interrupt demultiplexing or multi-owner acknowledgment arbitration.

### 9.7 Event coalescing and persistent conditions

FL/FH do not provide an event count or ordered history.

Multiple excursions can coalesce into one snapshot, and a persistent condition can cause later relatching. Do not promise one return per physical excursion or that repeated calls necessarily correspond to different temperature crossings.

On the pending fast path, leaving a newly relatched event unacknowledged for the next call is intentional. An extra trailing configuration read would reintroduce event loss.

### 9.8 #65 boundary

The waiter's specific destructive-read misuse is repaired:

- C0's flags now drive control flow.
- An already-pending event no longer triggers a redundant second acknowledgment.
- No constructor reads or polling loops are added.

Configuration-read counts do not increase:

- Pending interrupt path: one configuration read instead of the old entry-plus-post-wait pattern.
- No-pending interrupt path: two configuration reads, as before.
- Comparator path: one configuration read, as before.

The other destructive configuration entry points and their documentation remain out of scope. An event they consume before this waiter starts remains unrecoverable by this fix.

### 9.9 #62 boundary

The generated `Mode` decoder rejects M=`0b11`, even though the task identifies that encoding as valid continuous mode.

The selected implementation avoids that path:

- `Configuration::from` stores bytes without decoding M.
- `decode_config` does not call `m()`.
- `decode_alert_snapshot` does not call `m()`.

Do not opportunistically alter DDSL or generated Mode handling in this fix. Exhaustive snapshot decoder tests must include M=`0b11` so future refactoring does not accidentally introduce a dependency on the unresolved decoder.

### 9.10 Review and handoff

The design is ready for coordinator and reliability review, not an assertion that implementation is complete.

The reviewer should specifically attack:

1. Preservation of the entry snapshot before any second destructive read.
2. The no-pending read-to-arm race under both polarities.
3. The internal hardware read/clear timing boundary.
4. Cancellation and bus failure after acknowledgment.
5. Comparator branch isolation.
6. Mock fixtures that accidentally schedule a future edge and mask the transient defect.
7. Documentation that promises trigger-time temperature or a stronger edge contract than the dependency supplies.

No unresolved naming or API question blocks a minimal implementation. The substantive scope decision is whether the documented non-durable delivery behavior is acceptable until #58 receives its separate status-bearing design.

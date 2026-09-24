# TMP108 issue #67: observable alert cause and retained delivery

**Date:** 2026-09-24  
**Status:** proposed; design-only handoff for maintainer review  
**Baseline:** `fixes`, `e4a3607`; published compatibility baseline is 0.6.0  
**Related issues:** #67 (in scope), #58 / #59 (contracts preserved), #65 (future status boundary)  
**Audience:** maintainer, implementer, and reliability reviewer

## 1. Context and problem statement

The driver already observes information it cannot report. The private
`ops::AlertSnapshot { config, low, high }` decodes FL/FH, but
`wait_for_temperature_threshold` returns only a temperature. Its interrupt
fast path uses the flags to qualify a notification, then forgets which
flags were present. Retention after sample failure preserves only a boolean
delivery obligation.

This is missing expressiveness, not another lost-wakeup fix. Preserve the
ordering established by #59 and the retained delivery established by #58.

In particular:

> A successful interrupt level wait followed by a successful C1
> acknowledgment qualifies delivery even when C1 returns zero flags.
> Direction is information about that observation, never an additional
> qualification test.

The new API must preserve known direction, explicitly represent unknown
direction, and never infer direction from the returned temperature.

### 1.1 Hardware grounding

The authoritative hardware source is the
[TI TMP108 datasheet, SBOS663A](https://www.ti.com/lit/gpn/tmp108):

- **§7.5.3.4, Temperature Watchdog Flags:** FL/FH describe the comparison
  performed at the end of conversions. Exceeding THIGH sets FH; falling
  below TLOW sets FL.
- **§7.5.3.4, interrupt acknowledgment:** “Reading the configuration
  register clears both the flags and the pin.”
- **Table 8:** FH is bit 4 and FL is bit 3 of the first configuration byte.
- **§7.5.2:** the temperature register stores the output of the most recent
  conversion, not a trigger-time sample.
- **§7.5.4:** comparator assertion follows the threshold/hysteresis
  condition; interrupt assertion remains latched until acknowledgment or
  another documented clearing action.

The existing audit extraction was inspected at:

```text
C:/Users/febalbi/AppData/Local/Temp/opencode/tmp108-audit/sources/datasheet.txt
```

Its relevant anchors are:

| Fact | Extracted source |
|---|---|
| Latest-conversion temperature register | `sources/datasheet.txt:816-823` |
| FL/FH bit positions | `sources/datasheet.txt:882-889` |
| Flag comparison and interrupt clearing | `sources/datasheet.txt:919-927` |
| Comparator hysteresis and interrupt latch | `sources/datasheet.txt:979-990` |

Here, `sources/datasheet.txt` denotes the existing audit artifact, not an
in-tree file. Its manifest records revision SBOS663A and PDF SHA-256:

```text
ec086250fc4331e7fc923be62173062bbf0fccedad6894c2741b73cd1c084556
```

The maintainer’s #67 brief additionally establishes that both flags can be
latched together. Issue #65’s measured `FH=1, FH=0, FH=0` under a persistent
over-temperature condition confirms that reading destroys the evidence.
The post-fix #59 transient experiment returned an in-band latest sample
while delivering a genuine interrupt.

This document uses that existing extraction and issue evidence. It does
not claim a new multi-document compliance audit or new hardware
measurements. The interrupt-mode latch/clear interpretation must not be
generalized into comparator history.

### 1.2 Relevant implementation boundaries

At baseline `e4a3607`:

| Concern | Source |
|---|---|
| Public settings projection | `src/lib.rs:91-112` |
| Private snapshot and decoder | `src/lib.rs:388-422` |
| Wrapper retention and recovery contract | `src/lib.rs:762-977` |
| Destructive snapshot helper | `src/lib.rs:1209-1227` |
| Continuous entry and cleanup | `src/lib.rs:1747-1817` |
| Existing acquisition, arm, sample, clear | `src/lib.rs:2267-2352` |
| Compatibility test baseline | `tests::asynchronous::alert_ordering`, `src/lib.rs:3803-5889` |

The implementation remains within the existing private pure-codec and
async-wrapper boundaries. No new bus abstraction, blocking ALERT wrapper,
configuration cache, or background task is needed.

## 2. Chosen API

Accept the issue’s sketch:

- `AlertCause::{BelowLow, AboveHigh, Both, Unknown}`;
- `AlertEvent { cause, temperature: Celsius }`;
- additive inherent `AlertTmp108::wait_for_alert`;
- the existing `Error<I2C::Error, ALERT::Error>`.

Do not add a `Config` field or a new error variant.

### 2.1 Public types and shipping rustdoc

The following declarations and rustdoc are the intended public surface.
`AlertCause` is declared at the crate root with **no `#[cfg]` attribute**.
`AlertEvent` uses the explicit gate shown below.

````rust
/// Direction information available for an observed TMP108 alert.
///
/// The named directions report FL/FH returned by the configuration read
/// servicing an interrupt. They do not describe the current temperature.
///
/// In Interrupt mode, the flags record threshold excursions since they
/// were last observed and cleared, rather than a live temperature
/// comparison. Reading configuration clears them and releases ALERT;
/// reset also limits the retained history. See TMP108 datasheet SBOS663A,
/// sections 7.5.3.4 and 7.5.4.
///
/// This is an observation result, not the raw flag pair: `Unknown` means
/// direction is unavailable, not that no alert occurred. Multiple
/// excursions can coalesce into one observation. No count, order, or
/// trigger-time sample is supplied.
///
/// # Examples
///
/// ```
/// use tmp108::AlertCause;
///
/// let description = match AlertCause::Both {
///     AlertCause::BelowLow => "FL observed",
///     AlertCause::AboveHigh => "FH observed",
///     AlertCause::Both => "FL and FH observed",
///     AlertCause::Unknown => "direction unavailable",
/// };
/// assert_eq!(description, "FL and FH observed");
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AlertCause {
    /// FL was set and FH was clear in the servicing interrupt snapshot.
    BelowLow,
    /// FH was set and FL was clear in the servicing interrupt snapshot.
    AboveHigh,
    /// FL and FH were both set in the servicing interrupt snapshot.
    ///
    /// This does not establish the number or order of excursions.
    Both,
    /// An alert was observed, but its direction cannot be established.
    ///
    /// Fresh comparator observations always report this. Interrupt
    /// observations report it when the post-wait acknowledgment returns
    /// both flags clear. It is not an error and does not mean "no alert".
    Unknown,
}

/// An alert observation paired with a subsequently read temperature.
///
/// The fields are not an atomic measurement. The cause records the
/// evidence captured while servicing an interrupt, if available; the
/// temperature is the latest conversion available when the sample
/// transaction runs (TMP108 datasheet SBOS663A, sections 7.5.3.4 and 7.5.2).
///
/// A retained cause can describe an earlier interrupt even if another
/// interrupt has since latched in the opposite direction. The returned
/// sample need not support the retained direction: it may be inside the
/// limits or beyond the opposite limit. This is expected, not grounds to
/// replace the cause or reject the event.
///
/// In Comparator mode the observation is of an asserted level, not
/// necessarily a new crossing. Repeated calls can report the same
/// continuously asserted condition.
///
/// # Examples
///
/// ```
/// use tmp108::{AlertCause, AlertEvent, Celsius};
///
/// let event = AlertEvent {
///     cause: AlertCause::AboveHigh,
///     temperature: Celsius::try_from_degrees(25.0).unwrap(),
/// };
/// // Historical direction and a later sample are separate facts.
/// assert_eq!(event.cause, AlertCause::AboveHigh);
/// assert_eq!(event.temperature.sixteenths(), 400);
/// ```
#[cfg(feature = "embedded-sensors-hal-async")]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct AlertEvent {
    /// Direction evidence captured while servicing the alert, if available.
    pub cause: AlertCause,
    /// Latest available conversion read after observing the alert.
    ///
    /// This is not a trigger-time sample. A retained delivery reads this
    /// at retry time; neither conversion age nor time since the crossing
    /// is bounded. Do not infer direction from this value or discard an
    /// alert because this value is back inside the configured limits.
    pub temperature: Celsius,
}
````

These types have no fallible operations, so they require no `# Errors`
section.

### 2.2 Inherent method and shipping rustdoc

The method belongs in an inherent implementation with this gate and the
existing bounds:

```rust
#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C, ALERT> AlertTmp108<I2C, ALERT>
where
    I2C: embedded_hal_async::i2c::I2c,
    ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin,
```

The following is the intended method declaration and complete rustdoc.
The declaration’s semicolon specifies the interface; the implementation
body is defined by sections 3–6.

````rust
/// Deliver a retained interrupt, or observe ALERT, then read temperature.
///
/// Available with the `embedded-sensors-hal-async` feature. Unlike
/// `TemperatureThresholdWait::wait_for_temperature_threshold`, this
/// inherent method returns direction evidence as well as a [`Celsius`]
/// sample. It does not configure the chip or start conversions.
///
/// With no retained delivery, one configuration read selects mode and
/// polarity. In Interrupt mode, nonzero entry flags qualify the alert
/// immediately, without GPIO waiting or another acknowledgment.
/// Otherwise this method waits for the asserted pin level, then
/// acknowledges with one configuration read. Nonzero acknowledgment
/// flags supply direction; zero flags still deliver an event with
/// [`AlertCause::Unknown`].
///
/// In Comparator mode, this method waits for the asserted level and
/// reports [`AlertCause::Unknown`]. Entry flags do not establish direction
/// for the later level observation. There is no post-wait configuration
/// read. An already-asserted comparator level completes the wait
/// immediately; repeated calls need not represent distinct crossings.
///
/// In Interrupt mode, FL/FH describe threshold excursions since the flags
/// were last observed and cleared, subject to reset, not what is true at
/// the moment this method returns. Reading configuration consumes those
/// flags and releases ALERT (TMP108 datasheet SBOS663A, sections 7.5.3.4
/// and 7.5.4). Both flags can be reported together; their order and the
/// number of excursions are not known.
///
/// The temperature is the latest available conversion, not the
/// temperature at the crossing (SBOS663A, section 7.5.2). Known direction
/// comes only from FL/FH, never from comparing the sample with limits.
///
/// # Retained delivery
///
/// After a successful interrupt acknowledgment, the wrapper retains the
/// cause before awaiting temperature, including `Unknown` when
/// appropriate. Temperature failure or cancellation leaves it pending.
/// The next call to either this method or the threshold-wait trait reads
/// temperature only, without configuration or GPIO access. Retrying
/// preserves the original cause but obtains a new sample.
///
/// A retained cause can describe an earlier interrupt even if another
/// interrupt has since latched in the opposite direction. The returned
/// sample need not support the retained direction: it may be inside the
/// limits or beyond the opposite limit. This is expected, not grounds to
/// replace the cause or reject the event.
///
/// This method and the threshold-wait trait are two views of one
/// consumptive stream, not two subscribers. Either may retain an
/// obligation; successful delivery through either consumes it. A
/// successful trait delivery deliberately discards cause, and a later
/// call to this method cannot retrieve that consumed cause. If a trait
/// attempt fails or is cancelled during its temperature read, the
/// retained cause remains available to this method.
///
/// Only a successful temperature read settles a retained obligation.
/// This method does not return a partially successful event on a sample
/// error. No internal retries or asynchronous drop cleanup are performed.
///
/// Retention survives direct sensor access and reconfiguration, including
/// switching to Comparator mode. Direct temperature reads do not consume
/// it. Decomposing or dropping the wrapper abandons it without driver I/O;
/// a new wrapper starts empty. Fresh comparator observations never create
/// a retained obligation.
///
/// # Cancellation and ownership
///
/// This future is **not event-delivery cancel-safe during configuration
/// reads**: hardware may acknowledge before I2C reports success. Failure
/// or cancellation there can lose evidence before it can be retained.
/// HAL-specific bus and GPIO cancellation/recovery guarantees still
/// apply. There is no durable queue or exactly-once guarantee per
/// physical excursion.
///
/// Callers must provide retry bounds or backoff on a failing bus and
/// backoff for repeated comparator observations. An immediately failing
/// bus can make repeated retained-delivery attempts return errors without
/// yielding.
///
/// Use a dedicated ALERT pin and serialize device access. Do not change
/// mode or polarity, or acknowledge through an independent device
/// handle, during a pending acquisition. See [`AlertTmp108`] for
/// phase-specific recovery and lifecycle details.
///
/// # Errors
///
/// [`Error::Bus`] preserves an I2C error from configuration or temperature;
/// [`Error::Pin`] preserves a GPIO-wait error. This method does not
/// produce [`Error::InvalidInput`].
///
/// A bus error neither identifies the failing phase nor proves that no
/// interrupt occurred or that its flags survived. After successful
/// interrupt acknowledgment, a failed sample retains delivery on this
/// wrapper; failure during acknowledgment need not do so.
///
/// # Examples
///
/// ```
/// # tokio::runtime::Runtime::new().unwrap().block_on(async {
/// # use embedded_hal_mock::eh1::{digital, i2c::{Mock, Transaction}};
/// use tmp108::{AlertCause, AlertTmp108};
/// # let i2c = Mock::new(&[
/// #     Transaction::write_read(0x48, vec![0x01], vec![0x36, 0x10]),
/// #     Transaction::write_read(0x48, vec![0x00], vec![0x19, 0x00]),
/// # ]);
/// # let alert = digital::Mock::new(&[]);
///
/// let mut sensor = AlertTmp108::new_with_a0_gnd(i2c, alert);
/// // Here the chip has an already-latched high interrupt.
/// let event = sensor.wait_for_alert().await.unwrap();
/// assert_eq!(event.cause, AlertCause::AboveHigh);
/// assert_eq!(event.temperature.sixteenths(), 400); // latest sample: 25 C
/// # let (mut i2c, mut alert) = sensor.destroy();
/// # i2c.done();
/// # alert.done();
/// # });
/// ```
pub async fn wait_for_alert(
    &mut self,
) -> Result<AlertEvent, Error<I2C::Error, ALERT::Error>>;
````

### 2.3 Why `Unknown` belongs in this enum

The objection is substantive: three variants describe observed chip
flags, while one describes missing knowledge.

The resolution is to define the enum as **the driver’s available direction
information for an already-qualified alert**, not as the chip’s entire
status register. These are four mutually exclusive observation outcomes.

`Option<KnownAlertCause>`, where `KnownAlertCause` has three variants, is
equally sound and has the same four inhabitants. It is not rejected on
correctness grounds. Prefer the flat enum because `Unknown` makes the
meaning explicit at the matching site; `None` can too easily be read as
“no alert.”

A separate `known: bool` alongside a direction would introduce
contradictory combinations. A generic knowledge/provenance framework does
not earn its complexity here.

Crucially, `Unknown` is **not** the universal representation of a successful
raw register read containing zero flags. Section 5 preserves that
distinction for #65.

### 2.4 Why `AlertEvent` does not carry `Config`

Omit it.

The bytes are free on C0, but a coherent public promise is not:

- the slow path has settings from both C0 and C1;
- comparator C0 precedes the notification;
- retained delivery has no new configuration observation;
- settings observed at acknowledgment need not be settings at the
  historical crossing;
- `Config` omits threshold registers and Mode, so it would not fully
  describe the crossing even if retained.

Retaining a historical `Config` is technically possible without additional
I/O, but would require selecting and documenting its provenance.
An optional field would expose path-dependent metadata, while a
non-optional historical field would invite mistaken interpretation as
current settings.

Neither solves a stated #67 use case. Settings belong in a
configuration-snapshot contract, not this alert-and-sample result.

### 2.5 Temperature, extensibility, and naming

**Use `Celsius`, not `f32` or `DegreesCelsius`.** The inherent API should
preserve the crate’s 4096-value representation. Conversion to
`to_degrees()` belongs at the existing foreign-trait boundary. No new
conversion error is needed.

**Neither new type is `#[non_exhaustive]`.** The cause domain is
deliberately closed. A raw zero-flag observation is not a future fifth
cause. Exhaustive matching makes callers address `Unknown` explicitly.
The event is a small value pair that callers may construct in tests and
destructure normally.

Adding metadata later would require a deliberate API revision. That is
preferable to weakening current ergonomics for unspecified future fields,
especially in a small `no_std` driver at 0.x. No stable layout, size, or ABI
representation is promised.

**Keep `wait_for_alert`.** It distinguishes the GPIO/latch observation
from the conversion-period delay in `wait_for_temperature` and the
scalar-only foreign trait `wait_for_temperature_threshold`. “Wait”
includes immediate completion and retained delivery, as the existing
method already does. Documentation must make clear that an `AlertEvent`
need not represent a distinct edge.

### 2.6 Exact feature gates

| New item | Exact gate |
|---|---|
| Root `pub enum AlertCause` | None |
| Root `pub struct AlertEvent` | `#[cfg(feature = "embedded-sensors-hal-async")]` |
| Inherent impl containing `wait_for_alert` | `#[cfg(feature = "embedded-sensors-hal-async")]` |
| Private `ops::interrupt_alert_cause` helper | `#[cfg(feature = "embedded-sensors-hal-async")]` |
| Helper-specific imports and pure tests | `#[cfg(feature = "embedded-sensors-hal-async")]` |
| Event, waiter, and fake-world integration tests | `#[cfg(feature = "embedded-sensors-hal-async")]` |
| Root cause compile pins and cause doctest | None |

The existing feature definition already makes
`embedded-sensors-hal-async` imply `async`; redundant conjunctions are not
necessary for new items.

An ungated root-public enum does **not** create a `dead_code` problem:
it is externally reachable public API, and its unit variants are public
constructors. It does not need internal producers to be a valid exported
type. Users can construct, match, store, and test values in a no-feature
build. Its derives require only `core`, and its rustdoc must not link to
feature-gated items without appropriate gating.

The trade-off is intentional: no current no-feature driver method produces
`AlertCause`, but the concept is HAL-independent and can be reused by a
future blocking status API without depending on the GPIO-wrapper feature.
Do not add a dummy constructor, artificial internal usage, or
`allow(dead_code)` to justify it.

This arrangement is compatible by construction with all six legal feature
combinations. `cargo hack --feature-powerset check --locked` remains a
mandatory implementation gate; this design does not claim that unimplemented
API has already passed that command.

## 3. Acquisition protocol and retained state

Define:

- **C0:** the entry two-byte configuration read at pointer `0x01`;
- **C1:** the same read after an interrupt level wait;
- **T:** the two-byte temperature read at pointer `0x00`.

Addresses, byte order, register codecs, and transaction counts remain
unchanged.

### 3.1 Pure interpretation of qualified interrupt flags

Use a total, allocation-free helper in `mod ops`:

```rust
#[cfg(feature = "embedded-sensors-hal-async")]
pub(crate) fn interrupt_alert_cause(low: bool, high: bool) -> AlertCause;
```

Its exact mapping is:

| FL | FH | Cause |
|---|---|---|
| 0 | 0 | `AlertCause::Unknown` |
| 1 | 0 | `AlertCause::BelowLow` |
| 0 | 1 | `AlertCause::AboveHigh` |
| 1 | 1 | `AlertCause::Both` |

The helper interprets flags **for an already-qualified notification**.
Converting a random empty snapshot to `Unknown` must not create an event.

Keep `AlertSnapshot` and `decode_alert_snapshot` private. Their raw
booleans remain appropriate: all four flag pairs are legal at the register
boundary. Do not add duplicated bit decoding to the I/O shell or invoke
the generated Mode decoder.

### 3.2 Path-to-cause table

Rows describe successful T. Failure behavior is specified in section 6.

“Known mapping” means FL-only → `BelowLow`, FH-only → `AboveHigh`,
both → `Both`.

| Path | Exact operations | Reported cause | Why |
|---|---|---|---|
| Retained `Some(cause)` | T only | Exactly stored `cause`, including `Unknown` | Already-acknowledged evidence belongs to this delivery, not to current registers or mode. |
| Fresh Interrupt fast path, either polarity | Nonzero C0, T | Known mapping of C0 | C0 captures and acknowledges the qualifying latch; no GPIO or C1 is needed. |
| Fresh Interrupt slow path, nonzero C1, either polarity | Clear C0, asserted-level wait, C1, T | Known mapping of C1 | Report flags obtained while servicing the notification, without claiming an ordered or one-to-one edge history. |
| Fresh Interrupt slow path, zero C1, either polarity | Clear C0, asserted-level wait, C1, T | `Unknown` | The successful level wait qualifies the notification; zero C1 cannot invalidate it. |
| Fresh Comparator ActiveLow, any C0 flags | C0, `wait_for_low`, T | `Unknown` | C0 predates the qualifying level observation; no associated post-wait flags are observed. |
| Fresh Comparator ActiveHigh, any C0 flags | C0, `wait_for_high`, T | `Unknown` | Same rule; polarity selects asserted level, not threshold direction. |

A retained known interrupt delivered after switching to Comparator uses
the first row. “Comparator reports Unknown” applies to **fresh comparator
acquisition**, not every call made while the chip happens to be in
Comparator mode.

### 3.3 Exact replacement for the boolean

Replace the private field, retaining its name:

```rust
interrupt_sample_pending: Option<AlertCause>,
```

This explicitly represents the reliability review’s five required states:

| Semantic state | Representation |
|---|---|
| Empty | `None` |
| Pending, direction unknown | `Some(AlertCause::Unknown)` |
| Pending, FL observed | `Some(AlertCause::BelowLow)` |
| Pending, FH observed | `Some(AlertCause::AboveHigh)` |
| Pending, both observed | `Some(AlertCause::Both)` |

**Agree with reliability hazard 1:** Empty and Pending Unknown must never
be collapsed. Successful zero-flag C1 arms `Some(Unknown)`.

Initialize the slot to `None` in both `AlertTmp108::new` and
`AsyncTmp108::into_alert`. Convenience constructors continue delegating.
Construction remains synchronous, infallible, and I/O-free.

### 3.4 Exact set and clear points

1. **Retained entry:** inspect the slot before any I/O. If it contains
   `Some(cause)`, copy `cause` into a local value and leave the slot
   unchanged. Skip C0, GPIO, and C1.

2. **Fresh entry:** with `None`, perform C0. Use its settings to select
   mode and polarity exactly as today. C0 failure or cancellation arms
   nothing.

3. **Interrupt fast path:** after successful C0 with nonzero flags, assign
   `Some(interrupt_alert_cause(C0.low, C0.high))` synchronously, in the poll
   that receives C0 success and before awaiting T. No GPIO, C1, or
   intervening await occurs.

4. **Interrupt slow path:** after clear C0, await the asserted level once,
   then perform C1 once. After **successful** C1, assign
   `Some(interrupt_alert_cause(C1.low, C1.high))` synchronously before T.
   Zero flags produce `Some(Unknown)`. C1 flags do not requalify the
   notification; C1 settings do not restart acquisition.

5. **Fresh Comparator:** after its existing level wait, select a local
   `cause = Unknown`, but leave the slot `None`. No comparator branch
   may arm retention.

6. **T:** call `self.sensor_mut().temperature()` once. On failure, return
   `Error::Bus` and leave the slot unchanged. While T is Pending, all
   existing retained state remains on the wrapper.

7. **T success:** synchronously clear the slot to `None`, construct
   `AlertEvent { cause, temperature }`, and return it. There is no
   `.await` between T success, clearing, and returning Ready.

No additional configuration read is allowed to obtain, confirm, or
refresh cause.

**Agree with reliability hazard 3:** direction comes exclusively from the
C0/C1 snapshot already required by the protocol.

### 3.5 Copy discipline and cancellation

**Agree with H-4:** do not call `take()` before awaiting T and restore only
on `Err`. Cancellation can drop the future without executing an error
branch.

Because `AlertCause` is `Copy`, the implementation should copy the
discriminant into a local variable while retaining the authoritative slot
on the wrapper:

```text
retained entry:
    local cause := copy of slot's cause
    slot remains Some(cause)
    await T
    on error: return, slot unchanged
    on success: clear slot and return, without awaiting again
```

Do not keep a field borrow alive across a mutable call to the sensor.
Copying avoids both that borrowing problem and premature consumption.

There is no suspension between a successful qualifying configuration
read and recording its cause, nor between T success and settling the
debt. Dropping a future while T is Pending preserves the obligation.
Discarding an already-returned result does not restore it.

This proves the existing **T-stage** cancel-safety property only.
C0/C1 may consume hardware evidence before successful completion, and
underlying HAL cancellation guarantees remain outside the driver’s control.

### 3.6 Two APIs, one consumptive stream

**Adopt reliability hazard 2’s recommended contract:**

> The APIs are two views of one consumptive stream, not two subscribers.

There is one wrapper-local slot:

- Either method may acquire and retain a cause.
- Either method may settle that same obligation.
- A successful trait delivery intentionally discards cause.
- A later inherent call cannot retrieve an already-consumed cause.
- If the trait fails or is cancelled during T, the cause remains available
  to the inherent method.

Examples:

| Sequence | Required result |
|---|---|
| Trait captures FH, T fails; inherent method retries | T only; `AboveHigh` with the retry-time sample. |
| Inherent method captures FL, T fails; trait retries | T only; scalar temperature; FL cause is consumed and discarded. |
| Either method repeatedly fails or is cancelled during retained T | Same cause remains pending; no configuration or GPIO access. |
| Either method succeeds, then either method is called again | Fresh acquisition, not a second delivery of the old cause. |

The trait must therefore retain cause internally even though its successful
public result does not expose it.

### 3.7 Historical cause and later sample

**Agree with H-5:** a retained cause can become historical relative to both
the chip and the returned sample. This is expected.

If A is acknowledged with FH and T fails, then B latches FL before A is
delivered:

1. A’s retry returns `AboveHigh` and a newly read sample.
2. That sample may be below the low limit.
3. The retry performs no configuration read, so it does not consume B.
4. The following fresh acquisition can collect B and return `BelowLow`.

Do not refresh A’s cause, merge A and B into `Both`, or reject A because
the sample contradicts it. The exact wording in the shipping rustdoc is:

> A retained cause can describe an earlier interrupt even if another
> interrupt has since latched in the opposite direction. The returned
> sample need not support the retained direction: it may be inside the
> limits or beyond the opposite limit. This is expected, not grounds to
> replace the cause or reject the event.

The slot retains acknowledgment evidence, not an atomic event/sample pair.

### 3.8 Consumption and lifecycle

**Agree with H-8:** the return shape is all-or-nothing
`Result<AlertEvent, Error<...>>`. This design does **not** introduce a
partially successful receipt or a separately fallible sample field.

Only successful T settles a retained obligation. On T error, no event is
returned, but the cause remains available on a later successful attempt.

**Agree with H-7:** retention is an Interrupt acquisition operation, not a
generic consequence of having a local cause. Comparator produces
`Unknown` without arming the slot. An already-existing Interrupt debt
nevertheless takes precedence over a later mode change.

**Agree with I-25:** lifecycle abandonment should now be pinned explicitly:

- `sensor_mut()`, reconfiguration, and direct temperature reads do not
  clear the slot.
- `into_inner()`, `destroy()`, and dropping the wrapper abandon it without
  driver I/O.
- Rewrapping the recovered sensor or bus starts empty.
- No cause transfers implicitly into the bare driver.

## 4. Decisions 1 and 2

### Decision 1: comparator reports `Unknown`; add no status read

Agree with the existing comment at `src/lib.rs:2286-2288`: comparator C0
flags are not evidence of the event that the caller is about to wait for.

They are not meaningless bits. They describe an earlier comparison. What
is missing is a justified relationship to the subsequent qualifying level
observation.

Between C0 and completion of the level wait, an assertion can release and
another can occur, potentially on the opposite side. Even an immediately
ready wait does not prove continuity between those observations.
Hysteresis also separates the pin predicate from a simple current
threshold comparison.

Therefore:

- do not shortcut the comparator wait using C0 flags;
- do not label the eventual observation with C0’s direction;
- do not infer direction from T;
- do not add comparator C1.

An additional read changes an established protocol and still does not
create an atomic flag/pin/temperature snapshot. Explicit uncertainty is
the conservative, defensible result.

### Decision 2: implement the trait as a projection of `wait_for_alert`

Move the acquisition and delivery protocol into the inherent method. The
existing trait body becomes the projection:

```rust
self.wait_for_alert()
    .await
    .map(|event| event.temperature.to_degrees())
```

This is justified by having **one owner for a destructive-read protocol
and its retained state**, not merely by two call sites. Duplicated
protocols would invite divergent qualification, retry, and consumption
policies.

Preserve existing public signatures, bounds, associated errors, and scalar
conversion. The inherent method must not call back through the trait.
The trait must introduce no await after obtaining the event.

Behavioral equivalence requires:

1. **Unchanged baseline:** run the existing `alert_ordering` suite before
   implementation and after delegation. Keep its existing test bodies
   and assertions unchanged, including zero-C1, comparator, cancellation,
   retained retry, and newer-latch tests.

2. **Paired executions:** add independent fake-world runs using identical
   scripts for the trait and inherent method. Compare complete
   cross-interface logs, each poll’s Pending/Ready outcome, exact errors,
   remaining scripted steps, and successful scalar projection.

3. **Mixed-method executions:** prove that either method can arm, preserve,
   or consume the same obligation, and that a successful scalar delivery
   does not leave retrievable cause behind.

The paired matrix must cover every path in section 3.2 and every failure
or cancellation phase in section 6. Do not retarget the existing tests to
the inherent method; that would remove the published-interface baseline.

## 5. Decision 3: shared direction vocabulary, distinct status and delivery records

**Reuse `AlertCause` as the direction vocabulary for future APIs, but do
not treat it or `AlertEvent` as complete register status. Defer #65’s entry
points, not the semantic boundary.**

Issue #65 was inspected using:

```text
gh issue view 65 -R OpenDevicePartnership/tmp108 --json number,title,body,comments
```

It identifies configuration reads in `probe`, `read_configuration`,
`configure`, `one_shot`, `shutdown`, `wait_for_temperature`, and
`continuous`. Hysteresis delegation also reaches configuration reads.
`continuous` reads at entry and cleanup and permits further reads inside
its closure.

These operations are not GPIO waiters. Some have no temperature sample
at all.

### 5.1 Raw status and observed cause are not interchangeable

| Boundary | Zero flags | Unknown direction | Associated data |
|---|---|---|---|
| Successful configuration snapshot | Valid observed raw pair `(false, false)` | Both bits were read; their values are not unknown | Settings and flags from one read; no sample implied. |
| Qualified alert delivery | Cannot negate an already-observed notification | `AlertCause::Unknown` | Direction evidence, if available, plus a separate later sample. |

The issue’s sketch is right for the second row, not a substitute for the
first.

Do not add `NoAlert` to `AlertCause`, and do not claim that zero flags prove
no historical event occurred. Treating `Unknown` as “read zero flags”
universally would erase a useful knowledge distinction.

### 5.2 Recommended future #65 direction

A future explicitly named snapshot/acknowledge operation should exist on
both bare driver flavors and return settings and raw FL/FH from one read.

Two booleans are appropriate in that raw record: all four pairs are valid.
That record can expose a projection into the same direction vocabulary:

- zero flags → `None`;
- FL only → `Some(AlertCause::BelowLow)`;
- FH only → `Some(AlertCause::AboveHigh)`;
- both → `Some(AlertCause::Both)`.

A successful raw snapshot never synthesizes `Some(Unknown)`. This
`Option<AlertCause>` would be an accessor result with a documented subset
of outputs, not the raw stored representation or a validation burden on
snapshot construction.

There is no need for a second incompatible directional enum. Keeping
`AlertCause` HAL-independent and feature-independent enables both blocking
and async status APIs to reuse it.

Do not publish that snapshot or projection in #67. It still needs decisions
about Mode exposure, #62/#60 interactions, per-read provenance, and errors
that occur after destructive reads. The existing private snapshot is not
automatically the right public record merely because it already exists.

### 5.3 Why `continuous()` needs more than an `AlertCause` return value

A single cause returned at the end of `continuous()` is insufficient:

- Entry and cleanup acknowledge at different times.
- Merging their flags would erase which phase consumed the evidence.
- The entry read may succeed before the subsequent mode write fails.
- The closure may fail after additional destructive reads.
- Cleanup may consume flags and then fail.
- Existing error precedence returns the closure error ahead of a cleanup
  error.
- Cancellation may prevent any final report.

A success-only status wrapper would still discard useful observations
when subsequent operations fail.

Before a status-bearing continuous/RMW API is ready, it must specify:

1. which read produced each observation;
2. how observations survive later operation failure;
3. whether reporting is immediate or retained;
4. how entry, closure operations, and cleanup are distinguished;
5. ownership through mutable access and decomposition;
6. cancellation limitations and error precedence.

A future phase-specific result or observation channel can compose
configuration snapshots and the same cause vocabulary. #67 does not
choose that transport, change existing signatures, or add persistent
bare-driver state.

The architectural boundary is deliberate:

> Raw status belongs to the register observation; cause belongs to
> interpretation of a qualified notification; temperature belongs to a
> separate sample.

Reusing vocabulary and codecs is appropriate. Forcing all three into one
record would fabricate samples or conflate known-empty status with missing
direction.

## 6. Invariants and failure modes

| Phase | Failure or cancellation | Slot and next invocation |
|---|---|---|
| Fresh C0 | Bus error or dropped future | `None`; hardware may already have consumed evidence. Next call is fresh. No GPIO, T, or cleanup follows error. |
| GPIO level wait | Pin error or dropped future | `None`; no C1 or T follows error. Retry observes afresh; HAL recovery remains separate. |
| Interrupt C1 | Bus error or dropped future | `None`; acknowledgment outcome is uncertain. Do not arm `Some(Unknown)` before successful C1. No T or cleanup follows error. |
| Initial Interrupt T | Bus error or dropped future | Preserve `Some(cause)` armed after C0/C1 success; next call is T only. |
| Retained T | Bus error or dropped future | Preserve the identical `Some(cause)`; no acquisition or direction refresh. |
| Fresh Comparator T | Bus error or dropped future | Remain `None`; next call performs fresh C0 and a level observation. |
| Any successful T | No further await | Clear to `None`, return event or scalar projection in the completion poll. |

Additional invariants:

1. Nonzero interrupt C0 prohibits GPIO and C1 on that invocation.
2. Slow interrupt acquisition uses one level wait and one C1 even if C1
   is zero or the pin became inactive before task resumption.
3. No InputPin sampling, edge waits, speculative configuration reads,
   limit reads, writes, conversion delays, retries, or cleanup
   acknowledgments are added.
4. Fresh Comparator acquisition never arms the slot.
5. T never determines cause or qualifies/rejects a notification.
6. `Both` produces one result, not two ordered events.
7. `Error<E, P>` gains no variant and retains existing bounds and mappings.
8. A known C1 cause is authoritative for the observed C1 flags, not proof
   of a one-to-one correspondence with a particular GPIO edge.
9. A newer hardware latch is neither refreshed into nor merged with an
   older retained obligation.
10. There is no stronger atomic snapshot-and-clear guarantee relative to
    concurrent conversions than the hardware documents.
11. Permanent T failure can indefinitely delay fresh acquisition; the
    caller owns backoff and recovery.
12. Events consumed by #65 operations or independent readers before
    acquisition remain unrecoverable through this API.

### 6.1 Reliability review disposition

All supplied reliability findings are adopted, with these explicit
interpretations:

| Finding | Disposition |
|---|---|
| Five distinct pending states | Agree; represented exactly by `Option<AlertCause>`. |
| One slot shared by both APIs | Agree; two views of one consumptive stream. |
| No extra cause read | Agree; use only required C0/C1 snapshots. |
| H-4: premature `take()` | Agree; copy cause and leave authoritative state on the wrapper across T. |
| H-5: retained cause and latest sample may disagree | Agree; expected historical evidence, explicitly documented. |
| H-7: comparator must not arm retention | Agree; only Interrupt acquisition assigns `Some`. |
| H-8: consumption boundary | Agree; all-or-nothing result, settlement only on successful T. |
| I-25: lifecycle abandonment unpinned | Agree; add explicit lifecycle tests without weakening existing tests. |
| Five missing regression families | Adopt all five in section 8, including delayed T completion polling. |

No stronger durability, cancellation, or event-identity guarantee is inferred
from these agreements.

## 7. Alternatives considered

| Alternative | Reason not selected |
|---|---|
| `Option<{ low: bool, high: bool }>` on the event | Five inhabitants; `Some(false, false)` supplies no known direction for a qualified notification and requires an extra convention. Four explicit observation outcomes are clearer. |
| Three-variant known cause plus `Option` | Sound and equally small. Rejected only for call-site clarity: `Unknown` is less easily confused with “no event.” |
| Separate knowledge boolean and direction | Creates contradictory or meaningless combinations requiring validation. |
| Universal status enum with `NoAlert` | Zero flags do not negate an observed notification and would introduce an impossible successful waiter outcome. |
| Publish private `AlertSnapshot` directly | Raw status lacks the waiter’s qualification, retained delivery, and sample semantics; publishing it also prematurely settles #65. |
| Include optional or retained `Config` | Adds provenance and freshness promises without a stated use case; does not provide thresholds or trigger-time settings. |
| Trust comparator C0 flags | Attributes an earlier comparison to a later level observation without evidence of continuity. |
| Add comparator C1 | Changes preserved behavior and still cannot provide an atomic event snapshot. |
| Keep the boolean and return Unknown after retry | Discards direction the driver already knew; fails the core retention requirement. |
| Separate old/new wait implementations | Duplicates destructive protocol and consumption policy; more likely to diverge when mixed. |
| Return an event with separately fallible temperature | Could expose cause immediately on sample failure but introduces different success/consumption semantics. Not needed to preserve the existing retry contract. |
| Add a pending-cause getter or take operation | Creates another observation/consumption protocol and additional promises outside #67. |
| Use `f32` in the inherent result | Abandons the existing precise `Celsius` representation unnecessarily. |
| Mark both types `#[non_exhaustive]` | Pays an ongoing matching/construction cost for unspecified extensions; the chosen domain is intentionally small and closed. |
| Infer cause from T or fresh limits | Reintroduces #58’s invalid inference and may add destructive or unnecessary I/O. |

The rejection of optional booleans for an **event** does not reject two
booleans for a **raw snapshot**. The latter has four valid hardware
combinations, including zero.

## 8. Test specification and regression proof

Use existing dependencies and the existing fake-world infrastructure.
No new runtime or test dependency is required.

Keep the existing `alert_ordering` test bodies and assertions unchanged.
Add new tests alongside them.

### 8.1 Pure layer: `tests::ops_tests`

- Exhaust all four inputs to `interrupt_alert_cause`.
- Preserve the existing exhaustive snapshot decoder tests over all 65,536
  register patterns.
- Add composition checks establishing FL = first-byte bit 3 and FH =
  first-byte bit 4.
- Confirm every other configuration bit is irrelevant to cause mapping,
  including M=`0b11`.
- Do not call the generated `m()` decoder.
- Document that zero-to-Unknown is interpretation after qualification,
  not raw evidence that an alert occurred.

### 8.2 Fake-world layer: `tests::asynchronous::alert_ordering`

Use the shared I2C/GPIO operation log, destructive-read effects, strict
pin fake, and manual polling. No wall-clock sleeps or timeouts.

| Test family | Required coverage and assertions |
|---|---|
| Fast cause matrix | FL, FH, both × both polarities; exact cause and Celsius; C0/T only; strict no GPIO; in-band T does not change cause. |
| Slow cause matrix | Initially clear C0; successful level wait; all four C1 pairs × both polarities; exact cause and C0/wait/C1/T sequence. |
| Zero-C1 qualification | Return Ready with Unknown, never re-wait or resample InputPin. |
| Observation survives release | Pulse active then inactive before resumption; zero C1 still delivers Unknown. |
| Assertion before arming | Already-active level after C0 is accepted without needing another edge. |
| Comparator isolation | Both polarities, already-active and later-active waits, all four C0 pairs; always Unknown; C0/wait/T only. |
| Comparator stale entry direction | FL at C0 followed by a later high-side assertion does not report BelowLow; mirror the direction. |
| Repeated comparator level | Repeated calls while asserted each perform their own C0/wait/T and report Unknown. |
| Fast retention identity | Each known cause; T error then success with different sample; retry T only and original cause. |
| Slow retention identity | All four C1 outcomes, including Unknown; T error then retry T only with original cause. |
| Initial and retained T cancellation | Fast and slow origins, both polarities; drop Pending T and retry without losing cause. |
| Repeated failures | Several T errors/cancellations preserve identical state until one success. |
| Newer opposite latch | A=FH retained; B=FL latches; deliver AboveHigh via T only, then BelowLow via fresh C0/T. Mirror directions. |
| Mixed-method ownership | Trait-fail/inherent-success, inherent-fail/trait-success, and alternating failed/cancelled retries; one shared settlement. |
| Pre-retention failures | C0/C1 errors before and after modeled hardware acknowledgment, pin errors, and cancellation; no invented cause or cleanup. |
| Direct access and mode switch | Direct temperature reads and reconfiguration preserve the captured cause; retained Interrupt debt outranks Comparator mode. |
| Lifecycle abandonment | `into_inner`, `destroy`, and drop cause no driver I/O; reconstructing a wrapper starts empty. |
| Paired API equivalence | Independently replay identical scripts through both APIs; compare logs, poll outcomes, exact errors, and scalar projection. |

### 8.3 Required missing-test additions from reliability

These five families are explicit acceptance criteria, not optional
extensions to the table.

#### A. Cross-entry-point ownership symmetry

Exercise both acquisition paths and both directions of API handoff:

1. Trait captures a known cause, T fails or is cancelled.
2. Inherent retry returns the original cause using T only.
3. Following call is fresh.

And:

1. Inherent method captures a cause, T fails or is cancelled.
2. Trait retry returns scalar temperature using T only.
3. Following inherent call is fresh and cannot recover the consumed cause.

Include alternating failures before settlement and Unknown-origin
obligations. Assert transaction logs rather than only slot contents.

#### B. Unknown-direction retention

Use clear C0, successful level wait, and successful zero-flag C1.

- Fail or cancel initial T.
- Retry through either API.
- Require T only; retained state must not behave like Empty.
- On inherent success, require `AlertCause::Unknown`.
- Repeat under both polarities.

This detects accidental conversion from `Some(Unknown)` to `None`.

#### C. A’s cause survives opposite-direction B

Acknowledge FH for A, fail or cancel T, then latch FL for B.

- Retry A with a sample consistent with B or inside the band.
- Require `AboveHigh`, not `BelowLow`, `Both`, or `Unknown`.
- Require B’s hardware flag to remain set after A’s T-only delivery.
- Fresh acquisition then returns `BelowLow`.

Mirror FL/FH and include a mixed-method acquisition/retry sequence.

#### D. Delayed T completion settles in the completion poll

The existing permanently-Pending fake is insufficient by itself for this
test. Add a test-local controllable temperature response that:

1. records one T transaction;
2. returns Pending on the first poll;
3. becomes Ready with a successful sample when the test releases it;
4. does not record a second transaction when polled again.

Test both initial and retained delivery, through both APIs.

Assert:

- the completion poll returns Ready, not another Pending;
- no intervening GPIO/configuration work occurs;
- after dropping the completed future, the next call performs fresh C0;
- the old cause is not replayed.

This pins clearing and result projection in the same poll that resolves T,
not merely eventual successful clearing.

#### E. Fresh comparator cancellation versus old Interrupt debt

Test separately:

- Fresh Comparator with nonzero C0 flags reaches T, T remains Pending,
  and the future is dropped. Retry must perform fresh C0 and a level wait.
- Interrupt acquisition retains a known cause; then reconfigure to
  Comparator. A cancelled retained T retry must preserve that Interrupt
  obligation, and the next attempt must still be T only.

Both outcomes are required. A generic “cause exists, therefore retain”
branch would fail the first; an unconditional mode-based clear would fail
the second.

### 8.4 Lifecycle tests for I-25

Arm retained state using real fake-world acquisition, not by directly
assigning the field.

- **`into_inner`:** record the log, decompose, and confirm no added I/O.
  Rewrap using `into_alert`; confirm no constructor I/O and that the next
  call performs fresh C0 rather than retained T.
- **`destroy`:** recover the bus and pin without I/O; reconstruct the
  wrapper and require fresh acquisition.
- **Drop:** hold an external reference to the fake-world log, drop the
  wrapper, and confirm no new I/O. Wrapper-local state cannot survive
  destruction; no cleanup acknowledgment is allowed.

A fresh C0 returning zero flags with an inactive pin may remain Pending
under manual polling. That is appropriate in a fake test and must not
be copied into a hardware timeout harness.

### 8.5 Compile pins and doctests

In `tests/reexports.rs`:

- Pin root `AlertCause` unconditionally, all four variants, exhaustive
  matching, and `Clone`, `Copy`, `Debug`, `PartialEq`, `Eq`, `Hash`.
- Under `embedded-sensors-hal-async`, construct and destructure
  `AlertEvent` from outside the crate; pin both field types and derives.
- Pin the inherent method with a generic wrapper using the existing
  I2C/Wait/InputPin bounds and result type.
- Do not import the threshold-wait trait merely to call the inherent
  method.
- Preserve all existing pins.

Execute the shipping doctests:

- `AlertCause` must compile and run without optional features.
- `AlertEvent` and `wait_for_alert` run under their feature gate.
- The method’s fast-path mock example must consume C0 and T only, with no
  GPIO expectations.
- The documentation must never demonstrate direction inference from the
  returned sample.

### 8.6 What fails on today’s code

At baseline `e4a3607`, the new types, mapper, and method do not exist.
The new compile pins, doctests, and event tests initially fail to compile
for those missing items. Record that honestly; current code does not
violate an already-published status contract.

After adding minimal declarations and a provisional adapter that calls
today’s scalar waiter and returns Unknown, these must still fail by
assertion:

1. **Fast FL/FH/both cause matrix:** current code discards known direction.
2. **Slow nonzero-C1 matrix:** current code discards C1 direction.
3. **Retained known cause after T failure/cancellation:** the current
   boolean cannot preserve identity.
4. **Trait-fail/inherent-success:** the existing trait retains no cause to
   expose later.
5. **A=FH followed by B=FL separation:** an always-Unknown adapter or
   direction refresh cannot satisfy both expected identities and logs.

The provisional adapter is only a local red-test stage, not a required
commit or acceptable final behavior.

Zero-C1 qualification, comparator isolation, same-poll settlement, and
lifecycle tests may pass a faithful adapter. They are preservation guards,
not sufficient proof of the new capability. The unchanged baseline suite
likewise proves compatibility rather than #67’s new information flow.

### 8.7 Verification gates

Run the complete canonical AGENTS.md matrix during implementation:

- nightly formatting;
- pedantic clippy;
- rustdoc;
- unit tests and doctests;
- example builds;
- feature powerset;
- README snippet synchronization;
- cargo-vet;
- semver checks.

Specific focused commands include:

```text
cargo test --locked -F async,embedded-sensors-hal-async alert_ordering
cargo test --doc --locked
cargo test --doc --locked -F async,embedded-sensors-hal-async
cargo hack --feature-powerset check --locked
cargo semver-checks check-release
```

All six legal combinations must compile:

1. no features;
2. `embedded-sensors-hal`;
3. `async`;
4. `async,embedded-sensors-hal`;
5. `embedded-sensors-hal-async`, implying `async`;
6. `embedded-sensors-hal,embedded-sensors-hal-async`, implying `async`.

The ungated cause type must introduce no optional-crate imports.
Wrapper-only helpers and tests must be gated to avoid missing symbols or
unused private code when disabled.

The maintainer owns the version change. Expect the task’s public-API
minor-bump release requirement, record actual semver output, and do not
change Cargo.toml or suppress checks as part of this work’s implementation
plan.

No implementation test execution is claimed by this design document.

## 9. Documentation and compatibility

Existing callers may keep using the trait with the same signatures,
bounds, associated error type, scalar values, and wire behavior.

During implementation:

- Update wrapper rustdoc and private comments that currently say direction
  or all C1 flags are discarded.
- Replace that statement with: C1 flags supply direction when available
  but **never requalify** a successful level wait.
- Document shared consumption on both entry points.
- Preserve the phase-specific recovery table and backoff warning.
- Correct README’s assertion that no API exposes flags; distinguish the
  scalar trait from the new inherent method.
- Keep existing trait-based README snippets as compatibility examples
  unless there is a concrete reason to change them.
- If snippets change, edit the source example marker regions first and
  keep the synchronization gate. Do not hand-author drifting snippets.

Plan an entry under `CHANGELOG.md` **`[Unreleased]`**:

> Added `AlertTmp108::wait_for_alert`, returning `AlertEvent` with observed
> FL/FH direction (`BelowLow`, `AboveHigh`, `Both`, or `Unknown`) and a
> latest-conversion `Celsius` sample. Fresh comparator observations and
> zero-flag interrupt acknowledgments report `Unknown`; temperature is
> never used to infer direction (SBOS663A §7.5.3.4, §7.5.2). Acknowledged
> interrupt direction survives sample errors and cancellation on the same
> wrapper. The new method and existing threshold-wait trait share one
> consumptive retained-delivery obligation; the trait still returns only
> temperature.

Do not plan a Cargo.toml version bump; the maintainer performs it.

### 9.1 Explicit supersession of prior design text

This design refines the deferred-direction portions of:

- `2026-09-24-tmp108-alert-sample-failure-design.md`, particularly §§1.1,
  2.2, 5, and 6;
- `2026-09-24-tmp108-issue-59-alert-wait-ordering-design.md`, particularly
  §§2.8, 8.2, and 8.4.

The boolean becomes an option carrying cause. “C1 flags discarded” becomes
“C1 flags never gate delivery.”

Their acquisition ordering, qualification, lifecycle, error mapping, and
T-stage cancellation contracts remain in force. Historical specifications
need not be rewritten as though the status API already existed.

The June type-split design’s private codec and thin driver-shell boundary
is preserved.

## 10. Human hardware validation

AGENTS.md requires live verification before shipping alert-pin changes.
This design task does not run hardware.

Use the documented Pico de Gallo and TMP108 at `0x48`, dedicated GPIO0
ALERT, and appropriate pull-up. A maintainer must be present. Record board
and firmware versions, HAL versions, configuration, limits, returned
cause/sample, and available transaction/level traces.

### 10.1 Operational safety

The rig can wedge when a GPIO wait is abandoned.

- Do not race waits against timeouts.
- Do not drop or kill tasks with outstanding GPIO waits.
- Do not use a generic timeout harness.
- Arrange a controllable stimulus before starting each wait.
- Keep each waiter alive until it completes.
- If necessary, have the human supply a deliberate qualifying stimulus.
- Any forced stop and physical replug is manual recovery, not a passing
  test.

Cancellation tests belong in the fake world. Diagnostic configuration reads
must not consume flags before the operation under test.

### 10.2 Required scenarios

#### 1. Interrupt fast-path persistent/transient A/B

Configure mode, polarity, and conversion behavior before creating a latch.

For FH:

1. Move THIGH below ambient using a limit-register write.
2. Allow a conversion and verify assertion without reading configuration.
3. For the transient case, restore THIGH above ambient with another
   limit-register write.
4. Allow an in-band conversion without configuration polling.
5. Call `wait_for_alert`.

Expect `AboveHigh`, potentially with an in-band sample, and no need for a
new assertion. Trace C0/T only.

Repeat with the over-temperature condition held throughout. Mirror the
procedure for FL by moving and restoring TLOW. Repeat under both
polarities with verified wiring.

The transient case is mandatory: finger warming alone can mask #59’s
historical failure.

#### 2. Both flags

Without an intervening configuration read, create high-side and low-side
comparisons using limit writes, restoring safe limits between stimuli.
Allow conversions at each step, then restore an in-band window.

Expect one `Both` event, not two ordered events. Capture the driver’s C0
response through tracing, not an extra preflight configuration read.

#### 3. Slow Interrupt acquisition

Begin with clear flags and inactive ALERT. Let the waiter arm, then warm
or cool across each threshold.

Expect:

```text
C0 → one asserted-level wait → C1 → T
```

The reported direction must match the C1 flags obtained by that operation.

Exercise an assertion near the read-to-arm boundary if a controlled setup
permits it. Host timing alone is not a deterministic substitute for the
fake-world race tests.

#### 4. Comparator, both polarities

Await a deliberate assertion and expect Unknown. Repeat immediately while
the level remains asserted and expect another Unknown.

Allow release into the hysteresis band, then apply a new stimulus to
complete another wait.

Trace C0/level/T only, with no C1. Verify the old trait examples still show
the same repeated-level behavior.

#### 5. Retained cause and mixed methods

If a controlled HAL adapter is available, fail only T in software after
an actual successful C0/C1. Do not disturb GPIO RPCs or cancel a wait.

Restore T and retry on the same wrapper. Expect:

- original cause;
- retry-time sample;
- T only.

Exercise trait-fail/inherent-success and inherent-fail/trait-success.
If practical, latch a distinct opposite-direction event meanwhile and
verify that it is collected only by the following fresh acquisition.

If safe injection is unavailable, record this hardware subcase as
unperformed. Mock proof remains mandatory; do not casually substitute
cable disconnection or abandoned GPIO waits.

#### 6. Zero-flag C1

Deterministic proof belongs in the fake-world suite.

Only if a controlled test hook can acknowledge after the level wait has
completed but before C1, without abandoning GPIO RPCs, exercise this
diagnostic race on hardware. The result must complete with Unknown.

This adversarial test does not authorize concurrent acknowledgment in
normal operation.

### 10.3 Compatibility examples

The human should also run the canonical AGENTS.md hardware example matrix
after implementation, especially:

```text
cargo run --example alert_interrupt -F async,embedded-sensors-hal-async
cargo run --example alert_comparator -F async,embedded-sensors-hal-async
```

Supply the documented warming/cooling stimuli and allow waits to complete.
Keep these compatibility demonstrations separate from status-bearing
validation; scalar examples cannot prove direction preservation by
themselves.

## 11. Deliberate exclusions and remaining decisions

This work does not:

- repair #65’s seven entry points or perform their documentation sweep;
- publish a configuration snapshot/acknowledge API;
- add a status channel to `continuous`;
- change `read_configuration`, `Config`, or `Error`;
- repair Mode decoding or conversion-completion behavior;
- change DDSL or generated `src/inner.rs`;
- add configuration caching, shared-line demultiplexing, or multi-master
  arbitration;
- provide an event queue, timestamp, count, ordered history, trigger-time
  sample, or durable receipt;
- provide immediate cause access on a failed T;
- promise exactly-once delivery per physical excursion;
- strengthen hardware read/clear atomicity or HAL cancellation guarantees;
- introduce dependencies, allocation, `std`, or a blocking ALERT mirror;
- delegate version ownership away from the maintainer.

**No unresolved #67 API question blocks implementation after maintainer
approval.**

The maintainer must still approve the proposed public surface, select the
release version, and schedule human hardware verification. #65’s
per-read provenance and error/cancellation reporting remain explicit
follow-on design questions, not omissions to fill opportunistically while
implementing `wait_for_alert`.

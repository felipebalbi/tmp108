# TMP108 issue #58: retained interrupt delivery after sample failure

**Date:** 2026-09-24  
**Status:** design record of the implemented Gap 2 fix; release decision remains with the maintainer  
**Branch:** `fixes`  
**Audience:** maintainers familiar with Rust async polling and the TMP108 ALERT protocol  
**Diataxis quadrant:** explanation  
**Related issues:** #58 (Gap 2 addressed; status API still open), #59 (partially superseded), #65 / #62 (unchanged)

An acknowledged interrupt and a successful temperature sample are separate
facts. The wrapper now remembers that it owes delivery after acknowledgment,
so failure or cancellation of the temperature read does not force the caller
to wait for a different interrupt. This is a wrapper-local delivery obligation,
not a stored sample, durable receipt, or exactly-once event protocol.

## 1. Problem and scope

The [#59 design](2026-09-24-tmp108-issue-59-alert-wait-ordering-design.md)
fixed the lost wakeup caused by acknowledging an already-pending interrupt
before waiting for it. Its successful acquisition paths are preserved here.
The remaining Gap 2 was the final fallible temperature transaction: C0 or
C1 had already acknowledged the event, then T could fail and return only
`Error::Bus`. Retrying started over with hardware evidence already gone.

The authoritative refined brief is the last maintainer comment on
[issue #58](https://github.com/OpenDevicePartnership/tmp108/issues/58).
The supplied reliability review additionally requires cancellation analysis,
mode-switch precedence, lifecycle semantics, and an explicit liveness warning.
This record describes the existing implementation rather than proposing new
executable changes.

The hardware basis is TI TMP108 datasheet SBOS663A §7.5.3.4: reading
configuration clears the interrupt flags and ALERT pin. The temperature
register supplies the most recent conversion (§7.5.2), not trigger history.
The prior design and issue comments supply these citations and hardware
observations. No new hardware experiment or independent full datasheet audit
is claimed here.

### 1.1 Why Gap 2 is separable from Gap 1

Gap 1 concerns information the existing scalar-returning trait cannot expose:
FL/FH direction. C0 captures it privately but collapses it to a notification
decision. C1's flags are deliberately discarded, including when zero; a
successful level wait remains qualifying evidence. Gap 3 therefore matters
to any future status API: direction may be unknown, and must not be inferred
from T.

Gap 2 needs only the fact that an acknowledged notification is owed, not its
direction. A private boolean can preserve that obligation across T failures
without publishing a receipt or changing the trait's success/error types.
It does not solve Gap 1 or settle the future status API's ownership policy.

## 2. Chosen design

### 2.1 State and ownership

`AlertTmp108` has a private `interrupt_sample_pending: bool`, initialized
to `false` by `new` and `AsyncTmp108::into_alert`; the A0 convenience
constructors delegate to `new`. Despite its internal name, this boolean
records **delivery debt**, not a sample, flag direction, timestamp, count,
or configuration cache. Construction remains synchronous, infallible, and
I/O-free.

The obligation belongs to the wrapper, not to the chip or a waiter future.
Dropping a waiter at T leaves it on the same wrapper. `sensor_mut()`, direct
temperature reads, and reconfiguration do not clear it. **Retained delivery
takes precedence over the current mode**: switching to Comparator after an
interrupt acknowledgment does not cancel the debt. The next call still
performs T only; it does not consult TM or POL.

`into_inner()`, `destroy()`, and dropping the wrapper abandon the obligation
without driver I/O. Re-wrapping starts empty. Decomposition preserves the
inner sensor and/or owned resources, not the wrapper's delivery state.

### 2.2 Acquisition and delivery

C0 is the entry configuration snapshot; C1 is the post-wait configuration
acknowledgment. Each is one two-byte read using register pointer `0x01`.
T is one two-byte temperature read using pointer `0x00`. These are the same
transactions and codecs used by #59. GPIO waiting uses the asserted level
for the polarity observed in C0, never an edge.

- **Pending on entry:** skip C0, GPIO, and C1. Attempt T once.
- **Fresh Interrupt fast path:** successful C0 has FL or FH set. Arm the
  obligation synchronously before awaiting T. No GPIO or C1 occurs.
- **Fresh Interrupt slow path:** successful C0 has no flags; await the
  asserted level, then perform C1. After successful C1, arm before T.
  C1's flags are discarded even when zero: retention must not depend on
  requalifying the notification with those flags.
- **Fresh Comparator path:** successful C0, one level wait, then T.
  No C1, no flag-driven fast path, and no obligation is armed.
- **T succeeds:** clear the obligation synchronously and return
  `Ok(temperature.to_degrees())`, without an intervening await.
- **T fails:** return `Error::Bus`, leaving any obligation set.
- **C0 or C1 fails:** return `Error::Bus`, with no obligation armed.
- **GPIO fails:** return `Error::Pin`, with no obligation armed.

Every invocation stops at the first failure. There is no retry loop, delay,
cleanup acknowledgment, additional limit read, or register write.

Skipping C0 during retained delivery also avoids consuming a newer hardware
latch. After successful delivery clears the debt, the following call returns
to fresh acquisition and can collect that latch. This is not a queue: the
hardware still coalesces events, and no event count or ordering is promised.

### 2.3 Sample meaning and liveness

T on a retained retry reads the register at retry time. It is not a cached
value or the temperature associated with the crossing. There is no bound
on elapsed time since the crossing, or on conversion age in the register.
The reading can be inside or outside the configured band; comparing it with
limits cannot identify FL/FH direction or reject a historical notification.

An async call need not suspend. If the bus permanently returns immediately
with an error, a retained-delivery loop can repeatedly return immediately-ready
`Err` without yielding. The driver deliberately provides no internal retry
or scheduling policy. Callers must apply backoff or bounded retries. Retention
does not promise bus recovery or eventual successful delivery.

## 3. Cancellation and failure at each await point

The state transitions before and after T are synchronous in the same poll
that reaches them. There is no suspension between a successful qualifying
acknowledgment and arming, or between successful T and clearing/returning.
Cancellation here means dropping the waiter future, not dropping the wrapper.

| Await point | State on entry | Failure or cancellation consequence | Next call on the same wrapper |
|---|---|---|---|
| C0 | Empty | `Error::Bus` on failure; no result on cancellation. Hardware may have acknowledged before reporting success. No obligation is armed. | Fresh C0; the original event may be irrecoverable. |
| GPIO level wait | Empty; C0 succeeded with no interrupt flags, or selected Comparator | `Error::Pin` on failure; cancellation adds no cleanup read. No obligation is armed. | Fresh acquisition; an interrupt still latched can be captured by C0. HAL recovery remains separate. |
| C1 | Empty; interrupt level wait succeeded | `Error::Bus` on failure; cancellation adds no cleanup. The chip may already have cleared the event. No obligation is armed. | Fresh C0; the original event may be irrecoverable. |
| Initial interrupt T | Armed after successful C0 or C1 | `Error::Bus` on failure, or no result on cancellation; obligation remains. | T only. |
| Retained retry T | Already armed | Failure or cancellation does not clear the obligation. | T only again, regardless of current thermostat mode. |
| Fresh Comparator T | Empty | Failure or cancellation leaves no historical delivery state. | Fresh C0 and level observation, then T. |

A successfully completed T clears the debt before returning `Ok`. If the
application subsequently discards that result, the driver does not restore
the obligation. Rust's borrow ending does not prove hardware quiescence or
that a new conversion has occurred.

This is **not full event-delivery cancellation safety**. In-flight C0/C1
can consume evidence before a successful result allows retention. Even where
delivery debt survives cancellation, usability of the bus and GPIO depends
on the underlying HAL's cancellation and recovery guarantees. No asynchronous
drop cleanup runs and no speculative configuration read is allowed.

## 4. Alternatives rejected

| Alternative | Reason for rejection |
|---|---|
| Return NaN, a limit, or a cached sample after T failure | Fabricates a successful measurement or substitutes unrelated history for the promised latest register value. None reconstructs trigger temperature or direction. |
| Bounded internal retries | Adds latency and an arbitrary policy, still loses delivery when the bound is exhausted unless debt is retained anyway, and does not independently solve cancellation at T. |
| Unbounded internal retries | Can hide a permanent fault forever and busy-loop without yielding when the HAL errors immediately. Prevents the caller from owning recovery policy. |
| Move T before C1 | Changes slow-path ordering but leaves the fast path's C0 acknowledgment before T; it is not a complete Gap 2 fix. |
| Move T before C0 | Reads before notification qualification, adds unnecessary sampling when no event is pending, and cannot provide a trigger-time value or make the later destructive C0 cancel-safe. |
| Add a new error variant | Enlarges an exhaustive public enum and changes the error contract without itself retaining delivery. A richer receipt belongs to the separate status-API design. |
| Retain Comparator observations for replay | Changes level-following semantics into historical delivery even though no interrupt was acknowledged. Fresh Comparator retries must re-observe the level; only pre-existing interrupt debt outranks a later mode switch. |

## 5. Explicit non-guarantees and residual risks

- No durable delivery across wrapper destruction, decomposition, restart,
  or power loss; no event queue or background task.
- No exactly-once notification per physical excursion. Relatching in
  Interrupt mode or sustained Comparator assertion can produce multiple
  `Ok` results for the same excursion. FL/FH can coalesce multiple excursions.
- No recovery of an event consumed during unsuccessful/cancelled C0/C1,
  or by another configuration-reading operation before acquisition (#65).
- No FL/FH direction receipt, trigger-time sample, timestamp, or count.
- No automatic retry, fairness, timeout, maximum delivery age, or guarantee
  of eventual progress on a failing bus. Debt can indefinitely delay fresh
  acquisition; newer hardware events can coalesce in the meantime.
- No stronger snapshot/clear atomicity against concurrent conversions than
  the hardware documents; the #59 read/clear timing uncertainty remains.
- No multi-owner acknowledgment arbitration or shared ALERT demultiplexing.
  An independent owner must not change mode/polarity or acknowledge during
  a pending acquisition. `&mut self` only serializes this wrapper's access.
- No HAL cancellation/recovery guarantee and no repair to #62's generated
  Mode decoder. DDSL, generated code, and register codecs are unchanged.

## 6. Public compatibility and future Gap 1 API

This is a **behaviour change to a documented contract, but not a
source-breaking change**. No public API, signature, trait bound, associated
type, or error variant changes. `Error<E, P>` remains unchanged, including
the inability of `Error::Bus` to identify the failed phase. The coordinator
reports `cargo semver-checks check-release` found "no semver update required";
that API result does not erase the need to document the behavioral migration.
The maintainer owns version changes; this documentation pass does not bump
`Cargo.toml`.

The new private boolean does not commit a public representation for future
status. An additive inherent API can later preserve direction separately
from a fallible sample, but must define how it shares and consumes retained
delivery with the existing trait. In particular, it must represent unknown
direction after a successful level wait and zero-flag C1, and cannot invent
direction for an existing boolean obligation. Acknowledgment ownership,
coalescing, cancellation, and lifecycle behavior need their own contract.
Gap 2 is not permission to silently change those policies or the existing
trait's documented retained-delivery behavior.

## 7. Explicit supersession of the #59 design

The following references are to the unchanged #59 design's original line
numbers. Its acquisition ordering and hardware reasoning stand except for
these stated refinements:

- **Line 59, §2.2:** "no ... new persistent wrapper state" is superseded
  by the private delivery-obligation boolean. There is still no configuration
  cache, queue, or background task.
- **Line 63, §2.2:** "Every wait call obtains TM and POL" now applies only
  to fresh acquisition. Retained delivery bypasses configuration entirely.
- **Lines 249–252, §2.9:** C0/T, C0/level/C1/T, and C0/level/T sequences
  describe fresh acquisition only. Retained interrupt delivery is T alone,
  including after a switch to Comparator. No configuration read during GPIO
  waiting remains an invariant.
- **Lines 488–501, §4.10:** deferral of persistent retention is superseded
  for Gap 2. This document settles T-stage consumption, mutable access, and
  wrapper lifecycle policy; durable/status-bearing delivery remains deferred.
- **Lines 929–938, §7.10:** "no replay is promised" and loss after dropping
  at T are superseded. The next call must attempt T only. No cleanup read
  on drop remains required.
- **Lines 1005–1017, §8.3:** T failure is no longer an unrecoverable residual
  delivery failure on the same wrapper. The prohibition on "hidden event
  replay" is replaced by explicitly documented retained delivery. Fabricated
  temperatures, internal retry policy, and a new error variant remain rejected.
- **Lines 1109–1110, §9.4:** irrecoverable T evidence and the broad drop-loss
  statement are superseded by phase-specific retention. C0/C1 loss and HAL
  cancellation uncertainty remain unchanged.

The historical #59 document is not silently rewritten. Its earlier tests
expecting fresh acquisition after interrupt T failure/cancellation must not
be used as the current contract.

## 8. Verification and source anchors

The source of truth is `src/lib.rs`: construction in
`AsyncTmp108::into_alert` and `AlertTmp108::new`; wrapper lifecycle in
`sensor_mut`, `into_inner`, and `destroy`; and the
`TemperatureThresholdWait` implementation's guard, arm, T, and clear.
Its `tests::asynchronous` regressions cover both polarities, both interrupt
acquisition paths, zero-flag C1, repeated T errors, initial and retained T
cancellation, single-use settlement, newer latches, mode changes, direct
temperature reads, and fresh Comparator retry isolation.

Documentation must preserve the phase-specific recovery table and liveness
warning, build without rustdoc warnings, and keep README marker regions in
sync with examples. The documentation handoff records actual formatter,
rustdoc, doctest, unit-test, example-build, and snippet-check output rather
than treating this design as evidence of execution. Hardware examples must
not be run by this documentation task; any additional hardware validation
is a separate maintainer-coordinated activity.

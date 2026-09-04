# Removing `maybe-async-cfg` from the TMP108 driver

Date: 2026-09-04
Status: Approved

## Problem

The driver provides its blocking and async APIs from a single set of sources, unified by
`maybe-async-cfg`. This has two costs.

The first is readability. Seven `#[maybe_async_cfg::maybe(...)]` blocks carry attribute
payloads that restate the type substitutions on every item, and a
`#[cfg(feature = "async")]` / `#[cfg(not(feature = "async"))]` pair sits inside the body of
nearly every bus-touching method. The latter exist because `device-driver` generates
distinct method names for the two modes (`read` and `read_async`), which the macro does not
bridge for us. A reader has to mentally expand the macro to know what the crate actually
exposes.

The second cost is that `async` is not an additive feature. Enabling it does not add an
async API, it replaces the blocking one. Two crates in one dependency graph, one wanting
the blocking TMP108 and one wanting the async TMP108, cannot both be satisfied: Cargo
unifies the feature and one of them silently loses the API it compiled against.

## Decisions

Four decisions were taken before the design was fixed.

**Blocking and async will be usable simultaneously.** The blocking API is always present;
`async` becomes additive and adds the async API alongside it. This removes the feature
unification hazard.

**API churn is not a constraint.** The crate is pre-1.0 and the preceding
`device-driver` v2 migration already renamed public enum variants, so the design is chosen
on its merits rather than to preserve import paths.

**Two types in two modules, rather than a `Mode` marker generic.** A marker generic in the
style of `embassy-mcxa`'s `I2c<'d, M: Mode>` was considered and rejected. In `embassy-mcxa`
the marker earns its place because construction differs per mode — the async constructor
requires an interrupt binding and the DMA constructor requires channels — so the marker
prevents calling an async method on a driver that has no interrupt handler bound. TMP108
has no such resource: construction is identical in both modes. The marker would add a
public type parameter to every downstream signature that stores the driver, plus sealed
trait and `PhantomData` boilerplate, and it would not remove a single line of duplicated
method body, because the async and blocking bodies differ regardless.

A third option, one type with suffixed method names (`temperature` and
`temperature_async`), was also rejected. It is the least code, but it reintroduces the
asymmetric naming that `embedded-hal` moved away from, and it is unsafe in a specific way:
an I2C type that implements both the blocking and async HAL traits — as embassy's does —
would expose both method sets on one driver, so calling the wrong one compiles and is
silent.

**Module names are `blocking` and `asynch`.** `async` is a reserved word; `asynch` is the
spelling used by `embedded-hal-bus`, `embedded-io-async` and embassy for this reason.

## Design

### Module layout

```
src/lib.rs        crate docs, module decls, re-exports, shared vocabulary types
src/interface.rs  Interface<I2C> and its register-interface impls
src/blocking.rs   blocking::Tmp108
src/asynch.rs     asynch::Tmp108, asynch::AlertTmp108   (#[cfg(feature = "async")])
src/inner.rs      generated from tmp108.ddsl, untouched
```

The boundaries are vocabulary, transport, the two drivers, and generated code. The
`maybe-async-cfg` dependency is removed.

### Shared vocabulary

`A0`, `Config` and `Error` stay public in `lib.rs` with their current shape. They gain the
pure logic extracted from the driver methods, which is what makes the duplicated methods
short enough to be readable:

```rust
impl From<Configuration> for Config             // reads tm/pol/cr/hys
impl Config { fn apply(self, r: &mut Configuration) }
impl ConversionRate { fn delay_us(self) -> u32 }
pub(crate) fn to_celsius(raw: i16) -> f32
pub(crate) fn to_raw(c: f32) -> i16
```

`Config::apply` is deliberately not `impl From<Config> for Configuration`. `configure` uses
`modify`, which must preserve the `m`, `fl`, `fh` and `id` fields that `Config` does not
model. A `From` impl would zero them.

### Interface layer

This layer loses its duplication entirely. `RegisterInterface` and `AsyncRegisterInterface`
are distinct traits, so both impls coexist on one type without a marker. `ErrorType` is a
supertrait of both HAL `I2c` traits, so a single base impl covers both modes:

```rust
pub(crate) struct Interface<I2C> { i2c: I2C, addr: u8 }

impl<I2C: embedded_hal::i2c::ErrorType> RegisterInterfaceBase for Interface<I2C> {
    type Error = I2C::Error;
    type AddressType = u8;
}
impl<I2C: embedded_hal::i2c::I2c> RegisterInterface for Interface<I2C> { ... }
#[cfg(feature = "async")]
impl<I2C: embedded_hal_async::i2c::I2c> AsyncRegisterInterface for Interface<I2C> { ... }
```

### The drivers

`blocking::Tmp108<I2C>` and `asynch::Tmp108<I2C>` are separate types with identical method
names and no generics beyond the bus. Ten bus-touching methods are duplicated between them;
with the pure logic extracted, each body is two or three lines:

```rust
pub fn read_configuration(&mut self) -> Result<Config, I2C::Error> {
    Ok(self.inner.configuration().read()?.into())
}
pub fn configure(&mut self, config: Config) -> Result<(), I2C::Error> {
    self.inner.configuration().modify(|r| config.apply(r))
}
```

`AlertTmp108` moves to `asynch` and stays async-only.

### Sensor-hal trait impls

`embedded_sensors_hal_async::sensor` re-exports `Error`, `ErrorKind` and `ErrorType` from
`embedded_sensors_hal` rather than declaring parallel traits. Today the two sets of impls
are kept apart by the `async` / `not(async)` gates. Once `async` is additive those gates
are gone, and two impls of the same trait for `Error<E>` would collide.

The fix is for the `embedded-sensors-hal-async` feature to imply `embedded-sensors-hal`.
This costs nothing, because the async crate already depends on the blocking crate
unconditionally, and it lets each shared trait be implemented exactly once:

- `sensor::Error for Error<E>` — one impl, gated on `feature = "embedded-sensors-hal"`.
- `sensor::ErrorType` — one impl per driver type, same gate, serving both the blocking and
  the async `TemperatureSensor`.
- `temperature::TemperatureSensor` — genuinely distinct traits in the two crates, so the
  blocking trait is implemented on `blocking::Tmp108` and the async trait on
  `asynch::Tmp108`.
- The threshold, hysteresis and wait traits are async-only and stay on `asynch::Tmp108` and
  `asynch::AlertTmp108`.

Every `not(feature = "async")` gate in the crate disappears.

### Features

`async` keeps its name and becomes additive. `embedded-sensors-hal-async` implies both
`embedded-sensors-hal` and `async`.

### Examples and tests

`examples/oneshot.rs` currently carries two `cfg`-gated `main` functions. It splits into
`oneshot.rs` and `oneshot_async.rs`, the latter with
`required-features = ["async"]`.

Fieldset unit tests stay in `lib.rs`. The `blocking` and `asynchronous` test modules move
into their respective driver files, and both compile together under `--features async`,
which they cannot do today.

## Out of scope

`continuous` remains async-only, as it is today. Adding a blocking counterpart is a
separate change.

## Breaking changes

- `tmp108::Tmp108` becomes `tmp108::blocking::Tmp108` or `tmp108::asynch::Tmp108`.
- `tmp108::AlertTmp108` becomes `tmp108::asynch::AlertTmp108`.
- `--features async` no longer removes the blocking API.

## Commit convention

From this change onward the repository uses Conventional Commits, so that a release
manager and a `release.yml` workflow can derive versions and changelog entries from
history. Breaking changes must be marked with `!` after the type and carry a
`BREAKING CHANGE:` footer, otherwise the tooling will cut a patch release for an API break.

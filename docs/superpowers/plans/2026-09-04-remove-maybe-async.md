# Remove `maybe-async-cfg` Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace `maybe-async-cfg` with two hand-written driver types, `tmp108::blocking::Tmp108` and `tmp108::asynch::Tmp108`, that can be used simultaneously in one build.

**Architecture:** Pure logic (unit conversion, `Config` mapping) moves to `lib.rs`. The I2C `Interface` becomes mode-independent, because `RegisterInterface` and `AsyncRegisterInterface` are distinct traits that can both be implemented on one type. The two drivers then become short, duplicated-but-readable wrappers in their own modules.

**Tech Stack:** Rust 2024, `device-driver` 2.1.0, `embedded-hal` 1.0, `embedded-hal-async` 1.0, `embedded-sensors-hal`, `embedded-hal-mock` (dev), `tokio` (dev).

**Design doc:** `docs/superpowers/specs/2026-09-04-remove-maybe-async-design.md`

**Commit convention:** Conventional Commits. This branch feeds a release manager, so a commit that changes the public API must use `!` and a `BREAKING CHANGE:` footer.

---

## File Structure

| File | Responsibility |
|---|---|
| `src/lib.rs` | Crate docs, module decls, `pub use` re-exports, `A0`, `Config`, `Error`, pure conversion helpers, fieldset unit tests |
| `src/interface.rs` | `Interface<I2C>` and its three register-interface impls. No mode split. |
| `src/blocking.rs` | `blocking::Tmp108`, its blocking sensor-hal impls, blocking tests |
| `src/asynch.rs` | `asynch::Tmp108`, `asynch::AlertTmp108`, async sensor-hal impls, async tests. Whole module `#[cfg(feature = "async")]`. |
| `src/inner.rs` | Generated from `tmp108.ddsl`. Never edited by hand. |
| `tests/public_api.rs` | External-consumer test. Guards the `pub use` re-exports, which in-crate tests cannot. |
| `examples/oneshot.rs` | Blocking example |
| `examples/oneshot_async.rs` | Async example, `required-features = ["async"]` |

---

### Task 1: Extract pure logic into `lib.rs`

Shrinks the driver method bodies so that duplicating them later is tolerable. `maybe-async-cfg` stays in place for now; this task is behaviour-preserving.

**Files:**
- Modify: `src/lib.rs`

- [ ] **Step 1: Write the failing tests**

Add these to the existing `mod tests` in `src/lib.rs`, immediately after the `reset_configuration` helper:

```rust
    #[test]
    fn config_round_trips_through_fieldset() {
        let config = Config {
            thermostat_mode: Thermostat::Interrupt,
            alert_polarity: Polarity::ActiveHigh,
            conversion_rate: ConversionRate::SixteenHz,
            hysteresis: Hysteresis::FourC,
        };

        let mut fieldset = reset_configuration();
        config.apply(&mut fieldset);

        assert_eq!(Config::from(fieldset), config);
    }

    #[test]
    fn apply_preserves_unmodelled_fields() {
        let mut fieldset = reset_configuration();
        fieldset.set_m(Mode::OneShot);
        fieldset.set_fl(true);
        fieldset.set_fh(true);
        fieldset.set_id(true);

        Config::default().apply(&mut fieldset);

        assert_eq!(fieldset.m().unwrap(), Mode::OneShot);
        assert!(fieldset.fl());
        assert!(fieldset.fh());
        assert!(fieldset.id());
    }

    #[test]
    fn conversion_rate_delays() {
        assert_eq!(ConversionRate::QuarterHz.delay_us(), 4_000_000);
        assert_eq!(ConversionRate::OneHz.delay_us(), 1_000_000);
        assert_eq!(ConversionRate::FourHz.delay_us(), 250_000);
        assert_eq!(ConversionRate::SixteenHz.delay_us(), 62_500);
    }

    #[test]
    fn celsius_conversions() {
        assert!((to_celsius(0x7ff0_u16 as i16) - 127.9375).abs() < 1e-4);
        assert!((to_celsius(0) - 0.0).abs() < 1e-4);
        assert!((to_celsius(0xc900_u16 as i16) + 55.0).abs() < 1e-4);
        assert_eq!(to_raw(127.9375), 0x7ff0_u16 as i16);
        assert_eq!(to_raw(-55.0), 0xc900_u16 as i16);
    }
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test --lib config_round_trips_through_fieldset`

Expected: FAIL to compile, `no method named 'apply' found for struct 'Config'`.

- [ ] **Step 3: Add the helpers**

In `src/lib.rs`, extend the `inner` import to bring in `Configuration`:

```rust
use crate::inner::{Configuration, ConversionRate, Hysteresis, Inner, Mode, Polarity, THigh, TLow, Thermostat};
```

Then add this block immediately after the `impl Default for Config` block:

```rust
impl From<Configuration> for Config {
    fn from(c: Configuration) -> Self {
        Self {
            thermostat_mode: c.tm(),
            alert_polarity: c.pol(),
            conversion_rate: c.cr(),
            hysteresis: c.hys(),
        }
    }
}

impl Config {
    /// Write the modelled fields onto a configuration fieldset.
    ///
    /// Deliberately not a `From` impl: `Config` does not model the `m`, `fl`,
    /// `fh` and `id` fields, and `configure` uses `modify`, so those must be
    /// left untouched.
    pub(crate) fn apply(self, r: &mut Configuration) {
        r.set_tm(self.thermostat_mode);
        r.set_pol(self.alert_polarity);
        r.set_cr(self.conversion_rate);
        r.set_hys(self.hysteresis);
    }
}

impl ConversionRate {
    /// Time to wait for one conversion to complete, in microseconds.
    pub(crate) fn delay_us(self) -> u32 {
        match self {
            ConversionRate::QuarterHz => 4_000_000,
            ConversionRate::OneHz => 1_000_000,
            ConversionRate::FourHz => 250_000,
            ConversionRate::SixteenHz => 62_500,
        }
    }
}

const CELSIUS_PER_BIT: f32 = 0.0625;

pub(crate) fn to_celsius(t: i16) -> f32 {
    f32::from(t / 16) * CELSIUS_PER_BIT
}

#[allow(clippy::cast_possible_truncation)]
pub(crate) fn to_raw(t: f32) -> i16 {
    (t * 16.0 / CELSIUS_PER_BIT) as i16
}
```

- [ ] **Step 4: Run the new tests to verify they pass**

Run: `cargo test --lib`

Expected: PASS, 16 tests.

- [ ] **Step 5: Rewrite the driver methods to use the helpers**

In `src/lib.rs`, delete the line `const CELSIUS_PER_BIT: f32 = 0.0625;` from the `impl<I2C: AsyncI2c> Tmp108<I2C>` block, and delete the `to_celsius` and `to_raw` associated functions at the end of that block.

Replace the body of `read_configuration` with:

```rust
    pub async fn read_configuration(&mut self) -> Result<Config, I2C::Error> {
        #[cfg(feature = "async")]
        let c = self.inner.configuration().read_async().await?;

        #[cfg(not(feature = "async"))]
        let c = self.inner.configuration().read()?;

        Ok(c.into())
    }
```

Replace the body of `configure` with:

```rust
    pub async fn configure(&mut self, config: Config) -> Result<(), I2C::Error> {
        #[cfg(feature = "async")]
        let res = self
            .inner
            .configuration()
            .modify_async(|r| config.apply(r))
            .await;

        #[cfg(not(feature = "async"))]
        let res = self.inner.configuration().modify(|r| config.apply(r));

        res
    }
```

Replace the `delay_time_us` match in `wait_for_temperature` with:

```rust
        let delay_time_us = config.conversion_rate.delay_us();
```

Replace the four remaining `Self::to_celsius(...)` call sites with `crate::to_celsius(...)`, and the one `Self::to_raw(...)` call site in each of `set_low_limit` and `set_high_limit` with `crate::to_raw(...)`.

- [ ] **Step 6: Verify the whole suite still passes in every mode**

Run each and expect PASS:

```
cargo test --locked
cargo test --locked -F async
cargo test --locked -F async,embedded-sensors-hal-async
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
cargo +nightly fmt --check
```

- [ ] **Step 7: Commit**

```bash
git add src/lib.rs
git commit -m "refactor(driver): extract pure conversion logic from driver methods

Move the Config-to-fieldset mapping, the conversion-rate delay table and
the raw-to-celsius conversions out of the driver methods and into free
functions and trait impls on the vocabulary types.

Config::apply is deliberately not a From impl. Config does not model the
m, fl, fh and id fields and configure uses modify, so a From impl would
zero them. A regression test covers that.

This shrinks each bus-touching method to two or three lines, which is
what makes the coming blocking/async split readable."
```

---

### Task 2: Make `Interface` mode-independent

`RegisterInterface` and `AsyncRegisterInterface` are distinct traits, so one `Interface` type can implement both. `ErrorType` is a supertrait of both HAL `I2c` traits, so a single `RegisterInterfaceBase` impl covers them. This deletes four of the seven `maybe_async_cfg` blocks.

**Files:**
- Create: `src/interface.rs`
- Modify: `src/lib.rs`

- [ ] **Step 1: Create the new module**

Create `src/interface.rs`:

```rust
//! I2C transport for the TMP108 register map.
//!
//! One type serves both modes. `RegisterInterface` and `AsyncRegisterInterface`
//! are distinct traits, so both impls coexist without a marker type, and
//! `embedded_hal::i2c::ErrorType` is a supertrait of both HAL `I2c` traits, so
//! a single `RegisterInterfaceBase` impl covers both.

use device_driver::{FieldsetMetadata, RegisterInterfaceBase};

/// Number of bytes in every TMP108 register.
const REGISTER_BYTES: usize = 2;

#[derive(Debug)]
pub(crate) struct Interface<I2C> {
    pub(crate) i2c: I2C,
    pub(crate) addr: u8,
}

impl<I2C> Interface<I2C> {
    pub(crate) const fn new(i2c: I2C, addr: u8) -> Self {
        Self { i2c, addr }
    }
}

impl<I2C: embedded_hal::i2c::ErrorType> RegisterInterfaceBase for Interface<I2C> {
    type Error = I2C::Error;
    type AddressType = u8;
}

impl<I2C: embedded_hal::i2c::I2c> device_driver::RegisterInterface for Interface<I2C> {
    fn write_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        let mut buf = [0; REGISTER_BYTES + 1];

        buf[0] = address;
        buf[1..].copy_from_slice(data);

        self.i2c.write(self.addr, &buf)
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.addr, &[address], data)
    }
}

#[cfg(feature = "async")]
impl<I2C: embedded_hal_async::i2c::I2c> device_driver::AsyncRegisterInterface for Interface<I2C> {
    async fn write_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        let mut buf = [0; REGISTER_BYTES + 1];

        buf[0] = address;
        buf[1..].copy_from_slice(data);

        self.i2c.write(self.addr, &buf).await
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.addr, &[address], data).await
    }
}
```

- [ ] **Step 2: Delete the old `Interface` from `lib.rs` and wire in the module**

In `src/lib.rs`, delete all four `Interface`-related `#[maybe_async_cfg::maybe(...)]` blocks: the `struct Interface`, the `impl Interface { fn new }`, the `impl RegisterInterfaceBase for Interface`, and the `impl AsyncRegisterInterface for Interface`.

Add the module declaration next to `mod inner;`:

```rust
mod interface;
```

Add the import next to the other `use crate::...` lines:

```rust
use crate::interface::Interface;
```

Delete these now-unused imports from the top of `lib.rs`:

```rust
#[cfg(feature = "async")]
use device_driver::AsyncRegisterInterface;
#[cfg(not(feature = "async"))]
use device_driver::RegisterInterface;
```

Keep the `embedded_hal::{delay::DelayNs, i2c::I2c}` and `embedded_hal_async::{...}` imports; the driver still uses them.

- [ ] **Step 3: Verify**

Run each and expect PASS:

```
cargo test --locked
cargo test --locked -F async
cargo test --locked -F async,embedded-sensors-hal-async
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
cargo +nightly fmt --check
```

If `clippy` reports `REGISTER_BYTES` unused, the `data` slice length assumption is wrong; check `src/inner.rs` for `size-bytes` and adjust.

- [ ] **Step 4: Commit**

```bash
git add src/interface.rs src/lib.rs
git commit -m "refactor(driver): give Interface a single mode-independent impl

RegisterInterface and AsyncRegisterInterface are distinct traits, so one
Interface type can implement both, and embedded_hal::i2c::ErrorType is a
supertrait of both HAL I2c traits, so one RegisterInterfaceBase impl
covers both modes.

This removes four of the seven maybe_async_cfg blocks and both copies of
the Interface struct, with no marker type and no cfg on the blocking
path."
```

---

### Task 3: Split the driver into `blocking` and `asynch`

The atomic part of the change. Both drivers must appear together, because `lib.rs` cannot be left without a driver.

**Files:**
- Create: `src/blocking.rs`
- Create: `src/asynch.rs`
- Modify: `src/lib.rs`
- Modify: `Cargo.toml`

- [ ] **Step 1: Write the failing coexistence test**

Create `tests/coexistence.rs`. This is the new capability, and it cannot compile today:

```rust
//! Both drivers must be usable from one binary. Before the blocking/async
//! split this could not compile, because `async` replaced the blocking API
//! rather than adding to it.

#![cfg(feature = "async")]

use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

#[tokio::test]
async fn blocking_and_async_drivers_coexist() {
    let expectations = [Transaction::write_read(0x48, vec![0x00], vec![0x19, 0x00])];

    let mut blocking = tmp108::blocking::Tmp108::new_with_a0_gnd(Mock::new(&expectations));
    let temperature = blocking.temperature().unwrap();
    assert!((temperature - 25.0).abs() < 1e-4);
    let mut mock = blocking.destroy();
    mock.done();

    let mut asynchronous = tmp108::asynch::Tmp108::new_with_a0_gnd(Mock::new(&expectations));
    let temperature = asynchronous.temperature().await.unwrap();
    assert!((temperature - 25.0).abs() < 1e-4);
    let mut mock = asynchronous.destroy();
    mock.done();
}
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cargo test --locked -F async --test coexistence`

Expected: FAIL to compile, `could not find 'blocking' in 'tmp108'`.

- [ ] **Step 3: Create `src/blocking.rs`**

```rust
//! Blocking TMP108 driver.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::inner::{Inner, Mode, THigh, TLow};
use crate::interface::Interface;
use crate::{A0, Config, to_celsius, to_raw};

/// Blocking TMP108 device driver.
#[derive(Debug)]
pub struct Tmp108<I2C: I2c> {
    inner: Inner<Interface<I2C>>,
    addr: u8,
}

impl<I2C: I2c> Tmp108<I2C> {
    /// Create a new TMP108 instance.
    pub fn new(i2c: I2C, a0: A0) -> Self {
        let addr = a0.into();

        Self {
            inner: Inner::new(Interface::new(i2c, addr)),
            addr,
        }
    }

    /// Create a new TMP108 instance with A0 tied to GND, resulting in
    /// an instance responding to address `0x48`.
    pub fn new_with_a0_gnd(i2c: I2C) -> Self {
        Self::new(i2c, A0::Gnd)
    }

    /// Create a new TMP108 instance with A0 tied to V+, resulting in
    /// an instance responding to address `0x49`.
    pub fn new_with_a0_vplus(i2c: I2C) -> Self {
        Self::new(i2c, A0::Vplus)
    }

    /// Create a new TMP108 instance with A0 tied to SDA, resulting in
    /// an instance responding to address `0x4a`.
    pub fn new_with_a0_sda(i2c: I2C) -> Self {
        Self::new(i2c, A0::Sda)
    }

    /// Create a new TMP108 instance with A0 tied to SCL, resulting in
    /// an instance responding to address `0x4b`.
    pub fn new_with_a0_scl(i2c: I2C) -> Self {
        Self::new(i2c, A0::Scl)
    }

    /// Get the current I2C address
    pub fn addr(&self) -> u8 {
        self.addr
    }

    /// Destroy the driver instance, return the I2C bus instance.
    pub fn destroy(self) -> I2C {
        self.inner.free().i2c
    }

    /// Read configuration register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn read_configuration(&mut self) -> Result<Config, I2C::Error> {
        Ok(self.inner.configuration().read()?.into())
    }

    /// Configure device parameters.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn configure(&mut self, config: Config) -> Result<(), I2C::Error> {
        self.inner.configuration().modify(|r| config.apply(r))
    }

    /// Read the temperature sensor
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn temperature(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.inner.temperature().read()?;
        Ok(to_celsius(i16::from_be_bytes(raw.into())))
    }

    /// Configure device for one-shot conversion
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn one_shot(&mut self) -> Result<(), I2C::Error> {
        self.inner.configuration().modify(|r| r.set_m(Mode::OneShot))
    }

    /// Place device in shutdown mode
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn shutdown(&mut self) -> Result<(), I2C::Error> {
        self.inner.configuration().modify(|r| r.set_m(Mode::Shutdown))
    }

    /// Wait for conversion to complete. This method will block for the amount
    /// of time dictated by the CR bits in the configuration register.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn wait_for_temperature<DELAY: DelayNs>(&mut self, delay: &mut DELAY) -> Result<f32, I2C::Error> {
        let config = self.read_configuration()?;

        delay.delay_us(config.conversion_rate.delay_us());
        self.temperature()
    }

    /// Read temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn low_limit(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.inner.t_low().read()?;
        Ok(to_celsius(i16::from_be_bytes(raw.into())))
    }

    /// Set temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn set_low_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
        let raw = to_raw(limit).to_be_bytes();
        self.inner.t_low().write(|r| *r = TLow::from(raw))
    }

    /// Read temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn high_limit(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.inner.t_high().read()?;
        Ok(to_celsius(i16::from_be_bytes(raw.into())))
    }

    /// Set temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn set_high_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
        let raw = to_raw(limit).to_be_bytes();
        self.inner.t_high().write(|r| *r = THigh::from(raw))
    }
}

#[cfg(feature = "embedded-sensors-hal")]
impl<I2C: I2c> embedded_sensors_hal::sensor::ErrorType for Tmp108<I2C> {
    type Error = crate::Error<I2C::Error>;
}

#[cfg(feature = "embedded-sensors-hal")]
impl<I2C: I2c> embedded_sensors_hal::temperature::TemperatureSensor for Tmp108<I2C> {
    fn temperature(&mut self) -> Result<embedded_sensors_hal::temperature::DegreesCelsius, Self::Error> {
        self.temperature().map_err(crate::Error::Bus)
    }
}
```

- [ ] **Step 4: Move the blocking tests into `src/blocking.rs`**

Cut the `mod blocking { ... }` block, and the `#[cfg(not(feature = "async"))]` attribute above it, out of `mod tests` in `src/lib.rs`. It starts at line 781 and runs to the line before `#[cfg(feature = "async")] mod asynchronous {` at line 978. Append it to `src/blocking.rs`, changing the header from

```rust
    #[cfg(not(feature = "async"))]
    mod blocking {
        use assert_approx_eq::assert_approx_eq;
        use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

        use super::*;
```

to

```rust
#[cfg(test)]
mod tests {
    use assert_approx_eq::assert_approx_eq;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    use super::*;
    use crate::{Config, ConversionRate, Hysteresis, Polarity, Thermostat};
```

Dedent the moved block by four spaces. The test bodies are otherwise unchanged.

- [ ] **Step 5: Verify the blocking half**

Run: `cargo test --locked`

Expected: PASS. The crate root still has the `maybe-async` driver at this point, so both exist; that is fine and temporary.

- [ ] **Step 6: Create `src/asynch.rs`**

Move the async driver across. Take the body of every method from the current `lib.rs` `#[cfg(feature = "async")]` branches, dropping the `#[cfg(not(feature = "async"))]` branches:

```rust
//! Asynchronous TMP108 driver.

use core::future::Future;

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

use crate::inner::{Inner, Mode, THigh, TLow};
use crate::interface::Interface;
use crate::{A0, Config, to_celsius, to_raw};

/// Asynchronous TMP108 device driver.
#[derive(Debug)]
pub struct Tmp108<I2C: I2c> {
    inner: Inner<Interface<I2C>>,
    addr: u8,
}

impl<I2C: I2c> Tmp108<I2C> {
    /// Create a new TMP108 instance.
    pub fn new(i2c: I2C, a0: A0) -> Self {
        let addr = a0.into();

        Self {
            inner: Inner::new(Interface::new(i2c, addr)),
            addr,
        }
    }

    /// Create a new TMP108 instance with A0 tied to GND, resulting in
    /// an instance responding to address `0x48`.
    pub fn new_with_a0_gnd(i2c: I2C) -> Self {
        Self::new(i2c, A0::Gnd)
    }

    /// Create a new TMP108 instance with A0 tied to V+, resulting in
    /// an instance responding to address `0x49`.
    pub fn new_with_a0_vplus(i2c: I2C) -> Self {
        Self::new(i2c, A0::Vplus)
    }

    /// Create a new TMP108 instance with A0 tied to SDA, resulting in
    /// an instance responding to address `0x4a`.
    pub fn new_with_a0_sda(i2c: I2C) -> Self {
        Self::new(i2c, A0::Sda)
    }

    /// Create a new TMP108 instance with A0 tied to SCL, resulting in
    /// an instance responding to address `0x4b`.
    pub fn new_with_a0_scl(i2c: I2C) -> Self {
        Self::new(i2c, A0::Scl)
    }

    /// Get the current I2C address
    pub fn addr(&self) -> u8 {
        self.addr
    }

    /// Destroy the driver instance, return the I2C bus instance.
    pub fn destroy(self) -> I2C {
        self.inner.free().i2c
    }

    /// Create a new `AlertTmp108` instance by consuming the original instance.
    #[cfg(feature = "embedded-sensors-hal-async")]
    pub fn into_alert<ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>(
        self,
        alert: ALERT,
    ) -> AlertTmp108<I2C, ALERT> {
        AlertTmp108 { tmp108: self, alert }
    }

    /// Read configuration register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn read_configuration(&mut self) -> Result<Config, I2C::Error> {
        Ok(self.inner.configuration().read_async().await?.into())
    }

    /// Configure device parameters.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn configure(&mut self, config: Config) -> Result<(), I2C::Error> {
        self.inner
            .configuration()
            .modify_async(|r| config.apply(r))
            .await
    }

    /// Read the temperature sensor
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn temperature(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.inner.temperature().read_async().await?;
        Ok(to_celsius(i16::from_be_bytes(raw.into())))
    }

    /// Configure device for one-shot conversion
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn one_shot(&mut self) -> Result<(), I2C::Error> {
        self.inner
            .configuration()
            .modify_async(|r| r.set_m(Mode::OneShot))
            .await
    }

    /// Place device in shutdown mode
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn shutdown(&mut self) -> Result<(), I2C::Error> {
        self.inner
            .configuration()
            .modify_async(|r| r.set_m(Mode::Shutdown))
            .await
    }

    /// Initiate continuous conversions
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn continuous<F, Fut>(&mut self, f: F) -> Result<(), I2C::Error>
    where
        F: FnOnce(&mut Self) -> Fut,
        Fut: Future<Output = Result<(), I2C::Error>> + Send,
    {
        self.inner
            .configuration()
            .modify_async(|r| r.set_m(Mode::Continuous))
            .await?;

        f(self).await?;
        self.shutdown().await
    }

    /// Wait for conversion to complete. This method will wait for the amount
    /// of time dictated by the CR bits in the configuration register.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn wait_for_temperature<DELAY: DelayNs>(&mut self, delay: &mut DELAY) -> Result<f32, I2C::Error> {
        let config = self.read_configuration().await?;

        delay.delay_us(config.conversion_rate.delay_us()).await;
        self.temperature().await
    }

    /// Read temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn low_limit(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.inner.t_low().read_async().await?;
        Ok(to_celsius(i16::from_be_bytes(raw.into())))
    }

    /// Set temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn set_low_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
        let raw = to_raw(limit).to_be_bytes();
        self.inner.t_low().write_async(|r| *r = TLow::from(raw)).await
    }

    /// Read temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn high_limit(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.inner.t_high().read_async().await?;
        Ok(to_celsius(i16::from_be_bytes(raw.into())))
    }

    /// Set temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn set_high_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
        let raw = to_raw(limit).to_be_bytes();
        self.inner.t_high().write_async(|r| *r = THigh::from(raw)).await
    }
}
```

- [ ] **Step 7: Move `AlertTmp108` and the async sensor-hal impls into `src/asynch.rs`**

Move these items from `src/lib.rs` verbatim. Line numbers are positions *before any edit in this task*, given for orientation only; match on the item signature, not the line:

| Item | Approx. line |
|---|---|
| `pub struct AlertTmp108<..>` | 154 |
| `impl<..> AlertTmp108<I2C, ALERT>` (constructors, `destroy`) | 164 |
| `impl<..> embedded_sensors_hal_async::sensor::ErrorType for Tmp108<I2C>` | 543 |
| `impl<..> embedded_sensors_hal_async::temperature::TemperatureSensor for Tmp108<I2C>` | 548 |
| `impl<..> embedded_sensors_hal_async::sensor::ErrorType for AlertTmp108<..>` | 555 |
| `impl<..> embedded_sensors_hal_async::temperature::TemperatureSensor for AlertTmp108<..>` | 562 |
| `impl<..> TemperatureThresholdSet for Tmp108<I2C>` | 571 |
| `impl<..> TemperatureThresholdSet for AlertTmp108<..>` | 590 |
| `impl<..> TemperatureThresholdWait for AlertTmp108<..>` | 609 |
| `impl<..> TemperatureHysteresis for Tmp108<I2C>` | 654 |
| `impl<..> TemperatureHysteresis for AlertTmp108<..>` | 682 |

For each moved block:

- Change the gate from `#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]` to `#[cfg(feature = "embedded-sensors-hal-async")]`, because the whole module already sits behind `feature = "async"`.
- Replace bare `Error::` with `crate::Error::`, and `Thermostat::` / `Polarity::` / `Hysteresis::` with their `crate::`-qualified forms.
- Replace the `I2C: embedded_hal_async::i2c::I2c` bounds with `I2C: I2c`, since `asynch.rs` imports `embedded_hal_async::i2c::I2c` directly.

Then replace the two `ErrorType` impls with the shared-trait form. `embedded_sensors_hal_async::sensor::ErrorType` is a re-export of `embedded_sensors_hal::sensor::ErrorType`, not a parallel trait, so name the blocking crate's path:

```rust
#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: I2c> embedded_sensors_hal::sensor::ErrorType for Tmp108<I2C> {
    type Error = crate::Error<I2C::Error>;
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal::sensor::ErrorType for AlertTmp108<I2C, ALERT>
{
    type Error = crate::Error<I2C::Error>;
}
```

Do **not** move `impl<E> embedded_sensors_hal_async::sensor::Error for Error<E>` (line 536). It is deleted in Step 9, because it duplicates the blocking impl of the same trait.

- [ ] **Step 8: Move the async tests into `src/asynch.rs`**

Cut the `mod asynchronous { ... }` block, and the `#[cfg(feature = "async")]` attribute above it, out of `mod tests` in `src/lib.rs`. It starts at line 978 and runs to the closing brace of `mod tests`. Append it to `src/asynch.rs` as:

```rust
#[cfg(test)]
mod tests {
    use assert_approx_eq::assert_approx_eq;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    use super::*;
    use crate::{Config, ConversionRate, Hysteresis, Polarity, Thermostat};
```

Dedent by four spaces. Leave the `#[cfg(feature = "embedded-sensors-hal-async")]` attribute on `handle_threshold_alerts_properly` in place; the module is behind `async` but not behind the sensor feature.

- [ ] **Step 9: Strip the old driver out of `lib.rs`**

Delete from `src/lib.rs`, matching on item signature rather than line number since earlier steps have shifted things:

- `#[maybe_async_cfg::maybe(..)] pub struct Tmp108<I2C: AsyncI2c>` and the two `impl<I2C: AsyncI2c> Tmp108<I2C>` blocks it accompanies (originally lines 85-150 and 203-429),
- `pub struct AlertTmp108<..>` and `impl<..> AlertTmp108<I2C, ALERT>` (originally 153-201), now in `asynch.rs`,
- `impl<..> embedded_sensors_hal::sensor::ErrorType for Tmp108<I2C>` and `impl<..> embedded_sensors_hal::temperature::TemperatureSensor for Tmp108<I2C>` (originally 524 and 529), now in `blocking.rs`,
- every `embedded_sensors_hal_async` impl block (originally 535-692), now in `asynch.rs`,
- `impl<E> embedded_sensors_hal_async::sensor::Error for Error<E>` (originally 536), which duplicates the blocking impl of the same re-exported trait,
- the `mod blocking` and `mod asynchronous` test modules, now moved,
- the imports `core::future::Future`, `embedded_hal::{delay::DelayNs, i2c::I2c}` and `embedded_hal_async::{delay::DelayNs as AsyncDelayNs, i2c::I2c as AsyncI2c}`.

Keep the `Error<E>` enum. Change the gate on its remaining `sensor::Error` impl from `#[cfg(all(feature = "embedded-sensors-hal", not(feature = "async")))]` to:

```rust
#[cfg(feature = "embedded-sensors-hal")]
impl<E: embedded_hal::i2c::Error> embedded_sensors_hal::sensor::Error for Error<E> {
    fn kind(&self) -> embedded_sensors_hal::sensor::ErrorKind {
        embedded_sensors_hal::sensor::ErrorKind::Other
    }
}
```

Add the module declarations and re-exports near the top, alongside the `mod interface;` added in Task 2:

```rust
#[cfg(feature = "async")]
pub mod asynch;
pub mod blocking;

pub use crate::inner::{ConversionRate, Hysteresis, Mode, Polarity, Thermostat};
```

Reduce the private `use crate::inner::{...}` line to only what `lib.rs` still needs:

```rust
use crate::inner::Configuration;
```

`Inner`, `THigh` and `TLow` are no longer referenced from `lib.rs`; the drivers import them directly.

- [ ] **Step 10: Update `Cargo.toml`**

Remove the dependency:

```toml
maybe-async-cfg = "0.2.5"
```

Replace the `[features]` section with:

```toml
[features]
async = [ "dep:embedded-hal-async" ]
embedded-sensors-hal = [ "dep:embedded-sensors-hal" ]
embedded-sensors-hal-async = [ "dep:embedded-sensors-hal-async", "embedded-sensors-hal", "async" ]
```

- [ ] **Step 11: Run the coexistence test**

Run: `cargo test --locked -F async --test coexistence`

Expected: PASS.

- [ ] **Step 12: Verify every mode**

Run each and expect PASS:

```
cargo test --locked
cargo test --locked -F async
cargo test --locked -F async,embedded-sensors-hal-async
cargo test --locked --all-features
cargo hack --feature-powerset check --locked
cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style
cargo +nightly fmt --check
```

- [ ] **Step 13: Confirm `maybe-async-cfg` is gone**

Run: `cargo tree --locked -i maybe-async-cfg`

Expected: `error: package ID specification 'maybe-async-cfg' did not match any packages`.

- [ ] **Step 14: Commit**

```bash
git add src/lib.rs src/blocking.rs src/asynch.rs tests/coexistence.rs Cargo.toml Cargo.lock
git commit -m "refactor(driver)!: replace maybe-async-cfg with blocking and asynch modules

Split Tmp108 into two hand-written types, blocking::Tmp108 and
asynch::Tmp108, and drop the maybe-async-cfg dependency. With the pure
logic already extracted, each bus-touching method is two or three lines,
so duplicating the ten of them costs less than the macro did.

An embassy-style Mode marker generic was considered and rejected. There
the marker earns its place because construction differs per mode; TMP108
constructs identically either way, so a marker would add a public type
parameter and sealed-trait boilerplate without removing a single line of
duplicated body.

The async feature is now additive rather than a global switch, so both
drivers can be used from one binary. A new integration test covers that.
embedded-sensors-hal-async now implies embedded-sensors-hal, because the
async crate re-exports sensor::Error rather than declaring a parallel
trait, and two impls of one trait for Error<E> would otherwise collide.

BREAKING CHANGE: tmp108::Tmp108 is now tmp108::blocking::Tmp108 or
tmp108::asynch::Tmp108, and tmp108::AlertTmp108 is now
tmp108::asynch::AlertTmp108. Enabling the async feature no longer
removes the blocking API."
```

---

### Task 4: Guard the public API surface

The missing `pub use` was invisible to in-crate tests. Integration tests under `tests/` compile as external consumers, so they can see it.

**Files:**
- Create: `tests/public_api.rs`

- [ ] **Step 1: Write the failing test**

Create `tests/public_api.rs`:

```rust
//! Compiled as an external consumer, so it sees exactly what downstream
//! users see. In-crate tests cannot catch a missing `pub use`, which is
//! how the generated enums stayed unreachable for several releases.

use tmp108::{Config, ConversionRate, Hysteresis, Polarity, Thermostat};

#[test]
fn config_is_constructible_by_downstream_users() {
    let config = Config {
        thermostat_mode: Thermostat::Interrupt,
        alert_polarity: Polarity::ActiveHigh,
        conversion_rate: ConversionRate::SixteenHz,
        hysteresis: Hysteresis::FourC,
    };

    assert_ne!(config, Config::default());
}

#[test]
fn config_fields_are_matchable_by_downstream_users() {
    let delay_ms = match Config::default().conversion_rate {
        ConversionRate::QuarterHz => 4000,
        ConversionRate::OneHz => 1000,
        ConversionRate::FourHz => 250,
        ConversionRate::SixteenHz => 62,
    };

    assert_eq!(delay_ms, 1000);
}
```

- [ ] **Step 2: Run it**

Run: `cargo test --locked --test public_api`

Expected: PASS, because Task 3 Step 9 added the `pub use`. If it fails with `unresolved import`, the `pub use` line is missing from `src/lib.rs`.

- [ ] **Step 3: Verify `Mode` is reachable too**

Run: `cargo test --locked --test public_api`, after appending:

```rust
#[test]
fn mode_is_reachable() {
    assert_ne!(tmp108::Mode::OneShot, tmp108::Mode::Shutdown);
}
```

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add tests/public_api.rs
git commit -m "test: guard the public API against unreachable types

Integration tests compile as external consumers, so they see what
downstream users see. In-crate tests cannot detect a missing pub use,
which is how ConversionRate, Hysteresis, Mode, Polarity and Thermostat
stayed unnameable from outside the crate while appearing in Config's
public fields."
```

---

### Task 5: Split the example

**Files:**
- Modify: `examples/oneshot.rs`
- Create: `examples/oneshot_async.rs`
- Modify: `Cargo.toml`

- [ ] **Step 1: Rewrite `examples/oneshot.rs` as blocking-only**

```rust
//! TMP108 blocking temperature read example.

use anyhow::{Result, anyhow};
use pico_de_gallo_hal::Hal;
use tmp108::blocking::Tmp108;

fn main() -> Result<()> {
    let hal = Hal::new();
    let i2c = hal.i2c();

    let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    let temperature = tmp.temperature().map_err(|_| anyhow!("Failed to read temperature"))?;
    println!("Temperature: {temperature:.2} C");

    Ok(())
}
```

- [ ] **Step 2: Create `examples/oneshot_async.rs`**

```rust
//! TMP108 asynchronous temperature read example.

use anyhow::{Result, anyhow};
use pico_de_gallo_hal::Hal;
use tmp108::asynch::Tmp108;

#[tokio::main]
async fn main() -> Result<()> {
    let hal = Hal::new();
    let i2c = hal.i2c();

    let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    let temperature = tmp
        .temperature()
        .await
        .map_err(|_| anyhow!("Failed to read temperature"))?;
    println!("Temperature: {temperature:.2} C");

    Ok(())
}
```

- [ ] **Step 3: Declare the async example's feature requirement**

Append to `Cargo.toml`:

```toml
[[example]]
name = "oneshot_async"
required-features = ["async"]
```

- [ ] **Step 4: Verify both examples build**

Run: `cargo build --locked --example oneshot`

Expected: PASS.

Run: `cargo build --locked --example oneshot_async -F async`

Expected: PASS.

Run: `cargo build --locked --examples`

Expected: PASS, building only `oneshot`, since `async` is off.

- [ ] **Step 5: Commit**

```bash
git add examples/oneshot.rs examples/oneshot_async.rs Cargo.toml
git commit -m "docs(examples): split oneshot into blocking and async examples

The single example carried two cfg-gated main functions because the
async feature used to replace the blocking API. Now that both APIs
coexist, each example targets one of them, and the async one declares
required-features."
```

---

### Task 6: Fix the README

The usage block is `rust,ignore`, so it has never been compiled and no longer resembles the crate. Making it a real doctest stops it rotting again. The MSRV line is also stale, left over from the device-driver v2 migration.

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Replace the usage section**

Replace lines 27 to 72 of `README.md`, that is the `## Usage` heading and the whole fenced block, with:

````markdown
## Usage

```rust
use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
use tmp108::blocking::Tmp108;
use tmp108::{Config, ConversionRate, Hysteresis, Polarity, Thermostat};

# fn main() -> Result<(), Box<dyn std::error::Error>> {
# let expectations = [
#     // read_configuration
#     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
#     // configure -> modify: read, then write
#     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
#     Transaction::write(0x48, vec![0x01, 0x66, 0xb0]),
#     // set_low_limit(26.5) -> 26.5 * 256 = 0x1a80
#     Transaction::write(0x48, vec![0x02, 0x1a, 0x80]),
#     // set_high_limit(48.0) -> 48 * 256 = 0x3000
#     Transaction::write(0x48, vec![0x03, 0x30, 0x00]),
#     // temperature -> 0x1900 = 6400, 6400 / 16 * 0.0625 = 25.0
#     Transaction::write_read(0x48, vec![0x00], vec![0x19, 0x00]),
# ];
# let i2c = Mock::new(&expectations);
let mut tmp = Tmp108::new_with_a0_gnd(i2c);
// let mut tmp = Tmp108::new_with_a0_vplus(i2c);
// let mut tmp = Tmp108::new_with_a0_sda(i2c);
// let mut tmp = Tmp108::new_with_a0_scl(i2c);
// let mut tmp = Tmp108::new(i2c, A0::Gnd);

let mut config = tmp.read_configuration()?;

config.thermostat_mode = Thermostat::Interrupt;
config.alert_polarity = Polarity::ActiveHigh;
config.conversion_rate = ConversionRate::SixteenHz;
config.hysteresis = Hysteresis::FourC;

tmp.configure(config)?;

tmp.set_low_limit(26.5)?;
tmp.set_high_limit(48.0)?;

let temperature = tmp.temperature()?;
println!("Temperature: {temperature:.2} C");
# let mut i2c = tmp.destroy();
# i2c.done();
# Ok(())
# }
```

The asynchronous driver lives in `tmp108::asynch` behind the `async`
feature and mirrors the blocking API method for method.
````

- [ ] **Step 2: Run the doctest to check the mock expectations**

Run: `cargo test --locked --doc`

Expected: PASS, 1 test.

If it fails with an unexpected-transaction panic, the mock expectation bytes are wrong. Read the panic message, which prints the transaction actually attempted, and correct the byte sequence in the hidden `expectations` array. The four config bytes derive from `Config`, so recompute rather than guess: `0x66, 0xb0` is the existing value used by `change_configuration` in `src/blocking.rs`.

- [ ] **Step 3: Fix the MSRV line**

Replace lines 74 to 77 of `README.md`:

```markdown
## MSRV

Currently, rust `1.94` and up is supported, but some previous versions
may work.
```

- [ ] **Step 4: Verify MSRV agreement**

Run: `Select-String -Path README.md,Cargo.toml,.github/workflows/check.yml -Pattern "1\.94|1\.85"`

Expected: every hit says `1.94`; no hit says `1.85`.

- [ ] **Step 5: Commit**

```bash
git add README.md
git commit -m "docs: rewrite the README example and correct the MSRV

The usage block was marked rust,ignore, so it was never compiled and had
rotted badly: it showed a builder API that does not exist, a
two-argument constructor, and type names such as ConversionMode and
Hysteresis::FourCelsius with no counterpart in the crate.

Rewrite it against the current API and drop the ignore marker so it is
compiled as a doctest, using embedded-hal-mock for the bus. It cannot
rot again without failing CI.

Also correct the MSRV, which the device-driver 2.1.0 migration raised to
1.94 without updating the README."
```

---

### Task 7: Full verification sweep

**Files:** none

- [ ] **Step 1: Reproduce every CI job locally**

Run:

```powershell
$ok=$true
function Run($name,$cmd){ $out = Invoke-Expression "$cmd 2>&1"; if($LASTEXITCODE -ne 0){ $script:ok=$false; Write-Output "FAIL: $name"; $out | Select-Object -Last 20 } else { Write-Output "PASS: $name" } }
Run "fmt"              "cargo +nightly fmt --check"
Run "clippy"           "cargo clippy --all-features --all-targets -- -W clippy::suspicious -W clippy::correctness -W clippy::perf -W clippy::style"
Run "doc"              "cargo doc --no-deps --all-features --locked"
Run "hack powerset"    "cargo hack --feature-powerset check --locked"
Run "msrv 1.94"        "cargo +1.94.0 check --locked"
Run "test blocking"    "cargo test --locked"
Run "test async"       "cargo test --locked -F async"
Run "test sensors"     "cargo test --locked -F async,embedded-sensors-hal-async"
Run "test all"         "cargo test --locked --all-features"
Run "nostd thumbv6m"   "cargo check --target thumbv6m-none-eabi --no-default-features --locked"
Run "nostd thumbv7em"  "cargo check --target thumbv7em-none-eabihf --no-default-features --locked"
Run "nostd thumbv8m"   "cargo check --target thumbv8m.main-none-eabihf --no-default-features --locked"
Run "package"          "cargo package --locked"
Write-Output "ALL PASS: $ok"
```

Expected: `ALL PASS: True`.

- [ ] **Step 2: Confirm the working tree is clean**

Run: `git status --short`

Expected: no output.

- [ ] **Step 3: Review the commit series**

Run: `git log --oneline rolling..HEAD`

Expected, in order:

```
docs: rewrite the README example and correct the MSRV
docs(examples): split oneshot into blocking and async examples
test: guard the public API against unreachable types
refactor(driver)!: replace maybe-async-cfg with blocking and asynch modules
refactor(driver): give Interface a single mode-independent impl
refactor(driver): extract pure conversion logic from driver methods
docs: add design for removing maybe-async-cfg
feat(driver)!: migrate to device-driver 2.1.0 and DDSL
```

Every subject line must parse as a Conventional Commit, and both API-breaking commits must carry a `BREAKING CHANGE:` footer. Verify with:

Run: `git log rolling..HEAD --format=%B | Select-String "BREAKING CHANGE"`

Expected: two hits.

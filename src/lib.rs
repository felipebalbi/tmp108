//! This is a platform-agnostic Rust driver for the TMP108 temperature sensor
//! based on the [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal
//!
//! For further details of the device architecture and operation, please refer
//! to the official [`Datasheet`].
//!
//! [`Datasheet`]: https://www.ti.com/lit/gpn/tmp108
//!
//! # Operational notes
//!
//! ## I²C bus ownership
//!
//! The driver assumes single-master ownership of the TMP108. Several
//! methods (notably `configure`, `one_shot`, and `shutdown` on both
//! [`Tmp108`] and [`AsyncTmp108`]) perform a read-modify-write on the
//! configuration register as two distinct I²C transactions. On a
//! multi-master bus, a second master writing to register `0x01`
//! between the read and the write will silently lose those writes —
//! the TMP108 has no register-level lock. In multi-master designs,
//! serialise driver access at a higher level (e.g. a bus mutex around
//! the entire driver, not just individual I²C transactions).
//!
//! ## Driver lifecycle on drop
//!
//! Dropping a [`Tmp108`], [`AsyncTmp108`] or [`AlertTmp108`] does
//! **not** change the chip's operating mode. The chip retains whatever
//! `M` bits were last written. If you want the chip to stop drawing
//! current after the driver goes out of scope, call
//! [`Tmp108::shutdown`] or [`AsyncTmp108::shutdown`] (or finish an
//! [`AsyncTmp108::continuous`] call cleanly) before dropping.
//!
//! In particular, dropping the future returned by
//! [`AsyncTmp108::continuous`] mid-flight (e.g. via
//! `embassy_futures::select!` or `tokio::time::timeout`) leaves the
//! chip in `Mode::Continuous` indefinitely. See the cancel-safety note
//! on that method.

#![doc(html_root_url = "https://docs.rs/tmp108/latest")]
#![doc = include_str!("../README.md")]
#![cfg_attr(not(test), no_std)]

#[cfg(feature = "async")]
use device_driver::AsyncRegisterInterface;
use device_driver::{FieldsetMetadata, RegisterInterface, RegisterInterfaceBase};
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::delay::DelayNs as AsyncDelayNs;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c as AsyncI2c;

#[allow(clippy::all)]
#[allow(clippy::pedantic)]
#[allow(missing_docs)]
#[allow(unsafe_code)]
#[allow(unused)]
mod inner;

pub use crate::inner::{ConversionRate, Hysteresis, Mode, Polarity, Thermostat};
use crate::inner::{Inner, THigh, TLow};
pub use crate::ops::{Celsius, OutOfRange};

/// A0 pin logic level representation.
#[derive(Debug, Default)]
pub enum A0 {
    /// A0 tied to GND (default).
    #[default]
    Gnd,
    /// A0 tied to V+.
    Vplus,
    /// A0 tied to SDA.
    Sda,
    /// A0 tied to SCL.
    Scl,
}

impl From<A0> for u8 {
    fn from(connection: A0) -> Self {
        match connection {
            A0::Gnd => 0b100_1000,
            A0::Vplus => 0b100_1001,
            A0::Sda => 0b100_1010,
            A0::Scl => 0b100_1011,
        }
    }
}

/// Tmp108 configuration parameters
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Config {
    /// Thermostat mode.
    pub thermostat_mode: Thermostat,
    /// Alert pin polarity.
    pub alert_polarity: Polarity,
    /// Conversion rate.
    pub conversion_rate: ConversionRate,
    /// Temperature hysteresis.
    pub hysteresis: Hysteresis,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            thermostat_mode: Thermostat::Comparator,
            alert_polarity: Polarity::ActiveLow,
            conversion_rate: ConversionRate::OneHz,
            hysteresis: Hysteresis::OneC,
        }
    }
}

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
/// This type is always available, but the driver's only current producer
/// of it — `AlertTmp108::wait_for_alert` — requires the
/// `embedded-sensors-hal-async` feature.
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

/// Pure-function register codec.
///
/// All sync/async-agnostic chip logic lives here: the [`Celsius`]
/// temperature newtype and its register codec, snapping of
/// continuous-f32 hysteresis values to the four discrete chip
/// settings, and the conversion between the typed [`Config`] and the
/// generated `Configuration` field-set.
///
/// The blocking and async drivers both delegate to this module so the
/// meaningful work lives in exactly one place; the per-driver methods
/// are thin shells that perform I²C and call into `ops`.
///
/// This module is pure: no bus, no `async`, no HAL. Every function
/// here is total or explicitly fallible, and the domains are small
/// enough that the tests walk them exhaustively rather than sampling.
pub(crate) mod ops {
    #[cfg(feature = "embedded-sensors-hal-async")]
    use crate::AlertCause;
    use crate::Config;
    #[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
    use crate::Hysteresis;
    use crate::inner::Configuration;

    /// Documented power-on reset value of the configuration register.
    /// Used by [`crate::Tmp108::probe`] to verify chip presence.
    pub(crate) const POR_CONFIG: u16 = 0x1022;

    /// Tolerance band for snapping continuous-f32 hysteresis input
    /// to the four discrete chip settings, in °C.
    #[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
    pub(crate) const HYSTERESIS_TOLERANCE: f32 = 0.05;

    /// A temperature the TMP108 can represent.
    ///
    /// Stored as sixteenths of a degree Celsius, which is the part's
    /// native resolution: one LSB is 0.0625 °C. The representable
    /// range is the span of the sensor's 12-bit two's complement
    /// field, `-2048..=2047` sixteenths, or [`Celsius::MIN`] to
    /// [`Celsius::MAX`].
    ///
    /// That is 4096 inhabitants, every one of them a temperature the
    /// part can actually report or accept as a limit. An `f32` in the
    /// same position would admit roughly four billion, including
    /// `NaN`, both infinities, and −400 °C.
    ///
    /// # Examples
    ///
    /// ```
    /// use tmp108::Celsius;
    /// let t = Celsius::try_from_degrees(25.0).unwrap();
    /// assert_eq!(t.sixteenths(), 400);
    /// assert_eq!(t.to_degrees(), 25.0);
    /// ```
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct Celsius(i16);

    /// Why a value could not be converted into a [`Celsius`].
    ///
    /// This is the only fallible direction in the module. Bytes
    /// arriving from the device are always a valid temperature, so the
    /// failures here all come from the human-facing side of the
    /// boundary.
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    #[non_exhaustive]
    pub enum OutOfRange {
        /// The value was `NaN`.
        NotANumber,
        /// The value was below [`Celsius::MIN`].
        TooLow,
        /// The value was above [`Celsius::MAX`].
        TooHigh,
    }

    impl core::fmt::Display for Celsius {
        /// Renders as degrees Celsius, honouring precision: `{:.2}` works.
        ///
        /// This is the rendering edge the post-parse value is finally
        /// allowed to become a float at.
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            core::fmt::Display::fmt(&self.to_degrees(), f)
        }
    }

    impl core::fmt::Display for OutOfRange {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            match self {
                Self::NotANumber => f.write_str("temperature is not a number"),
                Self::TooLow => f.write_str("temperature is below -128 C"),
                Self::TooHigh => f.write_str("temperature is above 127.9375 C"),
            }
        }
    }

    /// Sixteenths of a degree in the lowest representable temperature.
    pub(crate) const MIN_SIXTEENTHS: i16 = -2048;

    /// Sixteenths of a degree in the highest representable temperature.
    pub(crate) const MAX_SIXTEENTHS: i16 = 2047;

    /// Bits the 12-bit temperature field is left-shifted by within the
    /// register.
    const REGISTER_SHIFT: u32 = 4;

    /// Values in sixteenths at or below this round to something below
    /// [`Celsius::MIN`].
    pub(crate) const LOWEST_ACCEPTED: f32 = -2048.5;

    /// Values in sixteenths at or above this round to something above
    /// [`Celsius::MAX`].
    pub(crate) const HIGHEST_ACCEPTED: f32 = 2047.5;

    impl Celsius {
        /// Lowest temperature the part can represent, −128 °C.
        pub const MIN: Self = Self(MIN_SIXTEENTHS);

        /// Highest temperature the part can represent, +127.9375 °C.
        ///
        /// Note this is one LSB short of +128 °C: two's complement
        /// affords one more negative code than positive.
        pub const MAX: Self = Self(MAX_SIXTEENTHS);

        /// Zero degrees.
        pub const ZERO: Self = Self(0);

        /// Build a temperature from sixteenths of a degree.
        ///
        /// # Errors
        ///
        /// [`OutOfRange`] if `sixteenths` is outside `-2048..=2047`.
        ///
        /// # Examples
        ///
        /// ```
        /// use tmp108::{Celsius, OutOfRange};
        /// assert_eq!(Celsius::from_sixteenths(400).unwrap().to_degrees(), 25.0);
        /// assert_eq!(Celsius::from_sixteenths(9999), Err(OutOfRange::TooHigh));
        /// ```
        pub const fn from_sixteenths(sixteenths: i16) -> Result<Self, OutOfRange> {
            if sixteenths < MIN_SIXTEENTHS {
                Err(OutOfRange::TooLow)
            } else if sixteenths > MAX_SIXTEENTHS {
                Err(OutOfRange::TooHigh)
            } else {
                Ok(Self(sixteenths))
            }
        }

        /// Parse a temperature in degrees Celsius.
        ///
        /// This is the boundary between what a human writes and what
        /// the part can store. Values are rounded to the nearest
        /// sixteenth of a degree, half away from zero.
        ///
        /// # Errors
        ///
        /// [`OutOfRange`] if `degrees` is `NaN`, infinite, or rounds to
        /// a value outside [`Celsius::MIN`]`..=`[`Celsius::MAX`].
        ///
        /// # Examples
        ///
        /// ```
        /// use tmp108::{Celsius, OutOfRange};
        /// assert_eq!(Celsius::try_from_degrees(-55.0).unwrap().sixteenths(), -880);
        /// assert_eq!(Celsius::try_from_degrees(128.0), Err(OutOfRange::TooHigh));
        /// assert_eq!(Celsius::try_from_degrees(f32::NAN), Err(OutOfRange::NotANumber));
        /// ```
        pub fn try_from_degrees(degrees: f32) -> Result<Self, OutOfRange> {
            if degrees.is_nan() {
                return Err(OutOfRange::NotANumber);
            }

            let scaled = degrees * 16.0;

            // Rounding half away from zero then truncating means a value is
            // representable exactly when it lies strictly inside this open
            // interval. Checking `scaled` rather than the rounded value keeps
            // the endpoints, -128.0 and 127.9375, inside the range where they
            // belong.
            if scaled <= LOWEST_ACCEPTED {
                return Err(OutOfRange::TooLow);
            }
            if scaled >= HIGHEST_ACCEPTED {
                return Err(OutOfRange::TooHigh);
            }

            // `f32::round` lives in `std`, and this crate is `no_std`. Adding
            // a half and truncating toward zero rounds half away from zero.
            let rounded = if scaled >= 0.0 { scaled + 0.5 } else { scaled - 0.5 };

            // The bounds above guarantee this lands in `-2048..=2047`.
            #[allow(clippy::cast_possible_truncation)]
            Ok(Self(rounded as i16))
        }

        /// The temperature in sixteenths of a degree.
        #[must_use]
        pub const fn sixteenths(self) -> i16 {
            self.0
        }

        /// The temperature in degrees Celsius.
        ///
        /// Exact: the scale factor is a power of two and the value
        /// always fits in an `f32` mantissa, so nothing is rounded
        /// here.
        #[must_use]
        pub fn to_degrees(self) -> f32 {
            f32::from(self.0) * 0.0625
        }

        /// Decode a temperature or limit register.
        ///
        /// Total. The register holds a 12-bit two's complement value
        /// left-justified in 16 bits, so every one of the 65,536
        /// possible bit patterns names a temperature in range. The four
        /// unused low bits are discarded — per the datasheet (Table 6
        /// and Table 12) they are hardwired zero and "always read 0".
        pub(crate) const fn from_register(raw: [u8; 2]) -> Self {
            // Arithmetic shift: sign-extends, and floors rather than
            // truncating toward zero, which matters only for the low bits the
            // part never sets.
            Self(i16::from_be_bytes(raw) >> REGISTER_SHIFT)
        }

        /// Encode into a temperature or limit register.
        ///
        /// Total. The invariant on the inner value guarantees the shift
        /// cannot overflow.
        pub(crate) const fn to_register(self) -> [u8; 2] {
            (self.0 << REGISTER_SHIFT).to_be_bytes()
        }
    }

    /// Snap a continuous-f32 hysteresis input to the nearest legal
    /// chip setting within the [`HYSTERESIS_TOLERANCE`] band.
    ///
    /// Returns `None` for inputs that are non-finite or further than
    /// the tolerance from every legal setting. The four legal
    /// settings are 0, 1, 2, and 4 °C.
    #[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
    pub(crate) fn snap_hysteresis(input: f32) -> Option<Hysteresis> {
        const HYS_VALUES: &[(f32, Hysteresis)] = &[
            (0.0, Hysteresis::ZeroC),
            (1.0, Hysteresis::OneC),
            (2.0, Hysteresis::TwoC),
            (4.0, Hysteresis::FourC),
        ];

        if !input.is_finite() {
            return None;
        }

        // HYS_VALUES is non-empty so min_by always returns Some.
        let (closest, snapped) = HYS_VALUES
            .iter()
            .copied()
            .min_by(|(a, _), (b, _)| (input - a).abs().total_cmp(&(input - b).abs()))
            .expect("HYS_VALUES is non-empty");

        if (input - closest).abs() > HYSTERESIS_TOLERANCE {
            return None;
        }

        Some(snapped)
    }

    /// Decode a configuration-register snapshot into a typed [`Config`].
    pub(crate) fn decode_config(c: Configuration) -> Config {
        Config {
            thermostat_mode: c.tm(),
            alert_polarity: c.pol(),
            conversion_rate: c.cr(),
            hysteresis: c.hys(),
        }
    }

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
    #[cfg(feature = "embedded-sensors-hal-async")]
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(crate) struct AlertSnapshot {
        pub(crate) config: Config,
        pub(crate) low: bool,
        pub(crate) high: bool,
    }

    /// Decode a configuration-register snapshot into its settings and
    /// the ALERT status flags it returned.
    ///
    /// Pure, total, and allocation-free. Notably it does **not** call
    /// `c.m()`: the generated `Mode` decoder rejects `M = 0b11`
    /// (issue #62), and this fix must not inherit that defect.
    #[cfg(feature = "embedded-sensors-hal-async")]
    pub(crate) fn decode_alert_snapshot(c: Configuration) -> AlertSnapshot {
        AlertSnapshot {
            config: decode_config(c),
            low: c.fl(),
            high: c.fh(),
        }
    }

    /// Interpret the FL/FH pair of an **already-qualified** interrupt
    /// notification as an [`AlertCause`].
    ///
    /// Pure, total, and allocation-free. `(false, false)` maps to
    /// [`AlertCause::Unknown`]: the notification was already qualified
    /// elsewhere — by nonzero entry flags, or by a successful level wait
    /// followed by a successful acknowledgment — and this function only
    /// reports what direction evidence that snapshot carried. It is
    /// **not** a test for whether an alert occurred, and converting an
    /// arbitrary empty snapshot through it must never create an event.
    #[cfg(feature = "embedded-sensors-hal-async")]
    pub(crate) fn interrupt_alert_cause(low: bool, high: bool) -> AlertCause {
        match (low, high) {
            (false, false) => AlertCause::Unknown,
            (true, false) => AlertCause::BelowLow,
            (false, true) => AlertCause::AboveHigh,
            (true, true) => AlertCause::Both,
        }
    }

    /// Apply a typed [`Config`] to a configuration-register snapshot,
    /// preserving untouched bits (M, FL, FH, ID).
    pub(crate) fn apply_config(r: &mut Configuration, cfg: Config) {
        r.set_tm(cfg.thermostat_mode);
        r.set_pol(cfg.alert_polarity);
        r.set_cr(cfg.conversion_rate);
        r.set_hys(cfg.hysteresis);
    }
}

/// Tmp108 device driver.
/// Blocking TMP108 driver.
///
/// Built on [`embedded_hal::i2c::I2c`]. The driver owns the I2C bus
/// for the lifetime of any operation; you can recover the bus by
/// calling [`destroy`][Self::destroy].
///
/// For the asynchronous flavor, see [`AsyncTmp108`].
pub struct Tmp108<I2C: I2c> {
    inner: Inner<Interface<I2C>>,
    addr: u8,
}

/// Asynchronous TMP108 driver.
///
/// Built on [`embedded_hal_async::i2c::I2c`]. The async flavor unlocks
/// [`AsyncTmp108::continuous`] (which has no blocking equivalent) and
/// is required by [`AlertTmp108`].
///
/// For the blocking flavor, see [`Tmp108`].
#[cfg(feature = "async")]
pub struct AsyncTmp108<I2C: AsyncI2c> {
    inner: Inner<AsyncInterface<I2C>>,
    addr: u8,
}

impl<I2C: I2c> Tmp108<I2C> {
    /// Create a new TMP108 instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::{A0, Tmp108};
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new(i2c, A0::Sda);
    /// assert_eq!(tmp.addr(), 0x4a);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn new(i2c: I2C, a0: A0) -> Self {
        let interface = Interface::new(i2c, a0);
        let addr = interface.addr;
        let inner = Inner::new(interface);

        Self { inner, addr }
    }

    /// Create a new TMP108 instance with A0 tied to GND, resulting in
    /// an instance responding to address `0x48`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_gnd(i2c);
    /// assert_eq!(tmp.addr(), 0x48);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn new_with_a0_gnd(i2c: I2C) -> Self {
        Self::new(i2c, A0::Gnd)
    }

    /// Create a new TMP108 instance with A0 tied to V+, resulting in
    /// an instance responding to address `0x49`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_vplus(i2c);
    /// assert_eq!(tmp.addr(), 0x49);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn new_with_a0_vplus(i2c: I2C) -> Self {
        Self::new(i2c, A0::Vplus)
    }

    /// Create a new TMP108 instance with A0 tied to SDA, resulting in
    /// an instance responding to address `0x4a`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_sda(i2c);
    /// assert_eq!(tmp.addr(), 0x4a);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn new_with_a0_sda(i2c: I2C) -> Self {
        Self::new(i2c, A0::Sda)
    }

    /// Create a new TMP108 instance with A0 tied to SCL, resulting in
    /// an instance responding to address `0x4b`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_scl(i2c);
    /// assert_eq!(tmp.addr(), 0x4b);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn new_with_a0_scl(i2c: I2C) -> Self {
        Self::new(i2c, A0::Scl)
    }

    /// Get the current I2C address
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_gnd(i2c);
    /// assert_eq!(tmp.addr(), 0x48);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn addr(&self) -> u8 {
        self.addr
    }

    /// Destroy the driver instance, return the I2C bus instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let mut i2c = tmp.destroy();
    /// i2c.done();
    /// ```
    pub fn destroy(self) -> I2C {
        self.inner.free().i2c
    }
}

#[cfg(feature = "async")]
impl<I2C: AsyncI2c> AsyncTmp108<I2C> {
    /// Create a new TMP108 instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::{A0, AsyncTmp108};
    /// let i2c = Mock::new(&[]);
    /// let tmp = AsyncTmp108::new(i2c, A0::Sda);
    /// assert_eq!(tmp.addr(), 0x4a);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub fn new(i2c: I2C, a0: A0) -> Self {
        let interface = AsyncInterface::new(i2c, a0);
        let addr = interface.addr;
        let inner = Inner::new(interface);

        Self { inner, addr }
    }

    /// Create a new TMP108 instance with A0 tied to GND, resulting in
    /// an instance responding to address `0x48`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// assert_eq!(tmp.addr(), 0x48);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub fn new_with_a0_gnd(i2c: I2C) -> Self {
        Self::new(i2c, A0::Gnd)
    }

    /// Create a new TMP108 instance with A0 tied to V+, resulting in
    /// an instance responding to address `0x49`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = AsyncTmp108::new_with_a0_vplus(i2c);
    /// assert_eq!(tmp.addr(), 0x49);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub fn new_with_a0_vplus(i2c: I2C) -> Self {
        Self::new(i2c, A0::Vplus)
    }

    /// Create a new TMP108 instance with A0 tied to SDA, resulting in
    /// an instance responding to address `0x4a`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = AsyncTmp108::new_with_a0_sda(i2c);
    /// assert_eq!(tmp.addr(), 0x4a);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub fn new_with_a0_sda(i2c: I2C) -> Self {
        Self::new(i2c, A0::Sda)
    }

    /// Create a new TMP108 instance with A0 tied to SCL, resulting in
    /// an instance responding to address `0x4b`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = AsyncTmp108::new_with_a0_scl(i2c);
    /// assert_eq!(tmp.addr(), 0x4b);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub fn new_with_a0_scl(i2c: I2C) -> Self {
        Self::new(i2c, A0::Scl)
    }

    /// Get the current I2C address
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// assert_eq!(tmp.addr(), 0x48);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub fn addr(&self) -> u8 {
        self.addr
    }

    /// Destroy the driver instance, return the I2C bus instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[]);
    /// let tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// let mut i2c = tmp.destroy();
    /// i2c.done();
    /// # });
    /// ```
    pub fn destroy(self) -> I2C {
        self.inner.free().i2c
    }

    /// Create a new [`AlertTmp108`] instance by consuming the original
    /// `AsyncTmp108` instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// let alert_tmp = tmp.into_alert(alert);
    /// let (mut i2c, mut alert) = alert_tmp.destroy();
    /// i2c.done();
    /// alert.done();
    /// # });
    /// ```
    #[cfg(feature = "embedded-sensors-hal-async")]
    pub fn into_alert<ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>(
        self,
        alert: ALERT,
    ) -> AlertTmp108<I2C, ALERT> {
        AlertTmp108 {
            tmp108: self,
            alert,
            interrupt_sample_pending: None,
        }
    }
}

/// Async TMP108 driver with an ALERT GPIO pin attached.
///
/// Wraps a bare [`Tmp108`] with a pin implementing
/// [`embedded_hal_async::digital::Wait`] (and
/// [`embedded_hal::digital::InputPin`]), so it can implement
/// [`embedded_sensors_hal_async::temperature::TemperatureThresholdWait`].
///
/// # Notes on alert behavior
///
/// The driver's [`wait_for_temperature_threshold`][1] implementation
/// uses the *level* waits [`wait_for_low`][3] / [`wait_for_high`][4],
/// never the edge waits. This is deliberate. The `embedded-hal-async`
/// [`Wait`][2] contract states that an edge wait does **not** return
/// for an already-active pin: `wait_for_falling_edge` on a pin that is
/// already low "does *not* return immediately, it'll wait for the pin
/// to go high and then low again". Edges that occurred before the wait
/// was armed are therefore not guaranteed to be replayed, and a driver
/// that waits for one after having already acknowledged the chip can
/// wait forever.
///
/// (This supersedes section H1 of
/// `docs/superpowers/specs/2026-06-03-tmp108-reliability-fixes-design.md`,
/// which claimed the `Wait` trait retains pending edges between calls.
/// See
/// `docs/superpowers/specs/2026-09-24-tmp108-issue-59-alert-wait-ordering-design.md`
/// section 9.1. The rest of that earlier design stands.)
///
/// Reading the configuration register is not a side-effect-free status
/// query: per TMP108 datasheet SBOS663A §7.5.3.4, it clears both the
/// FL/FH flags and the ALERT pin. The waiter first checks for a retained
/// interrupt delivery from an earlier call. If one exists, it performs
/// **only one temperature read**: no configuration read, GPIO wait, or
/// acknowledgment. Otherwise it starts a **fresh acquisition**. In
/// Interrupt mode, that entry configuration read captures FL/FH; either
/// flag makes it proceed to temperature without GPIO or a second
/// acknowledgment. Only a clear entry snapshot leads to an asserted-level
/// wait followed by a second configuration read acknowledging that alert.
///
/// The chip's `Polarity` (active-low vs active-high) is read from the
/// configuration register on each fresh acquisition, not on retained
/// delivery attempts.
/// Do **not** reconfigure polarity while a `wait_for_temperature_threshold`
/// future is pending — the awaiting future will continue to wait for the
/// old polarity while the chip's ALERT output follows the new one.
///
/// On a fresh Comparator-mode acquisition the ALERT pin remains asserted
/// as long as the temperature is outside the `[TLow + HYS, THigh − HYS]` band; calling
/// `wait_for_temperature_threshold` in a tight loop while the chip is
/// still over-temperature will return immediately on every iteration
/// (because [`wait_for_low`][3] / [`wait_for_high`][4] return
/// immediately when the pin is already at the requested level). For
/// repeated-trigger workflows prefer Interrupt mode, or apply
/// application-level backoff between iterations. Fresh Comparator acquisition
/// never consults FL/FH and never performs a post-wait acknowledgment.
///
/// # Retained interrupt delivery
///
/// After a successful interrupt acknowledgment, the wrapper records a
/// **delivery obligation** before awaiting temperature. This applies both
/// to the entry-flag fast path and to the post-wait acknowledgment, whose
/// flags supply direction when nonzero but never requalify the event: a
/// zero-flag acknowledgment still arms the obligation, with
/// [`AlertCause::Unknown`]. Temperature-read failure returns
/// [`Error::Bus`] and leaves the obligation pending; cancellation during
/// that read also leaves it pending. A successful temperature read clears
/// it synchronously, with no intervening await, before returning `Ok`.
/// There is no internal retry: each delivery attempt reads temperature once.
///
/// [`wait_for_alert`][Self::wait_for_alert] and
/// [`wait_for_temperature_threshold`][1] are two views of **one
/// consumptive stream**, not two subscribers: they share this single
/// obligation. Either may arm it, and a successful delivery through
/// either settles it. The trait deliberately discards the cause, so a
/// cause consumed by a successful scalar delivery cannot be retrieved
/// afterwards.
///
/// The obligation survives [`sensor_mut`][Self::sensor_mut], direct
/// temperature reads, and reconfiguration. **Retained delivery takes
/// precedence over the current mode**, even after switching to Comparator
/// mode. Fresh Comparator acquisition never creates an obligation.
///
/// Retention is wrapper-local: [`into_inner`][Self::into_inner],
/// [`destroy`][Self::destroy], or dropping the wrapper abandons it without
/// driver I/O. Re-wrapping starts empty; it does not restore this state.
///
/// **Liveness:** with a retained obligation and a permanently failing bus
/// that returns immediately, a loop awaiting this method receives an
/// immediately-ready `Err` each time and need not yield. Callers must
/// provide their own backoff or bounded retries.
///
/// # What the returned temperature is
///
/// [`wait_for_temperature_threshold`][1] returns the **latest
/// conversion**, read from the temperature register *after* an alert
/// was observed. It is not a sample captured at the moment the
/// threshold was crossed, and the driver does not retain one.
/// A retained delivery is a debt, **not a cached sample**: its temperature
/// is read at retry time. There is no bound on the time since the crossing,
/// nor on the age of the conversion in the register. The value may be
/// outside or inside the band; it does not reconstruct the historical event.
///
/// Consequently:
///
/// - the value need not equal the temperature that triggered the alert;
/// - it can be back inside the configured `[TLow + HYS, THigh − HYS]`
///   band, which is normal when a latched interrupt is delivered after
///   the excursion ended, and is **not** grounds to treat the
///   notification as spurious;
/// - it cannot identify whether FL, FH, or both caused the event. The
///   chip can set both flags, and it coalesces multiple excursions into
///   one latched state. Comparing the returned value against the limits
///   is a heuristic, not a receipt. Use
///   [`wait_for_alert`][Self::wait_for_alert], which reports the
///   observed [`AlertCause`] alongside the sample, instead of inferring
///   direction from the temperature.
///
/// Reading the configuration register acknowledges the interrupt before
/// the temperature transaction is even issued, so the acknowledgment is
/// never contingent on the temperature read succeeding.
///
/// # Errors and cancellation
///
/// This operation is **not event-delivery cancel-safe**, and its error
/// type cannot express a partially completed event delivery. Callers
/// that need durable, exactly-once threshold notifications require a
/// separate delivery design — one this driver does not currently
/// provide. Failure or cancellation during the entry or acknowledging
/// configuration read can consume hardware evidence before the driver
/// records an obligation; no caller-side retry or queue can reconstruct
/// that lost evidence. Only the subsequent temperature stage is retryable.
/// Relatching or sustained Comparator assertion can still produce more
/// than one `Ok` for a single physical excursion.
///
/// Specifically:
///
/// - A configuration read may acknowledge an interrupt — clearing the
///   chip's FL/FH and releasing ALERT — *before* this call returns.
///   That acknowledgment is not undone on any failure path.
/// - [`Error::Bus`] does not identify which transaction failed: the
///   entry configuration read, the post-wait acknowledgment, or the
///   temperature read. It also does **not** prove that the interrupt
///   survived, nor that no interrupt occurred. An error must never be
///   read as "nothing happened".
/// - Cancelling this future — by dropping it, or by racing it against a
///   timeout or a `select!` — during a configuration read can consume an
///   event unrecoverably, without returning a temperature or an error.
///   Cancellation during a retained interrupt's temperature read instead
///   preserves the delivery obligation. No asynchronous cleanup runs on
///   drop; in particular no cleanup configuration read is performed,
///   because such a read would itself consume a later event.
/// - Retrying with a retained delivery performs temperature only. Without
///   an obligation, retrying starts a **fresh observation**. If an entry
///   or acknowledging read consumed the latch without returning success,
///   that fresh retry may wait indefinitely for a different event.
/// - Whether the bus and the pin are usable after cancellation depends
///   entirely on the underlying HAL's cancellation and recovery
///   guarantees for an in-flight I2C transaction or GPIO wait. The
///   driver makes no additional promise here.
/// - The returned sample is current register data, **not** a
///   trigger-time sample and not a receipt identifying a threshold
///   direction.
///
/// ## Recovery contract, by phase — Interrupt mode
///
/// The table below applies to **interrupt acquisition and retained
/// interrupt delivery**, including delivery after reconfiguration to
/// Comparator mode. Only Interrupt mode latches FL/FH and consumes an
/// event on a configuration read. Recovery differs by *where* the call
/// stopped, and in two of the four phases the driver cannot tell you
/// whether an event was consumed:
///
/// | Stopped at | Consumed? | What a caller may assume |
/// |---|---|---|
/// | Entry configuration read | **Unknown** | The chip may or may not have latched, and may or may not have been acknowledged before the failure. Treat any pending event as possibly lost. |
/// | GPIO level wait ([`Error::Pin`]) | **Nothing** | No acknowledgment was performed after the entry read. A still-latched event remains visible to the next call's entry snapshot. |
/// | Post-wait acknowledgment | **Unknown** | ALERT was observed asserted, but whether the acknowledging read reached the chip is not determinable from the error. |
/// | Temperature read (initial or retained attempt) | **Definitely acknowledged** | Hardware flags are gone, but delivery remains pending after failure or cancellation. Retrying on this wrapper reads temperature once, with no configuration read, GPIO wait, or acknowledgment, even after switching to Comparator mode. |
///
/// ## Recovery contract — Comparator mode
///
/// This section applies to **fresh Comparator acquisition**, with no
/// retained interrupt delivery. Comparator mode has no latch and no
/// acknowledgment: the entry configuration read is used solely to learn the configured mode and
/// polarity, no FL/FH is consulted, and there is deliberately no
/// post-wait acknowledging read. Consequently **nothing is ever
/// consumed** on this path, and none of the "evidence is gone" rows
/// above apply.
///
/// On this fresh Comparator path, a retry after any failure — including
/// a temperature-read failure after the level wait succeeded — performs a fresh
/// *level observation*. If the temperature is still outside the
/// `[TLow + HYS, THigh − HYS]` band, ALERT is still asserted, and the
/// retry may legitimately complete immediately for the **same
/// continuously asserted condition**, with no new threshold crossing
/// having occurred. Callers must therefore not treat a completed
/// comparator-mode call as evidence of a distinct event, nor treat a
/// failed one as evidence that a condition was consumed and can no
/// longer be observed. Conversely, if the temperature has returned
/// inside the band in the meantime, ALERT is released and the retry
/// will wait for the next excursion.
///
/// Returning from this method — and thus releasing the `&mut self`
/// borrow — only permits a subsequent Rust call. It establishes nothing
/// about the hardware: it does not mean the chip has settled, that
/// ALERT has been released, or that a new conversion has occurred.
///
/// [1]: embedded_sensors_hal_async::temperature::TemperatureThresholdWait::wait_for_temperature_threshold
/// [2]: embedded_hal_async::digital::Wait
/// [3]: embedded_hal_async::digital::Wait::wait_for_low
/// [4]: embedded_hal_async::digital::Wait::wait_for_high
#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
pub struct AlertTmp108<
    I2C: embedded_hal_async::i2c::I2c,
    ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin,
> {
    tmp108: AsyncTmp108<I2C>,
    alert: ALERT,
    /// An interrupt-mode event that has been acknowledged on the chip
    /// (C0 or C1 consumed its FL/FH and released ALERT) but whose
    /// temperature sample was never handed to the caller, because the
    /// read failed or the future was dropped.
    ///
    /// The chip cannot re-report it, so the driver owes the caller one
    /// delivery: while this is `Some`, both
    /// [`wait_for_alert`][Self::wait_for_alert] and
    /// `wait_for_temperature_threshold` perform the temperature read and
    /// nothing else. Only a successful delivery through either of them
    /// clears it (issue #58, gap 2). Comparator mode never sets it — it
    /// acknowledges nothing.
    ///
    /// The payload is the direction evidence that acknowledgment
    /// carried, so the five states are distinct and must never be
    /// collapsed (issue #67):
    ///
    /// | State | Representation |
    /// |---|---|
    /// | Empty, nothing owed | `None` |
    /// | Owed, direction unknown | `Some(AlertCause::Unknown)` |
    /// | Owed, FL observed | `Some(AlertCause::BelowLow)` |
    /// | Owed, FH observed | `Some(AlertCause::AboveHigh)` |
    /// | Owed, both observed | `Some(AlertCause::Both)` |
    ///
    /// In particular `Some(AlertCause::Unknown)` is a real debt — a
    /// successful level wait plus a successful zero-flag acknowledgment
    /// — and is not interchangeable with `None`.
    interrupt_sample_pending: Option<AlertCause>,
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    AlertTmp108<I2C, ALERT>
{
    /// Create a new ALERTTMP108 instance.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::{A0, AlertTmp108};
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new(i2c, A0::Sda, alert);
    /// assert_eq!(tmp.sensor().addr(), 0x4a);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # });
    /// ```
    pub fn new(i2c: I2C, a0: A0, alert: ALERT) -> Self {
        let tmp108 = AsyncTmp108::new(i2c, a0);
        Self {
            tmp108,
            alert,
            interrupt_sample_pending: None,
        }
    }

    /// Create a new ALERTTMP108 instance with A0 tied to GND, resulting in an
    /// instance responding to address `0x48`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);
    /// assert_eq!(tmp.sensor().addr(), 0x48);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # });
    /// ```
    pub fn new_with_a0_gnd(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Gnd, alert)
    }

    /// Create a new ALERTTMP108 instance with A0 tied to V+, resulting in an
    /// instance responding to address `0x49`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_vplus(i2c, alert);
    /// assert_eq!(tmp.sensor().addr(), 0x49);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # });
    /// ```
    pub fn new_with_a0_vplus(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Vplus, alert)
    }

    /// Create a new ALERTTMP108 instance with A0 tied to SDA, resulting in an
    /// instance responding to address `0x4a`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_sda(i2c, alert);
    /// assert_eq!(tmp.sensor().addr(), 0x4a);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # });
    /// ```
    pub fn new_with_a0_sda(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Sda, alert)
    }

    /// Create a new ALERTTMP108 instance with A0 tied to SCL, resulting in an
    /// instance responding to address `0x4b`.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_scl(i2c, alert);
    /// assert_eq!(tmp.sensor().addr(), 0x4b);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # });
    /// ```
    pub fn new_with_a0_scl(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Scl, alert)
    }

    /// Borrow the inner [`AsyncTmp108`] for sensor operations
    /// (read temperature, configure, set thresholds…). Useful when you
    /// want to call sensor methods directly, without going through the
    /// alert-aware threshold trait.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);
    /// assert_eq!(tmp.sensor().addr(), 0x48);
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # });
    /// ```
    pub fn sensor(&self) -> &AsyncTmp108<I2C> {
        &self.tmp108
    }

    /// Mutably borrow the inner [`AsyncTmp108`] for sensor operations.
    /// See [`sensor`][Self::sensor].
    /// Direct reads and reconfiguration do not clear a retained interrupt
    /// delivery obligation; it takes precedence on the next threshold wait.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{AlertTmp108, Config};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    /// ]);
    /// let alert = digital::Mock::new(&[]);
    /// let mut tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);
    /// let cfg = tmp.sensor_mut().read_configuration().await.unwrap();
    /// assert_eq!(cfg, Config::default());
    /// # let (mut i2c, mut alert) = tmp.destroy();
    /// # i2c.done();
    /// # alert.done();
    /// # });
    /// ```
    pub fn sensor_mut(&mut self) -> &mut AsyncTmp108<I2C> {
        &mut self.tmp108
    }

    /// Destructure the wrapper back into its [`AsyncTmp108`] sensor and
    /// ALERT pin halves, without I/O.
    ///
    /// Unlike [`destroy`][Self::destroy] (which returns the raw I2C
    /// bus, dropping the sensor's typed wrapper), this preserves the
    /// sensor's state so the caller can continue using it directly.
    /// It abandons any wrapper-local delivery obligation, however:
    /// re-wrapping with [`AsyncTmp108::into_alert`] starts empty and is
    /// not a state-preserving round trip for retained interrupt delivery.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);
    /// let (sensor, alert) = tmp.into_inner();
    /// // The sensor still owns the I2C bus.
    /// let mut i2c = sensor.destroy();
    /// i2c.done();
    /// let mut alert = alert;
    /// alert.done();
    /// # });
    /// ```
    pub fn into_inner(self) -> (AsyncTmp108<I2C>, ALERT) {
        (self.tmp108, self.alert)
    }

    /// Destroy the driver instance, return the I2C bus instance and ALERT pin instance.
    /// Any retained delivery obligation is abandoned without I/O.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::digital;
    /// # use embedded_hal_mock::eh1::i2c::Mock;
    /// use tmp108::AlertTmp108;
    /// let i2c = Mock::new(&[]);
    /// let alert = digital::Mock::new(&[]);
    /// let tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);
    /// let (mut i2c, mut alert) = tmp.destroy();
    /// i2c.done();
    /// alert.done();
    /// # });
    /// ```
    pub fn destroy(self) -> (I2C, ALERT) {
        (self.tmp108.destroy(), self.alert)
    }

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
    ) -> Result<AlertEvent, Error<I2C::Error, <ALERT as embedded_hal::digital::ErrorType>::Error>> {
        // Inspect the slot *first*, and copy rather than take. `AlertCause`
        // is `Copy`, so the authoritative value stays on the wrapper
        // across the `.await` on T below: a dropped future never runs an
        // error arm, so any "take now, restore on Err" shape would lose
        // the event exactly at cancellation. `Some(AlertCause::Unknown)`
        // is a genuine debt and takes the retained branch, identically to
        // a known direction — collapsing it into `None` would reintroduce
        // the lost-event loop.
        let cause = if let Some(retained) = self.interrupt_sample_pending {
            // A previous call already acknowledged an interrupt on the
            // chip but never managed to complete delivery. The chip has
            // no copy left, so the delivery obligation is this driver's
            // to settle: no C0 (there is nothing to collect and a read
            // would destroy a *newer* latch), no GPIO wait (the pin was
            // released by the acknowledgment), no C1. Just T. See issue
            // #58, gap 2.
            retained
        } else {
            // C0. This single transaction both tells us how the chip is
            // configured and consumes whatever FL/FH it had latched. The
            // binding is immutable on purpose: the flags captured here are
            // evidence of an already-pending interrupt until transferred
            // into the delivery obligation; nothing below may clobber them.
            let entry_snapshot = self.read_alert_snapshot().await.map_err(Error::Bus)?;
            let config = entry_snapshot.config;

            match (config.thermostat_mode, config.alert_polarity) {
                // Comparator mode does not latch, so FL/FH are not evidence
                // of anything the caller is waiting for — the pin level is.
                // Deliberately no fast path and no acknowledgment here.
                //
                // The ALERT pin stays asserted while the temperature is
                // outside (Tlow + HYS)..(Thigh - HYS), so calling this in a
                // tight loop returns immediately on every iteration.
                //
                // Nothing is acknowledged, so nothing can be owed: these
                // two arms must never arm `interrupt_sample_pending`. The
                // local `Unknown` below is this call's report, not a debt.
                (Thermostat::Comparator, Polarity::ActiveLow) => {
                    self.alert.wait_for_low().await.map_err(Error::Pin)?;
                    AlertCause::Unknown
                }
                (Thermostat::Comparator, Polarity::ActiveHigh) => {
                    self.alert.wait_for_high().await.map_err(Error::Pin)?;
                    AlertCause::Unknown
                }

                // Interrupt mode latches into FL/FH and releases the pin on
                // a configuration read. C0 above has therefore already
                // collected — and destroyed — any pending event. If it found
                // one, that *is* the notification: waiting on the pin now
                // would wait for a transition that has already been consumed
                // and that may never recur (issue #59).
                //
                // Otherwise we await the asserted level. A level wait, not
                // an edge wait: an assertion can land between C0 and the
                // moment the wait is armed, and `Wait`'s edge methods do not
                // return for an already-active pin.
                (Thermostat::Interrupt, polarity) => {
                    let acquired = if entry_snapshot.low || entry_snapshot.high {
                        ops::interrupt_alert_cause(entry_snapshot.low, entry_snapshot.high)
                    } else {
                        match polarity {
                            Polarity::ActiveLow => self.alert.wait_for_low().await.map_err(Error::Pin)?,
                            Polarity::ActiveHigh => self.alert.wait_for_high().await.map_err(Error::Pin)?,
                        }

                        // C1: acknowledge the assertion we just observed.
                        // Its flags *inform* direction but never *decide*
                        // whether an event exists — the successful level
                        // wait is the qualifying evidence, and requiring
                        // nonzero flags here would reintroduce a lost-event
                        // loop. The zero-flag case therefore still
                        // qualifies, and arms `Some(AlertCause::Unknown)`.
                        let acknowledgment = self.read_alert_snapshot().await.map_err(Error::Bus)?;
                        ops::interrupt_alert_cause(acknowledgment.low, acknowledgment.high)
                    };

                    // The event is now acknowledged on the chip and owed to
                    // the caller. Arm retention *before* awaiting T, so that
                    // a failure or a drop at any point from here on leaves
                    // the debt recorded — together with the direction the
                    // chip can no longer re-report.
                    self.interrupt_sample_pending = Some(acquired);
                    acquired
                }
            }
        };

        // T. The latest conversion, read after observing an alert — not
        // the temperature at the moment the threshold was crossed, and
        // possibly back inside the configured band. It never determines
        // or requalifies `cause`.
        let temperature = self.sensor_mut().temperature().await.map_err(Error::Bus)?;

        // Delivery succeeded, so the debt is settled. This runs in the
        // same poll that resolved T, with no `.await` in between: there
        // is no suspension point the caller could cancel at, so the
        // clear cannot be skipped on the success path. On the error
        // path the `?` above returns first and the slot keeps its cause.
        self.interrupt_sample_pending = None;
        Ok(AlertEvent { cause, temperature })
    }

    /// Read the configuration register once and keep both the settings
    /// and the ALERT status flags it returned.
    ///
    /// This is a **destructive read and an acknowledgment**, not a
    /// non-destructive status query. Per TMP108 datasheet SBOS663A
    /// §7.5.3.4, reading the configuration register clears FL, FH, and
    /// the ALERT pin. The returned snapshot holds evidence no longer
    /// recoverable from the chip. The waiter transfers notification
    /// evidence into its delivery obligation before awaiting temperature:
    /// nonzero entry flags or a successful level wait plus acknowledgment
    /// qualify it. The flags of either read supply the delivered
    /// [`AlertCause`] when they are nonzero, but they **never requalify**
    /// a successful level wait: a zero-flag acknowledgment still delivers
    /// an event, with [`AlertCause::Unknown`] (issue #67). Failure or
    /// cancellation during this read can lose evidence before that transfer.
    /// Call this exactly as often as the protocol
    /// requires — never speculatively, and never as "cleanup".
    async fn read_alert_snapshot(&mut self) -> Result<ops::AlertSnapshot, I2C::Error> {
        let c = self.sensor_mut().inner.configuration().read_async().await?;
        Ok(ops::decode_alert_snapshot(c))
    }
}

impl<I2C: I2c> Tmp108<I2C> {
    /// Probe the chip's presence by reading the configuration register.
    ///
    /// The TMP108 does not expose a `WHO_AM_I` / device-ID register, so a
    /// true identity probe is impossible. This method does the next-best
    /// thing: it reads the configuration register and reports whether
    /// the value matches the chip's documented power-on reset (POR)
    /// value `0x1022`. Useful immediately after power-on to confirm the
    /// chip is freshly out of reset and on the bus.
    ///
    /// # Returns
    ///
    /// - `Ok(true)` — the read succeeded and the configuration register
    ///   matches the POR value. Strong evidence the chip is present and
    ///   has not yet been reconfigured.
    /// - `Ok(false)` — the read succeeded but the configuration differs
    ///   from POR. Still strong evidence the chip is present (it `ACKed`
    ///   and returned plausible register data) but it was already
    ///   reconfigured since power-on. False negatives are unavoidable on
    ///   any boot path where the chip was configured before this method
    ///   ran.
    /// - `Err(_)` — the I2C read failed. Most likely cause is that no
    ///   chip is present at the expected address (NACK), but any bus
    ///   error reports here as well.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C read fails.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::Tmp108;
    /// // Chip returns the POR configuration -> probe() reports true.
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// assert!(tmp.probe().unwrap());
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn probe(&mut self) -> Result<bool, I2C::Error> {
        let raw = self.inner.configuration().read()?;
        Ok(u16::from_le_bytes(raw.into()) == ops::POR_CONFIG)
    }

    /// Read configuration register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{Config, Tmp108};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let cfg = tmp.read_configuration().unwrap();
    /// assert_eq!(cfg, Config::default());
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn read_configuration(&mut self) -> Result<Config, I2C::Error> {
        let c = self.inner.configuration().read()?;
        Ok(ops::decode_config(c))
    }

    /// Configure device parameters.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{Config, ConversionRate, Hysteresis, Polarity, Thermostat, Tmp108};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x66, 0xb0]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.configure(Config {
    ///     thermostat_mode: Thermostat::Interrupt,
    ///     alert_polarity: Polarity::ActiveHigh,
    ///     conversion_rate: ConversionRate::SixteenHz,
    ///     hysteresis: Hysteresis::FourC,
    /// }).unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn configure(&mut self, config: Config) -> Result<(), I2C::Error> {
        self.inner.configuration().modify(|r| ops::apply_config(r, config))
    }

    /// Read the temperature sensor
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let temp = tmp.temperature().unwrap();
    /// assert_eq!(temp.to_degrees(), 50.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn temperature(&mut self) -> Result<Celsius, I2C::Error> {
        let raw = self.inner.temperature().read()?;
        Ok(Celsius::from_register(raw.into()))
    }

    /// Configure device for one-shot conversion
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x21, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.one_shot().unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn one_shot(&mut self) -> Result<(), I2C::Error> {
        self.inner.configuration().modify(|r| r.set_m(Mode::OneShot))
    }

    /// Place device in shutdown mode
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x20, 0x10]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.shutdown().unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn shutdown(&mut self) -> Result<(), I2C::Error> {
        self.inner.configuration().modify(|r| r.set_m(Mode::Shutdown))
    }

    /// Wait one conversion period, then read the temperature register.
    ///
    /// Reads the configuration register to discover the current
    /// [`ConversionRate`], delays for one period (1/CR), and then reads
    /// the temperature register. Intended for callers driving the chip
    /// in [`Mode::Continuous`] who want to align reads with the chip's
    /// conversion cadence.
    ///
    /// # Stale-reading on first call
    ///
    /// The TMP108's conversion period (1/CR — 4 s, 1 s, 250 ms, or
    /// 62.5 ms) is **not** the same as its conversion **time** (~30 ms
    /// regardless of CR). After entering [`Mode::Continuous`] the chip's
    /// next conversion is not phase-aligned with when you enabled it,
    /// so the first call to this method may return the previous
    /// conversion result. For "guaranteed fresh" semantics, use
    /// [`one_shot`][Self::one_shot] followed by a delay of one period
    /// and a [`temperature`][Self::temperature] read, or discard the
    /// first reading after entering Continuous.
    ///
    /// # I²C cost per call
    ///
    /// Each call performs **two** I²C transactions: a configuration
    /// read (to determine the CR) and a temperature read. Callers in
    /// power- or bandwidth-sensitive loops who know they will not
    /// change CR can avoid the per-call configuration read by calling
    /// [`read_configuration`][Self::read_configuration] once, computing
    /// the period delay themselves, and calling
    /// [`temperature`][Self::temperature] directly.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when either the configuration read or the
    /// temperature read fails.
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::delay::NoopDelay;
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let mut delay = NoopDelay::new();
    /// let temp = tmp.wait_for_temperature(&mut delay).unwrap();
    /// assert_eq!(temp.to_degrees(), 50.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn wait_for_temperature<DELAY: DelayNs>(&mut self, delay: &mut DELAY) -> Result<Celsius, I2C::Error> {
        let config = self.read_configuration()?;
        delay.delay_us(conversion_period_us(config.conversion_rate));
        self.temperature()
    }

    /// Read temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x02], vec![0x19, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let limit = tmp.low_limit().unwrap();
    /// assert_eq!(limit.to_degrees(), 25.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn low_limit(&mut self) -> Result<Celsius, I2C::Error> {
        let raw = self.inner.t_low().read()?;
        Ok(Celsius::from_register(raw.into()))
    }

    /// Set temperature low limit register
    ///
    /// Takes a [`Celsius`], which by construction is always
    /// representable in the chip's 12-bit limit register, so the only
    /// remaining failure mode is the bus. Build one with
    /// [`Celsius::try_from_degrees`] or [`Celsius::from_sixteenths`].
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{Celsius, Tmp108};
    /// let i2c = Mock::new(&[
    ///     Transaction::write(0x48, vec![0x02, 0x19, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.set_low_limit(Celsius::try_from_degrees(25.0).unwrap()).unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn set_low_limit(&mut self, limit: Celsius) -> Result<(), I2C::Error> {
        self.inner.t_low().write(|r| *r = TLow::from(limit.to_register()))
    }

    /// Read temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::Tmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x03], vec![0x50, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// let limit = tmp.high_limit().unwrap();
    /// assert_eq!(limit.to_degrees(), 80.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn high_limit(&mut self) -> Result<Celsius, I2C::Error> {
        let raw = self.inner.t_high().read()?;
        Ok(Celsius::from_register(raw.into()))
    }

    /// Set temperature high limit register
    ///
    /// Takes a [`Celsius`], which by construction is always
    /// representable in the chip's 12-bit limit register, so the only
    /// remaining failure mode is the bus. Build one with
    /// [`Celsius::try_from_degrees`] or [`Celsius::from_sixteenths`].
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{Celsius, Tmp108};
    /// let i2c = Mock::new(&[
    ///     Transaction::write(0x48, vec![0x03, 0x50, 0x00]),
    /// ]);
    /// let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    /// tmp.set_high_limit(Celsius::try_from_degrees(80.0).unwrap()).unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// ```
    pub fn set_high_limit(&mut self, limit: Celsius) -> Result<(), I2C::Error> {
        self.inner.t_high().write(|r| *r = THigh::from(limit.to_register()))
    }
}

#[cfg(feature = "async")]
impl<I2C: AsyncI2c> AsyncTmp108<I2C> {
    /// Probe the chip's presence by reading the configuration register.
    ///
    /// See [`Tmp108::probe`] for full semantics. The async flavor has
    /// the same `Ok(true)` / `Ok(false)` / `Err(_)` contract.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C read fails.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// assert!(tmp.probe().await.unwrap());
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn probe(&mut self) -> Result<bool, I2C::Error> {
        let raw = self.inner.configuration().read_async().await?;
        Ok(u16::from_le_bytes(raw.into()) == ops::POR_CONFIG)
    }

    /// Read configuration register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{AsyncTmp108, Config};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// let cfg = tmp.read_configuration().await.unwrap();
    /// assert_eq!(cfg, Config::default());
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn read_configuration(&mut self) -> Result<Config, I2C::Error> {
        let c = self.inner.configuration().read_async().await?;
        Ok(ops::decode_config(c))
    }

    /// Configure device parameters.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{AsyncTmp108, Config, ConversionRate, Hysteresis, Polarity, Thermostat};
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x66, 0xb0]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// tmp.configure(Config {
    ///     thermostat_mode: Thermostat::Interrupt,
    ///     alert_polarity: Polarity::ActiveHigh,
    ///     conversion_rate: ConversionRate::SixteenHz,
    ///     hysteresis: Hysteresis::FourC,
    /// }).await.unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn configure(&mut self, config: Config) -> Result<(), I2C::Error> {
        self.inner
            .configuration()
            .modify_async(|r| ops::apply_config(r, config))
            .await
    }

    /// Read the temperature sensor
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// let temp = tmp.temperature().await.unwrap();
    /// assert_eq!(temp.to_degrees(), 50.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn temperature(&mut self) -> Result<Celsius, I2C::Error> {
        let raw = self.inner.temperature().read_async().await?;
        Ok(Celsius::from_register(raw.into()))
    }

    /// Configure device for one-shot conversion
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x21, 0x10]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// tmp.one_shot().await.unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
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
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x20, 0x10]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// tmp.shutdown().await.unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn shutdown(&mut self) -> Result<(), I2C::Error> {
        self.inner
            .configuration()
            .modify_async(|r| r.set_m(Mode::Shutdown))
            .await
    }

    /// Initiate continuous conversions.
    ///
    /// Switches the chip into [`Mode::Continuous`], runs the user-supplied
    /// closure, and unconditionally returns the chip to [`Mode::Shutdown`]
    /// before returning, **regardless of whether the closure succeeded or
    /// failed**. This ensures the chip is not left burning current after
    /// a transient bus failure inside the closure.
    ///
    /// # Cancel-safety
    ///
    /// **The returned future is *not* cancel-safe.** If it is dropped
    /// before completion (e.g. by `embassy_futures::select!`,
    /// `tokio::time::timeout`, or a task cancellation), the chip is left
    /// in [`Mode::Continuous`] and will continue to draw current
    /// indefinitely. Callers that need cancellation must structure their
    /// own recovery, for example by calling
    /// [`shutdown`][Self::shutdown] after a cancelled call.
    ///
    /// # Errors
    ///
    /// - If the closure returns `Err(e)`, the cleanup `shutdown()` still
    ///   runs but its result is discarded; the closure's error is
    ///   returned.
    /// - If the closure returns `Ok(())` and the cleanup `shutdown()`
    ///   fails, that I2C error is returned.
    /// - If the initial transition into `Mode::Continuous` fails, the
    ///   closure is not invoked and the I2C error is returned.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     // Enter continuous: read cfg, modify M bits, write back.
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x22, 0x10]),
    ///     // Inside the closure: read temperature once.
    ///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
    ///     // continuous() returns the chip to shutdown on exit.
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write(0x48, vec![0x01, 0x20, 0x10]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// tmp.continuous(async |t| {
    ///     let _ = t.temperature().await?;
    ///     Ok(())
    /// }).await.unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn continuous<F>(&mut self, f: F) -> Result<(), I2C::Error>
    where
        F: AsyncFnOnce(&mut Self) -> Result<(), I2C::Error>,
    {
        self.inner
            .configuration()
            .modify_async(|r| r.set_m(Mode::Continuous))
            .await?;

        // Run the user closure and capture its result, but always attempt
        // shutdown afterwards so the chip is not left in Continuous mode.
        // The closure's error takes precedence over a shutdown failure:
        // the user's failure is the actionable signal, the cleanup error
        // is a secondary symptom.
        let user_result = f(self).await;
        let cleanup_result = self.shutdown().await;
        user_result.and(cleanup_result)
    }

    /// Wait one conversion period, then read the temperature register.
    ///
    /// See [`Tmp108::wait_for_temperature`] for the full semantics,
    /// including the stale-first-reading and per-call I²C cost notes.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when either the configuration read or the
    /// temperature read fails.
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::delay::NoopDelay;
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
    ///     Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// let mut delay = NoopDelay::new();
    /// let temp = tmp.wait_for_temperature(&mut delay).await.unwrap();
    /// assert_eq!(temp.to_degrees(), 50.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn wait_for_temperature<DELAY: AsyncDelayNs>(
        &mut self,
        delay: &mut DELAY,
    ) -> Result<Celsius, I2C::Error> {
        let config = self.read_configuration().await?;
        delay.delay_us(conversion_period_us(config.conversion_rate)).await;
        self.temperature().await
    }

    /// Read temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x02], vec![0x19, 0x00]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// let limit = tmp.low_limit().await.unwrap();
    /// assert_eq!(limit.to_degrees(), 25.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn low_limit(&mut self) -> Result<Celsius, I2C::Error> {
        let raw = self.inner.t_low().read_async().await?;
        Ok(Celsius::from_register(raw.into()))
    }

    /// Set temperature low limit register
    ///
    /// Takes a [`Celsius`], which by construction is always
    /// representable in the chip's 12-bit limit register, so the only
    /// remaining failure mode is the bus. Build one with
    /// [`Celsius::try_from_degrees`] or [`Celsius::from_sixteenths`].
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{AsyncTmp108, Celsius};
    /// let i2c = Mock::new(&[
    ///     Transaction::write(0x48, vec![0x02, 0x19, 0x00]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// tmp.set_low_limit(Celsius::try_from_degrees(25.0).unwrap()).await.unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn set_low_limit(&mut self, limit: Celsius) -> Result<(), I2C::Error> {
        self.inner
            .t_low()
            .write_async(|r| *r = TLow::from(limit.to_register()))
            .await
    }

    /// Read temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::AsyncTmp108;
    /// let i2c = Mock::new(&[
    ///     Transaction::write_read(0x48, vec![0x03], vec![0x50, 0x00]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// let limit = tmp.high_limit().await.unwrap();
    /// assert_eq!(limit.to_degrees(), 80.0);
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn high_limit(&mut self) -> Result<Celsius, I2C::Error> {
        let raw = self.inner.t_high().read_async().await?;
        Ok(Celsius::from_register(raw.into()))
    }

    /// Set temperature high limit register
    ///
    /// Takes a [`Celsius`], which by construction is always
    /// representable in the chip's 12-bit limit register, so the only
    /// remaining failure mode is the bus. Build one with
    /// [`Celsius::try_from_degrees`] or [`Celsius::from_sixteenths`].
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    ///
    /// # Examples
    ///
    /// ```
    /// # tokio::runtime::Runtime::new().unwrap().block_on(async {
    /// # use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    /// use tmp108::{AsyncTmp108, Celsius};
    /// let i2c = Mock::new(&[
    ///     Transaction::write(0x48, vec![0x03, 0x50, 0x00]),
    /// ]);
    /// let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);
    /// tmp.set_high_limit(Celsius::try_from_degrees(80.0).unwrap()).await.unwrap();
    /// # let mut i2c = tmp.destroy();
    /// # i2c.done();
    /// # });
    /// ```
    pub async fn set_high_limit(&mut self, limit: Celsius) -> Result<(), I2C::Error> {
        self.inner
            .t_high()
            .write_async(|r| *r = THigh::from(limit.to_register()))
            .await
    }
}

/// Compute the chip's conversion period (1/CR) in microseconds.
const fn conversion_period_us(rate: ConversionRate) -> u32 {
    match rate {
        ConversionRate::QuarterHz => 4_000_000,
        ConversionRate::OneHz => 1_000_000,
        ConversionRate::FourHz => 250_000,
        ConversionRate::SixteenHz => 62_500,
    }
}

/// Blocking-side I²C wire interface for [`Tmp108`].
///
/// Wraps an [`embedded_hal::i2c::I2c`] and the chip address; implements
/// [`device_driver::RegisterInterface`] so the `device-driver` codegen
/// can drive it.
struct Interface<I2C: I2c> {
    i2c: I2C,
    addr: u8,
}

impl<I2C: I2c> Interface<I2C> {
    fn new(i2c: I2C, a0: A0) -> Self {
        Self { i2c, addr: a0.into() }
    }
}

impl<I2C: I2c> RegisterInterfaceBase for Interface<I2C> {
    type Error = I2C::Error;
    type AddressType = u8;
}

impl<I2C: I2c> RegisterInterface for Interface<I2C> {
    fn write_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        debug_assert_eq!(data.len(), 2, "TMP108 registers are 16-bit");
        let mut buf = [0; 3];
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

/// Async-side I²C wire interface for [`AsyncTmp108`].
///
/// Wraps an [`embedded_hal_async::i2c::I2c`] and the chip address;
/// implements [`device_driver::AsyncRegisterInterface`] so the
/// `device-driver` codegen can drive it asynchronously.
#[cfg(feature = "async")]
struct AsyncInterface<I2C: AsyncI2c> {
    i2c: I2C,
    addr: u8,
}

#[cfg(feature = "async")]
impl<I2C: AsyncI2c> AsyncInterface<I2C> {
    fn new(i2c: I2C, a0: A0) -> Self {
        Self { i2c, addr: a0.into() }
    }
}

#[cfg(feature = "async")]
impl<I2C: AsyncI2c> RegisterInterfaceBase for AsyncInterface<I2C> {
    type Error = I2C::Error;
    type AddressType = u8;
}

#[cfg(feature = "async")]
impl<I2C: AsyncI2c> AsyncRegisterInterface for AsyncInterface<I2C> {
    async fn write_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        debug_assert_eq!(data.len(), 2, "TMP108 registers are 16-bit");
        let mut buf = [0; 3];
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

/// Tmp108 Errors
///
/// The `E` parameter is the underlying I2C error type. The `P` parameter
/// is the error type of an optional ALERT GPIO pin; it defaults to
/// [`core::convert::Infallible`] so bare [`Tmp108`] (which has no pin)
/// uses `Error<I2C::Error>` and never produces a [`Pin`][Self::Pin]
/// error. [`AlertTmp108`] specializes to
/// `Error<I2C::Error, ALERT::Error>` and uses the [`Pin`][Self::Pin]
/// variant when the GPIO peripheral fails.
///
/// `Error` implements [`Clone`], [`Copy`], [`PartialEq`], and [`Eq`]
/// when both `E` and `P` do. [`Debug`] is always available because
/// `embedded_hal::i2c::Error` and `embedded_hal::digital::Error` both
/// require it.
#[derive(Debug)]
pub enum Error<E: embedded_hal::i2c::Error, P: embedded_hal::digital::Error = core::convert::Infallible> {
    /// I2C bus error.
    Bus(E),
    /// Input failed validation (out of range, NaN, ±∞, unsupported value).
    InvalidInput,
    /// ALERT pin GPIO error.
    Pin(P),
}

// Manual Clone (rather than #[derive]) because the auto-derive would
// emit `where E: Clone, P: Clone, E: embedded_hal::i2c::Error,
// P: embedded_hal::digital::Error` and we want only the first two
// bounds — the trait bounds are already implied by the struct.
// `expl_impl_clone_on_copy` would prefer #[derive], but doing so would
// require the same fix and the manual form is clearer about the bounds.
#[allow(clippy::expl_impl_clone_on_copy)]
impl<E: embedded_hal::i2c::Error + Clone, P: embedded_hal::digital::Error + Clone> Clone for Error<E, P> {
    fn clone(&self) -> Self {
        match self {
            Self::Bus(e) => Self::Bus(e.clone()),
            Self::InvalidInput => Self::InvalidInput,
            Self::Pin(e) => Self::Pin(e.clone()),
        }
    }
}

impl<E: embedded_hal::i2c::Error + Copy, P: embedded_hal::digital::Error + Copy> Copy for Error<E, P> {}

impl<E: embedded_hal::i2c::Error + PartialEq, P: embedded_hal::digital::Error + PartialEq> PartialEq for Error<E, P> {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Bus(a), Self::Bus(b)) => a == b,
            (Self::InvalidInput, Self::InvalidInput) => true,
            (Self::Pin(a), Self::Pin(b)) => a == b,
            _ => false,
        }
    }
}

impl<E: embedded_hal::i2c::Error + Eq, P: embedded_hal::digital::Error + Eq> Eq for Error<E, P> {}

#[cfg(all(feature = "embedded-sensors-hal", not(feature = "async")))]
impl<E: embedded_hal::i2c::Error, P: embedded_hal::digital::Error> embedded_sensors_hal::sensor::Error for Error<E, P> {
    fn kind(&self) -> embedded_sensors_hal::sensor::ErrorKind {
        embedded_sensors_hal::sensor::ErrorKind::Other
    }
}

#[cfg(all(feature = "embedded-sensors-hal", not(feature = "async")))]
impl<I2C: embedded_hal::i2c::I2c> embedded_sensors_hal::sensor::ErrorType for Tmp108<I2C> {
    type Error = Error<I2C::Error>;
}

#[cfg(all(feature = "embedded-sensors-hal", not(feature = "async")))]
impl<I2C: embedded_hal::i2c::I2c> embedded_sensors_hal::temperature::TemperatureSensor for Tmp108<I2C> {
    fn temperature(&mut self) -> Result<embedded_sensors_hal::temperature::DegreesCelsius, Self::Error> {
        self.temperature().map(Celsius::to_degrees).map_err(Error::Bus)
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<E: embedded_hal_async::i2c::Error, P: embedded_hal::digital::Error> embedded_sensors_hal_async::sensor::Error
    for Error<E, P>
{
    fn kind(&self) -> embedded_sensors_hal_async::sensor::ErrorKind {
        embedded_sensors_hal_async::sensor::ErrorKind::Other
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c> embedded_sensors_hal_async::sensor::ErrorType for AsyncTmp108<I2C> {
    type Error = Error<I2C::Error>;
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c> embedded_sensors_hal_async::temperature::TemperatureSensor
    for AsyncTmp108<I2C>
{
    async fn temperature(&mut self) -> Result<embedded_sensors_hal_async::temperature::DegreesCelsius, Self::Error> {
        self.temperature().await.map(Celsius::to_degrees).map_err(Error::Bus)
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal_async::sensor::ErrorType for AlertTmp108<I2C, ALERT>
{
    type Error = Error<I2C::Error, <ALERT as embedded_hal::digital::ErrorType>::Error>;
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal_async::temperature::TemperatureSensor for AlertTmp108<I2C, ALERT>
{
    async fn temperature(&mut self) -> Result<embedded_sensors_hal_async::temperature::DegreesCelsius, Self::Error> {
        self.tmp108
            .temperature()
            .await
            .map(Celsius::to_degrees)
            .map_err(Error::Bus)
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c> embedded_sensors_hal_async::temperature::TemperatureThresholdSet
    for AsyncTmp108<I2C>
{
    async fn set_temperature_threshold_low(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        // The trait takes a continuous f32; the chip takes a 12-bit
        // fixed-point value. Parse at the boundary and reject anything
        // the part cannot hold.
        let limit = Celsius::try_from_degrees(threshold).map_err(|_| Error::InvalidInput)?;
        self.set_low_limit(limit).await.map_err(Error::Bus)
    }

    async fn set_temperature_threshold_high(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        let limit = Celsius::try_from_degrees(threshold).map_err(|_| Error::InvalidInput)?;
        self.set_high_limit(limit).await.map_err(Error::Bus)
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal_async::temperature::TemperatureThresholdSet for AlertTmp108<I2C, ALERT>
{
    async fn set_temperature_threshold_low(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        let limit = Celsius::try_from_degrees(threshold).map_err(|_| Error::InvalidInput)?;
        self.tmp108.set_low_limit(limit).await.map_err(Error::Bus)
    }

    async fn set_temperature_threshold_high(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        let limit = Celsius::try_from_degrees(threshold).map_err(|_| Error::InvalidInput)?;
        self.tmp108.set_high_limit(limit).await.map_err(Error::Bus)
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal_async::temperature::TemperatureThresholdWait for AlertTmp108<I2C, ALERT>
{
    /// Deliver a retained interrupt, or acquire a fresh ALERT notification,
    /// then return the latest temperature conversion.
    ///
    /// This is a scalar projection of
    /// [`AlertTmp108::wait_for_alert`], which owns the whole acquisition
    /// and delivery protocol. Both entry points are two views of one
    /// consumptive stream sharing a single retained obligation: this one
    /// deliberately discards the observed [`AlertCause`], so a successful
    /// delivery here cannot be re-read as a cause later.
    ///
    /// The returned value is **not** a trigger-time sample, the error
    /// type cannot express a partially delivered event, and this future
    /// is not event-delivery cancel-safe during configuration reads.
    /// After successful interrupt acknowledgment, temperature failure or
    /// cancellation retains a delivery obligation: the next call reads
    /// temperature only, even if the mode has since changed. See
    /// [`AlertTmp108`]'s "Retained interrupt delivery", "What
    /// the returned temperature is" and "Errors and cancellation"
    /// sections for the full contract before relying on either.
    ///
    /// # Errors
    ///
    /// [`Error::Bus`] if any I2C transaction fails, [`Error::Pin`] if
    /// the GPIO wait fails. Neither proves whether an interrupt was
    /// consumed; see the recovery table on [`AlertTmp108`].
    async fn wait_for_temperature_threshold(
        &mut self,
    ) -> Result<embedded_sensors_hal_async::temperature::DegreesCelsius, Self::Error> {
        self.wait_for_alert().await.map(|event| event.temperature.to_degrees())
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c> embedded_sensors_hal_async::temperature::TemperatureHysteresis
    for AsyncTmp108<I2C>
{
    async fn set_temperature_threshold_hysteresis(
        &mut self,
        hysteresis: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        // The trait method takes a continuous range of f32 °C values, but
        // the chip only supports four discrete hysteresis settings:
        // 0, 1, 2, and 4 °C. ops::snap_hysteresis snaps within a 0.05 °C
        // tolerance band; anything outside the band (or non-finite) is
        // rejected with `Error::InvalidInput`.
        let snapped = ops::snap_hysteresis(hysteresis).ok_or(Error::InvalidInput)?;

        let mut config = self.read_configuration().await.map_err(Error::Bus)?;
        config.hysteresis = snapped;
        self.configure(config).await.map_err(Error::Bus)
    }
}

#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
impl<I2C: embedded_hal_async::i2c::I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal_async::temperature::TemperatureHysteresis for AlertTmp108<I2C, ALERT>
{
    async fn set_temperature_threshold_hysteresis(
        &mut self,
        hysteresis: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        self.tmp108
            .set_temperature_threshold_hysteresis(hysteresis)
            .await
            .map_err(widen_pin_err)
    }
}

/// Widen an `Error<E>` (with Pin = Infallible) to `Error<E, P>` for any
/// `P`. Used by the `AlertTmp108` trait impls that delegate to bare
/// [`AsyncTmp108`] methods (which cannot themselves produce a `Pin`
/// error).
#[cfg(all(feature = "embedded-sensors-hal-async", feature = "async"))]
fn widen_pin_err<E: embedded_hal_async::i2c::Error, P: embedded_hal::digital::Error>(
    e: Error<E, core::convert::Infallible>,
) -> Error<E, P> {
    match e {
        Error::Bus(e) => Error::Bus(e),
        Error::InvalidInput => Error::InvalidInput,
        Error::Pin(never) => match never {},
    }
}

#[cfg(test)]
mod tests {
    use super::inner::Configuration;
    use super::*;

    /// A `Configuration` initialized to the chip's power-on reset
    /// value.
    ///
    /// In device-driver 2.x a fieldset's `Default` is all-zeroes; the
    /// documented reset value is carried by the *register operation*
    /// instead. This helper reads it back out of the generated
    /// register operation so the tests below stay pinned to the DDSL
    /// manifest rather than to a hand-written constant.
    fn por_configuration() -> Configuration {
        let mut tmp = Tmp108::new_with_a0_gnd(embedded_hal_mock::eh1::i2c::Mock::new(&[]));
        let cfg = tmp.inner.configuration().reset_value();
        let mut i2c = tmp.destroy();
        i2c.done();
        cfg
    }

    #[test]
    fn default_configuration() {
        let cfg = por_configuration();
        assert_eq!(u16::from_le_bytes(cfg.into()), 0x1022);
    }

    #[test]
    fn modify_mode() {
        let mut cfg = por_configuration();
        cfg.set_m(Mode::Shutdown);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1020);
        cfg.set_m(Mode::OneShot);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1021);
        cfg.set_m(Mode::Continuous);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1022);
    }

    #[test]
    fn modify_thermostat_mode() {
        let mut cfg = por_configuration();
        cfg.set_tm(Thermostat::Comparator);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1022);
        cfg.set_tm(Thermostat::Interrupt);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1026);
    }

    #[test]
    fn modify_watchdog_temperature_flags() {
        let mut cfg = por_configuration();
        cfg.set_fl(true);
        cfg.set_fh(false);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x102a);
        cfg.set_fl(false);
        cfg.set_fh(true);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1032);
        cfg.set_fl(true);
        cfg.set_fh(true);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x103a);
    }

    #[test]
    fn modify_conversion_rate() {
        let mut cfg = por_configuration();
        cfg.set_cr(ConversionRate::QuarterHz);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1002);
        cfg.set_cr(ConversionRate::OneHz);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1022);
        cfg.set_cr(ConversionRate::FourHz);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1042);
        cfg.set_cr(ConversionRate::SixteenHz);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1062);
    }

    #[test]
    fn modify_hysteresis() {
        let mut cfg = por_configuration();
        cfg.set_hys(Hysteresis::ZeroC);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x0022);
        cfg.set_hys(Hysteresis::OneC);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1022);
        cfg.set_hys(Hysteresis::TwoC);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x2022);
        cfg.set_hys(Hysteresis::FourC);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x3022);
    }

    #[test]
    fn modify_polarity() {
        let mut cfg = por_configuration();
        cfg.set_pol(Polarity::ActiveLow);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x1022);
        cfg.set_pol(Polarity::ActiveHigh);
        assert_eq!(u16::from_ne_bytes(cfg.into()), 0x9022);
    }

    mod ops_tests {
        use super::*;

        /// Section 7.9 of the #59 design: the alert-snapshot decoder is
        /// pure and total over every possible register value.
        #[cfg(feature = "embedded-sensors-hal-async")]
        mod alert_snapshot {
            use crate::inner::Configuration;
            use crate::ops::{decode_alert_snapshot, decode_config};

            /// Exhaust all 65,536 `[u8; 2]` patterns.
            ///
            /// This includes every `M = 0b11` encoding, which the
            /// generated `Mode` decoder rejects (issue #62). The
            /// snapshot decoder must not call `m()`, so it has to
            /// survive them; a future refactor that reintroduces that
            /// dependency panics here.
            #[test]
            fn decoding_is_total_and_matches_the_settings_decoder() {
                for word in 0..=u16::MAX {
                    let bytes = word.to_le_bytes();
                    let c = Configuration::from(bytes);
                    let snapshot = decode_alert_snapshot(c);

                    assert_eq!(
                        snapshot.config,
                        decode_config(c),
                        "settings projection diverged for {bytes:02x?}"
                    );
                    assert_eq!(snapshot.low, bytes[0] & 0x08 != 0, "FL wrong for {bytes:02x?}");
                    assert_eq!(snapshot.high, bytes[0] & 0x10 != 0, "FH wrong for {bytes:02x?}");
                }
            }

            /// All four flag combinations are representable and
            /// independent.
            #[test]
            fn both_flags_are_independent() {
                let cases = [
                    (0x22_u8, false, false),
                    (0x2a_u8, true, false),
                    (0x32_u8, false, true),
                    (0x3a_u8, true, true),
                ];

                for (byte0, low, high) in cases {
                    let snapshot = decode_alert_snapshot(Configuration::from([byte0, 0x10]));
                    assert_eq!((snapshot.low, snapshot.high), (low, high), "byte0 {byte0:#04x}");
                }
            }
        }

        /// Issue #67: interpreting the FL/FH pair of an
        /// already-qualified notification.
        ///
        /// `(false, false)` mapping to `Unknown` is *interpretation
        /// after qualification*, never raw evidence that an alert
        /// occurred: the caller reaches this function only because
        /// nonzero entry flags or a successful level wait plus a
        /// successful acknowledgment already qualified the event.
        #[cfg(feature = "embedded-sensors-hal-async")]
        mod alert_cause {
            use crate::AlertCause;
            use crate::inner::Configuration;
            use crate::ops::{decode_alert_snapshot, interrupt_alert_cause};

            /// All four inputs, exhaustively.
            #[test]
            fn the_mapping_is_exhaustive_and_total() {
                assert_eq!(interrupt_alert_cause(false, false), AlertCause::Unknown);
                assert_eq!(interrupt_alert_cause(true, false), AlertCause::BelowLow);
                assert_eq!(interrupt_alert_cause(false, true), AlertCause::AboveHigh);
                assert_eq!(interrupt_alert_cause(true, true), AlertCause::Both);
            }

            /// FL is bit 3 and FH is bit 4 of the first configuration
            /// byte (SBOS663A, Table 8), and **no other bit** in the
            /// register can influence the cause.
            ///
            /// Walking all 65,536 patterns includes every `M = 0b11`
            /// encoding, which the generated `Mode` decoder rejects
            /// (issue #62). Nothing on this path may call `c.m()`.
            #[test]
            fn only_bits_3_and_4_of_byte_0_decide_the_cause() {
                for word in 0..=u16::MAX {
                    let bytes = word.to_le_bytes();
                    let snapshot = decode_alert_snapshot(Configuration::from(bytes));
                    let cause = interrupt_alert_cause(snapshot.low, snapshot.high);

                    let want = match (bytes[0] & 0x08 != 0, bytes[0] & 0x10 != 0) {
                        (false, false) => AlertCause::Unknown,
                        (true, false) => AlertCause::BelowLow,
                        (false, true) => AlertCause::AboveHigh,
                        (true, true) => AlertCause::Both,
                    };

                    assert_eq!(cause, want, "cause wrong for {bytes:02x?}");
                }
            }

            /// The same cause for every setting of every other bit,
            /// holding FL/FH fixed. `M = 0b11` is included explicitly.
            #[test]
            fn other_configuration_bits_are_irrelevant() {
                for flags in 0..4_u8 {
                    let fl = flags & 1 != 0;
                    let fh = flags & 2 != 0;
                    let want = interrupt_alert_cause(fl, fh);
                    let flag_bits = u8::from(fl) << 3 | u8::from(fh) << 4;

                    for other in 0..=u16::MAX {
                        let other = other.to_le_bytes();
                        // Everything except bits 3 and 4 of byte 0 —
                        // M = 0b11 among them.
                        let bytes = [(other[0] & !0x18) | flag_bits, other[1]];
                        let snapshot = decode_alert_snapshot(Configuration::from(bytes));
                        assert_eq!(
                            interrupt_alert_cause(snapshot.low, snapshot.high),
                            want,
                            "cause moved for {bytes:02x?}"
                        );
                    }
                }
            }
        }

        /// Exhaustive tests for the [`Celsius`] newtype and its
        /// register codec.
        mod celsius {
            // Exactness is the property under test. The scale factor is a
            // power of two and every value fits an `f32` mantissa, so these
            // conversions are lossless. Comparing approximately would weaken
            // the assertions, not strengthen them.
            #![allow(clippy::float_cmp)]

            use super::*;
            use crate::ops::{HIGHEST_ACCEPTED, LOWEST_ACCEPTED, MAX_SIXTEENTHS, MIN_SIXTEENTHS};

            /// Every temperature the type can hold. 4096 of them.
            fn all() -> impl Iterator<Item = Celsius> {
                (MIN_SIXTEENTHS..=MAX_SIXTEENTHS)
                    .map(|s| Celsius::from_sixteenths(s).expect("in range by construction"))
            }

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

            #[test]
            fn every_temperature_roundtrips_through_a_register() {
                for c in all() {
                    assert_eq!(Celsius::from_register(c.to_register()), c);
                }
            }

            #[test]
            fn every_temperature_roundtrips_through_sixteenths() {
                for c in all() {
                    assert_eq!(Celsius::from_sixteenths(c.sixteenths()), Ok(c));
                }
            }

            #[test]
            fn every_temperature_roundtrips_through_degrees() {
                for c in all() {
                    assert_eq!(Celsius::try_from_degrees(c.to_degrees()), Ok(c));
                }
            }

            #[test]
            fn sixteenths_outside_the_range_are_rejected() {
                assert_eq!(Celsius::from_sixteenths(MIN_SIXTEENTHS - 1), Err(OutOfRange::TooLow));
                assert_eq!(Celsius::from_sixteenths(MAX_SIXTEENTHS + 1), Err(OutOfRange::TooHigh));
                assert_eq!(Celsius::from_sixteenths(i16::MIN), Err(OutOfRange::TooLow));
                assert_eq!(Celsius::from_sixteenths(i16::MAX), Err(OutOfRange::TooHigh));
            }

            #[test]
            fn degrees_the_part_cannot_hold_are_rejected() {
                // The values that silently became 0 C, 127.9375 C and -128 C before.
                assert_eq!(Celsius::try_from_degrees(f32::NAN), Err(OutOfRange::NotANumber));
                assert_eq!(Celsius::try_from_degrees(f32::INFINITY), Err(OutOfRange::TooHigh));
                assert_eq!(Celsius::try_from_degrees(f32::NEG_INFINITY), Err(OutOfRange::TooLow));
                assert_eq!(Celsius::try_from_degrees(1000.0), Err(OutOfRange::TooHigh));
                assert_eq!(Celsius::try_from_degrees(-500.0), Err(OutOfRange::TooLow));
                assert_eq!(Celsius::try_from_degrees(128.0), Err(OutOfRange::TooHigh));
            }

            #[test]
            fn the_extremes_are_exactly_representable() {
                assert_eq!(Celsius::MIN.to_degrees(), -128.0);
                assert_eq!(Celsius::MAX.to_degrees(), 127.9375);
                assert_eq!(Celsius::ZERO.to_degrees(), 0.0);

                // One LSB short of +128, not +128. Two's complement is lopsided.
                assert_eq!(Celsius::try_from_degrees(127.9375), Ok(Celsius::MAX));
                assert_eq!(Celsius::try_from_degrees(-128.0), Ok(Celsius::MIN));
            }

            #[test]
            fn rounding_bounds_bracket_the_representable_range() {
                // The accepted interval is open and sits exactly half an LSB
                // outside the representable range at each end.
                assert_eq!(LOWEST_ACCEPTED, f32::from(MIN_SIXTEENTHS) - 0.5);
                assert_eq!(HIGHEST_ACCEPTED, f32::from(MAX_SIXTEENTHS) + 0.5);

                // Just inside rounds back to the endpoint; exactly on the
                // bound does not.
                assert_eq!(Celsius::try_from_degrees(-128.03).unwrap(), Celsius::MIN);
                assert_eq!(Celsius::try_from_degrees(-128.04), Err(OutOfRange::TooLow));
                assert_eq!(Celsius::try_from_degrees(127.96).unwrap(), Celsius::MAX);
                assert_eq!(Celsius::try_from_degrees(127.97), Err(OutOfRange::TooHigh));
            }

            #[test]
            fn degrees_round_to_the_nearest_sixteenth() {
                // 0.0625 C per LSB, so 0.03 rounds down to 0 and 0.04 rounds up to 1.
                assert_eq!(Celsius::try_from_degrees(0.03).unwrap().sixteenths(), 0);
                assert_eq!(Celsius::try_from_degrees(0.04).unwrap().sixteenths(), 1);
                assert_eq!(Celsius::try_from_degrees(-0.03).unwrap().sixteenths(), 0);
                assert_eq!(Celsius::try_from_degrees(-0.04).unwrap().sixteenths(), -1);
            }

            #[test]
            fn known_datasheet_values_decode() {
                // Table 7 of the datasheet, as register words.
                for (word, degrees) in [
                    (0x7ff0_u16, 127.9375_f32),
                    (0x6400, 100.0),
                    (0x5000, 80.0),
                    (0x3200, 50.0),
                    (0x1900, 25.0),
                    (0x0040, 0.25),
                    (0x0000, 0.0),
                    (0xffc0, -0.25),
                    (0xe700, -25.0),
                    (0xc900, -55.0),
                    (0x8000, -128.0),
                ] {
                    let c = Celsius::from_register(word.to_be_bytes());
                    assert_eq!(c.to_degrees(), degrees, "word {word:#06x}");
                    assert_eq!(c.to_register(), word.to_be_bytes(), "word {word:#06x}");
                }
            }

            #[test]
            fn unused_low_bits_are_discarded_not_truncated_toward_zero() {
                // Per the datasheet (Table 6 / Table 12) bits 3..0 of the low
                // byte are hardwired zero and "always read 0". `from_register`
                // is nonetheless total and must floor rather than truncate
                // toward zero for negatives.
                assert_eq!(Celsius::from_register(0xffff_u16.to_be_bytes()).sixteenths(), -1);
                assert_eq!(Celsius::from_register(0x0001_u16.to_be_bytes()).sixteenths(), 0);

                // The power-on T_HIGH value, which the datasheet quotes as
                // 0x7FF8 (+127.9375 C) — it carries a stray bit 3 that the
                // register cannot store once written.
                assert_eq!(Celsius::from_register(0x7ff8_u16.to_be_bytes()), Celsius::MAX);
            }
        }

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

        #[test]
        fn decode_apply_config_roundtrip() {
            // For every non-default Config we set, applying it to the POR
            // configuration and reading it back must yield the same Config
            // (modulo bits we don't expose: M, FL, FH, ID).
            let cfg = Config {
                thermostat_mode: Thermostat::Interrupt,
                alert_polarity: Polarity::ActiveHigh,
                conversion_rate: ConversionRate::SixteenHz,
                hysteresis: Hysteresis::FourC,
            };

            let mut reg = por_configuration();
            ops::apply_config(&mut reg, cfg);
            assert_eq!(ops::decode_config(reg), cfg);
        }

        /// Exhaustive tests for the `Config` <-> configuration-register
        /// codec.
        ///
        /// Every domain here is small enough to walk in full: `Config`
        /// has 64 inhabitants (1 + 1 + 2 + 2 bits) and the register is
        /// a single 16-bit word, so 65,536 covers every bit pattern the
        /// part could ever hand back. Nothing is sampled.
        mod config_bits {
            use super::*;

            /// Every `Thermostat`, in encoding order.
            const THERMOSTATS: [Thermostat; 2] = [Thermostat::Comparator, Thermostat::Interrupt];

            /// Every `Polarity`, in encoding order.
            const POLARITIES: [Polarity; 2] = [Polarity::ActiveLow, Polarity::ActiveHigh];

            /// Every `ConversionRate`, in encoding order.
            const CONVERSION_RATES: [ConversionRate; 4] = [
                ConversionRate::QuarterHz,
                ConversionRate::OneHz,
                ConversionRate::FourHz,
                ConversionRate::SixteenHz,
            ];

            /// Every `Hysteresis`, in encoding order.
            const HYSTERESES: [Hysteresis; 4] =
                [Hysteresis::ZeroC, Hysteresis::OneC, Hysteresis::TwoC, Hysteresis::FourC];

            /// The number of `Config` inhabitants the four modelled
            /// fields admit: 2 * 2 * 4 * 4.
            const CONFIG_INHABITANTS: usize = 64;

            /// Bits of the configuration register that `Config` models:
            /// `tm` (bit 2), `cr` (6:5), `hys` (13:12) and `pol` (bit
            /// 15). Everything else — `m` (1:0), `fl` (bit 3), `fh`
            /// (bit 4), `id` (bit 7) and the reserved bits 11:8 and 14
            /// — must survive `apply_config` untouched.
            const MODELLED_MASK: u16 = 0b1011_0000_0110_0100;

            /// The `m` field, 1:0. Encoding 3 is reserved.
            const MODE_MASK: u16 = 0b11;

            /// The reserved `m` encoding: three variants in two bits.
            const RESERVED_MODE: u16 = 3;

            /// The 64 `Config` values, applied to `f` one at a time.
            ///
            /// Returns how many it produced so callers can pin the
            /// count.
            fn for_each_config(mut f: impl FnMut(Config)) -> usize {
                let mut count = 0;
                for thermostat_mode in THERMOSTATS {
                    for alert_polarity in POLARITIES {
                        for conversion_rate in CONVERSION_RATES {
                            for hysteresis in HYSTERESES {
                                f(Config {
                                    thermostat_mode,
                                    alert_polarity,
                                    conversion_rate,
                                    hysteresis,
                                });
                                count += 1;
                            }
                        }
                    }
                }
                count
            }

            /// A `Config` with all four modelled fields at their lowest
            /// encoding — every modelled bit clear.
            const fn all_min() -> Config {
                Config {
                    thermostat_mode: Thermostat::Comparator,
                    alert_polarity: Polarity::ActiveLow,
                    conversion_rate: ConversionRate::QuarterHz,
                    hysteresis: Hysteresis::ZeroC,
                }
            }

            /// A `Config` with all four modelled fields at their highest
            /// encoding — every modelled bit set.
            const fn all_max() -> Config {
                Config {
                    thermostat_mode: Thermostat::Interrupt,
                    alert_polarity: Polarity::ActiveHigh,
                    conversion_rate: ConversionRate::SixteenHz,
                    hysteresis: Hysteresis::FourC,
                }
            }

            /// The register word after applying `cfg` to `word`.
            fn applied(word: u16, cfg: Config) -> u16 {
                let mut reg = Configuration::from(word.to_le_bytes());
                ops::apply_config(&mut reg, cfg);
                u16::from_le_bytes(reg.into())
            }

            #[test]
            fn every_config_roundtrips_through_the_register() {
                let mut checked = 0;
                let produced = for_each_config(|cfg| {
                    let mut reg = por_configuration();
                    ops::apply_config(&mut reg, cfg);
                    assert_eq!(ops::decode_config(reg), cfg, "roundtrip failed for {cfg:?}");
                    checked += 1;
                });

                assert_eq!(
                    produced, CONFIG_INHABITANTS,
                    "Config has {produced} inhabitants, not {CONFIG_INHABITANTS}. A modelled \
                     field was widened or narrowed: revisit this test, MODELLED_MASK, and the \
                     bit-preservation tests below before changing the expected count."
                );
                assert_eq!(checked, produced);
            }

            #[test]
            fn apply_config_preserves_every_unmodelled_bit() {
                // Bracket the field values: a Config with every modelled
                // bit clear and one with every modelled bit set. A mask
                // that is too wide in the clearing direction fails on
                // all_min; too wide in the setting direction fails on
                // all_max.
                // First pin MODELLED_MASK itself, two-sidedly, so it
                // cannot be quietly too wide: clearing every modelled
                // field on an all-ones word must leave exactly the
                // complement, and setting every modelled field on an
                // all-zeroes word must produce exactly the mask.
                assert_eq!(applied(0xffff, all_min()), !MODELLED_MASK);
                assert_eq!(applied(0x0000, all_max()), MODELLED_MASK);

                for cfg in [all_min(), all_max()] {
                    for word in 0..=u16::MAX {
                        let out = applied(word, cfg);
                        assert_eq!(
                            out & !MODELLED_MASK,
                            word & !MODELLED_MASK,
                            "apply_config({cfg:?}) disturbed unmodelled bits of {word:#06x}: \
                             got {out:#06x}"
                        );
                    }
                }
            }

            #[test]
            fn every_config_applied_to_the_extreme_words() {
                // 0x0000 and 0xffff are where an over-wide setter mask
                // shows up: it either fails to set a bit it owns or
                // clobbers one it does not.
                for word in [0x0000_u16, 0xffff_u16] {
                    let produced = for_each_config(|cfg| {
                        let out = applied(word, cfg);

                        assert_eq!(
                            out & !MODELLED_MASK,
                            word & !MODELLED_MASK,
                            "apply_config({cfg:?}) on {word:#06x} disturbed unmodelled bits: \
                             got {out:#06x}"
                        );

                        let reg = Configuration::from(out.to_le_bytes());
                        assert_eq!(
                            ops::decode_config(reg),
                            cfg,
                            "apply_config({cfg:?}) on {word:#06x} did not read back"
                        );
                    });
                    assert_eq!(produced, CONFIG_INHABITANTS);
                }
            }

            #[test]
            fn the_mode_getter_fails_exactly_on_the_reserved_encoding() {
                // Two-sided: reserved encodings must fail, and nothing
                // else may. A getter that widened its failure set would
                // reject perfectly legal words.
                for word in 0..=u16::MAX {
                    let reg = Configuration::from(word.to_le_bytes());
                    let reserved = (word & MODE_MASK) == RESERVED_MODE;
                    assert_eq!(
                        reg.m().is_err(),
                        reserved,
                        "word {word:#06x}: m() error state disagrees with the reserved encoding"
                    );
                }
            }
        }

        #[test]
        fn por_config_matches_default_configuration() {
            // ops::POR_CONFIG must match the chip's documented POR value
            // (0x1022) and the generated configuration register's reset
            // value. If the DDSL manifest changes the reset value,
            // probe()'s contract changes too — this test pins it.
            let cfg = por_configuration();
            assert_eq!(u16::from_le_bytes(cfg.into()), ops::POR_CONFIG);
        }

        #[test]
        fn config_derives_eq() {
            fn assert_eq_trait<T: Eq>(_: &T) {}

            // Config is now `Eq` (and `PartialEq` and `Hash`); use the
            // stronger trait so we know it actually compiles.
            let a = Config::default();
            let b = Config::default();
            assert_eq_trait(&a);
            assert_eq!(a, b);
        }

        #[test]
        fn error_derives_with_eq_kind() {
            // Error<E, P> implements Clone/Copy/PartialEq/Eq when both
            // E and P do. Verify with concrete types that satisfy those
            // bounds (ErrorKind from embedded-hal is the canonical
            // small witness here).
            type EK = embedded_hal::i2c::ErrorKind;
            type PK = embedded_hal::digital::ErrorKind;

            fn assert_traits<T: Clone + Copy + PartialEq + Eq + core::fmt::Debug>(_: &T) {}

            let invalid_a: Error<EK, PK> = Error::InvalidInput;
            let invalid_b: Error<EK, PK> = Error::InvalidInput;
            assert_traits(&invalid_a);
            assert_eq!(invalid_a, invalid_b);
            let invalid_c = invalid_a; // Copy

            assert_eq!(invalid_a, invalid_c);
            assert_eq!(invalid_a.clone(), invalid_b);

            let bus_err: Error<EK, PK> = Error::Bus(EK::Other);
            let pin_err: Error<EK, PK> = Error::Pin(PK::Other);
            assert_ne!(bus_err, pin_err);
            assert_ne!(bus_err, invalid_a);
        }
    }

    #[cfg(not(feature = "async"))]
    mod blocking {
        use assert_approx_eq::assert_approx_eq;
        use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

        use super::*;

        #[test]
        fn limit_registers_reset_to_the_documented_window() {
            // Datasheet §7.5.4: THIGH = +127.9375 °C (0x7FF8) and
            // TLOW = -128 °C (0x8000), sent MSB first. Before these were
            // declared in the DDSL both registers advertised a reset value
            // of 0 — a zero-width limit window at 0 °C, which is not what
            // the part does.
            //
            // The reset value lives on the *register operation*, not on the
            // fieldset: `TLow::default()` is still `Fieldset::ZERO`. The only
            // path where the reset value is observable is a `write()` whose
            // closure changes nothing, which transmits it verbatim. That is
            // also why the wrong defaults stayed latent — every public setter
            // replaces the whole fieldset and never reads the reset value.
            //
            // Asserted against explicit wire bytes rather than against
            // `Fieldset::ZERO`, because comparing against ZERO is precisely
            // what let the wrong defaults through.
            //
            // THIGH is the value the datasheet contradicts itself about: the
            // §7.5.4 prose says 0x7FF8, Table 11 shows the low nibble fixed
            // at zero (implying 0x7FF0). Measured on real hardware the part
            // resets to 0x7FF8, bit 3 set inside the reserved nibble. The
            // prose is right.
            let expectations = vec![
                Transaction::write(0x48, vec![0x02, 0x80, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x7f, 0xf8]),
            ];
            let mock = Mock::new(&expectations);
            let mut tmp = Tmp108::new_with_a0_gnd(mock);

            tmp.inner.t_low().write(|_| {}).unwrap();
            tmp.inner.t_high().write(|_| {}).unwrap();

            let mut mock = tmp.destroy();
            mock.done();
        }

        #[test]
        fn handle_a0_pin_accordingly() {
            let expectations = vec![];

            let mock = Mock::new(&expectations);
            let tmp = Tmp108::new_with_a0_gnd(mock);
            assert_eq!(tmp.addr(), 0x48);
            let mut mock = tmp.destroy();
            mock.done();
            let mock = Mock::new(&expectations);
            let tmp = Tmp108::new_with_a0_vplus(mock);
            assert_eq!(tmp.addr(), 0x49);
            let mut mock = tmp.destroy();
            mock.done();

            let mock = Mock::new(&expectations);
            let tmp = Tmp108::new_with_a0_sda(mock);
            assert_eq!(tmp.addr(), 0x4a);
            let mut mock = tmp.destroy();
            mock.done();

            let mock = Mock::new(&expectations);
            let tmp = Tmp108::new_with_a0_scl(mock);
            assert_eq!(tmp.addr(), 0x4b);
            let mut mock = tmp.destroy();
            mock.done();
        }

        #[test]
        fn change_configuration() {
            let expectations = vec![
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x66, 0xb0]),
                Transaction::write_read(0x48, vec![0x01], vec![0x66, 0xb0]),
            ];

            let mock = Mock::new(&expectations);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);
            let result = tmp108.read_configuration();
            assert!(result.is_ok());

            let config = result.unwrap();
            assert_eq!(config, Config::default());

            let config = Config {
                thermostat_mode: Thermostat::Interrupt,
                alert_polarity: Polarity::ActiveHigh,
                conversion_rate: ConversionRate::SixteenHz,
                hysteresis: Hysteresis::FourC,
            };

            let result = tmp108.configure(config);
            assert!(result.is_ok());

            let result = tmp108.read_configuration();
            assert!(result.is_ok());

            let new_config = result.unwrap();
            assert_eq!(config, new_config);

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[test]
        fn read_temperature_default_address() {
            let expectations = vec![
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x7f, 0xf0])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x64, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x50, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x4b, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x19, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x40])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xff, 0xc0])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xe7, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xc9, 0x00])],
            ];
            let temps = [127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

            for (e, t) in expectations.iter().zip(temps.iter()) {
                let mock = Mock::new(e);
                let mut tmp108 = Tmp108::new_with_a0_gnd(mock);
                let result = tmp108.temperature();
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp.to_degrees(), *t, 1e-4);

                let mut mock = tmp108.destroy();
                mock.done();
            }
        }

        #[test]
        fn set_and_read_low_limit() {
            let expectations = vec![
                Transaction::write(0x48, vec![0x02, 0x7f, 0xf0]),
                Transaction::write_read(0x48, vec![0x02], vec![0x7f, 0xf0]),
                Transaction::write(0x48, vec![0x02, 0x64, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x64, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x50, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x50, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x4b, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x4b, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x32, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x32, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x19, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x19, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x00, 0x40]),
                Transaction::write_read(0x48, vec![0x02], vec![0x00, 0x40]),
                Transaction::write(0x48, vec![0x02, 0x00, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x00, 0x00]),
                Transaction::write(0x48, vec![0x02, 0xff, 0xc0]),
                Transaction::write_read(0x48, vec![0x02], vec![0xff, 0xc0]),
                Transaction::write(0x48, vec![0x02, 0xe7, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0xe7, 0x00]),
                Transaction::write(0x48, vec![0x02, 0xc9, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0xc9, 0x00]),
            ];
            let temps = [127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

            let mock = Mock::new(&expectations);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

            for t in &temps {
                let limit = Celsius::try_from_degrees(*t).expect("datasheet value is representable");
                let result = tmp108.set_low_limit(limit);
                assert!(result.is_ok());

                let result = tmp108.low_limit();
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp.to_degrees(), *t, 1e-4);
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[test]
        fn set_and_read_high_limit() {
            let expectations = vec![
                Transaction::write(0x48, vec![0x03, 0x7f, 0xf0]),
                Transaction::write_read(0x48, vec![0x03], vec![0x7f, 0xf0]),
                Transaction::write(0x48, vec![0x03, 0x64, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x64, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x50, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x50, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x4b, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x4b, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x32, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x32, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x19, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x19, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x00, 0x40]),
                Transaction::write_read(0x48, vec![0x03], vec![0x00, 0x40]),
                Transaction::write(0x48, vec![0x03, 0x00, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x00, 0x00]),
                Transaction::write(0x48, vec![0x03, 0xff, 0xc0]),
                Transaction::write_read(0x48, vec![0x03], vec![0xff, 0xc0]),
                Transaction::write(0x48, vec![0x03, 0xe7, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0xe7, 0x00]),
                Transaction::write(0x48, vec![0x03, 0xc9, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0xc9, 0x00]),
            ];
            let temps = [127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

            let mock = Mock::new(&expectations);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

            for t in &temps {
                let limit = Celsius::try_from_degrees(*t).expect("datasheet value is representable");
                let result = tmp108.set_high_limit(limit);
                assert!(result.is_ok());

                let result = tmp108.high_limit();
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp.to_degrees(), *t, 1e-4);
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[test]
        fn probe_returns_true_for_por_value() {
            // Configuration register at 0x01 returns the POR value 0x1022.
            // The register layout is little-endian per tmp108.toml so the
            // wire bytes are [0x22, 0x10].
            let expectations = vec![Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10])];
            let mock = Mock::new(&expectations);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

            assert_eq!(tmp108.probe(), Ok(true));

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[test]
        fn probe_returns_false_for_non_por_value() {
            // Chip is present (ACKs) but has been reconfigured.
            let expectations = vec![Transaction::write_read(0x48, vec![0x01], vec![0x66, 0xb0])];
            let mock = Mock::new(&expectations);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

            assert_eq!(tmp108.probe(), Ok(false));

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[test]
        fn probe_propagates_bus_error() {
            let expectations = vec![Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]).with_error(
                embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Address),
            )];
            let mock = Mock::new(&expectations);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

            assert!(tmp108.probe().is_err());

            let mut mock = tmp108.destroy();
            mock.done();
        }
    }

    #[cfg(feature = "async")]
    mod asynchronous {
        use assert_approx_eq::assert_approx_eq;
        use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

        use super::*;

        #[tokio::test]
        async fn handle_a0_pin_accordingly() {
            let expectations = vec![];

            let mock = Mock::new(&expectations);
            let tmp = AsyncTmp108::new_with_a0_gnd(mock);
            assert_eq!(tmp.addr(), 0x48);
            let mut mock = tmp.destroy();
            mock.done();

            let mock = Mock::new(&expectations);
            let tmp = AsyncTmp108::new_with_a0_vplus(mock);
            assert_eq!(tmp.addr(), 0x49);
            let mut mock = tmp.destroy();
            mock.done();

            let mock = Mock::new(&expectations);
            let tmp = AsyncTmp108::new_with_a0_sda(mock);
            assert_eq!(tmp.addr(), 0x4a);
            let mut mock = tmp.destroy();
            mock.done();

            let mock = Mock::new(&expectations);
            let tmp = AsyncTmp108::new_with_a0_scl(mock);
            assert_eq!(tmp.addr(), 0x4b);
            let mut mock = tmp.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn change_configuration() {
            let expectations = vec![
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x66, 0xb0]),
                Transaction::write_read(0x48, vec![0x01], vec![0x66, 0xb0]),
            ];

            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);
            let result = tmp108.read_configuration().await;
            assert!(result.is_ok());

            let config = result.unwrap();
            assert_eq!(config, Config::default());

            let config = Config {
                thermostat_mode: Thermostat::Interrupt,
                alert_polarity: Polarity::ActiveHigh,
                conversion_rate: ConversionRate::SixteenHz,
                hysteresis: Hysteresis::FourC,
            };

            let result = tmp108.configure(config).await;
            assert!(result.is_ok());

            let result = tmp108.read_configuration().await;
            assert!(result.is_ok());

            let new_config = result.unwrap();
            assert_eq!(config, new_config);

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn read_temperature_default_address() {
            let expectations = vec![
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x7f, 0xf0])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x64, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x50, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x4b, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x19, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x40])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xff, 0xc0])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xe7, 0x00])],
                vec![Transaction::write_read(0x48, vec![0x00], vec![0xc9, 0x00])],
            ];
            let temps = [127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

            for (e, t) in expectations.iter().zip(temps.iter()) {
                let mock = Mock::new(e);
                let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);
                let result = tmp108.temperature().await;
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp.to_degrees(), *t, 1e-4);

                let mut mock = tmp108.destroy();
                mock.done();
            }
        }

        #[tokio::test]
        async fn set_and_read_high_limit() {
            let expectations = vec![
                Transaction::write(0x48, vec![0x03, 0x7f, 0xf0]),
                Transaction::write_read(0x48, vec![0x03], vec![0x7f, 0xf0]),
                Transaction::write(0x48, vec![0x03, 0x64, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x64, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x50, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x50, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x4b, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x4b, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x32, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x32, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x19, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x19, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x00, 0x40]),
                Transaction::write_read(0x48, vec![0x03], vec![0x00, 0x40]),
                Transaction::write(0x48, vec![0x03, 0x00, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0x00, 0x00]),
                Transaction::write(0x48, vec![0x03, 0xff, 0xc0]),
                Transaction::write_read(0x48, vec![0x03], vec![0xff, 0xc0]),
                Transaction::write(0x48, vec![0x03, 0xe7, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0xe7, 0x00]),
                Transaction::write(0x48, vec![0x03, 0xc9, 0x00]),
                Transaction::write_read(0x48, vec![0x03], vec![0xc9, 0x00]),
            ];
            let temps = [127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            for t in &temps {
                let limit = Celsius::try_from_degrees(*t).expect("datasheet value is representable");
                let result = tmp108.set_high_limit(limit).await;
                assert!(result.is_ok());

                let result = tmp108.high_limit().await;
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp.to_degrees(), *t, 1e-4);
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn set_and_read_low_limit() {
            let expectations = vec![
                Transaction::write(0x48, vec![0x02, 0x7f, 0xf0]),
                Transaction::write_read(0x48, vec![0x02], vec![0x7f, 0xf0]),
                Transaction::write(0x48, vec![0x02, 0x64, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x64, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x50, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x50, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x4b, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x4b, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x32, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x32, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x19, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x19, 0x00]),
                Transaction::write(0x48, vec![0x02, 0x00, 0x40]),
                Transaction::write_read(0x48, vec![0x02], vec![0x00, 0x40]),
                Transaction::write(0x48, vec![0x02, 0x00, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0x00, 0x00]),
                Transaction::write(0x48, vec![0x02, 0xff, 0xc0]),
                Transaction::write_read(0x48, vec![0x02], vec![0xff, 0xc0]),
                Transaction::write(0x48, vec![0x02, 0xe7, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0xe7, 0x00]),
                Transaction::write(0x48, vec![0x02, 0xc9, 0x00]),
                Transaction::write_read(0x48, vec![0x02], vec![0xc9, 0x00]),
            ];
            let temps = [127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            for t in &temps {
                let limit = Celsius::try_from_degrees(*t).expect("datasheet value is representable");
                let result = tmp108.set_low_limit(limit).await;
                assert!(result.is_ok());

                let result = tmp108.low_limit().await;
                assert!(result.is_ok());

                let temp = result.unwrap();
                assert_approx_eq!(temp.to_degrees(), *t, 1e-4);
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        /// The `set_*_limit` methods can no longer reject their input —
        /// [`Celsius`] carries the range invariant. The remaining
        /// f32 boundary is the `TemperatureThresholdSet` trait, which
        /// must still reject unrepresentable degrees before any bus
        /// traffic occurs.
        #[cfg(feature = "embedded-sensors-hal-async")]
        #[tokio::test]
        async fn threshold_set_rejects_unrepresentable_degrees() {
            use embedded_sensors_hal_async::temperature::TemperatureThresholdSet;

            // No I2C transactions are expected.
            let mock = Mock::new(&[]);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            // Values outside the representable [-128.0, 127.9375] range.
            // Note 127.940 now *rounds* to 127.9375 and is accepted, so the
            // rejection cases start half an LSB beyond the endpoints.
            for bad in [128.0_f32, 127.98_f32, -128.05_f32, -200.0_f32, 200.0_f32] {
                assert!(matches!(
                    tmp108.set_temperature_threshold_low(bad).await,
                    Err(Error::InvalidInput)
                ));
                assert!(matches!(
                    tmp108.set_temperature_threshold_high(bad).await,
                    Err(Error::InvalidInput)
                ));
            }

            // Non-finite values.
            for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                assert!(matches!(
                    tmp108.set_temperature_threshold_low(bad).await,
                    Err(Error::InvalidInput)
                ));
                assert!(matches!(
                    tmp108.set_temperature_threshold_high(bad).await,
                    Err(Error::InvalidInput)
                ));
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn probe_returns_true_for_por_value() {
            let expectations = vec![Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10])];
            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            assert_eq!(tmp108.probe().await, Ok(true));

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn probe_returns_false_for_non_por_value() {
            let expectations = vec![Transaction::write_read(0x48, vec![0x01], vec![0x66, 0xb0])];
            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            assert_eq!(tmp108.probe().await, Ok(false));

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn continuous_runs_shutdown_when_closure_returns_err() {
            // Expectations:
            //  1. Enter Continuous: read cfg, write cfg with M=Continuous.
            //  2. Closure causes one temperature read that returns a bus error.
            //  3. Cleanup must still run: read cfg, write cfg with M=Shutdown.
            //
            // If the cleanup is skipped (the pre-fix behavior) the mock will
            // panic at destroy() because the last two expectations were not
            // consumed.
            let expectations = vec![
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x22, 0x10]),
                Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00])
                    .with_error(embedded_hal::i2c::ErrorKind::Other),
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x20, 0x10]),
            ];
            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            let result = tmp108
                .continuous(async |t| {
                    let _ = t.temperature().await?;
                    Ok(())
                })
                .await;

            // The closure's error must be propagated (closure error wins
            // over shutdown success).
            assert!(result.is_err());

            // All five expected transactions consumed -> the cleanup
            // shutdown ran.
            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[tokio::test]
        async fn continuous_returns_closure_error_when_shutdown_also_fails() {
            use embedded_hal_async::i2c::Error as _;

            // Closure fails AND shutdown fails. The closure's error must win.
            let closure_err = embedded_hal::i2c::ErrorKind::Bus;
            let shutdown_err = embedded_hal::i2c::ErrorKind::ArbitrationLoss;

            let expectations = vec![
                // Enter Continuous.
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x22, 0x10]),
                // Closure errors.
                Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00]).with_error(closure_err),
                // Shutdown read errors too.
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]).with_error(shutdown_err),
            ];
            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            let result = tmp108
                .continuous(async |t| {
                    let _ = t.temperature().await?;
                    Ok(())
                })
                .await;

            // Must propagate the closure error, not the shutdown error.
            assert_eq!(result.err().map(|e| e.kind()), Some(closure_err));

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[cfg(feature = "embedded-sensors-hal-async")]
        #[tokio::test]
        async fn handle_threshold_alerts_properly() {
            use embedded_hal_mock::eh1::digital;
            use embedded_sensors_hal_async::temperature::{TemperatureThresholdSet, TemperatureThresholdWait};

            // Sensor i2c bus mocks and expectations
            let i2c_expectations = vec![
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x26, 0x10]),
                Transaction::write(0x48, vec![0x02, 0x19, 0x00]),
                Transaction::write(0x48, vec![0x03, 0x50, 0x00]),
                // C0: no flag is latched yet, so the waiter goes on to
                // wait on the pin.
                Transaction::write_read(0x48, vec![0x01], vec![0x26, 0x10]),
                // C1: acknowledge the assertion the level wait observed.
                Transaction::write_read(0x48, vec![0x01], vec![0x26, 0x10]),
                Transaction::write_read(0x48, vec![0x00], vec![0x50, 0x00]),
            ];
            let i2c_mock = Mock::new(&i2c_expectations);

            // Threshold alert GPIO pin mocks and expectations. Interrupt
            // mode waits for the asserted *level*, never an edge: the
            // entry configuration read has already released the pin, so
            // an edge that happened before the wait was armed is gone.
            let pin_expectations = vec![digital::Transaction::wait_for_state(digital::State::Low)];
            let pin_mock = digital::Mock::new(&pin_expectations);

            // Create a ALERTTMP108 instance and configure it as active-low interrupt mode
            let mut tmp108 = AlertTmp108::new_with_a0_gnd(i2c_mock, pin_mock);

            let cfg = Config {
                thermostat_mode: Thermostat::Interrupt,
                alert_polarity: Polarity::ActiveLow,
                ..Default::default()
            };

            let result = tmp108.sensor_mut().configure(cfg).await;
            assert!(result.is_ok());

            // Set alert thresholds
            let result = tmp108.set_temperature_threshold_low(25.0).await;
            assert!(result.is_ok());
            let result = tmp108.set_temperature_threshold_high(80.0).await;
            assert!(result.is_ok());

            // Ensure alert pin waits for the asserted level
            let result = tmp108.wait_for_temperature_threshold().await;
            assert!(result.is_ok());

            // Check that recently sampled temperature is returned
            let temp = result.unwrap();
            assert_approx_eq!(temp, 80.0, 1e-4);

            let (mut i2c_mock, mut pin_mock) = tmp108.destroy();
            i2c_mock.done();
            pin_mock.done();
        }

        #[cfg(feature = "embedded-sensors-hal-async")]
        #[tokio::test]
        async fn hysteresis_snaps_within_tolerance() {
            use embedded_sensors_hal_async::temperature::TemperatureHysteresis;

            // For each legal value, every input within 0.05 °C must
            // succeed and program the corresponding chip setting (no I2C
            // mismatch). Each acceptance path performs: read cfg, write cfg.
            //
            // - 0.0 °C snaps to Hysteresis::ZeroC => HYS bits 0b00 => cfg word 0x0022 -> bytes [0x22, 0x00]
            // - 1.0 °C snaps to Hysteresis::OneC => HYS bits 0b01 => cfg word 0x1022 -> bytes [0x22, 0x10]
            // - 2.0 °C snaps to Hysteresis::TwoC => HYS bits 0b10 => cfg word 0x2022 -> bytes [0x22, 0x20]
            // - 4.0 °C snaps to Hysteresis::FourC => HYS bits 0b11 => cfg word 0x3022 -> bytes [0x22, 0x30]
            //
            // For each accepted input we expect: write-read of cfg, then a
            // write of the new cfg. The chip's POR is 0x1022 (HYS=01).
            let cases: &[(f32, [u8; 2])] = &[
                // Exact-match accepted values.
                (0.0, [0x22, 0x00]),
                (1.0, [0x22, 0x10]),
                (2.0, [0x22, 0x20]),
                (4.0, [0x22, 0x30]),
                // Within-tolerance inputs that previously failed under
                // f32::EPSILON snapping.
                (0.04_f32, [0x22, 0x00]),
                (1.000_000_1_f32, [0x22, 0x10]),
                (0.1_f32 + 0.9_f32, [0x22, 0x10]),
                (1.95_f32, [0x22, 0x20]),
                (3.97_f32, [0x22, 0x30]),
            ];

            // Each accepted input triggers:
            //   1. read_configuration() in the hysteresis impl: write_read
            //   2. configure() -> modify (read-modify-write): write_read + write
            // The current cfg byte stream is the chip's POR value 0x1022 ->
            // [0x22, 0x10]. After snapping the result is reflected in the
            // HYS bits of the final write.
            let mut expectations = Vec::new();
            for (_, written) in cases {
                // read_configuration
                expectations.push(Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]));
                // configure -> modify: read
                expectations.push(Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]));
                // configure -> modify: write
                expectations.push(Transaction::write(0x48, vec![0x01, written[0], written[1]]));
            }

            let mock = Mock::new(&expectations);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            for (input, _) in cases {
                let r = tmp108.set_temperature_threshold_hysteresis(*input).await;
                assert!(r.is_ok(), "input {input} should be accepted");
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[cfg(feature = "embedded-sensors-hal-async")]
        #[tokio::test]
        async fn hysteresis_rejects_out_of_tolerance_and_non_finite() {
            use embedded_sensors_hal_async::temperature::TemperatureHysteresis;

            // No I2C transactions expected; out-of-tolerance and non-finite
            // inputs must be rejected before any bus traffic.
            let mock = Mock::new(&[]);
            let mut tmp108 = AsyncTmp108::new_with_a0_gnd(mock);

            for bad in [-0.5_f32, 0.5_f32, 3.0_f32, 5.0_f32, -1.0_f32, 10.0_f32] {
                let r = tmp108.set_temperature_threshold_hysteresis(bad).await;
                assert!(
                    matches!(r, Err(Error::InvalidInput)),
                    "input {bad} should be rejected as InvalidInput, got {r:?}"
                );
            }

            for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                let r = tmp108.set_temperature_threshold_hysteresis(bad).await;
                assert!(
                    matches!(r, Err(Error::InvalidInput)),
                    "input {bad} should be rejected as InvalidInput, got {r:?}"
                );
            }

            let mut mock = tmp108.destroy();
            mock.done();
        }

        #[cfg(feature = "embedded-sensors-hal-async")]
        #[tokio::test]
        async fn alert_pin_error_is_propagated_as_error_pin() {
            use embedded_hal_mock::eh1::{MockError, digital};
            use embedded_sensors_hal_async::temperature::TemperatureThresholdWait;

            // Configure for Interrupt + ActiveLow so the level wait
            // (wait_for_low) is the gating operation. The pin then
            // errors; the driver must surface the GPIO error via
            // Error::Pin(_), not swallow it (the pre-fix behavior
            // collapsed all GPIO failures to Error::Other).
            let i2c_expectations = vec![
                // configure: read + write
                Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
                Transaction::write(0x48, vec![0x01, 0x26, 0x10]),
                // wait_for_temperature_threshold reads cfg first; no flag
                // is latched, so it goes on to wait on the pin.
                Transaction::write_read(0x48, vec![0x01], vec![0x26, 0x10]),
            ];
            let i2c_mock = Mock::new(&i2c_expectations);

            let pin_err = MockError::Io(std::io::ErrorKind::Other);
            let pin_expectations =
                vec![digital::Transaction::wait_for_state(digital::State::Low).with_error(pin_err.clone())];
            let pin_mock = digital::Mock::new(&pin_expectations);

            let mut tmp108 = AlertTmp108::new_with_a0_gnd(i2c_mock, pin_mock);

            let cfg = Config {
                thermostat_mode: Thermostat::Interrupt,
                alert_polarity: Polarity::ActiveLow,
                ..Default::default()
            };
            tmp108.sensor_mut().configure(cfg).await.unwrap();

            let result = tmp108.wait_for_temperature_threshold().await;
            match result {
                Err(Error::Pin(e)) => assert_eq!(e, pin_err),
                other => panic!("expected Error::Pin, got {other:?}"),
            }

            let (mut i2c_mock, mut pin_mock) = tmp108.destroy();
            i2c_mock.done();
            pin_mock.done();
        }

        /// Regression tests for the alert-wait ordering defect (#59).
        ///
        /// See
        /// `docs/superpowers/specs/2026-09-24-tmp108-issue-59-alert-wait-ordering-design.md`
        /// section 7. Every test here is driven by manual polling, so a
        /// waiter that parks on a notification that will never arrive
        /// fails immediately instead of hanging the suite.
        #[cfg(feature = "embedded-sensors-hal-async")]
        mod alert_ordering {
            use core::future::Future;
            use core::pin::pin;
            use core::task::{Context, Poll, Waker};
            use std::sync::{Arc, Mutex};

            use embedded_hal::digital::InputPin;
            use embedded_sensors_hal_async::temperature::TemperatureThresholdWait;

            use super::*;

            type I2cError = embedded_hal::i2c::ErrorKind;
            type PinError = embedded_hal::digital::ErrorKind;

            const ADDR: u8 = 0x48;

            /// 25 °C, per design section 2.3.
            const T25: [u8; 2] = [0x19, 0x00];
            /// 80 °C, per design section 2.3.
            const T80: [u8; 2] = [0x50, 0x00];

            /// Configuration byte 0 with FL/FH cleared.
            const INTERRUPT: u8 = 0x26;
            const COMPARATOR: u8 = 0x22;
            /// Configuration byte 1.
            const ACTIVE_LOW: u8 = 0x10;
            const ACTIVE_HIGH: u8 = 0x90;

            // ---------------------------------------------------------------
            // Shared operation log
            // ---------------------------------------------------------------

            #[derive(Clone, Copy, Debug, PartialEq, Eq)]
            enum PinOp {
                WaitLow,
                WaitHigh,
                WaitFalling,
                WaitRising,
                WaitAnyEdge,
                IsLow,
                IsHigh,
            }

            #[derive(Clone, Debug, PartialEq, Eq)]
            enum Op {
                /// `write_read(ADDR, written, returned)`.
                WriteRead(Vec<u8>, Vec<u8>),
                /// `write(ADDR, written)`.
                Write(Vec<u8>),
                /// A pin operation was *started*.
                PinCall(PinOp),
                /// A pin operation *completed* (successfully or with an error).
                PinDone(PinOp),
            }

            // ---------------------------------------------------------------
            // Scripted I2C steps
            // ---------------------------------------------------------------

            #[derive(Clone, Debug)]
            enum Resp {
                /// A write: nothing is returned.
                None,
                /// A fixed two-byte response (temperature, limits).
                Fixed([u8; 2]),
                /// A configuration response whose FL/FH bits are taken from
                /// the modeled chip state at the moment the transaction runs.
                Config { base0: u8, byte1: u8 },
            }

            #[derive(Clone, Copy, Debug)]
            enum Outcome {
                Ok,
                Err(I2cError),
                /// The transaction future never resolves.
                Pending,
                /// The transaction is recorded immediately — including its
                /// response bytes — but its future stays `Pending` until
                /// the test opens the gate, after which it resolves
                /// successfully *without* running a second transaction.
                ///
                /// Design section 8.3 D: `Pending` alone cannot express a
                /// delayed completion, because it never resolves.
                Gated,
            }

            /// One scripted I2C transaction.
            ///
            /// `effect` models the *hardware side effect* of the transaction
            /// and is applied before `outcome` is honoured, so a transaction
            /// can acknowledge the chip and then fail — the case the
            /// reliability reviewer requires (R1).
            struct Step {
                write: Vec<u8>,
                resp: Resp,
                effect: Option<fn(&mut World)>,
                outcome: Outcome,
            }

            impl Step {
                fn effect(mut self, f: fn(&mut World)) -> Self {
                    self.effect = Some(f);
                    self
                }

                fn err(mut self, e: I2cError) -> Self {
                    self.outcome = Outcome::Err(e);
                    self
                }

                fn pending(mut self) -> Self {
                    self.outcome = Outcome::Pending;
                    self
                }

                /// See [`Outcome::Gated`].
                fn gated(mut self) -> Self {
                    self.outcome = Outcome::Gated;
                    self
                }
            }

            /// A configuration read (C0 or C1): `write_read(ADDR, [0x01], _)`.
            fn cfg(base0: u8, byte1: u8) -> Step {
                Step {
                    write: vec![0x01],
                    resp: Resp::Config { base0, byte1 },
                    effect: None,
                    outcome: Outcome::Ok,
                }
            }

            /// A temperature read (T): `write_read(ADDR, [0x00], _)`.
            fn temp(bytes: [u8; 2]) -> Step {
                Step {
                    write: vec![0x00],
                    resp: Resp::Fixed(bytes),
                    effect: None,
                    outcome: Outcome::Ok,
                }
            }

            /// A register write.
            fn write(bytes: Vec<u8>) -> Step {
                Step {
                    write: bytes,
                    resp: Resp::None,
                    effect: None,
                    outcome: Outcome::Ok,
                }
            }

            // ---------------------------------------------------------------
            // Modeled chip + pin
            // ---------------------------------------------------------------

            // These are test fixtures, not domain models: the flags are
            // an explicit script of chip/pin state, and collapsing them
            // into enums would obscure what each case sets up.
            #[allow(clippy::struct_excessive_bools)]
            struct World {
                log: Vec<Op>,
                steps: std::collections::VecDeque<Step>,
                /// FL latched in the chip.
                fl: bool,
                /// FH latched in the chip.
                fh: bool,
                /// Current ALERT level.
                level_high: bool,
                /// An unconsumed observation of the pin having been HIGH.
                sticky_high: bool,
                /// An unconsumed observation of the pin having been LOW.
                sticky_low: bool,
                /// Panic on any pin call. Makes "must not touch GPIO"
                /// deterministic instead of a timeout.
                strict_pin: bool,
                /// Error returned by the next *level* wait.
                pin_error: Option<PinError>,
                /// Whether [`Outcome::Gated`] transactions may now resolve.
                gate_open: bool,
            }

            impl World {
                fn set_level(&mut self, high: bool) {
                    self.level_high = high;
                    if high {
                        self.sticky_high = true;
                        self.sticky_low = false;
                    } else {
                        self.sticky_low = true;
                        self.sticky_high = false;
                    }
                }

                /// Drive the pin to `high` and immediately back, leaving the
                /// observation of `high` unconsumed. Models R4: the level
                /// wait completed, but the level went inactive again before
                /// the woken task resumed.
                fn pulse_level(&mut self, high: bool) {
                    self.set_level(high);
                    self.level_high = !high;
                }

                fn sticky(&self, high: bool) -> bool {
                    if high { self.sticky_high } else { self.sticky_low }
                }

                fn clear_sticky(&mut self, high: bool) {
                    if high {
                        self.sticky_high = false;
                    } else {
                        self.sticky_low = false;
                    }
                }
            }

            type Shared = Arc<Mutex<World>>;

            /// C0's documented hardware side effect in interrupt mode:
            /// clear FL/FH and release the pin (`ActiveLow` -> HIGH).
            fn ack_release_high(w: &mut World) {
                w.fl = false;
                w.fh = false;
                w.set_level(true);
            }

            /// As above for `ActiveHigh`: release the pin to LOW.
            fn ack_release_low(w: &mut World) {
                w.fl = false;
                w.fh = false;
                w.set_level(false);
            }

            /// A new alert asserts `ActiveLow` between C0 and GPIO arming.
            fn assert_low(w: &mut World) {
                w.set_level(false);
            }

            /// A new alert asserts `ActiveHigh` between C0 and GPIO arming.
            fn assert_high(w: &mut World) {
                w.set_level(true);
            }

            // ---------------------------------------------------------------
            // Fakes
            // ---------------------------------------------------------------

            struct FakeI2c {
                shared: Shared,
            }

            impl embedded_hal_async::i2c::ErrorType for FakeI2c {
                type Error = I2cError;
            }

            impl FakeI2c {
                /// Consume one scripted step, record it, apply its hardware
                /// side effect, and return its outcome.
                fn step(&mut self, addr: u8, written: &[u8], read: Option<&mut [u8]>) -> Outcome {
                    let mut w = self.shared.lock().unwrap();
                    assert_eq!(addr, ADDR, "unexpected I2C address");
                    let step = w
                        .steps
                        .pop_front()
                        .unwrap_or_else(|| panic!("unexpected I2C transaction, wrote {written:x?}"));
                    assert_eq!(written, &step.write[..], "unexpected I2C write payload");

                    match (step.resp.clone(), read) {
                        (Resp::None, None) => w.log.push(Op::Write(written.to_vec())),
                        (Resp::Fixed(bytes), Some(buf)) => {
                            assert_eq!(buf.len(), 2, "unexpected read length");
                            buf.copy_from_slice(&bytes);
                            w.log.push(Op::WriteRead(written.to_vec(), bytes.to_vec()));
                        }
                        (Resp::Config { base0, byte1 }, Some(buf)) => {
                            assert_eq!(buf.len(), 2, "unexpected read length");
                            let b0 = base0 | u8::from(w.fl) << 3 | u8::from(w.fh) << 4;
                            let bytes = [b0, byte1];
                            buf.copy_from_slice(&bytes);
                            w.log.push(Op::WriteRead(written.to_vec(), bytes.to_vec()));
                        }
                        (resp, _) => panic!("scripted response {resp:?} does not match the transaction shape"),
                    }

                    if let Some(effect) = step.effect {
                        effect(&mut w);
                    }

                    step.outcome
                }

                /// Resolve a scripted outcome. The transaction itself has
                /// already been recorded by [`FakeI2c::step`], so a
                /// [`Outcome::Gated`] completion adds no further log entry.
                async fn finish(&self, outcome: Outcome) -> Result<(), I2cError> {
                    match outcome {
                        Outcome::Ok => Ok(()),
                        Outcome::Err(e) => Err(e),
                        Outcome::Pending => core::future::pending().await,
                        Outcome::Gated => {
                            let shared = self.shared.clone();
                            core::future::poll_fn(move |_cx| {
                                if shared.lock().unwrap().gate_open {
                                    Poll::Ready(Ok(()))
                                } else {
                                    Poll::Pending
                                }
                            })
                            .await
                        }
                    }
                }
            }

            impl embedded_hal_async::i2c::I2c for FakeI2c {
                async fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
                    let outcome = self.step(addr, bytes, None);
                    self.finish(outcome).await
                }

                async fn read(&mut self, _addr: u8, _buf: &mut [u8]) -> Result<(), Self::Error> {
                    unimplemented!("the driver never issues a bare read")
                }

                async fn write_read(&mut self, addr: u8, bytes: &[u8], buf: &mut [u8]) -> Result<(), Self::Error> {
                    let outcome = self.step(addr, bytes, Some(buf));
                    self.finish(outcome).await
                }

                async fn transaction(
                    &mut self,
                    _addr: u8,
                    _ops: &mut [embedded_hal_async::i2c::Operation<'_>],
                ) -> Result<(), Self::Error> {
                    unimplemented!("the driver never issues a compound transaction")
                }
            }

            struct FakePin {
                shared: Shared,
            }

            impl embedded_hal::digital::ErrorType for FakePin {
                type Error = PinError;
            }

            impl FakePin {
                fn record(&self, op: PinOp) {
                    let mut w = self.shared.lock().unwrap();
                    w.log.push(Op::PinCall(op));
                    assert!(
                        !w.strict_pin,
                        "strict pin fake: the waiter performed {op:?}, but this fixture models an \
                         already-latched event whose excursion has ended — no pin notification will \
                         ever arrive"
                    );
                }

                /// A level wait per `embedded_hal_async::digital::Wait`:
                /// returns immediately if the level is already active.
                async fn level_wait(&mut self, op: PinOp, want_high: bool) -> Result<(), PinError> {
                    self.record(op);
                    let shared = self.shared.clone();
                    core::future::poll_fn(move |_cx| {
                        let mut w = shared.lock().unwrap();
                        if let Some(e) = w.pin_error.take() {
                            w.log.push(Op::PinDone(op));
                            return Poll::Ready(Err(e));
                        }
                        if w.level_high == want_high || w.sticky(want_high) {
                            w.clear_sticky(want_high);
                            w.log.push(Op::PinDone(op));
                            Poll::Ready(Ok(()))
                        } else {
                            Poll::Pending
                        }
                    })
                    .await
                }

                /// An edge wait. These fixtures deliberately never schedule a
                /// further transition, so an edge wait parks forever — which
                /// is exactly the hardware-observed hang.
                async fn edge_wait(&mut self, op: PinOp) -> Result<(), PinError> {
                    self.record(op);
                    core::future::pending().await
                }
            }

            impl embedded_hal_async::digital::Wait for FakePin {
                async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
                    self.level_wait(PinOp::WaitHigh, true).await
                }

                async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
                    self.level_wait(PinOp::WaitLow, false).await
                }

                async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
                    self.edge_wait(PinOp::WaitRising).await
                }

                async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
                    self.edge_wait(PinOp::WaitFalling).await
                }

                async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
                    self.edge_wait(PinOp::WaitAnyEdge).await
                }
            }

            impl InputPin for FakePin {
                fn is_high(&mut self) -> Result<bool, Self::Error> {
                    self.record(PinOp::IsHigh);
                    Ok(self.shared.lock().unwrap().level_high)
                }

                fn is_low(&mut self) -> Result<bool, Self::Error> {
                    self.record(PinOp::IsLow);
                    Ok(!self.shared.lock().unwrap().level_high)
                }
            }

            // ---------------------------------------------------------------
            // Harness
            // ---------------------------------------------------------------

            // As for `World`: an explicit per-case script, not a model.
            #[allow(clippy::struct_excessive_bools)]
            struct Setup {
                level_high: bool,
                fl: bool,
                fh: bool,
                strict_pin: bool,
                pin_error: Option<PinError>,
                steps: Vec<Step>,
            }

            impl Default for Setup {
                fn default() -> Self {
                    Self {
                        level_high: true,
                        fl: false,
                        fh: false,
                        strict_pin: false,
                        pin_error: None,
                        steps: Vec::new(),
                    }
                }
            }

            fn build(setup: Setup) -> (Shared, AlertTmp108<FakeI2c, FakePin>) {
                let shared = Arc::new(Mutex::new(World {
                    log: Vec::new(),
                    steps: setup.steps.into_iter().collect(),
                    fl: setup.fl,
                    fh: setup.fh,
                    level_high: setup.level_high,
                    sticky_high: setup.level_high,
                    sticky_low: !setup.level_high,
                    strict_pin: setup.strict_pin,
                    pin_error: setup.pin_error,
                    gate_open: false,
                }));
                let i2c = FakeI2c { shared: shared.clone() };
                let pin = FakePin { shared: shared.clone() };
                (shared, AlertTmp108::new_with_a0_gnd(i2c, pin))
            }

            fn poll_once<F: Future>(fut: core::pin::Pin<&mut F>) -> Poll<F::Output> {
                let mut cx = Context::from_waker(Waker::noop());
                fut.poll(&mut cx)
            }

            fn log(shared: &Shared) -> Vec<Op> {
                shared.lock().unwrap().log.clone()
            }

            fn take_log(shared: &Shared) -> Vec<Op> {
                core::mem::take(&mut shared.lock().unwrap().log)
            }

            fn pin_calls(shared: &Shared) -> Vec<PinOp> {
                log(shared)
                    .into_iter()
                    .filter_map(|op| match op {
                        Op::PinCall(p) => Some(p),
                        _ => None,
                    })
                    .collect()
            }

            /// The written payload of each I2C transaction, in order.
            fn i2c_writes(shared: &Shared) -> Vec<Vec<u8>> {
                log(shared)
                    .into_iter()
                    .filter_map(|op| match op {
                        Op::WriteRead(w, _) | Op::Write(w) => Some(w),
                        _ => None,
                    })
                    .collect()
            }

            fn steps_left(shared: &Shared) -> usize {
                shared.lock().unwrap().steps.len()
            }

            fn push_steps(shared: &Shared, steps: Vec<Step>) {
                shared.lock().unwrap().steps.extend(steps);
            }

            type WaitPoll = Poll<Result<f32, Error<I2cError, PinError>>>;

            /// Assert a bus error, with a diagnostic that names the GPIO
            /// operations the waiter performed if it parked instead.
            fn expect_bus(shared: &Shared, p: WaitPoll, want: I2cError) {
                match p {
                    Poll::Ready(Err(Error::Bus(e))) => assert_eq!(e, want),
                    Poll::Pending => panic!(
                        "waiter parked instead of returning Err(Bus({want:?})); pin operations so \
                         far: {:?}",
                        pin_calls(shared)
                    ),
                    other @ Poll::Ready(_) => panic!("expected Err(Bus({want:?})), got {other:?}"),
                }
            }

            /// As [`expect_bus`], for pin errors.
            fn expect_pin(shared: &Shared, p: WaitPoll, want: PinError) {
                match p {
                    Poll::Ready(Err(Error::Pin(e))) => assert_eq!(e, want),
                    Poll::Pending => panic!(
                        "waiter parked instead of returning Err(Pin({want:?})); pin operations so \
                         far: {:?}",
                        pin_calls(shared)
                    ),
                    other @ Poll::Ready(_) => panic!("expected Err(Pin({want:?})), got {other:?}"),
                }
            }

            fn degrees(p: WaitPoll) -> f32 {
                match p {
                    Poll::Ready(Ok(t)) => t,
                    Poll::Ready(Err(e)) => panic!("expected a temperature, got {e:?}"),
                    Poll::Pending => panic!(
                        "waiter parked instead of completing — it is waiting for a notification \
                         this fixture will never deliver"
                    ),
                }
            }

            // ===============================================================
            // Section 7.2 — transient regression: already latched, excursion
            // over, and no further assertion will ever occur.
            // ===============================================================

            /// The crown-jewel regression. Shared body for all six flag /
            /// polarity permutations.
            fn transient_case(byte1: u8, fl: bool, fh: bool) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let release: fn(&mut World) = if asserted_high {
                    ack_release_low
                } else {
                    ack_release_high
                };

                let (shared, mut tmp) = build(Setup {
                    // The pin is already asserted.
                    level_high: asserted_high,
                    fl,
                    fh,
                    // Any GPIO touch is a bug: there is nothing left to observe.
                    strict_pin: true,
                    steps: vec![cfg(INTERRUPT, byte1).effect(release), temp(T25)],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };

                assert!(
                    pin_calls(&shared).is_empty(),
                    "a pending entry flag must suppress every GPIO operation, saw {:?}",
                    pin_calls(&shared)
                );
                assert_approx_eq!(degrees(result), 25.0, 1e-4);
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x01], vec![0x00]],
                    "the pending fast path performs exactly C0 and T (no C1)"
                );
                assert_eq!(steps_left(&shared), 0);
            }

            #[test]
            fn transient_latched_fh_active_low_completes_without_gpio() {
                transient_case(ACTIVE_LOW, false, true);
            }

            #[test]
            fn transient_latched_fl_active_low_completes_without_gpio() {
                transient_case(ACTIVE_LOW, true, false);
            }

            #[test]
            fn transient_latched_both_flags_active_low_completes_once() {
                transient_case(ACTIVE_LOW, true, true);
            }

            #[test]
            fn transient_latched_fh_active_high_completes_without_gpio() {
                transient_case(ACTIVE_HIGH, false, true);
            }

            #[test]
            fn transient_latched_fl_active_high_completes_without_gpio() {
                transient_case(ACTIVE_HIGH, true, false);
            }

            #[test]
            fn transient_latched_both_flags_active_high_completes_once() {
                transient_case(ACTIVE_HIGH, true, true);
            }

            /// The literal liveness variant from section 7.2: the modeled
            /// chip really does release the pin and clear the flags when C0
            /// completes, and nothing ever re-asserts. A waiter that parks on
            /// the pin parks forever.
            #[test]
            fn transient_liveness_no_further_assertion_ever_arrives() {
                let (shared, mut tmp) = build(Setup {
                    level_high: false,
                    fh: true,
                    steps: vec![cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high), temp(T25)],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };

                assert_eq!(
                    pin_calls(&shared),
                    Vec::<PinOp>::new(),
                    "no GPIO operation may be performed for an event already captured by C0"
                );
                assert_approx_eq!(degrees(result), 25.0, 1e-4);
            }

            // ===============================================================
            // Section 7.3 — persistent condition, already latched.
            // ===============================================================

            fn persistent_case(byte1: u8) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let (shared, mut tmp) = build(Setup {
                    level_high: asserted_high,
                    fh: true,
                    strict_pin: true,
                    steps: vec![cfg(INTERRUPT, byte1), temp(T80)],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };

                assert!(pin_calls(&shared).is_empty());
                assert_approx_eq!(degrees(result), 80.0, 1e-4);
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x01], vec![0x00]],
                    "no C1 on the fast path"
                );
                assert_eq!(steps_left(&shared), 0);
            }

            #[test]
            fn persistent_latched_interrupt_active_low_uses_entry_evidence() {
                persistent_case(ACTIVE_LOW);
            }

            #[test]
            fn persistent_latched_interrupt_active_high_uses_entry_evidence() {
                persistent_case(ACTIVE_HIGH);
            }

            // ===============================================================
            // Section 7.4 — no pending entry flag, later assertion.
            // ===============================================================

            fn no_pending_then_assertion_case(byte1: u8) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let (want_call, want_done) = if asserted_high {
                    (PinOp::WaitHigh, PinOp::WaitHigh)
                } else {
                    (PinOp::WaitLow, PinOp::WaitLow)
                };

                let (shared, mut tmp) = build(Setup {
                    // Pin starts inactive.
                    level_high: !asserted_high,
                    steps: vec![cfg(INTERRUPT, byte1), cfg(INTERRUPT, byte1), temp(T80)],
                    ..Default::default()
                });

                let mut fut = pin!(tmp.wait_for_temperature_threshold());

                // C0 runs, the level wait is armed, and the waiter parks.
                assert!(poll_once(fut.as_mut()).is_pending(), "must park until ALERT asserts");
                assert_eq!(
                    log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![INTERRUPT, byte1]),
                        Op::PinCall(want_call),
                    ],
                    "exactly one level wait, armed after C0 and before any C1"
                );

                // The alert asserts.
                {
                    let mut w = shared.lock().unwrap();
                    w.fh = true;
                    w.set_level(asserted_high);
                }

                let result = poll_once(fut.as_mut());
                assert_approx_eq!(degrees(result), 80.0, 1e-4);

                // Cross-interface ordering: C1 cannot precede pin completion.
                assert_eq!(
                    log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![INTERRUPT, byte1]),
                        Op::PinCall(want_call),
                        Op::PinDone(want_done),
                        Op::WriteRead(vec![0x01], vec![INTERRUPT | 0x10, byte1]),
                        Op::WriteRead(vec![0x00], T80.to_vec()),
                    ]
                );
                assert_eq!(steps_left(&shared), 0);
            }

            #[test]
            fn interrupt_active_low_without_entry_flag_uses_one_level_wait() {
                no_pending_then_assertion_case(ACTIVE_LOW);
            }

            #[test]
            fn interrupt_active_high_without_entry_flag_uses_one_level_wait() {
                no_pending_then_assertion_case(ACTIVE_HIGH);
            }

            /// Section 7.4's final paragraph: C1 returning clear flags after a
            /// successful level wait must not cause a re-qualification loop.
            #[test]
            fn interrupt_completes_even_when_c1_reports_clear_flags() {
                let (shared, mut tmp) = build(Setup {
                    level_high: true,
                    steps: vec![cfg(INTERRUPT, ACTIVE_LOW), cfg(INTERRUPT, ACTIVE_LOW), temp(T80)],
                    ..Default::default()
                });

                let mut fut = pin!(tmp.wait_for_temperature_threshold());
                assert!(poll_once(fut.as_mut()).is_pending());
                shared.lock().unwrap().set_level(false);

                let result = poll_once(fut.as_mut());
                assert_eq!(pin_calls(&shared), vec![PinOp::WaitLow], "exactly one level wait");
                assert_approx_eq!(degrees(result), 80.0, 1e-4);
                assert_eq!(i2c_writes(&shared), vec![vec![0x01], vec![0x01], vec![0x00]]);
            }

            // ===============================================================
            // Section 7.5 — assertion between C0 and GPIO arming.
            //
            // This rejects the "consume the flags, then wait for an edge"
            // variant, which would otherwise pass section 7.2.
            // ===============================================================

            fn assertion_before_arming_case(byte1: u8) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let assertion: fn(&mut World) = if asserted_high { assert_high } else { assert_low };
                let want = if asserted_high { PinOp::WaitHigh } else { PinOp::WaitLow };

                let (shared, mut tmp) = build(Setup {
                    level_high: !asserted_high,
                    // C0 sees no flags, but the alert asserts before the
                    // waiter gets a chance to arm the GPIO. No further
                    // transition ever occurs.
                    steps: vec![
                        cfg(INTERRUPT, byte1).effect(assertion),
                        cfg(INTERRUPT, byte1),
                        temp(T80),
                    ],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };

                assert_eq!(
                    pin_calls(&shared),
                    vec![want],
                    "an already-active level must be accepted; an edge wait would park forever"
                );
                assert_approx_eq!(degrees(result), 80.0, 1e-4);
                assert_eq!(i2c_writes(&shared), vec![vec![0x01], vec![0x01], vec![0x00]]);
                assert_eq!(steps_left(&shared), 0);
            }

            #[test]
            fn interrupt_active_low_accepts_assertion_that_beat_gpio_arming() {
                assertion_before_arming_case(ACTIVE_LOW);
            }

            #[test]
            fn interrupt_active_high_accepts_assertion_that_beat_gpio_arming() {
                assertion_before_arming_case(ACTIVE_HIGH);
            }

            // ===============================================================
            // Sections 7.6 / 7.7 — comparator mode must keep working.
            // ===============================================================

            fn comparator_case(byte1: u8, start_asserted: bool) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let want = if asserted_high { PinOp::WaitHigh } else { PinOp::WaitLow };

                let (shared, mut tmp) = build(Setup {
                    level_high: if start_asserted { asserted_high } else { !asserted_high },
                    steps: vec![cfg(COMPARATOR, byte1), temp(T80)],
                    ..Default::default()
                });

                let mut fut = pin!(tmp.wait_for_temperature_threshold());
                let mut result = poll_once(fut.as_mut());
                if !start_asserted {
                    assert!(result.is_pending(), "must park until the level becomes active");
                    shared.lock().unwrap().set_level(asserted_high);
                    result = poll_once(fut.as_mut());
                }

                assert_eq!(
                    pin_calls(&shared),
                    vec![want],
                    "exactly one level wait, no InputPin sampling"
                );
                assert_approx_eq!(degrees(result), 80.0, 1e-4);
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x01], vec![0x00]],
                    "comparator mode performs no post-wait acknowledgment"
                );
                assert_eq!(steps_left(&shared), 0);
            }

            #[test]
            fn comparator_active_low_already_low_completes_immediately() {
                comparator_case(ACTIVE_LOW, true);
            }

            #[test]
            fn comparator_active_low_waits_for_low() {
                comparator_case(ACTIVE_LOW, false);
            }

            #[test]
            fn comparator_active_high_already_high_completes_immediately() {
                comparator_case(ACTIVE_HIGH, true);
            }

            #[test]
            fn comparator_active_high_waits_for_high() {
                comparator_case(ACTIVE_HIGH, false);
            }

            /// Two consecutive comparator calls while the level stays active
            /// must each complete after their own C0 and T. Comparator mode
            /// does not latch, so nothing is consumed between calls.
            fn comparator_repeated_calls_case(byte1: u8) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let want = if asserted_high { PinOp::WaitHigh } else { PinOp::WaitLow };

                let (shared, mut tmp) = build(Setup {
                    level_high: asserted_high,
                    steps: vec![cfg(COMPARATOR, byte1), temp(T80), cfg(COMPARATOR, byte1), temp(T80)],
                    ..Default::default()
                });

                for call in 0..2 {
                    let result = {
                        let mut fut = pin!(tmp.wait_for_temperature_threshold());
                        poll_once(fut.as_mut())
                    };
                    assert_approx_eq!(degrees(result), 80.0, 1e-4);
                    assert_eq!(
                        take_log(&shared),
                        vec![
                            Op::WriteRead(vec![0x01], vec![COMPARATOR, byte1]),
                            Op::PinCall(want),
                            Op::PinDone(want),
                            Op::WriteRead(vec![0x00], T80.to_vec()),
                        ],
                        "call {call} must perform exactly C0, one level wait, and T"
                    );
                }
                assert_eq!(steps_left(&shared), 0);
            }

            #[test]
            fn comparator_repeated_calls_while_level_stays_active() {
                comparator_repeated_calls_case(ACTIVE_LOW);
            }

            #[test]
            fn comparator_active_high_repeated_calls_while_level_stays_active() {
                comparator_repeated_calls_case(ACTIVE_HIGH);
            }

            /// Section 7.7's branch-semantics test: comparator flags must not
            /// substitute for the pin predicate.
            fn comparator_flags_do_not_fast_path_case(byte1: u8) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let want = if asserted_high { PinOp::WaitHigh } else { PinOp::WaitLow };

                let (shared, mut tmp) = build(Setup {
                    // Flags set, but the modeled pin is inactive.
                    level_high: !asserted_high,
                    fl: true,
                    fh: true,
                    steps: vec![cfg(COMPARATOR, byte1), temp(T80)],
                    ..Default::default()
                });

                let mut fut = pin!(tmp.wait_for_temperature_threshold());
                assert!(
                    poll_once(fut.as_mut()).is_pending(),
                    "comparator mode must take the level wait, never the interrupt fast path"
                );
                assert_eq!(pin_calls(&shared), vec![want]);

                shared.lock().unwrap().set_level(asserted_high);
                let result = poll_once(fut.as_mut());
                assert_approx_eq!(degrees(result), 80.0, 1e-4);
                assert_eq!(i2c_writes(&shared), vec![vec![0x01], vec![0x00]]);
            }

            #[test]
            fn comparator_active_low_entry_flags_do_not_take_fast_path() {
                comparator_flags_do_not_fast_path_case(ACTIVE_LOW);
            }

            #[test]
            fn comparator_active_high_entry_flags_do_not_take_fast_path() {
                comparator_flags_do_not_fast_path_case(ACTIVE_HIGH);
            }

            // ===============================================================
            // Section 7.8 — freshness and consecutive calls.
            // ===============================================================

            #[test]
            fn waiter_uses_settings_from_its_own_entry_snapshot() {
                let (shared, mut tmp) = build(Setup {
                    // Nothing latched yet: the reconfiguration's own
                    // read-modify-write must see clear flags.
                    level_high: true,
                    strict_pin: true,
                    steps: vec![
                        // sensor_mut().configure(): read-modify-write.
                        cfg(COMPARATOR, ACTIVE_LOW),
                        write(vec![0x01, 0x26, 0x90]),
                        // C0 of the wait: the freshly programmed settings.
                        cfg(INTERRUPT, ACTIVE_HIGH),
                        temp(T25),
                    ],
                    ..Default::default()
                });

                let cfg_new = Config {
                    thermostat_mode: Thermostat::Interrupt,
                    alert_polarity: Polarity::ActiveHigh,
                    ..Default::default()
                };

                let configured = {
                    let mut fut = pin!(tmp.sensor_mut().configure(cfg_new));
                    poll_once(fut.as_mut())
                };
                assert!(matches!(configured, Poll::Ready(Ok(()))));

                // Now an ActiveHigh interrupt latches and ALERT asserts HIGH.
                {
                    let mut w = shared.lock().unwrap();
                    w.fh = true;
                    w.set_level(true);
                }

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                assert!(pin_calls(&shared).is_empty());
                assert_approx_eq!(degrees(result), 25.0, 1e-4);
                assert_eq!(steps_left(&shared), 0);
            }

            /// No trailing acknowledgment on the fast path may consume the
            /// event intended for the next call.
            #[test]
            fn two_consecutive_pending_interrupts_both_complete_without_gpio() {
                let (shared, mut tmp) = build(Setup {
                    level_high: false,
                    fh: true,
                    strict_pin: true,
                    steps: vec![cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high), temp(T80)],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                assert_approx_eq!(degrees(result), 80.0, 1e-4);
                assert_eq!(
                    take_log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![0x36, ACTIVE_LOW]),
                        Op::WriteRead(vec![0x00], T80.to_vec()),
                    ]
                );

                // A new condition latches.
                {
                    let mut w = shared.lock().unwrap();
                    w.fh = true;
                    w.set_level(false);
                }
                push_steps(
                    &shared,
                    vec![cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high), temp(T80)],
                );

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                assert_approx_eq!(degrees(result), 80.0, 1e-4);
                assert_eq!(
                    take_log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![0x36, ACTIVE_LOW]),
                        Op::WriteRead(vec![0x00], T80.to_vec()),
                    ]
                );
                assert_eq!(steps_left(&shared), 0);
            }

            // ===============================================================
            // Section 7.10 — error injection.
            // ===============================================================

            #[test]
            fn c0_bus_error_stops_before_gpio_and_temperature() {
                let (shared, mut tmp) = build(Setup {
                    level_high: true,
                    steps: vec![cfg(INTERRUPT, ACTIVE_LOW).err(I2cError::Bus)],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                expect_bus(&shared, result, I2cError::Bus);
                assert!(pin_calls(&shared).is_empty());
                assert_eq!(i2c_writes(&shared), vec![vec![0x01]]);
            }

            fn level_wait_pin_error_case(byte1: u8) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let want = if asserted_high { PinOp::WaitHigh } else { PinOp::WaitLow };

                let (shared, mut tmp) = build(Setup {
                    level_high: !asserted_high,
                    pin_error: Some(PinError::Other),
                    steps: vec![cfg(INTERRUPT, byte1)],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };

                assert_eq!(pin_calls(&shared), vec![want], "the waiter must use a level wait");
                expect_pin(&shared, result, PinError::Other);
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x01]],
                    "a pin error must not trigger a cleanup acknowledgment or a temperature read"
                );
            }

            #[test]
            fn level_wait_pin_error_active_low_is_error_pin() {
                level_wait_pin_error_case(ACTIVE_LOW);
            }

            #[test]
            fn level_wait_pin_error_active_high_is_error_pin() {
                level_wait_pin_error_case(ACTIVE_HIGH);
            }

            #[test]
            fn c1_bus_error_prevents_the_temperature_read() {
                let (shared, mut tmp) = build(Setup {
                    level_high: false,
                    steps: vec![
                        cfg(INTERRUPT, ACTIVE_LOW),
                        cfg(INTERRUPT, ACTIVE_LOW).err(I2cError::Bus),
                    ],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };

                assert_eq!(pin_calls(&shared), vec![PinOp::WaitLow]);
                expect_bus(&shared, result, I2cError::Bus);
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x01], vec![0x01]],
                    "no T after a failed C1"
                );
            }

            /// A failed T on the interrupt fast path is reported as
            /// [`Error::Bus`] — but the acknowledged event is *retained*,
            /// because C0 has already destroyed the only copy the chip had
            /// (issue #58, gap 2). The retry therefore re-reads the
            /// temperature and nothing else.
            #[test]
            fn temperature_error_on_the_pending_fast_path_retains_the_event() {
                let (shared, mut tmp) = build(Setup {
                    level_high: false,
                    fh: true,
                    strict_pin: true,
                    steps: vec![
                        cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high),
                        temp(T25).err(I2cError::Other),
                    ],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                expect_bus(&shared, result, I2cError::Other);
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x01], vec![0x00]],
                    "a failed T must not retry or read configuration again"
                );

                // The event was acknowledged but never delivered, so it is
                // retained: the next call must deliver it with T alone. No
                // C0 (the chip has nothing left to report) and no GPIO
                // (`strict_pin` stays armed to prove it).
                push_steps(&shared, vec![temp(T80)]);
                take_log(&shared);
                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                assert_approx_eq!(degrees(result), 80.0, 1e-4);
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x00]],
                    "the retained sample is delivered by T alone — no second C0"
                );
                assert!(pin_calls(&shared).is_empty());
                assert_eq!(steps_left(&shared), 0);
            }

            #[test]
            fn temperature_error_on_the_post_wait_path_is_bus_error() {
                let (shared, mut tmp) = build(Setup {
                    level_high: false,
                    steps: vec![
                        cfg(INTERRUPT, ACTIVE_LOW),
                        cfg(INTERRUPT, ACTIVE_LOW),
                        temp(T80).err(I2cError::Other),
                    ],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                assert_eq!(pin_calls(&shared), vec![PinOp::WaitLow]);
                expect_bus(&shared, result, I2cError::Other);
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x01], vec![0x01], vec![0x00]],
                    "the slow path performs exactly C0, C1 and T before failing"
                );
                assert_eq!(steps_left(&shared), 0);
            }

            #[test]
            fn temperature_error_on_the_comparator_path_is_bus_error() {
                let (shared, mut tmp) = build(Setup {
                    level_high: false,
                    steps: vec![cfg(COMPARATOR, ACTIVE_LOW), temp(T80).err(I2cError::Other)],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                assert_eq!(pin_calls(&shared), vec![PinOp::WaitLow]);
                expect_bus(&shared, result, I2cError::Other);
                assert_eq!(steps_left(&shared), 0);

                // Comparator mode acknowledges nothing, so there is nothing
                // to retain: the retry is a *fresh* observation — C0, one
                // level wait, then T.
                push_steps(&shared, vec![cfg(COMPARATOR, ACTIVE_LOW), temp(T25)]);
                take_log(&shared);
                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                assert_approx_eq!(degrees(result), 25.0, 1e-4);
                assert_eq!(
                    take_log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![COMPARATOR, ACTIVE_LOW]),
                        Op::PinCall(PinOp::WaitLow),
                        Op::PinDone(PinOp::WaitLow),
                        Op::WriteRead(vec![0x00], T25.to_vec()),
                    ],
                    "a comparator retry must re-observe the pin, never replay a retained sample"
                );
                assert_eq!(steps_left(&shared), 0);
            }

            // ===============================================================
            // R1 — C0's side effect happens before the transaction completes.
            // ===============================================================

            /// C0 acknowledges the chip and *then* fails. The event is gone;
            /// no GPIO, T, or cleanup may run, and the retry finds clear
            /// flags and must wait.
            #[test]
            fn c0_failure_after_acknowledgment_loses_the_event_and_waits_on_retry() {
                let (shared, mut tmp) = build(Setup {
                    level_high: false,
                    fh: true,
                    steps: vec![cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high).err(I2cError::Bus)],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                expect_bus(&shared, result, I2cError::Bus);
                assert!(pin_calls(&shared).is_empty(), "a failed C0 must not touch GPIO");
                assert_eq!(i2c_writes(&shared), vec![vec![0x01]], "no T and no cleanup read");

                // Retry: flags are clear, the pin is released, so the level
                // wait must park.
                push_steps(&shared, vec![cfg(INTERRUPT, ACTIVE_LOW)]);
                let mut fut = pin!(tmp.wait_for_temperature_threshold());
                assert!(poll_once(fut.as_mut()).is_pending());
                assert_eq!(pin_calls(&shared), vec![PinOp::WaitLow]);
            }

            /// C0 fails *before* acknowledging. The flag survives, so the
            /// retry captures it on the fast path.
            #[test]
            fn c0_failure_before_acknowledgment_preserves_the_flag_for_the_retry() {
                let (shared, mut tmp) = build(Setup {
                    level_high: false,
                    fh: true,
                    steps: vec![cfg(INTERRUPT, ACTIVE_LOW).err(I2cError::Bus)],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                expect_bus(&shared, result, I2cError::Bus);
                assert!(pin_calls(&shared).is_empty());

                push_steps(
                    &shared,
                    vec![cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high), temp(T80)],
                );
                shared.lock().unwrap().strict_pin = true;

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                assert_approx_eq!(degrees(result), 80.0, 1e-4);
                assert_eq!(steps_left(&shared), 0);
            }

            /// C0 acknowledges and then never completes. Dropping the waiter
            /// must not schedule any cleanup.
            #[test]
            fn dropping_a_waiter_stuck_in_c0_performs_no_cleanup() {
                let (shared, mut tmp) = build(Setup {
                    level_high: false,
                    fh: true,
                    steps: vec![cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high).pending()],
                    ..Default::default()
                });

                {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    assert!(poll_once(fut.as_mut()).is_pending());
                }

                assert_eq!(i2c_writes(&shared), vec![vec![0x01]]);
                assert!(pin_calls(&shared).is_empty());
                assert_eq!(steps_left(&shared), 0);
            }

            // ===============================================================
            // R2 — drop the GPIO wait after notification but before the task
            // resumes.
            // ===============================================================

            fn dropped_after_notification_case(byte1: u8) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let want = if asserted_high { PinOp::WaitHigh } else { PinOp::WaitLow };

                let (shared, mut tmp) = build(Setup {
                    level_high: !asserted_high,
                    steps: vec![cfg(INTERRUPT, byte1)],
                    ..Default::default()
                });

                {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    assert!(poll_once(fut.as_mut()).is_pending());
                    assert_eq!(pin_calls(&shared), vec![want]);

                    // The chip latches and ALERT asserts; the task is woken
                    // but dropped before it resumes.
                    let mut w = shared.lock().unwrap();
                    w.fh = true;
                    w.set_level(asserted_high);
                }

                assert_eq!(i2c_writes(&shared), vec![vec![0x01]], "no C1 and no T after the drop");

                // The next call captures the surviving flag on the fast path.
                push_steps(&shared, vec![cfg(INTERRUPT, byte1), temp(T80)]);
                shared.lock().unwrap().strict_pin = true;
                take_log(&shared);

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                assert_approx_eq!(degrees(result), 80.0, 1e-4);
                assert_eq!(steps_left(&shared), 0);
            }

            #[test]
            fn dropping_the_gpio_wait_after_notification_active_low_keeps_the_event() {
                dropped_after_notification_case(ACTIVE_LOW);
            }

            #[test]
            fn dropping_the_gpio_wait_after_notification_active_high_keeps_the_event() {
                dropped_after_notification_case(ACTIVE_HIGH);
            }

            // ===============================================================
            // R3 — C1 cancellation / error with both acknowledgment outcomes.
            // ===============================================================

            /// C1 fails *after* its read has already acknowledged the chip.
            ///
            /// The entry snapshot must be empty, otherwise the waiter takes
            /// the pending fast path and never reaches C1 at all (design
            /// §2.4 step 3, invariants §2.9.1/§2.9.3). The latch therefore
            /// appears *during* the level wait, so the acknowledgment C1
            /// performs is a real one — and the error that follows it
            /// destroys the only evidence of that event.
            #[test]
            fn c1_error_after_acknowledgment_leaves_nothing_to_replay() {
                let (shared, mut tmp) = build(Setup {
                    // ALERT inactive and nothing latched: the waiter must
                    // take the level-wait path.
                    level_high: true,
                    steps: vec![
                        cfg(INTERRUPT, ACTIVE_LOW),
                        // `.effect` runs before the outcome: this read
                        // acknowledges the chip and *then* the bus fails.
                        cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high).err(I2cError::Bus),
                    ],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    assert!(poll_once(fut.as_mut()).is_pending(), "must park until ALERT asserts");
                    assert_eq!(pin_calls(&shared), vec![PinOp::WaitLow]);

                    // The event the caller is waiting for now latches.
                    {
                        let mut w = shared.lock().unwrap();
                        w.fh = true;
                        w.set_level(false);
                    }

                    // The level wait completes, C1 acknowledges the chip,
                    // and only then does the bus fail.
                    poll_once(fut.as_mut())
                };

                expect_bus(&shared, result, I2cError::Bus);
                assert!(
                    !shared.lock().unwrap().fh,
                    "C1 must really have acknowledged the chip before failing — otherwise this \
                     test is not exercising post-acknowledgment failure at all"
                );
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x01], vec![0x01]],
                    "a failed C1 performs no T and no cleanup read"
                );

                // The acknowledged event is unrecoverable: the retry finds
                // clear flags and a released pin, and must wait for a new
                // event rather than replaying the lost one.
                push_steps(&shared, vec![cfg(INTERRUPT, ACTIVE_LOW)]);
                let mut retry = pin!(tmp.wait_for_temperature_threshold());
                assert!(poll_once(retry.as_mut()).is_pending(), "nothing to replay");
                assert_eq!(pin_calls(&shared), vec![PinOp::WaitLow, PinOp::WaitLow]);
                assert_eq!(steps_left(&shared), 0);
            }

            /// C1 fails *without* the chip having acknowledged.
            ///
            /// The latch therefore survives in the hardware, and the retry
            /// must be able to fast-path on the **original** event. The test
            /// never re-injects the flag after the failure: the FH the retry
            /// sees is the very one latched during the level wait. That is
            /// what distinguishes this from
            /// [`c1_error_after_acknowledgment_leaves_nothing_to_replay`].
            #[test]
            fn c1_error_before_acknowledgment_preserves_the_flag_for_the_retry() {
                let (shared, mut tmp) = build(Setup {
                    // ALERT inactive and nothing latched: the waiter must
                    // take the level-wait path.
                    level_high: true,
                    steps: vec![
                        cfg(INTERRUPT, ACTIVE_LOW),
                        // No `.effect`: the bus fails before this read can
                        // acknowledge anything.
                        cfg(INTERRUPT, ACTIVE_LOW).err(I2cError::Bus),
                    ],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    assert!(poll_once(fut.as_mut()).is_pending(), "must park until ALERT asserts");
                    assert_eq!(pin_calls(&shared), vec![PinOp::WaitLow]);

                    // The event the caller is waiting for latches now, so it
                    // is genuinely pending when C1 fails.
                    {
                        let mut w = shared.lock().unwrap();
                        w.fh = true;
                        w.set_level(false);
                    }

                    poll_once(fut.as_mut())
                };

                expect_bus(&shared, result, I2cError::Bus);
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x01], vec![0x01]],
                    "a failed C1 performs no T and no cleanup read"
                );

                // The decisive assertion: the chip never acknowledged, so the
                // latch is still there to be recovered.
                assert!(
                    shared.lock().unwrap().fh,
                    "C1 failed before acknowledging, so FH must still be latched — without this \
                     the retry below would be replaying an event the test injected rather than \
                     the original one"
                );

                // Retry. Note there is no `w.fh = true` here: the flag the
                // retry captures is the original event. `strict_pin` makes
                // any GPIO touch a failure, so this can only pass via the
                // pending fast path.
                push_steps(
                    &shared,
                    vec![cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high), temp(T80)],
                );
                shared.lock().unwrap().strict_pin = true;
                take_log(&shared);

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                assert_approx_eq!(degrees(result), 80.0, 1e-4);
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x01], vec![0x00]],
                    "the preserved event is delivered by C0 + T alone"
                );
                assert_eq!(steps_left(&shared), 0);
            }

            #[test]
            fn dropping_a_waiter_stuck_in_c1_performs_no_cleanup() {
                let (shared, mut tmp) = build(Setup {
                    level_high: false,
                    steps: vec![cfg(INTERRUPT, ACTIVE_LOW), cfg(INTERRUPT, ACTIVE_LOW).pending()],
                    ..Default::default()
                });

                {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    assert!(poll_once(fut.as_mut()).is_pending());
                }

                assert_eq!(pin_calls(&shared), vec![PinOp::WaitLow]);
                assert_eq!(i2c_writes(&shared), vec![vec![0x01], vec![0x01]], "no T and no cleanup");
                assert_eq!(steps_left(&shared), 0);
            }

            // ===============================================================
            // R4 — the observed level returns inactive before resumption.
            // ===============================================================

            fn level_goes_inactive_before_resumption_case(byte1: u8) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let want = if asserted_high { PinOp::WaitHigh } else { PinOp::WaitLow };

                let (shared, mut tmp) = build(Setup {
                    level_high: !asserted_high,
                    steps: vec![cfg(INTERRUPT, byte1), cfg(INTERRUPT, byte1), temp(T80)],
                    ..Default::default()
                });

                let mut fut = pin!(tmp.wait_for_temperature_threshold());
                assert!(poll_once(fut.as_mut()).is_pending());
                assert_eq!(pin_calls(&shared), vec![want]);

                // The level went active (completing the wait) and back again
                // before the woken task ran.
                shared.lock().unwrap().pulse_level(asserted_high);

                let result = poll_once(fut.as_mut());
                assert_approx_eq!(degrees(result), 80.0, 1e-4);
                assert_eq!(
                    pin_calls(&shared),
                    vec![want],
                    "the driver must not resample InputPin after a completed level wait"
                );
                assert_eq!(i2c_writes(&shared), vec![vec![0x01], vec![0x01], vec![0x00]]);
                assert_eq!(steps_left(&shared), 0);
            }

            #[test]
            fn active_low_level_going_inactive_before_resumption_still_completes() {
                level_goes_inactive_before_resumption_case(ACTIVE_LOW);
            }

            #[test]
            fn active_high_level_going_inactive_before_resumption_still_completes() {
                level_goes_inactive_before_resumption_case(ACTIVE_HIGH);
            }

            // ===============================================================
            // R5 — a new event survives a T failure or cancellation.
            // ===============================================================

            /// Event A is acknowledged by C0, then T fails. Its delivery
            /// obligation is retained; the next call reads a new temperature
            /// to settle it. Event B, latched meanwhile, is collected
            /// by the call *after* that, on a fresh C0.
            #[test]
            fn new_event_survives_a_temperature_failure_behind_the_retained_sample() {
                let (shared, mut tmp) = build(Setup {
                    level_high: false,
                    fh: true,
                    strict_pin: true,
                    steps: vec![
                        // C0 captures and clears event A ...
                        cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high),
                        // ... and T fails.
                        temp(T80).err(I2cError::Other),
                    ],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                expect_bus(&shared, result, I2cError::Other);

                // Event B latches afterwards.
                {
                    let mut w = shared.lock().unwrap();
                    w.fl = true;
                    w.set_level(false);
                }
                push_steps(
                    &shared,
                    vec![
                        // The retained delivery: T alone.
                        temp(T80),
                        // Only then is event B collected, on a fresh C0.
                        cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high),
                        temp(T25),
                    ],
                );
                take_log(&shared);

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                assert_approx_eq!(degrees(result), 80.0, 1e-4);
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x00]],
                    "event A's retained sample is delivered first, by T alone"
                );
                take_log(&shared);

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                assert_approx_eq!(degrees(result), 25.0, 1e-4);
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x01], vec![0x00]],
                    "event B is captured by the next C0 on the fast path"
                );
                assert_eq!(steps_left(&shared), 0);
            }

            /// As above, with the temperature read cancelled rather than
            /// failed. Cancellation after C0 has acknowledged is the same
            /// loss, so it retains the same way.
            #[test]
            fn new_event_survives_cancellation_behind_the_retained_sample() {
                let (shared, mut tmp) = build(Setup {
                    level_high: false,
                    fh: true,
                    strict_pin: true,
                    steps: vec![cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high), temp(T80).pending()],
                    ..Default::default()
                });

                {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    assert!(poll_once(fut.as_mut()).is_pending());

                    // Event B latches while T is in flight.
                    let mut w = shared.lock().unwrap();
                    w.fh = true;
                    w.set_level(false);
                }

                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x01], vec![0x00]],
                    "dropping the waiter must not perform a cleanup configuration read"
                );

                push_steps(
                    &shared,
                    vec![
                        temp(T80),
                        cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high),
                        temp(T25),
                    ],
                );
                take_log(&shared);

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                assert_approx_eq!(degrees(result), 80.0, 1e-4);
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x00]],
                    "the cancelled event is retained and delivered by T alone"
                );
                take_log(&shared);

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                assert_approx_eq!(degrees(result), 25.0, 1e-4);
                assert_eq!(i2c_writes(&shared), vec![vec![0x01], vec![0x00]]);
                assert_eq!(steps_left(&shared), 0);
            }

            // ===============================================================
            // Issue #58, gap 2 — an acknowledged-but-undelivered event is
            // retained across a failed or cancelled temperature read.
            //
            // Every test below turns on `strict_pin` for the retry, so a
            // retained delivery that touches GPIO panics with a named
            // diagnostic instead of parking.
            // ===============================================================

            /// Drive the waiter through the interrupt **fast path** with a
            /// failing T, leaving exactly one acknowledged-but-undelivered
            /// event retained. `strict_pin` is left armed.
            fn arm_retained_fast_path(byte1: u8) -> (Shared, AlertTmp108<FakeI2c, FakePin>) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let release: fn(&mut World) = if asserted_high {
                    ack_release_low
                } else {
                    ack_release_high
                };

                let (shared, mut tmp) = build(Setup {
                    level_high: asserted_high,
                    fh: true,
                    strict_pin: true,
                    steps: vec![cfg(INTERRUPT, byte1).effect(release), temp(T80).err(I2cError::Other)],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                expect_bus(&shared, result, I2cError::Other);
                assert_eq!(i2c_writes(&shared), vec![vec![0x01], vec![0x00]]);
                assert_eq!(steps_left(&shared), 0);
                take_log(&shared);
                (shared, tmp)
            }

            /// As above via the **slow path**: C0 with no flags, one level
            /// wait, then C1. `latch` selects whether C1 reports nonzero
            /// flags; per the design the retention decision must not depend
            /// on it.
            fn arm_retained_slow_path(byte1: u8, latch: bool) -> (Shared, AlertTmp108<FakeI2c, FakePin>) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let want = if asserted_high { PinOp::WaitHigh } else { PinOp::WaitLow };
                let release: fn(&mut World) = if asserted_high {
                    ack_release_low
                } else {
                    ack_release_high
                };

                let (shared, mut tmp) = build(Setup {
                    level_high: !asserted_high,
                    steps: vec![
                        cfg(INTERRUPT, byte1),
                        cfg(INTERRUPT, byte1).effect(release),
                        temp(T80).err(I2cError::Other),
                    ],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    assert!(poll_once(fut.as_mut()).is_pending(), "must park until ALERT asserts");
                    assert_eq!(pin_calls(&shared), vec![want]);
                    {
                        let mut w = shared.lock().unwrap();
                        w.fh = latch;
                        w.set_level(asserted_high);
                    }
                    poll_once(fut.as_mut())
                };
                expect_bus(&shared, result, I2cError::Other);

                let c1_flags = if latch { 0x10 } else { 0x00 };
                assert_eq!(
                    take_log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![INTERRUPT, byte1]),
                        Op::PinCall(want),
                        Op::PinDone(want),
                        Op::WriteRead(vec![0x01], vec![INTERRUPT | c1_flags, byte1]),
                        Op::WriteRead(vec![0x00], T80.to_vec()),
                    ],
                    "the slow path must be C0, one level wait, C1 (flags {c1_flags:#04x}), T"
                );
                assert_eq!(steps_left(&shared), 0);
                shared.lock().unwrap().strict_pin = true;
                (shared, tmp)
            }

            /// Assert that one call delivers `want` °C using T and nothing
            /// else: no configuration read, no GPIO.
            fn expect_retained_delivery(
                shared: &Shared,
                tmp: &mut AlertTmp108<FakeI2c, FakePin>,
                bytes: [u8; 2],
                want: f32,
            ) {
                push_steps(shared, vec![temp(bytes)]);
                take_log(shared);
                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };
                assert_approx_eq!(degrees(result), want, 1e-4);
                assert_eq!(
                    i2c_writes(shared),
                    vec![vec![0x00]],
                    "a retained event is delivered by T alone — no C0, no C1"
                );
                assert!(
                    pin_calls(shared).is_empty(),
                    "a retained event must not touch GPIO, saw {:?}",
                    pin_calls(shared)
                );
                assert_eq!(steps_left(shared), 0);
            }

            fn retained_fast_path_case(byte1: u8) {
                let (shared, mut tmp) = arm_retained_fast_path(byte1);
                expect_retained_delivery(&shared, &mut tmp, T80, 80.0);
            }

            #[test]
            fn retained_fast_path_active_low_retry_reads_temperature_only() {
                retained_fast_path_case(ACTIVE_LOW);
            }

            #[test]
            fn retained_fast_path_active_high_retry_reads_temperature_only() {
                retained_fast_path_case(ACTIVE_HIGH);
            }

            fn retained_slow_path_case(byte1: u8, latch: bool) {
                let (shared, mut tmp) = arm_retained_slow_path(byte1, latch);
                expect_retained_delivery(&shared, &mut tmp, T25, 25.0);
            }

            #[test]
            fn retained_slow_path_active_low_retry_reads_temperature_only() {
                retained_slow_path_case(ACTIVE_LOW, true);
            }

            #[test]
            fn retained_slow_path_active_high_retry_reads_temperature_only() {
                retained_slow_path_case(ACTIVE_HIGH, true);
            }

            /// Design rule: C1's flags are discarded, *including when they
            /// are zero*. A zero-flag C1 still qualifies the event, so it
            /// must still arm retention.
            #[test]
            fn retained_slow_path_with_zero_flag_c1_active_low_still_retains() {
                retained_slow_path_case(ACTIVE_LOW, false);
            }

            #[test]
            fn retained_slow_path_with_zero_flag_c1_active_high_still_retains() {
                retained_slow_path_case(ACTIVE_HIGH, false);
            }

            /// Retention is not consumed by a failed delivery attempt: it
            /// survives an unbounded number of them and is consumed only by
            /// the one that succeeds.
            #[test]
            fn repeated_temperature_failures_keep_the_event_retained() {
                let (shared, mut tmp) = arm_retained_fast_path(ACTIVE_LOW);

                for attempt in 0..3 {
                    push_steps(&shared, vec![temp(T80).err(I2cError::Other)]);
                    take_log(&shared);
                    let result = {
                        let mut fut = pin!(tmp.wait_for_temperature_threshold());
                        poll_once(fut.as_mut())
                    };
                    expect_bus(&shared, result, I2cError::Other);
                    assert_eq!(
                        i2c_writes(&shared),
                        vec![vec![0x00]],
                        "failed retry {attempt} must retry T alone and stay retained"
                    );
                }

                expect_retained_delivery(&shared, &mut tmp, T80, 80.0);
            }

            /// Cancelling the delivery attempt retains the event just as a
            /// failure does — the acknowledgment already happened.
            #[test]
            fn cancelling_the_temperature_read_keeps_the_event_retained() {
                let (shared, mut tmp) = build(Setup {
                    level_high: false,
                    fh: true,
                    strict_pin: true,
                    steps: vec![cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high), temp(T80).pending()],
                    ..Default::default()
                });

                {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    assert!(poll_once(fut.as_mut()).is_pending());
                }
                assert_eq!(i2c_writes(&shared), vec![vec![0x01], vec![0x00]]);
                assert_eq!(steps_left(&shared), 0);

                expect_retained_delivery(&shared, &mut tmp, T25, 25.0);
            }

            /// A successful delivery consumes the retention exactly once:
            /// the following call is a fresh full observation.
            #[test]
            fn a_successful_retained_delivery_is_consumed_exactly_once() {
                let (shared, mut tmp) = arm_retained_fast_path(ACTIVE_LOW);
                expect_retained_delivery(&shared, &mut tmp, T80, 80.0);

                // Nothing is latched and the pin is released, so a fresh
                // observation must read C0 and then park on the level wait.
                push_steps(&shared, vec![cfg(INTERRUPT, ACTIVE_LOW)]);
                shared.lock().unwrap().strict_pin = false;
                take_log(&shared);

                let mut fut = pin!(tmp.wait_for_temperature_threshold());
                assert!(
                    poll_once(fut.as_mut()).is_pending(),
                    "retention is single-use: the next call must observe the chip afresh"
                );
                assert_eq!(i2c_writes(&shared), vec![vec![0x01]], "a fresh C0 is mandatory");
                assert_eq!(pin_calls(&shared), vec![PinOp::WaitLow]);
            }

            /// Retained delivery takes precedence over the *current* mode.
            /// The delivery obligation comes from an already-acknowledged
            /// interrupt, not a cached sample; reconfiguring to comparator
            /// mode afterwards does not retroactively cancel the obligation.
            #[test]
            fn reconfiguring_to_comparator_does_not_discard_a_retained_event() {
                let (shared, mut tmp) = arm_retained_fast_path(ACTIVE_LOW);

                push_steps(
                    &shared,
                    vec![cfg(INTERRUPT, ACTIVE_LOW), write(vec![0x01, COMPARATOR, ACTIVE_LOW])],
                );
                let configured = {
                    let mut fut = pin!(tmp.sensor_mut().configure(Config {
                        thermostat_mode: Thermostat::Comparator,
                        alert_polarity: Polarity::ActiveLow,
                        ..Default::default()
                    }));
                    poll_once(fut.as_mut())
                };
                assert!(matches!(configured, Poll::Ready(Ok(()))));
                assert_eq!(steps_left(&shared), 0);

                // Still T only: no comparator C0, no level wait.
                expect_retained_delivery(&shared, &mut tmp, T25, 25.0);
            }

            /// A direct temperature read through `sensor_mut()` is not a
            /// delivery, so it must not consume the retained event.
            #[test]
            fn a_direct_temperature_read_does_not_consume_a_retained_event() {
                let (shared, mut tmp) = arm_retained_fast_path(ACTIVE_LOW);

                push_steps(&shared, vec![temp(T25)]);
                let direct = {
                    let mut fut = pin!(tmp.sensor_mut().temperature());
                    poll_once(fut.as_mut())
                };
                match direct {
                    Poll::Ready(Ok(t)) => assert_approx_eq!(t.to_degrees(), 25.0, 1e-4),
                    other => panic!("expected a direct temperature, got {other:?}"),
                }
                assert_eq!(steps_left(&shared), 0);

                expect_retained_delivery(&shared, &mut tmp, T80, 80.0);
            }

            /// Cancelling a *retained retry* must not discard the debt.
            ///
            /// The distinction from
            /// [`cancelling_the_temperature_read_keeps_the_event_retained`]
            /// is which attempt is cancelled: there it is the original
            /// delivery, here it is a later replay of an already-retained
            /// event. An implementation that cleared the flag on entry to
            /// the retained branch and only restored it on T's `Err` arm
            /// would pass every other test here and silently lose the
            /// event at exactly this point.
            #[test]
            fn cancelling_a_retained_retry_keeps_the_event_retained() {
                let (shared, mut tmp) = arm_retained_fast_path(ACTIVE_LOW);

                // First replay: T parks, then the waiter is dropped.
                push_steps(&shared, vec![temp(T80).pending()]);
                take_log(&shared);
                {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    assert!(poll_once(fut.as_mut()).is_pending());
                }
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x00]],
                    "a retained retry retries T alone — no C0, no C1"
                );
                assert!(
                    pin_calls(&shared).is_empty(),
                    "a retained retry must not touch GPIO, saw {:?}",
                    pin_calls(&shared)
                );
                assert_eq!(steps_left(&shared), 0);

                // Second replay: the debt must still be there to settle.
                expect_retained_delivery(&shared, &mut tmp, T80, 80.0);
            }

            /// As above, for an event retained from the **slow path**.
            #[test]
            fn cancelling_a_retained_retry_on_the_slow_path_keeps_the_event_retained() {
                let (shared, mut tmp) = arm_retained_slow_path(ACTIVE_LOW, true);

                push_steps(&shared, vec![temp(T80).pending()]);
                take_log(&shared);
                {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    assert!(poll_once(fut.as_mut()).is_pending());
                }
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x00]],
                    "a retained retry retries T alone — no C0, no C1"
                );
                assert!(
                    pin_calls(&shared).is_empty(),
                    "a retained retry must not touch GPIO, saw {:?}",
                    pin_calls(&shared)
                );
                assert_eq!(steps_left(&shared), 0);

                expect_retained_delivery(&shared, &mut tmp, T25, 25.0);
            }

            /// The slow-path analogue of
            /// [`cancelling_the_temperature_read_keeps_the_event_retained`]:
            /// the *first* T is cancelled rather than failed, after the
            /// event was acknowledged by C1 rather than by C0.
            #[test]
            fn cancelling_the_temperature_read_on_the_slow_path_keeps_the_event_retained() {
                let (shared, mut tmp) = build(Setup {
                    // ALERT inactive and nothing latched: the waiter must
                    // take the level-wait path.
                    level_high: true,
                    steps: vec![
                        cfg(INTERRUPT, ACTIVE_LOW),
                        cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high),
                        temp(T80).pending(),
                    ],
                    ..Default::default()
                });

                {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    assert!(poll_once(fut.as_mut()).is_pending(), "must park until ALERT asserts");
                    assert_eq!(pin_calls(&shared), vec![PinOp::WaitLow]);

                    {
                        let mut w = shared.lock().unwrap();
                        w.fh = true;
                        w.set_level(false);
                    }

                    // C1 acknowledges, T parks — and the waiter is dropped.
                    assert!(poll_once(fut.as_mut()).is_pending());
                }
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x01], vec![0x01], vec![0x00]],
                    "C0, C1 and the cancelled T — and no cleanup read"
                );
                assert_eq!(steps_left(&shared), 0);

                shared.lock().unwrap().strict_pin = true;
                expect_retained_delivery(&shared, &mut tmp, T25, 25.0);
            }

            // ===============================================================
            // Issue #67 — observable alert cause.
            //
            // Design section 8.2, "Fast cause matrix", "Slow cause matrix"
            // and "Zero-C1 qualification".
            // ===============================================================

            type AlertPoll = Poll<Result<AlertEvent, Error<I2cError, PinError>>>;

            fn event(p: AlertPoll) -> AlertEvent {
                match p {
                    Poll::Ready(Ok(e)) => e,
                    Poll::Ready(Err(e)) => panic!("expected an alert event, got {e:?}"),
                    Poll::Pending => panic!(
                        "waiter parked instead of completing — it is waiting for a notification \
                         this fixture will never deliver"
                    ),
                }
            }

            /// 25 °C as the driver's own representation, so the assertion
            /// is exact rather than approximate.
            fn celsius(sixteenths: i16) -> Celsius {
                Celsius::from_sixteenths(sixteenths).expect("in range by construction")
            }

            /// The asserted-level wait a given polarity byte selects.
            fn asserted_wait(byte1: u8) -> PinOp {
                if byte1 == ACTIVE_HIGH {
                    PinOp::WaitHigh
                } else {
                    PinOp::WaitLow
                }
            }

            // ---------------------------------------------------------------
            // Fast cause matrix: nonzero entry flags, C0 and T only.
            // ---------------------------------------------------------------

            fn fast_cause_case(byte1: u8, fl: bool, fh: bool, want: AlertCause) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let release: fn(&mut World) = if asserted_high {
                    ack_release_low
                } else {
                    ack_release_high
                };

                let (shared, mut tmp) = build(Setup {
                    level_high: asserted_high,
                    fl,
                    fh,
                    // Any GPIO touch is a bug on the fast path (issue #59).
                    strict_pin: true,
                    // T25 is deliberately *inside* any plausible band: an
                    // in-band sample must not alter the reported cause.
                    steps: vec![cfg(INTERRUPT, byte1).effect(release), temp(T25)],
                    ..Default::default()
                });

                let result = {
                    let mut fut = pin!(tmp.wait_for_alert());
                    poll_once(fut.as_mut())
                };
                let ev = event(result);

                assert_eq!(ev.cause, want, "C0 flags ({fl}, {fh}) must map to {want:?}");
                assert_eq!(ev.temperature, celsius(400));
                assert!(
                    pin_calls(&shared).is_empty(),
                    "a pending entry flag must suppress every GPIO operation, saw {:?}",
                    pin_calls(&shared)
                );
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x01], vec![0x00]],
                    "the fast path performs exactly C0 and T (no C1)"
                );
                assert_eq!(steps_left(&shared), 0);
            }

            #[test]
            fn fast_path_fl_active_low_reports_below_low() {
                fast_cause_case(ACTIVE_LOW, true, false, AlertCause::BelowLow);
            }

            #[test]
            fn fast_path_fh_active_low_reports_above_high() {
                fast_cause_case(ACTIVE_LOW, false, true, AlertCause::AboveHigh);
            }

            #[test]
            fn fast_path_both_flags_active_low_reports_both() {
                fast_cause_case(ACTIVE_LOW, true, true, AlertCause::Both);
            }

            #[test]
            fn fast_path_fl_active_high_reports_below_low() {
                fast_cause_case(ACTIVE_HIGH, true, false, AlertCause::BelowLow);
            }

            #[test]
            fn fast_path_fh_active_high_reports_above_high() {
                fast_cause_case(ACTIVE_HIGH, false, true, AlertCause::AboveHigh);
            }

            #[test]
            fn fast_path_both_flags_active_high_reports_both() {
                fast_cause_case(ACTIVE_HIGH, true, true, AlertCause::Both);
            }

            // ---------------------------------------------------------------
            // Slow cause matrix: clear C0, one level wait, then C1.
            // ---------------------------------------------------------------

            /// Clear entry snapshot, one asserted-level wait, one C1
            /// carrying `(fl, fh)`, then T. All four C1 flag pairs are
            /// legal, including zero: C1's flags inform the cause but
            /// never decide whether an event exists.
            fn slow_cause_case(byte1: u8, fl: bool, fh: bool, want: AlertCause) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let release: fn(&mut World) = if asserted_high {
                    ack_release_low
                } else {
                    ack_release_high
                };

                let (shared, mut tmp) = build(Setup {
                    // ALERT inactive and nothing latched: the waiter must
                    // take the level-wait path.
                    level_high: !asserted_high,
                    steps: vec![cfg(INTERRUPT, byte1), cfg(INTERRUPT, byte1).effect(release), temp(T25)],
                    ..Default::default()
                });

                let mut fut = pin!(tmp.wait_for_alert());

                assert!(poll_once(fut.as_mut()).is_pending(), "must park until ALERT asserts");
                assert_eq!(
                    log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![INTERRUPT, byte1]),
                        Op::PinCall(asserted_wait(byte1)),
                    ],
                    "exactly one level wait, armed after C0 and before any C1"
                );

                // The alert asserts, latching whatever C1 will report.
                {
                    let mut w = shared.lock().unwrap();
                    w.fl = fl;
                    w.fh = fh;
                    w.set_level(asserted_high);
                }

                let ev = event(poll_once(fut.as_mut()));

                assert_eq!(ev.cause, want, "C1 flags ({fl}, {fh}) must map to {want:?}");
                assert_eq!(ev.temperature, celsius(400));
                assert_eq!(
                    pin_calls(&shared),
                    vec![asserted_wait(byte1)],
                    "exactly one level wait, and never a second one"
                );
                assert_eq!(
                    i2c_writes(&shared),
                    vec![vec![0x01], vec![0x01], vec![0x00]],
                    "the slow path performs C0, C1 and T"
                );
                assert_eq!(steps_left(&shared), 0);
            }

            #[test]
            fn slow_path_zero_c1_active_low_reports_unknown() {
                slow_cause_case(ACTIVE_LOW, false, false, AlertCause::Unknown);
            }

            #[test]
            fn slow_path_fl_c1_active_low_reports_below_low() {
                slow_cause_case(ACTIVE_LOW, true, false, AlertCause::BelowLow);
            }

            #[test]
            fn slow_path_fh_c1_active_low_reports_above_high() {
                slow_cause_case(ACTIVE_LOW, false, true, AlertCause::AboveHigh);
            }

            #[test]
            fn slow_path_both_c1_active_low_reports_both() {
                slow_cause_case(ACTIVE_LOW, true, true, AlertCause::Both);
            }

            #[test]
            fn slow_path_zero_c1_active_high_reports_unknown() {
                slow_cause_case(ACTIVE_HIGH, false, false, AlertCause::Unknown);
            }

            #[test]
            fn slow_path_fl_c1_active_high_reports_below_low() {
                slow_cause_case(ACTIVE_HIGH, true, false, AlertCause::BelowLow);
            }

            #[test]
            fn slow_path_fh_c1_active_high_reports_above_high() {
                slow_cause_case(ACTIVE_HIGH, false, true, AlertCause::AboveHigh);
            }

            #[test]
            fn slow_path_both_c1_active_high_reports_both() {
                slow_cause_case(ACTIVE_HIGH, true, true, AlertCause::Both);
            }

            // ---------------------------------------------------------------
            // Zero-C1 qualification, stated on its own.
            // ---------------------------------------------------------------

            /// A successful level wait followed by a **successful** C1 is
            /// the qualification gate. C1 returning FL = FH = 0 does not
            /// invalidate it: the waiter must return `Ready` with
            /// `AlertCause::Unknown`, not park for another assertion and
            /// not sample `InputPin`. Requiring nonzero C1 flags would
            /// reintroduce the lost-event loop of issue #59.
            #[test]
            fn a_zero_flag_c1_still_qualifies_and_never_re_waits() {
                let (shared, mut tmp) = build(Setup {
                    level_high: true,
                    steps: vec![
                        cfg(INTERRUPT, ACTIVE_LOW),
                        // C1 reads the modeled chip, whose flags are still
                        // clear — nothing latched them.
                        cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high),
                        temp(T25),
                    ],
                    ..Default::default()
                });

                let mut fut = pin!(tmp.wait_for_alert());
                assert!(poll_once(fut.as_mut()).is_pending(), "must park until ALERT asserts");

                // Only the level moves; FL and FH stay clear.
                shared.lock().unwrap().set_level(false);

                let ev = event(poll_once(fut.as_mut()));
                assert_eq!(ev.cause, AlertCause::Unknown);
                assert_eq!(ev.temperature, celsius(400));
                assert_eq!(
                    pin_calls(&shared),
                    vec![PinOp::WaitLow],
                    "one level wait only — no second wait, and no InputPin sampling"
                );
                assert_eq!(i2c_writes(&shared), vec![vec![0x01], vec![0x01], vec![0x00]]);
                assert_eq!(steps_left(&shared), 0);
            }

            // ===============================================================
            // Issue #67 — design sections 8.3 and 8.4.
            //
            // Every expectation below is written out as a *literal* trace
            // derived from the protocol in design section 3, not as "API A
            // agrees with API B": both entry points now run one shared
            // implementation, so a shared regression would satisfy any
            // purely comparative assertion.
            // ===============================================================

            /// Open the gate of a [`Outcome::Gated`] transaction.
            fn release_gate(shared: &Shared) {
                shared.lock().unwrap().gate_open = true;
            }

            /// [`expect_bus`], for the inherent method's result type.
            fn expect_alert_bus(shared: &Shared, p: AlertPoll, want: I2cError) {
                match p {
                    Poll::Ready(Err(Error::Bus(e))) => assert_eq!(e, want),
                    Poll::Pending => panic!(
                        "waiter parked instead of returning Err(Bus({want:?})); pin operations so \
                         far: {:?}",
                        pin_calls(shared)
                    ),
                    other @ Poll::Ready(_) => panic!("expected Err(Bus({want:?})), got {other:?}"),
                }
            }

            /// The C0/C1 response byte 0 the modeled chip produces for a
            /// given latched flag pair.
            fn cfg_byte0(fl: bool, fh: bool) -> u8 {
                INTERRUPT | u8::from(fl) << 3 | u8::from(fh) << 4
            }

            /// Which entry point a test drives. Both are views of one
            /// consumptive stream (design section 3.6), so every arming and
            /// retry helper below is parameterised over this.
            #[derive(Clone, Copy, Debug, PartialEq, Eq)]
            enum Api {
                /// `AlertTmp108::wait_for_alert`.
                Inherent,
                /// `TemperatureThresholdWait::wait_for_temperature_threshold`.
                Trait,
            }

            /// Arm exactly one retained obligation through the interrupt
            /// **fast path**, via `api`, by failing T.
            ///
            /// Leaves `strict_pin` armed and the log empty.
            fn arm_fast_via(byte1: u8, fl: bool, fh: bool, api: Api) -> (Shared, AlertTmp108<FakeI2c, FakePin>) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let release: fn(&mut World) = if asserted_high {
                    ack_release_low
                } else {
                    ack_release_high
                };

                let (shared, mut tmp) = build(Setup {
                    level_high: asserted_high,
                    fl,
                    fh,
                    strict_pin: true,
                    steps: vec![cfg(INTERRUPT, byte1).effect(release), temp(T80).err(I2cError::Other)],
                    ..Default::default()
                });

                match api {
                    Api::Trait => {
                        let result = {
                            let mut fut = pin!(tmp.wait_for_temperature_threshold());
                            poll_once(fut.as_mut())
                        };
                        expect_bus(&shared, result, I2cError::Other);
                    }
                    Api::Inherent => {
                        let result = {
                            let mut fut = pin!(tmp.wait_for_alert());
                            poll_once(fut.as_mut())
                        };
                        expect_alert_bus(&shared, result, I2cError::Other);
                    }
                }

                assert_eq!(
                    take_log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![cfg_byte0(fl, fh), byte1]),
                        Op::WriteRead(vec![0x00], T80.to_vec()),
                    ],
                    "arming the fast path is exactly C0 and a failing T"
                );
                assert_eq!(steps_left(&shared), 0);
                (shared, tmp)
            }

            /// Arm one retained obligation through the interrupt **slow
            /// path** with a successful but **zero-flag** C1, via `api`.
            ///
            /// The resulting slot must be `Some(AlertCause::Unknown)` —
            /// a genuine debt, not `None`.
            fn arm_slow_unknown_via(byte1: u8, api: Api, cancel: bool) -> (Shared, AlertTmp108<FakeI2c, FakePin>) {
                let asserted_high = byte1 == ACTIVE_HIGH;
                let want = asserted_wait(byte1);
                let release: fn(&mut World) = if asserted_high {
                    ack_release_low
                } else {
                    ack_release_high
                };

                let last = if cancel {
                    temp(T80).pending()
                } else {
                    temp(T80).err(I2cError::Other)
                };

                let (shared, mut tmp) = build(Setup {
                    level_high: !asserted_high,
                    steps: vec![cfg(INTERRUPT, byte1), cfg(INTERRUPT, byte1).effect(release), last],
                    ..Default::default()
                });

                {
                    // One scope for both APIs would need one future type,
                    // so the arming is written out per API instead.
                    match api {
                        Api::Trait => {
                            let mut fut = pin!(tmp.wait_for_temperature_threshold());
                            assert!(poll_once(fut.as_mut()).is_pending(), "must park until ALERT asserts");
                            assert_eq!(pin_calls(&shared), vec![want]);
                            // Only the level moves: FL and FH stay clear,
                            // so C1 will return zero flags.
                            shared.lock().unwrap().set_level(asserted_high);
                            let result = poll_once(fut.as_mut());
                            if cancel {
                                assert!(result.is_pending(), "the gated T must park so it can be cancelled");
                            } else {
                                expect_bus(&shared, result, I2cError::Other);
                            }
                        }
                        Api::Inherent => {
                            let mut fut = pin!(tmp.wait_for_alert());
                            assert!(poll_once(fut.as_mut()).is_pending(), "must park until ALERT asserts");
                            assert_eq!(pin_calls(&shared), vec![want]);
                            shared.lock().unwrap().set_level(asserted_high);
                            let result = poll_once(fut.as_mut());
                            if cancel {
                                assert!(result.is_pending(), "the gated T must park so it can be cancelled");
                            } else {
                                expect_alert_bus(&shared, result, I2cError::Other);
                            }
                        }
                    }
                }

                assert_eq!(
                    take_log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![INTERRUPT, byte1]),
                        Op::PinCall(want),
                        Op::PinDone(want),
                        // C1 with FL = FH = 0: still a qualifying
                        // acknowledgment (design section 3.4 step 4).
                        Op::WriteRead(vec![0x01], vec![INTERRUPT, byte1]),
                        Op::WriteRead(vec![0x00], T80.to_vec()),
                    ],
                    "the slow path is C0, one level wait, a zero-flag C1, then T"
                );
                assert_eq!(steps_left(&shared), 0);
                shared.lock().unwrap().strict_pin = true;
                (shared, tmp)
            }

            /// Deliver a retained obligation through the inherent method
            /// and require the literal one-transaction trace.
            fn expect_retained_event(
                shared: &Shared,
                tmp: &mut AlertTmp108<FakeI2c, FakePin>,
                bytes: [u8; 2],
                want_cause: AlertCause,
                want_sixteenths: i16,
            ) {
                push_steps(shared, vec![temp(bytes)]);
                take_log(shared);
                let ev = event({
                    let mut fut = pin!(tmp.wait_for_alert());
                    poll_once(fut.as_mut())
                });

                assert_eq!(ev.cause, want_cause, "a retained delivery reports the *original* cause");
                assert_eq!(
                    ev.temperature,
                    celsius(want_sixteenths),
                    "with a fresh, retry-time sample"
                );
                assert_eq!(
                    take_log(shared),
                    vec![Op::WriteRead(vec![0x00], bytes.to_vec())],
                    "a retained delivery is exactly one transaction — T — and no GPIO at all"
                );
                assert_eq!(steps_left(shared), 0);
            }

            /// As above, through the trait: the scalar projection consumes
            /// the very same obligation and discards its direction.
            fn expect_retained_scalar(
                shared: &Shared,
                tmp: &mut AlertTmp108<FakeI2c, FakePin>,
                bytes: [u8; 2],
                want: f32,
            ) {
                push_steps(shared, vec![temp(bytes)]);
                take_log(shared);
                let result = {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    poll_once(fut.as_mut())
                };

                assert_approx_eq!(degrees(result), want, 1e-4);
                assert_eq!(
                    take_log(shared),
                    vec![Op::WriteRead(vec![0x00], bytes.to_vec())],
                    "a retained scalar delivery is exactly one transaction — T — and no GPIO"
                );
                assert_eq!(steps_left(shared), 0);
            }

            /// Fail one retained retry through `api`, leaving the debt in
            /// place, and require the literal T-only trace.
            fn expect_retained_failure(shared: &Shared, tmp: &mut AlertTmp108<FakeI2c, FakePin>, api: Api) {
                push_steps(shared, vec![temp(T80).err(I2cError::Other)]);
                take_log(shared);
                match api {
                    Api::Trait => {
                        let result = {
                            let mut fut = pin!(tmp.wait_for_temperature_threshold());
                            poll_once(fut.as_mut())
                        };
                        expect_bus(shared, result, I2cError::Other);
                    }
                    Api::Inherent => {
                        let result = {
                            let mut fut = pin!(tmp.wait_for_alert());
                            poll_once(fut.as_mut())
                        };
                        expect_alert_bus(shared, result, I2cError::Other);
                    }
                }
                assert_eq!(
                    take_log(shared),
                    vec![Op::WriteRead(vec![0x00], T80.to_vec())],
                    "a failed retained retry is still T alone — no C0, no C1, no GPIO"
                );
                assert_eq!(steps_left(shared), 0);
            }

            /// Cancel one retained retry through `api` by dropping the
            /// parked future, and require the literal T-only trace.
            fn expect_retained_cancellation(shared: &Shared, tmp: &mut AlertTmp108<FakeI2c, FakePin>, api: Api) {
                push_steps(shared, vec![temp(T80).pending()]);
                take_log(shared);
                match api {
                    Api::Trait => {
                        let mut fut = pin!(tmp.wait_for_temperature_threshold());
                        assert!(poll_once(fut.as_mut()).is_pending());
                    }
                    Api::Inherent => {
                        let mut fut = pin!(tmp.wait_for_alert());
                        assert!(poll_once(fut.as_mut()).is_pending());
                    }
                }
                assert_eq!(
                    take_log(shared),
                    vec![Op::WriteRead(vec![0x00], T80.to_vec())],
                    "a cancelled retained retry is T alone, with no cleanup acknowledgment"
                );
                assert_eq!(steps_left(shared), 0);
            }

            /// Require that the next call is a **fresh acquisition**: a new
            /// C0, and — since the modeled chip has nothing latched and the
            /// pin is released — a level wait it parks on.
            ///
            /// This is the negative half of "consumed exactly once": a
            /// replayed cause would produce a lone `[0x00]` instead.
            fn expect_fresh_acquisition(shared: &Shared, tmp: &mut AlertTmp108<FakeI2c, FakePin>, byte1: u8, api: Api) {
                push_steps(shared, vec![cfg(INTERRUPT, byte1)]);
                {
                    let mut w = shared.lock().unwrap();
                    w.strict_pin = false;
                    w.fl = false;
                    w.fh = false;
                    w.set_level(byte1 != ACTIVE_HIGH);
                }
                take_log(shared);

                match api {
                    Api::Trait => {
                        let mut fut = pin!(tmp.wait_for_temperature_threshold());
                        assert!(
                            poll_once(fut.as_mut()).is_pending(),
                            "a settled obligation must not be replayed"
                        );
                    }
                    Api::Inherent => {
                        let mut fut = pin!(tmp.wait_for_alert());
                        assert!(
                            poll_once(fut.as_mut()).is_pending(),
                            "a settled obligation must not be replayed"
                        );
                    }
                }

                assert_eq!(
                    take_log(shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![INTERRUPT, byte1]),
                        Op::PinCall(asserted_wait(byte1)),
                    ],
                    "a fresh acquisition is a new C0 followed by a level wait"
                );
                assert_eq!(steps_left(shared), 0);
            }

            // ---------------------------------------------------------------
            // Family A — cross-entry-point ownership symmetry (8.3 A).
            //
            // One consumptive stream, two views. Each test asserts the
            // literal transaction log on both sides of the handoff.
            // ---------------------------------------------------------------

            /// Trait acquires FH and fails T; the inherent method retries
            /// and must recover the *direction the trait captured* — which
            /// the chip can no longer report, having been acknowledged.
            #[test]
            fn a_failed_trait_attempt_hands_its_cause_to_the_inherent_method() {
                let (shared, mut tmp) = arm_fast_via(ACTIVE_LOW, false, true, Api::Trait);
                expect_retained_event(&shared, &mut tmp, T25, AlertCause::AboveHigh, 400);
                expect_fresh_acquisition(&shared, &mut tmp, ACTIVE_LOW, Api::Inherent);
            }

            /// As above, but the trait attempt is *cancelled* rather than
            /// failed: no error arm runs, so a "take on entry, restore on
            /// Err" shape would drop the cause here.
            #[test]
            fn a_cancelled_trait_attempt_hands_its_cause_to_the_inherent_method() {
                let (shared, mut tmp) = build(Setup {
                    level_high: false,
                    fh: true,
                    strict_pin: true,
                    steps: vec![cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high), temp(T80).pending()],
                    ..Default::default()
                });

                {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    assert!(poll_once(fut.as_mut()).is_pending());
                }
                assert_eq!(
                    take_log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![cfg_byte0(false, true), ACTIVE_LOW]),
                        Op::WriteRead(vec![0x00], T80.to_vec()),
                    ],
                    "C0 and the cancelled T — and no cleanup read"
                );

                expect_retained_event(&shared, &mut tmp, T25, AlertCause::AboveHigh, 400);
                expect_fresh_acquisition(&shared, &mut tmp, ACTIVE_LOW, Api::Inherent);
            }

            /// The mirror: the inherent method acquires FL and fails T, the
            /// trait settles the debt with T alone, and the *direction is
            /// then gone* — a later inherent call acquires afresh rather
            /// than re-reporting `BelowLow`.
            #[test]
            fn a_failed_inherent_attempt_is_settled_by_the_trait_and_consumed() {
                let (shared, mut tmp) = arm_fast_via(ACTIVE_LOW, true, false, Api::Inherent);
                expect_retained_scalar(&shared, &mut tmp, T25, 25.0);
                expect_fresh_acquisition(&shared, &mut tmp, ACTIVE_LOW, Api::Inherent);
            }

            /// As above with cancellation on the inherent side.
            #[test]
            fn a_cancelled_inherent_attempt_is_settled_by_the_trait_and_consumed() {
                let (shared, mut tmp) = build(Setup {
                    level_high: true,
                    fl: true,
                    strict_pin: true,
                    steps: vec![cfg(INTERRUPT, ACTIVE_HIGH).effect(ack_release_low), temp(T80).pending()],
                    ..Default::default()
                });

                {
                    let mut fut = pin!(tmp.wait_for_alert());
                    assert!(poll_once(fut.as_mut()).is_pending());
                }
                assert_eq!(
                    take_log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![cfg_byte0(true, false), ACTIVE_HIGH]),
                        Op::WriteRead(vec![0x00], T80.to_vec()),
                    ],
                );

                expect_retained_scalar(&shared, &mut tmp, T25, 25.0);
                expect_fresh_acquisition(&shared, &mut tmp, ACTIVE_HIGH, Api::Inherent);
            }

            /// Alternating failures and cancellations across both entry
            /// points before settlement. The obligation is one shared
            /// slot, so none of these attempts may duplicate, refresh or
            /// discard it, and exactly one of them settles it.
            #[test]
            fn alternating_cross_api_attempts_preserve_one_obligation_until_settlement() {
                let (shared, mut tmp) = arm_fast_via(ACTIVE_LOW, true, true, Api::Trait);

                expect_retained_failure(&shared, &mut tmp, Api::Inherent);
                expect_retained_cancellation(&shared, &mut tmp, Api::Trait);
                expect_retained_failure(&shared, &mut tmp, Api::Trait);
                expect_retained_cancellation(&shared, &mut tmp, Api::Inherent);

                // Still `Both` — neither API degraded it to `Unknown`, and
                // no direction was refreshed from the (now clear) chip.
                expect_retained_event(&shared, &mut tmp, T25, AlertCause::Both, 400);
                expect_fresh_acquisition(&shared, &mut tmp, ACTIVE_LOW, Api::Trait);
            }

            /// An `Unknown`-origin obligation crosses the same handoff.
            /// Direction is unavailable, but the *debt* is not: the retry
            /// must still be T-only and must still report `Unknown`.
            #[test]
            fn an_unknown_origin_obligation_hands_over_to_the_inherent_method() {
                let (shared, mut tmp) = arm_slow_unknown_via(ACTIVE_LOW, Api::Trait, false);
                expect_retained_event(&shared, &mut tmp, T25, AlertCause::Unknown, 400);
                expect_fresh_acquisition(&shared, &mut tmp, ACTIVE_LOW, Api::Inherent);
            }

            /// And the other way round: an `Unknown` acquired by the
            /// inherent method is settled by the trait's scalar result.
            #[test]
            fn an_unknown_origin_obligation_is_settled_by_the_trait() {
                let (shared, mut tmp) = arm_slow_unknown_via(ACTIVE_HIGH, Api::Inherent, false);
                expect_retained_scalar(&shared, &mut tmp, T25, 25.0);
                expect_fresh_acquisition(&shared, &mut tmp, ACTIVE_HIGH, Api::Trait);
            }

            // ---------------------------------------------------------------
            // Family B — `Some(Unknown)` is not `None` (8.3 B).
            //
            // The single most dangerous slip in this change is collapsing
            // "pending, direction unknown" into "empty". Both states have
            // the same *cause* information and completely different
            // *protocol* consequences: T alone versus C0 + level wait.
            // ---------------------------------------------------------------

            fn unknown_retention_case(byte1: u8, cancel: bool) {
                let (shared, mut tmp) = arm_slow_unknown_via(byte1, Api::Inherent, cancel);

                // If `Some(Unknown)` had collapsed to `None`, this call
                // would issue a C0 and then park on a level wait against a
                // released pin — the fixture's `strict_pin` turns that into
                // a named panic rather than a hang.
                expect_retained_event(&shared, &mut tmp, T25, AlertCause::Unknown, 400);
                expect_fresh_acquisition(&shared, &mut tmp, byte1, Api::Inherent);
            }

            #[test]
            fn a_zero_flag_c1_obligation_survives_a_failed_t_active_low() {
                unknown_retention_case(ACTIVE_LOW, false);
            }

            #[test]
            fn a_zero_flag_c1_obligation_survives_a_failed_t_active_high() {
                unknown_retention_case(ACTIVE_HIGH, false);
            }

            #[test]
            fn a_zero_flag_c1_obligation_survives_a_cancelled_t_active_low() {
                unknown_retention_case(ACTIVE_LOW, true);
            }

            #[test]
            fn a_zero_flag_c1_obligation_survives_a_cancelled_t_active_high() {
                unknown_retention_case(ACTIVE_HIGH, true);
            }

            /// The same `Some(Unknown)` debt, settled through the trait.
            #[test]
            fn a_zero_flag_c1_obligation_settles_through_the_trait_with_t_only() {
                let (shared, mut tmp) = arm_slow_unknown_via(ACTIVE_LOW, Api::Inherent, true);
                expect_retained_scalar(&shared, &mut tmp, T25, 25.0);
                expect_fresh_acquisition(&shared, &mut tmp, ACTIVE_LOW, Api::Trait);
            }

            // ---------------------------------------------------------------
            // Family C — event A's cause survives an opposite-direction B
            // (8.3 C, design section 3.7).
            // ---------------------------------------------------------------

            /// A is acknowledged with FH and its T fails. B then latches FL
            /// in the chip. A's retry must report `AboveHigh` — with a
            /// sample that deliberately contradicts it — must leave B's
            /// flag standing, and only the following *fresh* acquisition
            /// may collect B.
            fn opposite_latch_case(a_low: bool, a_high: bool, want_a: AlertCause, want_b: AlertCause) {
                let (shared, mut tmp) = arm_fast_via(ACTIVE_LOW, a_low, a_high, Api::Inherent);

                // B latches in the opposite direction and re-asserts ALERT.
                {
                    let mut w = shared.lock().unwrap();
                    w.fl = a_high;
                    w.fh = a_low;
                    w.set_level(false);
                }

                // T80 is above any plausible high limit, T25 inside the
                // band: whichever A's direction is, the sample argues for
                // the other one or for neither. It must change nothing.
                let contradicting = if want_a == AlertCause::AboveHigh { T25 } else { T80 };
                let sixteenths = if want_a == AlertCause::AboveHigh { 400 } else { 1280 };
                expect_retained_event(&shared, &mut tmp, contradicting, want_a, sixteenths);

                {
                    let w = shared.lock().unwrap();
                    assert_eq!(
                        (w.fl, w.fh),
                        (a_high, a_low),
                        "a retained delivery performs no configuration read, so B's flag must still \
                         be latched in the chip"
                    );
                }

                // Only now may B be collected — by a fresh C0, on the fast
                // path, because B's flag is set and the pin is asserted.
                push_steps(
                    &shared,
                    vec![cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high), temp(T25)],
                );
                take_log(&shared);
                let ev = event({
                    let mut fut = pin!(tmp.wait_for_alert());
                    poll_once(fut.as_mut())
                });
                assert_eq!(ev.cause, want_b, "the following fresh acquisition collects B, not A");
                assert_eq!(
                    take_log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![cfg_byte0(a_high, a_low), ACTIVE_LOW]),
                        Op::WriteRead(vec![0x00], T25.to_vec()),
                    ],
                    "B is collected by C0 and T — the fast path, with no GPIO"
                );
                assert_eq!(steps_left(&shared), 0);
            }

            #[test]
            fn a_retained_above_high_is_not_rewritten_by_a_newer_low_side_latch() {
                opposite_latch_case(false, true, AlertCause::AboveHigh, AlertCause::BelowLow);
            }

            #[test]
            fn a_retained_below_low_is_not_rewritten_by_a_newer_high_side_latch() {
                opposite_latch_case(true, false, AlertCause::BelowLow, AlertCause::AboveHigh);
            }

            /// The same separation across an API handoff: A is acquired by
            /// the trait, B latches, and the inherent retry must still
            /// report A's `AboveHigh`.
            #[test]
            fn a_mixed_method_retry_reports_a_across_a_newer_opposite_latch() {
                let (shared, mut tmp) = arm_fast_via(ACTIVE_HIGH, false, true, Api::Trait);

                {
                    let mut w = shared.lock().unwrap();
                    w.fl = true;
                    w.set_level(true);
                }

                expect_retained_event(&shared, &mut tmp, T25, AlertCause::AboveHigh, 400);
                assert!(shared.lock().unwrap().fl, "B's FL must survive A's T-only delivery");

                push_steps(
                    &shared,
                    vec![cfg(INTERRUPT, ACTIVE_HIGH).effect(ack_release_low), temp(T80)],
                );
                take_log(&shared);
                let ev = event({
                    let mut fut = pin!(tmp.wait_for_alert());
                    poll_once(fut.as_mut())
                });
                assert_eq!(ev.cause, AlertCause::BelowLow);
                assert_eq!(
                    take_log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![cfg_byte0(true, false), ACTIVE_HIGH]),
                        Op::WriteRead(vec![0x00], T80.to_vec()),
                    ],
                );
                assert_eq!(steps_left(&shared), 0);
            }

            // ---------------------------------------------------------------
            // Family D — settlement happens in the poll that resolves T
            // (8.3 D, design section 3.4 step 7).
            //
            // `Outcome::Pending` parks forever, so it can only prove that
            // an *unresolved* T keeps the debt. `Outcome::Gated` records
            // its transaction, parks, and then resolves — which is what
            // makes the completion poll itself observable.
            // ---------------------------------------------------------------

            /// Initial (post-C0) delivery through the inherent method.
            #[test]
            fn an_initial_delivery_settles_in_the_poll_that_resolves_temperature() {
                let (shared, mut tmp) = build(Setup {
                    level_high: false,
                    fh: true,
                    strict_pin: true,
                    steps: vec![cfg(INTERRUPT, ACTIVE_LOW).effect(ack_release_high), temp(T25).gated()],
                    ..Default::default()
                });

                let expected = vec![
                    Op::WriteRead(vec![0x01], vec![cfg_byte0(false, true), ACTIVE_LOW]),
                    Op::WriteRead(vec![0x00], T25.to_vec()),
                ];

                {
                    let mut fut = pin!(tmp.wait_for_alert());
                    assert!(poll_once(fut.as_mut()).is_pending(), "T has not resolved yet");
                    assert_eq!(log(&shared), expected, "C0 and T, issued once");

                    release_gate(&shared);
                    let ev = event(poll_once(fut.as_mut()));

                    assert_eq!(ev.cause, AlertCause::AboveHigh);
                    assert_eq!(ev.temperature, celsius(400));
                    assert_eq!(
                        log(&shared),
                        expected,
                        "the completion poll performs no further I/O: no second T, no GPIO, no \
                         configuration read"
                    );
                }
                assert_eq!(steps_left(&shared), 0);
                take_log(&shared);

                // The debt was cleared *in that poll*, not merely at some
                // later point: the next call acquires afresh.
                expect_fresh_acquisition(&shared, &mut tmp, ACTIVE_LOW, Api::Inherent);
            }

            /// Retained delivery through the inherent method.
            #[test]
            fn a_retained_delivery_settles_in_the_poll_that_resolves_temperature() {
                let (shared, mut tmp) = arm_fast_via(ACTIVE_LOW, true, false, Api::Inherent);
                push_steps(&shared, vec![temp(T25).gated()]);
                take_log(&shared);

                let expected = vec![Op::WriteRead(vec![0x00], T25.to_vec())];

                {
                    let mut fut = pin!(tmp.wait_for_alert());
                    assert!(poll_once(fut.as_mut()).is_pending());
                    assert_eq!(log(&shared), expected, "a retained delivery issues T and nothing else");

                    release_gate(&shared);
                    let ev = event(poll_once(fut.as_mut()));

                    assert_eq!(
                        ev.cause,
                        AlertCause::BelowLow,
                        "the original cause, not a refreshed one"
                    );
                    assert_eq!(ev.temperature, celsius(400));
                    assert_eq!(log(&shared), expected, "the completion poll adds no I/O");
                }
                assert_eq!(steps_left(&shared), 0);
                take_log(&shared);

                expect_fresh_acquisition(&shared, &mut tmp, ACTIVE_LOW, Api::Inherent);
            }

            /// Initial delivery through the trait projection.
            #[test]
            fn an_initial_scalar_delivery_settles_in_the_poll_that_resolves_temperature() {
                let (shared, mut tmp) = build(Setup {
                    level_high: true,
                    fl: true,
                    strict_pin: true,
                    steps: vec![cfg(INTERRUPT, ACTIVE_HIGH).effect(ack_release_low), temp(T25).gated()],
                    ..Default::default()
                });

                let expected = vec![
                    Op::WriteRead(vec![0x01], vec![cfg_byte0(true, false), ACTIVE_HIGH]),
                    Op::WriteRead(vec![0x00], T25.to_vec()),
                ];

                {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    assert!(poll_once(fut.as_mut()).is_pending());
                    assert_eq!(log(&shared), expected);

                    release_gate(&shared);
                    let result = poll_once(fut.as_mut());
                    assert_approx_eq!(degrees(result), 25.0, 1e-4);
                    assert_eq!(
                        log(&shared),
                        expected,
                        "the trait adds no await and no I/O after obtaining the event"
                    );
                }
                assert_eq!(steps_left(&shared), 0);
                take_log(&shared);

                expect_fresh_acquisition(&shared, &mut tmp, ACTIVE_HIGH, Api::Trait);
            }

            /// Retained delivery through the trait projection.
            #[test]
            fn a_retained_scalar_delivery_settles_in_the_poll_that_resolves_temperature() {
                let (shared, mut tmp) = arm_fast_via(ACTIVE_HIGH, false, true, Api::Trait);
                push_steps(&shared, vec![temp(T80).gated()]);
                take_log(&shared);

                let expected = vec![Op::WriteRead(vec![0x00], T80.to_vec())];

                {
                    let mut fut = pin!(tmp.wait_for_temperature_threshold());
                    assert!(poll_once(fut.as_mut()).is_pending());
                    assert_eq!(log(&shared), expected);

                    release_gate(&shared);
                    let result = poll_once(fut.as_mut());
                    assert_approx_eq!(degrees(result), 80.0, 1e-4);
                    assert_eq!(log(&shared), expected);
                }
                assert_eq!(steps_left(&shared), 0);
                take_log(&shared);

                // And the consumed cause is not replayed to the inherent
                // method afterwards.
                expect_fresh_acquisition(&shared, &mut tmp, ACTIVE_HIGH, Api::Inherent);
            }

            // ---------------------------------------------------------------
            // Family E — fresh comparator cancellation versus an old
            // interrupt debt (8.3 E).
            //
            // Both halves are required, and they fail *different* wrong
            // implementations.
            // ---------------------------------------------------------------

            /// A fresh comparator acquisition never arms the slot, even
            /// though it has a local cause and even though C0 returned
            /// nonzero flags. Cancelling its T therefore leaves nothing
            /// behind: the retry must observe the chip and the pin again.
            ///
            /// A generic "a cause exists, therefore retain" branch would
            /// turn this retry into a bare T.
            #[test]
            fn a_cancelled_fresh_comparator_delivery_retains_nothing() {
                let (shared, mut tmp) = build(Setup {
                    // Already asserted, so the level wait completes at once.
                    level_high: false,
                    // Nonzero entry flags: stale evidence of an earlier
                    // comparison, never a reason to retain.
                    fl: true,
                    fh: true,
                    steps: vec![cfg(COMPARATOR, ACTIVE_LOW), temp(T25).pending()],
                    ..Default::default()
                });

                {
                    let mut fut = pin!(tmp.wait_for_alert());
                    assert!(poll_once(fut.as_mut()).is_pending());
                }
                assert_eq!(
                    take_log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![COMPARATOR | 0x18, ACTIVE_LOW]),
                        Op::PinCall(PinOp::WaitLow),
                        Op::PinDone(PinOp::WaitLow),
                        Op::WriteRead(vec![0x00], T25.to_vec()),
                    ],
                    "fresh comparator: C0, one level wait, T"
                );
                assert_eq!(steps_left(&shared), 0);

                // The retry must be a full fresh acquisition again.
                push_steps(&shared, vec![cfg(COMPARATOR, ACTIVE_LOW), temp(T80)]);
                let ev = event({
                    let mut fut = pin!(tmp.wait_for_alert());
                    poll_once(fut.as_mut())
                });
                assert_eq!(ev.cause, AlertCause::Unknown, "fresh comparator is always Unknown");
                assert_eq!(ev.temperature, celsius(1280));
                assert_eq!(
                    take_log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![COMPARATOR | 0x18, ACTIVE_LOW]),
                        Op::PinCall(PinOp::WaitLow),
                        Op::PinDone(PinOp::WaitLow),
                        Op::WriteRead(vec![0x00], T80.to_vec()),
                    ],
                    "the retry repeats C0 and the level wait — it is not a retained T"
                );
                assert_eq!(steps_left(&shared), 0);
            }

            /// The converse: an interrupt debt outranks a later switch to
            /// comparator mode, and a *cancelled* retry in that state must
            /// keep it. An unconditional "we are in comparator mode, so
            /// clear the slot" would lose the event here.
            #[test]
            fn a_cancelled_retained_retry_in_comparator_mode_keeps_the_interrupt_debt() {
                let (shared, mut tmp) = arm_fast_via(ACTIVE_LOW, false, true, Api::Inherent);

                // Reconfigure the part to comparator mode through the inner
                // sensor. This is real I/O on the modeled chip, not a
                // rewritten fixture.
                push_steps(
                    &shared,
                    vec![cfg(INTERRUPT, ACTIVE_LOW), write(vec![0x01, COMPARATOR, ACTIVE_LOW])],
                );
                let configured = {
                    let mut fut = pin!(tmp.sensor_mut().configure(Config {
                        thermostat_mode: Thermostat::Comparator,
                        alert_polarity: Polarity::ActiveLow,
                        ..Default::default()
                    }));
                    poll_once(fut.as_mut())
                };
                assert!(matches!(configured, Poll::Ready(Ok(()))));
                assert_eq!(steps_left(&shared), 0);
                take_log(&shared);

                // Cancelled retry, then a failed one, then settlement —
                // all T-only, all still `AboveHigh`.
                expect_retained_cancellation(&shared, &mut tmp, Api::Inherent);
                expect_retained_failure(&shared, &mut tmp, Api::Trait);
                expect_retained_event(&shared, &mut tmp, T25, AlertCause::AboveHigh, 400);

                // Once settled, the chip's *current* comparator mode governs.
                push_steps(&shared, vec![cfg(COMPARATOR, ACTIVE_LOW), temp(T80)]);
                shared.lock().unwrap().strict_pin = false;
                shared.lock().unwrap().set_level(false);
                take_log(&shared);
                let ev = event({
                    let mut fut = pin!(tmp.wait_for_alert());
                    poll_once(fut.as_mut())
                });
                assert_eq!(ev.cause, AlertCause::Unknown);
                assert_eq!(
                    take_log(&shared),
                    vec![
                        Op::WriteRead(vec![0x01], vec![COMPARATOR, ACTIVE_LOW]),
                        Op::PinCall(PinOp::WaitLow),
                        Op::PinDone(PinOp::WaitLow),
                        Op::WriteRead(vec![0x00], T80.to_vec()),
                    ],
                );
                assert_eq!(steps_left(&shared), 0);
            }

            // ---------------------------------------------------------------
            // Family F — lifecycle abandonment, invariant I-25 (8.4).
            //
            // The retained state is armed by real acquisition, never by
            // assigning the private field. Decomposition must be silent:
            // no acknowledgment, no cleanup read, no GPIO.
            // ---------------------------------------------------------------

            /// `into_inner` abandons the obligation without I/O, and
            /// `into_alert` re-wraps into an **empty** slot.
            #[test]
            fn into_inner_abandons_a_retained_event_without_driver_io() {
                let (shared, tmp) = arm_fast_via(ACTIVE_LOW, false, true, Api::Inherent);

                let (sensor, alert) = tmp.into_inner();
                assert!(
                    log(&shared).is_empty(),
                    "decomposition must perform no I/O, saw {:?}",
                    log(&shared)
                );
                assert_eq!(steps_left(&shared), 0);

                let mut tmp = sensor.into_alert(alert);
                assert!(
                    log(&shared).is_empty(),
                    "construction must perform no I/O, saw {:?}",
                    log(&shared)
                );

                // The obligation did not travel with the parts.
                expect_fresh_acquisition(&shared, &mut tmp, ACTIVE_LOW, Api::Inherent);
            }

            /// `destroy` recovers the bus and the pin with no I/O, and a
            /// wrapper rebuilt from them starts empty.
            #[test]
            fn destroy_abandons_a_retained_event_without_driver_io() {
                let (shared, tmp) = arm_slow_unknown_via(ACTIVE_HIGH, Api::Trait, false);

                let (i2c, alert) = tmp.destroy();
                assert!(
                    log(&shared).is_empty(),
                    "destroy must perform no I/O, saw {:?}",
                    log(&shared)
                );
                assert_eq!(steps_left(&shared), 0);

                let mut tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);
                assert!(log(&shared).is_empty(), "construction must perform no I/O");

                expect_fresh_acquisition(&shared, &mut tmp, ACTIVE_HIGH, Api::Trait);
            }

            /// Dropping the wrapper is equally silent. The fake world is
            /// held by an external `Arc`, so it outlives the driver and can
            /// still be inspected — including for a cleanup acknowledgment
            /// that must not exist.
            #[test]
            fn dropping_the_wrapper_abandons_a_retained_event_without_driver_io() {
                let (shared, tmp) = arm_fast_via(ACTIVE_HIGH, true, true, Api::Inherent);

                drop(tmp);

                assert!(
                    log(&shared).is_empty(),
                    "dropping the wrapper must perform no cleanup acknowledgment, saw {:?}",
                    log(&shared)
                );
                assert_eq!(steps_left(&shared), 0, "and must consume no scripted transaction");
            }
        }
    }
}

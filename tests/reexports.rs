//! Public-API reachability and contract pins: Config field types,
//! driver types and the alert types must be nameable from outside the
//! crate, and a few of their runtime contracts are asserted here too.
use tmp108::{Celsius, Config, ConversionRate, Hysteresis, Mode, OutOfRange, Polarity, Thermostat};

#[test]
fn config_with_explicit_fields_compiles() {
    let cfg = Config {
        thermostat_mode: Thermostat::Interrupt,
        alert_polarity: Polarity::ActiveHigh,
        conversion_rate: ConversionRate::SixteenHz,
        hysteresis: Hysteresis::FourC,
    };
    // Avoid unused-binding warning; the assertion is on compilation.
    let _ = cfg;
    // Also pin Mode (re-exported even though not a Config field).
    let _ = Mode::Continuous;
}

/// `Mode`'s conversions are public surface: `Mode` is `pub use`'d, so
/// its `From<u8>`, `Default` and the `TryFrom<u8>` that follows from
/// `From` are all callable by downstream code and are semver-relevant.
///
/// `From<u8>` is total because every encoding of the two-bit M field
/// names a real mode: `M1 = 1` selects continuous conversion regardless
/// of `M0` (datasheet SBOS663A §7.4.3), so `3` is `Continuous`, not an
/// error. Because the conversion is infallible, the blanket
/// `TryFrom` applies and its `Error` is `Infallible`; pinning that
/// associated type is what catches a regression to a fallible decoder.
#[test]
// The associated-type pin below (`<Mode as TryFrom<u8>>::Error`) is
// what fixes the error type as `Infallible`; the `try_from` call alone
// would not. clippy's "use the infallible one" suggestion would remove
// the call that keeps that pin exercised.
#[allow(clippy::unnecessary_fallible_conversions)]
fn mode_conversions_are_public_and_total() {
    fn infallible(_: core::convert::Infallible) {}

    assert_eq!(Mode::from(0u8), Mode::Shutdown);
    assert_eq!(Mode::from(1u8), Mode::OneShot);
    assert_eq!(Mode::from(2u8), Mode::Continuous);
    assert_eq!(Mode::from(3u8), Mode::Continuous);

    // Power-on reset is 0x1022, i.e. M = 0b10.
    assert_eq!(Mode::default(), Mode::Continuous);

    // Pin `<Mode as TryFrom<u8>>::Error == Infallible`.
    let _: fn(<Mode as TryFrom<u8>>::Error) = infallible;
    assert_eq!(Mode::try_from(3u8), Ok(Mode::Continuous));

    // The write path is unchanged: Continuous still encodes as 0b10.
    assert_eq!(u8::from(Mode::Continuous), 2);
}

/// The temperature newtype and its parse error must be reachable from
/// outside the crate — without them a caller cannot build a value for
/// `set_low_limit` / `set_high_limit`.
#[test]
fn celsius_is_constructible_from_outside() {
    assert_eq!(Celsius::try_from_degrees(25.0).unwrap().sixteenths(), 400);
    assert_eq!(
        Celsius::from_sixteenths(400).unwrap(),
        Celsius::try_from_degrees(25.0).unwrap()
    );
    assert_eq!(Celsius::try_from_degrees(f32::NAN), Err(OutOfRange::NotANumber));
    let _ = (Celsius::MIN, Celsius::MAX, Celsius::ZERO);
}

/// The blocking driver type must be reachable from outside the crate.
#[allow(dead_code)]
fn pin_blocking_driver_type() {
    fn _type_check<I2C: embedded_hal::i2c::I2c>(_: tmp108::Tmp108<I2C>) {}
}

/// The async driver type must be reachable from outside the crate when
/// the `async` feature is enabled.
#[cfg(feature = "async")]
#[allow(dead_code)]
fn pin_async_driver_type() {
    fn _type_check<I2C: embedded_hal_async::i2c::I2c>(_: tmp108::AsyncTmp108<I2C>) {}
}

/// `AlertCause` is ungated public surface: it must be reachable, its
/// four variants constructible, and exhaustive matching must compile
/// with no optional features enabled.
// Pinning the `Clone` impl is the point of the assertion, so the
// redundant clone on a `Copy` type is deliberate.
#[allow(clippy::clone_on_copy)]
#[test]
fn alert_cause_is_reachable_and_exhaustive() {
    use tmp108::AlertCause;

    fn describe(cause: AlertCause) -> &'static str {
        match cause {
            AlertCause::BelowLow => "FL observed",
            AlertCause::AboveHigh => "FH observed",
            AlertCause::Both => "FL and FH observed",
            AlertCause::Unknown => "direction unavailable",
        }
    }

    fn eq_bound<T: Eq>(_: T) {}

    assert_eq!(describe(AlertCause::BelowLow), "FL observed");
    assert_eq!(describe(AlertCause::AboveHigh), "FH observed");
    assert_eq!(describe(AlertCause::Both), "FL and FH observed");
    assert_eq!(describe(AlertCause::Unknown), "direction unavailable");

    // Clone, Copy, Debug, PartialEq, Eq, Hash.
    let copied = AlertCause::Both;
    let cloned = copied.clone();
    assert_eq!(copied, cloned);
    assert_ne!(copied, AlertCause::Unknown);
    assert_eq!(format!("{copied:?}"), "Both");
    let mut set = std::collections::HashSet::new();
    assert!(set.insert(AlertCause::Unknown));
    assert!(!set.insert(AlertCause::Unknown));
    eq_bound(copied);
}

/// `AlertEvent` must be constructible and destructurable from outside
/// the crate, with both field types reachable.
// As above: the clone is pinning the derive, not avoiding a move.
#[allow(clippy::clone_on_copy)]
#[cfg(feature = "embedded-sensors-hal-async")]
#[test]
fn alert_event_is_constructible_from_outside() {
    use tmp108::{AlertCause, AlertEvent, Celsius};

    let event = AlertEvent {
        cause: AlertCause::AboveHigh,
        temperature: Celsius::try_from_degrees(25.0).unwrap(),
    };

    let AlertEvent { cause, temperature } = event;
    assert_eq!(cause, AlertCause::AboveHigh);
    assert_eq!(temperature.sixteenths(), 400);

    // Clone, Copy, Debug, PartialEq, Eq, Hash.
    let copied = event;
    assert_eq!(copied, event.clone());
    assert_ne!(format!("{event:?}"), "");
    let mut set = std::collections::HashSet::new();
    assert!(set.insert(event));
    assert!(!set.insert(copied));
}

/// `AlertTmp108::wait_for_alert` is public surface with the existing
/// I2C / `Wait` / `InputPin` bounds and the existing error type. Note
/// that `TemperatureThresholdWait` is deliberately *not* imported: the
/// inherent method must be callable without it.
#[cfg(feature = "embedded-sensors-hal-async")]
#[allow(dead_code)]
fn pin_wait_for_alert() {
    async fn _type_check<I2C, ALERT>(
        sensor: &mut tmp108::AlertTmp108<I2C, ALERT>,
    ) -> Result<tmp108::AlertEvent, tmp108::Error<I2C::Error, <ALERT as embedded_hal::digital::ErrorType>::Error>>
    where
        I2C: embedded_hal_async::i2c::I2c,
        ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin,
    {
        sensor.wait_for_alert().await
    }
}

/// `OneShotError` is ungated public surface: it must be nameable from
/// outside the crate, all four variants constructible, and the match
/// below must compile *without* a wildcard arm. That exhaustiveness is
/// the point — it pins the deliberate absence of `#[non_exhaustive]`,
/// which nothing inside the crate can pin for us.
// Pinning the `Clone` impl is the point of the assertion, so the
// redundant clone on a `Copy` type is deliberate.
#[allow(clippy::clone_on_copy)]
#[test]
fn one_shot_error_is_reachable_and_exhaustive() {
    use tmp108::{Mode, OneShotError};

    // A concrete `E` a downstream caller could plausibly instantiate.
    type Err = OneShotError<embedded_hal::i2c::ErrorKind>;

    fn describe(error: Err) -> &'static str {
        match error {
            OneShotError::Bus(_) => "bus",
            OneShotError::PreparationNotShutdown(_) => "not shutdown",
            OneShotError::UnexpectedMode(_) => "unexpected mode",
            OneShotError::Timeout => "timeout",
        }
    }

    fn eq_bound<T: Eq>(_: T) {}

    assert_eq!(describe(OneShotError::Bus(embedded_hal::i2c::ErrorKind::Bus)), "bus");
    assert_eq!(
        describe(OneShotError::PreparationNotShutdown(Mode::Continuous)),
        "not shutdown"
    );
    assert_eq!(
        describe(OneShotError::UnexpectedMode(Mode::Continuous)),
        "unexpected mode"
    );
    assert_eq!(describe(OneShotError::Timeout), "timeout");

    // Clone, Copy, Debug, PartialEq, Eq — the committed derive set.
    // Note: no `Hash`, unlike `AlertCause`.
    let copied: Err = OneShotError::Timeout;
    let cloned = copied.clone();
    assert_eq!(copied, cloned);
    assert_ne!(copied, OneShotError::UnexpectedMode(Mode::Continuous));
    assert_eq!(format!("{copied:?}"), "Timeout");
    eq_bound(copied);
}

/// `Tmp108::one_shot` is public surface: the generic delay
/// parameter, the `shutdown_settle_ms` scalar, and the
/// `Result<Celsius, OneShotError<I2C::Error>>` return type are all
/// pinned here from outside the crate.
#[allow(dead_code)]
fn pin_blocking_one_shot() {
    fn _type_check<I2C, DELAY>(
        tmp: &mut tmp108::Tmp108<I2C>,
        delay: &mut DELAY,
        settle_ms: u32,
    ) -> Result<Celsius, tmp108::OneShotError<I2C::Error>>
    where
        I2C: embedded_hal::i2c::I2c,
        DELAY: embedded_hal::delay::DelayNs,
    {
        tmp.one_shot(delay, settle_ms)
    }
}

/// The async `one_shot` carries the same signature over the
/// async I2C and delay traits.
#[cfg(feature = "async")]
#[allow(dead_code)]
fn pin_async_one_shot() {
    async fn _type_check<I2C, DELAY>(
        tmp: &mut tmp108::AsyncTmp108<I2C>,
        delay: &mut DELAY,
        settle_ms: u32,
    ) -> Result<Celsius, tmp108::OneShotError<I2C::Error>>
    where
        I2C: embedded_hal_async::i2c::I2c,
        DELAY: embedded_hal_async::delay::DelayNs,
    {
        tmp.one_shot(delay, settle_ms).await
    }
}

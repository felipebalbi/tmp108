//! Compile-only test: Config field types and driver types must be
//! reachable from outside the crate.
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
    assert!(!format!("{event:?}").is_empty());
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

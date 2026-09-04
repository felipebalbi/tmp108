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

#[test]
fn mode_is_reachable() {
    assert_ne!(tmp108::Mode::OneShot, tmp108::Mode::Shutdown);
}

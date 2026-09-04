//! TMP108 temperature threshold alert example.
//!
//! Exercises [`AlertTmp108`] against real hardware: the driver arms a
//! threshold, waits on the ALERT pin, and reports the temperature that
//! tripped it.
//!
//! # Wiring
//!
//! | TMP108 | Pico de Gallo    |
//! |--------|------------------|
//! | A0     | GND (address 0x48) |
//! | SDA    | SDA              |
//! | SCL    | SCL              |
//! | ALERT  | GPIO1            |
//!
//! The TMP108 drives ALERT open-drain but provides its own internal
//! 80-120 kOhm pull-up to V+, so no external resistor is required. This
//! example enables the Pico de Gallo's internal pull-up as well, which
//! parallels the two and sharpens the falling edge.

use std::time::Duration;

use anyhow::{Result, anyhow};
use embedded_sensors_hal_async::temperature::{TemperatureSensor, TemperatureThresholdSet, TemperatureThresholdWait};
use pico_de_gallo_hal::{GpioDirection, GpioPull, Hal};
use tmp108::asynch::AlertTmp108;
use tmp108::{Config, Polarity, Thermostat};

/// Pico de Gallo GPIO that the TMP108 ALERT pin is wired to.
const ALERT_PIN: u8 = 1;

/// Power-on limit window, restored before exiting.
const RESET_LOW_LIMIT: f32 = -128.0;
const RESET_HIGH_LIMIT: f32 = 127.9375;

/// How long to wait for ALERT before giving up.
///
/// The default conversion rate is 1 Hz, so one or two conversions is all
/// that should be needed.
const ALERT_TIMEOUT: Duration = Duration::from_secs(10);

#[tokio::main]
async fn main() -> Result<()> {
    let hal = Hal::new();
    let i2c = hal.i2c();

    let mut alert = hal.gpio(ALERT_PIN);
    alert
        .set_config(GpioDirection::Input, GpioPull::Up)
        .map_err(|e| anyhow!("failed to configure ALERT pin: {e}"))?;

    let mut tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);

    // Interrupt mode: ALERT asserts when a limit is crossed and stays
    // asserted until the configuration register is read.
    tmp.tmp108
        .configure(Config {
            thermostat_mode: Thermostat::Interrupt,
            alert_polarity: Polarity::ActiveLow,
            ..Default::default()
        })
        .await
        .map_err(|e| anyhow!("failed to configure sensor: {e:?}"))?;

    let ambient = tmp
        .temperature()
        .await
        .map_err(|e| anyhow!("failed to read temperature: {e:?}"))?;
    println!("Ambient: {ambient:.4} C");

    // Park the limit window either side of ambient so that nothing is
    // latched before we start waiting.
    tmp.set_temperature_threshold_low(ambient - 20.0)
        .await
        .map_err(|e| anyhow!("failed to set low threshold: {e:?}"))?;
    tmp.set_temperature_threshold_high(ambient + 20.0)
        .await
        .map_err(|e| anyhow!("failed to set high threshold: {e:?}"))?;

    // Now drop the high limit below ambient. The next conversion sees an
    // over-temperature condition and asserts ALERT.
    let threshold = ambient - 5.0;
    println!("Arming high threshold at {threshold:.4} C, waiting for ALERT...");
    tmp.set_temperature_threshold_high(threshold)
        .await
        .map_err(|e| anyhow!("failed to arm high threshold: {e:?}"))?;

    let tripped = tokio::time::timeout(ALERT_TIMEOUT, tmp.wait_for_temperature_threshold())
        .await
        .map_err(|_| anyhow!("timed out after {ALERT_TIMEOUT:?} waiting for ALERT"))?
        .map_err(|e| anyhow!("threshold wait failed: {e:?}"))?;

    println!("ALERT fired, temperature {tripped:.4} C");

    if tripped <= threshold {
        return Err(anyhow!(
            "ALERT fired at {tripped:.4} C, which does not exceed the {threshold:.4} C threshold"
        ));
    }

    // Restore the power-on limit window so the part is left as found.
    tmp.set_temperature_threshold_low(RESET_LOW_LIMIT)
        .await
        .map_err(|e| anyhow!("failed to restore low threshold: {e:?}"))?;
    tmp.set_temperature_threshold_high(RESET_HIGH_LIMIT)
        .await
        .map_err(|e| anyhow!("failed to restore high threshold: {e:?}"))?;
    tmp.tmp108
        .configure(Config::default())
        .await
        .map_err(|e| anyhow!("failed to restore configuration: {e:?}"))?;

    println!("Restored power-on defaults.");

    Ok(())
}

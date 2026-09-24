//! TMP108 ALERT pin example — interrupt mode.
//!
//! # Hardware
//!
//! - Pico de Gallo USB-attached host adapter
//! - TMP108 on the default I2C bus, A0 → GND (address `0x48`)
//! - TMP108 ALERT pin tied to Pico de Gallo GPIO0 with an external 2 kΩ
//!   pull-up to V+
//!
//! # Cargo features
//!
//! Requires `--features async,embedded-sensors-hal-async`. The `AlertTmp108`
//! wrapper that exposes `wait_for_temperature_threshold` only exists under
//! this combination.
//!
//! # Behavior
//!
//! Setup updates the configuration register and writes the low/high limit
//! registers. A retained interrupt delivery makes `wait_for_temperature_threshold`
//! read temperature only, once. Otherwise fresh acquisition reads configuration,
//! capturing FL/FH while acknowledging the interrupt and clearing the pin.
//! If either flag was set, it skips GPIO waiting and any second acknowledgment.
//! Otherwise it waits for the asserted pin level, then reads configuration
//! once more to acknowledge that alert. Finally it reads the temperature
//! register for the latest conversion, not a trigger-time sample.
//!
//! A latched transient can be reported after temperature returns inside the
//! band. The returned reading cannot identify which threshold caused it.
//! A persistent condition can relatch, so repeated calls need not correspond
//! to distinct threshold crossings. After successful interrupt acknowledgment,
//! a temperature-read failure or cancellation leaves a delivery obligation in
//! this wrapper. Retrying reads temperature at retry time, not a cached sample,
//! even after reconfiguration to Comparator mode. Direct sensor reads do not
//! clear the obligation; decomposition or dropping the wrapper abandons it.
//! Entry/acknowledging-read failure or cancellation can still lose an event
//! unrecoverably. The waiter is not fully cancel-safe or exactly-once.
//! It never retries internally: retrying callers need backoff or a bound,
//! since repeated immediately-ready bus errors need not yield.
//!
//! # Usage
//!
//! Run the example, then warm the TMP108 with a finger (or breathe on it) to
//! cross the 30 °C high threshold. The program will print the latest
//! temperature reading after servicing the alert and exit.

#[cfg(not(all(feature = "async", feature = "embedded-sensors-hal-async")))]
fn main() {
    eprintln!("examples/alert_interrupt.rs requires --features async,embedded-sensors-hal-async");
}

#[cfg(all(feature = "async", feature = "embedded-sensors-hal-async"))]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use anyhow::anyhow;
    use embedded_sensors_hal_async::temperature::{TemperatureThresholdSet, TemperatureThresholdWait};
    use pico_de_gallo_hal::Hal;
    use pico_de_gallo_lib::{GpioDirection, GpioPull};
    use tmp108::{AlertTmp108, Config, Polarity, Thermostat};

    let hal = Hal::new();
    let i2c = hal.i2c();
    let mut alert = hal.gpio(0);
    alert
        .set_config(GpioDirection::Input, GpioPull::None)
        .map_err(|_| anyhow!("Failed to configure GPIO0 as input"))?;

    // README-SNIPPET-START: alert
    let mut tmp = AlertTmp108::new_with_a0_gnd(i2c, alert);

    tmp.sensor_mut()
        .configure(Config {
            thermostat_mode: Thermostat::Interrupt,
            alert_polarity: Polarity::ActiveLow,
            ..Default::default()
        })
        .await
        .map_err(|_| anyhow!("Failed to configure TMP108"))?;

    tmp.set_temperature_threshold_low(15.0)
        .await
        .map_err(|_| anyhow!("Failed to set low threshold"))?;
    tmp.set_temperature_threshold_high(30.0)
        .await
        .map_err(|_| anyhow!("Failed to set high threshold"))?;

    println!("Waiting for ALERT (warm the sensor above 30 C)...");
    let temperature = tmp
        .wait_for_temperature_threshold()
        .await
        .map_err(|_| anyhow!("wait_for_temperature_threshold failed"))?;
    println!("ALERT serviced! Latest temperature: {temperature:.2} C");
    // README-SNIPPET-END: alert

    Ok(())
}

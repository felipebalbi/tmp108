//! TMP108 one-shot conversion example.
//!
//! # Hardware
//!
//! - Pico de Gallo USB-attached host adapter
//! - TMP108 on the default I2C bus, A0 → GND (address `0x48`)
//!
//! # Cargo features
//!
//! Works with default features (blocking). Building with `--features async`
//! produces the async variant.
//!
//! # Register interactions
//!
//! `one_shot` supervises the whole acquisition rather than just triggering
//! it:
//!
//! 1. Read + rewrite configuration with M = Shutdown
//! 2. Wait the settling delay, re-read configuration, require M = `0b00`
//! 3. Read + rewrite configuration with M = `OneShot`, which triggers
//! 4. Poll configuration until the chip clears M back to `0b00`
//! 5. Read the temperature register
//!
//! Step 4 is what makes the sample fresh: the chip publishes completion by
//! clearing M itself (datasheet SBOS663A §7.4.2), and the temperature
//! register is not read until that is observed. A plain `temperature()`
//! read returns whatever the register already held instead — 0 °C
//! immediately after reset (§7.5.2), or a stale sample on a part that has
//! been sitting in shutdown.
//!
//! Note this sequence performs several configuration reads, and in
//! interrupt thermostat mode every one of those acknowledges FL/FH and
//! releases the ALERT pin (§7.5.3.4). Collect any latched alert first.

use anyhow::{Result, anyhow};
use pico_de_gallo_hal::Hal;

#[cfg(not(feature = "async"))]
fn main() -> Result<()> {
    use tmp108::Tmp108;

    // README-SNIPPET-START: oneshot
    let hal = Hal::new();
    let i2c = hal.i2c();
    let mut delay = hal.delay();

    let mut tmp = Tmp108::new_with_a0_gnd(i2c);

    // 40 ms settling delay: the part defers shutdown until the conversion
    // already in progress finishes, so the write alone is not quiescence.
    let temperature = tmp
        .one_shot(&mut delay, 40)
        .map_err(|_| anyhow!("Failed to acquire a one-shot conversion"))?;
    println!("Temperature: {temperature:.2} C");
    // README-SNIPPET-END: oneshot

    Ok(())
}

#[cfg(feature = "async")]
#[tokio::main]
async fn main() -> Result<()> {
    use tmp108::AsyncTmp108;

    let hal = Hal::new();
    let i2c = hal.i2c();
    let mut delay = hal.delay();

    let mut tmp = AsyncTmp108::new_with_a0_gnd(i2c);

    let temperature = tmp
        .one_shot(&mut delay, 40)
        .await
        .map_err(|_| anyhow!("Failed to acquire a one-shot conversion"))?;
    println!("Temperature: {temperature:.2} C");

    Ok(())
}

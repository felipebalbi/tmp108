//! TMP108 temperature read example.

use anyhow::{Result, anyhow};
use pico_de_gallo_hal::Hal;
use tmp108::Tmp108;

fn main() -> Result<()> {
    let hal = Hal::new();
    let i2c = hal.i2c();

    let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    let temperature = tmp.temperature().map_err(|_| anyhow!("Failed to read temperature"))?;
    println!("Temperature: {:.2} C", temperature);

    Ok(())
}

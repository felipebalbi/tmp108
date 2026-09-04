//! TMP108 blocking temperature read example.

use anyhow::{Result, anyhow};
use pico_de_gallo_hal::Hal;
use tmp108::blocking::Tmp108;

fn main() -> Result<()> {
    let hal = Hal::new();
    let i2c = hal.i2c();

    let mut tmp = Tmp108::new_with_a0_gnd(i2c);
    let temperature = tmp.temperature().map_err(|_| anyhow!("Failed to read temperature"))?;
    println!("Temperature: {temperature:.4} C");

    Ok(())
}

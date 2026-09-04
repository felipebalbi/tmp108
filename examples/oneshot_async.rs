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
    println!("Temperature: {temperature:.4} C");

    Ok(())
}

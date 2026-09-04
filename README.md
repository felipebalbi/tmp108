# TMP108

[![no-std](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/nostd.yml/badge.svg)](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/nostd.yml)
[![check](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/check.yml/badge.svg)](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/check.yml)
[![rolling](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/rolling.yml/badge.svg)](https://github.com/OpenDevicePartnership/tmp108/actions/workflows/rolling.yml)
[![crates.io](https://img.shields.io/crates/v/tmp108.svg)](https://crates.io/crates/tmp108)
[![Documentation](https://docs.rs/tmp108/badge.svg)](https://docs.rs/tmp108)
[![LICENSE](https://img.shields.io/badge/License-MIT-blue)](./LICENSE)

A `#[no_std]` platform-agnostic driver for the
[TMP108](https://www.ti.com/lit/gpn/tmp108) temperature sensor using
the [embedded-hal](https://docs.rs/embedded-hal) traits.

The TMP108 can take one of 4 I2C addresses depending on the state of
the A0 pin, as described in the table below:

| A0  | Addr |
|-----|------|
| GND | 0x48 |
| V+  | 0x49 |
| SDA | 0x4a |
| SCL | 0x4b |

The driver has specific constructors for each of these states, to
ensure that an invalid address is not attempted.

## Usage

```rust
use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
use tmp108::blocking::Tmp108;
use tmp108::{Celsius, Config, ConversionRate, Hysteresis, Polarity, Thermostat};

# fn main() -> Result<(), embedded_hal::i2c::ErrorKind> {
# let expectations = [
#     // read_configuration
#     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
#     // configure -> modify: read, then write
#     Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
#     Transaction::write(0x48, vec![0x01, 0x66, 0xb0]),
#     // set_low_limit(26.5) -> 26.5 * 256 = 0x1a80
#     Transaction::write(0x48, vec![0x02, 0x1a, 0x80]),
#     // set_high_limit(48.0) -> 48 * 256 = 0x3000
#     Transaction::write(0x48, vec![0x03, 0x30, 0x00]),
#     // temperature -> 0x1900 = 6400, 6400 / 16 * 0.0625 = 25.0
#     Transaction::write_read(0x48, vec![0x00], vec![0x19, 0x00]),
# ];
# let i2c = Mock::new(&expectations);
let mut tmp = Tmp108::new_with_a0_gnd(i2c);
// let mut tmp = Tmp108::new_with_a0_vplus(i2c);
// let mut tmp = Tmp108::new_with_a0_sda(i2c);
// let mut tmp = Tmp108::new_with_a0_scl(i2c);
// let mut tmp = Tmp108::new(i2c, A0::Gnd);

let mut config = tmp.read_configuration()?;

config.thermostat_mode = Thermostat::Interrupt;
config.alert_polarity = Polarity::ActiveHigh;
config.conversion_rate = ConversionRate::SixteenHz;
config.hysteresis = Hysteresis::FourC;

tmp.configure(config)?;

// Thresholds are parsed once, into a type the part can actually store.
// Out-of-range values and NaN are rejected here rather than silently
// truncated into a bogus trip point.
let low = Celsius::try_from_degrees(26.5).expect("26.5 C is representable");
let high = Celsius::try_from_degrees(48.0).expect("48 C is representable");

tmp.set_low_limit(low)?;
tmp.set_high_limit(high)?;

let temperature = tmp.temperature()?;
println!("Temperature: {temperature:.2} C");
# let mut i2c = tmp.destroy();
# i2c.done();
# Ok(())
# }
```

The asynchronous driver lives in `tmp108::asynch` behind the `async`
feature and mirrors the blocking API method for method.

## MSRV

Currently, rust `1.94` and up is supported, but some previous versions
may work.

## License

Licensed under the terms of the [MIT license](http://opensource.org/licenses/MIT).

## Contribution

Unless you explicitly state otherwise, any contribution submitted for
inclusion in the work by you shall be licensed under the terms of the
MIT license.

License: MIT


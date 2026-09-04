//! Blocking TMP108 driver.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::inner::{Inner, Mode, THigh, TLow};
use crate::interface::Interface;
use crate::{A0, Celsius, Config};

/// Blocking TMP108 device driver.
#[derive(Debug)]
pub struct Tmp108<I2C: I2c> {
    inner: Inner<Interface<I2C>>,
    addr: u8,
}

impl<I2C: I2c> Tmp108<I2C> {
    /// Create a new TMP108 instance.
    pub fn new(i2c: I2C, a0: A0) -> Self {
        let addr = a0.into();

        Self {
            inner: Inner::new(Interface::new(i2c, addr)),
            addr,
        }
    }

    /// Create a new TMP108 instance with A0 tied to GND, resulting in
    /// an instance responding to address `0x48`.
    pub fn new_with_a0_gnd(i2c: I2C) -> Self {
        Self::new(i2c, A0::Gnd)
    }

    /// Create a new TMP108 instance with A0 tied to V+, resulting in
    /// an instance responding to address `0x49`.
    pub fn new_with_a0_vplus(i2c: I2C) -> Self {
        Self::new(i2c, A0::Vplus)
    }

    /// Create a new TMP108 instance with A0 tied to SDA, resulting in
    /// an instance responding to address `0x4a`.
    pub fn new_with_a0_sda(i2c: I2C) -> Self {
        Self::new(i2c, A0::Sda)
    }

    /// Create a new TMP108 instance with A0 tied to SCL, resulting in
    /// an instance responding to address `0x4b`.
    pub fn new_with_a0_scl(i2c: I2C) -> Self {
        Self::new(i2c, A0::Scl)
    }

    /// Get the current I2C address
    pub fn addr(&self) -> u8 {
        self.addr
    }

    /// Destroy the driver instance, return the I2C bus instance.
    pub fn destroy(self) -> I2C {
        self.inner.free().i2c
    }

    /// Read configuration register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn read_configuration(&mut self) -> Result<Config, I2C::Error> {
        Ok(self.inner.configuration().read()?.into())
    }

    /// Configure device parameters.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn configure(&mut self, config: Config) -> Result<(), I2C::Error> {
        self.inner.configuration().modify(|r| config.apply(r))
    }

    /// Read the temperature sensor
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn temperature(&mut self) -> Result<Celsius, I2C::Error> {
        let raw = self.inner.temperature().read()?;
        Ok(Celsius::from_register(raw.into()))
    }

    /// Configure device for one-shot conversion
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn one_shot(&mut self) -> Result<(), I2C::Error> {
        self.inner.configuration().modify(|r| r.set_m(Mode::OneShot))
    }

    /// Place device in shutdown mode
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn shutdown(&mut self) -> Result<(), I2C::Error> {
        self.inner.configuration().modify(|r| r.set_m(Mode::Shutdown))
    }

    /// Wait for conversion to complete. This method will block for the amount
    /// of time dictated by the CR bits in the configuration register.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn wait_for_temperature<DELAY: DelayNs>(&mut self, delay: &mut DELAY) -> Result<Celsius, I2C::Error> {
        let config = self.read_configuration()?;

        delay.delay_us(config.conversion_rate.delay_us());
        self.temperature()
    }

    /// Read temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn low_limit(&mut self) -> Result<Celsius, I2C::Error> {
        let raw = self.inner.t_low().read()?;
        Ok(Celsius::from_register(raw.into()))
    }

    /// Set temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn set_low_limit(&mut self, limit: Celsius) -> Result<(), I2C::Error> {
        let raw = limit.to_register();
        self.inner.t_low().write(|r| *r = TLow::from(raw))
    }

    /// Read temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn high_limit(&mut self) -> Result<Celsius, I2C::Error> {
        let raw = self.inner.t_high().read()?;
        Ok(Celsius::from_register(raw.into()))
    }

    /// Set temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub fn set_high_limit(&mut self, limit: Celsius) -> Result<(), I2C::Error> {
        let raw = limit.to_register();
        self.inner.t_high().write(|r| *r = THigh::from(raw))
    }
}

#[cfg(feature = "embedded-sensors-hal")]
impl<I2C: I2c> embedded_sensors_hal::sensor::ErrorType for Tmp108<I2C> {
    type Error = crate::Error<I2C::Error>;
}

#[cfg(feature = "embedded-sensors-hal")]
impl<I2C: I2c> embedded_sensors_hal::temperature::TemperatureSensor for Tmp108<I2C> {
    fn temperature(&mut self) -> Result<embedded_sensors_hal::temperature::DegreesCelsius, Self::Error> {
        // Translate at the boundary: the fleet speaks degrees, the part does not.
        self.temperature().map(Celsius::to_degrees).map_err(crate::Error::Bus)
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    use super::*;
    use crate::{Config, ConversionRate, Hysteresis, Polarity, Thermostat};

    #[test]
    fn handle_a0_pin_accordingly() {
        let expectations = vec![];

        let mock = Mock::new(&expectations);
        let tmp = Tmp108::new_with_a0_gnd(mock);
        assert_eq!(tmp.addr(), 0x48);
        let mut mock = tmp.destroy();
        mock.done();

        let mock = Mock::new(&expectations);
        let tmp = Tmp108::new_with_a0_vplus(mock);
        assert_eq!(tmp.addr(), 0x49);
        let mut mock = tmp.destroy();
        mock.done();

        let mock = Mock::new(&expectations);
        let tmp = Tmp108::new_with_a0_sda(mock);
        assert_eq!(tmp.addr(), 0x4a);
        let mut mock = tmp.destroy();
        mock.done();

        let mock = Mock::new(&expectations);
        let tmp = Tmp108::new_with_a0_scl(mock);
        assert_eq!(tmp.addr(), 0x4b);
        let mut mock = tmp.destroy();
        mock.done();
    }

    #[test]
    fn change_configuration() {
        let expectations = vec![
            Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
            Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
            Transaction::write(0x48, vec![0x01, 0x66, 0xb0]),
            Transaction::write_read(0x48, vec![0x01], vec![0x66, 0xb0]),
        ];

        let mock = Mock::new(&expectations);
        let mut tmp108 = Tmp108::new_with_a0_gnd(mock);
        let result = tmp108.read_configuration();
        assert!(result.is_ok());

        let config = result.unwrap();
        assert_eq!(config, Config::default());

        let config = Config {
            thermostat_mode: Thermostat::Interrupt,
            alert_polarity: Polarity::ActiveHigh,
            conversion_rate: ConversionRate::SixteenHz,
            hysteresis: Hysteresis::FourC,
        };

        let result = tmp108.configure(config);
        assert!(result.is_ok());

        let result = tmp108.read_configuration();
        assert!(result.is_ok());

        let new_config = result.unwrap();
        assert_eq!(config, new_config);

        let mut mock = tmp108.destroy();
        mock.done();
    }

    #[test]
    fn read_temperature_default_address() {
        let expectations = vec![
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x7f, 0xf0])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x64, 0x00])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x50, 0x00])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x4b, 0x00])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x32, 0x00])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x19, 0x00])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x40])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0x00, 0x00])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0xff, 0xc0])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0xe7, 0x00])],
            vec![Transaction::write_read(0x48, vec![0x00], vec![0xc9, 0x00])],
        ];
        let temps = [127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

        for (e, t) in expectations.iter().zip(temps.iter()) {
            let mock = Mock::new(e);
            let mut tmp108 = Tmp108::new_with_a0_gnd(mock);
            let result = tmp108.temperature();
            assert!(result.is_ok());

            let temp = result.unwrap();
            assert_eq!(temp, Celsius::try_from_degrees(*t).unwrap());

            let mut mock = tmp108.destroy();
            mock.done();
        }
    }

    #[test]
    fn set_and_read_low_limit() {
        let expectations = vec![
            Transaction::write(0x48, vec![0x02, 0x7f, 0xf0]),
            Transaction::write_read(0x48, vec![0x02], vec![0x7f, 0xf0]),
            Transaction::write(0x48, vec![0x02, 0x64, 0x00]),
            Transaction::write_read(0x48, vec![0x02], vec![0x64, 0x00]),
            Transaction::write(0x48, vec![0x02, 0x50, 0x00]),
            Transaction::write_read(0x48, vec![0x02], vec![0x50, 0x00]),
            Transaction::write(0x48, vec![0x02, 0x4b, 0x00]),
            Transaction::write_read(0x48, vec![0x02], vec![0x4b, 0x00]),
            Transaction::write(0x48, vec![0x02, 0x32, 0x00]),
            Transaction::write_read(0x48, vec![0x02], vec![0x32, 0x00]),
            Transaction::write(0x48, vec![0x02, 0x19, 0x00]),
            Transaction::write_read(0x48, vec![0x02], vec![0x19, 0x00]),
            Transaction::write(0x48, vec![0x02, 0x00, 0x40]),
            Transaction::write_read(0x48, vec![0x02], vec![0x00, 0x40]),
            Transaction::write(0x48, vec![0x02, 0x00, 0x00]),
            Transaction::write_read(0x48, vec![0x02], vec![0x00, 0x00]),
            Transaction::write(0x48, vec![0x02, 0xff, 0xc0]),
            Transaction::write_read(0x48, vec![0x02], vec![0xff, 0xc0]),
            Transaction::write(0x48, vec![0x02, 0xe7, 0x00]),
            Transaction::write_read(0x48, vec![0x02], vec![0xe7, 0x00]),
            Transaction::write(0x48, vec![0x02, 0xc9, 0x00]),
            Transaction::write_read(0x48, vec![0x02], vec![0xc9, 0x00]),
        ];
        let temps = [127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

        let mock = Mock::new(&expectations);
        let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

        for t in &temps {
            let result = tmp108.set_low_limit(Celsius::try_from_degrees(*t).unwrap());
            assert!(result.is_ok());

            let result = tmp108.low_limit();
            assert!(result.is_ok());

            let temp = result.unwrap();
            assert_eq!(temp, Celsius::try_from_degrees(*t).unwrap());
        }

        let mut mock = tmp108.destroy();
        mock.done();
    }

    #[test]
    fn set_and_read_high_limit() {
        let expectations = vec![
            Transaction::write(0x48, vec![0x03, 0x7f, 0xf0]),
            Transaction::write_read(0x48, vec![0x03], vec![0x7f, 0xf0]),
            Transaction::write(0x48, vec![0x03, 0x64, 0x00]),
            Transaction::write_read(0x48, vec![0x03], vec![0x64, 0x00]),
            Transaction::write(0x48, vec![0x03, 0x50, 0x00]),
            Transaction::write_read(0x48, vec![0x03], vec![0x50, 0x00]),
            Transaction::write(0x48, vec![0x03, 0x4b, 0x00]),
            Transaction::write_read(0x48, vec![0x03], vec![0x4b, 0x00]),
            Transaction::write(0x48, vec![0x03, 0x32, 0x00]),
            Transaction::write_read(0x48, vec![0x03], vec![0x32, 0x00]),
            Transaction::write(0x48, vec![0x03, 0x19, 0x00]),
            Transaction::write_read(0x48, vec![0x03], vec![0x19, 0x00]),
            Transaction::write(0x48, vec![0x03, 0x00, 0x40]),
            Transaction::write_read(0x48, vec![0x03], vec![0x00, 0x40]),
            Transaction::write(0x48, vec![0x03, 0x00, 0x00]),
            Transaction::write_read(0x48, vec![0x03], vec![0x00, 0x00]),
            Transaction::write(0x48, vec![0x03, 0xff, 0xc0]),
            Transaction::write_read(0x48, vec![0x03], vec![0xff, 0xc0]),
            Transaction::write(0x48, vec![0x03, 0xe7, 0x00]),
            Transaction::write_read(0x48, vec![0x03], vec![0xe7, 0x00]),
            Transaction::write(0x48, vec![0x03, 0xc9, 0x00]),
            Transaction::write_read(0x48, vec![0x03], vec![0xc9, 0x00]),
        ];
        let temps = [127.9375, 100.0, 80.0, 75.0, 50.0, 25.0, 0.25, 0.0, -0.25, -25.0, -55.0];

        let mock = Mock::new(&expectations);
        let mut tmp108 = Tmp108::new_with_a0_gnd(mock);

        for t in &temps {
            let result = tmp108.set_high_limit(Celsius::try_from_degrees(*t).unwrap());
            assert!(result.is_ok());

            let result = tmp108.high_limit();
            assert!(result.is_ok());

            let temp = result.unwrap();
            assert_eq!(temp, Celsius::try_from_degrees(*t).unwrap());
        }

        let mut mock = tmp108.destroy();
        mock.done();
    }
}

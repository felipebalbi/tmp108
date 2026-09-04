//! Asynchronous TMP108 driver.

use core::future::Future;

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

use crate::inner::{Inner, Mode, THigh, TLow};
use crate::interface::Interface;
use crate::{A0, Config, to_celsius, to_raw};

/// Asynchronous TMP108 device driver.
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

    /// Create a new `AlertTmp108` instance by consuming the original instance.
    #[cfg(feature = "embedded-sensors-hal-async")]
    pub fn into_alert<ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>(
        self,
        alert: ALERT,
    ) -> AlertTmp108<I2C, ALERT> {
        AlertTmp108 { tmp108: self, alert }
    }

    /// Read configuration register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn read_configuration(&mut self) -> Result<Config, I2C::Error> {
        Ok(self.inner.configuration().read_async().await?.into())
    }

    /// Configure device parameters.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn configure(&mut self, config: Config) -> Result<(), I2C::Error> {
        self.inner.configuration().modify_async(|r| config.apply(r)).await
    }

    /// Read the temperature sensor
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn temperature(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.inner.temperature().read_async().await?;
        Ok(to_celsius(i16::from_be_bytes(raw.into())))
    }

    /// Configure device for one-shot conversion
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn one_shot(&mut self) -> Result<(), I2C::Error> {
        self.inner
            .configuration()
            .modify_async(|r| r.set_m(Mode::OneShot))
            .await
    }

    /// Place device in shutdown mode
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn shutdown(&mut self) -> Result<(), I2C::Error> {
        self.inner
            .configuration()
            .modify_async(|r| r.set_m(Mode::Shutdown))
            .await
    }

    /// Initiate continuous conversions
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn continuous<F, Fut>(&mut self, f: F) -> Result<(), I2C::Error>
    where
        F: FnOnce(&mut Self) -> Fut,
        Fut: Future<Output = Result<(), I2C::Error>> + Send,
    {
        self.inner
            .configuration()
            .modify_async(|r| r.set_m(Mode::Continuous))
            .await?;

        f(self).await?;
        self.shutdown().await
    }

    /// Wait for conversion to complete. This method will wait for the amount
    /// of time dictated by the CR bits in the configuration register.
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn wait_for_temperature<DELAY: DelayNs>(&mut self, delay: &mut DELAY) -> Result<f32, I2C::Error> {
        let config = self.read_configuration().await?;

        delay.delay_us(config.conversion_rate.delay_us()).await;
        self.temperature().await
    }

    /// Read temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn low_limit(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.inner.t_low().read_async().await?;
        Ok(to_celsius(i16::from_be_bytes(raw.into())))
    }

    /// Set temperature low limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn set_low_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
        let raw = to_raw(limit).to_be_bytes();
        self.inner.t_low().write_async(|r| *r = TLow::from(raw)).await
    }

    /// Read temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn high_limit(&mut self) -> Result<f32, I2C::Error> {
        let raw = self.inner.t_high().read_async().await?;
        Ok(to_celsius(i16::from_be_bytes(raw.into())))
    }

    /// Set temperature high limit register
    ///
    /// # Errors
    ///
    /// `I2C::Error` when the I2C transaction fails
    pub async fn set_high_limit(&mut self, limit: f32) -> Result<(), I2C::Error> {
        let raw = to_raw(limit).to_be_bytes();
        self.inner.t_high().write_async(|r| *r = THigh::from(raw)).await
    }
}

/// Tmp108 asynchronous device driver (with alert pin)
#[cfg(feature = "embedded-sensors-hal-async")]
pub struct AlertTmp108<I2C: I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin> {
    /// Underlying TMP108 sensor.
    pub tmp108: Tmp108<I2C>,
    alert: ALERT,
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin> AlertTmp108<I2C, ALERT> {
    /// Create a new ALERTTMP108 instance.
    pub fn new(i2c: I2C, a0: A0, alert: ALERT) -> Self {
        let tmp108 = Tmp108::new(i2c, a0);
        Self { tmp108, alert }
    }

    /// Create a new ALERTTMP108 instance with A0 tied to GND, resulting in an
    /// instance responding to address `0x48`.
    pub fn new_with_a0_gnd(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Gnd, alert)
    }

    /// Create a new ALERTTMP108 instance with A0 tied to V+, resulting in an
    /// instance responding to address `0x49`.
    pub fn new_with_a0_vplus(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Vplus, alert)
    }

    /// Create a new ALERTTMP108 instance with A0 tied to SDA, resulting in an
    /// instance responding to address `0x4a`.
    pub fn new_with_a0_sda(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Sda, alert)
    }

    /// Create a new ALERTTMP108 instance with A0 tied to SCL, resulting in an
    /// instance responding to address `0x4b`.
    pub fn new_with_a0_scl(i2c: I2C, alert: ALERT) -> Self {
        Self::new(i2c, A0::Scl, alert)
    }

    /// Destroy the driver instance, return the I2C bus instance and ALERT pin instance.
    pub fn destroy(self) -> (I2C, ALERT) {
        (self.tmp108.destroy(), self.alert)
    }
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: I2c> embedded_sensors_hal::sensor::ErrorType for Tmp108<I2C> {
    type Error = crate::Error<I2C::Error>;
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: I2c> embedded_sensors_hal_async::temperature::TemperatureSensor for Tmp108<I2C> {
    async fn temperature(&mut self) -> Result<embedded_sensors_hal_async::temperature::DegreesCelsius, Self::Error> {
        self.temperature().await.map_err(crate::Error::Bus)
    }
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal::sensor::ErrorType for AlertTmp108<I2C, ALERT>
{
    type Error = crate::Error<I2C::Error>;
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal_async::temperature::TemperatureSensor for AlertTmp108<I2C, ALERT>
{
    async fn temperature(&mut self) -> Result<embedded_sensors_hal_async::temperature::DegreesCelsius, Self::Error> {
        self.tmp108.temperature().await.map_err(crate::Error::Bus)
    }
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: I2c> embedded_sensors_hal_async::temperature::TemperatureThresholdSet for Tmp108<I2C> {
    async fn set_temperature_threshold_low(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        self.set_low_limit(threshold).await.map_err(crate::Error::Bus)
    }

    async fn set_temperature_threshold_high(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        self.set_high_limit(threshold).await.map_err(crate::Error::Bus)
    }
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal_async::temperature::TemperatureThresholdSet for AlertTmp108<I2C, ALERT>
{
    async fn set_temperature_threshold_low(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        self.tmp108.set_low_limit(threshold).await.map_err(crate::Error::Bus)
    }

    async fn set_temperature_threshold_high(
        &mut self,
        threshold: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        self.tmp108.set_high_limit(threshold).await.map_err(crate::Error::Bus)
    }
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal_async::temperature::TemperatureThresholdWait for AlertTmp108<I2C, ALERT>
{
    async fn wait_for_temperature_threshold(
        &mut self,
    ) -> Result<embedded_sensors_hal_async::temperature::DegreesCelsius, Self::Error> {
        let config = self.tmp108.read_configuration().await.map_err(crate::Error::Bus)?;

        match (config.thermostat_mode, config.alert_polarity) {
            // In comparator mode, the ALERT pin remains active even after triggering.
            //
            // If called in a loop, next iteration would return immediately (after reading config
            // again) if temperature remains outside threshold.
            //
            // ALERT pin only resets when temperature falls within the range of (Tlow + HYS) and
            // (Thigh - HYS).
            (crate::Thermostat::Comparator, crate::Polarity::ActiveLow) => {
                self.alert.wait_for_low().await.map_err(|_| crate::Error::Other)?;
            }
            (crate::Thermostat::Comparator, crate::Polarity::ActiveHigh) => {
                self.alert.wait_for_high().await.map_err(|_| crate::Error::Other)?;
            }

            // In interrupt mode, the ALERT pin is immediately reset (by reading config register)
            // after triggering.
            //
            // If called in a loop, next iteration would wait even if temperature remains outside
            // threshold.
            (crate::Thermostat::Interrupt, crate::Polarity::ActiveLow) => {
                self.alert
                    .wait_for_falling_edge()
                    .await
                    .map_err(|_| crate::Error::Other)?;
                let _ = self.tmp108.read_configuration().await.map_err(crate::Error::Bus)?;
            }
            (crate::Thermostat::Interrupt, crate::Polarity::ActiveHigh) => {
                self.alert
                    .wait_for_rising_edge()
                    .await
                    .map_err(|_| crate::Error::Other)?;
                let _ = self.tmp108.read_configuration().await.map_err(crate::Error::Bus)?;
            }
        }

        // Return temperature at time of trigger for caller to determine which threshold was crossed.
        let temperature = self.tmp108.temperature().await.map_err(crate::Error::Bus)?;
        Ok(temperature)
    }
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: I2c> embedded_sensors_hal_async::temperature::TemperatureHysteresis for Tmp108<I2C> {
    async fn set_temperature_threshold_hysteresis(
        &mut self,
        hysteresis: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        // Trait method takes a continuous range of f32 values as argument, but internally driver
        // only accepts 4 discrete values for hysteresis.
        //
        // We ensure only a correct value for hysteresis is passed in, and return error otherwise.
        let hysteresis = if (hysteresis - 0.0).abs() < f32::EPSILON {
            crate::Hysteresis::ZeroC
        } else if (hysteresis - 1.0).abs() < f32::EPSILON {
            crate::Hysteresis::OneC
        } else if (hysteresis - 2.0).abs() < f32::EPSILON {
            crate::Hysteresis::TwoC
        } else if (hysteresis - 4.0).abs() < f32::EPSILON {
            crate::Hysteresis::FourC
        } else {
            return Err(crate::Error::InvalidInput);
        };

        let mut config = self.read_configuration().await.map_err(|_| crate::Error::Other)?;
        config.hysteresis = hysteresis;
        self.configure(config).await.map_err(crate::Error::Bus)
    }
}

#[cfg(feature = "embedded-sensors-hal-async")]
impl<I2C: I2c, ALERT: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin>
    embedded_sensors_hal_async::temperature::TemperatureHysteresis for AlertTmp108<I2C, ALERT>
{
    async fn set_temperature_threshold_hysteresis(
        &mut self,
        hysteresis: embedded_sensors_hal_async::temperature::DegreesCelsius,
    ) -> Result<(), Self::Error> {
        self.tmp108.set_temperature_threshold_hysteresis(hysteresis).await
    }
}

#[cfg(test)]
mod tests {
    use assert_approx_eq::assert_approx_eq;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    use super::*;
    use crate::{Config, ConversionRate, Hysteresis, Polarity, Thermostat};

    #[tokio::test]
    async fn handle_a0_pin_accordingly() {
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

    #[tokio::test]
    async fn change_configuration() {
        let expectations = vec![
            Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
            Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
            Transaction::write(0x48, vec![0x01, 0x66, 0xb0]),
            Transaction::write_read(0x48, vec![0x01], vec![0x66, 0xb0]),
        ];

        let mock = Mock::new(&expectations);
        let mut tmp108 = Tmp108::new_with_a0_gnd(mock);
        let result = tmp108.read_configuration().await;
        assert!(result.is_ok());

        let config = result.unwrap();
        assert_eq!(config, Config::default());

        let config = Config {
            thermostat_mode: Thermostat::Interrupt,
            alert_polarity: Polarity::ActiveHigh,
            conversion_rate: ConversionRate::SixteenHz,
            hysteresis: Hysteresis::FourC,
        };

        let result = tmp108.configure(config).await;
        assert!(result.is_ok());

        let result = tmp108.read_configuration().await;
        assert!(result.is_ok());

        let new_config = result.unwrap();
        assert_eq!(config, new_config);

        let mut mock = tmp108.destroy();
        mock.done();
    }

    #[tokio::test]
    async fn read_temperature_default_address() {
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
            let result = tmp108.temperature().await;
            assert!(result.is_ok());

            let temp = result.unwrap();
            assert_approx_eq!(temp, *t, 1e-4);

            let mut mock = tmp108.destroy();
            mock.done();
        }
    }

    #[tokio::test]
    async fn set_and_read_high_limit() {
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
            let result = tmp108.set_high_limit(*t).await;
            assert!(result.is_ok());

            let result = tmp108.high_limit().await;
            assert!(result.is_ok());

            let temp = result.unwrap();
            assert_approx_eq!(temp, *t, 1e-4);
        }

        let mut mock = tmp108.destroy();
        mock.done();
    }

    #[tokio::test]
    async fn set_and_read_low_limit() {
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
            let result = tmp108.set_low_limit(*t).await;
            assert!(result.is_ok());

            let result = tmp108.low_limit().await;
            assert!(result.is_ok());

            let temp = result.unwrap();
            assert_approx_eq!(temp, *t, 1e-4);
        }

        let mut mock = tmp108.destroy();
        mock.done();
    }

    #[cfg(feature = "embedded-sensors-hal-async")]
    #[tokio::test]
    async fn handle_threshold_alerts_properly() {
        use embedded_hal_mock::eh1::digital;
        use embedded_sensors_hal_async::temperature::{TemperatureThresholdSet, TemperatureThresholdWait};

        // Sensor i2c bus mocks and expectations
        let i2c_expectations = vec![
            Transaction::write_read(0x48, vec![0x01], vec![0x22, 0x10]),
            Transaction::write(0x48, vec![0x01, 0x26, 0x10]),
            Transaction::write(0x48, vec![0x02, 0x19, 0x00]),
            Transaction::write(0x48, vec![0x03, 0x50, 0x00]),
            Transaction::write_read(0x48, vec![0x01], vec![0x26, 0x10]),
            Transaction::write_read(0x48, vec![0x01], vec![0x26, 0x10]),
            Transaction::write_read(0x48, vec![0x00], vec![0x50, 0x00]),
        ];
        let i2c_mock = Mock::new(&i2c_expectations);

        // Threshold alert GPIO pin mocks and expectations
        let pin_expectations = vec![digital::Transaction::wait_for_edge(digital::Edge::Falling)];
        let pin_mock = digital::Mock::new(&pin_expectations);

        // Create a ALERTTMP108 instance and configure it as active-low interrupt mode
        let mut tmp108 = AlertTmp108::new_with_a0_gnd(i2c_mock, pin_mock);

        let cfg = Config {
            thermostat_mode: Thermostat::Interrupt,
            alert_polarity: Polarity::ActiveLow,
            ..Default::default()
        };

        let result = tmp108.tmp108.configure(cfg).await;
        assert!(result.is_ok());

        // Set alert thresholds
        let result = tmp108.set_temperature_threshold_low(25.0).await;
        assert!(result.is_ok());
        let result = tmp108.set_temperature_threshold_high(80.0).await;
        assert!(result.is_ok());

        // Ensure alert pin waits for a falling edge
        let result = tmp108.wait_for_temperature_threshold().await;
        assert!(result.is_ok());

        // Check that recently sampled temperature is returned
        let temp = result.unwrap();
        assert_approx_eq!(temp, 80.0, 1e-4);

        let (mut i2c_mock, mut pin_mock) = tmp108.destroy();
        i2c_mock.done();
        pin_mock.done();
    }
}

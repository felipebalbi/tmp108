//! I2C transport for the TMP108 register map.
//!
//! One type serves both modes. `RegisterInterface` and `AsyncRegisterInterface`
//! are distinct traits, so both impls coexist without a marker type, and
//! `embedded_hal::i2c::ErrorType` is a supertrait of both HAL `I2c` traits, so
//! a single `RegisterInterfaceBase` impl covers both.

use device_driver::{FieldsetMetadata, RegisterInterfaceBase};

/// Number of bytes in every TMP108 register.
const REGISTER_BYTES: usize = 2;

#[derive(Debug)]
pub(crate) struct Interface<I2C> {
    pub(crate) i2c: I2C,
    pub(crate) addr: u8,
}

impl<I2C> Interface<I2C> {
    pub(crate) const fn new(i2c: I2C, addr: u8) -> Self {
        Self { i2c, addr }
    }
}

impl<I2C: embedded_hal::i2c::ErrorType> RegisterInterfaceBase for Interface<I2C> {
    type Error = I2C::Error;
    type AddressType = u8;
}

impl<I2C: embedded_hal::i2c::I2c> device_driver::RegisterInterface for Interface<I2C> {
    fn write_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        // `copy_from_slice` panics on a length mismatch. Every TMP108 register
        // is 16 bits wide, so this holds for every register in `tmp108.ddsl`;
        // the assertion names the assumption rather than leaving it implicit.
        debug_assert_eq!(data.len(), REGISTER_BYTES, "every TMP108 register is 16 bits");

        let mut buf = [0; REGISTER_BYTES + 1];

        buf[0] = address;
        buf[1..].copy_from_slice(data);

        self.i2c.write(self.addr, &buf)
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.addr, &[address], data)
    }
}

#[cfg(feature = "async")]
impl<I2C: embedded_hal_async::i2c::I2c> device_driver::AsyncRegisterInterface for Interface<I2C> {
    async fn write_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        // `copy_from_slice` panics on a length mismatch. Every TMP108 register
        // is 16 bits wide, so this holds for every register in `tmp108.ddsl`;
        // the assertion names the assumption rather than leaving it implicit.
        debug_assert_eq!(data.len(), REGISTER_BYTES, "every TMP108 register is 16 bits");

        let mut buf = [0; REGISTER_BYTES + 1];

        buf[0] = address;
        buf[1..].copy_from_slice(data);

        self.i2c.write(self.addr, &buf).await
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.addr, &[address], data).await
    }
}

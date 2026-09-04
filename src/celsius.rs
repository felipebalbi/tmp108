//! Temperature in the units the TMP108 actually measures.
//!
//! This module is pure: no bus, no `async`, no HAL, no `f32` arithmetic on the
//! path that matters. Every function here is total or explicitly fallible, and
//! the domains are small enough that the tests walk them exhaustively rather
//! than sampling.

/// A temperature the TMP108 can represent.
///
/// Stored as sixteenths of a degree Celsius, which is the part's native
/// resolution: one LSB is 0.0625 °C. The representable range is the span of
/// the sensor's 12-bit two's complement field, `-2048..=2047` sixteenths, or
/// [`Celsius::MIN`] to [`Celsius::MAX`].
///
/// That is 4096 inhabitants, every one of them a temperature the part can
/// actually report or accept as a limit. An `f32` in the same position would
/// admit roughly four billion, including `NaN`, both infinities, and −400 °C.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Celsius(i16);

/// Why a value could not be converted into a [`Celsius`].
///
/// This is the only fallible direction in the module. Bytes arriving from the
/// device are always a valid temperature, so the failures here all come from
/// the human-facing side of the boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum OutOfRange {
    /// The value was `NaN`.
    NotANumber,
    /// The value was below [`Celsius::MIN`].
    TooLow,
    /// The value was above [`Celsius::MAX`].
    TooHigh,
}

impl core::fmt::Display for Celsius {
    /// Renders as degrees Celsius, honouring precision: `{:.2}` works.
    ///
    /// This is the rendering edge the post-parse value is finally allowed to
    /// become a float at.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Display::fmt(&self.to_degrees(), f)
    }
}

impl core::fmt::Display for OutOfRange {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NotANumber => f.write_str("temperature is not a number"),
            Self::TooLow => f.write_str("temperature is below -128 C"),
            Self::TooHigh => f.write_str("temperature is above 127.9375 C"),
        }
    }
}

/// Sixteenths of a degree in the lowest representable temperature.
const MIN_SIXTEENTHS: i16 = -2048;

/// Sixteenths of a degree in the highest representable temperature.
const MAX_SIXTEENTHS: i16 = 2047;

/// Bits the 12-bit temperature field is left-shifted by within the register.
const REGISTER_SHIFT: u32 = 4;

/// Values in sixteenths at or below this round to something below
/// [`Celsius::MIN`].
const LOWEST_ACCEPTED: f32 = -2048.5;

/// Values in sixteenths at or above this round to something above
/// [`Celsius::MAX`].
const HIGHEST_ACCEPTED: f32 = 2047.5;

impl Celsius {
    /// Lowest temperature the part can represent, −128 °C.
    pub const MIN: Self = Self(MIN_SIXTEENTHS);

    /// Highest temperature the part can represent, +127.9375 °C.
    ///
    /// Note this is one LSB short of +128 °C: two's complement affords one
    /// more negative code than positive.
    pub const MAX: Self = Self(MAX_SIXTEENTHS);

    /// Zero degrees.
    pub const ZERO: Self = Self(0);

    /// Build a temperature from sixteenths of a degree.
    ///
    /// # Errors
    ///
    /// [`OutOfRange`] if `sixteenths` is outside `-2048..=2047`.
    pub const fn from_sixteenths(sixteenths: i16) -> Result<Self, OutOfRange> {
        if sixteenths < MIN_SIXTEENTHS {
            Err(OutOfRange::TooLow)
        } else if sixteenths > MAX_SIXTEENTHS {
            Err(OutOfRange::TooHigh)
        } else {
            Ok(Self(sixteenths))
        }
    }

    /// Parse a temperature in degrees Celsius.
    ///
    /// This is the boundary between what a human writes and what the part can
    /// store. Values are rounded to the nearest sixteenth of a degree, half
    /// away from zero.
    ///
    /// # Errors
    ///
    /// [`OutOfRange`] if `degrees` is `NaN`, infinite, or rounds to a value
    /// outside [`Celsius::MIN`]`..=`[`Celsius::MAX`].
    pub fn try_from_degrees(degrees: f32) -> Result<Self, OutOfRange> {
        if degrees.is_nan() {
            return Err(OutOfRange::NotANumber);
        }

        let scaled = degrees * 16.0;

        // Rounding half away from zero then truncating means a value is
        // representable exactly when it lies strictly inside this open
        // interval. Checking `scaled` rather than the rounded value keeps the
        // endpoints, -128.0 and 127.9375, inside the range where they belong.
        if scaled <= LOWEST_ACCEPTED {
            return Err(OutOfRange::TooLow);
        }
        if scaled >= HIGHEST_ACCEPTED {
            return Err(OutOfRange::TooHigh);
        }

        // `f32::round` lives in `std`, and this crate is `no_std`. Adding a
        // half and truncating toward zero rounds half away from zero.
        let rounded = if scaled >= 0.0 { scaled + 0.5 } else { scaled - 0.5 };

        // The bounds above guarantee this lands in `-2048..=2047`.
        #[allow(clippy::cast_possible_truncation)]
        Ok(Self(rounded as i16))
    }

    /// The temperature in sixteenths of a degree.
    #[must_use]
    pub const fn sixteenths(self) -> i16 {
        self.0
    }

    /// The temperature in degrees Celsius.
    ///
    /// Exact: the scale factor is a power of two and the value always fits in
    /// an `f32` mantissa, so nothing is rounded here.
    #[must_use]
    pub fn to_degrees(self) -> f32 {
        f32::from(self.0) * 0.0625
    }

    /// Decode a temperature register.
    ///
    /// Total. The register holds a 12-bit two's complement value left-justified
    /// in 16 bits, so every one of the 65,536 possible bit patterns names a
    /// temperature in range. The four unused low bits are discarded.
    pub(crate) const fn from_register(raw: [u8; 2]) -> Self {
        // Arithmetic shift: sign-extends, and floors rather than truncating
        // toward zero, which matters only for the low bits the part never sets.
        Self(i16::from_be_bytes(raw) >> REGISTER_SHIFT)
    }

    /// Encode into a limit register.
    ///
    /// Total. The invariant on the inner value guarantees the shift cannot
    /// overflow.
    pub(crate) const fn to_register(self) -> [u8; 2] {
        (self.0 << REGISTER_SHIFT).to_be_bytes()
    }
}

#[cfg(test)]
mod tests {
    // Exactness is the property under test. The scale factor is a power of two
    // and every value fits an `f32` mantissa, so these conversions are lossless.
    // Comparing approximately would weaken the assertions, not strengthen them.
    #![allow(clippy::float_cmp)]

    use super::*;

    /// Every temperature the type can hold. 4096 of them.
    fn all() -> impl Iterator<Item = Celsius> {
        (MIN_SIXTEENTHS..=MAX_SIXTEENTHS).map(Celsius)
    }

    #[test]
    fn decoding_a_register_is_total() {
        // All 65,536 bit patterns, not a sample of them.
        for word in 0..=u16::MAX {
            let c = Celsius::from_register(word.to_be_bytes());
            assert!(
                (MIN_SIXTEENTHS..=MAX_SIXTEENTHS).contains(&c.sixteenths()),
                "word {word:#06x} decoded out of range: {c:?}"
            );
        }
    }

    #[test]
    fn every_temperature_roundtrips_through_a_register() {
        for c in all() {
            assert_eq!(Celsius::from_register(c.to_register()), c);
        }
    }

    #[test]
    fn every_temperature_roundtrips_through_sixteenths() {
        for c in all() {
            assert_eq!(Celsius::from_sixteenths(c.sixteenths()), Ok(c));
        }
    }

    #[test]
    fn every_temperature_roundtrips_through_degrees() {
        for c in all() {
            assert_eq!(Celsius::try_from_degrees(c.to_degrees()), Ok(c));
        }
    }

    #[test]
    fn sixteenths_outside_the_range_are_rejected() {
        assert_eq!(Celsius::from_sixteenths(MIN_SIXTEENTHS - 1), Err(OutOfRange::TooLow));
        assert_eq!(Celsius::from_sixteenths(MAX_SIXTEENTHS + 1), Err(OutOfRange::TooHigh));
        assert_eq!(Celsius::from_sixteenths(i16::MIN), Err(OutOfRange::TooLow));
        assert_eq!(Celsius::from_sixteenths(i16::MAX), Err(OutOfRange::TooHigh));
    }

    #[test]
    fn degrees_the_part_cannot_hold_are_rejected() {
        // The values that silently became 0 C, 127.9375 C and -128 C before.
        assert_eq!(Celsius::try_from_degrees(f32::NAN), Err(OutOfRange::NotANumber));
        assert_eq!(Celsius::try_from_degrees(f32::INFINITY), Err(OutOfRange::TooHigh));
        assert_eq!(Celsius::try_from_degrees(f32::NEG_INFINITY), Err(OutOfRange::TooLow));
        assert_eq!(Celsius::try_from_degrees(1000.0), Err(OutOfRange::TooHigh));
        assert_eq!(Celsius::try_from_degrees(-500.0), Err(OutOfRange::TooLow));
        assert_eq!(Celsius::try_from_degrees(128.0), Err(OutOfRange::TooHigh));
    }

    #[test]
    fn the_extremes_are_exactly_representable() {
        assert_eq!(Celsius::MIN.to_degrees(), -128.0);
        assert_eq!(Celsius::MAX.to_degrees(), 127.9375);
        assert_eq!(Celsius::ZERO.to_degrees(), 0.0);

        // One LSB short of +128, not +128. Two's complement is lopsided.
        assert_eq!(Celsius::try_from_degrees(127.9375), Ok(Celsius::MAX));
        assert_eq!(Celsius::try_from_degrees(-128.0), Ok(Celsius::MIN));
    }

    #[test]
    fn rounding_bounds_bracket_the_representable_range() {
        // The accepted interval is open and sits exactly half an LSB outside
        // the representable range at each end.
        assert_eq!(LOWEST_ACCEPTED, f32::from(MIN_SIXTEENTHS) - 0.5);
        assert_eq!(HIGHEST_ACCEPTED, f32::from(MAX_SIXTEENTHS) + 0.5);

        // Just inside rounds back to the endpoint; exactly on the bound does not.
        assert_eq!(Celsius::try_from_degrees(-128.03).unwrap(), Celsius::MIN);
        assert_eq!(Celsius::try_from_degrees(-128.04), Err(OutOfRange::TooLow));
        assert_eq!(Celsius::try_from_degrees(127.96).unwrap(), Celsius::MAX);
        assert_eq!(Celsius::try_from_degrees(127.97), Err(OutOfRange::TooHigh));
    }

    #[test]
    fn degrees_round_to_the_nearest_sixteenth() {
        // 0.0625 C per LSB, so 0.03 rounds down to 0 and 0.04 rounds up to 1.
        assert_eq!(Celsius::try_from_degrees(0.03).unwrap().sixteenths(), 0);
        assert_eq!(Celsius::try_from_degrees(0.04).unwrap().sixteenths(), 1);
        assert_eq!(Celsius::try_from_degrees(-0.03).unwrap().sixteenths(), 0);
        assert_eq!(Celsius::try_from_degrees(-0.04).unwrap().sixteenths(), -1);
    }

    #[test]
    fn known_datasheet_values_decode() {
        // Table 7 of the datasheet, as register words.
        for (word, degrees) in [
            (0x7ff0_u16, 127.9375_f32),
            (0x6400, 100.0),
            (0x5000, 80.0),
            (0x3200, 50.0),
            (0x1900, 25.0),
            (0x0040, 0.25),
            (0x0000, 0.0),
            (0xffc0, -0.25),
            (0xe700, -25.0),
            (0xc900, -55.0),
            (0x8000, -128.0),
        ] {
            let c = Celsius::from_register(word.to_be_bytes());
            assert_eq!(c.to_degrees(), degrees, "word {word:#06x}");
            assert_eq!(c.to_register(), word.to_be_bytes(), "word {word:#06x}");
        }
    }

    #[test]
    fn unused_low_bits_are_discarded_not_truncated_toward_zero() {
        // The part never sets these, but `from_register` is total and must
        // still floor rather than truncate toward zero for negatives.
        assert_eq!(Celsius::from_register(0xffff_u16.to_be_bytes()).sixteenths(), -1);
        assert_eq!(Celsius::from_register(0x0001_u16.to_be_bytes()).sixteenths(), 0);

        // The power-on T_HIGH value, which carries a stray bit 3 that the
        // register cannot store once written.
        assert_eq!(Celsius::from_register(0x7ff8_u16.to_be_bytes()), Celsius::MAX);
    }
}

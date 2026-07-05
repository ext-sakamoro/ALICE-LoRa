//! Spreading factor (`SpreadingFactor` SF7-SF12).

use crate::errors::LoRaError;

use core::fmt;

// Spreading Factor
// ---------------------------------------------------------------------------

/// `LoRa` spreading factor (SF7 through SF12).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SpreadingFactor {
    SF7,
    SF8,
    SF9,
    SF10,
    SF11,
    SF12,
}

impl SpreadingFactor {
    /// Numeric value of the spreading factor.
    #[must_use]
    pub const fn value(self) -> u8 {
        match self {
            Self::SF7 => 7,
            Self::SF8 => 8,
            Self::SF9 => 9,
            Self::SF10 => 10,
            Self::SF11 => 11,
            Self::SF12 => 12,
        }
    }

    /// Number of chips per symbol: 2^SF.
    #[must_use]
    pub const fn chips_per_symbol(self) -> u32 {
        1 << self.value()
    }

    /// Time-on-air for one symbol in seconds given a bandwidth in Hz.
    #[must_use]
    pub fn symbol_time_s(self, bandwidth_hz: f64) -> f64 {
        f64::from(self.chips_per_symbol()) / bandwidth_hz
    }

    /// Receiver sensitivity in dBm (approximate, for 125 kHz BW).
    #[must_use]
    pub const fn sensitivity_dbm(self) -> f64 {
        match self {
            Self::SF7 => -123.0,
            Self::SF8 => -126.0,
            Self::SF9 => -129.0,
            Self::SF10 => -132.0,
            Self::SF11 => -134.5,
            Self::SF12 => -137.0,
        }
    }

    /// Create from a raw u8 value.
    ///
    /// # Errors
    ///
    /// Returns `Err` if the value is not in the range 7..=12.
    pub const fn from_u8(v: u8) -> Result<Self, LoRaError> {
        match v {
            7 => Ok(Self::SF7),
            8 => Ok(Self::SF8),
            9 => Ok(Self::SF9),
            10 => Ok(Self::SF10),
            11 => Ok(Self::SF11),
            12 => Ok(Self::SF12),
            _ => Err(LoRaError::InvalidSpreadingFactor),
        }
    }
}

impl fmt::Display for SpreadingFactor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SF{}", self.value())
    }
}

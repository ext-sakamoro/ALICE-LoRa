//! Frequency plan (`FrequencyPlan`).

use crate::errors::LoRaError;
use crate::spreading_factor::SpreadingFactor;
use core::fmt;

// Frequency Plan
// ---------------------------------------------------------------------------

/// Regional frequency plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrequencyPlan {
    /// EU868 (863-870 MHz)
    Eu868,
    /// US915 (902-928 MHz)
    Us915,
    /// AU915 (915-928 MHz)
    Au915,
    /// AS923 (923 MHz)
    As923,
    /// KR920 (920-923 MHz)
    Kr920,
    /// IN865 (865-867 MHz)
    In865,
}

impl FrequencyPlan {
    /// Default uplink channels in Hz.
    #[must_use]
    pub fn default_channels(&self) -> Vec<u32> {
        match self {
            Self::Eu868 => vec![868_100_000, 868_300_000, 868_500_000],
            Self::Us915 => (0..8).map(|i| 902_300_000 + i * 200_000).collect(),
            Self::Au915 => (0..8).map(|i| 915_200_000 + i * 200_000).collect(),
            Self::As923 => vec![923_200_000, 923_400_000],
            Self::Kr920 => vec![922_100_000, 922_300_000, 922_500_000],
            Self::In865 => vec![865_062_500, 865_402_500, 865_985_000],
        }
    }

    /// Default RX2 frequency in Hz.
    #[must_use]
    pub const fn rx2_frequency(&self) -> u32 {
        match self {
            Self::Eu868 => 869_525_000,
            Self::Us915 => 923_300_000,
            Self::Au915 => 923_300_000,
            Self::As923 => 923_200_000,
            Self::Kr920 => 921_900_000,
            Self::In865 => 866_550_000,
        }
    }

    /// Default RX2 data rate (spreading factor).
    #[must_use]
    pub const fn rx2_default_sf(&self) -> SpreadingFactor {
        match self {
            Self::Eu868 | Self::As923 | Self::Kr920 | Self::In865 => SpreadingFactor::SF12,
            Self::Us915 | Self::Au915 => SpreadingFactor::SF12,
        }
    }

    /// Maximum EIRP in dBm for this plan.
    #[must_use]
    pub const fn max_eirp_dbm(&self) -> u8 {
        match self {
            Self::Eu868 => 16,
            Self::Us915 | Self::Au915 => 30,
            Self::As923 => 16,
            Self::Kr920 => 14,
            Self::In865 => 30,
        }
    }

    /// Maximum duty cycle (as a fraction, e.g. 0.01 = 1%).
    #[must_use]
    pub const fn max_duty_cycle(&self) -> f64 {
        match self {
            Self::Eu868 => 0.01,
            Self::Kr920 => 0.01,
            _ => 1.0, // No duty cycle restriction (FCC uses dwell time instead)
        }
    }

    /// Maximum payload size in bytes for a given SF.
    #[must_use]
    pub const fn max_payload_size(&self, sf: SpreadingFactor) -> u16 {
        match (self, sf) {
            (Self::Eu868, SpreadingFactor::SF7 | SpreadingFactor::SF8) => 222,
            (Self::Eu868, SpreadingFactor::SF9) => 115,
            (Self::Eu868, SpreadingFactor::SF10) => 51,
            (Self::Eu868, SpreadingFactor::SF11 | SpreadingFactor::SF12) => 51,
            (Self::Us915 | Self::Au915, SpreadingFactor::SF7 | SpreadingFactor::SF8) => 242,
            (Self::Us915 | Self::Au915, SpreadingFactor::SF9) => 115,
            (Self::Us915 | Self::Au915, SpreadingFactor::SF10) => 11,
            (Self::Us915 | Self::Au915, _) => 0,
            _ => 51,
        }
    }

    /// Create from a string name.
    ///
    /// # Errors
    ///
    /// Returns `Err` if the plan name is not recognized.
    pub fn from_str_val(s: &str) -> Result<Self, LoRaError> {
        match s.to_ascii_lowercase().as_str() {
            "eu868" => Ok(Self::Eu868),
            "us915" => Ok(Self::Us915),
            "au915" => Ok(Self::Au915),
            "as923" => Ok(Self::As923),
            "kr920" => Ok(Self::Kr920),
            "in865" => Ok(Self::In865),
            _ => Err(LoRaError::InvalidFrequencyPlan),
        }
    }
}

impl fmt::Display for FrequencyPlan {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Eu868 => write!(f, "EU868"),
            Self::Us915 => write!(f, "US915"),
            Self::Au915 => write!(f, "AU915"),
            Self::As923 => write!(f, "AS923"),
            Self::Kr920 => write!(f, "KR920"),
            Self::In865 => write!(f, "IN865"),
        }
    }
}

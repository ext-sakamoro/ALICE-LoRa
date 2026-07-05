//! Coding rate (`CodingRate` 4/5-4/8).

// Coding Rate
// ---------------------------------------------------------------------------

/// `LoRa` forward error correction coding rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CodingRate {
    /// 4/5
    Cr45,
    /// 4/6
    Cr46,
    /// 4/7
    Cr47,
    /// 4/8
    Cr48,
}

impl CodingRate {
    /// Denominator of the coding rate fraction (numerator is always 4).
    #[must_use]
    pub const fn denominator(self) -> u8 {
        match self {
            Self::Cr45 => 5,
            Self::Cr46 => 6,
            Self::Cr47 => 7,
            Self::Cr48 => 8,
        }
    }

    /// Coding rate as a floating-point ratio.
    #[must_use]
    pub fn ratio(self) -> f64 {
        4.0 / f64::from(self.denominator())
    }
}

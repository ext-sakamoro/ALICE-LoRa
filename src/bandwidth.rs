//! Bandwidth (`Bandwidth` 125/250/500 kHz).

use core::fmt;

// Bandwidth
// ---------------------------------------------------------------------------

/// `LoRa` channel bandwidth.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Bandwidth {
    /// 125 kHz
    Bw125,
    /// 250 kHz
    Bw250,
    /// 500 kHz
    Bw500,
}

impl Bandwidth {
    /// Bandwidth in Hz.
    #[must_use]
    pub const fn hz(self) -> u32 {
        match self {
            Self::Bw125 => 125_000,
            Self::Bw250 => 250_000,
            Self::Bw500 => 500_000,
        }
    }
}

impl fmt::Display for Bandwidth {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} kHz", self.hz() / 1000)
    }
}

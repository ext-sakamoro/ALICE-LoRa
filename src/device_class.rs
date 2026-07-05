//! Device class (`DeviceClass` A/B/C).

use crate::errors::LoRaError;
use core::fmt;

// Device Class
// ---------------------------------------------------------------------------

/// `LoRaWAN` device operating class.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DeviceClass {
    /// Class A: baseline, lowest power. Two short receive windows after each uplink.
    A,
    /// Class B: beacon-synchronized receive windows.
    B,
    /// Class C: continuous receive, highest power consumption.
    C,
}

impl DeviceClass {
    /// Create from a character identifier.
    ///
    /// # Errors
    ///
    /// Returns `Err` if the character is not 'A', 'B', or 'C'.
    pub const fn from_char(c: char) -> Result<Self, LoRaError> {
        match c {
            'A' | 'a' => Ok(Self::A),
            'B' | 'b' => Ok(Self::B),
            'C' | 'c' => Ok(Self::C),
            _ => Err(LoRaError::InvalidDeviceClass),
        }
    }

    /// Whether the device has continuous receive capability.
    #[must_use]
    pub const fn continuous_receive(self) -> bool {
        matches!(self, Self::C)
    }

    /// Whether the device uses beacon-synchronized windows.
    #[must_use]
    pub const fn beacon_synchronized(self) -> bool {
        matches!(self, Self::B)
    }

    /// Number of receive windows per uplink cycle.
    #[must_use]
    pub const fn rx_windows(self) -> u8 {
        match self {
            Self::A => 2,
            Self::B => 2, // plus ping slots
            Self::C => 2, // plus continuous
        }
    }
}

impl fmt::Display for DeviceClass {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::A => write!(f, "Class A"),
            Self::B => write!(f, "Class B"),
            Self::C => write!(f, "Class C"),
        }
    }
}

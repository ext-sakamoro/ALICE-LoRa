//! `LoRaError` — LoRaWAN engine error type.

use core::fmt;

// Error
// ---------------------------------------------------------------------------

/// Errors produced by this crate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoRaError {
    InvalidSpreadingFactor,
    InvalidFrameType,
    BufferTooShort,
    InvalidMic,
    InvalidPayloadLength,
    InvalidMacCommand,
    InvalidJoinType,
    InvalidDeviceClass,
    InvalidFrequencyPlan,
    AdrRejected,
}

impl fmt::Display for LoRaError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSpreadingFactor => write!(f, "invalid spreading factor"),
            Self::InvalidFrameType => write!(f, "invalid frame type"),
            Self::BufferTooShort => write!(f, "buffer too short"),
            Self::InvalidMic => write!(f, "invalid MIC"),
            Self::InvalidPayloadLength => write!(f, "invalid payload length"),
            Self::InvalidMacCommand => write!(f, "invalid MAC command"),
            Self::InvalidJoinType => write!(f, "invalid join type"),
            Self::InvalidDeviceClass => write!(f, "invalid device class"),
            Self::InvalidFrequencyPlan => write!(f, "invalid frequency plan"),
            Self::AdrRejected => write!(f, "ADR request rejected"),
        }
    }
}

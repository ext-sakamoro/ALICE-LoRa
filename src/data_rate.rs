//! Data rate table (`DataRate` / `eu868_data_rates` / `us915_data_rates`).

use crate::bandwidth::Bandwidth;
use crate::spreading_factor::SpreadingFactor;

// Data Rate Table
// ---------------------------------------------------------------------------

/// A data rate entry combining SF, BW, and nominal bit rate.
#[derive(Debug, Clone, Copy)]
pub struct DataRate {
    pub index: u8,
    pub spreading_factor: SpreadingFactor,
    pub bandwidth: Bandwidth,
    pub bit_rate_bps: u32,
}

/// Get the EU868 data rate table.
#[must_use]
pub fn eu868_data_rates() -> Vec<DataRate> {
    vec![
        DataRate {
            index: 0,
            spreading_factor: SpreadingFactor::SF12,
            bandwidth: Bandwidth::Bw125,
            bit_rate_bps: 250,
        },
        DataRate {
            index: 1,
            spreading_factor: SpreadingFactor::SF11,
            bandwidth: Bandwidth::Bw125,
            bit_rate_bps: 440,
        },
        DataRate {
            index: 2,
            spreading_factor: SpreadingFactor::SF10,
            bandwidth: Bandwidth::Bw125,
            bit_rate_bps: 980,
        },
        DataRate {
            index: 3,
            spreading_factor: SpreadingFactor::SF9,
            bandwidth: Bandwidth::Bw125,
            bit_rate_bps: 1760,
        },
        DataRate {
            index: 4,
            spreading_factor: SpreadingFactor::SF8,
            bandwidth: Bandwidth::Bw125,
            bit_rate_bps: 3125,
        },
        DataRate {
            index: 5,
            spreading_factor: SpreadingFactor::SF7,
            bandwidth: Bandwidth::Bw125,
            bit_rate_bps: 5470,
        },
        DataRate {
            index: 6,
            spreading_factor: SpreadingFactor::SF7,
            bandwidth: Bandwidth::Bw250,
            bit_rate_bps: 11_000,
        },
    ]
}

/// Get the US915 data rate table (uplink).
#[must_use]
pub fn us915_data_rates() -> Vec<DataRate> {
    vec![
        DataRate {
            index: 0,
            spreading_factor: SpreadingFactor::SF10,
            bandwidth: Bandwidth::Bw125,
            bit_rate_bps: 980,
        },
        DataRate {
            index: 1,
            spreading_factor: SpreadingFactor::SF9,
            bandwidth: Bandwidth::Bw125,
            bit_rate_bps: 1760,
        },
        DataRate {
            index: 2,
            spreading_factor: SpreadingFactor::SF8,
            bandwidth: Bandwidth::Bw125,
            bit_rate_bps: 3125,
        },
        DataRate {
            index: 3,
            spreading_factor: SpreadingFactor::SF7,
            bandwidth: Bandwidth::Bw125,
            bit_rate_bps: 5470,
        },
        DataRate {
            index: 4,
            spreading_factor: SpreadingFactor::SF8,
            bandwidth: Bandwidth::Bw500,
            bit_rate_bps: 12_500,
        },
    ]
}

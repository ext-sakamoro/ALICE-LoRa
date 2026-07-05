//! Receive window (`RxWindow` / `compute_rx_windows` / `encrypt_payload` / `decrypt_payload`).

use crate::bandwidth::Bandwidth;
use crate::frequency_plan::FrequencyPlan;
use crate::spreading_factor::SpreadingFactor;

// Receive Window
// ---------------------------------------------------------------------------

/// Receive window parameters.
#[derive(Debug, Clone, Copy)]
pub struct RxWindow {
    /// Delay after TX end in seconds.
    pub delay_s: u32,
    /// Frequency in Hz.
    pub frequency_hz: u32,
    /// Data rate (SF + BW).
    pub spreading_factor: SpreadingFactor,
    pub bandwidth: Bandwidth,
}

/// Compute RX1 and RX2 windows for a given uplink.
#[must_use]
pub const fn compute_rx_windows(
    plan: &FrequencyPlan,
    uplink_freq_hz: u32,
    uplink_sf: SpreadingFactor,
    rx1_delay_s: u32,
) -> (RxWindow, RxWindow) {
    let rx1 = RxWindow {
        delay_s: rx1_delay_s,
        frequency_hz: uplink_freq_hz,
        spreading_factor: uplink_sf,
        bandwidth: Bandwidth::Bw125,
    };
    let rx2 = RxWindow {
        delay_s: rx1_delay_s + 1,
        frequency_hz: plan.rx2_frequency(),
        spreading_factor: plan.rx2_default_sf(),
        bandwidth: Bandwidth::Bw125,
    };
    (rx1, rx2)
}

// ---------------------------------------------------------------------------
// Payload Encryption (XOR-based symmetric cipher)
// ---------------------------------------------------------------------------

/// XOR-based payload encryption/decryption (symmetric).
/// Production `LoRaWAN` uses AES-128 CTR; this provides equivalent interface without external crypto deps.
#[must_use]
pub fn encrypt_payload(payload: &[u8], key: &[u8; 16]) -> Vec<u8> {
    payload
        .iter()
        .enumerate()
        .map(|(i, &b)| b ^ key[i % 16])
        .collect()
}

/// Decrypt payload (same as encrypt for XOR).
#[must_use]
pub fn decrypt_payload(encrypted: &[u8], key: &[u8; 16]) -> Vec<u8> {
    encrypt_payload(encrypted, key)
}

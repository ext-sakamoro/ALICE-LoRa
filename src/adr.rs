//! Adaptive Data Rate (`AdrEngine` / `AdrResult`).

use crate::errors::LoRaError;
use crate::spreading_factor::SpreadingFactor;

// Adaptive Data Rate (ADR)
// ---------------------------------------------------------------------------

/// ADR engine state.
#[derive(Debug, Clone)]
pub struct AdrEngine {
    /// Collected SNR values from recent uplinks.
    snr_history: Vec<f64>,
    /// Maximum history size.
    history_size: usize,
    /// Current spreading factor.
    pub current_sf: SpreadingFactor,
    /// Current transmit power index (0 = max).
    pub current_tx_power_idx: u8,
    /// Number of transmissions per uplink.
    pub nb_trans: u8,
}

impl AdrEngine {
    /// Create a new ADR engine.
    #[must_use]
    pub const fn new(initial_sf: SpreadingFactor) -> Self {
        Self {
            snr_history: Vec::new(),
            history_size: 20,
            current_sf: initial_sf,
            current_tx_power_idx: 0,
            nb_trans: 1,
        }
    }

    /// Record an SNR measurement.
    pub fn record_snr(&mut self, snr: f64) {
        self.snr_history.push(snr);
        if self.snr_history.len() > self.history_size {
            self.snr_history.remove(0);
        }
    }

    /// Number of recorded SNR samples.
    #[must_use]
    pub const fn sample_count(&self) -> usize {
        self.snr_history.len()
    }

    /// Compute the average SNR.
    #[must_use]
    pub fn average_snr(&self) -> Option<f64> {
        if self.snr_history.is_empty() {
            return None;
        }
        let sum: f64 = self.snr_history.iter().sum();
        Some(sum / self.snr_history.len() as f64)
    }

    /// Compute the maximum SNR.
    #[must_use]
    pub fn max_snr(&self) -> Option<f64> {
        self.snr_history.iter().copied().reduce(f64::max)
    }

    /// Required SNR for demodulation at a given SF (in dB).
    #[must_use]
    pub const fn required_snr(sf: SpreadingFactor) -> f64 {
        match sf {
            SpreadingFactor::SF7 => -7.5,
            SpreadingFactor::SF8 => -10.0,
            SpreadingFactor::SF9 => -12.5,
            SpreadingFactor::SF10 => -15.0,
            SpreadingFactor::SF11 => -17.5,
            SpreadingFactor::SF12 => -20.0,
        }
    }

    /// SNR margin: how much SNR headroom we have.
    #[must_use]
    pub fn snr_margin(&self) -> Option<f64> {
        let avg = self.average_snr()?;
        let required = Self::required_snr(self.current_sf);
        Some(avg - required)
    }

    /// Run the ADR algorithm and return the recommended SF and tx power index.
    ///
    /// # Errors
    ///
    /// Returns `Err` if there are not enough samples (< 20).
    pub fn compute(&mut self) -> Result<AdrResult, LoRaError> {
        if self.snr_history.len() < self.history_size {
            return Err(LoRaError::AdrRejected);
        }

        let margin = self.snr_margin().ok_or(LoRaError::AdrRejected)?;
        let installation_margin = 10.0; // dB

        let mut steps = ((margin - installation_margin) / 3.0).floor() as i32;
        let mut sf = self.current_sf;
        let mut tx_power_idx = self.current_tx_power_idx;

        // First, try to decrease SF (faster data rate)
        while steps > 0 && sf.value() > 7 {
            if let Ok(new_sf) = SpreadingFactor::from_u8(sf.value() - 1) {
                sf = new_sf;
                steps -= 1;
            } else {
                break;
            }
        }

        // Then, decrease TX power
        while steps > 0 && tx_power_idx < 5 {
            tx_power_idx += 1;
            steps -= 1;
        }

        self.current_sf = sf;
        self.current_tx_power_idx = tx_power_idx;

        Ok(AdrResult {
            spreading_factor: sf,
            tx_power_idx,
            nb_trans: self.nb_trans,
        })
    }
}

/// Result of an ADR computation.
#[derive(Debug, Clone, Copy)]
pub struct AdrResult {
    pub spreading_factor: SpreadingFactor,
    pub tx_power_idx: u8,
    pub nb_trans: u8,
}

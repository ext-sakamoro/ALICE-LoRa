//! Duty cycle manager (`DutyCycleManager`).

// Duty Cycle Manager
// ---------------------------------------------------------------------------

/// Tracks duty cycle usage per sub-band.
#[derive(Debug, Clone)]
pub struct DutyCycleManager {
    /// Time-on-air accumulator per sub-band (in milliseconds).
    sub_band_toa_ms: Vec<u64>,
    /// Window duration for duty cycle calculation (in milliseconds).
    window_ms: u64,
    /// Maximum duty cycle fraction per sub-band.
    max_duty_cycle: f64,
}

impl DutyCycleManager {
    /// Create a new duty cycle manager.
    #[must_use]
    pub fn new(num_sub_bands: usize, max_duty_cycle: f64) -> Self {
        Self {
            sub_band_toa_ms: vec![0; num_sub_bands],
            window_ms: 3_600_000, // 1 hour
            max_duty_cycle,
        }
    }

    /// Record a transmission on a sub-band.
    pub fn record_tx(&mut self, sub_band: usize, toa_ms: u64) {
        if sub_band < self.sub_band_toa_ms.len() {
            self.sub_band_toa_ms[sub_band] += toa_ms;
        }
    }

    /// Check if a sub-band can transmit.
    #[must_use]
    pub fn can_transmit(&self, sub_band: usize) -> bool {
        if sub_band >= self.sub_band_toa_ms.len() {
            return false;
        }
        let used = self.sub_band_toa_ms[sub_band] as f64;
        let max_toa = self.window_ms as f64 * self.max_duty_cycle;
        used < max_toa
    }

    /// Remaining time-on-air budget in ms for a sub-band.
    #[must_use]
    pub fn remaining_ms(&self, sub_band: usize) -> u64 {
        if sub_band >= self.sub_band_toa_ms.len() {
            return 0;
        }
        let max_toa = (self.window_ms as f64 * self.max_duty_cycle) as u64;
        max_toa.saturating_sub(self.sub_band_toa_ms[sub_band])
    }

    /// Reset all accumulators (new window).
    pub fn reset(&mut self) {
        for v in &mut self.sub_band_toa_ms {
            *v = 0;
        }
    }
}

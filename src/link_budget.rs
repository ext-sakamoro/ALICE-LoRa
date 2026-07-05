//! Link budget calculation (`LinkBudget`).

use crate::spreading_factor::SpreadingFactor;

// Link Budget Calculation
// ---------------------------------------------------------------------------

/// Link budget calculation parameters and results.
#[derive(Debug, Clone, Copy)]
pub struct LinkBudget {
    /// Transmit power in dBm.
    pub tx_power_dbm: f64,
    /// Transmit antenna gain in dBi.
    pub tx_antenna_gain_dbi: f64,
    /// Receive antenna gain in dBi.
    pub rx_antenna_gain_dbi: f64,
    /// Cable/connector losses in dB.
    pub cable_loss_db: f64,
    /// Receiver sensitivity in dBm.
    pub rx_sensitivity_dbm: f64,
    /// Fade margin in dB.
    pub fade_margin_db: f64,
}

impl LinkBudget {
    /// Create a default link budget for the given spreading factor.
    #[must_use]
    pub const fn for_sf(sf: SpreadingFactor) -> Self {
        Self {
            tx_power_dbm: 14.0,
            tx_antenna_gain_dbi: 2.15,
            rx_antenna_gain_dbi: 6.0,
            cable_loss_db: 2.0,
            rx_sensitivity_dbm: sf.sensitivity_dbm(),
            fade_margin_db: 10.0,
        }
    }

    /// Maximum allowable path loss in dB.
    #[must_use]
    pub fn max_path_loss_db(&self) -> f64 {
        self.tx_power_dbm + self.tx_antenna_gain_dbi + self.rx_antenna_gain_dbi
            - self.cable_loss_db
            - self.rx_sensitivity_dbm
            - self.fade_margin_db
    }

    /// Estimated maximum range in km (free-space path loss model at 868 MHz).
    #[must_use]
    pub fn max_range_km(&self) -> f64 {
        self.max_range_km_at_freq(868.0)
    }

    /// Estimated maximum range in km at a given frequency in MHz.
    #[must_use]
    pub fn max_range_km_at_freq(&self, freq_mhz: f64) -> f64 {
        let pl = self.max_path_loss_db();
        // Free-space path loss: PL = 20*log10(d) + 20*log10(f) + 32.44
        // => d = 10^((PL - 20*log10(f) - 32.44) / 20)
        let exponent = (20.0f64.mul_add(-freq_mhz.log10(), pl) - 32.44) / 20.0;
        10.0_f64.powf(exponent)
    }

    /// Received signal strength in dBm at a given distance in km and frequency in MHz.
    #[must_use]
    pub fn rssi_at_distance(&self, distance_km: f64, freq_mhz: f64) -> f64 {
        let fspl = 20.0f64.mul_add(distance_km.log10(), 20.0 * freq_mhz.log10()) + 32.44;
        self.tx_power_dbm + self.tx_antenna_gain_dbi + self.rx_antenna_gain_dbi
            - self.cable_loss_db
            - fspl
    }

    /// Whether a link is viable at the given distance and frequency.
    #[must_use]
    pub fn is_viable(&self, distance_km: f64, freq_mhz: f64) -> bool {
        let rssi = self.rssi_at_distance(distance_km, freq_mhz);
        rssi >= self.rx_sensitivity_dbm + self.fade_margin_db
    }
}

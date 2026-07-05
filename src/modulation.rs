//! Chirp Spread Spectrum modulation (`ChirpConfig`).

use crate::bandwidth::Bandwidth;
use crate::coding_rate::CodingRate;
use crate::spreading_factor::SpreadingFactor;

// Chirp Spread Spectrum Modulation
// ---------------------------------------------------------------------------

/// Parameters for chirp spread spectrum (CSS) modulation.
#[derive(Debug, Clone, Copy)]
pub struct ChirpConfig {
    pub spreading_factor: SpreadingFactor,
    pub bandwidth: Bandwidth,
    pub coding_rate: CodingRate,
    /// Preamble length in symbols.
    pub preamble_length: u16,
    /// Whether the payload CRC is enabled.
    pub crc_enabled: bool,
    /// Whether the low data rate optimization is enabled.
    pub low_data_rate_optimize: bool,
    /// Explicit header mode (false = implicit).
    pub explicit_header: bool,
}

impl Default for ChirpConfig {
    fn default() -> Self {
        Self {
            spreading_factor: SpreadingFactor::SF7,
            bandwidth: Bandwidth::Bw125,
            coding_rate: CodingRate::Cr45,
            preamble_length: 8,
            crc_enabled: true,
            low_data_rate_optimize: false,
            explicit_header: true,
        }
    }
}

impl ChirpConfig {
    /// Bit rate in bits per second.
    #[must_use]
    pub fn bit_rate_bps(&self) -> f64 {
        let sf = f64::from(self.spreading_factor.value());
        let bw = f64::from(self.bandwidth.hz());
        let cr = self.coding_rate.ratio();
        sf * cr * bw / f64::from(self.spreading_factor.chips_per_symbol())
    }

    /// Symbol duration in seconds.
    #[must_use]
    pub fn symbol_duration_s(&self) -> f64 {
        self.spreading_factor
            .symbol_time_s(f64::from(self.bandwidth.hz()))
    }

    /// Compute the time-on-air in seconds for a given payload length in bytes.
    #[must_use]
    pub fn time_on_air_s(&self, payload_bytes: u16) -> f64 {
        let sf = f64::from(self.spreading_factor.value());
        let t_sym = self.symbol_duration_s();
        let t_preamble = (f64::from(self.preamble_length) + 4.25) * t_sym;

        let de = if self.low_data_rate_optimize {
            1.0
        } else {
            0.0
        };
        let ih = if self.explicit_header { 0.0 } else { 1.0 };
        let crc_val = if self.crc_enabled { 1.0 } else { 0.0 };

        let numerator = 20.0f64.mul_add(
            -ih,
            16.0f64.mul_add(
                crc_val,
                8.0f64.mul_add(f64::from(payload_bytes), -(4.0 * sf)) + 28.0,
            ),
        );
        let denominator = 4.0 * 2.0f64.mul_add(-de, sf);

        let n_payload = if numerator > 0.0 {
            (numerator / denominator)
                .ceil()
                .mul_add(f64::from(self.coding_rate.denominator()), 8.0)
        } else {
            8.0
        };

        t_preamble + n_payload * t_sym
    }

    /// Check if low data rate optimization should be enabled.
    /// It is required when symbol duration exceeds 16 ms.
    #[must_use]
    pub fn requires_low_data_rate_optimize(&self) -> bool {
        self.symbol_duration_s() > 0.016
    }
}

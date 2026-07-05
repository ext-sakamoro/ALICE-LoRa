//! Join procedure (`JoinType` / `JoinRequest` / `JoinAccept` / `AbpSession`).

use crate::errors::LoRaError;

// Join Procedure
// ---------------------------------------------------------------------------

/// `LoRaWAN` join procedure type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JoinType {
    /// Over-The-Air Activation.
    Otaa,
    /// Activation By Personalization.
    Abp,
}

impl JoinType {
    /// Create from a string representation.
    ///
    /// # Errors
    ///
    /// Returns `Err` if the string is not recognized.
    pub fn from_str_val(s: &str) -> Result<Self, LoRaError> {
        match s.to_ascii_lowercase().as_str() {
            "otaa" => Ok(Self::Otaa),
            "abp" => Ok(Self::Abp),
            _ => Err(LoRaError::InvalidJoinType),
        }
    }

    /// Whether this join type requires a join server.
    #[must_use]
    pub const fn requires_join_server(self) -> bool {
        matches!(self, Self::Otaa)
    }
}

/// OTAA join-request message.
#[derive(Debug, Clone)]
pub struct JoinRequest {
    pub join_eui: [u8; 8],
    pub dev_eui: [u8; 8],
    pub dev_nonce: u16,
}

impl JoinRequest {
    /// Encode the join-request into bytes (MHDR + fields + MIC).
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(23);
        // MHDR: join-request = 0x00
        buf.push(0x00);
        buf.extend_from_slice(&self.join_eui);
        buf.extend_from_slice(&self.dev_eui);
        buf.push((self.dev_nonce & 0xFF) as u8);
        buf.push(((self.dev_nonce >> 8) & 0xFF) as u8);
        // MIC (4 bytes, zeroed — caller should overwrite via compute_mic)
        buf.extend_from_slice(&[0u8; 4]);
        buf
    }

    /// Decode from raw bytes.
    ///
    /// # Errors
    ///
    /// Returns `Err` if the buffer is too short.
    pub fn decode(data: &[u8]) -> Result<Self, LoRaError> {
        // Expect at least 1 (MHDR) + 8 + 8 + 2 + 4 (MIC) = 23
        if data.len() < 23 {
            return Err(LoRaError::BufferTooShort);
        }
        let mut join_eui = [0u8; 8];
        join_eui.copy_from_slice(&data[1..9]);
        let mut dev_eui = [0u8; 8];
        dev_eui.copy_from_slice(&data[9..17]);
        let dev_nonce = u16::from(data[17]) | (u16::from(data[18]) << 8);
        Ok(Self {
            join_eui,
            dev_eui,
            dev_nonce,
        })
    }
}

/// OTAA join-accept message.
#[derive(Debug, Clone)]
pub struct JoinAccept {
    pub join_nonce: u32, // 24-bit
    pub net_id: u32,     // 24-bit
    pub dev_addr: u32,
    pub dl_settings: u8,
    pub rx_delay: u8,
}

impl JoinAccept {
    /// Encode the join-accept into bytes.
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(17);
        // MHDR: join-accept = 0x20
        buf.push(0x20);
        // JoinNonce (3 bytes, little-endian)
        buf.push((self.join_nonce & 0xFF) as u8);
        buf.push(((self.join_nonce >> 8) & 0xFF) as u8);
        buf.push(((self.join_nonce >> 16) & 0xFF) as u8);
        // NetID (3 bytes)
        buf.push((self.net_id & 0xFF) as u8);
        buf.push(((self.net_id >> 8) & 0xFF) as u8);
        buf.push(((self.net_id >> 16) & 0xFF) as u8);
        // DevAddr (4 bytes)
        buf.extend_from_slice(&self.dev_addr.to_le_bytes());
        // DLSettings
        buf.push(self.dl_settings);
        // RxDelay
        buf.push(self.rx_delay);
        // MIC (4 bytes, zeroed — caller should overwrite via compute_mic)
        buf.extend_from_slice(&[0u8; 4]);
        buf
    }

    /// Decode from raw bytes.
    ///
    /// # Errors
    ///
    /// Returns `Err` if the buffer is too short.
    pub fn decode(data: &[u8]) -> Result<Self, LoRaError> {
        if data.len() < 17 {
            return Err(LoRaError::BufferTooShort);
        }
        let join_nonce =
            u32::from(data[1]) | (u32::from(data[2]) << 8) | (u32::from(data[3]) << 16);
        let net_id = u32::from(data[4]) | (u32::from(data[5]) << 8) | (u32::from(data[6]) << 16);
        let dev_addr = u32::from_le_bytes([data[7], data[8], data[9], data[10]]);
        let dl_settings = data[11];
        let rx_delay = data[12];
        Ok(Self {
            join_nonce,
            net_id,
            dev_addr,
            dl_settings,
            rx_delay,
        })
    }
}

/// ABP session parameters.
#[derive(Debug, Clone)]
pub struct AbpSession {
    pub dev_addr: u32,
    pub nwk_s_key: [u8; 16],
    pub app_s_key: [u8; 16],
    pub f_cnt_up: u32,
    pub f_cnt_down: u32,
}

impl AbpSession {
    /// Create a new ABP session.
    #[must_use]
    pub const fn new(dev_addr: u32, nwk_s_key: [u8; 16], app_s_key: [u8; 16]) -> Self {
        Self {
            dev_addr,
            nwk_s_key,
            app_s_key,
            f_cnt_up: 0,
            f_cnt_down: 0,
        }
    }

    /// Increment the uplink frame counter and return the new value.
    pub const fn next_f_cnt_up(&mut self) -> u32 {
        self.f_cnt_up += 1;
        self.f_cnt_up
    }

    /// Increment the downlink frame counter and return the new value.
    pub const fn next_f_cnt_down(&mut self) -> u32 {
        self.f_cnt_down += 1;
        self.f_cnt_down
    }
}

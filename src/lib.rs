#![warn(clippy::all, clippy::pedantic, clippy::nursery)]
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_sign_loss)]
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::similar_names)]
#![allow(clippy::match_same_arms)]
#![allow(clippy::struct_excessive_bools)]

//! ALICE-LoRa: `LoRaWAN` protocol implementation in pure Rust.
//!
//! Provides chirp spread spectrum modulation, spreading factors (SF7-SF12),
//! adaptive data rate (ADR), OTAA/ABP join procedures, MAC commands,
//! frame encoding/decoding, device classes (A/B/C), frequency plans,
//! and link budget calculation.

use core::fmt;

// ---------------------------------------------------------------------------
// Spreading Factor
// ---------------------------------------------------------------------------

/// `LoRa` spreading factor (SF7 through SF12).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SpreadingFactor {
    SF7,
    SF8,
    SF9,
    SF10,
    SF11,
    SF12,
}

impl SpreadingFactor {
    /// Numeric value of the spreading factor.
    #[must_use]
    pub const fn value(self) -> u8 {
        match self {
            Self::SF7 => 7,
            Self::SF8 => 8,
            Self::SF9 => 9,
            Self::SF10 => 10,
            Self::SF11 => 11,
            Self::SF12 => 12,
        }
    }

    /// Number of chips per symbol: 2^SF.
    #[must_use]
    pub const fn chips_per_symbol(self) -> u32 {
        1 << self.value()
    }

    /// Time-on-air for one symbol in seconds given a bandwidth in Hz.
    #[must_use]
    pub fn symbol_time_s(self, bandwidth_hz: f64) -> f64 {
        f64::from(self.chips_per_symbol()) / bandwidth_hz
    }

    /// Receiver sensitivity in dBm (approximate, for 125 kHz BW).
    #[must_use]
    pub const fn sensitivity_dbm(self) -> f64 {
        match self {
            Self::SF7 => -123.0,
            Self::SF8 => -126.0,
            Self::SF9 => -129.0,
            Self::SF10 => -132.0,
            Self::SF11 => -134.5,
            Self::SF12 => -137.0,
        }
    }

    /// Create from a raw u8 value.
    ///
    /// # Errors
    ///
    /// Returns `Err` if the value is not in the range 7..=12.
    pub const fn from_u8(v: u8) -> Result<Self, LoRaError> {
        match v {
            7 => Ok(Self::SF7),
            8 => Ok(Self::SF8),
            9 => Ok(Self::SF9),
            10 => Ok(Self::SF10),
            11 => Ok(Self::SF11),
            12 => Ok(Self::SF12),
            _ => Err(LoRaError::InvalidSpreadingFactor),
        }
    }
}

impl fmt::Display for SpreadingFactor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SF{}", self.value())
    }
}

// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Coding Rate
// ---------------------------------------------------------------------------

/// `LoRa` forward error correction coding rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CodingRate {
    /// 4/5
    Cr45,
    /// 4/6
    Cr46,
    /// 4/7
    Cr47,
    /// 4/8
    Cr48,
}

impl CodingRate {
    /// Denominator of the coding rate fraction (numerator is always 4).
    #[must_use]
    pub const fn denominator(self) -> u8 {
        match self {
            Self::Cr45 => 5,
            Self::Cr46 => 6,
            Self::Cr47 => 7,
            Self::Cr48 => 8,
        }
    }

    /// Coding rate as a floating-point ratio.
    #[must_use]
    pub fn ratio(self) -> f64 {
        4.0 / f64::from(self.denominator())
    }
}

// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
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
    /// Encode the join-request into bytes (MHDR + fields + MIC placeholder).
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(23);
        // MHDR: join-request = 0x00
        buf.push(0x00);
        buf.extend_from_slice(&self.join_eui);
        buf.extend_from_slice(&self.dev_eui);
        buf.push((self.dev_nonce & 0xFF) as u8);
        buf.push(((self.dev_nonce >> 8) & 0xFF) as u8);
        // MIC placeholder (4 bytes)
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
        // MIC placeholder
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

// ---------------------------------------------------------------------------
// Frame Types & Encoding
// ---------------------------------------------------------------------------

/// `LoRaWAN` frame types (`FType` in MHDR).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameType {
    JoinRequest,
    JoinAccept,
    UnconfirmedDataUp,
    UnconfirmedDataDown,
    ConfirmedDataUp,
    ConfirmedDataDown,
    Proprietary,
}

impl FrameType {
    /// Encode as the 3-bit `MType` value.
    #[must_use]
    pub const fn mtype(self) -> u8 {
        match self {
            Self::JoinRequest => 0b000,
            Self::JoinAccept => 0b001,
            Self::UnconfirmedDataUp => 0b010,
            Self::UnconfirmedDataDown => 0b011,
            Self::ConfirmedDataUp => 0b100,
            Self::ConfirmedDataDown => 0b101,
            Self::Proprietary => 0b111,
        }
    }

    /// Decode from the MHDR byte.
    ///
    /// # Errors
    ///
    /// Returns `Err` for reserved/unknown frame types.
    pub const fn from_mhdr(mhdr: u8) -> Result<Self, LoRaError> {
        match (mhdr >> 5) & 0b111 {
            0b000 => Ok(Self::JoinRequest),
            0b001 => Ok(Self::JoinAccept),
            0b010 => Ok(Self::UnconfirmedDataUp),
            0b011 => Ok(Self::UnconfirmedDataDown),
            0b100 => Ok(Self::ConfirmedDataUp),
            0b101 => Ok(Self::ConfirmedDataDown),
            0b111 => Ok(Self::Proprietary),
            _ => Err(LoRaError::InvalidFrameType),
        }
    }

    /// Build the MHDR byte (`MType` | RFU | Major).
    #[must_use]
    pub const fn to_mhdr(self) -> u8 {
        self.mtype() << 5 // Major = 0 (LoRaWAN R1)
    }

    /// Whether this is an uplink frame.
    #[must_use]
    pub const fn is_uplink(self) -> bool {
        matches!(
            self,
            Self::JoinRequest | Self::UnconfirmedDataUp | Self::ConfirmedDataUp
        )
    }

    /// Whether this is a confirmed frame type.
    #[must_use]
    pub const fn is_confirmed(self) -> bool {
        matches!(self, Self::ConfirmedDataUp | Self::ConfirmedDataDown)
    }
}

/// `LoRaWAN` MAC frame (`PHYPayload`).
#[derive(Debug, Clone)]
pub struct Frame {
    pub frame_type: FrameType,
    pub dev_addr: u32,
    pub f_ctrl: FrameControl,
    pub f_cnt: u16,
    pub f_opts: Vec<u8>,
    pub f_port: Option<u8>,
    pub payload: Vec<u8>,
    pub mic: [u8; 4],
}

/// Frame control field.
#[derive(Debug, Clone, Copy, Default)]
pub struct FrameControl {
    pub adr: bool,
    pub adr_ack_req: bool,
    pub ack: bool,
    pub class_b: bool,
    pub f_opts_len: u8,
}

impl FrameControl {
    /// Encode to a single byte.
    #[must_use]
    pub const fn encode(self) -> u8 {
        let mut v = self.f_opts_len & 0x0F;
        if self.adr {
            v |= 0x80;
        }
        if self.adr_ack_req {
            v |= 0x40;
        }
        if self.ack {
            v |= 0x20;
        }
        if self.class_b {
            v |= 0x10;
        }
        v
    }

    /// Decode from a single byte.
    #[must_use]
    pub const fn decode(b: u8) -> Self {
        Self {
            adr: (b & 0x80) != 0,
            adr_ack_req: (b & 0x40) != 0,
            ack: (b & 0x20) != 0,
            class_b: (b & 0x10) != 0,
            f_opts_len: b & 0x0F,
        }
    }
}

impl Frame {
    /// Encode the frame into bytes.
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::new();
        // MHDR
        buf.push(self.frame_type.to_mhdr());
        // FHDR
        buf.extend_from_slice(&self.dev_addr.to_le_bytes());
        buf.push(self.f_ctrl.encode());
        buf.push((self.f_cnt & 0xFF) as u8);
        buf.push(((self.f_cnt >> 8) & 0xFF) as u8);
        // FOpts
        if !self.f_opts.is_empty() {
            buf.extend_from_slice(&self.f_opts);
        }
        // FPort + FRMPayload
        if let Some(port) = self.f_port {
            buf.push(port);
            buf.extend_from_slice(&self.payload);
        }
        // MIC
        buf.extend_from_slice(&self.mic);
        buf
    }

    /// Decode a frame from raw bytes.
    ///
    /// # Errors
    ///
    /// Returns `Err` if the buffer is too short or the frame type is invalid.
    pub fn decode(data: &[u8]) -> Result<Self, LoRaError> {
        if data.len() < 12 {
            return Err(LoRaError::BufferTooShort);
        }
        let frame_type = FrameType::from_mhdr(data[0])?;
        let dev_addr = u32::from_le_bytes([data[1], data[2], data[3], data[4]]);
        let f_ctrl = FrameControl::decode(data[5]);
        let f_cnt = u16::from(data[6]) | (u16::from(data[7]) << 8);
        let f_opts_end = 8 + usize::from(f_ctrl.f_opts_len);

        if data.len() < f_opts_end + 4 {
            return Err(LoRaError::BufferTooShort);
        }

        let f_opts = data[8..f_opts_end].to_vec();

        let mic_start = data.len() - 4;
        let mut mic = [0u8; 4];
        mic.copy_from_slice(&data[mic_start..]);

        let (f_port, payload) = if f_opts_end < mic_start {
            let port = data[f_opts_end];
            let payload = data[f_opts_end + 1..mic_start].to_vec();
            (Some(port), payload)
        } else {
            (None, Vec::new())
        };

        Ok(Self {
            frame_type,
            dev_addr,
            f_ctrl,
            f_cnt,
            f_opts,
            f_port,
            payload,
            mic,
        })
    }
}

// ---------------------------------------------------------------------------
// MIC (Message Integrity Code) — simplified CMAC-like
// ---------------------------------------------------------------------------

/// Compute a simplified MIC (4-byte integrity code) over the given data and key.
/// This is a non-cryptographic placeholder that demonstrates the MIC concept.
#[must_use]
pub fn compute_mic(data: &[u8], key: &[u8; 16]) -> [u8; 4] {
    let mut hash: u32 = 0x811c_9dc5; // FNV offset basis
    for &b in key {
        hash ^= u32::from(b);
        hash = hash.wrapping_mul(0x0100_0193);
    }
    for &b in data {
        hash ^= u32::from(b);
        hash = hash.wrapping_mul(0x0100_0193);
    }
    hash.to_le_bytes()
}

/// Verify a MIC against computed value.
#[must_use]
pub fn verify_mic(data: &[u8], key: &[u8; 16], expected: &[u8; 4]) -> bool {
    compute_mic(data, key) == *expected
}

// ---------------------------------------------------------------------------
// MAC Commands
// ---------------------------------------------------------------------------

/// `LoRaWAN` MAC command identifiers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MacCommandId {
    LinkCheckReq,
    LinkCheckAns,
    LinkAdrReq,
    LinkAdrAns,
    DutyCycleReq,
    DutyCycleAns,
    RxParamSetupReq,
    RxParamSetupAns,
    DevStatusReq,
    DevStatusAns,
    NewChannelReq,
    NewChannelAns,
    RxTimingSetupReq,
    RxTimingSetupAns,
    DlChannelReq,
    DlChannelAns,
}

impl MacCommandId {
    /// The CID byte value.
    #[must_use]
    pub const fn cid(self) -> u8 {
        match self {
            Self::LinkCheckReq => 0x02,
            Self::LinkCheckAns => 0x02,
            Self::LinkAdrReq => 0x03,
            Self::LinkAdrAns => 0x03,
            Self::DutyCycleReq => 0x04,
            Self::DutyCycleAns => 0x04,
            Self::RxParamSetupReq => 0x05,
            Self::RxParamSetupAns => 0x05,
            Self::DevStatusReq => 0x06,
            Self::DevStatusAns => 0x06,
            Self::NewChannelReq => 0x07,
            Self::NewChannelAns => 0x07,
            Self::RxTimingSetupReq => 0x08,
            Self::RxTimingSetupAns => 0x08,
            Self::DlChannelReq => 0x0A,
            Self::DlChannelAns => 0x0A,
        }
    }

    /// Payload length for this command (excluding CID byte).
    #[must_use]
    pub const fn payload_len(self) -> usize {
        match self {
            Self::LinkCheckReq => 0,
            Self::LinkCheckAns => 2,
            Self::LinkAdrReq => 4,
            Self::LinkAdrAns => 1,
            Self::DutyCycleReq => 1,
            Self::DutyCycleAns => 0,
            Self::RxParamSetupReq => 4,
            Self::RxParamSetupAns => 1,
            Self::DevStatusReq => 0,
            Self::DevStatusAns => 2,
            Self::NewChannelReq => 5,
            Self::NewChannelAns => 1,
            Self::RxTimingSetupReq => 1,
            Self::RxTimingSetupAns => 0,
            Self::DlChannelReq => 4,
            Self::DlChannelAns => 1,
        }
    }
}

/// A parsed MAC command with its payload.
#[derive(Debug, Clone)]
pub struct MacCommand {
    pub cid: u8,
    pub payload: Vec<u8>,
}

impl MacCommand {
    /// Create a `LinkCheckReq` command.
    #[must_use]
    pub const fn link_check_req() -> Self {
        Self {
            cid: MacCommandId::LinkCheckReq.cid(),
            payload: Vec::new(),
        }
    }

    /// Create a `LinkCheckAns` command.
    #[must_use]
    pub fn link_check_ans(margin: u8, gw_cnt: u8) -> Self {
        Self {
            cid: MacCommandId::LinkCheckAns.cid(),
            payload: vec![margin, gw_cnt],
        }
    }

    /// Create a `LinkADRReq` command.
    #[must_use]
    pub fn link_adr_req(data_rate_tx_power: u8, ch_mask: u16, redundancy: u8) -> Self {
        Self {
            cid: MacCommandId::LinkAdrReq.cid(),
            payload: vec![
                data_rate_tx_power,
                (ch_mask & 0xFF) as u8,
                ((ch_mask >> 8) & 0xFF) as u8,
                redundancy,
            ],
        }
    }

    /// Create a `LinkADRAns` command.
    #[must_use]
    pub fn link_adr_ans(status: u8) -> Self {
        Self {
            cid: MacCommandId::LinkAdrAns.cid(),
            payload: vec![status],
        }
    }

    /// Create a `DevStatusReq` command.
    #[must_use]
    pub const fn dev_status_req() -> Self {
        Self {
            cid: MacCommandId::DevStatusReq.cid(),
            payload: Vec::new(),
        }
    }

    /// Create a `DevStatusAns` command.
    #[must_use]
    pub fn dev_status_ans(battery: u8, margin: u8) -> Self {
        Self {
            cid: MacCommandId::DevStatusAns.cid(),
            payload: vec![battery, margin],
        }
    }

    /// Create a `DutyCycleReq` command.
    #[must_use]
    pub fn duty_cycle_req(max_duty_cycle: u8) -> Self {
        Self {
            cid: MacCommandId::DutyCycleReq.cid(),
            payload: vec![max_duty_cycle],
        }
    }

    /// Create a `DutyCycleAns` command.
    #[must_use]
    pub const fn duty_cycle_ans() -> Self {
        Self {
            cid: MacCommandId::DutyCycleAns.cid(),
            payload: Vec::new(),
        }
    }

    /// Create a `RxTimingSetupReq` command.
    #[must_use]
    pub fn rx_timing_setup_req(delay: u8) -> Self {
        Self {
            cid: MacCommandId::RxTimingSetupReq.cid(),
            payload: vec![delay & 0x0F],
        }
    }

    /// Create a `RxTimingSetupAns` command.
    #[must_use]
    pub const fn rx_timing_setup_ans() -> Self {
        Self {
            cid: MacCommandId::RxTimingSetupAns.cid(),
            payload: Vec::new(),
        }
    }

    /// Encode to bytes (CID + payload).
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(1 + self.payload.len());
        buf.push(self.cid);
        buf.extend_from_slice(&self.payload);
        buf
    }

    /// Total byte length of the encoded command.
    #[must_use]
    pub const fn encoded_len(&self) -> usize {
        1 + self.payload.len()
    }
}

/// Parse a sequence of MAC commands from a byte slice.
///
/// # Errors
///
/// Returns `Err` if a command is malformed.
pub fn parse_mac_commands(data: &[u8], uplink: bool) -> Result<Vec<MacCommand>, LoRaError> {
    let mut commands = Vec::new();
    let mut i = 0;
    while i < data.len() {
        let cid = data[i];
        let payload_len = mac_command_payload_len(cid, uplink)?;
        if i + 1 + payload_len > data.len() {
            return Err(LoRaError::BufferTooShort);
        }
        commands.push(MacCommand {
            cid,
            payload: data[i + 1..i + 1 + payload_len].to_vec(),
        });
        i += 1 + payload_len;
    }
    Ok(commands)
}

/// Get the payload length for a MAC command CID.
const fn mac_command_payload_len(cid: u8, uplink: bool) -> Result<usize, LoRaError> {
    match (cid, uplink) {
        (0x02, true) => Ok(0),  // LinkCheckReq
        (0x02, false) => Ok(2), // LinkCheckAns
        (0x03, true) => Ok(1),  // LinkAdrAns
        (0x03, false) => Ok(4), // LinkAdrReq
        (0x04, true) => Ok(0),  // DutyCycleAns
        (0x04, false) => Ok(1), // DutyCycleReq
        (0x05, true) => Ok(1),  // RxParamSetupAns
        (0x05, false) => Ok(4), // RxParamSetupReq
        (0x06, true) => Ok(2),  // DevStatusAns
        (0x06, false) => Ok(0), // DevStatusReq
        (0x07, true) => Ok(1),  // NewChannelAns
        (0x07, false) => Ok(5), // NewChannelReq
        (0x08, true) => Ok(0),  // RxTimingSetupAns
        (0x08, false) => Ok(1), // RxTimingSetupReq
        (0x0A, true) => Ok(1),  // DlChannelAns
        (0x0A, false) => Ok(4), // DlChannelReq
        _ => Err(LoRaError::InvalidMacCommand),
    }
}

// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Frequency Plan
// ---------------------------------------------------------------------------

/// Regional frequency plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrequencyPlan {
    /// EU868 (863-870 MHz)
    Eu868,
    /// US915 (902-928 MHz)
    Us915,
    /// AU915 (915-928 MHz)
    Au915,
    /// AS923 (923 MHz)
    As923,
    /// KR920 (920-923 MHz)
    Kr920,
    /// IN865 (865-867 MHz)
    In865,
}

impl FrequencyPlan {
    /// Default uplink channels in Hz.
    #[must_use]
    pub fn default_channels(&self) -> Vec<u32> {
        match self {
            Self::Eu868 => vec![868_100_000, 868_300_000, 868_500_000],
            Self::Us915 => (0..8).map(|i| 902_300_000 + i * 200_000).collect(),
            Self::Au915 => (0..8).map(|i| 915_200_000 + i * 200_000).collect(),
            Self::As923 => vec![923_200_000, 923_400_000],
            Self::Kr920 => vec![922_100_000, 922_300_000, 922_500_000],
            Self::In865 => vec![865_062_500, 865_402_500, 865_985_000],
        }
    }

    /// Default RX2 frequency in Hz.
    #[must_use]
    pub const fn rx2_frequency(&self) -> u32 {
        match self {
            Self::Eu868 => 869_525_000,
            Self::Us915 => 923_300_000,
            Self::Au915 => 923_300_000,
            Self::As923 => 923_200_000,
            Self::Kr920 => 921_900_000,
            Self::In865 => 866_550_000,
        }
    }

    /// Default RX2 data rate (spreading factor).
    #[must_use]
    pub const fn rx2_default_sf(&self) -> SpreadingFactor {
        match self {
            Self::Eu868 | Self::As923 | Self::Kr920 | Self::In865 => SpreadingFactor::SF12,
            Self::Us915 | Self::Au915 => SpreadingFactor::SF12,
        }
    }

    /// Maximum EIRP in dBm for this plan.
    #[must_use]
    pub const fn max_eirp_dbm(&self) -> u8 {
        match self {
            Self::Eu868 => 16,
            Self::Us915 | Self::Au915 => 30,
            Self::As923 => 16,
            Self::Kr920 => 14,
            Self::In865 => 30,
        }
    }

    /// Maximum duty cycle (as a fraction, e.g. 0.01 = 1%).
    #[must_use]
    pub const fn max_duty_cycle(&self) -> f64 {
        match self {
            Self::Eu868 => 0.01,
            Self::Kr920 => 0.01,
            _ => 1.0, // No duty cycle restriction (FCC uses dwell time instead)
        }
    }

    /// Maximum payload size in bytes for a given SF.
    #[must_use]
    pub const fn max_payload_size(&self, sf: SpreadingFactor) -> u16 {
        match (self, sf) {
            (Self::Eu868, SpreadingFactor::SF7 | SpreadingFactor::SF8) => 222,
            (Self::Eu868, SpreadingFactor::SF9) => 115,
            (Self::Eu868, SpreadingFactor::SF10) => 51,
            (Self::Eu868, SpreadingFactor::SF11 | SpreadingFactor::SF12) => 51,
            (Self::Us915 | Self::Au915, SpreadingFactor::SF7 | SpreadingFactor::SF8) => 242,
            (Self::Us915 | Self::Au915, SpreadingFactor::SF9) => 115,
            (Self::Us915 | Self::Au915, SpreadingFactor::SF10) => 11,
            (Self::Us915 | Self::Au915, _) => 0,
            _ => 51,
        }
    }

    /// Create from a string name.
    ///
    /// # Errors
    ///
    /// Returns `Err` if the plan name is not recognized.
    pub fn from_str_val(s: &str) -> Result<Self, LoRaError> {
        match s.to_ascii_lowercase().as_str() {
            "eu868" => Ok(Self::Eu868),
            "us915" => Ok(Self::Us915),
            "au915" => Ok(Self::Au915),
            "as923" => Ok(Self::As923),
            "kr920" => Ok(Self::Kr920),
            "in865" => Ok(Self::In865),
            _ => Err(LoRaError::InvalidFrequencyPlan),
        }
    }
}

impl fmt::Display for FrequencyPlan {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Eu868 => write!(f, "EU868"),
            Self::Us915 => write!(f, "US915"),
            Self::Au915 => write!(f, "AU915"),
            Self::As923 => write!(f, "AS923"),
            Self::Kr920 => write!(f, "KR920"),
            Self::In865 => write!(f, "IN865"),
        }
    }
}

// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
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
// Payload Encryption (XOR-based placeholder)
// ---------------------------------------------------------------------------

/// XOR-based payload encryption/decryption (symmetric).
/// This is a simplified demonstration; real `LoRaWAN` uses AES-128 CTR.
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

// ---------------------------------------------------------------------------
// Channel Mask
// ---------------------------------------------------------------------------

/// Channel mask for enabling/disabling channels.
#[derive(Debug, Clone)]
pub struct ChannelMask {
    mask: u64,
    max_channels: u8,
}

impl ChannelMask {
    /// Create a new channel mask with all channels enabled.
    #[must_use]
    pub const fn all_enabled(max_channels: u8) -> Self {
        let mask = if max_channels >= 64 {
            u64::MAX
        } else {
            (1u64 << max_channels) - 1
        };
        Self { mask, max_channels }
    }

    /// Create a new channel mask with no channels enabled.
    #[must_use]
    pub const fn none_enabled(max_channels: u8) -> Self {
        Self {
            mask: 0,
            max_channels,
        }
    }

    /// Enable a channel.
    pub const fn enable(&mut self, channel: u8) {
        if channel < self.max_channels {
            self.mask |= 1u64 << channel;
        }
    }

    /// Disable a channel.
    pub const fn disable(&mut self, channel: u8) {
        if channel < self.max_channels {
            self.mask &= !(1u64 << channel);
        }
    }

    /// Check if a channel is enabled.
    #[must_use]
    pub const fn is_enabled(&self, channel: u8) -> bool {
        if channel >= self.max_channels {
            return false;
        }
        (self.mask & (1u64 << channel)) != 0
    }

    /// Count of enabled channels.
    #[must_use]
    pub const fn enabled_count(&self) -> u32 {
        self.mask.count_ones()
    }

    /// Get the raw mask value.
    #[must_use]
    pub const fn raw(&self) -> u64 {
        self.mask
    }

    /// Get the 16-bit chunk for a given group (0-3).
    #[must_use]
    pub const fn chunk16(&self, group: u8) -> u16 {
        ((self.mask >> (group as u64 * 16)) & 0xFFFF) as u16
    }
}

// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Device State Machine
// ---------------------------------------------------------------------------

/// State of a `LoRaWAN` end device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceState {
    Idle,
    Transmitting,
    WaitingRx1,
    WaitingRx2,
    Receiving,
    Processing,
    Sleep,
}

impl DeviceState {
    /// Whether the device is in a receive-capable state.
    #[must_use]
    pub const fn can_receive(self) -> bool {
        matches!(self, Self::WaitingRx1 | Self::WaitingRx2 | Self::Receiving)
    }

    /// Whether the device is idle or sleeping.
    #[must_use]
    pub const fn is_low_power(self) -> bool {
        matches!(self, Self::Idle | Self::Sleep)
    }
}

/// Transition the device state machine for Class A.
#[must_use]
pub const fn class_a_next_state(current: DeviceState, event: DeviceEvent) -> DeviceState {
    match (current, event) {
        (DeviceState::Idle, DeviceEvent::TxRequest) => DeviceState::Transmitting,
        (DeviceState::Transmitting, DeviceEvent::TxComplete) => DeviceState::WaitingRx1,
        (DeviceState::WaitingRx1, DeviceEvent::Rx1Open) => DeviceState::Receiving,
        (DeviceState::WaitingRx1, DeviceEvent::Rx1Timeout) => DeviceState::WaitingRx2,
        (DeviceState::WaitingRx2, DeviceEvent::Rx2Open) => DeviceState::Receiving,
        (DeviceState::WaitingRx2, DeviceEvent::Rx2Timeout) => DeviceState::Idle,
        (DeviceState::Receiving, DeviceEvent::RxSuccess) => DeviceState::Processing,
        (DeviceState::Receiving, DeviceEvent::RxFail) => {
            // If was in RX1, go to WaitingRx2; simplified: go idle
            DeviceState::Idle
        }
        (DeviceState::Processing, DeviceEvent::ProcessComplete) => DeviceState::Idle,
        (DeviceState::Idle, DeviceEvent::GoSleep) => DeviceState::Sleep,
        (DeviceState::Sleep, DeviceEvent::WakeUp) => DeviceState::Idle,
        _ => current,
    }
}

/// Events driving the device state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceEvent {
    TxRequest,
    TxComplete,
    Rx1Open,
    Rx1Timeout,
    Rx2Open,
    Rx2Timeout,
    RxSuccess,
    RxFail,
    ProcessComplete,
    GoSleep,
    WakeUp,
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // === Spreading Factor ===

    #[test]
    fn sf_values() {
        assert_eq!(SpreadingFactor::SF7.value(), 7);
        assert_eq!(SpreadingFactor::SF12.value(), 12);
    }

    #[test]
    fn sf_chips_per_symbol() {
        assert_eq!(SpreadingFactor::SF7.chips_per_symbol(), 128);
        assert_eq!(SpreadingFactor::SF12.chips_per_symbol(), 4096);
    }

    #[test]
    fn sf_from_u8_valid() {
        for v in 7..=12 {
            assert!(SpreadingFactor::from_u8(v).is_ok());
        }
    }

    #[test]
    fn sf_from_u8_invalid() {
        assert_eq!(
            SpreadingFactor::from_u8(6),
            Err(LoRaError::InvalidSpreadingFactor)
        );
        assert_eq!(
            SpreadingFactor::from_u8(13),
            Err(LoRaError::InvalidSpreadingFactor)
        );
    }

    #[test]
    fn sf_display() {
        assert_eq!(format!("{}", SpreadingFactor::SF10), "SF10");
    }

    #[test]
    fn sf_symbol_time() {
        let t = SpreadingFactor::SF7.symbol_time_s(125_000.0);
        assert!((t - 0.001_024).abs() < 1e-6);
    }

    #[test]
    fn sf_sensitivity() {
        assert!(SpreadingFactor::SF12.sensitivity_dbm() < SpreadingFactor::SF7.sensitivity_dbm());
    }

    // === Bandwidth ===

    #[test]
    fn bw_hz() {
        assert_eq!(Bandwidth::Bw125.hz(), 125_000);
        assert_eq!(Bandwidth::Bw250.hz(), 250_000);
        assert_eq!(Bandwidth::Bw500.hz(), 500_000);
    }

    #[test]
    fn bw_display() {
        assert_eq!(format!("{}", Bandwidth::Bw125), "125 kHz");
    }

    // === Coding Rate ===

    #[test]
    fn cr_denominator() {
        assert_eq!(CodingRate::Cr45.denominator(), 5);
        assert_eq!(CodingRate::Cr48.denominator(), 8);
    }

    #[test]
    fn cr_ratio() {
        assert!((CodingRate::Cr45.ratio() - 0.8).abs() < 1e-10);
        assert!((CodingRate::Cr48.ratio() - 0.5).abs() < 1e-10);
    }

    // === Chirp Config ===

    #[test]
    fn chirp_default() {
        let c = ChirpConfig::default();
        assert_eq!(c.spreading_factor, SpreadingFactor::SF7);
        assert!(c.crc_enabled);
    }

    #[test]
    fn chirp_bit_rate() {
        let c = ChirpConfig::default();
        let br = c.bit_rate_bps();
        assert!(br > 5000.0);
        assert!(br < 6000.0);
    }

    #[test]
    fn chirp_symbol_duration() {
        let c = ChirpConfig::default();
        let d = c.symbol_duration_s();
        assert!((d - 0.001_024).abs() < 1e-6);
    }

    #[test]
    fn chirp_time_on_air() {
        let c = ChirpConfig::default();
        let toa = c.time_on_air_s(10);
        assert!(toa > 0.0);
        assert!(toa < 1.0);
    }

    #[test]
    fn chirp_toa_increases_with_payload() {
        let c = ChirpConfig::default();
        let toa10 = c.time_on_air_s(10);
        let toa50 = c.time_on_air_s(50);
        assert!(toa50 > toa10);
    }

    #[test]
    fn chirp_toa_increases_with_sf() {
        let c7 = ChirpConfig {
            spreading_factor: SpreadingFactor::SF7,
            ..ChirpConfig::default()
        };
        let c12 = ChirpConfig {
            spreading_factor: SpreadingFactor::SF12,
            ..ChirpConfig::default()
        };
        assert!(c12.time_on_air_s(20) > c7.time_on_air_s(20));
    }

    #[test]
    fn chirp_low_data_rate_optimize() {
        let c = ChirpConfig {
            spreading_factor: SpreadingFactor::SF12,
            bandwidth: Bandwidth::Bw125,
            ..ChirpConfig::default()
        };
        assert!(c.requires_low_data_rate_optimize());
    }

    #[test]
    fn chirp_no_low_data_rate_optimize() {
        let c = ChirpConfig {
            spreading_factor: SpreadingFactor::SF7,
            bandwidth: Bandwidth::Bw500,
            ..ChirpConfig::default()
        };
        assert!(!c.requires_low_data_rate_optimize());
    }

    // === Error ===

    #[test]
    fn error_display() {
        assert_eq!(
            format!("{}", LoRaError::InvalidSpreadingFactor),
            "invalid spreading factor"
        );
        assert_eq!(format!("{}", LoRaError::BufferTooShort), "buffer too short");
    }

    // === Device Class ===

    #[test]
    fn device_class_from_char() {
        assert_eq!(DeviceClass::from_char('A'), Ok(DeviceClass::A));
        assert_eq!(DeviceClass::from_char('b'), Ok(DeviceClass::B));
        assert_eq!(DeviceClass::from_char('C'), Ok(DeviceClass::C));
        assert!(DeviceClass::from_char('D').is_err());
    }

    #[test]
    fn device_class_continuous() {
        assert!(!DeviceClass::A.continuous_receive());
        assert!(DeviceClass::C.continuous_receive());
    }

    #[test]
    fn device_class_beacon() {
        assert!(DeviceClass::B.beacon_synchronized());
        assert!(!DeviceClass::A.beacon_synchronized());
    }

    #[test]
    fn device_class_rx_windows() {
        assert_eq!(DeviceClass::A.rx_windows(), 2);
    }

    #[test]
    fn device_class_display() {
        assert_eq!(format!("{}", DeviceClass::A), "Class A");
    }

    // === Join Type ===

    #[test]
    fn join_type_from_str() {
        assert_eq!(JoinType::from_str_val("otaa"), Ok(JoinType::Otaa));
        assert_eq!(JoinType::from_str_val("ABP"), Ok(JoinType::Abp));
        assert!(JoinType::from_str_val("unknown").is_err());
    }

    #[test]
    fn join_type_requires_server() {
        assert!(JoinType::Otaa.requires_join_server());
        assert!(!JoinType::Abp.requires_join_server());
    }

    // === Join Request ===

    #[test]
    fn join_request_encode_decode() {
        let req = JoinRequest {
            join_eui: [1, 2, 3, 4, 5, 6, 7, 8],
            dev_eui: [9, 10, 11, 12, 13, 14, 15, 16],
            dev_nonce: 0x1234,
        };
        let encoded = req.encode();
        assert_eq!(encoded.len(), 23);
        let decoded = JoinRequest::decode(&encoded).unwrap();
        assert_eq!(decoded.join_eui, req.join_eui);
        assert_eq!(decoded.dev_eui, req.dev_eui);
        assert_eq!(decoded.dev_nonce, req.dev_nonce);
    }

    #[test]
    fn join_request_decode_too_short() {
        assert!(JoinRequest::decode(&[0u8; 10]).is_err());
    }

    // === Join Accept ===

    #[test]
    fn join_accept_encode_decode() {
        let acc = JoinAccept {
            join_nonce: 0x123456 & 0xFFFFFF,
            net_id: 0xABCDEF & 0xFFFFFF,
            dev_addr: 0xDEAD_BEEF,
            dl_settings: 0x03,
            rx_delay: 1,
        };
        let encoded = acc.encode();
        assert_eq!(encoded.len(), 17);
        let decoded = JoinAccept::decode(&encoded).unwrap();
        assert_eq!(decoded.join_nonce, acc.join_nonce);
        assert_eq!(decoded.net_id, acc.net_id);
        assert_eq!(decoded.dev_addr, acc.dev_addr);
        assert_eq!(decoded.dl_settings, acc.dl_settings);
        assert_eq!(decoded.rx_delay, acc.rx_delay);
    }

    #[test]
    fn join_accept_decode_too_short() {
        assert!(JoinAccept::decode(&[0u8; 5]).is_err());
    }

    // === ABP Session ===

    #[test]
    fn abp_session_new() {
        let s = AbpSession::new(0x1234, [0u8; 16], [1u8; 16]);
        assert_eq!(s.dev_addr, 0x1234);
        assert_eq!(s.f_cnt_up, 0);
    }

    #[test]
    fn abp_session_counters() {
        let mut s = AbpSession::new(0, [0u8; 16], [0u8; 16]);
        assert_eq!(s.next_f_cnt_up(), 1);
        assert_eq!(s.next_f_cnt_up(), 2);
        assert_eq!(s.next_f_cnt_down(), 1);
    }

    // === Frame Type ===

    #[test]
    fn frame_type_mhdr_roundtrip() {
        let types = [
            FrameType::JoinRequest,
            FrameType::JoinAccept,
            FrameType::UnconfirmedDataUp,
            FrameType::UnconfirmedDataDown,
            FrameType::ConfirmedDataUp,
            FrameType::ConfirmedDataDown,
            FrameType::Proprietary,
        ];
        for ft in types {
            let mhdr = ft.to_mhdr();
            let decoded = FrameType::from_mhdr(mhdr).unwrap();
            assert_eq!(decoded, ft);
        }
    }

    #[test]
    fn frame_type_invalid() {
        // MType = 0b110 is reserved
        assert!(FrameType::from_mhdr(0b110_00000).is_err());
    }

    #[test]
    fn frame_type_is_uplink() {
        assert!(FrameType::JoinRequest.is_uplink());
        assert!(FrameType::UnconfirmedDataUp.is_uplink());
        assert!(!FrameType::UnconfirmedDataDown.is_uplink());
    }

    #[test]
    fn frame_type_is_confirmed() {
        assert!(FrameType::ConfirmedDataUp.is_confirmed());
        assert!(!FrameType::UnconfirmedDataUp.is_confirmed());
    }

    // === Frame Control ===

    #[test]
    fn fctrl_encode_decode() {
        let fc = FrameControl {
            adr: true,
            adr_ack_req: false,
            ack: true,
            class_b: false,
            f_opts_len: 3,
        };
        let byte = fc.encode();
        let decoded = FrameControl::decode(byte);
        assert_eq!(decoded.adr, true);
        assert_eq!(decoded.ack, true);
        assert_eq!(decoded.f_opts_len, 3);
    }

    #[test]
    fn fctrl_default() {
        let fc = FrameControl::default();
        assert!(!fc.adr);
        assert_eq!(fc.f_opts_len, 0);
    }

    // === Frame ===

    #[test]
    fn frame_encode_decode() {
        let frame = Frame {
            frame_type: FrameType::UnconfirmedDataUp,
            dev_addr: 0x01020304,
            f_ctrl: FrameControl::default(),
            f_cnt: 42,
            f_opts: Vec::new(),
            f_port: Some(1),
            payload: vec![0xAA, 0xBB, 0xCC],
            mic: [0x11, 0x22, 0x33, 0x44],
        };
        let encoded = frame.encode();
        let decoded = Frame::decode(&encoded).unwrap();
        assert_eq!(decoded.frame_type, FrameType::UnconfirmedDataUp);
        assert_eq!(decoded.dev_addr, 0x01020304);
        assert_eq!(decoded.f_cnt, 42);
        assert_eq!(decoded.f_port, Some(1));
        assert_eq!(decoded.payload, vec![0xAA, 0xBB, 0xCC]);
        assert_eq!(decoded.mic, [0x11, 0x22, 0x33, 0x44]);
    }

    #[test]
    fn frame_decode_too_short() {
        assert!(Frame::decode(&[0u8; 5]).is_err());
    }

    #[test]
    fn frame_with_fopts() {
        let frame = Frame {
            frame_type: FrameType::UnconfirmedDataUp,
            dev_addr: 0,
            f_ctrl: FrameControl {
                f_opts_len: 2,
                ..FrameControl::default()
            },
            f_cnt: 0,
            f_opts: vec![0x02, 0x00],
            f_port: Some(1),
            payload: vec![0xFF],
            mic: [0; 4],
        };
        let encoded = frame.encode();
        let decoded = Frame::decode(&encoded).unwrap();
        assert_eq!(decoded.f_opts.len(), 2);
        assert_eq!(decoded.payload, vec![0xFF]);
    }

    #[test]
    fn frame_no_payload() {
        let frame = Frame {
            frame_type: FrameType::UnconfirmedDataUp,
            dev_addr: 0,
            f_ctrl: FrameControl::default(),
            f_cnt: 0,
            f_opts: Vec::new(),
            f_port: None,
            payload: Vec::new(),
            mic: [0; 4],
        };
        let encoded = frame.encode();
        let decoded = Frame::decode(&encoded).unwrap();
        assert!(decoded.payload.is_empty());
    }

    // === MIC ===

    #[test]
    fn mic_compute() {
        let key = [0x01u8; 16];
        let data = b"hello lorawan";
        let mic = compute_mic(data, &key);
        assert_eq!(mic.len(), 4);
    }

    #[test]
    fn mic_verify_correct() {
        let key = [0x42u8; 16];
        let data = b"test data";
        let mic = compute_mic(data, &key);
        assert!(verify_mic(data, &key, &mic));
    }

    #[test]
    fn mic_verify_wrong_key() {
        let key1 = [0x01u8; 16];
        let key2 = [0x02u8; 16];
        let data = b"test";
        let mic = compute_mic(data, &key1);
        assert!(!verify_mic(data, &key2, &mic));
    }

    #[test]
    fn mic_verify_wrong_data() {
        let key = [0x01u8; 16];
        let mic = compute_mic(b"data1", &key);
        assert!(!verify_mic(b"data2", &key, &mic));
    }

    #[test]
    fn mic_deterministic() {
        let key = [0xABu8; 16];
        let data = b"deterministic";
        let m1 = compute_mic(data, &key);
        let m2 = compute_mic(data, &key);
        assert_eq!(m1, m2);
    }

    // === MAC Commands ===

    #[test]
    fn mac_link_check_req() {
        let cmd = MacCommand::link_check_req();
        assert_eq!(cmd.cid, 0x02);
        assert!(cmd.payload.is_empty());
    }

    #[test]
    fn mac_link_check_ans() {
        let cmd = MacCommand::link_check_ans(10, 3);
        assert_eq!(cmd.payload, vec![10, 3]);
    }

    #[test]
    fn mac_link_adr_req() {
        let cmd = MacCommand::link_adr_req(0x53, 0x00FF, 0x01);
        assert_eq!(cmd.cid, 0x03);
        assert_eq!(cmd.payload.len(), 4);
    }

    #[test]
    fn mac_link_adr_ans() {
        let cmd = MacCommand::link_adr_ans(0x07);
        assert_eq!(cmd.payload, vec![0x07]);
    }

    #[test]
    fn mac_dev_status_req() {
        let cmd = MacCommand::dev_status_req();
        assert_eq!(cmd.cid, 0x06);
        assert!(cmd.payload.is_empty());
    }

    #[test]
    fn mac_dev_status_ans() {
        let cmd = MacCommand::dev_status_ans(255, 20);
        assert_eq!(cmd.payload, vec![255, 20]);
    }

    #[test]
    fn mac_duty_cycle_req() {
        let cmd = MacCommand::duty_cycle_req(0x0F);
        assert_eq!(cmd.payload, vec![0x0F]);
    }

    #[test]
    fn mac_duty_cycle_ans() {
        let cmd = MacCommand::duty_cycle_ans();
        assert!(cmd.payload.is_empty());
    }

    #[test]
    fn mac_rx_timing_setup_req() {
        let cmd = MacCommand::rx_timing_setup_req(5);
        assert_eq!(cmd.payload, vec![5]);
    }

    #[test]
    fn mac_rx_timing_setup_ans() {
        let cmd = MacCommand::rx_timing_setup_ans();
        assert!(cmd.payload.is_empty());
    }

    #[test]
    fn mac_command_encode() {
        let cmd = MacCommand::link_check_ans(5, 2);
        let encoded = cmd.encode();
        assert_eq!(encoded, vec![0x02, 5, 2]);
    }

    #[test]
    fn mac_command_encoded_len() {
        let cmd = MacCommand::link_adr_req(0, 0, 0);
        assert_eq!(cmd.encoded_len(), 5);
    }

    #[test]
    fn parse_mac_commands_uplink() {
        // LinkCheckReq (CID=0x02, len=0)
        let data = vec![0x02];
        let cmds = parse_mac_commands(&data, true).unwrap();
        assert_eq!(cmds.len(), 1);
        assert_eq!(cmds[0].cid, 0x02);
    }

    #[test]
    fn parse_mac_commands_downlink() {
        // LinkCheckAns (CID=0x02, len=2)
        let data = vec![0x02, 10, 3];
        let cmds = parse_mac_commands(&data, false).unwrap();
        assert_eq!(cmds.len(), 1);
        assert_eq!(cmds[0].payload, vec![10, 3]);
    }

    #[test]
    fn parse_mac_commands_multiple() {
        // DevStatusReq (CID=0x06, len=0) + DutyCycleReq (CID=0x04, len=1)
        let data = vec![0x06, 0x04, 0x0F];
        let cmds = parse_mac_commands(&data, false).unwrap();
        assert_eq!(cmds.len(), 2);
    }

    #[test]
    fn parse_mac_commands_invalid_cid() {
        let data = vec![0xFF];
        assert!(parse_mac_commands(&data, true).is_err());
    }

    #[test]
    fn parse_mac_commands_truncated() {
        // LinkCheckAns expects 2 payload bytes
        let data = vec![0x02, 10]; // missing second byte
        assert!(parse_mac_commands(&data, false).is_err());
    }

    // === ADR ===

    #[test]
    fn adr_new() {
        let adr = AdrEngine::new(SpreadingFactor::SF12);
        assert_eq!(adr.current_sf, SpreadingFactor::SF12);
        assert_eq!(adr.sample_count(), 0);
    }

    #[test]
    fn adr_record_snr() {
        let mut adr = AdrEngine::new(SpreadingFactor::SF10);
        adr.record_snr(5.0);
        adr.record_snr(10.0);
        assert_eq!(adr.sample_count(), 2);
    }

    #[test]
    fn adr_average_snr() {
        let mut adr = AdrEngine::new(SpreadingFactor::SF10);
        adr.record_snr(5.0);
        adr.record_snr(15.0);
        assert!((adr.average_snr().unwrap() - 10.0).abs() < 1e-10);
    }

    #[test]
    fn adr_average_snr_empty() {
        let adr = AdrEngine::new(SpreadingFactor::SF10);
        assert!(adr.average_snr().is_none());
    }

    #[test]
    fn adr_max_snr() {
        let mut adr = AdrEngine::new(SpreadingFactor::SF10);
        adr.record_snr(-5.0);
        adr.record_snr(10.0);
        adr.record_snr(3.0);
        assert!((adr.max_snr().unwrap() - 10.0).abs() < 1e-10);
    }

    #[test]
    fn adr_required_snr() {
        assert!(
            AdrEngine::required_snr(SpreadingFactor::SF12)
                < AdrEngine::required_snr(SpreadingFactor::SF7)
        );
    }

    #[test]
    fn adr_not_enough_samples() {
        let mut adr = AdrEngine::new(SpreadingFactor::SF10);
        for _ in 0..10 {
            adr.record_snr(10.0);
        }
        assert!(adr.compute().is_err());
    }

    #[test]
    fn adr_compute_with_good_snr() {
        let mut adr = AdrEngine::new(SpreadingFactor::SF12);
        for _ in 0..20 {
            adr.record_snr(20.0); // very good SNR
        }
        let result = adr.compute().unwrap();
        // Should recommend a lower SF
        assert!(result.spreading_factor.value() < 12);
    }

    #[test]
    fn adr_compute_with_poor_snr() {
        let mut adr = AdrEngine::new(SpreadingFactor::SF7);
        for _ in 0..20 {
            adr.record_snr(-10.0); // barely above threshold
        }
        let result = adr.compute().unwrap();
        // Should keep SF7 (can't go lower)
        assert_eq!(result.spreading_factor, SpreadingFactor::SF7);
    }

    #[test]
    fn adr_snr_margin() {
        let mut adr = AdrEngine::new(SpreadingFactor::SF7);
        adr.record_snr(0.0);
        let margin = adr.snr_margin().unwrap();
        // required for SF7 is -7.5, so margin = 0 - (-7.5) = 7.5
        assert!((margin - 7.5).abs() < 1e-10);
    }

    // === Frequency Plan ===

    #[test]
    fn freq_plan_eu868_channels() {
        let channels = FrequencyPlan::Eu868.default_channels();
        assert_eq!(channels.len(), 3);
        assert_eq!(channels[0], 868_100_000);
    }

    #[test]
    fn freq_plan_us915_channels() {
        let channels = FrequencyPlan::Us915.default_channels();
        assert_eq!(channels.len(), 8);
    }

    #[test]
    fn freq_plan_rx2() {
        assert_eq!(FrequencyPlan::Eu868.rx2_frequency(), 869_525_000);
    }

    #[test]
    fn freq_plan_max_eirp() {
        assert_eq!(FrequencyPlan::Eu868.max_eirp_dbm(), 16);
        assert_eq!(FrequencyPlan::Us915.max_eirp_dbm(), 30);
    }

    #[test]
    fn freq_plan_duty_cycle() {
        assert!((FrequencyPlan::Eu868.max_duty_cycle() - 0.01).abs() < 1e-10);
        assert!((FrequencyPlan::Us915.max_duty_cycle() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn freq_plan_max_payload() {
        assert_eq!(
            FrequencyPlan::Eu868.max_payload_size(SpreadingFactor::SF7),
            222
        );
        assert_eq!(
            FrequencyPlan::Eu868.max_payload_size(SpreadingFactor::SF12),
            51
        );
    }

    #[test]
    fn freq_plan_from_str() {
        assert_eq!(
            FrequencyPlan::from_str_val("eu868"),
            Ok(FrequencyPlan::Eu868)
        );
        assert_eq!(
            FrequencyPlan::from_str_val("US915"),
            Ok(FrequencyPlan::Us915)
        );
        assert!(FrequencyPlan::from_str_val("xx").is_err());
    }

    #[test]
    fn freq_plan_display() {
        assert_eq!(format!("{}", FrequencyPlan::Eu868), "EU868");
        assert_eq!(format!("{}", FrequencyPlan::As923), "AS923");
    }

    #[test]
    fn freq_plan_all_have_channels() {
        let plans = [
            FrequencyPlan::Eu868,
            FrequencyPlan::Us915,
            FrequencyPlan::Au915,
            FrequencyPlan::As923,
            FrequencyPlan::Kr920,
            FrequencyPlan::In865,
        ];
        for plan in plans {
            assert!(!plan.default_channels().is_empty());
        }
    }

    // === Link Budget ===

    #[test]
    fn link_budget_path_loss() {
        let lb = LinkBudget::for_sf(SpreadingFactor::SF12);
        let pl = lb.max_path_loss_db();
        assert!(pl > 100.0);
    }

    #[test]
    fn link_budget_range() {
        let lb = LinkBudget::for_sf(SpreadingFactor::SF12);
        let range = lb.max_range_km();
        assert!(range > 1.0);
    }

    #[test]
    fn link_budget_range_sf7_less_than_sf12() {
        let lb7 = LinkBudget::for_sf(SpreadingFactor::SF7);
        let lb12 = LinkBudget::for_sf(SpreadingFactor::SF12);
        assert!(lb12.max_range_km() > lb7.max_range_km());
    }

    #[test]
    fn link_budget_rssi() {
        let lb = LinkBudget::for_sf(SpreadingFactor::SF7);
        let rssi = lb.rssi_at_distance(1.0, 868.0);
        assert!(rssi < lb.tx_power_dbm); // RSSI decreases with distance
    }

    #[test]
    fn link_budget_viable_close() {
        let lb = LinkBudget::for_sf(SpreadingFactor::SF12);
        assert!(lb.is_viable(0.1, 868.0));
    }

    #[test]
    fn link_budget_not_viable_far() {
        let lb = LinkBudget::for_sf(SpreadingFactor::SF7);
        assert!(!lb.is_viable(10000.0, 868.0));
    }

    // === Data Rates ===

    #[test]
    fn eu868_data_rate_table() {
        let rates = eu868_data_rates();
        assert_eq!(rates.len(), 7);
        assert_eq!(rates[0].spreading_factor, SpreadingFactor::SF12);
        assert_eq!(rates[5].spreading_factor, SpreadingFactor::SF7);
    }

    #[test]
    fn us915_data_rate_table() {
        let rates = us915_data_rates();
        assert_eq!(rates.len(), 5);
        assert_eq!(rates[0].spreading_factor, SpreadingFactor::SF10);
    }

    // === Receive Window ===

    #[test]
    fn rx_windows_eu868() {
        let (rx1, rx2) =
            compute_rx_windows(&FrequencyPlan::Eu868, 868_100_000, SpreadingFactor::SF7, 1);
        assert_eq!(rx1.delay_s, 1);
        assert_eq!(rx2.delay_s, 2);
        assert_eq!(rx1.frequency_hz, 868_100_000);
        assert_eq!(rx2.frequency_hz, 869_525_000);
    }

    #[test]
    fn rx_windows_us915() {
        let (_, rx2) =
            compute_rx_windows(&FrequencyPlan::Us915, 902_300_000, SpreadingFactor::SF10, 1);
        assert_eq!(rx2.frequency_hz, 923_300_000);
    }

    // === Payload Encryption ===

    #[test]
    fn encrypt_decrypt_roundtrip() {
        let key = [0x42u8; 16];
        let plaintext = b"Hello LoRa!";
        let encrypted = encrypt_payload(plaintext, &key);
        let decrypted = decrypt_payload(&encrypted, &key);
        assert_eq!(decrypted, plaintext);
    }

    #[test]
    fn encrypt_changes_data() {
        let key = [0x42u8; 16];
        let plaintext = b"test";
        let encrypted = encrypt_payload(plaintext, &key);
        assert_ne!(encrypted, plaintext);
    }

    #[test]
    fn encrypt_empty() {
        let key = [0u8; 16];
        let encrypted = encrypt_payload(&[], &key);
        assert!(encrypted.is_empty());
    }

    // === Channel Mask ===

    #[test]
    fn channel_mask_all_enabled() {
        let mask = ChannelMask::all_enabled(8);
        for i in 0..8 {
            assert!(mask.is_enabled(i));
        }
        assert!(!mask.is_enabled(8));
        assert_eq!(mask.enabled_count(), 8);
    }

    #[test]
    fn channel_mask_none_enabled() {
        let mask = ChannelMask::none_enabled(16);
        assert_eq!(mask.enabled_count(), 0);
        assert!(!mask.is_enabled(0));
    }

    #[test]
    fn channel_mask_enable_disable() {
        let mut mask = ChannelMask::none_enabled(8);
        mask.enable(3);
        assert!(mask.is_enabled(3));
        assert_eq!(mask.enabled_count(), 1);
        mask.disable(3);
        assert!(!mask.is_enabled(3));
    }

    #[test]
    fn channel_mask_chunk16() {
        let mask = ChannelMask::all_enabled(32);
        assert_eq!(mask.chunk16(0), 0xFFFF);
        assert_eq!(mask.chunk16(1), 0xFFFF);
    }

    #[test]
    fn channel_mask_out_of_range() {
        let mask = ChannelMask::all_enabled(4);
        assert!(!mask.is_enabled(5));
    }

    // === Duty Cycle Manager ===

    #[test]
    fn duty_cycle_new() {
        let dcm = DutyCycleManager::new(3, 0.01);
        assert!(dcm.can_transmit(0));
    }

    #[test]
    fn duty_cycle_record_and_check() {
        let mut dcm = DutyCycleManager::new(1, 0.01);
        dcm.record_tx(0, 35_000); // 35 seconds
        assert!(dcm.can_transmit(0)); // 36 seconds max for 1% of 1 hour
        dcm.record_tx(0, 2_000); // total 37 seconds, exceeds 36
        assert!(!dcm.can_transmit(0));
    }

    #[test]
    fn duty_cycle_remaining() {
        let dcm = DutyCycleManager::new(1, 0.01);
        assert_eq!(dcm.remaining_ms(0), 36_000);
    }

    #[test]
    fn duty_cycle_reset() {
        let mut dcm = DutyCycleManager::new(1, 0.01);
        dcm.record_tx(0, 36_001);
        assert!(!dcm.can_transmit(0));
        dcm.reset();
        assert!(dcm.can_transmit(0));
    }

    #[test]
    fn duty_cycle_invalid_subband() {
        let dcm = DutyCycleManager::new(2, 0.01);
        assert!(!dcm.can_transmit(5));
        assert_eq!(dcm.remaining_ms(5), 0);
    }

    // === Device State Machine ===

    #[test]
    fn device_state_can_receive() {
        assert!(DeviceState::WaitingRx1.can_receive());
        assert!(DeviceState::Receiving.can_receive());
        assert!(!DeviceState::Idle.can_receive());
        assert!(!DeviceState::Transmitting.can_receive());
    }

    #[test]
    fn device_state_is_low_power() {
        assert!(DeviceState::Idle.is_low_power());
        assert!(DeviceState::Sleep.is_low_power());
        assert!(!DeviceState::Transmitting.is_low_power());
    }

    #[test]
    fn class_a_tx_flow() {
        let s = class_a_next_state(DeviceState::Idle, DeviceEvent::TxRequest);
        assert_eq!(s, DeviceState::Transmitting);
        let s = class_a_next_state(s, DeviceEvent::TxComplete);
        assert_eq!(s, DeviceState::WaitingRx1);
    }

    #[test]
    fn class_a_rx1_success() {
        let s = class_a_next_state(DeviceState::WaitingRx1, DeviceEvent::Rx1Open);
        assert_eq!(s, DeviceState::Receiving);
        let s = class_a_next_state(s, DeviceEvent::RxSuccess);
        assert_eq!(s, DeviceState::Processing);
        let s = class_a_next_state(s, DeviceEvent::ProcessComplete);
        assert_eq!(s, DeviceState::Idle);
    }

    #[test]
    fn class_a_rx1_timeout_rx2() {
        let s = class_a_next_state(DeviceState::WaitingRx1, DeviceEvent::Rx1Timeout);
        assert_eq!(s, DeviceState::WaitingRx2);
        let s = class_a_next_state(s, DeviceEvent::Rx2Open);
        assert_eq!(s, DeviceState::Receiving);
    }

    #[test]
    fn class_a_rx2_timeout_idle() {
        let s = class_a_next_state(DeviceState::WaitingRx2, DeviceEvent::Rx2Timeout);
        assert_eq!(s, DeviceState::Idle);
    }

    #[test]
    fn class_a_sleep_wake() {
        let s = class_a_next_state(DeviceState::Idle, DeviceEvent::GoSleep);
        assert_eq!(s, DeviceState::Sleep);
        let s = class_a_next_state(s, DeviceEvent::WakeUp);
        assert_eq!(s, DeviceState::Idle);
    }

    #[test]
    fn class_a_invalid_transition() {
        // Sleep + TxRequest should stay in Sleep
        let s = class_a_next_state(DeviceState::Sleep, DeviceEvent::TxRequest);
        assert_eq!(s, DeviceState::Sleep);
    }

    // === MacCommandId ===

    #[test]
    fn mac_command_id_cid_values() {
        assert_eq!(MacCommandId::LinkCheckReq.cid(), 0x02);
        assert_eq!(MacCommandId::LinkAdrReq.cid(), 0x03);
        assert_eq!(MacCommandId::DutyCycleReq.cid(), 0x04);
        assert_eq!(MacCommandId::RxParamSetupReq.cid(), 0x05);
        assert_eq!(MacCommandId::DevStatusReq.cid(), 0x06);
        assert_eq!(MacCommandId::NewChannelReq.cid(), 0x07);
        assert_eq!(MacCommandId::RxTimingSetupReq.cid(), 0x08);
        assert_eq!(MacCommandId::DlChannelReq.cid(), 0x0A);
    }

    #[test]
    fn mac_command_id_payload_len() {
        assert_eq!(MacCommandId::LinkCheckReq.payload_len(), 0);
        assert_eq!(MacCommandId::LinkCheckAns.payload_len(), 2);
        assert_eq!(MacCommandId::LinkAdrReq.payload_len(), 4);
    }

    // === Additional edge-case tests ===

    #[test]
    fn all_error_variants_display() {
        let errors = [
            LoRaError::InvalidSpreadingFactor,
            LoRaError::InvalidFrameType,
            LoRaError::BufferTooShort,
            LoRaError::InvalidMic,
            LoRaError::InvalidPayloadLength,
            LoRaError::InvalidMacCommand,
            LoRaError::InvalidJoinType,
            LoRaError::InvalidDeviceClass,
            LoRaError::InvalidFrequencyPlan,
            LoRaError::AdrRejected,
        ];
        for e in errors {
            assert!(!format!("{e}").is_empty());
        }
    }

    #[test]
    fn chirp_config_implicit_header() {
        let c = ChirpConfig {
            explicit_header: false,
            ..ChirpConfig::default()
        };
        let toa = c.time_on_air_s(10);
        assert!(toa > 0.0);
    }

    #[test]
    fn chirp_config_no_crc() {
        let c = ChirpConfig {
            crc_enabled: false,
            ..ChirpConfig::default()
        };
        let toa = c.time_on_air_s(10);
        assert!(toa > 0.0);
    }

    #[test]
    fn link_budget_custom() {
        let lb = LinkBudget {
            tx_power_dbm: 20.0,
            tx_antenna_gain_dbi: 3.0,
            rx_antenna_gain_dbi: 3.0,
            cable_loss_db: 1.0,
            rx_sensitivity_dbm: -137.0,
            fade_margin_db: 5.0,
        };
        let range = lb.max_range_km();
        assert!(range > 10.0);
    }

    #[test]
    fn link_budget_range_at_915() {
        let lb = LinkBudget::for_sf(SpreadingFactor::SF10);
        let range_868 = lb.max_range_km_at_freq(868.0);
        let range_915 = lb.max_range_km_at_freq(915.0);
        // Higher frequency = shorter range
        assert!(range_868 > range_915);
    }

    #[test]
    fn frame_confirmed_data_down() {
        let frame = Frame {
            frame_type: FrameType::ConfirmedDataDown,
            dev_addr: 0xFFFF_FFFF,
            f_ctrl: FrameControl {
                ack: true,
                ..FrameControl::default()
            },
            f_cnt: 0xFFFF,
            f_opts: Vec::new(),
            f_port: Some(200),
            payload: vec![1, 2, 3, 4, 5],
            mic: [0xDE, 0xAD, 0xBE, 0xEF],
        };
        let encoded = frame.encode();
        let decoded = Frame::decode(&encoded).unwrap();
        assert_eq!(decoded.frame_type, FrameType::ConfirmedDataDown);
        assert!(decoded.f_ctrl.ack);
        assert_eq!(decoded.f_cnt, 0xFFFF);
    }

    #[test]
    fn channel_mask_raw() {
        let mut mask = ChannelMask::none_enabled(16);
        mask.enable(0);
        mask.enable(15);
        assert_eq!(mask.raw(), (1 << 0) | (1 << 15));
    }

    #[test]
    fn adr_history_eviction() {
        let mut adr = AdrEngine::new(SpreadingFactor::SF10);
        for i in 0..30 {
            adr.record_snr(f64::from(i));
        }
        assert_eq!(adr.sample_count(), 20);
    }
}

//! Frame types & encoding (`FrameType` / `Frame` / `FrameControl`).

use crate::errors::LoRaError;

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

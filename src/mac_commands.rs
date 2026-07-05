//! MAC commands (`MacCommandId` / `MacCommand` / `parse_mac_commands`).

use crate::errors::LoRaError;

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

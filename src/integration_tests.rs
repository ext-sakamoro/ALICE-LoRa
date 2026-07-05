//! Integration tests spanning multiple modules.

#![allow(
    clippy::float_cmp,
    clippy::unreadable_literal,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    clippy::too_many_lines,
    clippy::needless_range_loop,
    clippy::explicit_iter_loop,
    clippy::bool_to_int_with_if,
    clippy::approx_constant,
    clippy::cast_lossless,
    clippy::redundant_clone,
    clippy::format_collect,
    clippy::similar_names,
    clippy::needless_collect,
    clippy::iter_cloned_collect,
    clippy::suboptimal_flops,
    clippy::should_panic_without_expect,
    clippy::manual_range_contains,
    clippy::bool_assert_comparison
)]

use crate::adr::*;
use crate::bandwidth::*;
use crate::channel_mask::*;
use crate::coding_rate::*;
use crate::data_rate::*;
use crate::device_class::*;
use crate::device_state::*;
use crate::duty_cycle::*;
use crate::errors::*;
use crate::frame::*;
use crate::frequency_plan::*;
use crate::join::*;
use crate::link_budget::*;
use crate::mac_commands::*;
use crate::mic::*;
use crate::modulation::*;
use crate::rx_window::*;
use crate::spreading_factor::*;

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
    let (_, rx2) = compute_rx_windows(&FrequencyPlan::Us915, 902_300_000, SpreadingFactor::SF10, 1);
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

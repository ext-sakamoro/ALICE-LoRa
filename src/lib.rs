//! ALICE-LoRa: `LoRaWAN` protocol implementation in pure Rust.
//!
//! Provides chirp spread spectrum modulation, spreading factors (SF7-SF12),
//! adaptive data rate (ADR), OTAA/ABP join procedures, MAC commands,
//! frame encoding/decoding, device classes (A/B/C), frequency plans,
//! and link budget calculation.

#![warn(clippy::all, clippy::pedantic, clippy::nursery)]
#![allow(
    clippy::module_name_repetitions,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::match_same_arms,
    clippy::struct_excessive_bools,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::must_use_candidate,
    clippy::wildcard_imports,
    clippy::doc_markdown,
    clippy::too_many_lines,
    clippy::cast_lossless
)]

pub mod adr;
pub mod bandwidth;
pub mod channel_mask;
pub mod coding_rate;
pub mod data_rate;
pub mod device_class;
pub mod device_state;
pub mod duty_cycle;
pub mod errors;
pub mod frame;
pub mod frequency_plan;
pub mod join;
pub mod link_budget;
pub mod mac_commands;
pub mod mic;
pub mod modulation;
pub mod prelude;
pub mod rx_window;
pub mod spreading_factor;

#[cfg(test)]
mod integration_tests;

// Backward-compat re-exports.
pub use crate::adr::*;
pub use crate::bandwidth::*;
pub use crate::channel_mask::*;
pub use crate::coding_rate::*;
pub use crate::data_rate::*;
pub use crate::device_class::*;
pub use crate::device_state::*;
pub use crate::duty_cycle::*;
pub use crate::errors::*;
pub use crate::frame::*;
pub use crate::frequency_plan::*;
pub use crate::join::*;
pub use crate::link_budget::*;
pub use crate::mac_commands::*;
pub use crate::mic::*;
pub use crate::modulation::*;
pub use crate::rx_window::*;
pub use crate::spreading_factor::*;

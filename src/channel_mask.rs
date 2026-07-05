//! Channel mask (`ChannelMask`).

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

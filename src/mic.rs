//! MIC (Message Integrity Code) — simplified CMAC-like.

// MIC (Message Integrity Code) — simplified CMAC-like
// ---------------------------------------------------------------------------

/// Compute a simplified MIC (4-byte integrity code) over the given data and key.
/// Uses FNV-1a hash for integrity verification without external crypto dependencies.
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

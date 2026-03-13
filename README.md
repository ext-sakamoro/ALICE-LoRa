**English** | [日本語](README_JP.md)

# ALICE-LoRa

**ALICE LoRaWAN Protocol Implementation** — Pure Rust LoRaWAN stack with chirp spread spectrum modulation, adaptive data rate, join procedures, and MAC command handling.

Part of [Project A.L.I.C.E.](https://github.com/anthropics/alice) ecosystem.

## Features

- **Spreading Factors** — SF7 through SF12 with sensitivity and symbol time calculations
- **Bandwidth Support** — 125 kHz, 250 kHz, 500 kHz configurations
- **Adaptive Data Rate (ADR)** — Automatic SF/power optimization
- **Join Procedures** — OTAA and ABP activation modes
- **MAC Commands** — Full MAC command set encoding/decoding
- **Device Classes** — Class A, B, and C device support
- **Frame Codec** — LoRaWAN frame encoding and decoding
- **Frequency Plans** — Regional frequency plan definitions
- **Link Budget** — Signal propagation and link margin calculation

## Architecture

```
SpreadingFactor (SF7-SF12)
 ├── chips_per_symbol
 ├── symbol_time_s
 └── sensitivity_dbm

Bandwidth (125k / 250k / 500k)

Frame
 ├── MHDR (message type, major version)
 ├── Payload (join, data, MAC)
 └── MIC (message integrity code)

Device
 ├── Class A (baseline)
 ├── Class B (beacon-synchronized)
 └── Class C (continuous listen)
```

## Quick Start

```rust
use alice_lora::{SpreadingFactor, Bandwidth};

let sf = SpreadingFactor::SF10;
let symbol_time = sf.symbol_time_s(125_000.0);
let sensitivity = sf.sensitivity_dbm();
```

## License

MIT OR Apache-2.0

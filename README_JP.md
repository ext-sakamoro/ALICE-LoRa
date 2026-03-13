[English](README.md) | **日本語**

# ALICE-LoRa

**ALICE LoRaWANプロトコル実装** — チャープ拡散スペクトラム変調、適応データレート、参加手続き、MACコマンド処理を備えた純Rust LoRaWANスタック。

[Project A.L.I.C.E.](https://github.com/anthropics/alice) エコシステムの一部。

## 機能

- **拡散率** — SF7〜SF12の感度・シンボル時間計算
- **帯域幅** — 125 kHz、250 kHz、500 kHz構成
- **適応データレート（ADR）** — SF/送信電力の自動最適化
- **参加手続き** — OTAAおよびABPアクティベーション
- **MACコマンド** — MACコマンドセットのエンコード/デコード
- **デバイスクラス** — クラスA/B/Cデバイス対応
- **フレームコーデック** — LoRaWANフレームのエンコード・デコード
- **周波数プラン** — 地域別周波数プラン定義
- **リンクバジェット** — 信号伝搬・リンクマージン計算

## アーキテクチャ

```
SpreadingFactor（SF7-SF12）
 ├── chips_per_symbol（シンボルあたりチップ数）
 ├── symbol_time_s（シンボル時間）
 └── sensitivity_dbm（受信感度）

Bandwidth（125k / 250k / 500k）

Frame（フレーム）
 ├── MHDR（メッセージタイプ、メジャーバージョン）
 ├── Payload（参加、データ、MAC）
 └── MIC（メッセージ完全性コード）

Device（デバイス）
 ├── Class A（基本）
 ├── Class B（ビーコン同期）
 └── Class C（常時受信）
```

## クイックスタート

```rust
use alice_lora::{SpreadingFactor, Bandwidth};

let sf = SpreadingFactor::SF10;
let symbol_time = sf.symbol_time_s(125_000.0);
let sensitivity = sf.sensitivity_dbm();
```

## ライセンス

MIT OR Apache-2.0

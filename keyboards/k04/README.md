# Ergohaven K:04 Series

One firmware crate for K:04, K:04 Mini, and K:04 Micro in both Standalone and
Qube topologies. Module settings, pointing devices, battery reader, layer
names, dependencies, and build logic are shared. Connection topology is still
selected at compile time by the binary and profile.

- Standalone: `central` is the left half with a local matrix; `peripheral` is
  the right half.
- Qube: `qube` is the matrix-less USB dongle; `left` and `right` are BLE
  peripherals.

Each topology and model keeps its own matrix, factory keymap, Vial definition,
Product ID, Vial keyboard ID, storage, and UF2 artifact.

| Topology | Profile | Keyboard config | Vial config | Matrix | Product ID |
|----------|---------|-----------------|-------------|--------|------------|
| Standalone | K:04 | `keyboard.toml` | `vial.json` | 10×6 | `0x0074` |
| Standalone | Mini | `keyboard_mini.toml` | `vial_mini.json` | 8×6 | `0x0075` |
| Standalone | Micro | `keyboard_micro.toml` | `vial_micro.json` | 8×6 | `0x0076` |
| Qube | K:04 | `keyboard_qube.toml` | `vial_qube.json` | 10×6 | `0x0071` |
| Qube | Mini | `keyboard_qube_mini.toml` | `vial_qube_mini.json` | 8×6 | `0x0072` |
| Qube | Micro | `keyboard_qube_micro.toml` | `vial_qube_micro.json` | 8×6 | `0x0073` |

## Build

The Standalone `production_v30g` profile preserves the `production_v22`
runtime DPI/acceleration and split-role behavior, adds host-first split wake,
and uses the hardware-validated pointer transport: signed 16-bit X/Y over BLE
and USB, signed 8-bit wheel/pan, one 15 ms paced BLE notification, and
direction-preserving reversal handling. `production_v30g_rtt_diag` enables RTT
on that exact production feature graph rather than selecting a different
pointer profile.

```sh
KEYBOARD_TOML_PATH="$PWD/keyboard.toml" \
VIAL_JSON_PATH="$PWD/vial.json" \
CARGO_TARGET_DIR=target/k04 \
cargo build --release --bin central --bin peripheral --bin hardreset --features production_v30g

KEYBOARD_TOML_PATH="$PWD/keyboard_mini.toml" \
VIAL_JSON_PATH="$PWD/vial_mini.json" \
CARGO_TARGET_DIR=target/mini \
cargo build --release --bin central --bin peripheral --bin hardreset --features production_v30g

KEYBOARD_TOML_PATH="$PWD/keyboard_micro.toml" \
VIAL_JSON_PATH="$PWD/vial_micro.json" \
CARGO_TARGET_DIR=target/micro \
cargo build --release --bin central --bin peripheral --bin hardreset --features production_v30g
```

Qube K:04:

```sh
KEYBOARD_TOML_PATH="$PWD/keyboard_qube.toml" \
VIAL_JSON_PATH="$PWD/vial_qube.json" \
CARGO_TARGET_DIR=target/qube/k04/dongle \
cargo build --release --bin qube --no-default-features --features qube

KEYBOARD_TOML_PATH="$PWD/keyboard_qube.toml" \
VIAL_JSON_PATH="$PWD/vial_qube.json" \
CARGO_TARGET_DIR=target/qube/k04/halves \
cargo build --release --bin left --bin right --no-default-features --features qube-half
```

Use the matching `*_mini` or `*_micro` pair for the other Qube models.
`--no-default-features` keeps Qube's USB-log backend separate from the
Standalone `defmt` backend.

The repository build matrix builds all six profiles:

```sh
./scripts/build_k04_matrix.sh
```

## Battery

The halves use `src/battery_nrf.rs`, which samples `P0_31` without
`calibrate().await` and re-publishes `BatteryStatusEvent` periodically.

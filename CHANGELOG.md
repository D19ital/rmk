# Changelog

## Unreleased

### Changed

- Fixed standalone K:04 host disconnect at 30 minutes and removed the stale hidden timeout setting from firmware and Vial definitions

### Fixes

- Hardened bonded BLE wake through encryption, fast reconnect, local HID suspend, safe split-radio scheduling, stale-peer recovery, and fail-closed recovery from stalled HID notifications
- Released temporary wake-input observers immediately after reconnect so their queues cannot block a key release and leave the last key repeating

## v0.1.8-rc.5

### Improvements

- Advertised firmware-native Repeat/Again support so Entropy can expose it independently from QMK Alt Repeat slots

### Verification

- Kept every production Ergohaven profile at 32 Morse / Tap Dance entries

## v0.1.8-rc.4

### Features

- Added two-stage host BLE power saving for standalone K:04, K:04 Mini, and K:04 Micro: low-duty connection parameters after two minutes, then a configurable full disconnect after 10 minutes to 5 hours
- Preserved the first keyboard, encoder, trackball, or touchpad event while reconnecting after idle sleep

### Fixes

- Applied host disconnect timeout changes immediately and migrated existing v0.1.7 settings to the new 30-minute default
- Kept wake-key press and release reports ordered while a sleeping bonded host reconnects, without blocking later keyboard input

## v0.1.7

### Features

- Added firmware-native Universal Symbols and Russian letters to dynamic Combo and Tap Dance actions
- Added persistent per-combo activation layers configurable from Entropy
- Advertised exact firmware update package identities for all 14 supported Ergohaven keyboard and Qube profiles

### Fixes

- Suspended K:04 trackball and touchpad modules during automatic sleep while preserving their selected type and settings after wake
- Stabilized repeated macro saves, split BLE traffic, pointing-mode changes, and K:04 thumb-cluster geometry

### Removed

- Removed RMK firmware and release builds for the standalone Trackball Mini v3.0, Mini v3.1, and Royale devices

## v0.1.5-rc.1

### Features

- Added modular firmware-native Universal Symbols for EN/RU punctuation, autonomous layout controls, PC/macOS mappings, and optional Entropy Layout Sync
- Advertised Universal Symbols support through the Ergohaven native key-action capability protocol and enabled it for every bundled Ergohaven keyboard profile
- Added firmware-native Russian `х`, `б`, `ю`, and `ъ` actions that behave like regular shifted letter keys while the Russian layout is active

## v0.1.4

### Features

- Added unified Velvet UI firmware for Standalone and Qube, with the right half as the Standalone central and optional PMW3610 trackball support
- Added persistent Velvet trackball enable and Mouse auto-layer timeout controls for Entropy
- Added left/right battery telemetry, live Qube display data, and consistent factory layouts across the unified Ergohaven profiles
- Embedded the Ergohaven firmware version `0.1.4` in every released keyboard definition and exposed it through VIA `id_firmware_version`

### Fixes

- Fixed Velvet startup and split-runtime panics caused by exhausted settings and layer subscribers
- Fixed PMW3610 report starvation, idle jitter, auto-layer timeouts, and stale-motion accumulation
- Fixed split BLE framing, link cadence, HCI command serialization, reconnect synchronization, and wake responsiveness
- Restored RP2040/Pico split compatibility by avoiding unsupported ARMv6-M atomic read-modify-write operations
- Fixed Qube display redraws blocking pointer reports; Shift and Command indicators now update atomically without cursor freezes
- Fixed K:04 encoder detents and USB layer indication, classic keyboard defaults, and live split status reporting

## v0.1.3

### Features

- Added complete K:04 Series firmware for K:04, Mini, and Micro in standalone and Qube configurations
- Added left/right battery telemetry and Qube live display data across the complete K:04 Series
- Added module-aware pointing settings, configurable encoder steps, and factory-enabled touchpad acceleration and gestures for the K:04 Series
- Embedded the Ergohaven manufacturer and firmware version `0.1.3` in the released K:04 and trackball definitions and exposed the same version through VIA `id_firmware_version`

### Fixes

- Fixed split wake latency after idle while retaining the power-saving connection interval
- Fixed excessive idle polling for trackball and touchpad modules
- Fixed BLE Vial report framing, host-session responsiveness, discovery compatibility, and split battery updates
- Fixed K:04 settings persistence, Layer LED writes, touch gestures, and Qube pointing runtime parity

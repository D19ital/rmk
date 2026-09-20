//! K:04 auto-layer timeout wire/storage compatibility helpers.

pub const AUTO_LAYER_TIMEOUT_MIN_MS: u16 = if cfg!(feature = "qube") { 250 } else { 50 };
pub const AUTO_LAYER_TIMEOUT_MAX_MS: u16 = 1500;
pub const AUTO_LAYER_TIMEOUT_DEFAULT_MS: u16 = 500;
pub const AUTO_LAYER_TIMEOUT_PRESETS_MS: [u16; 14] =
    [50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 750, 1000, 1250, 1500];
pub const AUTO_LAYER_TIMEOUT_DEFAULT_INDEX: u8 = 9;

pub const fn auto_layer_timeout_index(value_ms: u16) -> Option<u8> {
    let mut index = 0;
    while index < AUTO_LAYER_TIMEOUT_PRESETS_MS.len() {
        if AUTO_LAYER_TIMEOUT_PRESETS_MS[index] == value_ms {
            return Some(index as u8);
        }
        index += 1;
    }
    None
}

pub const LEGACY_AUTO_LAYER_TIMEOUT_MS: [u16; 6] = [250, 500, 750, 1000, 1250, 1500];

pub const MODULE_SETTINGS_STORAGE_VERSION_V9: u8 = 9;
pub const MODULE_SETTINGS_STORAGE_VERSION: u8 = 10;
pub const MODULE_SETTINGS_STORAGE_LEN_V9: usize = 33;
pub const MODULE_SETTINGS_STORAGE_LEN: usize = 33;

pub const MODULE_SETTINGS_SYNC_VERSION: u8 = 9;
pub const MODULE_SETTINGS_ENCODER_PACKET: u8 = MODULE_SETTINGS_SYNC_VERSION | 0x40;
pub const MODULE_SETTINGS_SYNC_TIMEOUT_LO: usize = 3;
pub const MODULE_SETTINGS_SYNC_TIMEOUT_HI: usize = 4;

const MODULE_SETTINGS_PALETTE_PADDING_BYTE: usize = 28;
const MODULE_SETTINGS_LEGACY_TIMEOUT_BYTE: usize = 30;
const MODULE_SETTINGS_PALETTE_BIT_MASK: u8 = 0x01;
const MODULE_SETTINGS_RIGHT_ENCODER_INTERVAL_MASK: u8 = 0xf0;

pub const fn clamp_auto_layer_timeout_ms(value: u16) -> u16 {
    if !cfg!(feature = "qube") {
        return if auto_layer_timeout_index(value).is_some() {
            value
        } else {
            AUTO_LAYER_TIMEOUT_DEFAULT_MS
        };
    }
    if value < AUTO_LAYER_TIMEOUT_MIN_MS {
        AUTO_LAYER_TIMEOUT_MIN_MS
    } else if value > AUTO_LAYER_TIMEOUT_MAX_MS {
        AUTO_LAYER_TIMEOUT_MAX_MS
    } else {
        value
    }
}

pub const fn legacy_auto_layer_timeout_ms(index: u8) -> u16 {
    LEGACY_AUTO_LAYER_TIMEOUT_MS[if index > 5 { 1 } else { index as usize }]
}

/// Qube retains its legacy nearest-index fallback unchanged. Standalone uses
/// an exact old preset or the default; the extension packet carries the actual
/// preset milliseconds, without rounding new presets to old ones.
pub const fn legacy_auto_layer_timeout_index(value_ms: u16) -> u8 {
    if !cfg!(feature = "qube") {
        let mut index = 0;
        while index < LEGACY_AUTO_LAYER_TIMEOUT_MS.len() {
            if LEGACY_AUTO_LAYER_TIMEOUT_MS[index] == value_ms {
                return index as u8;
            }
            index += 1;
        }
        return 1;
    }
    let clamped = clamp_auto_layer_timeout_ms(value_ms);
    let index = (clamped.saturating_sub(250) + 125) / 250;
    if index > 5 {
        5
    } else {
        index as u8
    }
}

/// QSID 324 is a one-byte select index on Standalone. Ignore the fixed-size
/// report tail, as for other select fields; invalid indices reject the write.
/// Qube retains the test2 little-endian milliseconds/legacy decoder.
pub fn decode_vial_auto_layer_timeout(data: &[u8]) -> Option<u16> {
    if !cfg!(feature = "qube") {
        return data
            .first()
            .and_then(|index| AUTO_LAYER_TIMEOUT_PRESETS_MS.get(usize::from(*index)))
            .copied();
    }
    match data {
        [lo, hi, ..] => {
            let value = u16::from_le_bytes([*lo, *hi]);
            if cfg!(feature = "qube") && value <= 5 {
                Some(legacy_auto_layer_timeout_ms(value as u8))
            } else {
                Some(clamp_auto_layer_timeout_ms(value))
            }
        }
        [legacy_index] if cfg!(feature = "qube") && *legacy_index <= 5 => {
            Some(legacy_auto_layer_timeout_ms(*legacy_index))
        }
        _ => None,
    }
}

/// Return the same wire width that the topology's JSON advertises.
pub fn encode_vial_auto_layer_timeout(value_ms: u16, out: &mut [u8]) -> Option<usize> {
    if cfg!(feature = "qube") {
        if out.len() < 2 {
            return None;
        }
        out[..2].copy_from_slice(&value_ms.to_le_bytes());
        Some(2)
    } else {
        *out.first_mut()? = auto_layer_timeout_index(value_ms).unwrap_or(AUTO_LAYER_TIMEOUT_DEFAULT_INDEX);
        Some(1)
    }
}

/// Store the exact 11-bit timeout in the seven unused palette bits at byte 28
/// plus the legacy timeout nibble at byte 30. The last packed palette bit and
/// the right encoder interval nibble remain unchanged, so the module payload
/// and following compact layer-name layout keep their v9 lengths.
pub fn encode_storage_auto_layer_timeout(data: &mut [u8; MODULE_SETTINGS_STORAGE_LEN], value_ms: u16) {
    let value_ms = clamp_auto_layer_timeout_ms(value_ms);
    data[MODULE_SETTINGS_PALETTE_PADDING_BYTE] = (data[MODULE_SETTINGS_PALETTE_PADDING_BYTE]
        & MODULE_SETTINGS_PALETTE_BIT_MASK)
        | (((value_ms >> 4) as u8) << 1);
    data[MODULE_SETTINGS_LEGACY_TIMEOUT_BYTE] = (data[MODULE_SETTINGS_LEGACY_TIMEOUT_BYTE]
        & MODULE_SETTINGS_RIGHT_ENCODER_INTERVAL_MASK)
        | (value_ms as u8 & 0x0f);
}

pub fn decode_storage_auto_layer_timeout(data: &[u8; MODULE_SETTINGS_STORAGE_LEN]) -> u16 {
    clamp_auto_layer_timeout_ms(
        u16::from(data[MODULE_SETTINGS_LEGACY_TIMEOUT_BYTE] & 0x0f)
            | (u16::from(data[MODULE_SETTINGS_PALETTE_PADDING_BYTE] >> 1) << 4),
    )
}

/// Upgrade the complete packed v9 module payload without changing any
/// unrelated field or its length. Byte 0 becomes schema v10; timeout bits use
/// only storage bits that were padding or the old timeout index.
pub fn migrate_v9_module_settings(old: &[u8; MODULE_SETTINGS_STORAGE_LEN_V9]) -> [u8; MODULE_SETTINGS_STORAGE_LEN] {
    let mut migrated = *old;
    migrated[0] = MODULE_SETTINGS_STORAGE_VERSION;
    let timeout_ms = legacy_auto_layer_timeout_ms(old[MODULE_SETTINGS_LEGACY_TIMEOUT_BYTE] & 0x0f);
    encode_storage_auto_layer_timeout(&mut migrated, timeout_ms);
    migrated
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_v9_index_migrates_without_touching_unrelated_fields_or_length() {
        for (index, expected_ms) in LEGACY_AUTO_LAYER_TIMEOUT_MS.into_iter().enumerate() {
            let mut old = [0u8; MODULE_SETTINGS_STORAGE_LEN_V9];
            for (offset, byte) in old.iter_mut().enumerate() {
                *byte = offset as u8 ^ 0x5a;
            }
            old[0] = MODULE_SETTINGS_STORAGE_VERSION_V9;
            old[MODULE_SETTINGS_LEGACY_TIMEOUT_BYTE] = (old[MODULE_SETTINGS_LEGACY_TIMEOUT_BYTE] & 0xf0) | index as u8;
            let old_palette_bit = old[MODULE_SETTINGS_PALETTE_PADDING_BYTE] & MODULE_SETTINGS_PALETTE_BIT_MASK;
            let old_encoder_interval =
                old[MODULE_SETTINGS_LEGACY_TIMEOUT_BYTE] & MODULE_SETTINGS_RIGHT_ENCODER_INTERVAL_MASK;

            let migrated = migrate_v9_module_settings(&old);

            assert_eq!(migrated.len(), old.len());
            assert_eq!(migrated[0], MODULE_SETTINGS_STORAGE_VERSION);
            assert_eq!(decode_storage_auto_layer_timeout(&migrated), expected_ms);
            assert_eq!(
                migrated[MODULE_SETTINGS_PALETTE_PADDING_BYTE] & MODULE_SETTINGS_PALETTE_BIT_MASK,
                old_palette_bit
            );
            assert_eq!(
                migrated[MODULE_SETTINGS_LEGACY_TIMEOUT_BYTE] & MODULE_SETTINGS_RIGHT_ENCODER_INTERVAL_MASK,
                old_encoder_interval
            );
            for byte in 1..MODULE_SETTINGS_STORAGE_LEN {
                if byte != MODULE_SETTINGS_PALETTE_PADDING_BYTE && byte != MODULE_SETTINGS_LEGACY_TIMEOUT_BYTE {
                    assert_eq!(migrated[byte], old[byte], "unrelated byte {byte} changed");
                }
            }
        }
    }

    #[test]
    #[cfg(feature = "qube")]
    fn exact_storage_codec_preserves_contract_values_and_adjacent_fields() {
        for input in [0u16, 1, 5, 249, 1500, 1501] {
            let expected = input.clamp(AUTO_LAYER_TIMEOUT_MIN_MS, AUTO_LAYER_TIMEOUT_MAX_MS);
            let mut data = [0u8; MODULE_SETTINGS_STORAGE_LEN];
            data[MODULE_SETTINGS_PALETTE_PADDING_BYTE] = 0x01;
            data[MODULE_SETTINGS_LEGACY_TIMEOUT_BYTE] = 0x90;

            encode_storage_auto_layer_timeout(&mut data, input);

            assert_eq!(decode_storage_auto_layer_timeout(&data), expected);
            assert_eq!(data[MODULE_SETTINGS_PALETTE_PADDING_BYTE] & 0x01, 0x01);
            assert_eq!(data[MODULE_SETTINGS_LEGACY_TIMEOUT_BYTE] & 0xf0, 0x90);
        }
    }

    #[test]
    #[cfg(not(feature = "qube"))]
    fn all_fourteen_presets_roundtrip_vial_and_storage() {
        assert_eq!(AUTO_LAYER_TIMEOUT_MIN_MS, 50);
        assert_eq!(auto_layer_timeout_index(AUTO_LAYER_TIMEOUT_DEFAULT_MS), Some(9));
        for (index, ms) in AUTO_LAYER_TIMEOUT_PRESETS_MS.into_iter().enumerate() {
            assert_eq!(auto_layer_timeout_index(ms), Some(index as u8));
            assert_eq!(decode_vial_auto_layer_timeout(&[index as u8]), Some(ms));
            assert_eq!(decode_vial_auto_layer_timeout(&[index as u8, 0, 0, 0]), Some(ms));
            let mut out = [255; 4];
            assert_eq!(encode_vial_auto_layer_timeout(ms, &mut out), Some(1));
            assert_eq!(out, [index as u8, 255, 255, 255]);
            let mut data = [0xa5; MODULE_SETTINGS_STORAGE_LEN];
            encode_storage_auto_layer_timeout(&mut data, ms);
            assert_eq!(decode_storage_auto_layer_timeout(&data), ms);
        }
    }

    #[test]
    #[cfg(not(feature = "qube"))]
    fn invalid_indices_reject_without_arbitrary_ms_or_rounding() {
        for index in 14..=255 {
            assert_eq!(decode_vial_auto_layer_timeout(&[index]), None);
            assert_eq!(decode_vial_auto_layer_timeout(&[index, 0]), None);
        }
        assert_eq!(decode_vial_auto_layer_timeout(&[]), None);
        assert_eq!(encode_vial_auto_layer_timeout(500, &mut []), None);
        assert_eq!(encode_vial_auto_layer_timeout(500, &mut [0]), Some(1));
        for ms in 0..=u16::MAX {
            let expected = if AUTO_LAYER_TIMEOUT_PRESETS_MS.contains(&ms) {
                ms
            } else {
                500
            };
            assert_eq!(clamp_auto_layer_timeout_ms(ms), expected);
            let mut out = [0];
            encode_vial_auto_layer_timeout(ms, &mut out);
            assert_eq!(decode_vial_auto_layer_timeout(&out), Some(expected));
        }
    }

    #[test]
    #[cfg(not(feature = "qube"))]
    fn every_v10_raw_storage_value_preserves_only_presets_and_all_unrelated_bits() {
        // Construct test2's raw 11-bit representation, not the new encoder.
        for ms in 0..=2047u16 {
            let mut data = [0xa5; MODULE_SETTINGS_STORAGE_LEN];
            data[0] = 10;
            data[28] = 1 | (((ms >> 4) as u8) << 1);
            data[30] = 0x90 | (ms as u8 & 15);
            let old = data;
            let expected = if AUTO_LAYER_TIMEOUT_PRESETS_MS.contains(&ms) {
                ms
            } else {
                500
            };
            assert_eq!(decode_storage_auto_layer_timeout(&data), expected);
            assert_eq!(data, old, "decoding must not mutate storage");
            encode_storage_auto_layer_timeout(&mut data, expected);
            assert_eq!(data[28] & 1, old[28] & 1);
            assert_eq!(data[30] & 0xf0, old[30] & 0xf0);
            for offset in 0..data.len() {
                if offset != 28 && offset != 30 {
                    assert_eq!(data[offset], old[offset]);
                }
            }
        }
    }

    #[test]
    fn invalid_v9_indices_migrate_to_default_only() {
        for index in 6..=15 {
            let mut old = [0xa5; MODULE_SETTINGS_STORAGE_LEN_V9];
            old[0] = 9;
            old[30] = 0x90 | index;
            let new = migrate_v9_module_settings(&old);
            assert_eq!(decode_storage_auto_layer_timeout(&new), 500);
            assert_eq!(new[28] & 1, old[28] & 1);
            assert_eq!(new[30] & 0xf0, old[30] & 0xf0);
            for offset in 1..old.len() {
                if offset != 28 && offset != 30 {
                    assert_eq!(new[offset], old[offset]);
                }
            }
        }
    }

    #[test]
    #[cfg(not(feature = "qube"))]
    fn legacy_packet_fallback_is_exact_or_default_never_nearest() {
        for (index, ms) in LEGACY_AUTO_LAYER_TIMEOUT_MS.into_iter().enumerate() {
            assert_eq!(legacy_auto_layer_timeout_index(ms), index as u8);
        }
        for ms in [0, 50, 100, 150, 200, 300, 350, 400, 450, 666, 1499, 65535] {
            assert_eq!(legacy_auto_layer_timeout_index(ms), 1);
        }
    }

    #[test]
    #[cfg(feature = "qube")]
    fn qube_codec_retains_test1_legacy_decoder_and_250ms_minimum() {
        assert_eq!(AUTO_LAYER_TIMEOUT_MIN_MS, 250);
        assert_eq!(AUTO_LAYER_TIMEOUT_DEFAULT_MS, 500);
        for (index, expected_ms) in LEGACY_AUTO_LAYER_TIMEOUT_MS.into_iter().enumerate() {
            assert_eq!(
                decode_vial_auto_layer_timeout(&(index as u16).to_le_bytes()),
                Some(expected_ms)
            );
            assert_eq!(decode_vial_auto_layer_timeout(&[index as u8]), Some(expected_ms));
        }
        assert_eq!(decode_vial_auto_layer_timeout(&249u16.to_le_bytes()), Some(250));
        assert_eq!(decode_vial_auto_layer_timeout(&250u16.to_le_bytes()), Some(250));
        assert_eq!(decode_vial_auto_layer_timeout(&1500u16.to_le_bytes()), Some(1500));
        assert_eq!(decode_vial_auto_layer_timeout(&1501u16.to_le_bytes()), Some(1500));
    }

    #[test]
    #[cfg(feature = "qube")]
    fn arbitrary_values_have_a_safe_nearest_preset_fallback() {
        assert_eq!(legacy_auto_layer_timeout_index(0), 0);
        assert_eq!(legacy_auto_layer_timeout_index(1), 0);
        assert_eq!(legacy_auto_layer_timeout_index(5), 0);
        assert_eq!(legacy_auto_layer_timeout_index(249), 0);
        assert_eq!(legacy_auto_layer_timeout_index(250), 0);
        assert_eq!(legacy_auto_layer_timeout_index(333), 0);
        assert_eq!(legacy_auto_layer_timeout_index(666), 2);
        assert_eq!(legacy_auto_layer_timeout_index(1499), 5);
        assert_eq!(legacy_auto_layer_timeout_index(1500), 5);
    }
}

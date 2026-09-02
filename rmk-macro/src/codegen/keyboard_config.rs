use quote::quote;
use rmk_config::KeyboardTomlConfig;
use rmk_config::resolved::{Host, Identity, Layout};

pub(crate) fn read_keyboard_toml_config() -> KeyboardTomlConfig {
    // Get the path of the keyboard config file from the environment variable
    let config_toml_path = std::env::var("KEYBOARD_TOML_PATH")
        .expect("[ERROR]: KEYBOARD_TOML_PATH should be set in `.cargo/config.toml`");

    KeyboardTomlConfig::new_from_toml_path(&config_toml_path)
}

pub(crate) fn expand_keyboard_info(
    identity: &Identity,
    layout: &Layout,
) -> proc_macro2::TokenStream {
    let pid = identity.product_id;
    let vid = identity.vendor_id;
    let product_name = identity.product_name.clone();
    let manufacturer = identity.manufacturer.clone();
    let serial_number_tokens = match &identity.serial_number {
        Some(s) => quote! { #s },
        None => quote! { ::rmk::config::RMK_BUILD_INFO },
    };
    let device_release = configured_usb_device_release();

    let num_col = layout.cols as usize;
    let num_row = layout.rows as usize;
    let num_layer = layout.layers as usize;
    let num_encoder = &layout.encoder_counts;
    let total_num_encoder: usize = num_encoder.iter().sum();
    quote! {
        pub(crate) const COL: usize = #num_col;
        pub(crate) const ROW: usize = #num_row;
        pub(crate) const NUM_LAYER: usize = #num_layer;
        pub(crate) const NUM_ENCODER: usize = #total_num_encoder;
        const KEYBOARD_DEVICE_CONFIG: ::rmk::config::DeviceConfig = ::rmk::config::DeviceConfig {
            vid: #vid,
            pid: #pid,
            manufacturer: #manufacturer,
            product_name: #product_name,
            serial_number: #serial_number_tokens,
            device_release: #device_release,
        };
    }
}

pub(crate) fn configured_usb_device_release() -> u16 {
    std::env::var("RMK_FIRMWARE_VERSION_BCD")
        .ok()
        .map(|value| {
            parse_usb_device_release(&value).unwrap_or_else(|| {
                panic!("RMK_FIRMWARE_VERSION_BCD must be a hexadecimal u16 such as 0x0109, got {value:?}")
            })
        })
        .unwrap_or(0x0010)
}

fn parse_usb_device_release(value: &str) -> Option<u16> {
    let digits = value
        .trim()
        .strip_prefix("0x")
        .or_else(|| value.trim().strip_prefix("0X"))?;
    u16::from_str_radix(digits, 16).ok()
}

pub(crate) fn expand_vial_config(host: &Host) -> proc_macro2::TokenStream {
    if !host.vial_enabled {
        return quote! {};
    }
    let firmware_version = std::env::var("RMK_FIRMWARE_VERSION")
        .ok()
        .map(|value| {
            parse_firmware_version(&value).unwrap_or_else(|| {
                panic!(
                    "RMK_FIRMWARE_VERSION must be major.minor.patch with minor and patch in 0..=255, got {value:?}"
                )
            })
        })
        .unwrap_or(rmk_types::protocol::vial::VIA_FIRMWARE_VERSION);
    let unlock_keys = if !host.unlock_keys.is_empty() {
        let keys_expr = host
            .unlock_keys
            .iter()
            .map(|key| {
                let row = key[0];
                let col = key[1];
                quote! { (#row, #col) }
            })
            .collect::<Vec<_>>();
        quote! { &[#(#keys_expr), *] }
    } else {
        quote! { &[] }
    };
    let vial_insecure = host.vial_insecure;
    let device_settings = std::env::var("RMK_VIAL_DEVICE_SETTINGS_FN")
        .ok()
        .and_then(|path| syn::parse_str::<syn::Path>(&path).ok())
        .map(|path| quote! { Some(#path()) })
        .unwrap_or_else(|| quote! { None });
    quote! {
        include!(concat!(env!("OUT_DIR"), "/config_generated.rs"));
        const VIAL_CONFIG: ::rmk::config::VialConfig = ::rmk::config::VialConfig {
            vial_keyboard_id: &VIAL_KEYBOARD_ID,
            vial_keyboard_def: &VIAL_KEYBOARD_DEF,
            unlock_keys: #unlock_keys,
            device_settings: #device_settings,
            vial_insecure: #vial_insecure,
            firmware_version: #firmware_version,
        };
    }
}

fn parse_firmware_version(value: &str) -> Option<u32> {
    let mut components = value.trim().split('.');
    let major = components.next()?.parse::<u16>().ok()?;
    let minor = components.next()?.parse::<u8>().ok()?;
    let patch = components.next()?.parse::<u8>().ok()?;
    if components.next().is_some() {
        return None;
    }
    Some((u32::from(major) << 16) | (u32::from(minor) << 8) | u32::from(patch))
}

#[cfg(test)]
mod tests {
    use super::{parse_firmware_version, parse_usb_device_release};

    #[test]
    fn parses_via_runtime_firmware_version() {
        assert_eq!(parse_firmware_version("0.1.3"), Some(0x0000_0103));
        assert_eq!(parse_firmware_version("12.34.56"), Some(0x000C_2238));
    }

    #[test]
    fn rejects_invalid_runtime_firmware_versions() {
        assert_eq!(parse_firmware_version("0.1"), None);
        assert_eq!(parse_firmware_version("0.1.256"), None);
        assert_eq!(parse_firmware_version("0.1.3.4"), None);
    }

    #[test]
    fn parses_usb_device_release_bcd() {
        assert_eq!(parse_usb_device_release("0x0109"), Some(0x0109));
        assert_eq!(parse_usb_device_release("0X1234"), Some(0x1234));
    }

    #[test]
    fn rejects_invalid_usb_device_release_bcd() {
        assert_eq!(parse_usb_device_release("0109"), None);
        assert_eq!(parse_usb_device_release("0x10000"), None);
        assert_eq!(parse_usb_device_release("release"), None);
    }
}

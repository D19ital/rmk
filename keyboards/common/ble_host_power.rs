//! Shared host BLE power policy for Ergohaven standalone keyboards.

use rmk::config::BleHostPowerConfig;

const HOST_DISCONNECT_TIMEOUT_SECONDS: u64 = 30 * 60;

pub(crate) fn ble_host_power_config() -> BleHostPowerConfig {
    BleHostPowerConfig::new(
        rmk::embassy_time::Duration::from_secs(u64::from(rmk::types::constants::SPLIT_CENTRAL_SLEEP_TIMEOUT_SECONDS)),
        host_disconnect_timeout_seconds,
    )
}

fn host_disconnect_timeout_seconds() -> u64 {
    HOST_DISCONNECT_TIMEOUT_SECONDS
}

use embassy_time::Duration;

/// Runtime power policy for the BLE link between a keyboard and its host.
///
/// The connection first switches to low-duty parameters after `idle_timeout`.
/// `disconnect_timeout_seconds` is evaluated at runtime so a keyboard setting
/// can change the full-disconnect deadline without rebuilding the firmware.
#[derive(Clone, Copy)]
pub struct BleHostPowerConfig {
    pub(crate) idle_timeout: Duration,
    pub(crate) disconnect_timeout_seconds: fn() -> u64,
}

impl BleHostPowerConfig {
    pub const fn new(idle_timeout: Duration, disconnect_timeout_seconds: fn() -> u64) -> Self {
        Self {
            idle_timeout,
            disconnect_timeout_seconds,
        }
    }

    pub(crate) fn disconnect_timeout(&self) -> Duration {
        Duration::from_secs((self.disconnect_timeout_seconds)())
    }
}

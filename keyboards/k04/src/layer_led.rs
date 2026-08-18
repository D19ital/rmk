use embassy_nrf::pwm::{SequenceConfig, SequencePwm, SingleSequenceMode, SingleSequencer};
use embassy_time::{Duration, Instant, Timer};
use rmk::event::{
    BatteryStatusEvent, BleAdvertisingMode, BleAdvertisingModeEvent, ConnectionStatusChangeEvent, LayerChangeEvent,
    PeripheralBatteryRefreshEvent, PeripheralSettingsEvent, SleepStateEvent, SplitConnectionState,
    SplitConnectionStateEvent,
};
use rmk::macros::processor;
use rmk::types::battery::BatteryStatus;
use rmk::types::ble::BleState;
use rmk::types::connection::{ConnectionStatus, ConnectionType};

use crate::module_settings::{self, Rgb};

const LED_COUNT: usize = 1;
const LOW_BATTERY_MAX: u8 = 20;
const CHARGED_BATTERY_MIN: u8 = 95;
const BATTERY_PULSE_INTERVAL_MS: u64 = 2_000;
const BATTERY_PULSE_ON_MS: u64 = 120;
const INDICATOR_DURATION_MS: u64 = 1_000;
const STATUS_BLINK_PERIOD_MS: u64 = 360;
const STATUS_BLINK_ON_MS: u64 = 180;
const PWM_POLARITY_INVERTED: u16 = 0x8000;
const PWM_T0H: u16 = PWM_POLARITY_INVERTED | 6;
const PWM_T1H: u16 = PWM_POLARITY_INVERTED | 13;
const RESET_SLOTS: usize = 80;
const FRAME_WORDS: usize = LED_COUNT * 24 + RESET_SLOTS;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Overlay {
    Profile(u8),
    HostConnected,
    SplitConnected,
    Battery(u8),
    LowBattery,
}

#[derive(Clone, Copy)]
struct TimedOverlay {
    kind: Overlay,
    ends: Instant,
}

#[processor(
    subscribe = [
        LayerChangeEvent,
        ConnectionStatusChangeEvent,
        BleAdvertisingModeEvent,
        SplitConnectionStateEvent,
        SleepStateEvent,
        PeripheralSettingsEvent,
        BatteryStatusEvent,
        PeripheralBatteryRefreshEvent
    ],
    poll_interval = 10
)]
pub struct LayerLed {
    led: SequencePwm<'static>,
    current_layer: Option<u8>,
    layer_deadline: Option<Instant>,
    current_color: Option<Rgb>,
    connection_status: Option<ConnectionStatus>,
    ble_profile: u8,
    ble_state: BleState,
    ble_advertising_mode: BleAdvertisingMode,
    ble_snapshot_initialized: bool,
    split_state: SplitConnectionState,
    sleeping: bool,
    indicator_phase_started: Instant,
    overlay: Option<TimedOverlay>,
    latest_battery: Option<u8>,
    pending_battery_display: bool,
    last_low_battery_pulse: Instant,
}

impl LayerLed {
    pub fn new(led: SequencePwm<'static>) -> Self {
        let now = Instant::now();
        Self {
            led,
            current_layer: Some(0),
            layer_deadline: None,
            current_color: None,
            connection_status: None,
            ble_profile: 0,
            ble_state: BleState::Inactive,
            ble_advertising_mode: BleAdvertisingMode::Pairing,
            ble_snapshot_initialized: false,
            split_state: SplitConnectionState::Searching,
            sleeping: false,
            indicator_phase_started: now,
            overlay: None,
            latest_battery: None,
            pending_battery_display: false,
            last_low_battery_pulse: now,
        }
    }

    async fn on_layer_change_event(&mut self, event: LayerChangeEvent) {
        let now = Instant::now();
        self.current_layer = Some(event.0);
        self.arm_layer_timeout(now);
        self.render(now).await;
    }

    async fn on_connection_status_change_event(&mut self, event: ConnectionStatusChangeEvent) {
        let now = Instant::now();
        let previous = self.connection_status;
        let repeated = previous == Some(event.0);
        let profile_changed = previous.is_some_and(|status| status.ble.profile != event.0.ble.profile);
        let state_changed = previous.is_none_or(|status| status.ble.state != event.0.ble.state);
        let previous_ble_state = previous.map(|status| status.ble.state);

        self.connection_status = Some(event.0);
        self.ble_profile = event.0.ble.profile;
        self.ble_state = event.0.ble.state;

        if state_changed {
            self.apply_ble_state_transition(previous_ble_state, self.ble_state);
        }
        if profile_changed || repeated {
            if self.split_state != SplitConnectionState::Searching {
                self.start_overlay(Overlay::Profile(self.ble_profile), indicator_duration());
            }
        }

        self.render(now).await;
    }

    async fn on_ble_advertising_mode_event(&mut self, event: BleAdvertisingModeEvent) {
        let now = Instant::now();
        self.ble_advertising_mode = event.0;
        self.render(now).await;
    }

    async fn on_split_connection_state_event(&mut self, event: SplitConnectionStateEvent) {
        self.set_split_state(event.0);
        self.render(Instant::now()).await;
    }

    async fn on_sleep_state_event(&mut self, event: SleepStateEvent) {
        self.sleeping = event.0;
        self.render(Instant::now()).await;
    }

    async fn on_peripheral_settings_event(&mut self, event: PeripheralSettingsEvent) {
        module_settings::apply_settings_packet(&event.0);
        self.current_color = None;
        self.arm_layer_timeout(Instant::now());
        self.render(Instant::now()).await;
    }

    async fn on_battery_status_event(&mut self, event: BatteryStatusEvent) {
        match event.0 {
            BatteryStatus::Available { level, .. } => {
                self.latest_battery = level;
                if self.pending_battery_display {
                    if let Some(level) = level {
                        self.pending_battery_display = false;
                        self.start_overlay(Overlay::Battery(level), indicator_duration());
                    }
                }
            }
            BatteryStatus::Unavailable => {
                self.latest_battery = None;
            }
        }
        self.render(Instant::now()).await;
    }

    async fn on_peripheral_battery_refresh_event(&mut self, _event: PeripheralBatteryRefreshEvent) {
        self.pending_battery_display = true;
        if let Some(level) = self.latest_battery {
            self.pending_battery_display = false;
            self.start_overlay(Overlay::Battery(level), indicator_duration());
            self.render(Instant::now()).await;
        }
    }

    async fn poll(&mut self) {
        let now = Instant::now();
        self.initialize_ble_snapshot();
        self.expire_overlay(now);

        if self.overlay.is_none() {
            if !crate::battery_nrf::usb_powered()
                && self.latest_battery.is_some_and(|level| level <= LOW_BATTERY_MAX)
                && now.duration_since(self.last_low_battery_pulse).as_millis() >= BATTERY_PULSE_INTERVAL_MS
            {
                self.last_low_battery_pulse = now;
                self.start_overlay(Overlay::LowBattery, Duration::from_millis(BATTERY_PULSE_ON_MS));
            }
        }

        self.render(now).await;
    }

    fn initialize_ble_snapshot(&mut self) {
        if self.ble_snapshot_initialized {
            return;
        }
        self.ble_snapshot_initialized = true;
        let status = rmk::state::current_connection_status();
        self.connection_status = Some(status);
        self.ble_profile = status.ble.profile;
        self.ble_state = status.ble.state;
        self.ble_advertising_mode = rmk::state::current_ble_advertising_mode();
    }

    fn apply_ble_state_transition(&mut self, previous: Option<BleState>, state: BleState) {
        match state {
            BleState::Connected => {
                self.start_overlay(Overlay::HostConnected, indicator_duration());
            }
            BleState::Advertising | BleState::Inactive | BleState::Sleeping => {
                if previous == Some(BleState::Connected) {
                    self.overlay = None;
                }
            }
        }
    }

    fn set_split_state(&mut self, state: SplitConnectionState) {
        if self.split_state == state {
            if state != SplitConnectionState::Connected {
                self.overlay = None;
            }
            return;
        }
        let now = Instant::now();
        self.split_state = state;
        self.indicator_phase_started = now;
        if state == SplitConnectionState::Connected {
            self.start_overlay(Overlay::SplitConnected, indicator_duration());
        } else {
            self.overlay = None;
        }
    }

    fn start_overlay(&mut self, kind: Overlay, duration: Duration) {
        let now = Instant::now();
        self.expire_overlay(now);
        if self.split_state == SplitConnectionState::Searching {
            return;
        }
        if let Some(overlay) = self.overlay {
            if overlay.kind == kind || (is_success_overlay(overlay.kind) && !is_success_overlay(kind)) {
                return;
            }
        }
        self.overlay = Some(TimedOverlay {
            kind,
            ends: now + duration,
        });
    }

    fn expire_overlay(&mut self, now: Instant) {
        if self.overlay.is_some_and(|overlay| now >= overlay.ends) {
            self.overlay = None;
        }
    }

    fn arm_layer_timeout(&mut self, now: Instant) {
        let timeout = module_settings::led_timeout_sec();
        // A non-base layer is direct feedback for a held or locked layer and
        // must stay visible for as long as that layer remains active. The
        // configurable idle timeout still applies to the base-layer color.
        self.layer_deadline =
            (self.current_layer == Some(0) && timeout != 0).then(|| now + Duration::from_secs(u64::from(timeout)));
    }

    async fn render(&mut self, now: Instant) {
        self.expire_overlay(now);
        let color = self.display_color(now);

        if self.current_color == Some(color) {
            return;
        }
        self.current_color = Some(color);
        send_color(&mut self.led, color).await;
    }

    fn display_color(&self, now: Instant) -> Rgb {
        if self.current_layer == Some(0)
            && crate::battery_nrf::usb_powered()
            && module_settings::charge_indicator_enabled()
        {
            return if self.latest_battery.is_some_and(|level| level >= CHARGED_BATTERY_MIN) {
                color_green()
            } else {
                color_yellow()
            };
        }

        if self.sleeping {
            return color_off();
        }

        if self.split_state == SplitConnectionState::Searching {
            let elapsed_ms = now.duration_since(self.indicator_phase_started).as_millis();
            return split_missing_blink_color(elapsed_ms);
        }

        if let Some(overlay) = self.overlay.filter(|overlay| now < overlay.ends) {
            if is_connection_overlay(overlay.kind) {
                return overlay_color(overlay);
            }
        }

        // USB_OUT is an explicit host-transport choice. BLE may keep
        // advertising in the background, but its pairing/reconnect state is
        // irrelevant while USB output is selected and must not replace the
        // active-layer indicator.
        let show_host_ble_status = self
            .connection_status
            .is_none_or(|status| status.preferred == ConnectionType::Ble);
        if show_host_ble_status {
            if matches!(self.ble_state, BleState::Inactive | BleState::Sleeping) {
                return color_off();
            }

            if self.ble_state == BleState::Advertising {
                let elapsed_ms = now.duration_since(self.indicator_phase_started).as_millis();
                return match self.ble_advertising_mode {
                    BleAdvertisingMode::Pairing => pairing_blink_color(elapsed_ms),
                    BleAdvertisingMode::Reconnecting => {
                        blink_color(color_white(), elapsed_ms, STATUS_BLINK_PERIOD_MS, STATUS_BLINK_ON_MS)
                    }
                };
            }
        }

        self.overlay
            .filter(|overlay| now < overlay.ends)
            .map(overlay_color)
            .unwrap_or_else(|| self.layer_color(now))
    }

    fn layer_color(&self, now: Instant) -> Rgb {
        if self.layer_deadline.is_some_and(|deadline| now >= deadline) {
            return color_off();
        }
        self.current_layer.map(color_for_layer).unwrap_or_else(color_off)
    }
}

fn indicator_duration() -> Duration {
    Duration::from_millis(INDICATOR_DURATION_MS)
}

fn overlay_color(overlay: TimedOverlay) -> Rgb {
    match overlay.kind {
        Overlay::Profile(profile) => color_for_bt_profile(profile),
        Overlay::HostConnected => color_green(),
        Overlay::SplitConnected => color_green(),
        Overlay::Battery(level) => color_for_battery(level),
        Overlay::LowBattery => color_red(),
    }
}

fn is_success_overlay(kind: Overlay) -> bool {
    matches!(kind, Overlay::HostConnected | Overlay::SplitConnected)
}

fn is_connection_overlay(kind: Overlay) -> bool {
    matches!(
        kind,
        Overlay::Profile(_) | Overlay::HostConnected | Overlay::SplitConnected
    )
}

fn split_missing_blink_color(elapsed_ms: u64) -> Rgb {
    blink_color(color_yellow(), elapsed_ms, STATUS_BLINK_PERIOD_MS, STATUS_BLINK_ON_MS)
}

fn pairing_blink_color(elapsed_ms: u64) -> Rgb {
    blink_color(color_cyan(), elapsed_ms, STATUS_BLINK_PERIOD_MS, STATUS_BLINK_ON_MS)
}

fn blink_color(color: Rgb, elapsed_ms: u64, period_ms: u64, on_ms: u64) -> Rgb {
    if elapsed_ms % period_ms < on_ms {
        color
    } else {
        color_off()
    }
}

fn color_for_layer(layer: u8) -> Rgb {
    scale_color(module_settings::layer_color(layer))
}

fn color_for_bt_profile(profile: u8) -> Rgb {
    scale_color(module_settings::bt_profile_color(profile))
}

fn color_for_battery(level: u8) -> Rgb {
    let color = match level {
        0..=20 => Rgb { r: 255, g: 0, b: 0 },
        21..=40 => Rgb { r: 255, g: 80, b: 0 },
        41..=74 => Rgb { r: 255, g: 220, b: 0 },
        _ => Rgb { r: 0, g: 255, b: 0 },
    };
    scale_color(color)
}

fn color_cyan() -> Rgb {
    scale_color(Rgb { r: 0, g: 180, b: 255 })
}

fn color_green() -> Rgb {
    scale_color(Rgb { r: 0, g: 255, b: 0 })
}

fn color_red() -> Rgb {
    scale_color(Rgb { r: 255, g: 0, b: 0 })
}

fn color_yellow() -> Rgb {
    scale_color(Rgb { r: 255, g: 180, b: 0 })
}

fn color_white() -> Rgb {
    scale_color(Rgb { r: 255, g: 255, b: 255 })
}

fn color_off() -> Rgb {
    Rgb { r: 0, g: 0, b: 0 }
}

fn scale_color(color: Rgb) -> Rgb {
    Rgb {
        r: scale(color.r),
        g: scale(color.g),
        b: scale(color.b),
    }
}

fn scale(value: u8) -> u8 {
    ((u16::from(value) * u16::from(module_settings::led_brightness())) / 255).min(255) as u8
}

async fn send_color(led: &mut SequencePwm<'static>, color: Rgb) {
    let mut words = [0u16; FRAME_WORDS];
    let mut i = 0usize;

    for byte in [color.g, color.r, color.b] {
        for bit in (0..8).rev() {
            words[i] = if (byte & (1 << bit)) != 0 { PWM_T1H } else { PWM_T0H };
            i += 1;
        }
    }

    let sequencer = SingleSequencer::new(led, &words, SequenceConfig::default());
    let _ = sequencer.start(SingleSequenceMode::Times(1));
    Timer::after(Duration::from_micros(200)).await;
    sequencer.stop();
}

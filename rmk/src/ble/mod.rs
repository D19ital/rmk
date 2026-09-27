use bt_hci::cmd::le::{LeReadLocalSupportedFeatures, LeReadPhy, LeSetPhy};
use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync};
use embassy_futures::join::join3;
use embassy_futures::select::{Either, Either4, select, select4};
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
#[cfg(feature = "host_first_split_wake")]
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use rmk_types::battery::BatteryStatus;
use rmk_types::ble::BleState;
use rmk_types::connection::ConnectionType;
use rmk_types::led_indicator::LedIndicator;
use trouble_host::prelude::appearance::human_interface_device::KEYBOARD;
use trouble_host::prelude::service::{BATTERY, HUMAN_INTERFACE_DEVICE};
use trouble_host::prelude::*;
use usbd_hid::descriptor::MouseReport;

#[cfg(any(
    all(feature = "mouse_realtime_age_cap_30ms", feature = "mouse_realtime_burst_budget_3"),
    all(
        feature = "mouse_realtime_age_cap_30ms",
        feature = "mouse_realtime_reversal_budget_3"
    ),
    all(
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ),
))]
compile_error!("mouse realtime policies are mutually exclusive");

#[cfg(all(feature = "mouse_bounded_multi_notification_3", feature = "mouse_ble_16bit_report"))]
compile_error!("B7 burst and B8 16-bit transport are mutually exclusive");

use crate::ble::battery_service::BleBatteryServer;
use crate::ble::ble_server::{BleHidServer, Server};
use crate::ble::device_info::{PnPID, VidSource};
use crate::ble::led::BleLedReader;
#[cfg(feature = "passkey_entry")]
use crate::ble::passkey::{PasskeyInputState, next_gatt_event};
use crate::ble::profile::{ProfileInfo, ProfileManager, UPDATED_CCCD_TABLE, UPDATED_PROFILE};
use crate::ble::sleep::{
    InputActivityWaiter, report_activity, request_local_sleep, request_sleep, reset_host_power_input,
    take_host_power_input, wait_for_host_power_input,
};
use crate::channel::{BLE_REPORT_CHANNEL, LED_SIGNAL, QueuedReportPayload, WideMouseReport};
use crate::config::{BleBatteryConfig, BleHostPowerConfig, RmkConfig};
use crate::core_traits::Runnable;
use crate::event::BleAdvertisingMode;
use crate::hid::{HidWriterTrait, run_led_reader};
use crate::state::set_ble_state;

pub(crate) mod battery_service;
pub(crate) mod ble_server;
pub(crate) mod device_info;
pub(crate) mod led;
#[cfg(feature = "_nrf_ble")]
pub(crate) mod nrf;
pub mod passkey;
pub(crate) mod profile;
pub(crate) mod sleep;

/// Max number of connections
pub(crate) const CONNECTIONS_MAX: usize = crate::SPLIT_PERIPHERALS_NUM + 1;

/// Max number of L2CAP channels
pub(crate) const L2CAP_CHANNELS_MAX: usize = CONNECTIONS_MAX * 4; // Signal + att + smp + hid

// High-duty directed advertising terminates in the controller at roughly
// 1.28 s. Starting filtered undirected advertising on the same boundary can
// race that termination and make nrf-sdc return HCI Command Disallowed, then
// panic. Use the already bond-filtered undirected path from the first packet.
const DIRECTED_RECONNECT_WINDOW_MS: u64 = 0;
const FAST_BONDED_RECONNECT_TOTAL_MS: u64 = 5_000;
const FAST_ADVERTISING_TIMEOUT_SECS: u64 = 30;
const HOST_PHY_UPDATE_ATTEMPTS: u8 = 3;
const HOST_PHY_UPDATE_SETTLE_MS: u64 = 80;
const HOST_CONNECTION_LIVENESS_POLL_MS: u64 = 250;
const HOST_DISCONNECT_EVENT_TIMEOUT_MS: u64 = 750;
const HOST_SESSION_RELEASE_GRACE_MS: u64 = 100;
const HOST_SESSION_RELEASE_SETTLE_MS: u64 = 100;
const HOST_CONN_PARAM_UPDATE_TIMEOUT_SECS: u64 = 2;
#[cfg(feature = "host_first_split_wake")]
const HOST_ACTIVE_CONN_PARAM_ATTEMPTS: u8 = 2;
#[cfg(feature = "host_first_split_wake")]
const HOST_ACTIVE_CONN_PARAM_RETRY_MS: u64 = 100;
#[cfg(feature = "host_fixed_15ms")]
const HOST_FIXED_CONN_PARAM_ATTEMPTS: u8 = 3;
#[cfg(feature = "host_fixed_15ms")]
const HOST_FIXED_CONN_PARAM_RETRY_MS: u64 = 250;
const HID_WRITE_TIMEOUT_SECS: u64 = 2;
#[cfg(all(
    feature = "mouse_interval_control",
    feature = "mouse_vector_preserve",
    not(any(feature = "host_fixed_15ms", feature = "fixed_mouse_pacing_15ms"))
))]
const MOUSE_CONTROL_INTERVAL: Duration = Duration::from_micros(7_500);
#[cfg(all(
    feature = "mouse_interval_control",
    any(
        not(feature = "mouse_vector_preserve"),
        feature = "host_fixed_15ms",
        feature = "fixed_mouse_pacing_15ms"
    )
))]
const MOUSE_CONTROL_INTERVAL: Duration = Duration::from_millis(15);
#[cfg(not(feature = "host_first_split_wake"))]
const HOST_IDLE_MAX_LATENCY: u16 = 30;
#[cfg(feature = "host_first_split_wake")]
const HOST_IDLE_MAX_LATENCY: u16 = 4;
#[cfg(feature = "host_first_split_wake")]
const HOST_LOW_DUTY_EFFECTIVE_INTERVAL_US: u64 = 150_000;
const HOST_INTERACTIVE_MAX_LATENCY: u16 = 0;
const VIAL_LINK_IDLE_TIMEOUT_SECS: u64 = 30;
const HCI_LINK_UPDATE_ATTEMPTS: u8 = 12;
const HCI_LINK_UPDATE_RETRY_MS: u64 = 20;

// The controller accepts only one link-control procedure at a time. Host PHY
// updates and one or more split links share it, so serialize our commands
// before handling controller-level collisions from procedures started by the
// peer or stack itself.
static BLE_HCI_LINK_UPDATE_MUTEX: Mutex<crate::RawMutex, ()> = Mutex::new(());
#[cfg(feature = "host")]
static VIAL_BLE_ACTIVITY: Signal<crate::RawMutex, ()> = Signal::new();

/// Wakes the connected host-power task when a runtime policy changes.
static HOST_POWER_CONFIG_CHANGED: Signal<crate::RawMutex, ()> = Signal::new();

/// Latched cross-link ordering state for an opt-in keyboard-wide wake.
#[cfg(feature = "host_first_split_wake")]
static HOST_WAKE_ORDER_GATE: Watch<crate::RawMutex, HostWakeOrderGate, CONNECTIONS_MAX> =
    Watch::new_with(HostWakeOrderGate::Open);

#[cfg(feature = "host_first_split_wake")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum HostWakeOrderGate {
    Open,
    Pending,
}

#[cfg(feature = "host_first_split_wake")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum HostWakeOrderEvent {
    HostEnteredIdle,
    HostActiveConfirmed,
    SessionEnded,
}

#[cfg(feature = "host_first_split_wake")]
const fn next_host_wake_order_gate(_current: HostWakeOrderGate, event: HostWakeOrderEvent) -> HostWakeOrderGate {
    match event {
        HostWakeOrderEvent::HostEnteredIdle => HostWakeOrderGate::Pending,
        HostWakeOrderEvent::HostActiveConfirmed | HostWakeOrderEvent::SessionEnded => HostWakeOrderGate::Open,
    }
}

#[cfg(feature = "host_first_split_wake")]
fn set_host_wake_order_gate_on<const N: usize>(
    gate: &Watch<crate::RawMutex, HostWakeOrderGate, N>,
    event: HostWakeOrderEvent,
) {
    let current = gate.try_get().unwrap_or(HostWakeOrderGate::Open);
    gate.sender().send(next_host_wake_order_gate(current, event));
}

#[cfg(feature = "host_first_split_wake")]
pub(crate) fn host_wake_order_gate() -> HostWakeOrderGate {
    HOST_WAKE_ORDER_GATE.try_get().unwrap_or(HostWakeOrderGate::Open)
}

#[cfg(feature = "host_first_split_wake")]
async fn wait_for_host_wake_order_gate_on<const N: usize>(gate: &Watch<crate::RawMutex, HostWakeOrderGate, N>) {
    let Some(mut receiver) = gate.receiver() else {
        // Capacity is sized for every possible connection. Fail open if a
        // future topology violates that invariant rather than deadlocking.
        warn!("[WAKE_ORDER_V30G] split_waiter_capacity_exhausted gate=bypass");
        return;
    };
    receiver.get_and(|state| *state == HostWakeOrderGate::Open).await;
}

#[cfg(feature = "host_first_split_wake")]
pub(crate) async fn wait_for_host_wake_order_gate() {
    wait_for_host_wake_order_gate_on(&HOST_WAKE_ORDER_GATE).await;
}

/// Opens the gate when a host-power task is cancelled or leaves its session.
#[cfg(feature = "host_first_split_wake")]
struct HostWakeOrderSession<'a, const N: usize> {
    gate: &'a Watch<crate::RawMutex, HostWakeOrderGate, N>,
    pending: bool,
}

#[cfg(feature = "host_first_split_wake")]
impl<'a, const N: usize> HostWakeOrderSession<'a, N> {
    fn new_on(gate: &'a Watch<crate::RawMutex, HostWakeOrderGate, N>) -> Self {
        set_host_wake_order_gate_on(gate, HostWakeOrderEvent::SessionEnded);
        Self { gate, pending: false }
    }

    fn close_for_idle(&mut self) {
        self.pending = true;
        set_host_wake_order_gate_on(self.gate, HostWakeOrderEvent::HostEnteredIdle);
        info!("[WAKE_ORDER_V30G] event=host_idle gate=closed");
    }

    fn open_after_confirmation(&mut self, applied: HostConnParamsSnapshot) {
        self.pending = false;
        set_host_wake_order_gate_on(self.gate, HostWakeOrderEvent::HostActiveConfirmed);
        info!(
            "[WAKE_ORDER_V30G] event=host_active_confirmed gate=open interval_us={} latency={}",
            applied.interval.as_micros(),
            applied.latency
        );
    }
}

#[cfg(feature = "host_first_split_wake")]
impl HostWakeOrderSession<'static, CONNECTIONS_MAX> {
    fn new() -> Self {
        Self::new_on(&HOST_WAKE_ORDER_GATE)
    }
}

#[cfg(feature = "host_first_split_wake")]
impl<const N: usize> Drop for HostWakeOrderSession<'_, N> {
    fn drop(&mut self) {
        if self.pending {
            warn!("[WAKE_ORDER_V30G] event=host_session_end gate=open reason=teardown_or_failure");
        }
        set_host_wake_order_gate_on(self.gate, HostWakeOrderEvent::SessionEnded);
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct HostConnParamsSnapshot {
    interval: Duration,
    latency: u16,
}

/// Carries the parameters that the controller actually applied. This lets the
/// bootstrap task distinguish a host that accepted 7.5 ms from an Apple host
/// that retained 15 ms and would otherwise keep the preceding slave latency.
static HOST_CONN_PARAMS_UPDATED: Signal<crate::RawMutex, HostConnParamsSnapshot> = Signal::new();

/// Fixed pacing starts a fresh 15 ms slot after every completed HID write.
#[cfg(feature = "fixed_mouse_pacing_15ms")]
fn fixed_mouse_pacing_deadline(completed_at: Instant) -> Instant {
    completed_at + MOUSE_CONTROL_INTERVAL
}

/// Notify the BLE transport that its runtime host-power policy changed.
pub fn notify_host_power_config_changed() {
    HOST_POWER_CONFIG_CHANGED.signal(());
}

/// Build the BLE stack.
pub async fn build_ble_stack<'a, C: Controller + ControllerCmdAsync<LeSetPhy>, P: PacketPool>(
    controller: C,
    host_address: [u8; 6],
    resources: &'a mut HostResources<P, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX>,
) -> Stack<'a, C, P> {
    // Initialize trouble host stack
    trouble_host::new(controller, resources)
        .set_random_address(Address::random(host_address))
        .build()
}

/// BLE transport runnable. Owns the trouble-host server and profile manager;
/// `run` joins the background `ble_task` runner with the advertise→connect→serve
/// loop and runs forever.
//
pub struct BleTransport<'b, 's, C>
where
    's: 'b,
    C: Controller
        + ControllerCmdAsync<LeSetPhy>
        + ControllerCmdSync<LeReadLocalSupportedFeatures>
        + ControllerCmdSync<LeReadPhy>,
{
    stack: &'b Stack<'s, C, DefaultPacketPool>,
    server: Server<'static>,
    profile_manager: ProfileManager<'b, 's, C, DefaultPacketPool>,
    product_name: &'static str,
    config: BleBatteryConfig<'b>,
    host_power_config: Option<BleHostPowerConfig>,
    use_2m_phy: bool,
}

impl<'b, 's, C> BleTransport<'b, 's, C>
where
    's: 'b,
    C: Controller
        + ControllerCmdAsync<LeSetPhy>
        + ControllerCmdSync<LeReadLocalSupportedFeatures>
        + ControllerCmdSync<LeReadPhy>,
{
    pub async fn new(stack: &'b Stack<'s, C, DefaultPacketPool>, rmk_config: RmkConfig<'static>) -> Self {
        Self::new_with_host_power_and_phy_config(stack, rmk_config, None, true).await
    }

    /// Create a BLE transport with an optional host-link power policy.
    ///
    /// Generated keyboards use this constructor so handwritten `RmkConfig`
    /// initializers remain source-compatible with earlier RMK versions.
    pub async fn new_with_host_power_config(
        stack: &'b Stack<'s, C, DefaultPacketPool>,
        rmk_config: RmkConfig<'static>,
        host_power_config: Option<BleHostPowerConfig>,
    ) -> Self {
        Self::new_with_host_power_and_phy_config(stack, rmk_config, host_power_config, true).await
    }

    /// Create a BLE transport with the complete generated host-link policy.
    ///
    /// Keeping the PHY choice explicit prevents generated configurations with
    /// `use_2m_phy = false` from issuing unsupported runtime PHY requests.
    pub async fn new_with_host_power_and_phy_config(
        stack: &'b Stack<'s, C, DefaultPacketPool>,
        rmk_config: RmkConfig<'static>,
        host_power_config: Option<BleHostPowerConfig>,
        use_2m_phy: bool,
    ) -> Self {
        #[cfg(feature = "_nrf_ble")]
        let serial_number = crate::ble::nrf::get_serial_number();
        #[cfg(not(feature = "_nrf_ble"))]
        let serial_number = rmk_config.device_config.serial_number;

        let profile_manager = ProfileManager::new(stack);

        info!("Starting advertising and GATT service");
        let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
            name: rmk_config.device_config.product_name,
            appearance: &appearance::human_interface_device::KEYBOARD,
        }))
        .unwrap();

        server
            .set(
                &server.device_config_service.pnp_id,
                &PnPID {
                    vid_source: VidSource::UsbIF,
                    vendor_id: rmk_config.device_config.vid,
                    product_id: rmk_config.device_config.pid,
                    product_version: 0x0001,
                },
            )
            .unwrap();
        // The serial number characteristic is length limited, so truncate at a char
        // boundary instead of panicking when the configured serial is too long.
        let mut serial_number_trimmed = heapless::String::new();
        for c in serial_number.chars() {
            if serial_number_trimmed.push(c).is_err() {
                break;
            }
        }
        server
            .set(&server.device_config_service.serial_number, &serial_number_trimmed)
            .unwrap();
        server
            .set(
                &server.device_config_service.manufacturer_name,
                &heapless::String::try_from(rmk_config.device_config.manufacturer).unwrap(),
            )
            .unwrap();

        Self {
            stack,
            server,
            profile_manager,
            product_name: rmk_config.device_config.product_name,
            config: rmk_config.ble_battery_config,
            host_power_config,
            use_2m_phy,
        }
    }
}

impl<'b, 's, C> Runnable for BleTransport<'b, 's, C>
where
    's: 'b,
    C: Controller
        + ControllerCmdAsync<LeSetPhy>
        + ControllerCmdSync<LeReadLocalSupportedFeatures>
        + ControllerCmdSync<LeReadPhy>,
{
    async fn run(&mut self) -> ! {
        // Load the preferred connection from storage
        let preferred = crate::state::load_preferred_connection().await;
        crate::state::set_preferred_connection(preferred);
        // Load the bonded devices from storage
        #[cfg(feature = "storage")]
        self.profile_manager.load_bonded_devices().await;
        self.profile_manager.update_stack_bonds();

        // Copy the &Stack reference so it doesn't tie a borrow to &mut self.
        let stack: &'b Stack<'s, C, DefaultPacketPool> = self.stack;
        let mut peripheral = stack.peripheral();
        let runner = stack.runner();

        let server = &self.server;
        let profile_manager = &mut self.profile_manager;
        let product_name = self.product_name;
        let host_power_config = self.host_power_config;
        let use_2m_phy = self.use_2m_phy;

        let connection_loop = async {
            let mut resuming_from_sleep = false;
            let mut session_sequence = 0u32;
            loop {
                #[cfg(feature = "split")]
                if let Either::Second(()) = select(
                    crate::split::ble::central::wait_for_split_connection_window(),
                    profile_manager.update_profile(),
                )
                .await
                {
                    continue;
                }

                #[cfg(feature = "storage")]
                let active_bond_info = profile_manager.active_bond_info();
                #[cfg(feature = "storage")]
                let active_peer = active_bond_info.as_ref().map(|info| info.info.identity.addr);
                #[cfg(feature = "storage")]
                let host_link_policy =
                    host_link_startup_policy(active_bond_info.is_some(), host_power_config.is_some(), use_2m_phy);
                #[cfg(not(feature = "storage"))]
                let active_peer = None;
                #[cfg(not(feature = "storage"))]
                let host_link_policy = host_link_startup_policy(false, host_power_config.is_some(), use_2m_phy);

                // During wake advertising, subscribe before opening the radio
                // window so a second input can request another attempt even if
                // the current reconnect window expires.
                let wake_during_advertising = WakeAdvertisingInput::new(resuming_from_sleep);

                match select(
                    advertise(product_name, &mut peripheral, server, active_peer, resuming_from_sleep),
                    profile_manager.update_profile(),
                )
                .await
                {
                    Either::First(Ok(conn)) => {
                        session_sequence = session_sequence.wrapping_add(1);
                        let session_id = session_sequence;
                        info!(
                            "[BLE_SESSION_V15] id={} phase=start raw_connected={} connections_max={} l2cap_max={}",
                            session_id,
                            conn.raw().is_connected(),
                            CONNECTIONS_MAX,
                            L2CAP_CHANNELS_MAX
                        );
                        // The wake observer is needed only until advertising
                        // succeeds. Drop both of its PubSub subscribers before
                        // entering a connection that can live for hours.
                        wake_during_advertising.connected();

                        // Do NOT emit BleState::Connected here. gatt_events_task emits
                        // Connected when it sees GattConnectionEvent::Encrypted.
                        let connection_was_resume = resuming_from_sleep;
                        match select(
                            run_ble_keyboard(
                                server,
                                &conn,
                                stack,
                                #[cfg(feature = "storage")]
                                active_bond_info,
                                &self.config,
                                host_power_config,
                                host_link_policy,
                            ),
                            profile_manager.update_profile(),
                        )
                        .await
                        {
                            Either::First(BleKeyboardExit::Disconnected) => {
                                // If wake advertising connected but encryption
                                // never completed, retain Sleeping and retry.
                                resuming_from_sleep = connection_was_resume
                                    && crate::state::current_ble_status().state == BleState::Sleeping;
                            }
                            Either::First(BleKeyboardExit::IdleTimeout) => {
                                info!("Host BLE idle timeout, disconnecting until local input");

                                // Subscribe before disconnecting so input during
                                // teardown is retained as the wake event.
                                let wake = InputActivityWaiter::new();
                                let activity_during_transition = take_host_power_input() == Some(false);

                                if conn.raw().is_connected() {
                                    disconnect_and_wait(&conn).await;
                                }

                                if activity_during_transition {
                                    report_activity();
                                    resuming_from_sleep = true;
                                } else {
                                    match select(wake.wait(), profile_manager.update_profile()).await {
                                        Either::First(()) => {
                                            report_activity();
                                            resuming_from_sleep = true;
                                        }
                                        Either::Second(()) => {
                                            report_activity();
                                            resuming_from_sleep = false;
                                        }
                                    }
                                }
                            }
                            Either::First(BleKeyboardExit::HidWriteStalled) => {
                                error!("BLE HID output stalled, disconnecting for a fail-closed reconnect");

                                // Abandon every report from the unhealthy
                                // session and make the first report after the
                                // reconnect an all-up keyboard state. Sleeping
                                // keeps any new local input queued while the
                                // bonded host reconnects.
                                report_activity();
                                prepare_hid_write_recovery();

                                if conn.raw().is_connected() {
                                    disconnect_and_wait(&conn).await;
                                }
                                resuming_from_sleep = true;
                            }
                            #[cfg(feature = "host_first_split_wake")]
                            Either::First(BleKeyboardExit::ConnectionParamsStalled) => {
                                error!(
                                    "BLE active connection parameters were not restored, disconnecting for a fail-closed reconnect"
                                );

                                // Reports produced while the host remained in
                                // low-duty mode are stale; reconnect from a
                                // clean all-up HID state.
                                report_activity();
                                prepare_hid_write_recovery();

                                if conn.raw().is_connected() {
                                    disconnect_and_wait(&conn).await;
                                }
                                resuming_from_sleep = true;
                            }
                            Either::Second(()) => {
                                resuming_from_sleep = false;
                                report_activity();

                                // When the profile changes, manually disconnect
                                // from the current host.
                                if conn.raw().is_connected() {
                                    disconnect_and_wait(&conn).await;
                                }
                            }
                        }

                        // A logical GATT exit can precede nrf-sdc's final
                        // DisconnectionComplete processing by several
                        // milliseconds. Starting advertising in that gap leaks
                        // the old ACL/GATT resources and eventually produces a
                        // permanent OutOfMemory reconnect loop. Do not leave
                        // this scope until the physical link is down.
                        ensure_host_session_released(&conn, session_id).await;
                        drop(conn);
                        Timer::after_millis(HOST_SESSION_RELEASE_SETTLE_MS).await;
                        info!("[BLE_SESSION_V15] id={} phase=released", session_id);
                    }
                    Either::First(Err(BleHostError::BleHost(Error::Timeout))) => {
                        // A failed BLE host window must not put the whole
                        // keyboard to sleep while another host transport is
                        // still available. This is especially important for a
                        // USB Qube: its BLE stack is also needed for split
                        // links, but the Qube itself is already connected to
                        // the PC over USB.
                        if crate::state::active_transport().is_some() {
                            warn!("Advertising timeout while another transport is active, staying awake");
                            report_activity();
                            resuming_from_sleep = false;
                            set_ble_state(BleState::Inactive);
                            continue;
                        }

                        set_ble_state(BleState::Sleeping);
                        request_sleep();

                        let wake = wake_during_advertising.into_waiter();
                        warn!("Advertising timeout, sleeping until local input");

                        match select(wake.wait(), profile_manager.update_profile()).await {
                            Either::First(()) => {
                                report_activity();
                                resuming_from_sleep = true;
                            }
                            Either::Second(()) => {
                                report_activity();
                                resuming_from_sleep = false;
                            }
                        }
                    }
                    Either::First(Err(e)) => {
                        #[cfg(feature = "defmt")]
                        let e = defmt::Debug2Format(&e);
                        error!("Advertise error: {:?}", e);
                        // This also rate-limits a controller that is still
                        // completing the preceding disconnection.
                        Timer::after_millis(250).await;
                    }
                    Either::Second(()) => {
                        report_activity();
                        resuming_from_sleep = false;
                    }
                };

                // Sleeping remains set while wake advertising is in progress so
                // HID reports stay in the BLE queue for the reconnecting host.
                if !matches!(
                    crate::state::current_ble_status().state,
                    BleState::Advertising | BleState::Sleeping
                ) {
                    set_ble_state(BleState::Inactive);
                }
            }
        };

        // Sleep ownership must outlive every host and split connection. Keeping
        // it beside the BLE runner prevents a disconnected link from leaving
        // the keyboard latched asleep.
        join3(ble_task(runner), connection_loop, sleep::run_sleep_manager()).await;
        unreachable!("BleTransport sub-tasks must run forever")
    }
}

/// This is a background task that is required to run forever alongside any other BLE tasks.
pub(crate) async fn ble_task<C: Controller + ControllerCmdAsync<LeSetPhy>, P: PacketPool>(
    mut runner: Runner<'_, C, P>,
) {
    loop {
        #[cfg(not(feature = "split"))]
        if let Err(_e) = runner.run().await {
            error!("[ble_task] runner.run() error");
            embassy_time::Timer::after_millis(100).await;
        }

        #[cfg(feature = "split")]
        {
            // Signal to indicate the stack is started
            crate::split::ble::central::STACK_STARTED.signal(true);
            if let Err(_e) = runner
                .run_with_handler(&crate::split::ble::central::ScanHandler {})
                .await
            {
                error!("[ble_task] runner.run_with_handler error");
                embassy_time::Timer::after_millis(100).await;
            }
        }
    }
}

/// Stream Events until the connection closes.
///
/// This function will handle the GATT events and process them.
/// This is how we interact with read and write requests.
async fn gatt_events_task<C>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    stack: &Stack<'_, C, DefaultPacketPool>,
    session_ready: &Signal<crate::RawMutex, ()>,
    local_hid_suspend: bool,
) -> Result<(), Error>
where
    C: Controller,
{
    let level = server.battery_service.level;
    let output_keyboard = server.hid_service.output_keyboard;
    let hid_control_point = server.hid_service.hid_control_point;
    let input_keyboard = server.hid_service.input_keyboard;
    #[cfg(feature = "host")]
    let (hid_output_host, hid_input_host) = (server.hid_service.vial_output, server.hid_service.vial_input);
    #[cfg(feature = "host")]
    let (gatt_output_host, gatt_input_host) = (server.vial_gatt_service.output, server.vial_gatt_service.input);
    let mouse = server.hid_service.mouse_report;
    let media = server.hid_service.media_report;
    let system_control = server.hid_service.system_report;

    #[cfg(feature = "passkey_entry")]
    let mut passkey_state = PasskeyInputState::new();

    loop {
        #[cfg(feature = "passkey_entry")]
        let Some(event) = next_gatt_event(conn, &mut passkey_state).await else {
            continue;
        };
        #[cfg(not(feature = "passkey_entry"))]
        let event = conn.next().await;

        match event {
            GattConnectionEvent::Disconnected { reason } => {
                #[cfg(feature = "passkey_entry")]
                passkey_state.clear();
                info!("[gatt] disconnected: {:?}", reason);
                break;
            }
            GattConnectionEvent::PairingComplete { security_level, bond } => {
                #[cfg(feature = "passkey_entry")]
                passkey_state.clear();
                info!("[gatt] pairing complete: {:?}", security_level);
                let profile = crate::state::current_profile();
                if let Some(bond_info) = bond {
                    let cccd_table = server
                        .get_client_att_table(conn.raw())
                        .and_then(|t| heapless::Vec::from_slice(t.raw()).ok())
                        .unwrap_or_default();
                    let profile_info = ProfileInfo {
                        slot_num: profile,
                        info: bond_info,
                        removed: false,
                        cccd_table,
                    };
                    UPDATED_PROFILE.signal(profile_info);
                }
            }
            GattConnectionEvent::PairingFailed(err) => {
                #[cfg(feature = "passkey_entry")]
                passkey_state.clear();
                error!("[gatt] pairing error: {:?}", err);
            }
            GattConnectionEvent::Encrypted { security_level, .. } => {
                info!("[gatt] encrypted: {:?}", security_level);
                mark_ble_session_ready(session_ready);
            }
            GattConnectionEvent::Gatt { event: gatt_event } => {
                let mut cccd_updated = false;
                let result = match &gatt_event {
                    GattEvent::Read(event) => {
                        if event.handle() == level.handle {
                            let value = server.get(&level);
                            debug!("Read GATT Event to Level: {:?}", value);
                        } else {
                            debug!("Read GATT Event to Unknown: {:?}", event.handle());
                        }

                        if conn.raw().security_level()?.encrypted() {
                            None
                        } else {
                            Some(AttErrorCode::INSUFFICIENT_ENCRYPTION)
                        }
                    }
                    GattEvent::Write(event) => {
                        // trouble-host 0.7 exposes written bytes via a closure; copy them out
                        // once so the dispatch below (which awaits) can use them freely.
                        let mut data_buf = [0u8; 32];
                        let data_len = event.with_data(|_, data| {
                            let n = data.len().min(data_buf.len());
                            data_buf[..n].copy_from_slice(&data[..n]);
                            data.len()
                        });
                        let data = &data_buf[..data_len.min(data_buf.len())];

                        if event.handle() == output_keyboard.handle {
                            if data_len == 1 {
                                let led_indicator = LedIndicator::from_bits(data[0]);
                                debug!("Got keyboard state: {:?}", led_indicator);
                                LED_SIGNAL.signal(led_indicator);
                            } else {
                                warn!("Wrong keyboard state data: {:?}", data);
                            }
                        } else if event.handle() == input_keyboard.cccd_handle.expect("No CCCD for input keyboard")
                            || event.handle() == mouse.cccd_handle.expect("No CCCD for mouse report")
                            || event.handle() == media.cccd_handle.expect("No CCCD for media report")
                            || event.handle() == system_control.cccd_handle.expect("No CCCD for system report")
                            || event.handle() == level.cccd_handle.expect("No CCCD for battery level")
                        {
                            cccd_updated = true;
                        } else if event.handle() == hid_control_point.handle {
                            info!("Write GATT Event to Control Point: {:?}", event.handle());
                            // Forward HID suspend/resume to the persistent sleep manager.
                            // HID Class control point opcodes:
                            //   - 0: HID_CTRL_SUSPEND
                            //   - 1: HID_CTRL_EXIT_SUSPEND
                            if data_len == 1 {
                                match hid_control_point_action(data[0], local_hid_suspend) {
                                    HidControlPointAction::Disconnect => request_sleep(),
                                    HidControlPointAction::LocalSleep => request_local_sleep(),
                                    HidControlPointAction::Activity => report_activity(),
                                    HidControlPointAction::Ignore => {}
                                }
                            }
                        } else {
                            #[cfg(feature = "host")]
                            if event.handle() == hid_output_host.handle || event.handle() == gatt_output_host.handle {
                                debug!("Got host packet: {:?}", data);
                                if data_len == 32 {
                                    VIAL_BLE_ACTIVITY.signal(());
                                    report_activity();
                                    let endpoint = if event.handle() == gatt_output_host.handle {
                                        crate::channel::BleHostTransport::VendorGatt
                                    } else {
                                        crate::channel::BleHostTransport::Hid
                                    };
                                    crate::channel::enqueue_host_request(
                                        crate::channel::HostTransport::Ble(endpoint),
                                        data_buf,
                                    )
                                    .await;
                                } else {
                                    warn!("Wrong host packet data: {:?}", data);
                                }
                            } else if event.handle() == hid_input_host.cccd_handle.expect("No CCCD for HID input host")
                                || event.handle() == gatt_input_host.cccd_handle.expect("No CCCD for GATT input host")
                            {
                                cccd_updated = true;
                            } else {
                                debug!("Write GATT Event to Unknown: {:?}", event.handle());
                            }
                            #[cfg(not(feature = "host"))]
                            debug!("Write GATT Event to Unknown: {:?}", event.handle());
                        }

                        if conn.raw().security_level()?.encrypted() {
                            None
                        } else {
                            Some(AttErrorCode::INSUFFICIENT_ENCRYPTION)
                        }
                    }
                    GattEvent::Other(_) => None,
                    GattEvent::NotAllowed(_) => None,
                };

                // This step is also performed at drop(), but writing it explicitly is necessary
                // in order to ensure reply is sent.
                let result = if let Some(code) = result {
                    gatt_event.reject(code)
                } else {
                    gatt_event.accept()
                };
                match result {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!("[gatt] error sending response: {:?}", e),
                }

                // Update CCCD table after processing the event
                if cccd_updated {
                    // When macOS wakes up from sleep mode, it won't send EXIT SUSPEND command
                    // So we need to monitor the sleep state by using CCCD write event
                    report_activity();

                    if let Some(table) = server.get_client_att_table(conn.raw())
                        && let Ok(bytes) = heapless::Vec::from_slice(table.raw())
                    {
                        UPDATED_CCCD_TABLE.signal(bytes);
                    }
                }
            }
            GattConnectionEvent::PhyUpdated { tx_phy, rx_phy } => {
                info!("[gatt] PhyUpdated: {:?}, {:?}", tx_phy, rx_phy)
            }
            GattConnectionEvent::ConnectionParamsUpdated {
                conn_interval,
                peripheral_latency,
                supervision_timeout,
            } => {
                info!(
                    "[gatt] ConnectionParamsUpdated: {:?}ms, {:?}, {:?}ms",
                    conn_interval.as_millis(),
                    peripheral_latency,
                    supervision_timeout.as_millis()
                );
                HOST_CONN_PARAMS_UPDATED.signal(HostConnParamsSnapshot {
                    interval: conn_interval,
                    latency: peripheral_latency,
                });
            }
            GattConnectionEvent::RequestConnectionParams(req) => {
                info!(
                    "[gatt] RequestConnectionParams: interval: ({:?}, {:?})ms, {:?}, {:?}ms",
                    req.params().min_connection_interval.as_millis(),
                    req.params().max_connection_interval.as_millis(),
                    req.params().max_latency,
                    req.params().supervision_timeout.as_millis(),
                );

                // The host connection policy is owned locally; reject peer
                // updates so an unsolicited request cannot replace it.
                let response = req.reject(stack).await;
                if let Err(e) = response {
                    #[cfg(feature = "defmt")]
                    let e = defmt::Debug2Format(&e);
                    warn!("[gatt] failed to respond to connection parameters: {:?}", e);
                }
            }
            GattConnectionEvent::DataLengthUpdated {
                max_tx_octets,
                max_tx_time,
                max_rx_octets,
                max_rx_time,
            } => {
                info!(
                    "[gatt] DataLengthUpdated: tx/rx octets: ({:?}, {:?}), tx/rx time: ({:?}, {:?})",
                    max_tx_octets, max_rx_octets, max_tx_time, max_rx_time
                );
            }
            GattConnectionEvent::FrameSpaceUpdated {
                frame_space,
                initiator,
                phys,
                spacing_types,
            } => {
                info!(
                    "[gatt] FrameSpaceUpdated: {:?}, {:?}, {:?}, {:?}",
                    frame_space, initiator, phys, spacing_types
                );
            }
            GattConnectionEvent::ConnectionRateChanged {
                conn_interval,
                subrate_factor,
                peripheral_latency,
                continuation_number,
                supervision_timeout,
            } => {
                info!(
                    "[gatt] ConnectionRateChanged: {:?}ms, {:?}, {:?}, {:?}, {:?}ms",
                    conn_interval.as_millis(),
                    subrate_factor,
                    peripheral_latency,
                    continuation_number,
                    supervision_timeout.as_millis()
                );
            }
            GattConnectionEvent::PassKeyDisplay(pass_key) => info!("[gatt] PassKeyDisplay: {:?}", pass_key),
            GattConnectionEvent::PassKeyConfirm(pass_key) => info!("[gatt] PassKeyConfirm: {:?}", pass_key),
            GattConnectionEvent::PassKeyInput => {
                #[cfg(feature = "passkey_entry")]
                if crate::PASSKEY_ENTRY_ENABLED {
                    info!("[gatt] PassKeyInput: entering passkey entry mode");
                    passkey_state.begin();
                } else {
                    warn!("[gatt] PassKeyInput: disabled in config, cancelling pairing, this shouldn't happen");
                    if let Err(e) = conn.raw().pass_key_cancel() {
                        error!("[gatt] pass_key_cancel error: {:?}", e);
                    }
                }
                #[cfg(not(feature = "passkey_entry"))]
                warn!("[gatt] PassKeyInput event, should not happen")
            }
            GattConnectionEvent::BondLost => warn!("[gatt] BondLost"),
            GattConnectionEvent::OobRequest => warn!("[gatt] OobRequest"),
        }
    }
    info!("[gatt] task finished");
    Ok(())
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
async fn advertise<'a, 'b, C: Controller>(
    name: &'a str,
    peripheral: &mut Peripheral<'a, C, DefaultPacketPool>,
    server: &'b Server<'_>,
    active_peer: Option<Address>,
    resuming_from_sleep: bool,
) -> Result<GattConnection<'a, 'b, DefaultPacketPool>, BleHostError<C::Error>> {
    // Wait for 10ms to ensure the USB is checked
    embassy_time::Timer::after_millis(10).await;
    let mut advertiser_data = [0; 31];
    AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteServiceUuids16(&[BATTERY.to_le_bytes(), HUMAN_INTERFACE_DEVICE.to_le_bytes()]),
            AdStructure::CompleteLocalName(name.as_bytes()),
            AdStructure::Unknown {
                ty: 0x19, // Appearance
                data: &KEYBOARD.to_le_bytes(),
            },
        ],
        &mut advertiser_data[..],
    )?;

    let fast_advertise_config = AdvertisementParameters {
        // Keep discovery compatible with hosts that scan advertising on LE 1M.
        // The established connection is still upgraded to LE 2M below.
        primary_phy: PhyKind::Le1M,
        secondary_phy: PhyKind::Le1M,
        tx_power: TxPower::Plus8dBm,
        interval_min: Duration::from_millis(30),
        interval_max: Duration::from_millis(30),
        ..Default::default()
    };
    let slow_advertise_config = AdvertisementParameters {
        interval_min: Duration::from_millis(200),
        interval_max: Duration::from_millis(200),
        ..fast_advertise_config
    };

    let reconnect_timeout_secs = u64::from(crate::BLE_RECONNECT_TIMEOUT_SECONDS);
    let reconnect_timeout_ms = reconnect_timeout_secs.saturating_mul(1_000);
    let bonded_windows = bonded_reconnect_windows(reconnect_timeout_ms);
    let configured_pairing_timeout = u64::from(crate::BLE_PAIRING_TIMEOUT_SECONDS);
    let has_active_peer = active_peer.is_some();
    let pairing_window_secs =
        pairing_window_timeout_secs(has_active_peer, configured_pairing_timeout, reconnect_timeout_secs);

    crate::state::set_ble_advertising_mode(advertising_mode(has_active_peer));
    if !resuming_from_sleep {
        set_ble_state(BleState::Advertising);
    }

    if let Some(peer) = active_peer {
        info!("[ADV_GUARD_V13] directed_high_duty=off strategy=filtered_undirected");
        let high_duty_window_ms = bonded_windows.directed_high_duty_ms;
        if high_duty_window_ms > 0 {
            info!("[adv] directed high duty reconnect");
            let advertiser = peripheral
                .advertise(
                    &fast_advertise_config,
                    Advertisement::ConnectableNonscannableDirectedHighDuty { peer },
                )
                .await?;
            match with_timeout(Duration::from_millis(high_duty_window_ms), advertiser.accept()).await {
                Ok(Ok(conn)) => {
                    let conn = conn.with_attribute_server(server)?;
                    info!("[adv] directed connection established");
                    if let Err(e) = conn.raw().set_bondable(true) {
                        error!("Set bondable error: {:?}", e);
                    }
                    return Ok(conn);
                }
                Ok(Err(error)) if directed_reconnect_should_continue(&error) => {
                    info!("[adv] directed reconnect timed out");
                }
                Err(_) => {
                    info!("[adv] directed reconnect window elapsed");
                }
                Ok(Err(error)) => return Err(BleHostError::BleHost(error)),
            }
        }

        if bonded_windows.fast_undirected_ms > 0 || bonded_windows.slow_undirected_ms > 0 {
            // Directed advertising is not rediscovered reliably by every host
            // after a peripheral-initiated idle disconnect. Fall back to an
            // undirected advertisement restricted to the bonded peer: the OS
            // can scan and reconnect normally, while a new host still cannot
            // connect or start pairing.
            peripheral.set_filter_accept_list(&[peer]).await?;
        }

        if bonded_windows.fast_undirected_ms > 0 {
            let fast_bonded_reconnect_config = AdvertisementParameters {
                filter_policy: bonded_reconnect_filter_policy(),
                ..fast_advertise_config
            };
            info!("[adv] fast filtered bonded-host reconnect");
            let advertiser = peripheral
                .advertise(
                    &fast_bonded_reconnect_config,
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &advertiser_data[..],
                        scan_data: &[],
                    },
                )
                .await?;
            match with_timeout(
                Duration::from_millis(bonded_windows.fast_undirected_ms),
                advertiser.accept(),
            )
            .await
            {
                Ok(conn_res) => {
                    let conn = conn_res?.with_attribute_server(server)?;
                    info!("[adv] bonded host connection established");
                    if let Err(e) = conn.raw().set_bondable(false) {
                        error!("Set bondable error: {:?}", e);
                    }
                    return Ok(conn);
                }
                Err(_) => info!("[adv] fast bonded-host reconnect window elapsed"),
            }
        }

        if bonded_windows.slow_undirected_ms > 0 {
            let slow_bonded_reconnect_config = AdvertisementParameters {
                filter_policy: bonded_reconnect_filter_policy(),
                ..slow_advertise_config
            };
            info!("[adv] slow filtered bonded-host reconnect");
            let advertiser = peripheral
                .advertise(
                    &slow_bonded_reconnect_config,
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &advertiser_data[..],
                        scan_data: &[],
                    },
                )
                .await?;
            match with_timeout(
                Duration::from_millis(bonded_windows.slow_undirected_ms),
                advertiser.accept(),
            )
            .await
            {
                Ok(conn_res) => {
                    let conn = conn_res?.with_attribute_server(server)?;
                    info!("[adv] bonded host connection established");
                    if let Err(e) = conn.raw().set_bondable(false) {
                        error!("Set bondable error: {:?}", e);
                    }
                    return Ok(conn);
                }
                Err(_) => info!("[adv] bonded host reconnect timeout"),
            }
        }

        // A bonded profile must never become discoverable for a new host
        // automatically. Opening a pairing window requires an explicit bond
        // clear or switching to an unbonded profile.
        return Err(BleHostError::BleHost(Error::Timeout));
    }

    let Some(undirected_timeout_secs) = pairing_window_secs else {
        return Err(BleHostError::BleHost(Error::Timeout));
    };

    if undirected_timeout_secs == 0 {
        return Err(BleHostError::BleHost(Error::Timeout));
    }

    info!("[adv] fast undirected advertising");
    let advertiser = peripheral
        .advertise(
            &fast_advertise_config,
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..],
                scan_data: &[],
            },
        )
        .await?;

    let fast_timeout_secs = undirected_timeout_secs.min(FAST_ADVERTISING_TIMEOUT_SECS);
    match with_timeout(Duration::from_secs(fast_timeout_secs), advertiser.accept()).await {
        Ok(conn_res) => {
            let conn = conn_res?.with_attribute_server(server)?;
            info!("[adv] connection established");
            if let Err(e) = conn.raw().set_bondable(true) {
                error!("Set bondable error: {:?}", e);
            }
            Ok(conn)
        }
        Err(_) => {
            let slow_timeout_secs = undirected_timeout_secs.saturating_sub(fast_timeout_secs);
            if slow_timeout_secs == 0 {
                return Err(BleHostError::BleHost(Error::Timeout));
            }
            info!("[adv] slow undirected advertising");
            let advertiser = peripheral
                .advertise(
                    &slow_advertise_config,
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &advertiser_data[..],
                        scan_data: &[],
                    },
                )
                .await?;
            match with_timeout(Duration::from_secs(slow_timeout_secs), advertiser.accept()).await {
                Ok(conn_res) => {
                    let conn = conn_res?.with_attribute_server(server)?;
                    info!("[adv] connection established");
                    if let Err(e) = conn.raw().set_bondable(true) {
                        error!("Set bondable error: {:?}", e);
                    }
                    Ok(conn)
                }
                Err(_) => Err(BleHostError::BleHost(Error::Timeout)),
            }
        }
    }
}

fn advertising_mode(has_active_bond: bool) -> BleAdvertisingMode {
    if has_active_bond {
        BleAdvertisingMode::Reconnecting
    } else {
        BleAdvertisingMode::Pairing
    }
}

fn bonded_reconnect_filter_policy() -> AdvFilterPolicy {
    AdvFilterPolicy::FilterConn
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct BondedReconnectWindows {
    directed_high_duty_ms: u64,
    fast_undirected_ms: u64,
    slow_undirected_ms: u64,
}

fn bonded_reconnect_windows(reconnect_timeout_ms: u64) -> BondedReconnectWindows {
    // Directed high-duty reconnect is disabled for this profile, so the
    // reserved window is always zero.
    let directed_high_duty_ms = DIRECTED_RECONNECT_WINDOW_MS;
    let fast_undirected_ms = reconnect_timeout_ms
        .min(FAST_BONDED_RECONNECT_TOTAL_MS)
        .saturating_sub(directed_high_duty_ms);
    let slow_undirected_ms = reconnect_timeout_ms
        .saturating_sub(directed_high_duty_ms)
        .saturating_sub(fast_undirected_ms);

    BondedReconnectWindows {
        directed_high_duty_ms,
        fast_undirected_ms,
        slow_undirected_ms,
    }
}

fn pairing_window_timeout_secs(
    has_active_bond: bool,
    configured_pairing_timeout_secs: u64,
    reconnect_timeout_secs: u64,
) -> Option<u64> {
    if has_active_bond {
        None
    } else if configured_pairing_timeout_secs == 0 {
        Some(reconnect_timeout_secs)
    } else {
        Some(configured_pairing_timeout_secs)
    }
}

fn directed_reconnect_should_continue(error: &Error) -> bool {
    matches!(error, Error::Timeout)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BleKeyboardExit {
    Disconnected,
    IdleTimeout,
    HidWriteStalled,
    #[cfg(feature = "host_first_split_wake")]
    ConnectionParamsStalled,
}

/// Owns the temporary input subscriptions used only while a sleeping host is
/// being advertised to.
///
/// A successful connection must consume this guard before the long-lived BLE
/// session starts. Otherwise its unread PubSub subscribers retain every key
/// and pointing event until the bounded event queues stop the input producers.
struct WakeAdvertisingInput {
    waiter: Option<InputActivityWaiter>,
}

impl WakeAdvertisingInput {
    fn new(resuming_from_sleep: bool) -> Self {
        Self {
            waiter: resuming_from_sleep.then(InputActivityWaiter::new),
        }
    }

    fn connected(self) {
        drop(self.waiter);
    }

    fn into_waiter(self) -> InputActivityWaiter {
        self.waiter.unwrap_or_else(InputActivityWaiter::new)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum HidControlPointAction {
    Disconnect,
    LocalSleep,
    Activity,
    Ignore,
}

fn hid_control_point_action(opcode: u8, local_suspend: bool) -> HidControlPointAction {
    match opcode {
        0 if local_suspend => HidControlPointAction::LocalSleep,
        0 => HidControlPointAction::Disconnect,
        1 => HidControlPointAction::Activity,
        _ => HidControlPointAction::Ignore,
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum HostConnParamBootstrap {
    Legacy,
    BondedRefresh,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct HostLinkStartupPolicy {
    update_phy: bool,
    conn_params: HostConnParamBootstrap,
}

fn host_link_startup_policy(
    has_active_bond: bool,
    preserve_bonded_link: bool,
    use_2m_phy: bool,
) -> HostLinkStartupPolicy {
    if has_active_bond && preserve_bonded_link {
        HostLinkStartupPolicy {
            update_phy: false,
            conn_params: HostConnParamBootstrap::BondedRefresh,
        }
    } else {
        HostLinkStartupPolicy {
            update_phy: use_2m_phy,
            conn_params: HostConnParamBootstrap::Legacy,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum HostPowerTransition {
    EnterIdle,
    Disconnect,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum HostPowerTimer {
    Power(HostPowerTransition),
    VialIdle,
}

fn next_host_power_transition(config: BleHostPowerConfig, idle_connection: bool) -> (Duration, HostPowerTransition) {
    let disconnect_timeout = config.disconnect_timeout();
    if !idle_connection && config.idle_timeout < disconnect_timeout {
        (config.idle_timeout, HostPowerTransition::EnterIdle)
    } else {
        (disconnect_timeout, HostPowerTransition::Disconnect)
    }
}

fn next_host_power_timer(
    config: BleHostPowerConfig,
    idle_connection: bool,
    last_activity: Instant,
    vial_active: bool,
    last_vial_activity: Instant,
) -> (Instant, HostPowerTimer) {
    let (power_after, power_transition) = next_host_power_transition(config, idle_connection);
    let power_deadline = last_activity + power_after;
    let vial_deadline = last_vial_activity + Duration::from_secs(VIAL_LINK_IDLE_TIMEOUT_SECS);

    if vial_active && vial_deadline < power_deadline {
        (vial_deadline, HostPowerTimer::VialIdle)
    } else {
        (power_deadline, HostPowerTimer::Power(power_transition))
    }
}

async fn wait_for_vial_activity() {
    #[cfg(feature = "host")]
    VIAL_BLE_ACTIVITY.wait().await;

    #[cfg(not(feature = "host"))]
    core::future::pending::<()>().await;
}

#[cfg(feature = "host_first_split_wake")]
const fn host_input_requires_active_confirmation(idle_connection: bool, _vial_active: bool) -> bool {
    idle_connection
}

fn host_power_transition_allowed(active_transport: Option<ConnectionType>) -> bool {
    active_transport != Some(ConnectionType::Usb)
}

#[cfg(feature = "host_first_split_wake")]
async fn set_conn_params<'a, 'b, C: Controller + ControllerCmdSync<LeReadLocalSupportedFeatures>, P: PacketPool>(
    stack: &Stack<'_, C, P>,
    conn: &GattConnection<'a, 'b, P>,
    host_power_config: Option<BleHostPowerConfig>,
    bootstrap: HostConnParamBootstrap,
) -> BleKeyboardExit {
    // The guard also clears any stale pending value before bootstrap. Once the
    // task enters host-power idle it owns the gate until confirmation or Drop.
    let mut wake_order_session = HostWakeOrderSession::new();

    if host_power_config.is_some() {
        reset_host_power_input();
        HOST_POWER_CONFIG_CHANGED.reset();
    }

    match bootstrap {
        HostConnParamBootstrap::Legacy => info!("Fresh BLE session, applying current host connection parameters"),
        HostConnParamBootstrap::BondedRefresh => {
            info!("Bonded BLE session, refreshing host connection parameters")
        }
    }

    #[cfg(feature = "host_fixed_15ms")]
    let mut active_params = {
        info!("[HOST_DIAG_V9] mode=fixed15 requested_interval_ms=15 requested_latency=0");
        Timer::after_secs(5).await;
        let target = HostConnParamsSnapshot {
            interval: Duration::from_millis(15),
            latency: HOST_INTERACTIVE_MAX_LATENCY,
        };
        match request_confirmed_active_params(
            stack,
            conn.raw(),
            target,
            0,
            HOST_FIXED_CONN_PARAM_ATTEMPTS,
            HOST_FIXED_CONN_PARAM_RETRY_MS,
        )
        .await
        {
            Some(applied) => applied,
            None if host_power_config.is_some() => return BleKeyboardExit::ConnectionParamsStalled,
            None => target,
        }
    };

    #[cfg(not(feature = "host_fixed_15ms"))]
    let mut active_params = {
        // Ported narrowly from upstream RMK #1088. Apple hosts accept the first
        // 15 ms request; other hosts can accept the later 7.5 ms request. Run the
        // sequence for bonded sessions too so an old 15 ms bond can be upgraded
        // without deleting the profile. The delay keeps link-control procedures
        // away from pairing/encryption and mirrors the upstream timing.
        let requests = host_bootstrap_connection_requests();
        let mut fast_applied = None;
        for (request_index, (interval, max_latency, supervision_timeout)) in requests.into_iter().enumerate() {
            Timer::after_secs(5).await;
            let mut params = host_connection_params(interval, max_latency);
            params.supervision_timeout = supervision_timeout;
            HOST_CONN_PARAMS_UPDATED.reset();
            update_conn_params(stack, conn.raw(), &params).await;

            if request_index == 1 {
                fast_applied = with_timeout(
                    Duration::from_secs(HOST_CONN_PARAM_UPDATE_TIMEOUT_SECS),
                    HOST_CONN_PARAMS_UPDATED.wait(),
                )
                .await
                .ok();
            }
        }

        let target = host_interactive_target(fast_applied);
        if host_requires_apple_safe_fallback(fast_applied) {
            match fast_applied {
                Some(snapshot) => info!(
                    "Host retained {:?}ms latency {}; restoring 15ms latency 0",
                    snapshot.interval.as_millis(),
                    snapshot.latency
                ),
                None => info!("No 7.5ms parameter update observed; restoring 15ms latency 0"),
            }
        } else {
            info!("Host accepted 7.5ms; removing bootstrap slave latency");
        }

        match request_confirmed_active_params(
            stack,
            conn.raw(),
            target,
            0,
            HOST_ACTIVE_CONN_PARAM_ATTEMPTS,
            HOST_ACTIVE_CONN_PARAM_RETRY_MS,
        )
        .await
        {
            Some(applied) => applied,
            None if host_power_config.is_some() => return BleKeyboardExit::ConnectionParamsStalled,
            None => target,
        }
    };

    if let Some(config) = host_power_config {
        let mut last_activity = Instant::now();
        let mut last_vial_activity = last_activity;
        let mut idle_connection = false;
        let mut vial_active = false;

        loop {
            let (deadline, timer_action) =
                next_host_power_timer(config, idle_connection, last_activity, vial_active, last_vial_activity);
            let timer = async move {
                Timer::at(deadline).await;
                timer_action
            };

            match select4(
                wait_for_host_power_input(),
                HOST_POWER_CONFIG_CHANGED.wait(),
                wait_for_vial_activity(),
                timer,
            )
            .await
            {
                Either4::First(immediate_suspend) => {
                    if immediate_suspend {
                        set_ble_state(BleState::Sleeping);
                        // The producer already signalled the persistent sleep
                        // manager. Re-signalling here can overwrite a key's
                        // concurrent activity notification in HOST_POWER_INPUT.
                        return BleKeyboardExit::IdleTimeout;
                    }

                    last_activity = Instant::now();
                    if host_input_requires_active_confirmation(idle_connection, vial_active) {
                        info!("Host BLE activity, restoring active connection parameters");
                        match request_confirmed_active_params(
                            stack,
                            conn.raw(),
                            active_params,
                            1,
                            HOST_ACTIVE_CONN_PARAM_ATTEMPTS,
                            HOST_ACTIVE_CONN_PARAM_RETRY_MS,
                        )
                        .await
                        {
                            Some(applied) => {
                                active_params = applied;
                                wake_order_session.open_after_confirmation(applied);
                            }
                            None => return BleKeyboardExit::ConnectionParamsStalled,
                        }
                    }
                    idle_connection = false;
                }
                Either4::Second(()) => {
                    // Preserve last_activity and recalculate the deadline from
                    // the caller's updated runtime policy.
                }
                Either4::Third(()) => {
                    let now = Instant::now();
                    last_activity = now;
                    last_vial_activity = now;
                    if !vial_active || idle_connection {
                        if idle_connection {
                            match request_confirmed_active_params(
                                stack,
                                conn.raw(),
                                active_params,
                                2,
                                HOST_ACTIVE_CONN_PARAM_ATTEMPTS,
                                HOST_ACTIVE_CONN_PARAM_RETRY_MS,
                            )
                            .await
                            {
                                Some(applied) => {
                                    active_params = applied;
                                    wake_order_session.open_after_confirmation(applied);
                                }
                                None => return BleKeyboardExit::ConnectionParamsStalled,
                            }
                        } else {
                            update_conn_params(
                                stack,
                                conn.raw(),
                                &host_connection_params(active_params.interval, HOST_INTERACTIVE_MAX_LATENCY),
                            )
                            .await;
                        }
                    }
                    vial_active = true;
                    idle_connection = false;
                }
                Either4::Fourth(HostPowerTimer::VialIdle) => {
                    vial_active = false;
                    update_conn_params(
                        stack,
                        conn.raw(),
                        &host_connection_params(active_params.interval, HOST_IDLE_MAX_LATENCY),
                    )
                    .await;
                }
                Either4::Fourth(HostPowerTimer::Power(HostPowerTransition::EnterIdle)) => {
                    if !host_power_transition_allowed(crate::state::active_transport()) {
                        info!("Host BLE idle transition deferred while USB is active");
                        last_activity = Instant::now();
                        continue;
                    }

                    let low_duty = host_low_duty_connection_params(active_params.interval);
                    info!(
                        "[HOST_IDLE_V26] state=request interval_us={} latency={} effective_us={}",
                        low_duty.max_connection_interval.as_micros(),
                        low_duty.max_latency,
                        low_duty.max_connection_interval.as_micros() * (u64::from(low_duty.max_latency) + 1)
                    );
                    // Close before HCI submission so a concurrent keyboard
                    // wake cannot observe a stale open gate in this window.
                    wake_order_session.close_for_idle();
                    update_conn_params(stack, conn.raw(), &low_duty).await;
                    idle_connection = true;
                }
                Either4::Fourth(HostPowerTimer::Power(HostPowerTransition::Disconnect)) => {
                    if !host_power_transition_allowed(crate::state::active_transport()) {
                        info!("Host BLE disconnect deferred while USB is active");
                        last_activity = Instant::now();
                        continue;
                    }

                    set_ble_state(BleState::Sleeping);
                    request_sleep();
                    return BleKeyboardExit::IdleTimeout;
                }
            }
        }
    }

    #[cfg(feature = "host")]
    loop {
        // Slave latency reduces radio duty while idle, but it also stretches
        // every sequential Vial round trip. Switch only the configuration
        // session to latency 0;
        // repeated Vial traffic extends the session without polling.
        VIAL_BLE_ACTIVITY.wait().await;
        update_conn_params(
            stack,
            conn.raw(),
            &host_active_connection_params(HOST_INTERACTIVE_MAX_LATENCY),
        )
        .await;

        while with_timeout(
            Duration::from_secs(VIAL_LINK_IDLE_TIMEOUT_SECS),
            VIAL_BLE_ACTIVITY.wait(),
        )
        .await
        .is_ok()
        {}

        update_conn_params(stack, conn.raw(), &host_active_connection_params(HOST_IDLE_MAX_LATENCY)).await;
    }

    #[cfg(not(feature = "host"))]
    core::future::pending::<BleKeyboardExit>().await
}

#[cfg(not(feature = "host_first_split_wake"))]
async fn set_conn_params<'a, 'b, C: Controller + ControllerCmdSync<LeReadLocalSupportedFeatures>, P: PacketPool>(
    stack: &Stack<'_, C, P>,
    conn: &GattConnection<'a, 'b, P>,
    host_power_config: Option<BleHostPowerConfig>,
    bootstrap: HostConnParamBootstrap,
) -> BleKeyboardExit {
    if host_power_config.is_some() {
        reset_host_power_input();
        HOST_POWER_CONFIG_CHANGED.reset();
    }

    match bootstrap {
        HostConnParamBootstrap::Legacy => info!("Fresh BLE session, applying current host connection parameters"),
        HostConnParamBootstrap::BondedRefresh => {
            info!("Bonded BLE session, refreshing host connection parameters")
        }
    }

    #[cfg(feature = "host_fixed_15ms")]
    {
        info!("[HOST_DIAG_V9] mode=fixed15 requested_interval_ms=15 requested_latency=0");
        Timer::after_secs(5).await;
        let expected = HostConnParamsSnapshot {
            interval: Duration::from_millis(15),
            latency: HOST_INTERACTIVE_MAX_LATENCY,
        };
        for attempt in 1..=HOST_FIXED_CONN_PARAM_ATTEMPTS {
            HOST_CONN_PARAMS_UPDATED.reset();
            update_conn_params(
                stack,
                conn.raw(),
                &host_connection_params(expected.interval, expected.latency),
            )
            .await;

            match with_timeout(
                Duration::from_secs(HOST_CONN_PARAM_UPDATE_TIMEOUT_SECS),
                HOST_CONN_PARAMS_UPDATED.wait(),
            )
            .await
            {
                Ok(applied) if applied == expected => {
                    info!("[HOST_DIAG_V9] confirmed interval_ms=15 latency=0 attempt={}", attempt);
                    break;
                }
                Ok(applied) => warn!(
                    "[HOST_DIAG_V9] mismatch interval_ms={} latency={} attempt={}",
                    applied.interval.as_millis(),
                    applied.latency,
                    attempt
                ),
                Err(_) => warn!("[HOST_DIAG_V9] confirmation_timeout attempt={}", attempt),
            }

            if attempt < HOST_FIXED_CONN_PARAM_ATTEMPTS {
                Timer::after_millis(HOST_FIXED_CONN_PARAM_RETRY_MS).await;
            } else {
                error!("[HOST_DIAG_V9] fixed15_not_confirmed");
            }
        }
    }

    #[cfg(not(feature = "host_fixed_15ms"))]
    {
        // Ported narrowly from upstream RMK #1088. Apple hosts accept the first
        // 15 ms request; other hosts can accept the later 7.5 ms request. Run the
        // sequence for bonded sessions too so an old 15 ms bond can be upgraded
        // without deleting the profile. The delay keeps link-control procedures
        // away from pairing/encryption and mirrors the upstream timing.
        let requests = host_bootstrap_connection_requests();
        for (request_index, (interval, max_latency, supervision_timeout)) in requests.into_iter().enumerate() {
            Timer::after_secs(5).await;
            let mut params = host_connection_params(interval, max_latency);
            params.supervision_timeout = supervision_timeout;
            HOST_CONN_PARAMS_UPDATED.reset();
            update_conn_params(stack, conn.raw(), &params).await;

            if request_index == 1 {
                let applied = with_timeout(
                    Duration::from_secs(HOST_CONN_PARAM_UPDATE_TIMEOUT_SECS),
                    HOST_CONN_PARAMS_UPDATED.wait(),
                )
                .await
                .ok();

                if host_requires_apple_safe_fallback(applied) {
                    match applied {
                        Some(snapshot) => info!(
                            "Host retained {:?}ms latency {}; restoring 15ms latency 0",
                            snapshot.interval.as_millis(),
                            snapshot.latency
                        ),
                        None => info!("No 7.5ms parameter update observed; restoring 15ms latency 0"),
                    }

                    HOST_CONN_PARAMS_UPDATED.reset();
                    update_conn_params(
                        stack,
                        conn.raw(),
                        &host_connection_params(Duration::from_millis(15), HOST_INTERACTIVE_MAX_LATENCY),
                    )
                    .await;
                }
            }
        }
    }

    if let Some(config) = host_power_config {
        let mut last_activity = Instant::now();
        let mut last_vial_activity = last_activity;
        let mut idle_connection = false;
        let mut vial_active = false;

        loop {
            let (deadline, timer_action) =
                next_host_power_timer(config, idle_connection, last_activity, vial_active, last_vial_activity);
            let timer = async move {
                Timer::at(deadline).await;
                timer_action
            };

            match select4(
                wait_for_host_power_input(),
                HOST_POWER_CONFIG_CHANGED.wait(),
                wait_for_vial_activity(),
                timer,
            )
            .await
            {
                Either4::First(immediate_suspend) => {
                    if immediate_suspend {
                        set_ble_state(BleState::Sleeping);
                        // The producer already signalled the persistent sleep
                        // manager. Re-signalling here can overwrite a key's
                        // concurrent activity notification in HOST_POWER_INPUT.
                        return BleKeyboardExit::IdleTimeout;
                    }

                    last_activity = Instant::now();
                    if idle_connection && !vial_active {
                        info!("Host BLE activity, restoring active connection parameters");
                        update_conn_params(stack, conn.raw(), &host_active_connection_params(HOST_IDLE_MAX_LATENCY))
                            .await;
                    }
                    idle_connection = false;
                }
                Either4::Second(()) => {
                    // Preserve last_activity and recalculate the deadline from
                    // the caller's updated runtime policy.
                }
                Either4::Third(()) => {
                    let now = Instant::now();
                    last_activity = now;
                    last_vial_activity = now;
                    if !vial_active || idle_connection {
                        update_conn_params(
                            stack,
                            conn.raw(),
                            &host_active_connection_params(HOST_INTERACTIVE_MAX_LATENCY),
                        )
                        .await;
                    }
                    vial_active = true;
                    idle_connection = false;
                }
                Either4::Fourth(HostPowerTimer::VialIdle) => {
                    vial_active = false;
                    update_conn_params(stack, conn.raw(), &host_active_connection_params(HOST_IDLE_MAX_LATENCY)).await;
                }
                Either4::Fourth(HostPowerTimer::Power(HostPowerTransition::EnterIdle)) => {
                    if !host_power_transition_allowed(crate::state::active_transport()) {
                        info!("Host BLE idle transition deferred while USB is active");
                        last_activity = Instant::now();
                        continue;
                    }

                    info!("Host BLE idle, switching to low-duty connection parameters");
                    update_conn_params(
                        stack,
                        conn.raw(),
                        &host_connection_params(Duration::from_millis(30), HOST_IDLE_MAX_LATENCY),
                    )
                    .await;
                    idle_connection = true;
                }
                Either4::Fourth(HostPowerTimer::Power(HostPowerTransition::Disconnect)) => {
                    if !host_power_transition_allowed(crate::state::active_transport()) {
                        info!("Host BLE disconnect deferred while USB is active");
                        last_activity = Instant::now();
                        continue;
                    }

                    set_ble_state(BleState::Sleeping);
                    request_sleep();
                    return BleKeyboardExit::IdleTimeout;
                }
            }
        }
    }

    #[cfg(feature = "host")]
    loop {
        // Slave latency 30 lets an idle keyboard skip up to 30 connection
        // events, but it also makes every sequential Vial round trip wait up
        // to 232.5 ms. Switch only the configuration session to latency 0;
        // repeated Vial traffic extends the session without polling.
        VIAL_BLE_ACTIVITY.wait().await;
        update_conn_params(
            stack,
            conn.raw(),
            &host_active_connection_params(HOST_INTERACTIVE_MAX_LATENCY),
        )
        .await;

        while with_timeout(
            Duration::from_secs(VIAL_LINK_IDLE_TIMEOUT_SECS),
            VIAL_BLE_ACTIVITY.wait(),
        )
        .await
        .is_ok()
        {}

        update_conn_params(stack, conn.raw(), &host_active_connection_params(HOST_IDLE_MAX_LATENCY)).await;
    }

    #[cfg(not(feature = "host"))]
    core::future::pending::<BleKeyboardExit>().await
}

fn host_connection_params(interval: Duration, max_latency: u16) -> RequestedConnParams {
    RequestedConnParams {
        min_connection_interval: interval,
        max_connection_interval: interval,
        max_latency,
        min_event_length: Duration::from_secs(0),
        max_event_length: Duration::from_secs(0),
        supervision_timeout: Duration::from_secs(5),
    }
}

#[cfg(feature = "host_first_split_wake")]
fn host_low_duty_connection_params(active_interval: Duration) -> RequestedConnParams {
    let interval_us = active_interval.as_micros().max(1);
    let event_count = HOST_LOW_DUTY_EFFECTIVE_INTERVAL_US.saturating_add(interval_us - 1) / interval_us;
    let max_latency = event_count.saturating_sub(1).min(u64::from(u16::MAX)) as u16;
    host_connection_params(active_interval, max_latency)
}

fn host_active_connection_params(max_latency: u16) -> RequestedConnParams {
    #[cfg(feature = "host_fixed_15ms")]
    {
        let _ = max_latency;
        host_connection_params(Duration::from_millis(15), HOST_INTERACTIVE_MAX_LATENCY)
    }
    #[cfg(not(feature = "host_fixed_15ms"))]
    {
        host_connection_params(Duration::from_micros(7500), max_latency)
    }
}

fn host_bootstrap_connection_requests() -> [(Duration, u16, Duration); 2] {
    [
        (Duration::from_millis(15), 30, Duration::from_secs(6)),
        (Duration::from_micros(7500), 60, Duration::from_secs(6)),
    ]
}

fn host_requires_apple_safe_fallback(applied: Option<HostConnParamsSnapshot>) -> bool {
    applied.is_none_or(|snapshot| snapshot.interval > Duration::from_micros(7500))
}

#[cfg(feature = "host_first_split_wake")]
fn host_interactive_target(applied_fast: Option<HostConnParamsSnapshot>) -> HostConnParamsSnapshot {
    HostConnParamsSnapshot {
        interval: if host_requires_apple_safe_fallback(applied_fast) {
            Duration::from_millis(15)
        } else {
            Duration::from_micros(7500)
        },
        latency: HOST_INTERACTIVE_MAX_LATENCY,
    }
}

#[cfg(feature = "host_first_split_wake")]
fn host_active_params_confirmed(target: HostConnParamsSnapshot, applied: HostConnParamsSnapshot) -> bool {
    applied.latency == HOST_INTERACTIVE_MAX_LATENCY && applied.interval <= target.interval
}

#[cfg(feature = "host_first_split_wake")]
async fn request_confirmed_active_params<
    'a,
    'b,
    C: Controller + ControllerCmdSync<LeReadLocalSupportedFeatures>,
    P: PacketPool,
>(
    stack: &Stack<'a, C, P>,
    conn: &Connection<'b, P>,
    target: HostConnParamsSnapshot,
    phase: u8,
    attempts: u8,
    retry_ms: u64,
) -> Option<HostConnParamsSnapshot> {
    for attempt in 1..=attempts {
        info!(
            "[HOST_ACTIVE_V25] phase={} state=request interval_us={} latency={} attempt={}",
            phase,
            target.interval.as_micros(),
            target.latency,
            attempt
        );
        HOST_CONN_PARAMS_UPDATED.reset();
        let submitted = update_conn_params(stack, conn, &host_connection_params(target.interval, target.latency)).await;

        if submitted {
            match with_timeout(
                Duration::from_secs(HOST_CONN_PARAM_UPDATE_TIMEOUT_SECS),
                HOST_CONN_PARAMS_UPDATED.wait(),
            )
            .await
            {
                Ok(applied) if host_active_params_confirmed(target, applied) => {
                    info!(
                        "[HOST_ACTIVE_V25] phase={} state=confirmed interval_us={} latency={} attempt={}",
                        phase,
                        applied.interval.as_micros(),
                        applied.latency,
                        attempt
                    );
                    return Some(applied);
                }
                Ok(applied) => warn!(
                    "[HOST_ACTIVE_V25] phase={} state=mismatch interval_us={} latency={} attempt={}",
                    phase,
                    applied.interval.as_micros(),
                    applied.latency,
                    attempt
                ),
                Err(_) => warn!(
                    "[HOST_ACTIVE_V25] phase={} state=confirmation_timeout attempt={}",
                    phase, attempt
                ),
            }
        } else {
            warn!(
                "[HOST_ACTIVE_V25] phase={} state=request_rejected attempt={}",
                phase, attempt
            );
        }

        if attempt < attempts {
            Timer::after_millis(retry_ms).await;
        }
    }

    error!("[HOST_ACTIVE_V25] phase={} state=not_confirmed", phase);
    None
}

/// Seed the battery characteristic before the host can read it.
fn seed_battery_level(server: &Server<'_>, status: BatteryStatus) {
    if let BatteryStatus::Available { level: Some(level), .. } = status {
        server.set(&server.battery_service.level, &level).unwrap();
    }
}

/// Run BLE keyboard for one connection.
///
/// Returns when the connection drops or its full idle timeout expires. The
/// GATT event pump starts immediately so security events cannot queue behind
/// PHY setup; only HID output waits for bonded encryption.
async fn run_ble_keyboard<
    'a,
    'b,
    C: Controller
        + ControllerCmdAsync<LeSetPhy>
        + ControllerCmdSync<LeReadLocalSupportedFeatures>
        + ControllerCmdSync<LeReadPhy>,
>(
    server: &'b Server<'_>,
    conn: &GattConnection<'a, 'b, DefaultPacketPool>,
    stack: &Stack<'_, C, DefaultPacketPool>,
    #[cfg(feature = "storage")] active_bond_info: Option<crate::ble::profile::ProfileInfo>,
    config: &BleBatteryConfig<'a>,
    host_power_config: Option<BleHostPowerConfig>,
    host_link_policy: HostLinkStartupPolicy,
) -> BleKeyboardExit {
    #[cfg(feature = "host")]
    VIAL_BLE_ACTIVITY.reset();

    // Seed the readable GATT value before processing host requests. Otherwise
    // Windows can read the characteristic's default 0% before the delayed
    // battery notification publishes the measured level.
    if config.enabled {
        seed_battery_level(server, crate::input_device::battery::current_battery_status());
    }

    let mut ble_hid_server = BleHidServer::new(server, conn);
    let mut ble_led_reader = BleLedReader;
    let mut ble_battery_server = config.enabled.then(|| BleBatteryServer::new(server, conn));

    // CCCD lookup uses cached bond info to avoid a cancellable flash read while
    // this future is racing other arms of an outer `select`.
    #[cfg(feature = "storage")]
    if let Some(bond_info) = active_bond_info
        && bond_info.info.identity.match_identity(&conn.raw().peer_identity())
    {
        info!("Loading CCCD table: {:?}", bond_info.cccd_table);
        match ClientAttTableView::try_from_raw(&bond_info.cccd_table) {
            Ok(view) => server.set_client_att_table(conn.raw(), &view),
            Err(e) => warn!("Invalid stored CCCD table: {:?}", e),
        }
    }

    // This is a per-connection barrier, not a second connection state. The
    // GATT event pump must start immediately because trouble-host's bounded
    // connection-event queue can otherwise lose the Encrypted notification
    // while local PHY setup is still pending. Only queued HID output waits for
    // bonded encryption.
    let session_ready: Signal<crate::RawMutex, ()> = Signal::new();

    let gatt_task = run_until_physical_disconnect(
        async {
            let e = gatt_events_task(server, conn, stack, &session_ready, host_power_config.is_some()).await;
            error!("[gatt_events_task] end: {:?}", e);
            BleKeyboardExit::Disconnected
        },
        || conn.raw().is_connected(),
    );
    let communication_task = run_ble_communication_tasks(
        gatt_task,
        set_conn_params(stack, conn, host_power_config, host_link_policy.conn_params),
        ble_battery_server.run(),
        async {
            if host_link_policy.update_phy {
                ensure_host_ble_2m_phy(stack, conn.raw()).await;
            } else {
                info!("Bonded BLE session, preserving host-negotiated PHY");
            }
        },
    );

    let writer_task = run_ble_hid_writer(&mut ble_hid_server, host_power_config.is_some());
    let led_task = run_led_reader(&mut ble_led_reader, ConnectionType::Ble);

    #[cfg(feature = "host")]
    let host_task = crate::host::ble::run_ble_host(server.hid_service.vial_input, server.vial_gatt_service.input, conn);
    #[cfg(not(feature = "host"))]
    let host_task = core::future::pending::<()>();

    let workers = run_ble_session_workers(&session_ready, writer_task, led_task, host_task);

    match select(communication_task, workers).await {
        Either::First(exit) => exit,
        Either::Second(exit) => exit,
    }
}

async fn run_until_physical_disconnect<G, F>(gatt_task: G, is_connected: F) -> BleKeyboardExit
where
    G: core::future::Future<Output = BleKeyboardExit>,
    F: FnMut() -> bool,
{
    match select(gatt_task, wait_for_physical_disconnect(is_connected)).await {
        Either::First(exit) => exit,
        Either::Second(()) => {
            warn!("[gatt] physical link closed without a connection event");
            BleKeyboardExit::Disconnected
        }
    }
}

async fn wait_for_physical_disconnect<F>(mut is_connected: F)
where
    F: FnMut() -> bool,
{
    loop {
        if !is_connected() {
            return;
        }
        Timer::after_millis(HOST_CONNECTION_LIVENESS_POLL_MS).await;
    }
}

async fn disconnect_and_wait<P: PacketPool>(conn: &GattConnection<'_, '_, P>) {
    info!(
        "[BLE_TEARDOWN_V15] phase=request raw_connected={}",
        conn.raw().is_connected()
    );
    conn.raw().disconnect();
    let disconnected_event = async {
        loop {
            if let GattConnectionEvent::Disconnected { .. } = conn.next().await {
                return;
            }
        }
    };

    // In v14 this returned as soon as is_connected() became false. The host
    // runner processed DisconnectionComplete roughly 5 ms later, so the next
    // advertising command raced resource cleanup. Prefer the actual GATT
    // disconnect event; retain a bounded fallback for an overflowing event
    // queue, then still wait for the physical flag.
    match with_timeout(
        Duration::from_millis(HOST_DISCONNECT_EVENT_TIMEOUT_MS),
        disconnected_event,
    )
    .await
    {
        Ok(()) => info!("[BLE_TEARDOWN_V15] phase=disconnect_event"),
        Err(_) => warn!("[BLE_TEARDOWN_V15] phase=disconnect_event_timeout"),
    }
    wait_for_physical_disconnect(|| conn.raw().is_connected()).await;
    info!("[BLE_TEARDOWN_V15] phase=physical_down");
}

async fn ensure_host_session_released<P: PacketPool>(conn: &GattConnection<'_, '_, P>, session_id: u32) {
    if conn.raw().is_connected() {
        warn!(
            "[BLE_SESSION_V15] id={} phase=logical_exit_raw_up waiting_grace",
            session_id
        );
        if with_timeout(
            Duration::from_millis(HOST_SESSION_RELEASE_GRACE_MS),
            wait_for_physical_disconnect(|| conn.raw().is_connected()),
        )
        .await
        .is_err()
        {
            warn!("[BLE_SESSION_V15] id={} phase=forcing_disconnect", session_id);
            conn.raw().disconnect();
            wait_for_physical_disconnect(|| conn.raw().is_connected()).await;
        }
    }
    info!("[BLE_SESSION_V15] id={} phase=physical_down", session_id);
}

async fn run_ble_communication_tasks<G, C, B, P>(
    gatt_task: G,
    conn_params_task: C,
    battery_task: B,
    phy_task: P,
) -> BleKeyboardExit
where
    G: core::future::Future<Output = BleKeyboardExit>,
    C: core::future::Future<Output = BleKeyboardExit>,
    B: core::future::Future,
    P: core::future::Future,
{
    // PHY setup is finite but must stay part of the connection lifetime after
    // it completes. Its HCI mutex still serializes controller commands with
    // split-link updates; running it beside GATT only keeps event consumption
    // live while that procedure is pending.
    let phy_task = async {
        phy_task.await;
        core::future::pending::<()>().await;
    };

    match select4(gatt_task, conn_params_task, battery_task, phy_task).await {
        Either4::First(exit) | Either4::Second(exit) => exit,
        Either4::Third(_) => unreachable!("BLE battery service must run forever"),
        Either4::Fourth(_) => unreachable!("BLE PHY keeper must run forever"),
    }
}

#[cfg(test)]
async fn join_ble_session_workers<H, L, V>(
    session_ready: &Signal<crate::RawMutex, ()>,
    hid_task: H,
    led_task: L,
    host_task: V,
) -> (H::Output, L::Output, V::Output)
where
    H: core::future::Future,
    L: core::future::Future,
    V: core::future::Future,
{
    join3(after_ble_session_ready(session_ready, hid_task), led_task, host_task).await
}

async fn run_ble_session_workers<H, L, V>(
    session_ready: &Signal<crate::RawMutex, ()>,
    hid_task: H,
    led_task: L,
    host_task: V,
) -> H::Output
where
    H: core::future::Future,
    L: core::future::Future,
    V: core::future::Future,
{
    let background_tasks = async {
        embassy_futures::join::join(led_task, host_task).await;
        core::future::pending::<H::Output>().await
    };

    match select(after_ble_session_ready(session_ready, hid_task), background_tasks).await {
        Either::First(exit) => exit,
        Either::Second(exit) => exit,
    }
}

async fn after_ble_session_ready<F>(session_ready: &Signal<crate::RawMutex, ()>, task: F) -> F::Output
where
    F: core::future::Future,
{
    session_ready.wait().await;
    task.await
}

fn mark_ble_session_ready(session_ready: &Signal<crate::RawMutex, ()>) {
    // A physical BLE connection is not ready for HID traffic until bonded
    // encryption has completed. Publish the authoritative state first, then
    // release the per-connection workers so queued reports cannot race it.
    set_ble_state(BleState::Connected);
    session_ready.signal(());
}

async fn run_ble_hid_writer<W>(writer: &mut W, fail_closed: bool) -> BleKeyboardExit
where
    W: HidWriterTrait<ReportType = crate::hid::Report>,
{
    #[cfg(all(feature = "rtt_diag", not(feature = "mouse_interval_control")))]
    info!("[HID_DIAG_V7] mode=age_gatt_baseline");
    #[cfg(all(
        feature = "rtt_diag",
        feature = "mouse_interval_control",
        not(feature = "mouse_vector_preserve"),
        not(feature = "host_fixed_15ms")
    ))]
    info!("[HID_DIAG_V8] mode=ble15_axis_diag interval_ms=15 chunk=independent");
    #[cfg(all(
        feature = "rtt_diag",
        feature = "mouse_vector_preserve",
        not(feature = "host_fixed_15ms")
    ))]
    info!(
        "[HID_DIAG_V22] mode=ble7500_vector_preserve interval_us=7500 chunk=proportional split_source=15ms_i16 windows_per_report=2"
    );
    #[cfg(all(
        feature = "rtt_diag",
        feature = "mouse_interval_control",
        not(feature = "mouse_vector_preserve"),
        feature = "host_fixed_15ms"
    ))]
    info!("[HID_DIAG_V9] mode=axis_fixed15 interval_ms=15 latency=0 chunk=independent");
    #[cfg(all(feature = "rtt_diag", feature = "mouse_vector_preserve", feature = "host_fixed_15ms"))]
    info!("[HID_DIAG_V9] mode=vector_fixed15 interval_ms=15 latency=0 chunk=proportional");
    #[cfg(feature = "mouse_realtime_age_cap_30ms")]
    info!("[MOUSE_REALTIME_AGE_CAP_V1] age_cap_ms=30 stale_xy=single_proportional commit=after_gatt_ok");
    #[cfg(feature = "mouse_realtime_burst_budget_3")]
    info!(
        "[MOUSE_REALTIME_B2_V1] age_cap_ms=30 stale_xy=proportional_budget vectors=3 cadence_ms=15 commit=after_gatt_ok"
    );
    #[cfg(all(
        feature = "mouse_realtime_reversal_budget_3",
        not(feature = "mouse_bounded_multi_notification_3")
    ))]
    info!(
        "[MOUSE_REALTIME_B6_V1] policy=ble_diag_only age_cap_ms=30 stale_xy=proportional_budget vectors=3 pacing=one_notification_per_15ms reversal=raw_pre_acceleration_source_sign deadband=100 gesture_idle_us=1000000 confirm=two_consecutive_meaningful_source_reports seq=duplicate_ignore_gap_restart_wrap_ok first_epoch_report=baseline_only residual_cannot_seed=true per_source=true per_axis=true retirement=after_gatt_ok retry=byte_state_identical burst=disabled"
    );
    #[cfg(feature = "mouse_bounded_multi_notification_3")]
    info!(
        "[MOUSE_REALTIME_B7_V1] policy=ble_diag_only age_cap_ms=30 stale_xy=proportional_budget vectors=3 pacing=one_slot_per_15ms bounded_stack_handoffs_per_stale_epoch=3 sequential=true fresh_residual_burst=false reversal=b6_raw_pre_acceleration_source_sign confirmed_identity=first_second_frozen_until_stack_handoff_ok retirement=after_stack_handoff_ok retry=byte_state_identical hid_delivery_ack=unavailable"
    );
    #[cfg(feature = "mouse_ble_16bit_report")]
    info!(
        "[MOUSE_REALTIME_B8_V1] policy=ble_diag_only report_id=2 axes=signed_i16_le payload_bytes=7 pacing=one_notification_per_15ms burst=disabled stale_drop=disabled reversal=b6_raw_pre_acceleration_source_sign commit=after_stack_handoff_ok retry=byte_state_identical usb=unchanged"
    );

    let mut deferred_report = None;
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    let mut reversal_memories = [ReversalMemory::default(); 4];
    #[cfg(not(any(
        feature = "mouse_realtime_age_cap_30ms",
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    )))]
    let mut pending_mouse = None;
    #[cfg(any(
        feature = "mouse_realtime_age_cap_30ms",
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    let mut retry_chunk_plan = None;
    #[cfg(any(
        feature = "mouse_realtime_age_cap_30ms",
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    let mut pending_mouse = BLE_MOUSE_RETRY.try_take().map(|retry| {
        retry_chunk_plan = Some(retry.plan);
        retry.mouse
    });
    #[cfg(feature = "mouse_interval_control")]
    let mut next_mouse_slot = None;
    #[cfg(feature = "mouse_bounded_multi_notification_3")]
    let mut burst_slot = 0u32;
    loop {
        let mut mouse = if let Some(mouse) = pending_mouse.take() {
            mouse
        } else {
            let queued = if let Some(report) = deferred_report.take() {
                report
            } else {
                BLE_REPORT_CHANNEL.receive().await
            };
            let enqueued_at = queued.enqueued_at();

            match queued.into_payload() {
                QueuedReportPayload::Hid(crate::hid::Report::MouseReport(mouse)) => {
                    #[allow(unused_mut)]
                    let mut accumulated = AccumulatedMouseReport::new(mouse, enqueued_at);
                    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                    accumulated.restore_reversal_memory_and_process_first(
                        reversal_memories[0],
                        #[cfg(feature = "rtt_diag")]
                        None,
                    );
                    accumulated
                }
                QueuedReportPayload::WideMouse(mouse) => {
                    #[cfg(feature = "rtt_diag")]
                    let source = mouse.source;
                    #[cfg(all(feature = "mouse_realtime_reversal_budget_3", feature = "rtt_diag"))]
                    let memory_index = source.map(|meta| usize::from(meta.device_id.min(3))).unwrap_or(0);
                    #[cfg(all(feature = "mouse_realtime_reversal_budget_3", not(feature = "rtt_diag")))]
                    let memory_index = 0usize;
                    #[allow(unused_mut)]
                    let mut accumulated = AccumulatedMouseReport::new_wide(mouse, enqueued_at);
                    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                    accumulated.restore_reversal_memory_and_process_first(
                        reversal_memories[memory_index],
                        #[cfg(feature = "rtt_diag")]
                        source,
                    );
                    accumulated
                }
                QueuedReportPayload::Hid(report) => {
                    if let Err(exit) = write_ble_hid_report(writer, &report, fail_closed, None).await {
                        return exit;
                    }
                    #[cfg(feature = "fixed_mouse_pacing_15ms")]
                    {
                        // Keyboard and consumer reports consume a host event
                        // just like mouse notifications.
                        next_mouse_slot = Some(fixed_mouse_pacing_deadline(Instant::now()));
                    }
                    continue;
                }
            }
        };

        // The control build intentionally offers at most one merged mouse
        // report per configured host interval. Waiting before draining lets
        // motion samples accumulate while keyboard/button edges remain
        // ordering boundaries. The baseline build compiles this block out.
        #[cfg(feature = "mouse_interval_control")]
        {
            let wait_started = Instant::now();
            if let Some(deadline) = next_mouse_slot {
                Timer::at(deadline).await;
            }
            #[cfg(feature = "rtt_diag")]
            crate::rtt_diag::record_mouse_slot_wait(Instant::now().duration_since(wait_started).as_micros() as u32);
            #[cfg(not(feature = "rtt_diag"))]
            let _ = wait_started;
        }

        // Healthy links send every report immediately (about 125 Hz on K:04).
        // If the previous GATT write stalled, producers will have queued several
        // adjacent motion samples; fold those samples into the next report before
        // writing again. Button edges remain ordering boundaries.
        let mut merged_reports = 0u32;
        // A previously deferred button/keyboard edge must stay ahead of any
        // reports that arrived after it while a large relative delta is being
        // emitted in multiple HID-sized chunks.
        #[cfg(any(
            feature = "mouse_realtime_age_cap_30ms",
            feature = "mouse_realtime_burst_budget_3",
            feature = "mouse_realtime_reversal_budget_3"
        ))]
        let retrying_frozen = retry_chunk_plan.is_some();
        #[cfg(not(any(
            feature = "mouse_realtime_age_cap_30ms",
            feature = "mouse_realtime_burst_budget_3",
            feature = "mouse_realtime_reversal_budget_3"
        )))]
        let retrying_frozen = false;
        if deferred_report.is_none() && !retrying_frozen {
            while let Ok(queued) = BLE_REPORT_CHANNEL.try_receive() {
                let mergeable = mouse.can_merge_payload(queued.payload());
                if mergeable {
                    let enqueued_at = queued.enqueued_at();
                    mouse.merge_payload(queued.into_payload(), enqueued_at);
                    merged_reports = merged_reports.saturating_add(1);
                } else {
                    deferred_report = Some(queued);
                    break;
                }
            }
        }

        #[cfg(not(any(
            feature = "mouse_realtime_age_cap_30ms",
            feature = "mouse_realtime_burst_budget_3",
            feature = "mouse_realtime_reversal_budget_3"
        )))]
        let mouse_diag = mouse.take_write_diag();
        #[cfg(any(
            feature = "mouse_realtime_age_cap_30ms",
            feature = "mouse_realtime_burst_budget_3",
            feature = "mouse_realtime_reversal_budget_3"
        ))]
        let mouse_diag_base = mouse.write_diag();
        #[cfg(not(any(
            feature = "mouse_realtime_age_cap_30ms",
            feature = "mouse_realtime_burst_budget_3",
            feature = "mouse_realtime_reversal_budget_3"
        )))]
        let (mouse_report, chunk_diag) = mouse.take_chunk();
        #[cfg(any(
            feature = "mouse_realtime_age_cap_30ms",
            feature = "mouse_realtime_burst_budget_3",
            feature = "mouse_realtime_reversal_budget_3"
        ))]
        let chunk_plan = retry_chunk_plan
            .take()
            .unwrap_or_else(|| mouse.prepare_chunk(Instant::now()));
        #[cfg(any(
            feature = "mouse_realtime_age_cap_30ms",
            feature = "mouse_realtime_burst_budget_3",
            feature = "mouse_realtime_reversal_budget_3"
        ))]
        let (mouse_report, chunk_diag) = (chunk_plan.report, chunk_plan.diag);
        #[cfg(any(
            feature = "mouse_realtime_age_cap_30ms",
            feature = "mouse_realtime_burst_budget_3",
            feature = "mouse_realtime_reversal_budget_3"
        ))]
        let mouse_diag = MouseWriteDiag {
            input_x: chunk_diag.input_x,
            input_y: chunk_diag.input_y,
            residual_x: chunk_diag.residual_x,
            residual_y: chunk_diag.residual_y,
            ..mouse_diag_base
        };
        #[cfg(not(feature = "rtt_diag"))]
        let _ = (chunk_diag, mouse_diag);
        #[cfg(not(feature = "mouse_ble_16bit_report"))]
        let report = crate::hid::Report::MouseReport(mouse_report);
        #[cfg(feature = "mouse_ble_16bit_report")]
        let report = crate::hid::BleMouse16Report {
            buttons: mouse_report.buttons,
            x: chunk_plan.emitted_x,
            y: chunk_plan.emitted_y,
            wheel: mouse_report.wheel,
            pan: mouse_report.pan,
        };
        #[cfg(not(any(
            feature = "mouse_realtime_age_cap_30ms",
            feature = "mouse_realtime_burst_budget_3",
            feature = "mouse_realtime_reversal_budget_3"
        )))]
        let has_residual = mouse.has_relative_motion();
        #[cfg(not(any(
            feature = "mouse_realtime_age_cap_30ms",
            feature = "mouse_realtime_burst_budget_3",
            feature = "mouse_realtime_reversal_budget_3"
        )))]
        if has_residual {
            pending_mouse = Some(mouse);
        }
        #[cfg(all(
            feature = "rtt_diag",
            not(any(
                feature = "mouse_realtime_age_cap_30ms",
                feature = "mouse_realtime_burst_budget_3",
                feature = "mouse_realtime_reversal_budget_3"
            ))
        ))]
        crate::rtt_diag::record_mouse_coalesce(
            merged_reports,
            has_residual,
            chunk_diag.input_x,
            chunk_diag.input_y,
            chunk_diag.residual_x,
            chunk_diag.residual_y,
        );
        #[cfg(all(feature = "mouse_interval_control", not(feature = "fixed_mouse_pacing_15ms")))]
        {
            next_mouse_slot = Some(Instant::now() + MOUSE_CONTROL_INTERVAL);
        }

        #[cfg(feature = "mouse_bounded_multi_notification_3")]
        {
            burst_slot = if burst_slot == u32::MAX { 1 } else { burst_slot + 1 };
        }
        #[cfg(feature = "mouse_ble_16bit_report")]
        let b8_write_started = Instant::now();
        #[cfg(not(feature = "mouse_ble_16bit_report"))]
        let write_result = write_ble_hid_report(writer, &report, fail_closed, Some(mouse_diag)).await;
        #[cfg(feature = "mouse_ble_16bit_report")]
        let write_result = write_ble_mouse16_report(writer, &report, fail_closed).await;
        #[cfg(feature = "mouse_ble_16bit_report")]
        let b8_write_us = Instant::now().duration_since(b8_write_started).as_micros() as u32;
        #[cfg(all(feature = "mouse_ble_16bit_report", not(feature = "rtt_diag")))]
        let _ = b8_write_us;
        if let Err(exit) = write_result {
            #[cfg(all(feature = "mouse_bounded_multi_notification_3", feature = "rtt_diag"))]
            crate::rtt_diag::record_mouse_burst_notification(
                burst_slot,
                1,
                false,
                &mouse_report,
                chunk_diag.residual_x,
                chunk_diag.residual_y,
                chunk_plan.remaining_stale_vectors,
                BLE_REPORT_CHANNEL.len(),
                5,
            );
            #[cfg(all(feature = "mouse_ble_16bit_report", feature = "rtt_diag"))]
            crate::rtt_diag::record_mouse_b8_handoff(
                false,
                &report,
                chunk_diag.input_x,
                chunk_diag.input_y,
                chunk_diag.residual_x,
                chunk_diag.residual_y,
                chunk_plan.age_us,
                mouse_diag.source_reports,
                mouse_diag.source,
                b8_write_us,
                BLE_REPORT_CHANNEL.len(),
            );
            #[cfg(any(
                feature = "mouse_realtime_age_cap_30ms",
                feature = "mouse_realtime_burst_budget_3",
                feature = "mouse_realtime_reversal_budget_3"
            ))]
            BLE_MOUSE_RETRY.signal(MouseRetry {
                mouse,
                plan: chunk_plan,
            });
            return exit;
        }
        #[cfg(all(feature = "mouse_ble_16bit_report", feature = "rtt_diag"))]
        crate::rtt_diag::record_mouse_b8_handoff(
            true,
            &report,
            chunk_diag.input_x,
            chunk_diag.input_y,
            chunk_diag.residual_x,
            chunk_diag.residual_y,
            chunk_plan.age_us,
            mouse_diag.source_reports,
            mouse_diag.source,
            b8_write_us,
            BLE_REPORT_CHANNEL.len(),
        );
        #[cfg(any(
            feature = "mouse_realtime_age_cap_30ms",
            feature = "mouse_realtime_burst_budget_3",
            feature = "mouse_realtime_reversal_budget_3"
        ))]
        {
            mouse.commit_chunk(chunk_plan);
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            {
                let memory = mouse.reversal_memory();
                reversal_memories[usize::from(memory.device_id.min(3))] = memory;
            }
            #[allow(unused_mut)]
            let mut has_residual = mouse.has_relative_motion();
            #[cfg(feature = "rtt_diag")]
            crate::rtt_diag::record_mouse_coalesce(
                merged_reports,
                has_residual,
                chunk_diag.input_x,
                chunk_diag.input_y,
                chunk_diag.residual_x,
                chunk_diag.residual_y,
            );
            #[cfg(feature = "rtt_diag")]
            if chunk_plan.stale_compress {
                crate::rtt_diag::record_stale_mouse_compress(
                    chunk_plan.dropped_x,
                    chunk_plan.dropped_y,
                    chunk_plan.age_us,
                );
            }
            #[cfg(all(feature = "rtt_diag", feature = "mouse_realtime_reversal_budget_3"))]
            {
                if chunk_plan.reversal_write_x {
                    crate::rtt_diag::record_mouse_reversal_write(
                        b'x',
                        chunk_plan.reversal_device_id,
                        mouse_report.x,
                        chunk_plan.reversal_first_seq_x,
                        chunk_plan.reversal_first_us_x,
                        chunk_plan.reversal_source_seq_x,
                        chunk_plan.reversal_source_us_x,
                    );
                }
                if chunk_plan.reversal_write_y {
                    crate::rtt_diag::record_mouse_reversal_write(
                        b'y',
                        chunk_plan.reversal_device_id,
                        mouse_report.y,
                        chunk_plan.reversal_first_seq_y,
                        chunk_plan.reversal_first_us_y,
                        chunk_plan.reversal_source_seq_y,
                        chunk_plan.reversal_source_us_y,
                    );
                }
                if chunk_plan.reversal_unconfirmed_x {
                    crate::rtt_diag::record_mouse_reversal_unconfirmed_flush(
                        b'x',
                        mouse_report.x,
                        chunk_plan.reversal_source_seq_x,
                        chunk_plan.reversal_source_us_x,
                    );
                }
                if chunk_plan.reversal_unconfirmed_y {
                    crate::rtt_diag::record_mouse_reversal_unconfirmed_flush(
                        b'y',
                        mouse_report.y,
                        chunk_plan.reversal_source_seq_y,
                        chunk_plan.reversal_source_us_y,
                    );
                }
            }
            #[cfg(feature = "mouse_bounded_multi_notification_3")]
            {
                let mut burst_index = 1u8;
                #[cfg(feature = "rtt_diag")]
                crate::rtt_diag::record_mouse_burst_notification(
                    burst_slot,
                    burst_index,
                    true,
                    &mouse_report,
                    chunk_diag.residual_x,
                    chunk_diag.residual_y,
                    mouse.stale_vectors_remaining,
                    BLE_REPORT_CHANNEL.len(),
                    if mouse.stale_vectors_remaining > 0 { 0 } else { 2 },
                );

                // Only a frozen stale epoch may use the immediate handoffs.
                // New/fresh residuals retain one notification per 15 ms slot.
                // No queue items are merged between members, so ordering is
                // preserved and any HID boundary waits for at most this
                // compile-time budget of three sequential stack handoffs.
                while continue_bounded_stale_burst(burst_index, mouse.stale_vectors_remaining) {
                    burst_index += 1;
                    let extra_plan = mouse.prepare_chunk(Instant::now());
                    let extra_report = extra_plan.report;
                    let extra_diag = extra_plan.diag;
                    let extra_mouse_diag_base = mouse.write_diag();
                    let extra_mouse_diag = MouseWriteDiag {
                        input_x: extra_diag.input_x,
                        input_y: extra_diag.input_y,
                        residual_x: extra_diag.residual_x,
                        residual_y: extra_diag.residual_y,
                        ..extra_mouse_diag_base
                    };
                    let extra_hid_report = crate::hid::Report::MouseReport(extra_report);
                    if let Err(exit) =
                        write_ble_hid_report(writer, &extra_hid_report, fail_closed, Some(extra_mouse_diag)).await
                    {
                        #[cfg(feature = "rtt_diag")]
                        crate::rtt_diag::record_mouse_burst_notification(
                            burst_slot,
                            burst_index,
                            false,
                            &extra_report,
                            extra_diag.residual_x,
                            extra_diag.residual_y,
                            extra_plan.remaining_stale_vectors,
                            BLE_REPORT_CHANNEL.len(),
                            5,
                        );
                        BLE_MOUSE_RETRY.signal(MouseRetry {
                            mouse,
                            plan: extra_plan,
                        });
                        return exit;
                    }

                    mouse.commit_chunk(extra_plan);
                    let memory = mouse.reversal_memory();
                    reversal_memories[usize::from(memory.device_id.min(3))] = memory;
                    has_residual = mouse.has_relative_motion();
                    #[cfg(feature = "rtt_diag")]
                    crate::rtt_diag::record_mouse_coalesce(
                        0,
                        has_residual,
                        extra_diag.input_x,
                        extra_diag.input_y,
                        extra_diag.residual_x,
                        extra_diag.residual_y,
                    );
                    #[cfg(feature = "rtt_diag")]
                    if extra_plan.stale_compress {
                        crate::rtt_diag::record_stale_mouse_compress(
                            extra_plan.dropped_x,
                            extra_plan.dropped_y,
                            extra_plan.age_us,
                        );
                    }
                    #[cfg(feature = "rtt_diag")]
                    {
                        if extra_plan.reversal_write_x {
                            crate::rtt_diag::record_mouse_reversal_write(
                                b'x',
                                extra_plan.reversal_device_id,
                                extra_report.x,
                                extra_plan.reversal_first_seq_x,
                                extra_plan.reversal_first_us_x,
                                extra_plan.reversal_source_seq_x,
                                extra_plan.reversal_source_us_x,
                            );
                        }
                        if extra_plan.reversal_write_y {
                            crate::rtt_diag::record_mouse_reversal_write(
                                b'y',
                                extra_plan.reversal_device_id,
                                extra_report.y,
                                extra_plan.reversal_first_seq_y,
                                extra_plan.reversal_first_us_y,
                                extra_plan.reversal_source_seq_y,
                                extra_plan.reversal_source_us_y,
                            );
                        }
                        if extra_plan.reversal_unconfirmed_x {
                            crate::rtt_diag::record_mouse_reversal_unconfirmed_flush(
                                b'x',
                                extra_report.x,
                                extra_plan.reversal_source_seq_x,
                                extra_plan.reversal_source_us_x,
                            );
                        }
                        if extra_plan.reversal_unconfirmed_y {
                            crate::rtt_diag::record_mouse_reversal_unconfirmed_flush(
                                b'y',
                                extra_report.y,
                                extra_plan.reversal_source_seq_y,
                                extra_plan.reversal_source_us_y,
                            );
                        }
                    }
                    #[cfg(feature = "rtt_diag")]
                    crate::rtt_diag::record_mouse_burst_notification(
                        burst_slot,
                        burst_index,
                        true,
                        &extra_report,
                        extra_diag.residual_x,
                        extra_diag.residual_y,
                        mouse.stale_vectors_remaining,
                        BLE_REPORT_CHANNEL.len(),
                        if burst_index == 3 {
                            1
                        } else if mouse.stale_vectors_remaining > 0 {
                            0
                        } else {
                            2
                        },
                    );
                }
            }
            if has_residual {
                pending_mouse = Some(mouse);
            }
        }
        #[cfg(feature = "fixed_mouse_pacing_15ms")]
        {
            // Start the next slot after completion so slow GATT writes cannot
            // create an expired deadline and an immediate catch-up report.
            next_mouse_slot = Some(fixed_mouse_pacing_deadline(Instant::now()));
        }
    }
}

#[cfg(feature = "mouse_bounded_multi_notification_3")]
fn continue_bounded_stale_burst(index: u8, stale_vectors_remaining: u8) -> bool {
    index < 3 && stale_vectors_remaining > 0
}

#[derive(Clone, Copy)]
struct MouseWriteDiag {
    oldest_enqueued_at: Instant,
    source_reports: u32,
    input_x: i32,
    input_y: i32,
    residual_x: i32,
    residual_y: i32,
    #[cfg(feature = "rtt_diag")]
    source: Option<crate::rtt_diag::MouseSourceMeta>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct MouseChunkDiag {
    input_x: i32,
    input_y: i32,
    residual_x: i32,
    residual_y: i32,
}

#[cfg(feature = "mouse_ble_16bit_report")]
async fn write_ble_mouse16_report<W>(
    writer: &mut W,
    report: &crate::hid::BleMouse16Report,
    fail_closed: bool,
) -> Result<(), BleKeyboardExit>
where
    W: HidWriterTrait<ReportType = crate::hid::Report>,
{
    let result = if fail_closed {
        match with_timeout(
            Duration::from_secs(HID_WRITE_TIMEOUT_SECS),
            writer.write_ble_mouse16_report(report),
        )
        .await
        {
            Ok(result) => result,
            Err(_) => {
                error!("Timed out sending BLE 16-bit mouse report");
                return Err(BleKeyboardExit::HidWriteStalled);
            }
        }
    } else {
        writer.write_ble_mouse16_report(report).await
    };
    if let Err(e) = result {
        error!("Failed to send BLE 16-bit mouse report: {:?}", e);
        return Err(BleKeyboardExit::HidWriteStalled);
    }
    Ok(())
}

async fn write_ble_hid_report<W>(
    writer: &mut W,
    report: &crate::hid::Report,
    fail_closed: bool,
    mouse_diag: Option<MouseWriteDiag>,
) -> Result<(), BleKeyboardExit>
where
    W: HidWriterTrait<ReportType = crate::hid::Report>,
{
    #[cfg(feature = "rtt_diag")]
    let diag_started = Instant::now();

    let result = if fail_closed {
        match with_timeout(Duration::from_secs(HID_WRITE_TIMEOUT_SECS), writer.write_report(report)).await {
            Ok(result) => result,
            Err(_) => {
                #[cfg(feature = "rtt_diag")]
                record_ble_hid_write_diag(report, diag_started, false, mouse_diag);
                error!("Timed out sending BLE HID report");
                return Err(BleKeyboardExit::HidWriteStalled);
            }
        }
    } else {
        writer.write_report(report).await
    };

    #[cfg(feature = "rtt_diag")]
    record_ble_hid_write_diag(report, diag_started, result.is_ok(), mouse_diag);

    #[cfg(not(feature = "rtt_diag"))]
    let _ = mouse_diag;

    if let Err(e) = result {
        if fail_closed
            || cfg!(any(
                feature = "mouse_realtime_age_cap_30ms",
                feature = "mouse_realtime_burst_budget_3",
                feature = "mouse_realtime_reversal_budget_3"
            ))
        {
            error!("Failed to send BLE HID report: {:?}", e);
            return Err(BleKeyboardExit::HidWriteStalled);
        }
        error!("Failed to send report: {:?}", e);
    }

    Ok(())
}

#[cfg(feature = "rtt_diag")]
fn record_ble_hid_write_diag(
    report: &crate::hid::Report,
    started_at: Instant,
    ok: bool,
    mouse_diag: Option<MouseWriteDiag>,
) {
    let completed_at = Instant::now();
    let (motion_age_us, source_reports) = mouse_diag
        .map(|diag| {
            (
                completed_at.duration_since(diag.oldest_enqueued_at).as_micros() as u32,
                diag.source_reports,
            )
        })
        .unwrap_or((0, 0));
    crate::rtt_diag::record_hid_write(
        report,
        completed_at.duration_since(started_at).as_micros() as u32,
        ok,
        BLE_REPORT_CHANNEL.len(),
        motion_age_us,
        source_reports,
        mouse_diag.map(|diag| (diag.input_x, diag.input_y, diag.residual_x, diag.residual_y)),
        mouse_diag.and_then(|diag| diag.source),
    );
}

/// Relative mouse fields are accumulated at a wider width, then emitted in
/// the minimum number of valid HID-sized chunks. This preserves total motion
/// without replaying every stale sample individually.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct AccumulatedMouseReport {
    buttons: u8,
    x: i32,
    y: i32,
    wheel: i32,
    pan: i32,
    oldest_enqueued_at: Instant,
    source_reports: u32,
    preserve_vector: bool,
    #[cfg(feature = "rtt_diag")]
    source: Option<crate::rtt_diag::MouseSourceMeta>,
    #[cfg(any(
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    stale_vectors_remaining: u8,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_x_candidate: i32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_y_candidate: i32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_x_samples: u8,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_y_samples: u8,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_x_write_pending: bool,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_y_write_pending: bool,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_x_direction: i8,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_y_direction: i8,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_x_source_seq: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_y_source_seq: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_x_source_us: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_y_source_us: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_x_confirmed_seq: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_y_confirmed_seq: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_x_confirmed_us: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_y_confirmed_us: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_x_first_seq: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_y_first_seq: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_x_first_us: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_y_first_us: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_x_candidate_flushed: bool,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_y_candidate_flushed: bool,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_device_id: u8,
}

/// Symmetric source-space deadband selected from B3/B4 captures. B3 false
/// candidates topped out at 70 counts; all 16 B4 physical reversal entries
/// began at 110 counts or above (median 139). A 100-count threshold preserves
/// all 16 entries with 30 counts of observed noise margin and remains below
/// the clipped local +127 boundary, giving i8 and wide i16 sources one rule.
#[cfg(feature = "mouse_realtime_reversal_budget_3")]
const REVERSAL_SOURCE_DEADBAND: u32 = 100;

/// A source axis starts a new gesture after one second without a meaningful
/// raw report.  Across all eight B5 hardware streams, the largest inactivity
/// that must remain inside a genuine reversal epoch was 848,541 us; the
/// smallest pre-main idle involved in a false confirmation was 8,387,451 us.
/// Thus 1 s lies inside the replay-proven inclusive safe range
/// 848,542..=8,387,451 us with margin on both observed boundaries.
#[cfg(feature = "mouse_realtime_reversal_budget_3")]
const REVERSAL_GESTURE_IDLE_US: u32 = 1_000_000;

#[cfg(feature = "mouse_realtime_reversal_budget_3")]
const REVERSAL_CONFIRM_WINDOW_US: u32 = 45_000;

#[cfg(feature = "mouse_realtime_reversal_budget_3")]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct ReversalMemory {
    x_candidate: i32,
    y_candidate: i32,
    x_samples: u8,
    y_samples: u8,
    x_flushed: bool,
    y_flushed: bool,
    x_direction: i8,
    y_direction: i8,
    x_seq: u32,
    y_seq: u32,
    x_us: u32,
    y_us: u32,
    x_confirmed_seq: u32,
    y_confirmed_seq: u32,
    x_confirmed_us: u32,
    y_confirmed_us: u32,
    x_first_seq: u32,
    y_first_seq: u32,
    x_first_us: u32,
    y_first_us: u32,
    device_id: u8,
}

#[cfg(feature = "mouse_realtime_reversal_budget_3")]
fn meaningful_source_sign(value: i32) -> i8 {
    if value.unsigned_abs() < REVERSAL_SOURCE_DEADBAND {
        0
    } else if value < 0 {
        -1
    } else {
        1
    }
}

#[cfg(any(
    feature = "mouse_realtime_age_cap_30ms",
    feature = "mouse_realtime_burst_budget_3",
    feature = "mouse_realtime_reversal_budget_3"
))]
static BLE_MOUSE_RETRY: Signal<crate::RawMutex, MouseRetry> = Signal::new();

#[cfg(any(
    feature = "mouse_realtime_age_cap_30ms",
    feature = "mouse_realtime_burst_budget_3",
    feature = "mouse_realtime_reversal_budget_3"
))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct MouseChunkPlan {
    report: MouseReport,
    #[cfg(feature = "mouse_ble_16bit_report")]
    emitted_x: i16,
    #[cfg(feature = "mouse_ble_16bit_report")]
    emitted_y: i16,
    diag: MouseChunkDiag,
    remaining_x: i32,
    remaining_y: i32,
    remaining_wheel: i32,
    remaining_pan: i32,
    remaining_oldest_enqueued_at: Instant,
    stale_compress: bool,
    #[cfg(any(
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    remaining_stale_vectors: u8,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    commit_reversal_x: bool,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    commit_reversal_y: bool,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_write_x: bool,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_write_y: bool,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_unconfirmed_x: bool,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_unconfirmed_y: bool,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_source_seq_x: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_source_seq_y: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_source_us_x: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_source_us_y: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_first_seq_x: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_first_seq_y: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_first_us_x: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_first_us_y: u32,
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    reversal_device_id: u8,
    dropped_x: u32,
    dropped_y: u32,
    age_us: u32,
}

#[cfg(any(
    feature = "mouse_realtime_age_cap_30ms",
    feature = "mouse_realtime_burst_budget_3",
    feature = "mouse_realtime_reversal_budget_3"
))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct MouseRetry {
    mouse: AccumulatedMouseReport,
    plan: MouseChunkPlan,
}

impl AccumulatedMouseReport {
    fn new(report: MouseReport, enqueued_at: Instant) -> Self {
        Self {
            buttons: report.buttons,
            x: i32::from(report.x),
            y: i32::from(report.y),
            wheel: i32::from(report.wheel),
            pan: i32::from(report.pan),
            oldest_enqueued_at: enqueued_at,
            source_reports: 1,
            preserve_vector: cfg!(feature = "mouse_vector_preserve"),
            #[cfg(feature = "rtt_diag")]
            source: None,
            #[cfg(any(
                feature = "mouse_realtime_burst_budget_3",
                feature = "mouse_realtime_reversal_budget_3"
            ))]
            stale_vectors_remaining: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_candidate: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_candidate: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_samples: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_samples: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_write_pending: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_write_pending: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_direction: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_direction: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_source_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_source_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_source_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_source_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_confirmed_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_confirmed_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_confirmed_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_confirmed_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_first_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_first_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_first_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_first_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_candidate_flushed: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_candidate_flushed: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_device_id: 0,
        }
    }

    fn new_wide(report: WideMouseReport, enqueued_at: Instant) -> Self {
        Self {
            buttons: report.buttons,
            x: report.x,
            y: report.y,
            wheel: report.wheel,
            pan: report.pan,
            oldest_enqueued_at: enqueued_at,
            source_reports: 1,
            // The old pointing path vector-chunked every native i16 event
            // before enqueueing. Preserve that behavior after moving the
            // chunking into the transport writer.
            preserve_vector: true,
            #[cfg(feature = "rtt_diag")]
            source: report.source,
            #[cfg(any(
                feature = "mouse_realtime_burst_budget_3",
                feature = "mouse_realtime_reversal_budget_3"
            ))]
            stale_vectors_remaining: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_candidate: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_candidate: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_samples: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_samples: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_write_pending: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_write_pending: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_direction: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_direction: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_source_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_source_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_source_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_source_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_confirmed_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_confirmed_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_confirmed_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_confirmed_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_first_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_first_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_first_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_first_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_candidate_flushed: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_candidate_flushed: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_device_id: 0,
        }
    }

    fn can_merge(&self, report: &MouseReport) -> bool {
        self.buttons == report.buttons
    }

    fn can_merge_payload(&self, payload: &QueuedReportPayload) -> bool {
        match payload {
            QueuedReportPayload::Hid(crate::hid::Report::MouseReport(report)) => self.can_merge(report),
            QueuedReportPayload::WideMouse(report) => self.buttons == report.buttons,
            QueuedReportPayload::Hid(_) => false,
        }
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    fn reversal_memory(&self) -> ReversalMemory {
        ReversalMemory {
            x_candidate: self.reversal_x_candidate,
            y_candidate: self.reversal_y_candidate,
            x_samples: self.reversal_x_samples,
            y_samples: self.reversal_y_samples,
            x_flushed: self.reversal_x_candidate_flushed,
            y_flushed: self.reversal_y_candidate_flushed,
            x_direction: self.reversal_x_direction,
            y_direction: self.reversal_y_direction,
            x_seq: self.reversal_x_source_seq,
            y_seq: self.reversal_y_source_seq,
            x_us: self.reversal_x_source_us,
            y_us: self.reversal_y_source_us,
            x_confirmed_seq: self.reversal_x_confirmed_seq,
            y_confirmed_seq: self.reversal_y_confirmed_seq,
            x_confirmed_us: self.reversal_x_confirmed_us,
            y_confirmed_us: self.reversal_y_confirmed_us,
            x_first_seq: self.reversal_x_first_seq,
            y_first_seq: self.reversal_y_first_seq,
            x_first_us: self.reversal_x_first_us,
            y_first_us: self.reversal_y_first_us,
            device_id: self.reversal_device_id,
        }
    }

    /// Restore committed per-source history before feeding the first report
    /// of a new aggregate.  Only immutable raw source metadata is detector
    /// evidence; transformed movement is kept separately for HID output.
    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    fn restore_reversal_memory_and_process_first(
        &mut self,
        memory: ReversalMemory,
        #[cfg(feature = "rtt_diag")] source: Option<crate::rtt_diag::MouseSourceMeta>,
    ) {
        #[cfg(feature = "rtt_diag")]
        let source_device = source.map(|meta| meta.device_id).unwrap_or(memory.device_id);
        #[cfg(not(feature = "rtt_diag"))]
        let source_device = memory.device_id;
        #[cfg(feature = "rtt_diag")]
        let (current_source_seq, current_source_us, raw_x, raw_y) = source
            .map(|meta| (meta.seq, meta.timestamp_us, Some(meta.raw_x), Some(meta.raw_y)))
            .unwrap_or((0, 0, None, None));
        #[cfg(not(feature = "rtt_diag"))]
        let (current_source_seq, current_source_us, raw_x, raw_y) = (0, 0, None, None);
        let memory = if memory.device_id == 0 || memory.device_id == source_device {
            memory
        } else {
            ReversalMemory {
                device_id: source_device,
                ..ReversalMemory::default()
            }
        };
        let first_x = core::mem::take(&mut self.x);
        let first_y = core::mem::take(&mut self.y);
        self.reversal_x_candidate = memory.x_candidate;
        self.reversal_y_candidate = memory.y_candidate;
        self.reversal_x_samples = memory.x_samples;
        self.reversal_y_samples = memory.y_samples;
        self.reversal_x_candidate_flushed = memory.x_flushed;
        self.reversal_y_candidate_flushed = memory.y_flushed;
        self.reversal_x_direction = memory.x_direction;
        self.reversal_y_direction = memory.y_direction;
        self.reversal_x_source_seq = memory.x_seq;
        self.reversal_y_source_seq = memory.y_seq;
        self.reversal_x_source_us = memory.x_us;
        self.reversal_y_source_us = memory.y_us;
        self.reversal_x_confirmed_seq = memory.x_confirmed_seq;
        self.reversal_y_confirmed_seq = memory.y_confirmed_seq;
        self.reversal_x_confirmed_us = memory.x_confirmed_us;
        self.reversal_y_confirmed_us = memory.y_confirmed_us;
        self.reversal_x_first_seq = memory.x_first_seq;
        self.reversal_y_first_seq = memory.y_first_seq;
        self.reversal_x_first_us = memory.x_first_us;
        self.reversal_y_first_us = memory.y_first_us;
        self.reversal_device_id = source_device;
        Self::merge_reversal_axis(
            &mut self.x,
            &mut self.reversal_x_candidate,
            &mut self.reversal_x_samples,
            &mut self.reversal_x_candidate_flushed,
            &mut self.reversal_x_write_pending,
            &mut self.reversal_x_direction,
            &mut self.reversal_x_source_seq,
            &mut self.reversal_x_source_us,
            &mut self.reversal_x_confirmed_seq,
            &mut self.reversal_x_confirmed_us,
            &mut self.reversal_x_first_seq,
            &mut self.reversal_x_first_us,
            first_x,
            raw_x,
            b'x',
            source_device,
            current_source_seq,
            current_source_us,
        );
        Self::merge_reversal_axis(
            &mut self.y,
            &mut self.reversal_y_candidate,
            &mut self.reversal_y_samples,
            &mut self.reversal_y_candidate_flushed,
            &mut self.reversal_y_write_pending,
            &mut self.reversal_y_direction,
            &mut self.reversal_y_source_seq,
            &mut self.reversal_y_source_us,
            &mut self.reversal_y_confirmed_seq,
            &mut self.reversal_y_confirmed_us,
            &mut self.reversal_y_first_seq,
            &mut self.reversal_y_first_us,
            first_y,
            raw_y,
            b'y',
            source_device,
            current_source_seq,
            current_source_us,
        );
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    #[allow(clippy::too_many_arguments)]
    fn merge_reversal_axis(
        residual: &mut i32,
        candidate: &mut i32,
        samples: &mut u8,
        candidate_flushed: &mut bool,
        write_pending: &mut bool,
        source_direction: &mut i8,
        source_seq: &mut u32,
        source_us: &mut u32,
        confirmed_seq: &mut u32,
        confirmed_us: &mut u32,
        first_seq: &mut u32,
        first_us: &mut u32,
        motion_input: i32,
        raw_input: Option<i32>,
        axis: u8,
        device: u8,
        current_source_seq: u32,
        current_source_us: u32,
    ) {
        #[cfg(not(feature = "rtt_diag"))]
        let _ = (axis, device);
        let Some(raw_input) = raw_input else {
            *residual = residual.saturating_add(motion_input);
            return;
        };
        if *source_seq != 0 && current_source_seq == *source_seq {
            // A duplicate never advances evidence.  Preserve the historical
            // B2 movement semantics rather than silently dropping its bytes.
            *residual = residual.saturating_add(motion_input);
            return;
        }

        let input_direction = meaningful_source_sign(raw_input);
        if input_direction == 0 {
            // Sub-deadband raw reports are movement only.  Neither their
            // transformed values nor an aggregate total can seed history.
            // They still advance the observed source sequence so an ordinary
            // quiet report between meaningful samples is not a transport gap.
            *residual = residual.saturating_add(motion_input);
            *source_seq = current_source_seq;
            return;
        }

        let idle = *source_us != 0
            && current_source_us != 0
            && current_source_us.wrapping_sub(*source_us) >= REVERSAL_GESTURE_IDLE_US;
        if idle {
            if *candidate != 0 && !*candidate_flushed {
                *residual = residual.saturating_add(*candidate);
            }
            *candidate = 0;
            *samples = 0;
            *candidate_flushed = false;
            *write_pending = false;
            *confirmed_seq = 0;
            *confirmed_us = 0;
            *first_seq = 0;
            *first_us = 0;
            *source_direction = input_direction;
            *confirmed_seq = 0;
            *confirmed_us = 0;
            *source_seq = current_source_seq;
            *source_us = current_source_us;
            *residual = residual.saturating_add(motion_input);
            #[cfg(feature = "rtt_diag")]
            crate::rtt_diag::record_mouse_reversal_epoch(axis, device, current_source_seq, current_source_us);
            return;
        }

        if *write_pending {
            if input_direction != *source_direction {
                *candidate = candidate.saturating_add(motion_input);
            } else {
                *residual = residual.saturating_add(motion_input);
            }
            *source_seq = current_source_seq;
            *source_us = current_source_us;
            return;
        }

        if *source_direction == 0 {
            // First meaningful report of an epoch is baseline only.
            *source_direction = input_direction;
            *confirmed_seq = 0;
            *confirmed_us = 0;
            *source_seq = current_source_seq;
            *source_us = current_source_us;
            *residual = residual.saturating_add(motion_input);
            return;
        }

        if *candidate != 0 {
            if input_direction != *source_direction {
                let expected_seq = if *source_seq == u32::MAX {
                    1
                } else {
                    source_seq.wrapping_add(1)
                };
                let contiguous = *source_seq == 0 || current_source_seq == 0 || current_source_seq == expected_seq;
                let timely = *source_us == 0
                    || current_source_us == 0
                    || current_source_us.wrapping_sub(*source_us) <= REVERSAL_CONFIRM_WINDOW_US;
                if !contiguous || !timely {
                    #[cfg(feature = "rtt_diag")]
                    crate::rtt_diag::record_mouse_reversal_cancel(
                        axis,
                        device,
                        *first_seq,
                        *first_us,
                        current_source_seq,
                        current_source_us,
                    );
                    if !*candidate_flushed {
                        *residual = residual.saturating_add(*candidate);
                    }
                    *candidate = motion_input;
                    *samples = 1;
                    *candidate_flushed = false;
                    *confirmed_seq = 0;
                    *confirmed_us = 0;
                    *first_seq = current_source_seq;
                    *first_us = current_source_us;
                    *source_seq = current_source_seq;
                    *source_us = current_source_us;
                    #[cfg(feature = "rtt_diag")]
                    crate::rtt_diag::record_mouse_reversal_candidate(
                        axis, device, *residual, raw_input, *first_seq, *first_us,
                    );
                    return;
                }
                *candidate = if *candidate_flushed {
                    motion_input
                } else {
                    candidate.saturating_add(motion_input)
                };
                *samples = samples.saturating_add(1);
                *candidate_flushed = false;
                *source_seq = current_source_seq;
                *source_us = current_source_us;
                if *samples >= 2 {
                    *write_pending = true;
                    *confirmed_seq = current_source_seq;
                    *confirmed_us = current_source_us;
                    #[cfg(feature = "rtt_diag")]
                    crate::rtt_diag::record_mouse_reversal_confirm(
                        axis,
                        device,
                        *candidate,
                        *samples,
                        *first_seq,
                        *first_us,
                        current_source_seq,
                        current_source_us,
                    );
                }
            } else {
                if !*candidate_flushed {
                    *residual = residual.saturating_add(*candidate);
                }
                *residual = residual.saturating_add(motion_input);
                #[cfg(feature = "rtt_diag")]
                crate::rtt_diag::record_mouse_reversal_cancel(
                    axis,
                    device,
                    *first_seq,
                    *first_us,
                    current_source_seq,
                    current_source_us,
                );
                *candidate = 0;
                *samples = 0;
                *candidate_flushed = false;
                *confirmed_seq = 0;
                *confirmed_us = 0;
                *first_seq = 0;
                *first_us = 0;
                *source_seq = current_source_seq;
                *source_us = current_source_us;
            }
            return;
        }

        if input_direction != *source_direction {
            *candidate = motion_input;
            *samples = 1;
            *candidate_flushed = false;
            *confirmed_seq = 0;
            *confirmed_us = 0;
            *first_seq = current_source_seq;
            *first_us = current_source_us;
            *source_seq = current_source_seq;
            *source_us = current_source_us;
            #[cfg(feature = "rtt_diag")]
            crate::rtt_diag::record_mouse_reversal_candidate(axis, device, *residual, raw_input, *first_seq, *first_us);
        } else {
            *residual = residual.saturating_add(motion_input);
            *source_seq = current_source_seq;
            *source_us = current_source_us;
        }
    }

    fn merge(&mut self, report: MouseReport, enqueued_at: Instant) {
        debug_assert!(self.can_merge(&report));
        #[cfg(not(feature = "mouse_realtime_reversal_budget_3"))]
        {
            self.x = self.x.saturating_add(i32::from(report.x));
            self.y = self.y.saturating_add(i32::from(report.y));
        }
        #[cfg(feature = "mouse_realtime_reversal_budget_3")]
        {
            // Generic HID reports have no immutable optical source identity.
            // They remain lossless movement but are never detector evidence.
            self.x = self.x.saturating_add(i32::from(report.x));
            self.y = self.y.saturating_add(i32::from(report.y));
        }
        self.wheel = self.wheel.saturating_add(i32::from(report.wheel));
        self.pan = self.pan.saturating_add(i32::from(report.pan));
        self.oldest_enqueued_at = self.oldest_enqueued_at.min(enqueued_at);
        self.source_reports = self.source_reports.saturating_add(1);
    }

    fn merge_wide(&mut self, report: WideMouseReport, enqueued_at: Instant) {
        debug_assert_eq!(self.buttons, report.buttons);
        #[cfg(feature = "rtt_diag")]
        let source = report.source;
        #[cfg(feature = "rtt_diag")]
        if source.is_some() {
            self.source = source;
        }
        #[cfg(all(feature = "rtt_diag", feature = "mouse_realtime_reversal_budget_3"))]
        let (current_source_seq, current_source_us, raw_x, raw_y, source_device) = source
            .map(|meta| {
                (
                    meta.seq,
                    meta.timestamp_us,
                    Some(meta.raw_x),
                    Some(meta.raw_y),
                    meta.device_id,
                )
            })
            .unwrap_or((0, 0, None, None, self.reversal_device_id));
        #[cfg(all(not(feature = "rtt_diag"), feature = "mouse_realtime_reversal_budget_3"))]
        let (current_source_seq, current_source_us, raw_x, raw_y, source_device) =
            (0, 0, None, None, self.reversal_device_id);
        #[cfg(not(feature = "mouse_realtime_reversal_budget_3"))]
        {
            self.x = self.x.saturating_add(report.x);
            self.y = self.y.saturating_add(report.y);
        }
        #[cfg(feature = "mouse_realtime_reversal_budget_3")]
        {
            Self::merge_reversal_axis(
                &mut self.x,
                &mut self.reversal_x_candidate,
                &mut self.reversal_x_samples,
                &mut self.reversal_x_candidate_flushed,
                &mut self.reversal_x_write_pending,
                &mut self.reversal_x_direction,
                &mut self.reversal_x_source_seq,
                &mut self.reversal_x_source_us,
                &mut self.reversal_x_confirmed_seq,
                &mut self.reversal_x_confirmed_us,
                &mut self.reversal_x_first_seq,
                &mut self.reversal_x_first_us,
                report.x,
                raw_x,
                b'x',
                source_device,
                current_source_seq,
                current_source_us,
            );
            Self::merge_reversal_axis(
                &mut self.y,
                &mut self.reversal_y_candidate,
                &mut self.reversal_y_samples,
                &mut self.reversal_y_candidate_flushed,
                &mut self.reversal_y_write_pending,
                &mut self.reversal_y_direction,
                &mut self.reversal_y_source_seq,
                &mut self.reversal_y_source_us,
                &mut self.reversal_y_confirmed_seq,
                &mut self.reversal_y_confirmed_us,
                &mut self.reversal_y_first_seq,
                &mut self.reversal_y_first_us,
                report.y,
                raw_y,
                b'y',
                source_device,
                current_source_seq,
                current_source_us,
            );
        }
        self.wheel = self.wheel.saturating_add(report.wheel);
        self.pan = self.pan.saturating_add(report.pan);
        self.oldest_enqueued_at = self.oldest_enqueued_at.min(enqueued_at);
        self.source_reports = self.source_reports.saturating_add(1);
        self.preserve_vector = true;
    }

    fn merge_payload(&mut self, payload: QueuedReportPayload, enqueued_at: Instant) {
        match payload {
            QueuedReportPayload::Hid(crate::hid::Report::MouseReport(report)) => self.merge(report, enqueued_at),
            QueuedReportPayload::WideMouse(report) => self.merge_wide(report, enqueued_at),
            QueuedReportPayload::Hid(_) => unreachable!("non-mouse payload passed the merge boundary"),
        }
    }

    fn take_write_diag(&mut self) -> MouseWriteDiag {
        let diag = MouseWriteDiag {
            oldest_enqueued_at: self.oldest_enqueued_at,
            source_reports: self.source_reports,
            input_x: 0,
            input_y: 0,
            residual_x: 0,
            residual_y: 0,
            #[cfg(feature = "rtt_diag")]
            source: self.source,
        };
        self.source_reports = 0;
        diag
    }

    #[cfg(any(
        feature = "mouse_realtime_age_cap_30ms",
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    fn write_diag(&self) -> MouseWriteDiag {
        MouseWriteDiag {
            oldest_enqueued_at: self.oldest_enqueued_at,
            source_reports: self.source_reports,
            input_x: 0,
            input_y: 0,
            residual_x: 0,
            residual_y: 0,
            #[cfg(feature = "rtt_diag")]
            source: self.source,
        }
    }

    #[cfg(any(
        feature = "mouse_realtime_age_cap_30ms",
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    #[cfg(feature = "mouse_ble_16bit_report")]
    fn prepare_chunk(&self, now: Instant) -> MouseChunkPlan {
        // Reuse the proven B6 reversal decision/identity metadata, then widen
        // only the BLE transport chunk. The legacy plan's diag.input fields
        // are the effective post-acceleration values selected for this write
        // before i8 retirement.
        let mut plan = self.prepare_chunk_8(now);
        plan.emitted_x = plan.diag.input_x.clamp(-32_767, 32_767) as i16;
        plan.emitted_y = plan.diag.input_y.clamp(-32_767, 32_767) as i16;
        plan.remaining_x = plan.diag.input_x - i32::from(plan.emitted_x);
        plan.remaining_y = plan.diag.input_y - i32::from(plan.emitted_y);
        plan.diag.residual_x = plan.remaining_x;
        plan.diag.residual_y = plan.remaining_y;
        plan.remaining_oldest_enqueued_at = if plan.remaining_x == 0 && plan.remaining_y == 0 {
            now
        } else {
            self.oldest_enqueued_at
        };
        plan.stale_compress = false;
        plan.remaining_stale_vectors = 0;
        plan.dropped_x = 0;
        plan.dropped_y = 0;
        plan
    }

    #[cfg(all(
        not(feature = "mouse_ble_16bit_report"),
        any(
            feature = "mouse_realtime_age_cap_30ms",
            feature = "mouse_realtime_burst_budget_3",
            feature = "mouse_realtime_reversal_budget_3"
        )
    ))]
    fn prepare_chunk(&self, now: Instant) -> MouseChunkPlan {
        self.prepare_chunk_8(now)
    }

    #[cfg(any(
        feature = "mouse_realtime_age_cap_30ms",
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    fn prepare_chunk_8(&self, now: Instant) -> MouseChunkPlan {
        const AGE_CAP_US: u32 = 30_000;

        fn proportional_xy(x: i32, y: i32, max_component: u32) -> (i32, i32) {
            let max_abs = x.unsigned_abs().max(y.unsigned_abs());
            debug_assert!(max_abs > 0);
            let scale = |value: i32| -> i32 {
                if value == 0 {
                    return 0;
                }
                let rounded = (u64::from(value.unsigned_abs()) * u64::from(max_component) + u64::from(max_abs) / 2)
                    / u64::from(max_abs);
                let magnitude = rounded.clamp(1, u64::from(max_component)) as i32;
                if value < 0 { -magnitude } else { magnitude }
            };
            (scale(x), scale(y))
        }

        #[cfg(feature = "mouse_realtime_reversal_budget_3")]
        let unconfirmed_flush_x = self.x == 0
            && self.reversal_x_candidate != 0
            && !self.reversal_x_write_pending
            && !self.reversal_x_candidate_flushed;
        #[cfg(feature = "mouse_realtime_reversal_budget_3")]
        let unconfirmed_flush_y = self.y == 0
            && self.reversal_y_candidate != 0
            && !self.reversal_y_write_pending
            && !self.reversal_y_candidate_flushed;
        #[cfg(feature = "mouse_realtime_reversal_budget_3")]
        let commit_reversal_x = self.reversal_x_write_pending;
        #[cfg(feature = "mouse_realtime_reversal_budget_3")]
        let commit_reversal_y = self.reversal_y_write_pending;
        #[cfg(not(feature = "mouse_realtime_reversal_budget_3"))]
        let commit_reversal_x = false;
        #[cfg(not(feature = "mouse_realtime_reversal_budget_3"))]
        let commit_reversal_y = false;
        #[cfg(feature = "mouse_realtime_reversal_budget_3")]
        let effective_x = if commit_reversal_x || unconfirmed_flush_x {
            self.reversal_x_candidate
        } else {
            self.x
        };
        #[cfg(not(feature = "mouse_realtime_reversal_budget_3"))]
        let effective_x = self.x;
        #[cfg(feature = "mouse_realtime_reversal_budget_3")]
        let effective_y = if commit_reversal_y || unconfirmed_flush_y {
            self.reversal_y_candidate
        } else {
            self.y
        };
        #[cfg(not(feature = "mouse_realtime_reversal_budget_3"))]
        let effective_y = self.y;

        let age_us = now
            .duration_since(self.oldest_enqueued_at)
            .as_micros()
            .min(u64::from(u32::MAX)) as u32;
        let oversized_xy = effective_x < i32::from(i8::MIN)
            || effective_x > i32::from(i8::MAX)
            || effective_y < i32::from(i8::MIN)
            || effective_y > i32::from(i8::MAX);

        #[cfg(feature = "mouse_realtime_age_cap_30ms")]
        if age_us > AGE_CAP_US && oversized_xy {
            let (x, y) = proportional_xy(effective_x, effective_y, 127);
            let x = x as i8;
            let y = y as i8;
            let wheel = self.wheel.clamp(i8::MIN as i32, i8::MAX as i32) as i8;
            let pan = self.pan.clamp(i8::MIN as i32, i8::MAX as i32) as i8;
            return MouseChunkPlan {
                #[cfg(feature = "mouse_ble_16bit_report")]
                emitted_x: 0,
                #[cfg(feature = "mouse_ble_16bit_report")]
                emitted_y: 0,
                report: MouseReport {
                    buttons: self.buttons,
                    x,
                    y,
                    wheel,
                    pan,
                },
                diag: MouseChunkDiag {
                    input_x: effective_x,
                    input_y: effective_y,
                    residual_x: 0,
                    residual_y: 0,
                },
                remaining_x: 0,
                remaining_y: 0,
                remaining_wheel: self.wheel - i32::from(wheel),
                remaining_pan: self.pan - i32::from(pan),
                remaining_oldest_enqueued_at: now,
                stale_compress: true,
                dropped_x: effective_x.unsigned_abs().saturating_sub(i32::from(x).unsigned_abs()),
                dropped_y: effective_y.unsigned_abs().saturating_sub(i32::from(y).unsigned_abs()),
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                commit_reversal_x,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                commit_reversal_y,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_write_x: self.reversal_x_write_pending,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_write_y: self.reversal_y_write_pending,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_unconfirmed_x: unconfirmed_flush_x,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_unconfirmed_y: unconfirmed_flush_y,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_source_seq_x: if self.reversal_x_write_pending {
                    self.reversal_x_confirmed_seq
                } else {
                    self.reversal_x_source_seq
                },
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_source_seq_y: if self.reversal_y_write_pending {
                    self.reversal_y_confirmed_seq
                } else {
                    self.reversal_y_source_seq
                },
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_source_us_x: if self.reversal_x_write_pending {
                    self.reversal_x_confirmed_us
                } else {
                    self.reversal_x_source_us
                },
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_source_us_y: if self.reversal_y_write_pending {
                    self.reversal_y_confirmed_us
                } else {
                    self.reversal_y_source_us
                },
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_first_seq_x: self.reversal_x_first_seq,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_first_seq_y: self.reversal_y_first_seq,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_first_us_x: self.reversal_x_first_us,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_first_us_y: self.reversal_y_first_us,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_device_id: self.reversal_device_id,
                age_us,
            };
        }

        #[cfg(any(
            feature = "mouse_realtime_burst_budget_3",
            feature = "mouse_realtime_reversal_budget_3"
        ))]
        if self.stale_vectors_remaining > 0 && effective_x == 0 && effective_y == 0 {
            let mut remainder = *self;
            remainder.x = effective_x;
            remainder.y = effective_y;
            remainder.stale_vectors_remaining = 0;
            let (report, diag) = remainder.take_chunk();
            return MouseChunkPlan {
                report,
                #[cfg(feature = "mouse_ble_16bit_report")]
                emitted_x: 0,
                #[cfg(feature = "mouse_ble_16bit_report")]
                emitted_y: 0,
                diag,
                remaining_x: remainder.x,
                remaining_y: remainder.y,
                remaining_wheel: remainder.wheel,
                remaining_pan: remainder.pan,
                remaining_oldest_enqueued_at: now,
                stale_compress: true,
                remaining_stale_vectors: 0,
                dropped_x: 0,
                dropped_y: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                commit_reversal_x,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                commit_reversal_y,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_write_x: self.reversal_x_write_pending,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_write_y: self.reversal_y_write_pending,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_unconfirmed_x: unconfirmed_flush_x,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_unconfirmed_y: unconfirmed_flush_y,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_source_seq_x: if self.reversal_x_write_pending {
                    self.reversal_x_confirmed_seq
                } else {
                    self.reversal_x_source_seq
                },
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_source_seq_y: if self.reversal_y_write_pending {
                    self.reversal_y_confirmed_seq
                } else {
                    self.reversal_y_source_seq
                },
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_source_us_x: if self.reversal_x_write_pending {
                    self.reversal_x_confirmed_us
                } else {
                    self.reversal_x_source_us
                },
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_source_us_y: if self.reversal_y_write_pending {
                    self.reversal_y_confirmed_us
                } else {
                    self.reversal_y_source_us
                },
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_first_seq_x: self.reversal_x_first_seq,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_first_seq_y: self.reversal_y_first_seq,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_first_us_x: self.reversal_x_first_us,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_first_us_y: self.reversal_y_first_us,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_device_id: self.reversal_device_id,
                age_us,
            };
        }

        #[cfg(any(
            feature = "mouse_realtime_burst_budget_3",
            feature = "mouse_realtime_reversal_budget_3"
        ))]
        if self.stale_vectors_remaining > 0 || (age_us > AGE_CAP_US && oversized_xy) {
            let budget = if self.stale_vectors_remaining == 0 {
                3
            } else {
                self.stale_vectors_remaining
            };
            let retained_cap = u32::from(budget) * 127;
            let retained_target = effective_x
                .unsigned_abs()
                .max(effective_y.unsigned_abs())
                .min(retained_cap);
            let (retained_x, retained_y) = proportional_xy(effective_x, effective_y, retained_target);
            let (x, y) = proportional_xy(
                retained_x,
                retained_y,
                retained_x.unsigned_abs().max(retained_y.unsigned_abs()).min(127),
            );
            let x = x as i8;
            let y = y as i8;
            let wheel = self.wheel.clamp(i8::MIN as i32, i8::MAX as i32) as i8;
            let pan = self.pan.clamp(i8::MIN as i32, i8::MAX as i32) as i8;
            let remaining_stale_vectors = budget - 1;
            let remaining_x = if remaining_stale_vectors == 0 {
                0
            } else {
                retained_x - i32::from(x)
            };
            let remaining_y = if remaining_stale_vectors == 0 {
                0
            } else {
                retained_y - i32::from(y)
            };
            return MouseChunkPlan {
                #[cfg(feature = "mouse_ble_16bit_report")]
                emitted_x: 0,
                #[cfg(feature = "mouse_ble_16bit_report")]
                emitted_y: 0,
                report: MouseReport {
                    buttons: self.buttons,
                    x,
                    y,
                    wheel,
                    pan,
                },
                diag: MouseChunkDiag {
                    input_x: effective_x,
                    input_y: effective_y,
                    residual_x: remaining_x,
                    residual_y: remaining_y,
                },
                remaining_x,
                remaining_y,
                remaining_wheel: self.wheel - i32::from(wheel),
                remaining_pan: self.pan - i32::from(pan),
                remaining_oldest_enqueued_at: if remaining_stale_vectors == 0
                    || cfg!(feature = "mouse_realtime_reversal_budget_3") && (commit_reversal_x || commit_reversal_y)
                {
                    now
                } else {
                    self.oldest_enqueued_at
                },
                stale_compress: true,
                remaining_stale_vectors,
                dropped_x: effective_x
                    .unsigned_abs()
                    .saturating_sub(i32::from(x).unsigned_abs().saturating_add(remaining_x.unsigned_abs())),
                dropped_y: effective_y
                    .unsigned_abs()
                    .saturating_sub(i32::from(y).unsigned_abs().saturating_add(remaining_y.unsigned_abs())),
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                commit_reversal_x,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                commit_reversal_y,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_write_x: self.reversal_x_write_pending,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_write_y: self.reversal_y_write_pending,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_unconfirmed_x: unconfirmed_flush_x,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_unconfirmed_y: unconfirmed_flush_y,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_source_seq_x: if self.reversal_x_write_pending {
                    self.reversal_x_confirmed_seq
                } else {
                    self.reversal_x_source_seq
                },
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_source_seq_y: if self.reversal_y_write_pending {
                    self.reversal_y_confirmed_seq
                } else {
                    self.reversal_y_source_seq
                },
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_source_us_x: if self.reversal_x_write_pending {
                    self.reversal_x_confirmed_us
                } else {
                    self.reversal_x_source_us
                },
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_source_us_y: if self.reversal_y_write_pending {
                    self.reversal_y_confirmed_us
                } else {
                    self.reversal_y_source_us
                },
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_first_seq_x: self.reversal_x_first_seq,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_first_seq_y: self.reversal_y_first_seq,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_first_us_x: self.reversal_x_first_us,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_first_us_y: self.reversal_y_first_us,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_device_id: self.reversal_device_id,
                age_us,
            };
        }

        let mut remainder = *self;
        remainder.x = effective_x;
        remainder.y = effective_y;
        let (report, diag) = remainder.take_chunk();
        MouseChunkPlan {
            report,
            #[cfg(feature = "mouse_ble_16bit_report")]
            emitted_x: 0,
            #[cfg(feature = "mouse_ble_16bit_report")]
            emitted_y: 0,
            diag,
            remaining_x: remainder.x,
            remaining_y: remainder.y,
            remaining_wheel: remainder.wheel,
            remaining_pan: remainder.pan,
            remaining_oldest_enqueued_at: if cfg!(feature = "mouse_realtime_reversal_budget_3")
                && (commit_reversal_x || commit_reversal_y)
            {
                now
            } else {
                self.oldest_enqueued_at
            },
            stale_compress: false,
            #[cfg(any(
                feature = "mouse_realtime_burst_budget_3",
                feature = "mouse_realtime_reversal_budget_3"
            ))]
            remaining_stale_vectors: 0,
            dropped_x: 0,
            dropped_y: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            commit_reversal_x,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            commit_reversal_y,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_write_x: self.reversal_x_write_pending,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_write_y: self.reversal_y_write_pending,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_unconfirmed_x: unconfirmed_flush_x,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_unconfirmed_y: unconfirmed_flush_y,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_source_seq_x: if self.reversal_x_write_pending {
                self.reversal_x_confirmed_seq
            } else {
                self.reversal_x_source_seq
            },
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_source_seq_y: if self.reversal_y_write_pending {
                self.reversal_y_confirmed_seq
            } else {
                self.reversal_y_source_seq
            },
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_source_us_x: if self.reversal_x_write_pending {
                self.reversal_x_confirmed_us
            } else {
                self.reversal_x_source_us
            },
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_source_us_y: if self.reversal_y_write_pending {
                self.reversal_y_confirmed_us
            } else {
                self.reversal_y_source_us
            },
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_first_seq_x: self.reversal_x_first_seq,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_first_seq_y: self.reversal_y_first_seq,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_first_us_x: self.reversal_x_first_us,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_first_us_y: self.reversal_y_first_us,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_device_id: self.reversal_device_id,
            age_us,
        }
    }

    #[cfg(any(
        feature = "mouse_realtime_age_cap_30ms",
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    fn commit_chunk(&mut self, plan: MouseChunkPlan) {
        self.x = plan.remaining_x;
        self.y = plan.remaining_y;
        self.wheel = plan.remaining_wheel;
        self.pan = plan.remaining_pan;
        self.oldest_enqueued_at = plan.remaining_oldest_enqueued_at;
        self.source_reports = 0;
        #[cfg(any(
            feature = "mouse_realtime_burst_budget_3",
            feature = "mouse_realtime_reversal_budget_3"
        ))]
        {
            self.stale_vectors_remaining = plan.remaining_stale_vectors;
        }
        #[cfg(feature = "mouse_realtime_reversal_budget_3")]
        {
            if plan.commit_reversal_x {
                self.reversal_x_direction = -self.reversal_x_direction;
                self.reversal_x_candidate = 0;
                self.reversal_x_samples = 0;
                self.reversal_x_write_pending = false;
                self.reversal_x_candidate_flushed = false;
                self.reversal_x_confirmed_seq = 0;
                self.reversal_x_confirmed_us = 0;
                self.reversal_x_first_seq = 0;
                self.reversal_x_first_us = 0;
            } else if plan.reversal_unconfirmed_x {
                self.x = 0;
                if plan.remaining_x == 0 {
                    self.reversal_x_candidate = self.reversal_x_candidate.signum();
                    self.reversal_x_candidate_flushed = true;
                } else {
                    self.reversal_x_candidate = plan.remaining_x;
                    self.reversal_x_candidate_flushed = false;
                }
            }
            if plan.commit_reversal_y {
                self.reversal_y_direction = -self.reversal_y_direction;
                self.reversal_y_candidate = 0;
                self.reversal_y_samples = 0;
                self.reversal_y_write_pending = false;
                self.reversal_y_candidate_flushed = false;
                self.reversal_y_confirmed_seq = 0;
                self.reversal_y_confirmed_us = 0;
                self.reversal_y_first_seq = 0;
                self.reversal_y_first_us = 0;
            } else if plan.reversal_unconfirmed_y {
                self.y = 0;
                if plan.remaining_y == 0 {
                    self.reversal_y_candidate = self.reversal_y_candidate.signum();
                    self.reversal_y_candidate_flushed = true;
                } else {
                    self.reversal_y_candidate = plan.remaining_y;
                    self.reversal_y_candidate_flushed = false;
                }
            }
        }
    }

    fn take_chunk(&mut self) -> (MouseReport, MouseChunkDiag) {
        let input_x = self.x;
        let input_y = self.y;

        fn take_axis(value: &mut i32) -> i8 {
            let chunk = (*value).clamp(i8::MIN as i32, i8::MAX as i32) as i8;
            *value -= i32::from(chunk);
            chunk
        }

        let vector_chunk = self
            .preserve_vector
            .then(|| crate::mouse_chunk::take_vector_chunk(&mut self.x, &mut self.y, &mut self.wheel, &mut self.pan));

        let report = MouseReport {
            buttons: self.buttons,
            x: vector_chunk
                .map(|chunk| chunk.0)
                .unwrap_or_else(|| take_axis(&mut self.x)),
            y: vector_chunk
                .map(|chunk| chunk.1)
                .unwrap_or_else(|| take_axis(&mut self.y)),
            wheel: vector_chunk
                .map(|chunk| chunk.2)
                .unwrap_or_else(|| take_axis(&mut self.wheel)),
            pan: vector_chunk
                .map(|chunk| chunk.3)
                .unwrap_or_else(|| take_axis(&mut self.pan)),
        };

        (
            report,
            MouseChunkDiag {
                input_x,
                input_y,
                residual_x: self.x,
                residual_y: self.y,
            },
        )
    }

    fn has_relative_motion(&self) -> bool {
        #[cfg(feature = "mouse_realtime_reversal_budget_3")]
        let has_reversal_candidate = (self.reversal_x_candidate != 0 && !self.reversal_x_candidate_flushed)
            || (self.reversal_y_candidate != 0 && !self.reversal_y_candidate_flushed);
        #[cfg(not(feature = "mouse_realtime_reversal_budget_3"))]
        let has_reversal_candidate = false;
        self.x != 0 || self.y != 0 || self.wheel != 0 || self.pan != 0 || has_reversal_candidate
    }
}

fn prepare_hid_write_recovery() {
    crate::channel::clear_and_release_report_channel(ConnectionType::Ble);
    set_ble_state(BleState::Sleeping);
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum HostPhyUpdateState {
    Verified,
    Retry,
    Exhausted,
}

fn host_phy_update_state(tx_phy: PhyKind, rx_phy: PhyKind, attempt: u8) -> HostPhyUpdateState {
    if tx_phy == PhyKind::Le2M && rx_phy == PhyKind::Le2M {
        HostPhyUpdateState::Verified
    } else if attempt < HOST_PHY_UPDATE_ATTEMPTS {
        HostPhyUpdateState::Retry
    } else {
        HostPhyUpdateState::Exhausted
    }
}

async fn ensure_host_ble_2m_phy<C, P>(stack: &Stack<'_, C, P>, conn: &Connection<'_, P>)
where
    C: Controller + ControllerCmdAsync<LeSetPhy> + ControllerCmdSync<LeReadPhy>,
    P: PacketPool,
{
    let _guard = BLE_HCI_LINK_UPDATE_MUTEX.lock().await;
    for attempt in 1..=HOST_PHY_UPDATE_ATTEMPTS {
        match conn.set_phy(stack, PhyKind::Le2M).await {
            Ok(()) => info!(
                "[host_phy] LE 2M update requested ({}/{})",
                attempt, HOST_PHY_UPDATE_ATTEMPTS
            ),
            Err(BleHostError::BleHost(Error::Hci(error))) => {
                warn!(
                    "[host_phy] LE 2M update request failed ({}/{}): {:?}",
                    attempt, HOST_PHY_UPDATE_ATTEMPTS, error
                );
            }
            Err(e) => {
                #[cfg(feature = "defmt")]
                let e = defmt::Debug2Format(&e);
                warn!(
                    "[host_phy] LE 2M update request failed ({}/{}): {:?}",
                    attempt, HOST_PHY_UPDATE_ATTEMPTS, e
                );
            }
        }

        // LE Set PHY completes asynchronously. Give the controller enough
        // time for more than one normal connection event before reading the
        // negotiated PHY back.
        Timer::after_millis(HOST_PHY_UPDATE_SETTLE_MS).await;

        match conn.read_phy(stack).await {
            Ok((tx_phy, rx_phy)) => match host_phy_update_state(tx_phy, rx_phy, attempt) {
                HostPhyUpdateState::Verified => {
                    info!("[host_phy] LE 2M verified");
                    return;
                }
                HostPhyUpdateState::Retry => {
                    warn!(
                        "[host_phy] still on {:?}/{:?} after attempt {}/{}",
                        tx_phy, rx_phy, attempt, HOST_PHY_UPDATE_ATTEMPTS
                    );
                }
                HostPhyUpdateState::Exhausted => {
                    warn!(
                        "[host_phy] LE 2M not negotiated; continuing on {:?}/{:?}",
                        tx_phy, rx_phy
                    );
                    return;
                }
            },
            Err(e) => {
                #[cfg(feature = "defmt")]
                let e = defmt::Debug2Format(&e);
                warn!(
                    "[host_phy] failed to read negotiated PHY ({}/{}): {:?}",
                    attempt, HOST_PHY_UPDATE_ATTEMPTS, e
                );
            }
        }

        if !conn.is_connected() {
            return;
        }
    }
}

// Update the PHY to 2M
pub(crate) async fn update_ble_phy<P: PacketPool>(
    stack: &Stack<'_, impl Controller + ControllerCmdAsync<LeSetPhy>, P>,
    conn: &Connection<'_, P>,
) {
    let _guard = BLE_HCI_LINK_UPDATE_MUTEX.lock().await;
    for attempt in 1..=HCI_LINK_UPDATE_ATTEMPTS {
        if !conn.is_connected() {
            return;
        }

        match conn.set_phy(stack, PhyKind::Le2M).await {
            Err(BleHostError::BleHost(Error::Hci(error))) => {
                if is_hci_link_update_busy(error.to_status().into_inner()) && attempt < HCI_LINK_UPDATE_ATTEMPTS {
                    info!(
                        "[update_ble_phy] HCI busy, retry {}/{}: {:?}",
                        attempt, HCI_LINK_UPDATE_ATTEMPTS, error
                    );
                    Timer::after_millis(HCI_LINK_UPDATE_RETRY_MS).await;
                    continue;
                } else {
                    error!("[update_ble_phy] HCI error: {:?}", error);
                }
            }
            Err(e) => {
                #[cfg(feature = "defmt")]
                let e = defmt::Debug2Format(&e);
                error!("[update_ble_phy] error: {:?}", e);
            }
            Ok(_) => {
                info!("[update_ble_phy] PHY updated");
            }
        }
        return;
    }
}

// Update the connection parameters
pub(crate) async fn update_conn_params<
    'a,
    'b,
    C: Controller + ControllerCmdSync<LeReadLocalSupportedFeatures>,
    P: PacketPool,
>(
    stack: &Stack<'a, C, P>,
    conn: &Connection<'b, P>,
    params: &RequestedConnParams,
) -> bool {
    let _guard = BLE_HCI_LINK_UPDATE_MUTEX.lock().await;
    for attempt in 1..=HCI_LINK_UPDATE_ATTEMPTS {
        if !conn.is_connected() {
            return false;
        }

        match conn.update_connection_params(stack, params).await {
            Err(BleHostError::BleHost(Error::Hci(error))) => {
                if is_hci_link_update_busy(error.to_status().into_inner()) && attempt < HCI_LINK_UPDATE_ATTEMPTS {
                    info!(
                        "[update_conn_params] HCI busy, retry {}/{}: {:?}",
                        attempt, HCI_LINK_UPDATE_ATTEMPTS, error
                    );
                    Timer::after_millis(HCI_LINK_UPDATE_RETRY_MS).await;
                    continue;
                } else {
                    error!("[update_conn_params] HCI error: {:?}", error);
                    return false;
                }
            }
            Err(e) => {
                #[cfg(feature = "defmt")]
                let e = defmt::Debug2Format(&e);
                error!("[update_conn_params] BLE host error: {:?}", e);
                return false;
            }
            Ok(_) => return true,
        }
    }
    false
}

fn is_hci_link_update_busy(status: u8) -> bool {
    // 0x2a: Different Transaction Collision
    // 0x3a: Controller Busy
    matches!(status, 0x2a | 0x3a)
}

#[cfg(test)]
mod tests {
    use core::cell::Cell;
    use std::sync::{Mutex, OnceLock};

    use embassy_futures::join::join;
    #[cfg(feature = "host_first_split_wake")]
    use embassy_futures::join::join3;
    use embassy_futures::select::{Either, select};
    use embassy_sync::signal::Signal;
    #[cfg(feature = "host_first_split_wake")]
    use embassy_sync::watch::Watch;
    use embassy_time::{Duration, Instant, Timer};
    use rmk_types::battery::{BatteryStatus, ChargeState};
    use rmk_types::ble::{BleState, BleStatus};
    use trouble_host::Error;
    use trouble_host::prelude::{AdvFilterPolicy, PhyKind};
    use usbd_hid::descriptor::MouseReport;

    use super::{
        BleKeyboardExit, BondedReconnectWindows, HidControlPointAction, HostConnParamBootstrap, HostLinkStartupPolicy,
        HostPhyUpdateState, HostPowerTransition, Server, WakeAdvertisingInput, advertising_mode,
        bonded_reconnect_filter_policy, bonded_reconnect_windows, directed_reconnect_should_continue,
        hid_control_point_action, host_link_startup_policy, host_phy_update_state, host_power_transition_allowed,
        is_hci_link_update_busy, join_ble_session_workers, mark_ble_session_ready, next_host_power_transition,
        pairing_window_timeout_secs, prepare_hid_write_recovery, run_ble_communication_tasks, run_ble_hid_writer,
        run_ble_session_workers, run_until_physical_disconnect, seed_battery_level,
    };
    use crate::ble::sleep::wait_for_input_activity;
    use crate::channel::{BLE_REPORT_CHANNEL, QueuedReport};
    use crate::config::BleHostPowerConfig;
    use crate::event::{
        Axis, AxisEvent, AxisValType, BleAdvertisingMode, EventSubscriber, KeyboardEvent, PointingEvent,
        SubscribableEvent, publish_event, publish_event_async,
    };
    use crate::hid::{KeyboardReport, Report};
    use crate::state::{
        current_ble_advertising_mode, current_ble_status, set_ble_advertising_mode, set_ble_profile, set_ble_state,
    };
    use crate::test_support::test_block_on as block_on;

    fn ble_status_test_lock() -> &'static Mutex<()> {
        static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        LOCK.get_or_init(|| Mutex::new(()))
    }

    #[test]
    fn successful_wake_advertising_releases_temporary_input_subscribers() {
        let mut keyboard = KeyboardEvent::subscriber();
        WakeAdvertisingInput::new(true).connected();

        let final_event = block_on(async {
            select(
                async {
                    join(
                        async {
                            // The wake release plus seven taps fill all 15
                            // remaining slots held by the leaked subscriber.
                            publish_event_async(KeyboardEvent::key(0, 0, false)).await;
                            for _ in 0..7 {
                                publish_event_async(KeyboardEvent::key(0, 0, true)).await;
                                publish_event_async(KeyboardEvent::key(0, 0, false)).await;
                            }

                            // Previously this press reached HID as event 16,
                            // while its release (event 17) blocked forever.
                            publish_event_async(KeyboardEvent::key(0, 0, true)).await;
                            publish_event_async(KeyboardEvent::key(0, 0, false)).await;
                        },
                        async {
                            let mut final_event = None;
                            for _ in 0..17 {
                                final_event = Some(keyboard.next_event().await);
                            }
                            final_event.unwrap()
                        },
                    )
                    .await
                    .1
                },
                async {
                    Timer::after_millis(10).await;
                    panic!("keyboard publication remained blocked by a stale wake subscriber")
                },
            )
            .await
        });

        assert!(matches!(
            final_event,
            Either::First(KeyboardEvent { pressed: false, .. })
        ));
    }

    #[test]
    fn only_transaction_collision_and_controller_busy_retry_link_updates() {
        assert!(is_hci_link_update_busy(0x2a));
        assert!(is_hci_link_update_busy(0x3a));
        assert!(!is_hci_link_update_busy(0x00));
        assert!(!is_hci_link_update_busy(0x08));
    }

    #[test]
    fn advertising_without_active_bond_uses_pairing_mode() {
        assert_eq!(advertising_mode(false), BleAdvertisingMode::Pairing);
    }

    #[test]
    fn advertising_with_active_bond_uses_reconnecting_mode() {
        assert_eq!(advertising_mode(true), BleAdvertisingMode::Reconnecting);
    }

    #[test]
    fn bonded_reconnect_is_undirected_but_connection_filtered() {
        assert_eq!(bonded_reconnect_filter_policy(), AdvFilterPolicy::FilterConn);
        assert_eq!(pairing_window_timeout_secs(true, 30, 10), None);
    }

    #[test]
    fn bonded_reconnect_uses_filtered_undirected_from_the_first_packet() {
        assert_eq!(
            bonded_reconnect_windows(10_000),
            BondedReconnectWindows {
                directed_high_duty_ms: 0,
                fast_undirected_ms: 5_000,
                slow_undirected_ms: 5_000,
            }
        );
    }

    #[test]
    fn bonded_reconnect_windows_fit_short_timeouts_without_underflow() {
        assert_eq!(
            bonded_reconnect_windows(500),
            BondedReconnectWindows {
                directed_high_duty_ms: 0,
                fast_undirected_ms: 500,
                slow_undirected_ms: 0,
            }
        );
        assert_eq!(
            bonded_reconnect_windows(3_000),
            BondedReconnectWindows {
                directed_high_duty_ms: 0,
                fast_undirected_ms: 3_000,
                slow_undirected_ms: 0,
            }
        );
    }

    #[test]
    fn bonded_reconnect_windows_preserve_zero_and_large_timeouts() {
        assert_eq!(
            bonded_reconnect_windows(0),
            BondedReconnectWindows {
                directed_high_duty_ms: 0,
                fast_undirected_ms: 0,
                slow_undirected_ms: 0,
            }
        );
        assert_eq!(
            bonded_reconnect_windows(u64::MAX),
            BondedReconnectWindows {
                directed_high_duty_ms: 0,
                fast_undirected_ms: 5_000,
                slow_undirected_ms: u64::MAX - 5_000,
            }
        );
    }

    #[test]
    fn fresh_session_always_keeps_the_production_link_bootstrap() {
        assert_eq!(
            host_link_startup_policy(false, true, true),
            HostLinkStartupPolicy {
                update_phy: true,
                conn_params: HostConnParamBootstrap::Legacy,
            }
        );
    }

    #[test]
    fn bonded_host_power_session_refreshes_params_without_repeating_phy_update() {
        assert_eq!(
            host_link_startup_policy(true, true, true),
            HostLinkStartupPolicy {
                update_phy: false,
                conn_params: HostConnParamBootstrap::BondedRefresh,
            }
        );
    }

    #[test]
    fn bonded_session_without_host_power_policy_uses_legacy_bootstrap() {
        assert_eq!(
            host_link_startup_policy(true, false, true),
            HostLinkStartupPolicy {
                update_phy: true,
                conn_params: HostConnParamBootstrap::Legacy,
            }
        );
    }

    #[test]
    fn disabled_2m_phy_is_preserved_for_fresh_sessions() {
        assert_eq!(
            host_link_startup_policy(false, true, false),
            HostLinkStartupPolicy {
                update_phy: false,
                conn_params: HostConnParamBootstrap::Legacy,
            }
        );
    }

    #[test]
    fn hid_suspend_is_local_only_for_host_power_managed_links() {
        assert_eq!(hid_control_point_action(0, true), HidControlPointAction::LocalSleep);
        assert_eq!(hid_control_point_action(0, false), HidControlPointAction::Disconnect);
        assert_eq!(hid_control_point_action(1, true), HidControlPointAction::Activity);
        assert_eq!(hid_control_point_action(2, true), HidControlPointAction::Ignore);
    }

    #[test]
    fn bonded_profile_does_not_open_pairing_window() {
        assert_eq!(pairing_window_timeout_secs(true, 60, 300), None);
    }

    #[test]
    fn unbonded_profile_uses_configured_pairing_window() {
        assert_eq!(pairing_window_timeout_secs(false, 60, 300), Some(60));
    }

    #[test]
    fn unbonded_profile_preserves_legacy_pairing_timeout_fallback() {
        assert_eq!(pairing_window_timeout_secs(false, 0, 300), Some(300));
    }

    #[test]
    fn high_duty_timeout_continues_with_low_duty_reconnect() {
        assert!(directed_reconnect_should_continue(&Error::Timeout));
        assert!(!directed_reconnect_should_continue(&Error::Disconnected));
    }

    #[test]
    fn host_phy_update_stops_only_after_bidirectional_2m_is_verified() {
        assert_eq!(
            host_phy_update_state(PhyKind::Le2M, PhyKind::Le2M, 1),
            HostPhyUpdateState::Verified
        );
        assert_eq!(
            host_phy_update_state(PhyKind::Le2M, PhyKind::Le1M, 1),
            HostPhyUpdateState::Retry
        );
        assert_eq!(
            host_phy_update_state(PhyKind::Le1M, PhyKind::Le2M, 1),
            HostPhyUpdateState::Retry
        );
    }

    #[test]
    fn host_phy_update_stops_retrying_after_bounded_attempts() {
        assert_eq!(
            host_phy_update_state(PhyKind::Le1M, PhyKind::Le1M, super::HOST_PHY_UPDATE_ATTEMPTS - 1),
            HostPhyUpdateState::Retry
        );
        assert_eq!(
            host_phy_update_state(PhyKind::Le1M, PhyKind::Le1M, super::HOST_PHY_UPDATE_ATTEMPTS),
            HostPhyUpdateState::Exhausted
        );
    }

    #[cfg(not(feature = "host_first_split_wake"))]
    #[test]
    fn vial_interactive_connection_params_remove_only_slave_latency() {
        let idle = super::host_connection_params(Duration::from_micros(7500), super::HOST_IDLE_MAX_LATENCY);
        let interactive =
            super::host_connection_params(Duration::from_micros(7500), super::HOST_INTERACTIVE_MAX_LATENCY);

        assert!(idle.is_valid());
        assert!(interactive.is_valid());
        assert_eq!(idle.min_connection_interval, interactive.min_connection_interval);
        assert_eq!(idle.max_connection_interval, interactive.max_connection_interval);
        assert_eq!(idle.max_latency, 30);
        assert_eq!(interactive.max_latency, 0);
        assert_eq!(idle.supervision_timeout, interactive.supervision_timeout);
        assert_eq!(idle.supervision_timeout, Duration::from_secs(5));
    }

    #[test]
    fn host_bootstrap_requests_apple_safe_then_fast_interval() {
        let [apple_safe, fast] = super::host_bootstrap_connection_requests();

        assert_eq!(apple_safe, (Duration::from_millis(15), 30, Duration::from_secs(6)));
        assert_eq!(fast, (Duration::from_micros(7500), 60, Duration::from_secs(6)));
    }

    #[test]
    fn host_parameter_fallback_only_when_fast_interval_was_not_applied() {
        use super::{HostConnParamsSnapshot, host_requires_apple_safe_fallback};

        assert!(!host_requires_apple_safe_fallback(Some(HostConnParamsSnapshot {
            interval: Duration::from_micros(7500),
            latency: 60,
        })));
        assert!(host_requires_apple_safe_fallback(Some(HostConnParamsSnapshot {
            interval: Duration::from_millis(15),
            latency: 30,
        })));
        assert!(host_requires_apple_safe_fallback(None));
    }

    #[cfg(feature = "host_first_split_wake")]
    #[test]
    fn interactive_target_uses_zero_latency_and_the_host_supported_interval() {
        use super::{HostConnParamsSnapshot, host_interactive_target};

        assert_eq!(
            host_interactive_target(Some(HostConnParamsSnapshot {
                interval: Duration::from_micros(7500),
                latency: 60,
            })),
            HostConnParamsSnapshot {
                interval: Duration::from_micros(7500),
                latency: 0,
            }
        );
        assert_eq!(
            host_interactive_target(Some(HostConnParamsSnapshot {
                interval: Duration::from_millis(15),
                latency: 30,
            })),
            HostConnParamsSnapshot {
                interval: Duration::from_millis(15),
                latency: 0,
            }
        );
    }

    #[cfg(feature = "host_first_split_wake")]
    #[test]
    fn active_parameters_require_zero_latency_and_no_slower_interval() {
        use super::{HostConnParamsSnapshot, host_active_params_confirmed};

        let target = HostConnParamsSnapshot {
            interval: Duration::from_millis(15),
            latency: 0,
        };
        assert!(host_active_params_confirmed(target, target));
        assert!(host_active_params_confirmed(
            target,
            HostConnParamsSnapshot {
                interval: Duration::from_micros(7500),
                latency: 0,
            }
        ));
        assert!(!host_active_params_confirmed(
            target,
            HostConnParamsSnapshot {
                interval: Duration::from_millis(15),
                latency: 1,
            }
        ));
    }

    #[cfg(feature = "host_first_split_wake")]
    #[test]
    fn host_input_reconfirms_idle_connection_even_while_vial_is_active() {
        assert!(super::host_input_requires_active_confirmation(true, true));
        assert!(super::host_input_requires_active_confirmation(true, false));
        assert!(!super::host_input_requires_active_confirmation(false, true));
    }

    #[cfg(feature = "host_first_split_wake")]
    #[test]
    fn host_confirmation_and_session_end_open_wake_order_gate() {
        use super::{HostWakeOrderEvent, HostWakeOrderGate, HostWakeOrderSession, next_host_wake_order_gate};

        assert_eq!(
            next_host_wake_order_gate(HostWakeOrderGate::Open, HostWakeOrderEvent::HostEnteredIdle),
            HostWakeOrderGate::Pending
        );
        assert_eq!(
            next_host_wake_order_gate(HostWakeOrderGate::Pending, HostWakeOrderEvent::HostActiveConfirmed),
            HostWakeOrderGate::Open
        );
        let gate: Watch<crate::RawMutex, HostWakeOrderGate, 1> = Watch::new_with(HostWakeOrderGate::Open);
        let mut session = HostWakeOrderSession::new_on(&gate);
        session.close_for_idle();
        assert_eq!(gate.try_get(), Some(HostWakeOrderGate::Pending));
        drop(session);
        assert_eq!(gate.try_get(), Some(HostWakeOrderGate::Open));
    }

    #[cfg(feature = "host_first_split_wake")]
    #[test]
    fn host_confirmation_releases_existing_and_late_split_waiters() {
        use super::{HostWakeOrderGate, wait_for_host_wake_order_gate_on};

        let gate: Watch<crate::RawMutex, HostWakeOrderGate, 2> = Watch::new_with(HostWakeOrderGate::Pending);
        block_on(async {
            let first = wait_for_host_wake_order_gate_on(&gate);
            let second = wait_for_host_wake_order_gate_on(&gate);
            let confirm = async { gate.sender().send(HostWakeOrderGate::Open) };
            join3(first, second, confirm).await;
            wait_for_host_wake_order_gate_on(&gate).await;
        });
    }

    #[cfg(feature = "host_fixed_15ms")]
    #[test]
    fn fixed_diagnostic_active_params_ignore_runtime_latency_requests() {
        let idle = super::host_active_connection_params(super::HOST_IDLE_MAX_LATENCY);
        let interactive = super::host_active_connection_params(super::HOST_INTERACTIVE_MAX_LATENCY);

        for params in [idle, interactive] {
            assert_eq!(params.min_connection_interval, Duration::from_millis(15));
            assert_eq!(params.max_connection_interval, Duration::from_millis(15));
            assert_eq!(params.max_latency, 0);
            assert_eq!(params.supervision_timeout, Duration::from_secs(5));
        }
    }

    #[test]
    fn host_power_transitions_are_deferred_only_for_active_usb_output() {
        assert!(!host_power_transition_allowed(Some(
            rmk_types::connection::ConnectionType::Usb
        )));
        assert!(host_power_transition_allowed(Some(
            rmk_types::connection::ConnectionType::Ble
        )));
        assert!(host_power_transition_allowed(None));
    }

    fn ten_minute_disconnect_timeout() -> u64 {
        10 * 60
    }

    fn one_minute_disconnect_timeout() -> u64 {
        60
    }

    #[test]
    fn host_power_policy_enters_idle_before_full_disconnect() {
        let config = BleHostPowerConfig::new(Duration::from_secs(2 * 60), ten_minute_disconnect_timeout);

        assert_eq!(
            next_host_power_transition(config, false),
            (Duration::from_secs(2 * 60), HostPowerTransition::EnterIdle)
        );
        assert_eq!(
            next_host_power_transition(config, true),
            (Duration::from_secs(10 * 60), HostPowerTransition::Disconnect)
        );
    }

    #[test]
    fn host_power_policy_disconnects_directly_when_timeout_precedes_idle() {
        let config = BleHostPowerConfig::new(Duration::from_secs(2 * 60), one_minute_disconnect_timeout);

        assert_eq!(
            next_host_power_transition(config, false),
            (Duration::from_secs(60), HostPowerTransition::Disconnect)
        );
    }

    #[cfg(not(feature = "host_first_split_wake"))]
    #[test]
    fn low_duty_connection_params_use_a_longer_interval() {
        let active = super::host_connection_params(Duration::from_micros(7500), super::HOST_IDLE_MAX_LATENCY);
        let low_duty = super::host_connection_params(Duration::from_millis(30), super::HOST_IDLE_MAX_LATENCY);

        assert!(active.is_valid());
        assert!(low_duty.is_valid());
        assert!(low_duty.min_connection_interval > active.min_connection_interval);
        assert_eq!(low_duty.max_latency, active.max_latency);
    }

    #[cfg(feature = "host_first_split_wake")]
    #[test]
    fn low_duty_connection_params_keep_active_anchor_and_150ms_cadence() {
        for (interval, expected_latency) in [(Duration::from_micros(7500), 19), (Duration::from_millis(15), 9)] {
            let low_duty = super::host_low_duty_connection_params(interval);

            assert!(low_duty.is_valid());
            assert_eq!(low_duty.min_connection_interval, interval);
            assert_eq!(low_duty.max_connection_interval, interval);
            assert_eq!(low_duty.max_latency, expected_latency);
            assert_eq!(
                low_duty.max_connection_interval.as_micros() * (u64::from(low_duty.max_latency) + 1),
                super::HOST_LOW_DUTY_EFFECTIVE_INTERVAL_US
            );
        }
    }

    #[test]
    fn advertising_mode_snapshot_tracks_latest_state() {
        let _guard = ble_status_test_lock().lock().unwrap();

        set_ble_advertising_mode(BleAdvertisingMode::Pairing);
        assert_eq!(current_ble_advertising_mode(), BleAdvertisingMode::Pairing);

        set_ble_advertising_mode(BleAdvertisingMode::Reconnecting);
        assert_eq!(current_ble_advertising_mode(), BleAdvertisingMode::Reconnecting);
    }

    #[test]
    fn cached_battery_level_is_seeded_into_gatt_server() {
        let server = Server::new_default("test").unwrap();

        seed_battery_level(
            &server,
            BatteryStatus::Available {
                charge_state: ChargeState::Discharging,
                level: Some(87),
            },
        );
        assert_eq!(server.get(&server.battery_service.level).unwrap(), 87);

        seed_battery_level(&server, BatteryStatus::Unavailable);
        assert_eq!(server.get(&server.battery_service.level).unwrap(), 87);

        seed_battery_level(
            &server,
            BatteryStatus::Available {
                charge_state: ChargeState::Discharging,
                level: Some(0),
            },
        );
        assert_eq!(server.get(&server.battery_service.level).unwrap(), 0);
    }

    #[test]
    fn set_ble_state_preserves_current_profile() {
        let _guard = ble_status_test_lock().lock().unwrap();

        set_ble_profile(2);
        set_ble_state(BleState::Advertising);

        assert_eq!(
            current_ble_status(),
            BleStatus {
                profile: 2,
                state: BleState::Advertising,
            }
        );
    }

    #[test]
    fn set_ble_profile_resets_state_when_profile_changes() {
        let _guard = ble_status_test_lock().lock().unwrap();

        set_ble_profile(1);
        set_ble_state(BleState::Connected);
        set_ble_profile(3);

        assert_eq!(
            current_ble_status(),
            BleStatus {
                profile: 3,
                state: BleState::Inactive,
            }
        );
    }

    #[test]
    fn hid_worker_exit_ends_session_while_background_workers_are_pending() {
        let session_ready: Signal<crate::RawMutex, ()> = Signal::new();
        session_ready.signal(());

        let exit = block_on(run_ble_session_workers(
            &session_ready,
            async { BleKeyboardExit::Disconnected },
            core::future::pending::<()>(),
            core::future::pending::<()>(),
        ));

        assert_eq!(exit, BleKeyboardExit::Disconnected);
    }

    struct PendingBleHidWriter;

    impl crate::hid::HidWriterTrait for PendingBleHidWriter {
        type ReportType = Report;

        fn write_report(
            &mut self,
            _report: &Self::ReportType,
        ) -> impl core::future::Future<Output = Result<usize, crate::hid::HidError>> {
            core::future::pending()
        }
    }

    struct FailingBleHidWriter;

    impl crate::hid::HidWriterTrait for FailingBleHidWriter {
        type ReportType = Report;

        async fn write_report(&mut self, _report: &Self::ReportType) -> Result<usize, crate::hid::HidError> {
            Err(crate::hid::HidError::BleError)
        }
    }

    #[cfg(any(
        feature = "mouse_realtime_age_cap_30ms",
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    #[test]
    fn realtime_policy_does_not_treat_tolerated_gatt_error_as_success() {
        let mut writer = FailingBleHidWriter;
        let report = Report::MouseReport(mouse_report(0, 1, 0, 0, 0));
        assert_eq!(
            block_on(super::write_ble_hid_report(&mut writer, &report, false, None)),
            Err(super::BleKeyboardExit::HidWriteStalled)
        );
    }

    fn mouse_report(buttons: u8, x: i8, y: i8, wheel: i8, pan: i8) -> MouseReport {
        MouseReport {
            buttons,
            x,
            y,
            wheel,
            pan,
        }
    }

    #[test]
    fn mouse_coalescer_sums_relative_motion_with_unchanged_buttons() {
        let now = Instant::now();
        let mut accumulated = super::AccumulatedMouseReport::new(mouse_report(1, 40, -30, 2, 0), now);
        assert!(accumulated.can_merge(&mouse_report(1, 50, -20, 3, -4)));
        accumulated.merge(mouse_report(1, 50, -20, 3, -4), now);

        let (chunk, _) = accumulated.take_chunk();
        assert_eq!(chunk.buttons, 1);
        assert_eq!((chunk.x, chunk.y, chunk.wheel, chunk.pan), (90, -50, 5, -4));
        assert!(!accumulated.has_relative_motion());
    }

    #[cfg(feature = "fixed_mouse_pacing_15ms")]
    #[test]
    fn fixed_mouse_pacing_uses_completion_time_and_one_host_event() {
        let write_started = Instant::from_millis(100);
        let completed_at = write_started + Duration::from_millis(38);
        let deadline = super::fixed_mouse_pacing_deadline(completed_at);

        assert!(cfg!(feature = "mouse_vector_preserve"));
        assert_eq!(super::MOUSE_CONTROL_INTERVAL, Duration::from_millis(15));
        assert_eq!(deadline, completed_at + Duration::from_millis(15));
        assert!(deadline > write_started + Duration::from_millis(15));
    }

    #[test]
    fn mouse_coalescer_keeps_button_edges_as_ordering_boundaries() {
        let accumulated = super::AccumulatedMouseReport::new(mouse_report(0, 10, 0, 0, 0), Instant::now());
        assert!(!accumulated.can_merge(&mouse_report(1, 5, 0, 0, 0)));
    }

    #[test]
    fn mouse_coalescer_splits_large_motion_without_losing_distance() {
        let now = Instant::now();
        let mut accumulated = super::AccumulatedMouseReport::new(mouse_report(0, 127, -128, 0, 0), now);
        accumulated.merge(mouse_report(0, 127, -128, 0, 0), now);
        accumulated.merge(mouse_report(0, 46, -44, 0, 0), now);

        let mut total_x = 0i32;
        let mut total_y = 0i32;
        let mut chunks = 0;
        loop {
            let (chunk, _) = accumulated.take_chunk();
            total_x += i32::from(chunk.x);
            total_y += i32::from(chunk.y);
            chunks += 1;
            if !accumulated.has_relative_motion() {
                break;
            }
        }

        assert_eq!((total_x, total_y), (300, -300));
        assert_eq!(chunks, 3);
    }

    #[cfg(feature = "mouse_vector_preserve")]
    #[test]
    fn mouse_vector_chunks_preserve_asymmetric_direction() {
        let now = Instant::now();
        let mut accumulated = super::AccumulatedMouseReport::new(mouse_report(0, -125, 10, 0, 0), now);
        accumulated.merge(mouse_report(0, -125, 10, 0, 0), now);

        let (first, first_diag) = accumulated.take_chunk();
        let (second, second_diag) = accumulated.take_chunk();

        assert_eq!((first.x, first.y), (-125, 10));
        assert_eq!((second.x, second.y), (-125, 10));
        assert_eq!((first_diag.residual_x, first_diag.residual_y), (-125, 10));
        assert_eq!((second_diag.residual_x, second_diag.residual_y), (0, 0));
        assert!(!accumulated.has_relative_motion());
    }

    #[cfg(feature = "mouse_vector_preserve")]
    #[test]
    fn mouse_vector_chunks_distribute_all_quadrants_without_loss() {
        for (x, y) in [(300, 90), (300, -90), (-300, 90), (-300, -90)] {
            let now = Instant::now();
            let mut accumulated = super::AccumulatedMouseReport {
                buttons: 0,
                x,
                y,
                wheel: 0,
                pan: 0,
                oldest_enqueued_at: now,
                source_reports: 1,
                preserve_vector: true,
                #[cfg(any(
                    feature = "mouse_realtime_burst_budget_3",
                    feature = "mouse_realtime_reversal_budget_3"
                ))]
                stale_vectors_remaining: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_x_candidate: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_y_candidate: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_x_samples: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_y_samples: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_x_write_pending: false,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_y_write_pending: false,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_x_direction: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_y_direction: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_x_source_seq: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_y_source_seq: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_x_source_us: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_y_source_us: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_x_confirmed_seq: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_y_confirmed_seq: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_x_confirmed_us: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_y_confirmed_us: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_x_first_seq: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_y_first_seq: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_x_first_us: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_y_first_us: 0,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_x_candidate_flushed: false,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_y_candidate_flushed: false,
                #[cfg(feature = "mouse_realtime_reversal_budget_3")]
                reversal_device_id: 0,
            };
            let mut total_x = 0i32;
            let mut total_y = 0i32;
            let mut chunks = 0u32;
            while accumulated.has_relative_motion() {
                let (chunk, _) = accumulated.take_chunk();
                total_x += i32::from(chunk.x);
                total_y += i32::from(chunk.y);
                chunks += 1;
            }
            assert_eq!((total_x, total_y), (x, y));
            assert_eq!(chunks, 3);
        }
    }

    #[cfg(any(
        feature = "mouse_realtime_age_cap_30ms",
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    fn aged_mouse(x: i32, y: i32, wheel: i32, pan: i32, buttons: u8) -> super::AccumulatedMouseReport {
        super::AccumulatedMouseReport {
            buttons,
            x,
            y,
            wheel,
            pan,
            oldest_enqueued_at: Instant::from_millis(0),
            source_reports: 3,
            preserve_vector: true,
            #[cfg(any(
                feature = "mouse_realtime_burst_budget_3",
                feature = "mouse_realtime_reversal_budget_3"
            ))]
            stale_vectors_remaining: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_candidate: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_candidate: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_samples: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_samples: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_write_pending: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_write_pending: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_direction: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_direction: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_source_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_source_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_source_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_source_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_confirmed_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_confirmed_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_confirmed_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_confirmed_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_first_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_first_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_first_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_first_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_candidate_flushed: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_candidate_flushed: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_device_id: 0,
        }
    }

    #[cfg(feature = "mouse_realtime_age_cap_30ms")]
    #[test]
    fn realtime_age_cap_keeps_at_or_below_30ms_lossless() {
        let mut accumulated = aged_mouse(300, 90, 0, 0, 0);
        let mut total = (0i32, 0i32);
        while accumulated.has_relative_motion() {
            let plan = accumulated.prepare_chunk(Instant::from_millis(30));
            assert!(!plan.stale_compress);
            total.0 += i32::from(plan.report.x);
            total.1 += i32::from(plan.report.y);
            accumulated.commit_chunk(plan);
        }
        assert_eq!(total, (300, 90));
    }

    #[cfg(feature = "mouse_realtime_age_cap_30ms")]
    #[test]
    fn realtime_age_cap_compresses_huge_single_axes() {
        for (input, expected) in [((10_000, 0), (127, 0)), ((0, -10_000), (0, -127))] {
            let accumulated = aged_mouse(input.0, input.1, 0, 0, 0);
            let plan = accumulated.prepare_chunk(Instant::from_millis(31));
            assert!(plan.stale_compress);
            assert_eq!((plan.report.x, plan.report.y), expected);
            assert_eq!((plan.remaining_x, plan.remaining_y), (0, 0));
        }
    }

    #[cfg(feature = "mouse_realtime_age_cap_30ms")]
    #[test]
    fn realtime_age_cap_preserves_diagonal_ratio_and_quadrant_signs() {
        for (x, y, expected_x, expected_y) in [
            (300, 90, 127, 38),
            (300, -90, 127, -38),
            (-300, 90, -127, 38),
            (-300, -90, -127, -38),
            (10_000, 1, 127, 1),
        ] {
            let plan = aged_mouse(x, y, 0, 0, 0).prepare_chunk(Instant::from_millis(31));
            assert_eq!((plan.report.x, plan.report.y), (expected_x, expected_y));
        }
    }

    #[cfg(feature = "mouse_realtime_age_cap_30ms")]
    #[test]
    fn realtime_age_cap_commits_only_after_success_and_retry_is_identical() {
        let mut accumulated = aged_mouse(500, -250, 0, 0, 0);
        let original_oldest = accumulated.oldest_enqueued_at;
        let first_attempt = accumulated.prepare_chunk(Instant::from_millis(31));
        let retry = accumulated.prepare_chunk(Instant::from_millis(32));
        assert_eq!(first_attempt.report, retry.report);
        assert_eq!((accumulated.x, accumulated.y), (500, -250));
        assert_eq!(accumulated.oldest_enqueued_at, original_oldest);
        assert_eq!(accumulated.source_reports, 3);

        accumulated.commit_chunk(retry);
        assert_eq!((accumulated.x, accumulated.y), (0, 0));
        assert!(!accumulated.has_relative_motion());
    }

    #[cfg(feature = "mouse_realtime_age_cap_30ms")]
    #[test]
    fn realtime_age_cap_rebases_wheel_remainder_before_new_xy() {
        let mut accumulated = aged_mouse(5_000, -2_500, 300, 0, 0);
        let stale = accumulated.prepare_chunk(Instant::from_millis(31));
        assert!(stale.stale_compress);
        assert_eq!(stale.remaining_wheel, 173);

        accumulated.commit_chunk(stale);
        assert_eq!(accumulated.oldest_enqueued_at, Instant::from_millis(31));
        assert_eq!(accumulated.source_reports, 0);
        accumulated.merge_wide(
            crate::channel::WideMouseReport {
                buttons: 0,
                x: 500,
                y: -250,
                wheel: 0,
                pan: 0,
                #[cfg(feature = "rtt_diag")]
                source: None,
            },
            Instant::from_millis(32),
        );
        assert_eq!(accumulated.source_reports, 1);

        let fresh = accumulated.prepare_chunk(Instant::from_millis(33));
        assert!(!fresh.stale_compress);
        assert_ne!((fresh.remaining_x, fresh.remaining_y), (0, 0));
        assert_eq!(i32::from(fresh.report.x) + fresh.remaining_x, 500);
        assert_eq!(i32::from(fresh.report.y) + fresh.remaining_y, -250);

        let mut total_x = i32::from(fresh.report.x);
        let mut total_y = i32::from(fresh.report.y);
        accumulated.commit_chunk(fresh);
        while accumulated.x != 0 || accumulated.y != 0 {
            let next = accumulated.prepare_chunk(Instant::from_millis(34));
            assert!(!next.stale_compress);
            total_x += i32::from(next.report.x);
            total_y += i32::from(next.report.y);
            accumulated.commit_chunk(next);
        }
        assert_eq!((total_x, total_y), (500, -250));
    }

    #[cfg(feature = "mouse_realtime_age_cap_30ms")]
    #[test]
    fn realtime_age_cap_preserves_buttons_wheel_and_pan() {
        let mut accumulated = aged_mouse(5_000, 2_500, 300, -300, 5);
        let first = accumulated.prepare_chunk(Instant::from_millis(31));
        assert_eq!(first.report.buttons, 5);
        assert_eq!((first.report.wheel, first.report.pan), (127, -128));
        accumulated.commit_chunk(first);
        assert_eq!((accumulated.wheel, accumulated.pan), (173, -172));
        assert!(accumulated.has_relative_motion());

        accumulated.merge(mouse_report(5, 10, -20, 0, 0), Instant::from_millis(32));
        assert_eq!((accumulated.x, accumulated.y), (10, -20));
    }

    #[cfg(any(
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    #[cfg(not(feature = "mouse_ble_16bit_report"))]
    #[test]
    fn realtime_b2_preserves_quadrant_ratio_and_three_vector_distance_budget() {
        for (x, y, expected) in [
            (10_000, 5_000, (381, 191)),
            (10_000, -5_000, (381, -191)),
            (-10_000, 5_000, (-381, 191)),
            (-10_000, -5_000, (-381, -191)),
        ] {
            let mut accumulated = aged_mouse(x, y, 0, 0, 0);
            let mut total = (0, 0);
            let mut writes = 0;
            while accumulated.has_relative_motion() {
                let plan = accumulated.prepare_chunk(Instant::from_millis(31 + writes * 15));
                assert!(plan.stale_compress);
                total.0 += i32::from(plan.report.x);
                total.1 += i32::from(plan.report.y);
                writes += 1;
                accumulated.commit_chunk(plan);
            }
            assert_eq!(writes, 3);
            assert_eq!(total, expected);
            assert!(total.0.unsigned_abs().max(total.1.unsigned_abs()) > 127);
        }
    }

    #[cfg(feature = "mouse_bounded_multi_notification_3")]
    #[test]
    fn realtime_b7_drains_exactly_one_frozen_stale_epoch_in_three_handoffs() {
        let mut accumulated = aged_mouse(10_000, 5_000, 0, 0, 0);
        let mut total = (0i32, 0i32);
        let mut index = 0u8;
        loop {
            index += 1;
            let plan = accumulated.prepare_chunk(Instant::from_millis(31));
            total.0 += i32::from(plan.report.x);
            total.1 += i32::from(plan.report.y);
            accumulated.commit_chunk(plan);
            if !super::continue_bounded_stale_burst(index, accumulated.stale_vectors_remaining) {
                break;
            }
        }
        assert_eq!(index, 3);
        assert_eq!(total, (381, 191));
        assert_eq!(accumulated.stale_vectors_remaining, 0);
        assert!(!accumulated.has_relative_motion());
    }

    #[cfg(feature = "mouse_bounded_multi_notification_3")]
    #[test]
    fn realtime_b7_never_bursts_fresh_residual_and_budget_is_hard_bounded() {
        assert!(!super::continue_bounded_stale_burst(1, 0));
        assert!(super::continue_bounded_stale_burst(1, 2));
        assert!(super::continue_bounded_stale_burst(2, 1));
        assert!(!super::continue_bounded_stale_burst(3, 1));
    }

    #[cfg(feature = "mouse_bounded_multi_notification_3")]
    #[test]
    fn realtime_b7_failed_second_handoff_is_byte_and_state_identical() {
        let mut accumulated = aged_mouse(5_000, -2_500, 300, -300, 5);
        let first = accumulated.prepare_chunk(Instant::from_millis(31));
        accumulated.commit_chunk(first);
        let after_first = accumulated;
        let failed_second = accumulated.prepare_chunk(Instant::from_millis(31));
        let frozen_retry = super::MouseRetry {
            mouse: accumulated,
            plan: failed_second,
        };
        assert_eq!(frozen_retry.mouse, after_first);
        assert_eq!(frozen_retry.plan.report, failed_second.report);
        assert_eq!(frozen_retry.plan.report.buttons, 5);
        accumulated.commit_chunk(frozen_retry.plan);
        assert_eq!(accumulated.stale_vectors_remaining, 1);
    }

    #[cfg(any(
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    #[cfg(not(feature = "mouse_ble_16bit_report"))]
    #[test]
    fn realtime_b2_hard_bounds_stale_epoch_to_three_successful_writes() {
        let mut accumulated = aged_mouse(100_000, -80_000, 0, 0, 0);
        for write in 0..3 {
            let plan = accumulated.prepare_chunk(Instant::from_millis(31 + write * 15));
            assert_eq!(plan.remaining_stale_vectors, 2 - write as u8);
            accumulated.commit_chunk(plan);
        }
        assert!(!accumulated.has_relative_motion());
        assert_eq!(accumulated.stale_vectors_remaining, 0);
        assert_eq!(accumulated.oldest_enqueued_at, Instant::from_millis(61));
    }

    #[cfg(any(
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    #[cfg(not(feature = "mouse_ble_16bit_report"))]
    #[test]
    fn realtime_b2_error_retry_is_identical_and_mutates_only_after_success() {
        let mut accumulated = aged_mouse(5_000, -2_500, 300, -300, 5);
        let original = accumulated;
        let first = accumulated.prepare_chunk(Instant::from_millis(31));
        let retry = accumulated.prepare_chunk(Instant::from_millis(47));
        assert_eq!(first.report, retry.report);
        assert_eq!(first.remaining_x, retry.remaining_x);
        assert_eq!(first.remaining_y, retry.remaining_y);
        assert_eq!(accumulated, original);
        accumulated.commit_chunk(retry);
        assert_eq!(accumulated.stale_vectors_remaining, 2);
        assert_eq!(accumulated.source_reports, 0);
    }

    #[cfg(any(
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    #[test]
    fn realtime_b2_keeps_wheel_pan_buttons_lossless_across_stale_epoch() {
        let mut accumulated = aged_mouse(5_000, 2_500, 300, -300, 5);
        let mut wheel = 0;
        let mut pan = 0;
        while accumulated.has_relative_motion() {
            let plan = accumulated.prepare_chunk(Instant::from_millis(31));
            assert_eq!(plan.report.buttons, 5);
            wheel += i32::from(plan.report.wheel);
            pan += i32::from(plan.report.pan);
            accumulated.commit_chunk(plan);
        }
        assert_eq!((wheel, pan), (300, -300));
    }

    #[cfg(any(
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    #[cfg(not(feature = "mouse_ble_16bit_report"))]
    #[test]
    fn realtime_b2_new_motion_does_not_extend_active_stale_budget() {
        let mut accumulated = aged_mouse(5_000, 2_500, 0, 0, 0);
        let first = accumulated.prepare_chunk(Instant::from_millis(31));
        accumulated.commit_chunk(first);
        accumulated.merge_wide(
            crate::channel::WideMouseReport {
                buttons: 0,
                x: 9_000,
                y: -4_500,
                wheel: 0,
                pan: 0,
                #[cfg(feature = "rtt_diag")]
                source: None,
            },
            Instant::from_millis(40),
        );
        assert_eq!(accumulated.stale_vectors_remaining, 2);
        for now in [46, 61] {
            let plan = accumulated.prepare_chunk(Instant::from_millis(now));
            accumulated.commit_chunk(plan);
        }
        assert_eq!(accumulated.stale_vectors_remaining, 0);
        assert_eq!((accumulated.x, accumulated.y), (0, 0));
        assert_eq!(accumulated.oldest_enqueued_at, Instant::from_millis(61));
    }

    #[cfg(feature = "mouse_realtime_burst_budget_3")]
    #[test]
    fn realtime_b2_never_amplifies_residual_after_direction_cancellation() {
        let mut accumulated = aged_mouse(5_000, 2_500, 0, 0, 0);
        let first = accumulated.prepare_chunk(Instant::from_millis(31));
        accumulated.commit_chunk(first);
        accumulated.merge_wide(
            crate::channel::WideMouseReport {
                buttons: 0,
                x: -200,
                y: -100,
                wheel: 0,
                pan: 0,
                #[cfg(feature = "rtt_diag")]
                source: None,
            },
            Instant::from_millis(40),
        );
        let before = (accumulated.x, accumulated.y);
        assert!(before.0.unsigned_abs().max(before.1.unsigned_abs()) < 127);
        let plan = accumulated.prepare_chunk(Instant::from_millis(46));
        assert_eq!((i32::from(plan.report.x), i32::from(plan.report.y)), before);
        assert_eq!((plan.remaining_x, plan.remaining_y), (0, 0));
    }

    #[cfg(any(
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    ))]
    #[test]
    fn realtime_b2_rebases_timestamp_before_fresh_movement() {
        let mut accumulated = aged_mouse(5_000, -2_500, 300, 0, 0);
        for now in [31, 46, 61] {
            let plan = accumulated.prepare_chunk(Instant::from_millis(now));
            accumulated.commit_chunk(plan);
        }
        assert_eq!(accumulated.oldest_enqueued_at, Instant::from_millis(61));
        accumulated.merge_wide(
            crate::channel::WideMouseReport {
                buttons: 0,
                x: 500,
                y: -250,
                wheel: 0,
                pan: 0,
                #[cfg(feature = "rtt_diag")]
                source: None,
            },
            Instant::from_millis(62),
        );
        let fresh = accumulated.prepare_chunk(Instant::from_millis(63));
        assert!(!fresh.stale_compress);
        assert_eq!(fresh.age_us, 2_000);
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    #[derive(Default)]
    struct DetectorAxis {
        residual: i32,
        candidate: i32,
        samples: u8,
        flushed: bool,
        write_pending: bool,
        direction: i8,
        seq: u32,
        source_us: u32,
        confirmed_seq: u32,
        confirmed_us: u32,
        first_seq: u32,
        first_us: u32,
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    impl DetectorAxis {
        fn feed(&mut self, input: i32, seq: u32, source_us: u32) {
            self.feed_motion(input, input, seq, source_us);
        }

        fn feed_motion(&mut self, raw_input: i32, motion_input: i32, seq: u32, source_us: u32) {
            super::AccumulatedMouseReport::merge_reversal_axis(
                &mut self.residual,
                &mut self.candidate,
                &mut self.samples,
                &mut self.flushed,
                &mut self.write_pending,
                &mut self.direction,
                &mut self.seq,
                &mut self.source_us,
                &mut self.confirmed_seq,
                &mut self.confirmed_us,
                &mut self.first_seq,
                &mut self.first_us,
                motion_input,
                Some(raw_input),
                b'x',
                0,
                seq,
                source_us,
            );
        }

        fn commit_confirmed(&mut self) {
            assert!(self.write_pending);
            self.direction = -self.direction;
            self.candidate = 0;
            self.samples = 0;
            self.flushed = false;
            self.write_pending = false;
        }
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    #[test]
    fn realtime_b6_idle_starts_new_epoch_with_baseline_only() {
        let mut axis = DetectorAxis::default();
        axis.feed(120, 1, 1_000);
        axis.feed(-130, 2, 1_001_000);
        assert_eq!(axis.direction, -1);
        assert_eq!(axis.candidate, 0);
        assert!(!axis.write_pending);
        axis.feed(-140, 3, 1_016_000);
        assert!(!axis.write_pending);
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    #[test]
    fn realtime_b6_acceleration_does_not_change_detector_decision() {
        let mut off = DetectorAxis::default();
        let mut on = DetectorAxis::default();
        off.feed_motion(110, 110, 1, 1_000);
        on.feed_motion(110, 220, 1, 1_000);
        off.feed_motion(-105, -105, 2, 16_000);
        on.feed_motion(-105, -210, 2, 16_000);
        off.feed_motion(10, 10, 3, 20_000);
        on.feed_motion(10, 10, 3, 20_000);
        off.feed_motion(-122, -122, 4, 31_000);
        on.feed_motion(-122, -244, 4, 31_000);
        assert!(off.write_pending && on.write_pending);
        assert_eq!(off.samples, on.samples);
        assert_eq!(off.first_seq, on.first_seq);
        assert_eq!(off.seq, on.seq);
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    #[test]
    fn realtime_b7_hid_plan_freezes_confirmed_second_identity_until_success() {
        let mut axis = DetectorAxis::default();
        axis.feed(120, 9, 1_000);
        axis.feed(-110, 10, 16_000);
        axis.feed(-125, 11, 31_000);
        assert!(axis.write_pending);
        assert_eq!((axis.first_seq, axis.confirmed_seq), (10, 11));

        // A later report belongs to the same physical direction and may
        // advance detector history, but must not replace confirmation identity.
        axis.feed(-127, 12, 46_000);
        assert_eq!(axis.seq, 12);
        assert_eq!((axis.first_seq, axis.confirmed_seq), (10, 11));
        assert_eq!((axis.first_us, axis.confirmed_us), (16_000, 31_000));

        let mut accumulated = aged_mouse(0, 0, 0, 0, 0);
        accumulated.reversal_x_candidate = axis.candidate;
        accumulated.reversal_x_samples = axis.samples;
        accumulated.reversal_x_write_pending = axis.write_pending;
        accumulated.reversal_x_direction = axis.direction;
        accumulated.reversal_x_source_seq = axis.seq;
        accumulated.reversal_x_source_us = axis.source_us;
        accumulated.reversal_x_confirmed_seq = axis.confirmed_seq;
        accumulated.reversal_x_confirmed_us = axis.confirmed_us;
        accumulated.reversal_x_first_seq = axis.first_seq;
        accumulated.reversal_x_first_us = axis.first_us;

        let plan = accumulated.prepare_chunk(Instant::from_millis(31));
        assert!(plan.reversal_write_x);
        assert_eq!(plan.reversal_first_seq_x, 10);
        assert_eq!(plan.reversal_source_seq_x, 11);
        assert_eq!(plan.reversal_first_us_x, 16_000);
        assert_eq!(plan.reversal_source_us_x, 31_000);

        let frozen_retry = super::MouseRetry {
            mouse: accumulated,
            plan,
        };
        assert_eq!(frozen_retry.plan.reversal_source_seq_x, 11);
        assert_eq!(frozen_retry.plan.reversal_source_us_x, 31_000);
        assert_eq!(frozen_retry.mouse.reversal_x_confirmed_seq, 11);

        accumulated.commit_chunk(frozen_retry.plan);
        assert_eq!(accumulated.reversal_x_confirmed_seq, 0);
        assert_eq!(accumulated.reversal_x_confirmed_us, 0);
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    #[test]
    fn realtime_b6_residual_and_sub_deadband_reports_cannot_seed_direction() {
        let mut axis = DetectorAxis {
            residual: -5_000,
            ..DetectorAxis::default()
        };
        axis.feed_motion(-99, -198, 1, 1_000);
        axis.feed_motion(-70, -140, 2, 16_000);
        assert_eq!(axis.direction, 0);
        axis.feed_motion(110, 220, 3, 31_000);
        assert_eq!(axis.direction, 1);
        assert_eq!(axis.candidate, 0);
        assert!(!axis.write_pending);
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    #[test]
    fn realtime_b5_symmetric_boundary_and_first_report_semantics() {
        for value in [99, -99] {
            let mut axis = DetectorAxis::default();
            axis.feed(value, 1, 1_000);
            assert_eq!(axis.direction, 0);
            assert_eq!(axis.candidate, 0);
        }
        for value in [100, 127, -100, -128] {
            let mut axis = DetectorAxis::default();
            axis.feed(value, 1, 1_000);
            assert_eq!(
                axis.direction,
                value.signum() as i8,
                "first report must establish direction"
            );
            axis.feed(-value.signum() * 100, 2, 16_000);
            assert_eq!(axis.samples, 1);
        }
    }

    #[cfg(all(feature = "mouse_realtime_reversal_budget_3", not(feature = "rtt_diag")))]
    #[test]
    fn realtime_b6_unsourced_motion_cannot_establish_detector_history() {
        let report = |x| crate::channel::WideMouseReport {
            buttons: 0,
            x,
            y: 0,
            wheel: 0,
            pan: 0,
        };
        let mut first = super::AccumulatedMouseReport::new_wide(report(110), Instant::from_millis(1));
        first.restore_reversal_memory_and_process_first(super::ReversalMemory::default());
        let plan = first.prepare_chunk(Instant::from_millis(2));
        first.commit_chunk(plan);
        let memory = first.reversal_memory();
        assert_eq!(memory.x_direction, 0);

        let mut second = super::AccumulatedMouseReport::new_wide(report(-110), Instant::from_millis(20));
        second.restore_reversal_memory_and_process_first(memory);
        assert_eq!(second.reversal_x_samples, 0);
        assert_eq!(second.reversal_x_candidate, 0);
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    #[test]
    fn realtime_b5_four_real_flips_confirm_and_same_direction_stops_do_not() {
        let mut axis = DetectorAxis::default();
        let mut seq = 1u32;
        let mut confirmed = 0;
        for sign in [1, -1, 1, -1, 1] {
            for _ in 0..2 {
                axis.feed(sign * 110, seq, seq * 15_000);
                seq += 1;
                if axis.write_pending {
                    confirmed += 1;
                    axis.commit_confirmed();
                }
            }
        }
        assert_eq!(confirmed, 4);

        let mut same = DetectorAxis::default();
        for stroke in 0..5u32 {
            same.feed(110, stroke * 10 + 1, stroke * 100_000 + 1_000);
            same.feed(120, stroke * 10 + 2, stroke * 100_000 + 16_000);
            assert!(!same.write_pending);
            assert_eq!(same.direction, 1);
        }
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    #[test]
    fn realtime_b5_noise_isolated_sample_confirmation_and_axes_are_independent() {
        let mut x = DetectorAxis::default();
        let mut y = DetectorAxis::default();
        x.feed(127, 1, 1_000);
        y.feed(-128, 1, 1_000);
        for (seq, noise) in [(2, -1), (3, -70), (4, 99)] {
            x.feed(noise, seq, seq * 1_000);
        }
        assert_eq!(x.candidate, 0);
        x.feed(-110, 5, 10_000);
        assert_eq!(x.samples, 1);
        assert!(!x.write_pending, "an isolated opposite sample is not confirmation");
        x.feed(-120, 6, 25_000);
        assert!(x.write_pending);
        assert!(!y.write_pending);
        assert_eq!(y.direction, -1);
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    #[test]
    fn realtime_b5_sequence_duplicate_gap_and_nonzero_wrap_are_explicit() {
        let mut duplicate = DetectorAxis::default();
        duplicate.feed(110, 9, 1_000);
        duplicate.feed(-110, 10, 16_000);
        duplicate.feed(-120, 10, 17_000);
        assert_eq!(duplicate.samples, 1);
        assert!(!duplicate.write_pending);

        duplicate.feed(-130, 13, 31_000);
        assert_eq!(duplicate.samples, 1, "a gap restarts evidence");
        duplicate.feed(-140, 14, 46_000);
        assert!(duplicate.write_pending);

        let mut wrapped = DetectorAxis::default();
        wrapped.feed(110, u32::MAX - 1, 1_000);
        wrapped.feed(-110, u32::MAX, 16_000);
        wrapped.feed(-120, 1, 31_000);
        assert!(
            wrapped.write_pending,
            "sequence wrap skips reserved zero and stays contiguous"
        );
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    #[test]
    fn realtime_b4_tiny_opposite_sign_jitter_never_becomes_candidate() {
        let mut accumulated = aged_mouse(5_000, 0, 0, 0, 0);
        for (at, x) in [(10, -1), (11, -8)] {
            accumulated.merge_wide(
                crate::channel::WideMouseReport {
                    buttons: 0,
                    x,
                    y: 0,
                    wheel: 0,
                    pan: 0,
                    #[cfg(feature = "rtt_diag")]
                    source: None,
                },
                Instant::from_millis(at),
            );
        }
        assert_eq!(accumulated.x, 4_991);
        assert_eq!(accumulated.reversal_x_candidate, 0);
        assert!(!accumulated.reversal_x_write_pending);
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    #[test]
    fn realtime_b5_lone_candidate_flush_does_not_commit_direction() {
        let mut accumulated = aged_mouse(0, 0, 0, 0, 0);
        accumulated.reversal_x_direction = 1;
        accumulated.reversal_x_candidate = -200;
        accumulated.reversal_x_samples = 1;
        accumulated.reversal_x_source_seq = 10;
        accumulated.reversal_x_source_us = 10_000;
        accumulated.reversal_x_first_seq = 10;
        accumulated.reversal_x_first_us = 10_000;
        let plan = accumulated.prepare_chunk(Instant::from_millis(11));
        assert!(!plan.commit_reversal_x);
        assert!(plan.reversal_unconfirmed_x);
        assert!(!plan.reversal_write_x);
        assert_eq!(accumulated.x, 0, "planning must not retire any residual");
        accumulated.commit_chunk(plan);
        while accumulated.has_relative_motion() {
            let tail = accumulated.prepare_chunk(Instant::from_millis(12));
            assert!(tail.reversal_unconfirmed_x);
            accumulated.commit_chunk(tail);
        }
        assert_eq!(accumulated.reversal_x_direction, 1);
        assert!(accumulated.reversal_x_candidate_flushed);
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    #[test]
    fn realtime_b4_confirmed_reversal_retires_only_after_success() {
        let mut accumulated = aged_mouse(5_000, 0, 0, 0, 0);
        accumulated.reversal_x_direction = 1;
        accumulated.reversal_x_candidate = -440;
        accumulated.reversal_x_samples = 2;
        accumulated.reversal_x_write_pending = true;
        accumulated.reversal_x_source_seq = 11;
        accumulated.reversal_x_source_us = 11_000;
        accumulated.reversal_x_first_seq = 10;
        accumulated.reversal_x_first_us = 10_000;
        let original = accumulated;
        let first = accumulated.prepare_chunk(Instant::from_millis(12));
        let retry = accumulated.prepare_chunk(Instant::from_millis(13));
        assert_eq!(first.report, retry.report);
        assert_eq!(first.remaining_x, retry.remaining_x);
        assert!(first.report.x < 0);
        assert!(first.commit_reversal_x);
        assert_eq!(accumulated, original);

        accumulated.commit_chunk(retry);
        assert!(accumulated.x <= 0);
        assert!(accumulated.x.unsigned_abs() < 440);
        assert_eq!(accumulated.reversal_x_candidate, 0);
        assert!(!accumulated.reversal_x_write_pending);
        assert_eq!(accumulated.oldest_enqueued_at, Instant::from_millis(13));
    }

    #[cfg(all(
        feature = "mouse_realtime_reversal_budget_3",
        not(feature = "mouse_ble_16bit_report")
    ))]
    #[test]
    fn realtime_b4_reversal_is_per_axis_and_preserves_diagonal_ratio() {
        let mut accumulated = aged_mouse(5_000, 100, 0, 0, 0);
        accumulated.reversal_x_direction = 1;
        accumulated.reversal_x_candidate = -400;
        accumulated.reversal_x_samples = 2;
        accumulated.reversal_x_write_pending = true;
        let plan = accumulated.prepare_chunk(Instant::from_millis(12));
        assert_eq!(i32::from(plan.report.y) + plan.remaining_y, 100);
        assert!(plan.report.x < 0);
        assert!(plan.report.y > 0);
        assert!(plan.commit_reversal_x);
        assert!(!plan.commit_reversal_y);
        accumulated.commit_chunk(plan);
        assert_eq!(accumulated.x, plan.remaining_x);
        assert_eq!(accumulated.y, plan.remaining_y);
    }

    #[cfg(feature = "mouse_realtime_reversal_budget_3")]
    #[test]
    fn realtime_b4_keeps_wheel_pan_buttons_lossless_during_reversal() {
        let mut accumulated = aged_mouse(5_000, 0, 300, -300, 5);
        accumulated.reversal_x_direction = 1;
        accumulated.reversal_x_candidate = -400;
        accumulated.reversal_x_samples = 2;
        accumulated.reversal_x_write_pending = true;
        let mut wheel = 0;
        let mut pan = 0;
        while accumulated.has_relative_motion() {
            let plan = accumulated.prepare_chunk(Instant::from_millis(12));
            assert_eq!(plan.report.buttons, 5);
            wheel += i32::from(plan.report.wheel);
            pan += i32::from(plan.report.pan);
            accumulated.commit_chunk(plan);
        }
        assert_eq!((wheel, pan), (300, -300));
    }

    #[cfg(not(any(
        feature = "mouse_realtime_age_cap_30ms",
        feature = "mouse_realtime_burst_budget_3",
        feature = "mouse_realtime_reversal_budget_3"
    )))]
    #[test]
    fn feature_off_keeps_lossless_large_residual_chunking() {
        let mut accumulated = super::AccumulatedMouseReport {
            buttons: 0,
            x: 500,
            y: -250,
            wheel: 0,
            pan: 0,
            oldest_enqueued_at: Instant::from_millis(0),
            source_reports: 1,
            preserve_vector: true,
            #[cfg(any(
                feature = "mouse_realtime_burst_budget_3",
                feature = "mouse_realtime_reversal_budget_3"
            ))]
            stale_vectors_remaining: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_candidate: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_candidate: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_samples: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_samples: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_write_pending: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_write_pending: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_direction: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_direction: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_source_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_source_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_source_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_source_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_confirmed_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_confirmed_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_confirmed_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_confirmed_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_first_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_first_seq: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_first_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_first_us: 0,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_x_candidate_flushed: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_y_candidate_flushed: false,
            #[cfg(feature = "mouse_realtime_reversal_budget_3")]
            reversal_device_id: 0,
        };
        let mut total = (0i32, 0i32);
        while accumulated.has_relative_motion() {
            let (report, _) = accumulated.take_chunk();
            total.0 += i32::from(report.x);
            total.1 += i32::from(report.y);
        }
        assert_eq!(total, (500, -250));
    }

    #[test]
    fn one_extreme_wide_queue_item_preserves_the_complete_delta() {
        let mut accumulated = super::AccumulatedMouseReport::new_wide(
            crate::channel::WideMouseReport {
                buttons: 0,
                x: i32::from(i16::MAX),
                y: i32::from(i16::MIN),
                wheel: 0,
                pan: 0,
                #[cfg(feature = "rtt_diag")]
                source: None,
            },
            Instant::now(),
        );
        let mut total_x = 0i32;
        let mut total_y = 0i32;
        let mut chunks = 0u32;

        while accumulated.has_relative_motion() {
            let (chunk, _) = accumulated.take_chunk();
            total_x += i32::from(chunk.x);
            total_y += i32::from(chunk.y);
            chunks += 1;
        }

        assert_eq!((total_x, total_y), (i32::from(i16::MAX), i32::from(i16::MIN)));
        assert_eq!(chunks, 259);
    }

    #[test]
    fn stalled_hid_write_exits_after_bounded_timeout() {
        let _guard = ble_status_test_lock().lock().unwrap();
        BLE_REPORT_CHANNEL.clear();
        BLE_REPORT_CHANNEL
            .try_send(QueuedReport::new(Report::KeyboardReport(KeyboardReport::default())))
            .expect("BLE report channel should have capacity");

        let mut writer = PendingBleHidWriter;
        let exit = block_on(run_ble_hid_writer(&mut writer, true));

        assert_eq!(exit, BleKeyboardExit::HidWriteStalled);
        assert!(BLE_REPORT_CHANNEL.is_empty());
    }

    #[test]
    fn hid_stall_recovery_discards_stale_reports_and_queues_all_up() {
        let _guard = ble_status_test_lock().lock().unwrap();
        BLE_REPORT_CHANNEL.clear();
        set_ble_state(BleState::Connected);
        BLE_REPORT_CHANNEL
            .try_send(QueuedReport::new(Report::KeyboardReport(KeyboardReport {
                modifier: 0,
                reserved: 0,
                leds: 0,
                keycodes: [4, 0, 0, 0, 0, 0],
            })))
            .expect("BLE report channel should have capacity");

        prepare_hid_write_recovery();

        assert_eq!(current_ble_status().state, BleState::Sleeping);
        assert_eq!(BLE_REPORT_CHANNEL.len(), 1);
        assert!(matches!(
            BLE_REPORT_CHANNEL.try_receive().map(QueuedReport::into_report),
            Ok(Report::KeyboardReport(report)) if report.modifier == 0 && report.keycodes == [0; 6]
        ));

        set_ble_state(BleState::Inactive);
        BLE_REPORT_CHANNEL.clear();
    }

    #[test]
    fn only_hid_output_waits_for_encrypted_session() {
        let _guard = ble_status_test_lock().lock().unwrap();
        let session_ready: Signal<crate::RawMutex, ()> = Signal::new();
        let hid_started = Cell::new(false);
        let led_started = Cell::new(false);
        let host_started = Cell::new(false);

        BLE_REPORT_CHANNEL.clear();
        set_ble_state(BleState::Sleeping);
        assert!(
            BLE_REPORT_CHANNEL
                .try_send(QueuedReport::new(Report::KeyboardReport(KeyboardReport {
                    modifier: 0,
                    reserved: 0,
                    leds: 0,
                    keycodes: [4, 0, 0, 0, 0, 0],
                })))
                .is_ok()
        );
        assert!(
            BLE_REPORT_CHANNEL
                .try_send(QueuedReport::new(Report::KeyboardReport(KeyboardReport::default())))
                .is_ok()
        );

        let ((pressed, released), (), ()) = block_on(async {
            join(
                join_ble_session_workers(
                    &session_ready,
                    async {
                        hid_started.set(true);
                        (BLE_REPORT_CHANNEL.receive().await, BLE_REPORT_CHANNEL.receive().await)
                    },
                    async { led_started.set(true) },
                    async { host_started.set(true) },
                ),
                async {
                    Timer::after_millis(1).await;
                    assert!(led_started.get(), "LED work must retain its established startup order");
                    assert!(
                        host_started.get(),
                        "host service must retain its established startup order"
                    );
                    assert!(!hid_started.get(), "physical connection must not release HID output");
                    assert_eq!(BLE_REPORT_CHANNEL.len(), 2, "wake press/release must remain queued");

                    mark_ble_session_ready(&session_ready);
                },
            )
            .await
            .0
        });

        assert!(hid_started.get());
        assert!(matches!(pressed.into_report(), Report::KeyboardReport(report) if report.keycodes[0] == 4));
        assert!(matches!(released.into_report(), Report::KeyboardReport(report) if report.keycodes == [0; 6]));
        assert_eq!(current_ble_status().state, BleState::Connected);
        assert!(BLE_REPORT_CHANNEL.is_empty());

        set_ble_state(BleState::Inactive);
    }

    #[test]
    fn gatt_event_pump_runs_while_host_phy_setup_is_pending() {
        let gatt_polled = Cell::new(false);
        let phy_polled = Cell::new(false);

        let exit = block_on(run_ble_communication_tasks(
            async {
                gatt_polled.set(true);
                Timer::after_millis(1).await;
                assert!(phy_polled.get(), "PHY setup must run beside the GATT event pump");
                BleKeyboardExit::Disconnected
            },
            core::future::pending::<BleKeyboardExit>(),
            core::future::pending::<()>(),
            async {
                assert!(gatt_polled.get(), "GATT event consumption must start before PHY setup");
                phy_polled.set(true);
                core::future::pending::<()>().await;
            },
        ));

        assert_eq!(exit, BleKeyboardExit::Disconnected);
        assert!(gatt_polled.get());
        assert!(phy_polled.get());
    }

    #[test]
    fn physical_disconnect_recovers_when_connection_event_is_missing() {
        let physically_connected = Cell::new(true);

        let exit = block_on(async {
            join(
                run_until_physical_disconnect(core::future::pending::<BleKeyboardExit>(), || {
                    physically_connected.get()
                }),
                async {
                    Timer::after_millis(super::HOST_CONNECTION_LIVENESS_POLL_MS + 1).await;
                    physically_connected.set(false);
                },
            )
            .await
            .0
        });

        assert_eq!(exit, BleKeyboardExit::Disconnected);
    }

    #[test]
    fn wake_activity_ignores_noise_and_accepts_real_pointing() {
        let _guard = ble_status_test_lock().lock().unwrap();

        block_on(async {
            let woke = core::cell::Cell::new(false);
            let wake = async {
                wait_for_input_activity().await;
                woke.set(true);
            };
            join(wake, async {
                Timer::after_millis(1).await;
                publish_event(PointingEvent {
                    device_id: 0,
                    axes: [
                        AxisEvent {
                            typ: AxisValType::Rel,
                            axis: Axis::X,
                            value: 1,
                        },
                        AxisEvent {
                            typ: AxisValType::Rel,
                            axis: Axis::Y,
                            value: 0,
                        },
                        AxisEvent {
                            typ: AxisValType::Rel,
                            axis: Axis::Z,
                            value: 0,
                        },
                    ],
                });
                Timer::after_millis(1).await;
                assert!(!woke.get(), "PMW3610 settling noise must not wake BLE");

                publish_event(PointingEvent {
                    device_id: 0,
                    axes: [
                        AxisEvent {
                            typ: AxisValType::Rel,
                            axis: Axis::X,
                            value: 2,
                        },
                        AxisEvent {
                            typ: AxisValType::Rel,
                            axis: Axis::Y,
                            value: 0,
                        },
                        AxisEvent {
                            typ: AxisValType::Rel,
                            axis: Axis::Z,
                            value: 0,
                        },
                    ],
                });
            })
            .await;
            assert!(woke.get());
        });
    }

    #[cfg(feature = "mouse_ble_16bit_report")]
    #[test]
    fn realtime_b8_single_notification_preserves_i16_motion_and_aux_fields() {
        let now = Instant::from_millis(100);
        let mut accumulated = super::AccumulatedMouseReport::new_wide(
            crate::channel::WideMouseReport {
                buttons: 5,
                x: 12_345,
                y: -23_456,
                wheel: 200,
                pan: -200,
                #[cfg(feature = "rtt_diag")]
                source: None,
            },
            now,
        );
        let plan = accumulated.prepare_chunk(now + Duration::from_millis(50));
        assert_eq!((plan.emitted_x, plan.emitted_y), (12_345, -23_456));
        assert_eq!((plan.report.wheel, plan.report.pan), (127, -128));
        assert_eq!((plan.remaining_x, plan.remaining_y), (0, 0));
        assert_eq!((plan.remaining_wheel, plan.remaining_pan), (73, -72));
        assert_eq!((plan.dropped_x, plan.dropped_y), (0, 0));
        assert!(!plan.stale_compress);
        assert_eq!(plan.remaining_stale_vectors, 0);
        // Preparing/retrying is byte/state identical and does not retire data.
        assert_eq!(plan, accumulated.prepare_chunk(now + Duration::from_millis(50)));
        assert_eq!(
            (accumulated.x, accumulated.y, accumulated.wheel, accumulated.pan),
            (12_345, -23_456, 200, -200)
        );
        accumulated.commit_chunk(plan);
        assert_eq!(
            (accumulated.x, accumulated.y, accumulated.wheel, accumulated.pan),
            (0, 0, 73, -72)
        );
        assert!(!cfg!(feature = "mouse_bounded_multi_notification_3"));
    }

    #[cfg(feature = "mouse_ble_16bit_report")]
    #[test]
    fn realtime_b8_i16_boundary_is_lossless_across_paced_slots() {
        let now = Instant::from_millis(1);
        let mut accumulated = super::AccumulatedMouseReport::new_wide(
            crate::channel::WideMouseReport {
                buttons: 0,
                x: -40_000,
                y: 40_000,
                wheel: 0,
                pan: 0,
                #[cfg(feature = "rtt_diag")]
                source: None,
            },
            now,
        );
        let first = accumulated.prepare_chunk(now);
        assert_eq!((first.emitted_x, first.emitted_y), (-32_767, 32_767));
        assert_eq!((first.remaining_x, first.remaining_y), (-7_233, 7_233));
        accumulated.commit_chunk(first);
        let second = accumulated.prepare_chunk(now + Duration::from_millis(15));
        assert_eq!((second.emitted_x, second.emitted_y), (-7_233, 7_233));
        assert_eq!((second.remaining_x, second.remaining_y), (0, 0));
        assert_eq!(i32::from(first.emitted_x) + i32::from(second.emitted_x), -40_000);
        assert_eq!(i32::from(first.emitted_y) + i32::from(second.emitted_y), 40_000);
    }
}

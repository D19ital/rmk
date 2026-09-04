use bt_hci::cmd::le::{LeReadLocalSupportedFeatures, LeReadPhy, LeSetPhy};
use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync};
use embassy_futures::join::join3;
use embassy_futures::select::{Either, Either4, select, select4};
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use rmk_types::battery::BatteryStatus;
use rmk_types::ble::BleState;
use rmk_types::connection::ConnectionType;
use rmk_types::led_indicator::LedIndicator;
use trouble_host::prelude::appearance::human_interface_device::KEYBOARD;
use trouble_host::prelude::service::{BATTERY, HUMAN_INTERFACE_DEVICE};
use trouble_host::prelude::*;
use usbd_hid::descriptor::MouseReport;

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
#[cfg(feature = "host_fixed_15ms")]
const HOST_FIXED_CONN_PARAM_ATTEMPTS: u8 = 3;
#[cfg(feature = "host_fixed_15ms")]
const HOST_FIXED_CONN_PARAM_RETRY_MS: u64 = 250;
const HID_WRITE_TIMEOUT_SECS: u64 = 2;
#[cfg(all(
    feature = "mouse_interval_control",
    feature = "mouse_vector_preserve",
    not(feature = "host_fixed_15ms")
))]
const MOUSE_CONTROL_INTERVAL: Duration = Duration::from_micros(7_500);
#[cfg(all(
    feature = "mouse_interval_control",
    any(not(feature = "mouse_vector_preserve"), feature = "host_fixed_15ms")
))]
const MOUSE_CONTROL_INTERVAL: Duration = Duration::from_millis(15);
const HOST_IDLE_MAX_LATENCY: u16 = 30;
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct HostConnParamsSnapshot {
    interval: Duration,
    latency: u16,
}

/// Carries the parameters that the controller actually applied. This lets the
/// bootstrap task distinguish a host that accepted 7.5 ms from an Apple host
/// that retained 15 ms and would otherwise keep the preceding slave latency.
static HOST_CONN_PARAMS_UPDATED: Signal<crate::RawMutex, HostConnParamsSnapshot> = Signal::new();

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
        Self::new_with_host_power_config(stack, rmk_config, None).await
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
                    host_link_startup_policy(active_bond_info.is_some(), host_power_config.is_some());
                #[cfg(not(feature = "storage"))]
                let active_peer = None;
                #[cfg(not(feature = "storage"))]
                let host_link_policy = host_link_startup_policy(false, host_power_config.is_some());

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

fn host_link_startup_policy(has_active_bond: bool, preserve_bonded_link: bool) -> HostLinkStartupPolicy {
    if has_active_bond && preserve_bonded_link {
        HostLinkStartupPolicy {
            update_phy: false,
            conn_params: HostConnParamBootstrap::BondedRefresh,
        }
    } else {
        HostLinkStartupPolicy {
            update_phy: true,
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

fn host_power_transition_allowed(active_transport: Option<ConnectionType>) -> bool {
    active_transport != Some(ConnectionType::Usb)
}

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

    let mut deferred_report = None;
    let mut pending_mouse = None;
    #[cfg(feature = "mouse_interval_control")]
    let mut next_mouse_slot = None;
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
                    AccumulatedMouseReport::new(mouse, enqueued_at)
                }
                QueuedReportPayload::WideMouse(mouse) => AccumulatedMouseReport::new_wide(mouse, enqueued_at),
                QueuedReportPayload::Hid(report) => {
                    if let Err(exit) = write_ble_hid_report(writer, &report, fail_closed, None).await {
                        return exit;
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
        if deferred_report.is_none() {
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

        let mouse_diag = mouse.take_write_diag();
        let (mouse_report, chunk_diag) = mouse.take_chunk();
        #[cfg(not(feature = "rtt_diag"))]
        let _ = chunk_diag;
        let report = crate::hid::Report::MouseReport(mouse_report);
        let has_residual = mouse.has_relative_motion();
        if has_residual {
            pending_mouse = Some(mouse);
        }
        #[cfg(feature = "rtt_diag")]
        crate::rtt_diag::record_mouse_coalesce(
            merged_reports,
            has_residual,
            chunk_diag.input_x,
            chunk_diag.input_y,
            chunk_diag.residual_x,
            chunk_diag.residual_y,
        );
        #[cfg(feature = "mouse_interval_control")]
        {
            next_mouse_slot = Some(Instant::now() + MOUSE_CONTROL_INTERVAL);
        }

        if let Err(exit) = write_ble_hid_report(writer, &report, fail_closed, Some(mouse_diag)).await {
            return exit;
        }
    }
}

#[derive(Clone, Copy)]
struct MouseWriteDiag {
    oldest_enqueued_at: Instant,
    source_reports: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct MouseChunkDiag {
    input_x: i32,
    input_y: i32,
    residual_x: i32,
    residual_y: i32,
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
        if fail_closed {
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

    fn merge(&mut self, report: MouseReport, enqueued_at: Instant) {
        debug_assert!(self.can_merge(&report));
        self.x = self.x.saturating_add(i32::from(report.x));
        self.y = self.y.saturating_add(i32::from(report.y));
        self.wheel = self.wheel.saturating_add(i32::from(report.wheel));
        self.pan = self.pan.saturating_add(i32::from(report.pan));
        self.oldest_enqueued_at = self.oldest_enqueued_at.min(enqueued_at);
        self.source_reports = self.source_reports.saturating_add(1);
    }

    fn merge_wide(&mut self, report: WideMouseReport, enqueued_at: Instant) {
        debug_assert_eq!(self.buttons, report.buttons);
        self.x = self.x.saturating_add(report.x);
        self.y = self.y.saturating_add(report.y);
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
        };
        self.source_reports = 0;
        diag
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
        self.x != 0 || self.y != 0 || self.wheel != 0 || self.pan != 0
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
    use embassy_futures::select::{Either, select};
    use embassy_sync::signal::Signal;
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
        is_hci_link_update_busy,
        join_ble_session_workers, mark_ble_session_ready, next_host_power_transition, pairing_window_timeout_secs,
        prepare_hid_write_recovery, run_ble_communication_tasks, run_ble_hid_writer, run_ble_session_workers,
        run_until_physical_disconnect, seed_battery_level,
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
            host_link_startup_policy(false, true),
            HostLinkStartupPolicy {
                update_phy: true,
                conn_params: HostConnParamBootstrap::Legacy,
            }
        );
    }

    #[test]
    fn bonded_host_power_session_refreshes_params_without_repeating_phy_update() {
        assert_eq!(
            host_link_startup_policy(true, true),
            HostLinkStartupPolicy {
                update_phy: false,
                conn_params: HostConnParamBootstrap::BondedRefresh,
            }
        );
    }

    #[test]
    fn bonded_session_without_host_power_policy_uses_legacy_bootstrap() {
        assert_eq!(
            host_link_startup_policy(true, false),
            HostLinkStartupPolicy {
                update_phy: true,
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

    #[test]
    fn low_duty_connection_params_use_a_longer_interval() {
        let active = super::host_connection_params(Duration::from_micros(7500), super::HOST_IDLE_MAX_LATENCY);
        let low_duty = super::host_connection_params(Duration::from_millis(30), super::HOST_IDLE_MAX_LATENCY);

        assert!(active.is_valid());
        assert!(low_duty.is_valid());
        assert!(low_duty.min_connection_interval > active.min_connection_interval);
        assert_eq!(low_duty.max_latency, active.max_latency);
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

    #[test]
    fn one_extreme_wide_queue_item_preserves_the_complete_delta() {
        let mut accumulated = super::AccumulatedMouseReport::new_wide(
            crate::channel::WideMouseReport {
                buttons: 0,
                x: i32::from(i16::MAX),
                y: i32::from(i16::MIN),
                wheel: 0,
                pan: 0,
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
}

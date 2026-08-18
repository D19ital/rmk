use bt_hci::cmd::le::{LeReadLocalSupportedFeatures, LeReadPhy, LeSetPhy};
use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync};
use embassy_futures::join::join3;
use embassy_futures::select::{Either, Either3, Either4, select, select3, select4};
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

use crate::ble::battery_service::BleBatteryServer;
use crate::ble::ble_server::{BleHidServer, Server};
use crate::ble::device_info::{PnPID, VidSource};
use crate::ble::led::BleLedReader;
#[cfg(feature = "passkey_entry")]
use crate::ble::passkey::{PasskeyInputState, next_gatt_event};
use crate::ble::profile::{ProfileInfo, ProfileManager, UPDATED_CCCD_TABLE, UPDATED_PROFILE};
use crate::ble::sleep::{
    InputActivityWaiter, report_activity, request_sleep, reset_host_power_input, take_host_power_input,
    wait_for_host_power_input,
};
use crate::channel::{BLE_REPORT_CHANNEL, LED_SIGNAL};
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

const DIRECTED_RECONNECT_WINDOW_MS: u64 = 1_300;
const FAST_ADVERTISING_TIMEOUT_SECS: u64 = 30;
const HOST_PHY_UPDATE_ATTEMPTS: u8 = 3;
const HOST_PHY_UPDATE_SETTLE_MS: u64 = 80;
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

/// Wakes the connected host-power task when a runtime timeout setting changes.
static HOST_POWER_CONFIG_CHANGED: Signal<crate::RawMutex, ()> = Signal::new();

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
            host_power_config: rmk_config.ble_host_power_config,
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
                #[cfg(not(feature = "storage"))]
                let active_peer = None;

                // During wake advertising, subscribe before opening the radio
                // window so a second input can request another attempt even if
                // the current reconnect window expires.
                let wake_during_advertising = resuming_from_sleep.then(InputActivityWaiter::new);

                match select(
                    advertise(product_name, &mut peripheral, server, active_peer, resuming_from_sleep),
                    profile_manager.update_profile(),
                )
                .await
                {
                    Either::First(Ok(conn)) => {
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
                                    conn.raw().disconnect();
                                    loop {
                                        if let GattConnectionEvent::Disconnected { .. } = conn.next().await {
                                            break;
                                        }
                                    }
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
                            Either::Second(()) => {
                                resuming_from_sleep = false;
                                report_activity();

                                // When the profile changes, manually disconnect
                                // from the current host.
                                if conn.raw().is_connected() {
                                    conn.raw().disconnect();
                                    loop {
                                        if let GattConnectionEvent::Disconnected { .. } = conn.next().await {
                                            break;
                                        }
                                    }
                                }
                            }
                        }
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

                        let wake = wake_during_advertising.unwrap_or_else(InputActivityWaiter::new);
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
                        Timer::after_millis(200).await;
                    }
                    Either::Second(()) => {
                        report_activity();
                        resuming_from_sleep = false;
                    }
                };

                // Sleeping remains set while wake advertising is in progress so
                // the first HID report waits for the host to reconnect.
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
async fn gatt_events_task(server: &Server<'_>, conn: &GattConnection<'_, '_, DefaultPacketPool>) -> Result<(), Error> {
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
                set_ble_state(BleState::Connected);
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
                                match data[0] {
                                    0 => request_sleep(),
                                    1 => report_activity(),
                                    _ => {}
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
            }
            GattConnectionEvent::RequestConnectionParams(req) => info!(
                "[gatt] RequestConnectionParams: interval: ({:?}, {:?})ms, {:?}, {:?}ms",
                req.params().min_connection_interval.as_millis(),
                req.params().max_connection_interval.as_millis(),
                req.params().max_latency,
                req.params().supervision_timeout.as_millis(),
            ),
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
    let reconnect_timeout_ms = reconnect_timeout_secs * 1_000;
    let configured_pairing_timeout = u64::from(crate::BLE_PAIRING_TIMEOUT_SECONDS);
    let has_active_peer = active_peer.is_some();
    let pairing_window_secs =
        pairing_window_timeout_secs(has_active_peer, configured_pairing_timeout, reconnect_timeout_secs);

    crate::state::set_ble_advertising_mode(advertising_mode(has_active_peer));
    if !resuming_from_sleep {
        set_ble_state(BleState::Advertising);
    }

    if let Some(peer) = active_peer {
        let high_duty_window_ms = reconnect_timeout_ms.min(DIRECTED_RECONNECT_WINDOW_MS);
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

        let remaining_reconnect_ms = reconnect_timeout_ms.saturating_sub(high_duty_window_ms);
        if remaining_reconnect_ms > 0 {
            info!("[adv] directed reconnect");
            let advertiser = peripheral
                .advertise(
                    &slow_advertise_config,
                    Advertisement::ConnectableNonscannableDirected { peer },
                )
                .await?;
            match with_timeout(Duration::from_millis(remaining_reconnect_ms), advertiser.accept()).await {
                Ok(conn_res) => {
                    let conn = conn_res?.with_attribute_server(server)?;
                    info!("[adv] directed connection established");
                    if let Err(e) = conn.raw().set_bondable(true) {
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

async fn set_conn_params<'a, 'b, C: Controller + ControllerCmdSync<LeReadLocalSupportedFeatures>, P: PacketPool>(
    stack: &Stack<'_, C, P>,
    conn: &GattConnection<'a, 'b, P>,
    host_power_config: Option<BleHostPowerConfig>,
) -> BleKeyboardExit {
    if host_power_config.is_some() {
        reset_host_power_input();
        HOST_POWER_CONFIG_CHANGED.reset();
    }

    // Wait for 5 seconds before setting connection parameters to avoid connection drop
    embassy_time::Timer::after_secs(5).await;

    // For macOS/iOS(aka Apple devices), both interval should be set to 15ms
    // Reference: https://developer.apple.com/accessories/Accessory-Design-Guidelines.pdf
    update_conn_params(
        stack,
        conn.raw(),
        &host_connection_params(Duration::from_millis(15), HOST_IDLE_MAX_LATENCY),
    )
    .await;

    embassy_time::Timer::after_secs(5).await;

    // Setting the conn param the second time ensures that we have best performance on all platforms
    update_conn_params(
        stack,
        conn.raw(),
        &host_connection_params(Duration::from_micros(7500), HOST_IDLE_MAX_LATENCY),
    )
    .await;

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
                        request_sleep();
                        return BleKeyboardExit::IdleTimeout;
                    }

                    last_activity = Instant::now();
                    if idle_connection && !vial_active {
                        info!("Host BLE activity, restoring active connection parameters");
                        update_conn_params(
                            stack,
                            conn.raw(),
                            &host_connection_params(Duration::from_micros(7500), HOST_IDLE_MAX_LATENCY),
                        )
                        .await;
                    }
                    idle_connection = false;
                }
                Either4::Second(()) => {
                    // Preserve last_activity and recalculate the deadline from
                    // the newly selected runtime setting.
                }
                Either4::Third(()) => {
                    let now = Instant::now();
                    last_activity = now;
                    last_vial_activity = now;
                    if !vial_active || idle_connection {
                        update_conn_params(
                            stack,
                            conn.raw(),
                            &host_connection_params(Duration::from_micros(7500), HOST_INTERACTIVE_MAX_LATENCY),
                        )
                        .await;
                    }
                    vial_active = true;
                    idle_connection = false;
                }
                Either4::Fourth(HostPowerTimer::VialIdle) => {
                    vial_active = false;
                    update_conn_params(
                        stack,
                        conn.raw(),
                        &host_connection_params(Duration::from_micros(7500), HOST_IDLE_MAX_LATENCY),
                    )
                    .await;
                }
                Either4::Fourth(HostPowerTimer::Power(HostPowerTransition::EnterIdle)) => {
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
            &host_connection_params(Duration::from_micros(7500), HOST_INTERACTIVE_MAX_LATENCY),
        )
        .await;

        while with_timeout(
            Duration::from_secs(VIAL_LINK_IDLE_TIMEOUT_SECS),
            VIAL_BLE_ACTIVITY.wait(),
        )
        .await
        .is_ok()
        {}

        update_conn_params(
            stack,
            conn.raw(),
            &host_connection_params(Duration::from_micros(7500), HOST_IDLE_MAX_LATENCY),
        )
        .await;
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

/// Run BLE keyboard for one connection.
///
/// Returns when the connection drops or its full idle timeout expires.
/// `writer_task`, `led_task`, and `host_task` are all infinite, so the outer
/// `select(communication_task, inner)` cancels them as a side-effect of
/// `communication_task` returning. `inner` itself never completes.
fn seed_battery_level(server: &Server<'_>, status: BatteryStatus) {
    if let BatteryStatus::Available { level: Some(level), .. } = status {
        server.set(&server.battery_service.level, &level).unwrap();
    }
}

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

    // Advertising stays on the universally discoverable LE 1M PHY. Verify
    // that the established host link actually upgrades to LE 2M: accepting
    // LE Set PHY only schedules the controller procedure and does not prove
    // that the peer completed it.
    ensure_host_ble_2m_phy(stack, conn.raw()).await;

    let communication_task = async {
        match select3(
            gatt_events_task(server, conn),
            set_conn_params(stack, conn, host_power_config),
            ble_battery_server.run(),
        )
        .await
        {
            Either3::First(e) => {
                error!("[gatt_events_task] end: {:?}", e);
                BleKeyboardExit::Disconnected
            }
            Either3::Second(exit) => exit,
            Either3::Third(_) => unreachable!("BLE battery service must run forever"),
        }
    };

    let writer_task = async {
        loop {
            let report = BLE_REPORT_CHANNEL.receive().await;
            if let Err(e) = ble_hid_server.write_report(&report).await {
                error!("Failed to send report: {:?}", e);
            }
        }
    };

    let led_task = run_led_reader(&mut ble_led_reader, ConnectionType::Ble);

    #[cfg(feature = "host")]
    let host_task = crate::host::ble::run_ble_host(server.hid_service.vial_input, server.vial_gatt_service.input, conn);
    #[cfg(not(feature = "host"))]
    let host_task = core::future::pending::<()>();

    let inner = embassy_futures::join::join3(writer_task, led_task, host_task);
    match select(communication_task, inner).await {
        Either::First(exit) => exit,
        Either::Second(_) => unreachable!("BLE session workers must run forever"),
    }
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
    use std::sync::{Mutex, OnceLock};

    use embassy_futures::join::join;
    use embassy_time::{Duration, Timer};
    use rmk_types::battery::{BatteryStatus, ChargeState};
    use rmk_types::ble::{BleState, BleStatus};
    use trouble_host::Error;
    use trouble_host::prelude::PhyKind;

    use super::{
        HostPhyUpdateState, HostPowerTransition, Server, advertising_mode, directed_reconnect_should_continue,
        host_phy_update_state, is_hci_link_update_busy, next_host_power_transition, pairing_window_timeout_secs,
        seed_battery_level,
    };
    use crate::ble::sleep::wait_for_input_activity;
    use crate::config::BleHostPowerConfig;
    use crate::event::{Axis, AxisEvent, AxisValType, BleAdvertisingMode, PointingEvent, publish_event};
    use crate::state::{
        current_ble_advertising_mode, current_ble_status, set_ble_advertising_mode, set_ble_profile, set_ble_state,
    };
    use crate::test_support::test_block_on as block_on;

    fn ble_status_test_lock() -> &'static Mutex<()> {
        static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        LOCK.get_or_init(|| Mutex::new(()))
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

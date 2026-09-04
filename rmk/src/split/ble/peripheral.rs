use bt_hci::cmd::le::LeSetPhy;
use bt_hci::cmd::status::ReadRssi;
use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync};
use embassy_futures::join::join;
use embassy_futures::select::{Either, select};
#[cfg(feature = "rtt_diag")]
use embassy_time::Instant;
use embassy_time::{Duration, Timer, with_timeout};
use rmk_types::connection::ConnectionStatus;
use trouble_host::prelude::*;

#[cfg(feature = "storage")]
use super::PeerAddress;
use crate::ble::sleep::wait_for_input_activity;
use crate::event::{
    CentralConnectedEvent, SleepStateEvent, SplitConnectionState, SplitConnectionStateEvent, publish_event,
    set_current_split_connection_state,
};
use crate::split::driver::{SplitDriverError, SplitReader, SplitWriter};
use crate::split::peripheral::SplitPeripheral;
use crate::split::{SPLIT_MESSAGE_MAX_SIZE, SplitMessage, encode_split_message};
use crate::state::update_status;

const SPLIT_COMPANY_ID: u16 = 0xe118;
const SPLIT_RSSI_INTERVAL: Duration = Duration::from_secs(1);
const SPLIT_RSSI_FAILURE_LIMIT: u8 = 2;
const DIRECTED_SPLIT_RECONNECT_SECS: u64 = 2;
const SPLIT_ADVERTISING_RELEASE_GUARD_MS: u64 = 100;

fn publish_peripheral_split_state(id: usize, state: SplitConnectionState) {
    set_current_split_connection_state(state);
    info!("[SPLIT_STATE_R_V15] id={} state={:?}", id, state);
    publish_event(SplitConnectionStateEvent(state));
}

/// Gatt service used in split peripheral to send split message to central
#[gatt_service(uuid = "4dd5fbaa-18e5-4b07-bf0a-353698659946")]
pub(crate) struct SplitBleService {
    #[characteristic(uuid = "0e6313e3-bd0b-45c2-8d2e-37a2e8128bc3", read, notify, indicate)]
    pub(crate) message_to_central: [u8; SPLIT_MESSAGE_MAX_SIZE],

    #[characteristic(uuid = "4b3514fb-cae4-4d38-a097-3a2a3d1c3b9c", write_without_response, read, notify)]
    pub(crate) message_to_peripheral: [u8; SPLIT_MESSAGE_MAX_SIZE],
}

/// Gatt server in split peripheral
#[gatt_server]
pub(crate) struct BleSplitPeripheralServer {
    pub(crate) service: SplitBleService,
}

/// BLE driver for split peripheral
pub(crate) struct BleSplitPeripheralDriver<'stack, 'server, 'c, P: PacketPool> {
    message_to_peripheral: Characteristic<[u8; SPLIT_MESSAGE_MAX_SIZE]>,
    message_to_central: Characteristic<[u8; SPLIT_MESSAGE_MAX_SIZE]>,
    conn: &'c GattConnection<'stack, 'server, P>,
}

impl<'stack, 'server, 'c, P: PacketPool> BleSplitPeripheralDriver<'stack, 'server, 'c, P> {
    pub(crate) fn new(server: &'server BleSplitPeripheralServer, conn: &'c GattConnection<'stack, 'server, P>) -> Self {
        Self {
            message_to_central: server.service.message_to_central,
            message_to_peripheral: server.service.message_to_peripheral,
            conn,
        }
    }
}

impl<'stack, 'server, 'c, P: PacketPool> SplitReader for BleSplitPeripheralDriver<'stack, 'server, 'c, P> {
    async fn read(&mut self) -> Result<SplitMessage, SplitDriverError> {
        let message = loop {
            match self.conn.next().await {
                GattConnectionEvent::Disconnected { reason } => {
                    error!("Disconnected from central: {:?}", reason);
                    update_status(|c| *c = ConnectionStatus::new());
                    return Err(SplitDriverError::Disconnected);
                }
                GattConnectionEvent::Gatt { event: gatt_event } => {
                    match &gatt_event {
                        GattEvent::Read(event) => {
                            info!("Gatt read event: {:?}", event.handle());
                        }
                        GattEvent::Write(event) => {
                            // Write to peripheral
                            if event.handle() == self.message_to_peripheral.handle {
                                let parsed = event.with_data(|_, data| {
                                    trace!("Got message from central: {:?}", data);
                                    postcard::from_bytes::<SplitMessage>(data)
                                });
                                match parsed {
                                    Ok(message) => {
                                        trace!("Message from central: {:?}", message);
                                        break message;
                                    }
                                    Err(e) => error!("Postcard deserialize split message error: {}", e),
                                }
                            } else {
                                info!("Gatt write other event: {:?}", event.handle());
                            }
                        }
                        _ => debug!("Other gatt event"),
                    };
                    match gatt_event.accept() {
                        Ok(r) => r.send().await,
                        Err(e) => warn!("[gatt] error sending response: {:?}", e),
                    }
                }
                GattConnectionEvent::ConnectionParamsUpdated {
                    conn_interval,
                    peripheral_latency,
                    supervision_timeout,
                } => {
                    info!(
                        "Connection parameters updated: {:?}ms, {:?}, {:?}ms",
                        conn_interval.as_millis(),
                        peripheral_latency,
                        supervision_timeout.as_millis()
                    );
                }
                GattConnectionEvent::PhyUpdated { tx_phy, rx_phy } => {
                    info!("PHY updated: {:?}, {:?}", tx_phy, rx_phy);
                }
                _ => (),
            }
        };
        Ok(message)
    }
}

impl<'stack, 'server, 'c, P: PacketPool> SplitWriter for BleSplitPeripheralDriver<'stack, 'server, 'c, P> {
    async fn write(&mut self, message: &SplitMessage) -> Result<usize, SplitDriverError> {
        let mut buf = [0_u8; SPLIT_MESSAGE_MAX_SIZE];
        let encoded = encode_split_message(message, &mut buf).map_err(|e| {
            error!("Postcard serialize split message error: {}", e);
            SplitDriverError::SerializeError
        })?;
        // Pointing messages can run at 125 Hz. Keep per-packet diagnostics at
        // trace level so USB logging cannot stall the split transport.
        trace!("Writing split message to central: {:?}", message);
        #[cfg(feature = "rtt_diag")]
        let diag_started = Instant::now();
        let result = self.message_to_central.notify_raw(self.conn, encoded, false).await;
        #[cfg(feature = "rtt_diag")]
        crate::rtt_diag::record_split_tx(
            message,
            Instant::now().duration_since(diag_started).as_micros() as u32,
            result.is_ok(),
        );
        result.map_err(|e| {
            error!("BLE notify error: {:?}", e);
            SplitDriverError::BleError(1)
        })?;
        Ok(encoded.len())
    }
}

/// Initialize and run the nRF peripheral keyboard service via BLE.
///
/// # Arguments
///
/// * `id` - The id of the peripheral
/// * `central_addr` - The address of the central
/// * `stack` - The stack to use
pub async fn initialize_nrf_ble_split_peripheral_and_run<
    'b,
    's: 'b,
    C: Controller + ControllerCmdAsync<LeSetPhy> + ControllerCmdSync<ReadRssi>,
>(
    id: usize,
    stack: &'b Stack<'s, C, DefaultPacketPool>,
) {
    publish_event(CentralConnectedEvent { connected: false });
    publish_peripheral_split_state(id, SplitConnectionState::Searching);
    info!("[BOOT_R_V15] id={} split_task=started", id);

    let mut peripheral = stack.peripheral();
    let runner = stack.runner();

    // Read the previously validated split central address from storage.
    let mut central_saved = false;
    let mut central_addr = crate::storage::read_peer_address(0)
        .await
        .filter(|a| a.is_valid)
        .map(|a| {
            central_saved = true;
            a.address
        });

    let peri_task = async {
        let server = BleSplitPeripheralServer::new_default("rmk").unwrap();
        loop {
            update_status(|c| *c = ConnectionStatus::new());
            publish_event(CentralConnectedEvent { connected: false });
            publish_peripheral_split_state(id, SplitConnectionState::Searching);
            match split_peripheral_advertise(id, central_addr, &mut peripheral, &server).await {
                Ok((conn, allow_rebind)) => {
                    info!("Connected to the split central");
                    let new_addr = conn.raw().peer_address().addr.into_inner();
                    if !split_central_address_allowed(central_addr, new_addr, allow_rebind) {
                        warn!("Rejecting non-paired split central address");
                        drop(conn);
                        Timer::after_millis(500).await;
                        continue;
                    }

                    let mut split_driver = BleSplitPeripheralDriver::new(&server, &conn);
                    if !validate_split_central(&mut split_driver).await {
                        warn!("Rejecting split central after product validation");
                        drop(conn);
                        Timer::after_millis(500).await;
                        continue;
                    }

                    publish_event(CentralConnectedEvent { connected: true });
                    publish_peripheral_split_state(id, SplitConnectionState::Connected);
                    if !central_saved || central_addr != Some(new_addr) {
                        info!("Saving validated split central address to storage");
                        if crate::storage::write_peer_address(PeerAddress {
                            peer_id: 0,
                            is_valid: true,
                            address: new_addr,
                        })
                        .await
                        {
                            central_saved = true;
                            central_addr = Some(new_addr);
                        }
                    }
                    let mut peripheral = SplitPeripheral::new(split_driver);
                    match select(peripheral.run(), watch_split_link_health(stack, conn.raw(), id)).await {
                        Either::First(_) => {
                            info!("[SPLIT_RECOVERY_V19] side=R id={} source=transport", id);
                        }
                        Either::Second(result) => {
                            #[cfg(feature = "defmt")]
                            let result = defmt::Debug2Format(&result);
                            warn!("[SPLIT_RECOVERY_V19] side=R id={} source=rssi result={:?}", id, result);
                        }
                    }
                    info!("Disconnected from the split central");
                }
                Err(BleHostError::BleHost(Error::Timeout)) => {
                    error!("Connect to split central timeout");
                    publish_peripheral_split_state(id, SplitConnectionState::Idle);
                    publish_event(SleepStateEvent::new(true));

                    wait_for_input_activity().await;

                    publish_event(SleepStateEvent::new(false));
                    continue;
                }
                Err(e) => {
                    #[cfg(feature = "defmt")]
                    let e = defmt::Debug2Format(&e);
                    error!("Split advertise error: {:?}", e);
                    Timer::after_millis(500).await;
                    continue;
                }
            };
        }
    };

    join(ble_task(runner), peri_task).await;
}

async fn watch_split_link_health<'b, 's: 'b, C, P>(
    stack: &'b Stack<'s, C, P>,
    conn: &Connection<'b, P>,
    peripheral_id: usize,
) -> Result<(), BleHostError<C::Error>>
where
    C: Controller + ControllerCmdSync<ReadRssi>,
    P: PacketPool,
{
    let mut consecutive_failures = 0u8;
    loop {
        Timer::after(SPLIT_RSSI_INTERVAL).await;
        match conn.rssi(stack).await {
            Ok(rssi) => {
                split_rssi_requires_restart(&mut consecutive_failures, true);
                #[cfg(feature = "rtt_diag")]
                info!("[SPLIT_RSSI] side=R id={} rssi_dbm={}", peripheral_id, rssi);
                #[cfg(not(feature = "rtt_diag"))]
                let _ = (peripheral_id, rssi);
            }
            Err(e) => {
                let restart = split_rssi_requires_restart(&mut consecutive_failures, false);
                #[cfg(feature = "defmt")]
                warn!(
                    "[SPLIT_RSSI] side=R id={} read_error={:?} failure={}/{}",
                    peripheral_id,
                    defmt::Debug2Format(&e),
                    consecutive_failures,
                    SPLIT_RSSI_FAILURE_LIMIT
                );
                // A reset central can leave the GATT runner waiting longer
                // than the controller's RSSI command. Treat two consecutive
                // failures as authoritative link loss so one transient
                // controller collision cannot flap a healthy connection.
                if restart {
                    return Err(e);
                }
            }
        }
    }
}

fn split_rssi_requires_restart(consecutive_failures: &mut u8, succeeded: bool) -> bool {
    if succeeded {
        *consecutive_failures = 0;
        false
    } else {
        *consecutive_failures = consecutive_failures.saturating_add(1);
        *consecutive_failures >= SPLIT_RSSI_FAILURE_LIMIT
    }
}

async fn validate_split_central<T: SplitReader + SplitWriter>(driver: &mut T) -> bool {
    match with_timeout(Duration::from_millis(1500), driver.read()).await {
        Ok(Ok(SplitMessage::ProductId(product_id))) if product_id == crate::SPLIT_PRODUCT_ID => driver
            .write(&SplitMessage::ProductId(crate::SPLIT_PRODUCT_ID))
            .await
            .is_ok(),
        Ok(Ok(SplitMessage::ProductId(product_id))) => {
            warn!(
                "Split central product id mismatch: got {}, expected {}",
                product_id,
                crate::SPLIT_PRODUCT_ID
            );
            false
        }
        Ok(Ok(message)) => {
            warn!("Unexpected pre-handshake split message: {:?}", message);
            false
        }
        Ok(Err(e)) => {
            warn!("Split central product check read failed: {:?}", e);
            false
        }
        Err(_) => {
            warn!("Split central product check timeout");
            false
        }
    }
}

fn split_central_address_allowed(
    saved_central_addr: Option<[u8; 6]>,
    new_central_addr: [u8; 6],
    allow_rebind: bool,
) -> bool {
    saved_central_addr.is_none() || saved_central_addr == Some(new_central_addr) || allow_rebind
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
async fn split_peripheral_advertise<'a, 'b, C: Controller>(
    id: usize,
    central_addr: Option<[u8; 6]>,
    peripheral: &mut Peripheral<'a, C, DefaultPacketPool>,
    server: &'b BleSplitPeripheralServer<'_>,
) -> Result<(GattConnection<'a, 'b, DefaultPacketPool>, bool), BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    let (directed_timeout_secs, discoverable_timeout_secs, retry_timeout_secs) =
        split_advertising_windows(central_addr.is_some(), crate::SPLIT_PAIRING_TIMEOUT_SECONDS);

    if directed_timeout_secs > 0 {
        let advertisement = get_peri_advertiser::<C>(id, central_addr, &mut advertiser_data)?;
        let advertiser = peripheral
            .advertise(&AdvertisementParameters::default(), advertisement)
            .await?;
        match with_timeout(Duration::from_secs(directed_timeout_secs), advertiser.accept()).await {
            Ok(conn_res) => {
                let conn = conn_res?.with_attribute_server(server)?;
                info!("[adv] directed split connection established");
                return Ok((conn, false));
            }
            Err(_) => {
                warn!("[adv] directed split reconnect timeout, falling back to discoverable advertising");
                // Cancelling advertiser.accept() does not guarantee that
                // nrf-sdc has already released the advertising set. Avoid a
                // second advertising command on that teardown boundary.
                Timer::after_millis(SPLIT_ADVERTISING_RELEASE_GUARD_MS).await;
            }
        }
    }

    let advertisement = get_peri_advertiser::<C>(id, None, &mut advertiser_data)?;
    let advertiser = peripheral
        .advertise(&AdvertisementParameters::default(), advertisement)
        .await?;
    match with_timeout(Duration::from_secs(discoverable_timeout_secs), advertiser.accept()).await {
        Ok(conn_res) => {
            let conn = conn_res?.with_attribute_server(server)?;
            info!("[adv] discoverable split connection established");
            Ok((conn, true))
        }
        Err(_) if retry_timeout_secs > 0 => {
            warn!("[adv] retry discoverable split advertising");
            let advertisement = get_peri_advertiser::<C>(id, None, &mut advertiser_data)?;
            let advertiser = peripheral
                .advertise(&AdvertisementParameters::default(), advertisement)
                .await?;
            match with_timeout(Duration::from_secs(retry_timeout_secs), advertiser.accept()).await {
                Ok(re) => Ok((re?.with_attribute_server(server)?, true)),
                Err(_e) => Err(BleHostError::BleHost(Error::Timeout)),
            }
        }
        Err(_) => Err(BleHostError::BleHost(Error::Timeout)),
    }
}

fn split_advertising_windows(has_saved_central: bool, configured_timeout_secs: u32) -> (u64, u64, u64) {
    if configured_timeout_secs == 0 {
        return if has_saved_central { (10, 10, 300) } else { (0, 10, 300) };
    }

    let total = u64::from(configured_timeout_secs);
    let directed = if has_saved_central {
        total.min(DIRECTED_SPLIT_RECONNECT_SECS)
    } else {
        0
    };
    (directed, total.saturating_sub(directed), 0)
}

fn get_peri_advertiser<'a, C: Controller>(
    id: usize,
    central_addr: Option<[u8; 6]>,
    advertiser_data: &'a mut [u8; 31],
) -> Result<Advertisement<'a>, BleHostError<C::Error>> {
    let advertisement = match central_addr {
        Some(addr) => Advertisement::ConnectableNonscannableDirected {
            peer: Address::random(addr),
        },
        None => {
            info!("No split central address provided, advertising as undirected");
            AdStructure::encode_slice(
                &[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::CompleteServiceUuids128(&[[
                        70u8, 153u8, 101u8, 152u8, 54u8, 53u8, 10u8, 191u8, 7u8, 75u8, 229u8, 24u8, 170u8, 251u8,
                        213u8, 77u8,
                    ]]),
                    AdStructure::ManufacturerSpecificData {
                        company_identifier: SPLIT_COMPANY_ID,
                        payload: &[
                            (crate::SPLIT_PRODUCT_ID & 0xff) as u8,
                            (crate::SPLIT_PRODUCT_ID >> 8) as u8,
                            id as u8,
                        ],
                    },
                ],
                &mut advertiser_data[..],
            )?;
            trace!("Split advertising data: {:?}", advertiser_data);
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..],
                scan_data: &[],
            }
        }
    };
    Ok(advertisement)
}

/// This is a background task that is required to run forever alongside any other BLE tasks.
async fn ble_task<C: Controller + ControllerCmdAsync<LeSetPhy>, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            panic!("[ble_task] error: {:?}", e);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{split_advertising_windows, split_central_address_allowed, split_rssi_requires_restart};

    const OLD_QUBE: [u8; 6] = [1, 2, 3, 4, 5, 6];
    const NEW_QUBE: [u8; 6] = [7, 8, 9, 10, 11, 12];

    #[test]
    fn saved_qube_is_accepted_during_directed_reconnect() {
        assert!(split_central_address_allowed(Some(OLD_QUBE), OLD_QUBE, false));
    }

    #[test]
    fn different_qube_is_rejected_during_directed_reconnect() {
        assert!(!split_central_address_allowed(Some(OLD_QUBE), NEW_QUBE, false));
    }

    #[test]
    fn validated_discoverable_connection_can_rebind_qube() {
        assert!(split_central_address_allowed(Some(OLD_QUBE), NEW_QUBE, true));
    }

    #[test]
    fn configured_split_timeout_is_one_total_window() {
        assert_eq!(split_advertising_windows(true, 30), (2, 28, 0));
        assert_eq!(split_advertising_windows(false, 30), (0, 30, 0));
    }

    #[test]
    fn zero_split_timeout_preserves_legacy_windows() {
        assert_eq!(split_advertising_windows(true, 0), (10, 10, 300));
        assert_eq!(split_advertising_windows(false, 0), (0, 10, 300));
    }

    #[test]
    fn split_health_watchdog_restarts_only_after_two_consecutive_failures() {
        let mut failures = 0;
        assert!(!split_rssi_requires_restart(&mut failures, false));
        assert_eq!(failures, 1);
        assert!(!split_rssi_requires_restart(&mut failures, true));
        assert_eq!(failures, 0);
        assert!(!split_rssi_requires_restart(&mut failures, false));
        assert!(split_rssi_requires_restart(&mut failures, false));
    }
}

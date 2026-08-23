use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicU32, Ordering};

use bt_hci::cmd::le::{LeReadLocalSupportedFeatures, LeSetPhy, LeSetScanParams};
use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync};
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use heapless::VecView;
use trouble_host::prelude::*;

use crate::SPLIT_PAIRING_TIMEOUT_SECONDS;
use crate::ble::sleep::{is_sleeping, report_activity, report_pointing_activity};
use crate::ble::{update_ble_phy, update_conn_params};
use crate::channel::FLASH_CHANNEL;
use crate::event::{
    EventSubscriber, PeripheralConnectedEvent, PointingEvent, SleepStateEvent, SplitConnectionState,
    SplitConnectionStateEvent, SubscribableEvent, publish_event,
};
#[cfg(feature = "storage")]
use crate::split::ble::PeerAddress;
use crate::split::driver::{PeripheralManager, SplitDriverError, SplitReader, SplitWriter};
use crate::split::{SPLIT_MESSAGE_MAX_SIZE, SplitMessage, encode_split_message};
use crate::storage::FlashOperationMessage;

pub(crate) static STACK_STARTED: Signal<crate::RawMutex, bool> = Signal::new();
pub(crate) static PERIPHERAL_FOUND: Signal<crate::RawMutex, (u8, BdAddr)> = Signal::new();

// Signals and mutex for syncing scanning state between scanning task and peripheral manager
static START_SCANNING: Signal<crate::RawMutex, ()> = Signal::new();
static STOP_SCANNING: Signal<crate::RawMutex, ()> = Signal::new();
static SCANNING_MUTEX: Mutex<crate::RawMutex, ()> = Mutex::new(());
static UNCOMMITTED_PEER_CANDIDATES: BlockingMutex<crate::RawMutex, Cell<u32>> = BlockingMutex::new(Cell::new(0));
static CONNECTED_PERIPHERALS: BlockingMutex<crate::RawMutex, Cell<u32>> = BlockingMutex::new(Cell::new(0));
static PERIPHERAL_CONNECTION_CHANGED: Signal<crate::RawMutex, ()> = Signal::new();
static SPLIT_WINDOW_RESTART: Signal<crate::RawMutex, u32> = Signal::new();
static SPLIT_WINDOW_DONE: Signal<crate::RawMutex, u32> = Signal::new();
static SPLIT_WINDOW_GENERATION: BlockingMutex<crate::RawMutex, Cell<u32>> = BlockingMutex::new(Cell::new(0));
#[derive(Clone, Copy)]
struct LinkProfileOverrides {
    configured: u32,
    pointing: u32,
}
static LINK_PROFILE_OVERRIDES: BlockingMutex<crate::RawMutex, Cell<LinkProfileOverrides>> =
    BlockingMutex::new(Cell::new(LinkProfileOverrides {
        configured: 0,
        pointing: 0,
    }));
static LINK_PROFILE_CHANGED: [Signal<crate::RawMutex, ()>; u32::BITS as usize] =
    [const { Signal::new() }; u32::BITS as usize];

static LAST_POINTING_ACTIVITY_MS: AtomicU32 = AtomicU32::new(0);
// PMW3610 may emit background +/-1 reports while settling. Velvet's
// auto-mouse layer uses the same threshold so UI work does not remain deferred
// after meaningful cursor movement has stopped.
const POINTING_ACTIVITY_THRESHOLD: u16 = 2;

const SPLIT_SERVICE_UUID: [u8; 16] = [70, 153, 101, 152, 54, 53, 10, 191, 7, 75, 229, 24, 170, 251, 213, 77];
const SPLIT_COMPANY_ID: u16 = 0xe118;
const VALIDATED_PEER_FAILURE_LIMIT: u8 = 3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FailedPeerAction {
    Retain { consecutive_failures: u8 },
    Forget,
}

#[derive(Default)]
struct PeerRetryState {
    validated_failures: u8,
}

impl PeerRetryState {
    fn on_failed_attempt(&mut self, uncommitted: bool) -> FailedPeerAction {
        if uncommitted {
            self.validated_failures = 0;
            return FailedPeerAction::Forget;
        }

        self.validated_failures = self.validated_failures.saturating_add(1);
        if self.validated_failures >= VALIDATED_PEER_FAILURE_LIMIT {
            self.validated_failures = 0;
            FailedPeerAction::Forget
        } else {
            FailedPeerAction::Retain {
                consecutive_failures: self.validated_failures,
            }
        }
    }

    fn on_validated(&mut self) {
        self.validated_failures = 0;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct SplitScanTiming {
    interval: Duration,
    window: Duration,
}

fn split_scan_timing() -> SplitScanTiming {
    // A continuous split scan can starve an already-connected host link on a
    // single radio. Leave 70% of each cycle to established link traffic.
    SplitScanTiming {
        interval: Duration::from_millis(100),
        window: Duration::from_millis(30),
    }
}

fn split_liveness_poll() -> Duration {
    Duration::from_millis(250)
}

/// Active connection cadence for a generated split keyboard.
///
/// Generated split keyboards apply the low-latency profile only to a
/// peripheral that owns a pointing device. Key-only links retain the
/// lower-power 15 ms cadence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SplitLinkProfile {
    Keyboard,
    Pointing,
}

/// Override the active BLE cadence for one split peripheral at runtime.
///
/// Generated keyboards keep their compile-time profile until this is called.
/// This lets a hot-configurable board switch between a key-only link and a
/// 125 Hz pointing link without declaring a second hardware device in TOML.
pub fn set_split_link_profile(peripheral_id: usize, profile: SplitLinkProfile) -> bool {
    let Some(changed) = LINK_PROFILE_CHANGED.get(peripheral_id) else {
        return false;
    };
    let bit = bit_for_peri(peripheral_id);
    let (was_configured, previous_pointing) = LINK_PROFILE_OVERRIDES.lock(|state| {
        let current = state.get();
        let pointing = match profile {
            SplitLinkProfile::Keyboard => current.pointing & !bit,
            SplitLinkProfile::Pointing => current.pointing | bit,
        };
        state.set(LinkProfileOverrides {
            configured: current.configured | bit,
            pointing,
        });
        (current.configured & bit != 0, current.pointing & bit != 0)
    });
    let is_pointing = profile == SplitLinkProfile::Pointing;
    if !was_configured || previous_pointing != is_pointing {
        changed.signal(());
    }
    true
}

fn effective_split_link_profile(peripheral_id: usize, generated: SplitLinkProfile) -> SplitLinkProfile {
    let bit = bit_for_peri(peripheral_id);
    LINK_PROFILE_OVERRIDES.lock(|state| {
        let state = state.get();
        if state.configured & bit == 0 {
            generated
        } else if state.pointing & bit != 0 {
            SplitLinkProfile::Pointing
        } else {
            SplitLinkProfile::Keyboard
        }
    })
}

fn required_peripheral_mask() -> u32 {
    if crate::SPLIT_PERIPHERALS_NUM >= u32::BITS as usize {
        u32::MAX
    } else {
        (1u32 << crate::SPLIT_PERIPHERALS_NUM) - 1
    }
}

fn all_peripherals_connected() -> bool {
    CONNECTED_PERIPHERALS.lock(Cell::get) & required_peripheral_mask() == required_peripheral_mask()
}

fn publish_peripheral_connection(id: usize, connected: bool) {
    let bit = bit_for_peri(id);
    CONNECTED_PERIPHERALS.lock(|state| {
        let next = if connected {
            state.get() | bit
        } else {
            state.get() & !bit
        };
        state.set(next);
    });
    publish_event(PeripheralConnectedEvent { id, connected });
    PERIPHERAL_CONNECTION_CHANGED.signal(());
}

fn publish_split_connection_state(state: SplitConnectionState, generation: u32, terminal: bool) {
    publish_event(SplitConnectionStateEvent(state));
    if terminal {
        SPLIT_WINDOW_DONE.signal(generation);
    }
}

/// Supervise the complete split-link search window.
///
/// Peripheral managers keep reconnecting in the background, while this task
/// owns the visible `Searching -> Connected/Idle` state and its timeout.
pub async fn run_split_connection_supervisor() {
    let timeout = Duration::from_secs(u64::from(SPLIT_PAIRING_TIMEOUT_SECONDS));
    let mut generation = SPLIT_WINDOW_GENERATION.lock(Cell::get);
    let mut state = if all_peripherals_connected() {
        SplitConnectionState::Connected
    } else {
        SplitConnectionState::Searching
    };
    let mut deadline = Instant::now() + timeout;
    publish_split_connection_state(state, generation, state == SplitConnectionState::Connected);

    loop {
        match state {
            SplitConnectionState::Searching if SPLIT_PAIRING_TIMEOUT_SECONDS == 0 => {
                match select(PERIPHERAL_CONNECTION_CHANGED.wait(), SPLIT_WINDOW_RESTART.wait()).await {
                    Either::First(()) => {
                        if all_peripherals_connected() {
                            state = SplitConnectionState::Connected;
                            publish_split_connection_state(state, generation, true);
                        }
                    }
                    Either::Second(next_generation) => {
                        generation = next_generation;
                        state = if all_peripherals_connected() {
                            SplitConnectionState::Connected
                        } else {
                            SplitConnectionState::Searching
                        };
                        publish_split_connection_state(state, generation, state == SplitConnectionState::Connected);
                    }
                }
            }
            SplitConnectionState::Searching => {
                match select3(
                    PERIPHERAL_CONNECTION_CHANGED.wait(),
                    SPLIT_WINDOW_RESTART.wait(),
                    Timer::at(deadline),
                )
                .await
                {
                    Either3::First(()) => {
                        if all_peripherals_connected() {
                            state = SplitConnectionState::Connected;
                            publish_split_connection_state(state, generation, true);
                        }
                    }
                    Either3::Second(next_generation) => {
                        generation = next_generation;
                        deadline = Instant::now() + timeout;
                        state = if all_peripherals_connected() {
                            SplitConnectionState::Connected
                        } else {
                            SplitConnectionState::Searching
                        };
                        publish_split_connection_state(state, generation, state == SplitConnectionState::Connected);
                    }
                    Either3::Third(()) => {
                        state = SplitConnectionState::Idle;
                        publish_split_connection_state(state, generation, true);
                    }
                }
            }
            SplitConnectionState::Connected => {
                match select(PERIPHERAL_CONNECTION_CHANGED.wait(), SPLIT_WINDOW_RESTART.wait()).await {
                    Either::First(()) => {
                        if !all_peripherals_connected() {
                            state = SplitConnectionState::Searching;
                            deadline = Instant::now() + timeout;
                            publish_split_connection_state(state, generation, false);
                        }
                    }
                    Either::Second(next_generation) => {
                        generation = next_generation;
                        if all_peripherals_connected() {
                            publish_split_connection_state(state, generation, true);
                        } else {
                            state = SplitConnectionState::Searching;
                            deadline = Instant::now() + timeout;
                            publish_split_connection_state(state, generation, false);
                        }
                    }
                }
            }
            SplitConnectionState::Idle => {
                match select(PERIPHERAL_CONNECTION_CHANGED.wait(), SPLIT_WINDOW_RESTART.wait()).await {
                    Either::First(()) => {
                        if all_peripherals_connected() {
                            state = SplitConnectionState::Connected;
                            publish_split_connection_state(state, generation, true);
                        }
                    }
                    Either::Second(next_generation) => {
                        generation = next_generation;
                        deadline = Instant::now() + timeout;
                        state = if all_peripherals_connected() {
                            SplitConnectionState::Connected
                        } else {
                            SplitConnectionState::Searching
                        };
                        publish_split_connection_state(state, generation, state == SplitConnectionState::Connected);
                    }
                }
            }
        }
    }
}

/// Start a fresh split acquisition phase and wait for either all peripherals
/// or the configured split timeout.
pub(crate) async fn wait_for_split_connection_window() {
    if SPLIT_PAIRING_TIMEOUT_SECONDS == 0 || all_peripherals_connected() {
        return;
    }

    let generation = SPLIT_WINDOW_GENERATION.lock(|state| {
        let next = state.get().wrapping_add(1);
        state.set(next);
        next
    });
    SPLIT_WINDOW_RESTART.signal(generation);
    loop {
        if SPLIT_WINDOW_DONE.wait().await == generation {
            return;
        }
    }
}

/// Gatt service used in split central to send split message to peripheral
#[gatt_service(uuid = "4dd5fbaa-18e5-4b07-bf0a-353698659946")]
struct SplitBleCentralService {
    #[characteristic(uuid = "0e6313e3-bd0b-45c2-8d2e-37a2e8128bc3", read, notify)]
    message_to_central: [u8; SPLIT_MESSAGE_MAX_SIZE],

    #[characteristic(uuid = "4b3514fb-cae4-4d38-a097-3a2a3d1c3b9c", write_without_response, read, notify)]
    message_to_peripheral: [u8; SPLIT_MESSAGE_MAX_SIZE],
}

/// Gatt server in split peripheral
#[gatt_server]
struct BleSplitCentralServer {
    service: SplitBleCentralService,
}

pub async fn scan_peripherals<
    'b,
    's: 'b,
    C: Controller
        + ControllerCmdSync<LeSetScanParams>
        + ControllerCmdAsync<LeSetPhy>
        + ControllerCmdSync<LeReadLocalSupportedFeatures>,
>(
    stack: &'b Stack<'s, C, DefaultPacketPool>,
    addrs: &RefCell<VecView<Option<[u8; 6]>>>,
) {
    loop {
        // Wait unitil `START_SCANNING` is signaled
        START_SCANNING.wait().await;
        // Check whether the scanning is needed, aka there's empty slot in the addr list.
        let need_scan = !addrs.borrow().iter().all(|a| a.is_some());
        if need_scan {
            let scanning_fut = async {
                loop {
                    let mut central = stack.central();
                    wait_for_stack_started().await;
                    let mut scanner = Scanner::new(&mut central);
                    let timing = split_scan_timing();
                    let scan_config = ScanConfig {
                        active: false,
                        interval: timing.interval,
                        window: timing.window,
                        ..Default::default()
                    };
                    let _guard = SCANNING_MUTEX.lock().await;
                    if let Ok(_session) = scanner.scan(&scan_config).await {
                        info!("Start scanning peripherals");
                        STOP_SCANNING.wait().await;
                        info!("Stop scanning");
                    }
                }
            };
            let update_addrs_fut = async {
                loop {
                    let (found_peripheral_id, addr) = PERIPHERAL_FOUND.wait().await;
                    let scanned_addr = addr.into_inner();
                    if let Some(Some(stored_addr)) = addrs.borrow_mut().get_mut(found_peripheral_id as usize)
                        && *stored_addr == scanned_addr
                    {
                        continue;
                    }

                    info!("Scanned split peripheral {:?}", scanned_addr);
                    let mut slot_updated = false;
                    if let Some(slot) = addrs.borrow_mut().get_mut(found_peripheral_id as usize)
                        && slot.is_none()
                    {
                        *slot = Some(scanned_addr);
                        slot_updated = true;
                    }

                    // Do not persist a scanned address until the GATT product-id
                    // handshake proves that it belongs to this keyboard model.
                    if slot_updated {
                        mark_uncommitted_peer_candidate(found_peripheral_id as usize);
                    }

                    if addrs.borrow().iter().all(|a| a.is_some()) {
                        break;
                    }
                }
            };

            // Scan until all peripherals are scanned
            // TODO: Timeout?
            select(scanning_fut, update_addrs_fut).await;
        }
    }
}

// When no peripheral address is saved, the central should first scan for peripheral.
// This handler is used to handle the scan result.
pub(crate) struct ScanHandler {}

impl EventHandler for ScanHandler {
    fn on_adv_reports(&self, mut it: LeAdvReportsIter<'_>) {
        while let Some(Ok(report)) = it.next() {
            if let Some(peripheral_id) = split_peripheral_id_from_advertisement(report.data)
                .or_else(|| legacy_split_peripheral_id_from_advertisement(report.data))
            {
                info!("Found split peripheral: id={:?}, addr={:?}", peripheral_id, report.addr);
                PERIPHERAL_FOUND.signal((peripheral_id, report.addr));
                break;
            }
        }
    }
}

// Migration compatibility for the previous upstream/Qube advertisement,
// which carried only the peripheral id. Product identity is still verified
// by the GATT handshake before the address is persisted.
fn legacy_split_peripheral_id_from_advertisement(data: &[u8]) -> Option<u8> {
    if data.len() > 25
        && data[4] == 0x07
        && data[5..].starts_with(&SPLIT_SERVICE_UUID)
        && data[21..25] == [0x04, 0xff, 0x18, 0xe1]
    {
        Some(data[25])
    } else {
        None
    }
}

fn split_peripheral_id_from_advertisement(data: &[u8]) -> Option<u8> {
    let mut has_split_service = false;
    let mut matching_product_peripheral_id = None;
    let mut offset = 0usize;

    while offset < data.len() {
        let len = data[offset] as usize;
        if len == 0 {
            break;
        }
        let end = offset + 1 + len;
        if end > data.len() || len < 1 {
            break;
        }

        let ad_type = data[offset + 1];
        let payload = &data[offset + 2..end];
        match ad_type {
            0x07 if payload == SPLIT_SERVICE_UUID => {
                has_split_service = true;
            }
            0xff if payload.len() >= 5 => {
                let company_id = u16::from_le_bytes([payload[0], payload[1]]);
                let product_id = u16::from_le_bytes([payload[2], payload[3]]);
                if company_id == SPLIT_COMPANY_ID && product_id == crate::SPLIT_PRODUCT_ID {
                    matching_product_peripheral_id = Some(payload[4]);
                }
            }
            _ => {}
        }

        offset = end;
    }

    has_split_service.then_some(matching_product_peripheral_id).flatten()
}

fn bit_for_peri(peri_id: usize) -> u32 {
    1u32 << peri_id.min(31)
}

fn mark_uncommitted_peer_candidate(peri_id: usize) {
    let bit = bit_for_peri(peri_id);
    UNCOMMITTED_PEER_CANDIDATES.lock(|cell| cell.set(cell.get() | bit));
}

fn take_uncommitted_peer_candidate(peri_id: usize) -> bool {
    let bit = bit_for_peri(peri_id);
    UNCOMMITTED_PEER_CANDIDATES.lock(|cell| {
        let current = cell.get();
        cell.set(current & !bit);
        current & bit != 0
    })
}

async fn handle_failed_peer(
    peri_id: usize,
    addrs: &RefCell<VecView<Option<[u8; 6]>>>,
    retry_state: &mut PeerRetryState,
) {
    let uncommitted = take_uncommitted_peer_candidate(peri_id);
    match retry_state.on_failed_attempt(uncommitted) {
        FailedPeerAction::Retain { consecutive_failures } => {
            warn!(
                "Retaining validated split peer {} after transient connection failure {}/{}",
                peri_id, consecutive_failures, VALIDATED_PEER_FAILURE_LIMIT
            );
            return;
        }
        FailedPeerAction::Forget => {}
    }

    if let Some(addr) = addrs.borrow_mut().get_mut(peri_id) {
        *addr = None;
    }

    // An address learned by the current scan is not trusted until product
    // validation succeeds, so forget it immediately. A previously validated
    // peer gets a bounded retry window before it is cleared, allowing both
    // transient recovery and eventual replacement/re-pairing.
    #[cfg(feature = "storage")]
    FLASH_CHANNEL
        .send(FlashOperationMessage::PeerAddress(PeerAddress::new(
            peri_id as u8,
            false,
            [0; 6],
        )))
        .await;
}

async fn commit_peer_candidate(peri_id: usize, peer_address: [u8; 6]) {
    if !take_uncommitted_peer_candidate(peri_id) {
        return;
    }

    #[cfg(feature = "storage")]
    FLASH_CHANNEL
        .send(FlashOperationMessage::PeerAddress(PeerAddress::new(
            peri_id as u8,
            true,
            peer_address,
        )))
        .await;
}

pub(crate) async fn run_ble_peripheral_manager<
    'b,
    's: 'b,
    C: Controller
        + ControllerCmdSync<LeSetScanParams>
        + ControllerCmdAsync<LeSetPhy>
        + ControllerCmdSync<LeReadLocalSupportedFeatures>,
    const ROW: usize,
    const COL: usize,
    const ROW_OFFSET: usize,
    const COL_OFFSET: usize,
>(
    peri_id: usize,
    addrs: &RefCell<VecView<Option<[u8; 6]>>>,
    stack: &'b Stack<'s, C, DefaultPacketPool>,
    profile: SplitLinkProfile,
) {
    trace!("SPLIT_MESSAGE_MAX_SIZE: {}", SPLIT_MESSAGE_MAX_SIZE);
    let mut peer_retry_state = PeerRetryState::default();

    loop {
        // Check until the address is available
        let address = loop {
            if let Some(Some(addr)) = addrs.borrow().get(peri_id) {
                break Address::random(*addr);
            }
            if !START_SCANNING.signaled() {
                START_SCANNING.signal(());
            }
            // Check again after 500ms
            embassy_time::Timer::after_millis(500).await;
        };
        info!("Peripheral peer address: {:?}", address);

        let mut central = stack.central();
        let active_profile = effective_split_link_profile(peri_id, profile);
        let timing = split_scan_timing();
        let config = ConnectConfig {
            connect_params: active_central_conn_param(active_profile),
            scan_config: ScanConfig {
                filter_accept_list: &[address],
                active: false,
                interval: timing.interval,
                window: timing.window,
                ..Default::default()
            },
        };
        wait_for_stack_started().await;

        publish_peripheral_connection(peri_id, false);

        // Connect to peripheral
        match with_timeout(Duration::from_secs(5), async {
            if let Ok(_guard) = SCANNING_MUTEX.try_lock() {
                info!("Start connecting to peripheral {}", peri_id);
                central.connect(&config).await
            } else {
                STOP_SCANNING.signal(());
                let _guard = SCANNING_MUTEX.lock().await;
                // Wait a little bit to ensure that the scanning has been fully stopped
                embassy_time::Timer::after_millis(100).await;
                info!("Start connecting to peripheral {}", peri_id);
                central.connect(&config).await
            }
        })
        .await
        {
            Ok(Ok(conn)) => {
                info!("Connected to peripheral {}", peri_id);
                let peer_validated = Cell::new(false);

                if let Err(e) = run_central_manager_task::<_, _, ROW, COL, ROW_OFFSET, COL_OFFSET>(
                    peri_id,
                    address.addr.into_inner(),
                    stack,
                    &conn,
                    &peer_validated,
                    profile,
                )
                .await
                {
                    #[cfg(feature = "defmt")]
                    let e = defmt::Debug2Format(&e);
                    error!("BLE central error: {:?}", e);
                }
                publish_peripheral_connection(peri_id, false);
                if peer_validated.get() {
                    peer_retry_state.on_validated();
                } else {
                    warn!("Split peripheral {} disconnected before validation", peri_id);
                    handle_failed_peer(peri_id, addrs, &mut peer_retry_state).await;
                }
            }
            Ok(Err(e)) => {
                #[cfg(feature = "defmt")]
                let e = defmt::Debug2Format(&e);
                error!("Connect to peripheral {} error: {:?}", peri_id, e);
                handle_failed_peer(peri_id, addrs, &mut peer_retry_state).await;
            }
            Err(_) => {
                warn!("Connect to peripheral {} timeout", peri_id);
                handle_failed_peer(peri_id, addrs, &mut peer_retry_state).await;
            }
        }
        // Reconnect after 500ms
        embassy_time::Timer::after_millis(500).await;
    }
}

fn active_central_conn_param(profile: SplitLinkProfile) -> RequestedConnParams {
    let interval = match profile {
        SplitLinkProfile::Keyboard => Duration::from_millis(15),
        SplitLinkProfile::Pointing => Duration::from_micros(7_500),
    };
    RequestedConnParams {
        min_connection_interval: interval,
        max_connection_interval: interval,
        // Active split links must attend every event. In particular, a
        // pointing link cannot sustain 125 Hz with peripheral latency.
        max_latency: 0,
        supervision_timeout: Duration::from_secs(5),
        ..Default::default()
    }
}

fn sleeping_central_conn_param() -> RequestedConnParams {
    RequestedConnParams {
        // Keep a short base interval so a peripheral with queued key events
        // can attend the next connection event and drain the burst promptly.
        // Slave latency retains an effective idle cadence of about 210 ms
        // (30 ms * 7) while avoiding the old 200 ms-per-event wake backlog.
        min_connection_interval: Duration::from_millis(30),
        max_connection_interval: Duration::from_millis(30),
        max_latency: 6,
        supervision_timeout: Duration::from_secs(11),
        ..Default::default()
    }
}

async fn validate_split_product<T: SplitReader + SplitWriter>(driver: &mut T) -> bool {
    if let Err(e) = driver.write(&SplitMessage::ProductId(crate::SPLIT_PRODUCT_ID)).await {
        warn!("Split product check write failed: {:?}", e);
        return false;
    }

    match with_timeout(Duration::from_millis(1500), async {
        loop {
            match driver.read().await {
                Ok(SplitMessage::ProductId(product_id)) if product_id == crate::SPLIT_PRODUCT_ID => return true,
                Ok(SplitMessage::ProductId(product_id)) => {
                    warn!(
                        "Split product id mismatch: got {}, expected {}",
                        product_id,
                        crate::SPLIT_PRODUCT_ID
                    );
                    return false;
                }
                Ok(message) => debug!("Ignoring pre-handshake split message: {:?}", message),
                Err(e) => {
                    warn!("Split product check read failed: {:?}", e);
                    return false;
                }
            }
        }
    })
    .await
    {
        Ok(valid) => valid,
        Err(_) => {
            warn!("Split product check timeout");
            false
        }
    }
}

async fn run_central_manager_task<
    'b,
    's: 'b,
    C: Controller + ControllerCmdAsync<LeSetPhy> + ControllerCmdSync<LeReadLocalSupportedFeatures>,
    P: PacketPool,
    const ROW: usize,
    const COL: usize,
    const ROW_OFFSET: usize,
    const COL_OFFSET: usize,
>(
    id: usize,
    peer_address: [u8; 6],
    stack: &'b Stack<'s, C, P>,
    conn: &Connection<'b, P>,
    peer_validated: &Cell<bool>,
    profile: SplitLinkProfile,
) -> Result<(), BleHostError<C::Error>> {
    let client = GattClient::<C, P, 10>::new(stack, conn).await?;

    // Use 2M Phy.
    update_ble_phy(stack, conn).await;

    info!("Updating connection parameters for peripheral");
    let active_profile = effective_split_link_profile(id, profile);
    update_conn_params(stack, conn, &active_central_conn_param(active_profile)).await;

    match select3(
        ble_central_task(&client, conn),
        run_peripheral_manager::<_, _, ROW, COL, ROW_OFFSET, COL_OFFSET>(id, peer_address, &client, peer_validated),
        follow_sleep_state(stack, conn, id, profile),
    )
    .await
    {
        Either3::First(e) => e,
        Either3::Second(e) => e,
        Either3::Third(e) => e,
    }
}

async fn ble_central_task<'a, C: Controller + ControllerCmdAsync<LeSetPhy>, P: PacketPool>(
    client: &GattClient<'a, C, P, 10>,
    conn: &Connection<'a, P>,
) -> Result<(), BleHostError<C::Error>> {
    // Simply monitor connection status
    let conn_check = async {
        while conn.is_connected() {
            Timer::after(split_liveness_poll()).await;
        }
    };

    match select(client.task(), conn_check).await {
        Either::First(e) => e,
        Either::Second(_) => {
            info!("Connection lost");
            Ok(())
        }
    }
}

async fn run_peripheral_manager<
    'a,
    C: Controller + ControllerCmdAsync<LeSetPhy>,
    P: PacketPool,
    const ROW: usize,
    const COL: usize,
    const ROW_OFFSET: usize,
    const COL_OFFSET: usize,
>(
    id: usize,
    peer_address: [u8; 6],
    client: &GattClient<'a, C, P, 10>,
    peer_validated: &Cell<bool>,
) -> Result<(), BleHostError<C::Error>> {
    let services = client.services_by_uuid(&Uuid::new_long(SPLIT_SERVICE_UUID)).await?;
    info!("Services found");
    if let Some(service) = services.first() {
        let message_to_central = client
            .characteristic_by_uuid::<[u8; SPLIT_MESSAGE_MAX_SIZE]>(
                service,
                // uuid: 0e6313e3-bd0b-45c2-8d2e-37a2e8128bc3
                &Uuid::Uuid128([
                    195u8, 139u8, 18u8, 232u8, 162u8, 55u8, 46u8, 141u8, 194u8, 69u8, 11u8, 189u8, 227u8, 19u8, 99u8,
                    14u8,
                ]),
            )
            .await?;
        info!("Message to central found");
        let message_to_peripheral = client
            .characteristic_by_uuid::<[u8; SPLIT_MESSAGE_MAX_SIZE]>(
                service,
                // uuid: 4b3514fb-cae4-4d38-a097-3a2a3d1c3b9c
                &Uuid::Uuid128([
                    156u8, 59u8, 28u8, 61u8, 42u8, 58u8, 151u8, 160u8, 56u8, 77u8, 228u8, 202u8, 251u8, 20u8, 53u8,
                    75u8,
                ]),
            )
            .await?;
        info!("Subscribing notifications");
        let listener = client.subscribe(&message_to_central, false).await?;
        let mut split_ble_driver = BleSplitCentralDriver::new(listener, message_to_peripheral, client);
        if !validate_split_product(&mut split_ble_driver).await {
            warn!("Rejecting split peripheral {} after product validation", id);
            return Ok(());
        }
        peer_validated.set(true);
        commit_peer_candidate(id, peer_address).await;
        publish_peripheral_connection(id, true);

        let peripheral_manager = PeripheralManager::<ROW, COL, ROW_OFFSET, COL_OFFSET, _>::new(split_ble_driver, id);
        peripheral_manager.run().await;
        info!("Peripheral manager stopped");
    };
    Ok(())
}

/// Ble central driver which reads and writes the split message.
///
/// Different from serial, BLE split message is processed in a separate service.
/// The BLE service should keep running, it processes the split message in the callback, which is not async.
/// It's impossible to implement `SplitReader` or `SplitWriter` for BLE service,
/// so we need this wrapper to forward split message to channel.
pub(crate) struct BleSplitCentralDriver<'a, 'b, 'c, C: Controller + ControllerCmdAsync<LeSetPhy>, P: PacketPool> {
    // Listener for split message from peripheral
    listener: NotificationListener<'b, 512>,
    // Characteristic to send split message to peripheral
    message_to_peripheral: Characteristic<[u8; SPLIT_MESSAGE_MAX_SIZE]>,
    // Client
    client: &'c GattClient<'a, C, P, 10>,
}

impl<'a, 'b, 'c, C: Controller + ControllerCmdAsync<LeSetPhy>, P: PacketPool> BleSplitCentralDriver<'a, 'b, 'c, C, P> {
    pub(crate) fn new(
        listener: NotificationListener<'b, 512>,
        message_to_peripheral: Characteristic<[u8; SPLIT_MESSAGE_MAX_SIZE]>,
        client: &'c GattClient<'a, C, P, 10>,
    ) -> Self {
        Self {
            listener,
            message_to_peripheral,
            client,
        }
    }
}

impl<'a, 'b, 'c, C: Controller + ControllerCmdAsync<LeSetPhy>, P: PacketPool> SplitReader
    for BleSplitCentralDriver<'a, 'b, 'c, C, P>
{
    async fn read(&mut self) -> Result<SplitMessage, SplitDriverError> {
        let data = self.listener.next().await;
        let message = postcard::from_bytes(data.as_ref()).map_err(|_| SplitDriverError::DeserializeError)?;
        trace!("Received split message: {:?}", message);

        match &message {
            SplitMessage::Pointing(event) => update_pointing_activity_time(event),
            SplitMessage::Key(_) => report_activity(),
            _ => {}
        }

        Ok(message)
    }
}

impl<'a, 'b, 'c, C: Controller + ControllerCmdAsync<LeSetPhy>, P: PacketPool> SplitWriter
    for BleSplitCentralDriver<'a, 'b, 'c, C, P>
{
    async fn write(&mut self, message: &SplitMessage) -> Result<usize, SplitDriverError> {
        let mut buf = [0_u8; SPLIT_MESSAGE_MAX_SIZE];
        match encode_split_message(message, &mut buf) {
            Ok(encoded) => {
                if let Err(e) = self
                    .client
                    .write_characteristic_without_response(&self.message_to_peripheral, encoded)
                    .await
                {
                    if let BleHostError::BleHost(Error::NotFound) = e {
                        error!("Peripheral disconnected");
                        return Err(SplitDriverError::Disconnected);
                    }
                    #[cfg(feature = "defmt")]
                    let e = defmt::Debug2Format(&e);
                    error!("BLE message_to_peripheral_write error: {:?}", e);
                }
                return Ok(encoded.len());
            }
            Err(e) => error!("Postcard serialize split message error: {}", e),
        };

        Err(SplitDriverError::SerializeError)
    }
}

/// Wait for the BLE stack to start.
///
/// If the BLE stack has been started, wait 500ms then quit.
pub(crate) async fn wait_for_stack_started() {
    loop {
        if STACK_STARTED.signaled() {
            embassy_time::Timer::after_millis(500).await;
            break;
        }
        embassy_time::Timer::after_millis(500).await;
    }
}

/// Keep one split link synchronized with the keyboard-wide sleep manager.
/// The manager outlives every connection; this follower is recreated with the
/// link and owns only that link's connection parameters.
async fn follow_sleep_state<
    'b,
    's: 'b,
    C: Controller + ControllerCmdAsync<LeSetPhy> + ControllerCmdSync<LeReadLocalSupportedFeatures>,
    P: PacketPool,
>(
    stack: &'b Stack<'s, C, P>,
    conn: &Connection<'b, P>,
    peripheral_id: usize,
    generated_profile: SplitLinkProfile,
) -> Result<(), BleHostError<C::Error>> {
    let mut sleep_events = SleepStateEvent::subscriber();
    let profile_changed = LINK_PROFILE_CHANGED.get(peripheral_id);

    // A new link must follow the already-latched keyboard state. Treating link
    // creation as user input can wake the host and every other split link.
    let mut applied_sleeping = if is_sleeping() {
        info!("New split link inherits sleep mode");
        update_conn_params(stack, conn, &sleeping_central_conn_param()).await
    } else {
        false
    };
    let mut applied_profile = effective_split_link_profile(peripheral_id, generated_profile);
    loop {
        let update = if let Some(profile_changed) = profile_changed {
            select(sleep_events.next_event(), profile_changed.wait()).await
        } else {
            Either::First(sleep_events.next_event().await)
        };
        match update {
            Either::First(event) => {
                let sleeping = event.0;
                if sleeping == applied_sleeping {
                    continue;
                }

                let profile = effective_split_link_profile(peripheral_id, generated_profile);
                let conn_params = if sleeping {
                    info!("Split link entering sleep mode");
                    sleeping_central_conn_param()
                } else {
                    info!("Split link restoring active mode");
                    active_central_conn_param(profile)
                };
                if update_conn_params(stack, conn, &conn_params).await {
                    applied_sleeping = sleeping;
                    if !sleeping {
                        applied_profile = profile;
                    }
                }
            }
            Either::Second(_) => {
                let profile = effective_split_link_profile(peripheral_id, generated_profile);
                if applied_sleeping || profile == applied_profile {
                    continue;
                }
                info!("Split link applying runtime active profile");
                if update_conn_params(stack, conn, &active_central_conn_param(profile)).await {
                    applied_profile = profile;
                }
            }
        }
    }
}

fn quiet_period_remaining(now_ms: u32, last_activity_ms: u32, quiet_period: Duration) -> Duration {
    if last_activity_ms == 0 {
        return Duration::MIN;
    }

    quiet_period
        .checked_sub(Duration::from_millis(u64::from(now_ms.wrapping_sub(last_activity_ms))))
        .unwrap_or(Duration::MIN)
}

/// Record motion separately from general split activity so status-only work
/// can yield until the real-time pointing path is quiet.
fn update_pointing_activity_time(event: &PointingEvent) {
    let now_ms = Instant::now().as_millis() as u32;
    if event.has_relative_xy_motion(POINTING_ACTIVITY_THRESHOLD) {
        LAST_POINTING_ACTIVITY_MS.store(now_ms, Ordering::Release);
    }
    report_pointing_activity(event);
}

/// Return the time remaining before pointing has been idle for `quiet_period`.
///
/// Qube's display uses this to defer SPI rendering while relative motion is
/// arriving; it does not alter the connection cadence or sleep policy.
pub fn pointing_quiet_period_remaining(quiet_period: Duration) -> Duration {
    quiet_period_remaining(
        Instant::now().as_millis() as u32,
        LAST_POINTING_ACTIVITY_MS.load(Ordering::Acquire),
        quiet_period,
    )
}

#[cfg(test)]
mod advertisement_tests {
    use super::*;

    #[test]
    fn uncommitted_peer_is_forgotten_after_first_failure() {
        let mut retry = PeerRetryState::default();

        assert_eq!(retry.on_failed_attempt(true), FailedPeerAction::Forget);
    }

    #[test]
    fn validated_peer_is_retried_twice_then_forgotten() {
        let mut retry = PeerRetryState::default();

        assert_eq!(
            retry.on_failed_attempt(false),
            FailedPeerAction::Retain {
                consecutive_failures: 1
            }
        );
        assert_eq!(
            retry.on_failed_attempt(false),
            FailedPeerAction::Retain {
                consecutive_failures: 2
            }
        );
        assert_eq!(retry.on_failed_attempt(false), FailedPeerAction::Forget);
    }

    #[test]
    fn successful_validation_resets_peer_failure_count() {
        let mut retry = PeerRetryState::default();
        assert!(matches!(
            retry.on_failed_attempt(false),
            FailedPeerAction::Retain {
                consecutive_failures: 1
            }
        ));
        assert!(matches!(
            retry.on_failed_attempt(false),
            FailedPeerAction::Retain {
                consecutive_failures: 2
            }
        ));

        retry.on_validated();

        assert_eq!(
            retry.on_failed_attempt(false),
            FailedPeerAction::Retain {
                consecutive_failures: 1
            }
        );
    }

    #[test]
    fn split_scan_leaves_radio_time_for_established_links() {
        assert_eq!(
            split_scan_timing(),
            SplitScanTiming {
                interval: Duration::from_millis(100),
                window: Duration::from_millis(30),
            }
        );
        assert_eq!(split_liveness_poll(), Duration::from_millis(250));
    }

    fn current_advertisement(product_id: u16, peripheral_id: u8) -> [u8; 28] {
        let mut data = [0u8; 28];
        data[0..3].copy_from_slice(&[2, 0x01, LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED]);
        data[3] = 17;
        data[4] = 0x07;
        data[5..21].copy_from_slice(&SPLIT_SERVICE_UUID);
        data[21..28].copy_from_slice(&[
            6,
            0xff,
            (SPLIT_COMPANY_ID & 0xff) as u8,
            (SPLIT_COMPANY_ID >> 8) as u8,
            (product_id & 0xff) as u8,
            (product_id >> 8) as u8,
            peripheral_id,
        ]);
        data
    }

    #[test]
    fn current_advertisement_requires_matching_product() {
        let matching = current_advertisement(crate::SPLIT_PRODUCT_ID, 1);
        assert_eq!(split_peripheral_id_from_advertisement(&matching), Some(1));

        let mismatched = current_advertisement(crate::SPLIT_PRODUCT_ID.wrapping_add(1), 1);
        assert_eq!(split_peripheral_id_from_advertisement(&mismatched), None);
    }

    #[test]
    fn current_advertisement_requires_split_service() {
        let mut data = current_advertisement(crate::SPLIT_PRODUCT_ID, 0);
        data[4] = 0x06;
        assert_eq!(split_peripheral_id_from_advertisement(&data), None);
    }

    #[test]
    fn legacy_advertisement_remains_discoverable_during_migration() {
        let mut data = [0u8; 26];
        data[0..3].copy_from_slice(&[2, 0x01, LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED]);
        data[3] = 17;
        data[4] = 0x07;
        data[5..21].copy_from_slice(&SPLIT_SERVICE_UUID);
        data[21..26].copy_from_slice(&[4, 0xff, 0x18, 0xe1, 1]);

        assert_eq!(legacy_split_peripheral_id_from_advertisement(&data), Some(1));
    }

    #[test]
    fn pointing_profile_uses_7_5_ms_without_slave_latency() {
        let params = active_central_conn_param(SplitLinkProfile::Pointing);

        assert_eq!(params.min_connection_interval, Duration::from_micros(7_500));
        assert_eq!(params.max_connection_interval, Duration::from_micros(7_500));
        assert_eq!(params.max_latency, 0);
    }

    #[test]
    fn keyboard_profile_retains_15_ms_cadence() {
        let params = active_central_conn_param(SplitLinkProfile::Keyboard);

        assert_eq!(params.min_connection_interval, Duration::from_millis(15));
        assert_eq!(params.max_connection_interval, Duration::from_millis(15));
        assert_eq!(params.max_latency, 0);
    }

    #[test]
    fn runtime_profile_override_replaces_generated_profile_per_link() {
        assert_eq!(
            effective_split_link_profile(30, SplitLinkProfile::Keyboard),
            SplitLinkProfile::Keyboard
        );
        assert!(set_split_link_profile(30, SplitLinkProfile::Pointing));
        assert_eq!(
            effective_split_link_profile(30, SplitLinkProfile::Keyboard),
            SplitLinkProfile::Pointing
        );
        assert!(set_split_link_profile(30, SplitLinkProfile::Keyboard));
        assert_eq!(
            effective_split_link_profile(30, SplitLinkProfile::Pointing),
            SplitLinkProfile::Keyboard
        );
        assert!(!set_split_link_profile(32, SplitLinkProfile::Pointing));
    }

    #[test]
    fn pointing_quiet_period_waits_only_for_recent_motion() {
        let quiet_period = Duration::from_millis(100);

        assert_eq!(quiet_period_remaining(1_000, 0, quiet_period), Duration::MIN);
        assert_eq!(
            quiet_period_remaining(1_050, 1_000, quiet_period),
            Duration::from_millis(50)
        );
        assert_eq!(quiet_period_remaining(1_100, 1_000, quiet_period), Duration::MIN);
    }

    #[test]
    fn pointing_activity_threshold_ignores_pmw3610_settling_noise() {
        use crate::event::{Axis, AxisEvent, AxisValType};

        let event = |x, y| PointingEvent {
            device_id: 0,
            axes: [
                AxisEvent {
                    typ: AxisValType::Rel,
                    axis: Axis::X,
                    value: x,
                },
                AxisEvent {
                    typ: AxisValType::Rel,
                    axis: Axis::Y,
                    value: y,
                },
                AxisEvent {
                    typ: AxisValType::Rel,
                    axis: Axis::Z,
                    value: 0,
                },
            ],
        };

        assert!(!event(1, -1).has_relative_xy_motion(POINTING_ACTIVITY_THRESHOLD));
        assert!(event(2, 0).has_relative_xy_motion(POINTING_ACTIVITY_THRESHOLD));
    }

    #[test]
    fn pointing_quiet_period_handles_millisecond_counter_wrap() {
        let quiet_period = Duration::from_millis(100);
        let last = u32::MAX - 20;

        assert_eq!(
            quiet_period_remaining(29, last, quiet_period),
            Duration::from_millis(50)
        );
    }

    #[test]
    fn sleeping_split_link_keeps_short_burst_interval() {
        let params = sleeping_central_conn_param();

        assert_eq!(params.min_connection_interval, Duration::from_millis(30));
        assert_eq!(params.max_connection_interval, Duration::from_millis(30));
        assert_eq!(params.max_latency, 6);
        assert_eq!(
            params.max_connection_interval.as_millis() * (u64::from(params.max_latency) + 1),
            210
        );
    }
}

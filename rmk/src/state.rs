use core::cell::Cell;

use embassy_sync::blocking_mutex::Mutex;
use rmk_types::ble::BleState;
#[cfg(feature = "_ble")]
use rmk_types::ble::BleStatus;
use rmk_types::connection::{ConnectionStatus, ConnectionType, UsbState};

use crate::RawMutex;
#[cfg(feature = "_ble")]
use crate::event::{BleAdvertisingMode, BleAdvertisingModeEvent};
use crate::event::{ConnectionStatusChangeEvent, publish_event};

/// Single source of truth for transport state and routing. All writes go
/// through the mutator helpers below so the active-output cascade runs and
/// change events fire on every transition.
pub(crate) static CONNECTION_STATUS: Mutex<RawMutex, Cell<ConnectionStatus>> =
    Mutex::new(Cell::new(ConnectionStatus::new()));

#[cfg(feature = "_ble")]
static BLE_ADVERTISING_MODE: Mutex<RawMutex, Cell<BleAdvertisingMode>> =
    Mutex::new(Cell::new(BleAdvertisingMode::Pairing));

pub(crate) fn active_transport() -> Option<ConnectionType> {
    CONNECTION_STATUS.lock(|c| c.get().decide_active())
}

/// Return the authoritative transport state snapshot.
///
/// Runtime processors can use this once during startup to recover transitions
/// published before their event subscriptions were installed.
pub fn current_connection_status() -> ConnectionStatus {
    CONNECTION_STATUS.lock(|c| c.get())
}

/// Re-publish the current status for user-visible actions that do not mutate
/// it, such as selecting the already-active BLE profile.
pub(crate) fn notify_connection_status() {
    publish_event(ConnectionStatusChangeEvent(current_connection_status()));
}

pub(crate) fn current_usb_state() -> UsbState {
    CONNECTION_STATUS.lock(|c| c.get().usb)
}

#[cfg(feature = "_ble")]
pub(crate) fn current_ble_status() -> BleStatus {
    CONNECTION_STATUS.lock(|c| c.get().ble)
}

/// Read-modify-write the connection status atomically.
pub(crate) fn update_status(f: impl FnOnce(&mut ConnectionStatus)) {
    let Some((prev, new)) = CONNECTION_STATUS.lock(|c| {
        let prev = c.get();
        let mut new = prev;
        f(&mut new);
        if prev == new {
            return None;
        }
        c.set(new);
        Some((prev, new))
    }) else {
        return;
    };

    let prev_active = prev.decide_active();
    let new_active = new.decide_active();

    if prev_active != new_active
        && let Some(prev_active) = prev_active
    {
        // Drain after the commit so any producer racing past the mutex reads
        // the new state and routes to the new channel rather than the one
        // about to be cleared.
        crate::channel::clear_and_release_report_channel(prev_active);
    }

    #[cfg(feature = "_ble")]
    if prev.ble.state == BleState::Sleeping && new_active != Some(ConnectionType::Ble) {
        // Reports accumulated for a sleeping bonded host must survive only the
        // Sleeping -> Connected handoff. A profile/transport switch abandons
        // that wake attempt and must not replay its keystrokes to another host.
        crate::channel::BLE_REPORT_CHANNEL.clear();
    }

    publish_event(ConnectionStatusChangeEvent(new));
}

pub fn set_usb_state(s: UsbState) {
    update_status(|c| c.usb = s);
}

pub(crate) fn set_ble_state(s: BleState) {
    update_status(|c| c.ble.state = s);
}

#[cfg(feature = "_ble")]
pub(crate) fn set_ble_advertising_mode(mode: BleAdvertisingMode) {
    BLE_ADVERTISING_MODE.lock(|current| current.set(mode));
    publish_event(BleAdvertisingModeEvent(mode));
}

/// Return the authoritative host-advertising mode snapshot.
///
/// This complements [`current_connection_status`] for indicators that need to
/// distinguish pairing from reconnecting without changing the public BLE wire
/// format.
#[cfg(feature = "_ble")]
pub fn current_ble_advertising_mode() -> BleAdvertisingMode {
    BLE_ADVERTISING_MODE.lock(|current| current.get())
}

/// Switching profiles always drops the BLE state back to `Inactive`; the
/// connection loop re-advertises and updates state from there.
pub(crate) fn set_ble_profile(profile: u8) {
    update_status(|c| {
        c.ble.profile = profile;
        c.ble.state = BleState::Inactive;
    });
}

/// Persistence is the caller's responsibility — enqueue
/// `FlashOperationMessage::ConnectionType` on `FLASH_CHANNEL`.
pub(crate) fn set_preferred_connection(t: ConnectionType) {
    update_status(|c| c.preferred = t);
}

/// Set and persist the preferred transport when storage is available.
pub(crate) async fn set_preferred_connection_persistent(t: ConnectionType) {
    set_preferred_connection(t);
    #[cfg(feature = "storage")]
    crate::channel::FLASH_CHANNEL
        .send(crate::storage::FlashOperationMessage::ConnectionType(t))
        .await;
}

/// Load the preferred connection type at startup.
///
/// With the `storage` feature, reads the persisted `ConnectionType` from flash;
/// otherwise falls back to a build-time default — `Ble` when USB is disabled, `Usb` otherwise.
#[cfg(feature = "_ble")]
pub(crate) async fn load_preferred_connection() -> ConnectionType {
    #[cfg(feature = "storage")]
    let stored = crate::storage::read_connection_type().await;
    #[cfg(not(feature = "storage"))]
    let stored: Option<ConnectionType> = None;
    match stored {
        Some(c) => c,
        #[cfg(feature = "_no_usb")]
        None => ConnectionType::Ble,
        #[cfg(not(feature = "_no_usb"))]
        None => ConnectionType::Usb,
    }
}

#[cfg(all(feature = "_ble", not(feature = "_no_usb")))]
pub(crate) async fn toggle_preferred() {
    let mut new = ConnectionType::Usb;
    update_status(|c| {
        c.preferred = match c.preferred {
            ConnectionType::Usb => ConnectionType::Ble,
            ConnectionType::Ble => ConnectionType::Usb,
        };
        new = c.preferred;
    });
    info!("Switching preferred transport to: {:?}", new);
    #[cfg(feature = "storage")]
    crate::channel::FLASH_CHANNEL
        .send(crate::storage::FlashOperationMessage::ConnectionType(new))
        .await;
}

#[cfg(feature = "_ble")]
pub(crate) fn current_profile() -> u8 {
    CONNECTION_STATUS.lock(|c| c.get().ble.profile)
}

#[cfg(test)]
mod tests {
    use std::sync::{Mutex, OnceLock};

    use embassy_futures::select::{Either, select};
    use embassy_time::{Duration, Timer};
    #[cfg(feature = "_ble")]
    use rmk_types::ble::BleState;

    #[cfg(feature = "_ble")]
    use super::set_ble_state;
    use super::{
        CONNECTION_STATUS, ConnectionStatus, ConnectionType, UsbState, set_preferred_connection, set_usb_state,
    };
    use crate::event::{ConnectionStatusChangeEvent, EventSubscriber, SubscribableEvent};
    use crate::hid::{KeyboardReport, Report};
    use crate::test_support::test_block_on as block_on;

    fn state_test_lock() -> &'static Mutex<()> {
        static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        LOCK.get_or_init(|| Mutex::new(()))
    }

    fn reset_state() {
        CONNECTION_STATUS.lock(|c| c.set(ConnectionStatus::default()));
        #[cfg(not(feature = "_no_usb"))]
        crate::channel::USB_REPORT_CHANNEL.clear();
        #[cfg(feature = "_ble")]
        crate::channel::BLE_REPORT_CHANNEL.clear();
    }

    fn pressed_keyboard_report() -> Report {
        Report::KeyboardReport(KeyboardReport {
            modifier: 0x02,
            reserved: 0,
            leds: 0,
            keycodes: [4, 0, 0, 0, 0, 0],
        })
    }

    fn assert_all_up_keyboard_report(report: Report) {
        match report {
            Report::KeyboardReport(r) => {
                assert_eq!(r.modifier, 0);
                assert_eq!(r.reserved, 0);
                assert_eq!(r.leds, 0);
                assert_eq!(r.keycodes, [0; 6]);
            }
            _ => panic!("expected keyboard all-up report"),
        }
    }

    #[test]
    fn preferred_transport_change_publishes_status_event() {
        let _guard = state_test_lock().lock().unwrap();
        reset_state();
        set_usb_state(UsbState::Configured);
        let mut sub = ConnectionStatusChangeEvent::subscriber();

        set_preferred_connection(ConnectionType::Ble);

        let event = block_on(sub.next_event());
        assert_eq!(event.0.preferred, ConnectionType::Ble);
    }

    #[test]
    fn usb_state_change_publishes_status_event() {
        let _guard = state_test_lock().lock().unwrap();
        reset_state();
        let mut sub = ConnectionStatusChangeEvent::subscriber();

        set_usb_state(UsbState::Configured);

        let event = block_on(sub.next_event());
        assert_eq!(event.0.usb, UsbState::Configured);
    }

    #[test]
    fn unchanged_status_does_not_publish_event() {
        let _guard = state_test_lock().lock().unwrap();
        reset_state();
        set_usb_state(UsbState::Configured);
        let mut sub = ConnectionStatusChangeEvent::subscriber();

        // Re-setting the same value should not publish.
        set_usb_state(UsbState::Configured);

        match block_on(select(Timer::after(Duration::from_millis(1)), sub.next_event())) {
            Either::First(_) => {}
            Either::Second(event) => panic!("unexpected status change event: {:?}", event),
        }
    }

    #[cfg(feature = "_ble")]
    #[test]
    fn wake_report_queues_without_blocking_keyboard_processing() {
        use crate::channel::{BLE_REPORT_CHANNEL, send_hid_report};

        let _guard = state_test_lock().lock().unwrap();
        reset_state();
        set_ble_state(BleState::Sleeping);

        block_on(async {
            send_hid_report(pressed_keyboard_report()).await;
            send_hid_report(Report::KeyboardReport(KeyboardReport::default())).await;
        });

        match BLE_REPORT_CHANNEL
            .try_receive()
            .expect("the wake report should be retained before BLE reconnects")
        {
            Report::KeyboardReport(report) => assert_eq!(report.keycodes[0], 4),
            _ => panic!("expected keyboard wake report"),
        }
        assert_all_up_keyboard_report(
            BLE_REPORT_CHANNEL
                .try_receive()
                .expect("the wake key release should queue behind its press"),
        );
    }

    #[cfg(all(not(feature = "_no_usb"), feature = "_ble"))]
    #[test]
    fn abandoning_sleep_for_usb_discards_pending_ble_reports() {
        use crate::channel::{BLE_REPORT_CHANNEL, send_hid_report};

        let _guard = state_test_lock().lock().unwrap();
        reset_state();
        set_preferred_connection(ConnectionType::Usb);
        set_ble_state(BleState::Sleeping);
        block_on(send_hid_report(pressed_keyboard_report()));
        assert_eq!(BLE_REPORT_CHANNEL.len(), 1);

        set_usb_state(UsbState::Configured);

        assert!(BLE_REPORT_CHANNEL.try_receive().is_err());
    }

    #[cfg(not(feature = "_no_usb"))]
    #[test]
    fn flipping_away_from_active_clears_stale_reports_and_queues_all_up() {
        use crate::channel::USB_REPORT_CHANNEL;

        let _guard = state_test_lock().lock().unwrap();
        reset_state();
        set_usb_state(UsbState::Configured);
        assert_eq!(super::active_transport(), Some(ConnectionType::Usb));

        // Drain anything left over from earlier tests, then queue a sentinel
        // that would otherwise persist across a flip.
        USB_REPORT_CHANNEL.clear();
        USB_REPORT_CHANNEL
            .try_send(pressed_keyboard_report())
            .expect("channel should have capacity for sentinel");
        assert!(USB_REPORT_CHANNEL.try_receive().is_ok());
        USB_REPORT_CHANNEL
            .try_send(pressed_keyboard_report())
            .expect("channel should have capacity for sentinel");

        set_usb_state(UsbState::Disabled);
        assert!(super::active_transport().is_none());
        assert_all_up_keyboard_report(
            USB_REPORT_CHANNEL
                .try_receive()
                .expect("USB_REPORT_CHANNEL should contain keyboard all-up report"),
        );
        assert!(
            USB_REPORT_CHANNEL.try_receive().is_err(),
            "USB_REPORT_CHANNEL should contain only the all-up report"
        );
    }

    #[cfg(not(feature = "_no_usb"))]
    #[test]
    fn blocked_send_drops_report_after_transport_change() {
        use embassy_futures::join::join;

        use crate::channel::{USB_REPORT_CHANNEL, send_hid_report};

        let _guard = state_test_lock().lock().unwrap();
        reset_state();
        set_usb_state(UsbState::Configured);

        for _ in 0..crate::REPORT_CHANNEL_SIZE {
            USB_REPORT_CHANNEL
                .try_send(pressed_keyboard_report())
                .expect("channel should have capacity while filling");
        }

        block_on(join(
            send_hid_report(Report::KeyboardReport(KeyboardReport::default())),
            async {
                Timer::after(Duration::from_millis(1)).await;
                set_usb_state(UsbState::Disabled);
            },
        ));

        assert_all_up_keyboard_report(
            USB_REPORT_CHANNEL
                .try_receive()
                .expect("USB_REPORT_CHANNEL should contain keyboard all-up report"),
        );
        assert!(
            USB_REPORT_CHANNEL.try_receive().is_err(),
            "USB_REPORT_CHANNEL should contain only the all-up report"
        );
    }

    #[cfg(all(not(feature = "_no_usb"), feature = "_ble"))]
    #[test]
    fn usb_preference_flip_releases_previous_ble_transport() {
        use crate::channel::BLE_REPORT_CHANNEL;
        use crate::state::{BleState, set_ble_state};

        let _guard = state_test_lock().lock().unwrap();
        reset_state();
        set_preferred_connection(ConnectionType::Usb);
        set_ble_state(BleState::Connected);
        assert_eq!(super::active_transport(), Some(ConnectionType::Ble));

        BLE_REPORT_CHANNEL
            .try_send(pressed_keyboard_report())
            .expect("BLE report channel should have capacity for sentinel");

        set_usb_state(UsbState::Configured);
        assert_eq!(super::active_transport(), Some(ConnectionType::Usb));
        assert_all_up_keyboard_report(
            BLE_REPORT_CHANNEL
                .try_receive()
                .expect("BLE_REPORT_CHANNEL should contain keyboard all-up report"),
        );
        assert!(
            BLE_REPORT_CHANNEL.try_receive().is_err(),
            "BLE_REPORT_CHANNEL should contain only the all-up report"
        );
    }

    #[cfg(not(feature = "_no_usb"))]
    #[test]
    fn blocked_send_enqueues_when_transport_stays_active() {
        use embassy_futures::join::join;

        use crate::channel::{USB_REPORT_CHANNEL, send_hid_report};

        let _guard = state_test_lock().lock().unwrap();
        reset_state();
        set_usb_state(UsbState::Configured);

        for _ in 0..crate::REPORT_CHANNEL_SIZE {
            USB_REPORT_CHANNEL
                .try_send(Report::KeyboardReport(KeyboardReport::default()))
                .expect("channel should have capacity while filling");
        }

        block_on(join(
            send_hid_report(Report::KeyboardReport(KeyboardReport::default())),
            async {
                Timer::after(Duration::from_millis(1)).await;
                let _ = USB_REPORT_CHANNEL.try_receive();
            },
        ));

        assert_eq!(USB_REPORT_CHANNEL.len(), crate::REPORT_CHANNEL_SIZE);
    }
}

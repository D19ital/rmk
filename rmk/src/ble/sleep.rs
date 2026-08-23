//! Keyboard-wide BLE sleep management.
//!
//! One always-running manager owns the sleep state. Connection-specific tasks
//! only follow that state, so disconnecting or recreating a host/split link
//! cannot leave the keyboard permanently asleep.

use core::cell::Cell;

use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};

use crate::SPLIT_CENTRAL_SLEEP_TIMEOUT_SECONDS;
use crate::event::{EventSubscriber, KeyboardEvent, PointingEvent, SleepStateEvent, SubscribableEvent, publish_event};

/// Latched keyboard sleep state for synchronous users such as the battery
/// service. A blocking mutex keeps this compatible with ARMv6-M targets, which
/// do not provide every atomic read-modify-write operation.
static SLEEPING_STATE: BlockingMutex<crate::RawMutex, Cell<bool>> = BlockingMutex::new(Cell::new(false));

/// Input to [`run_sleep_manager`]: `true` requests immediate sleep, while
/// `false` reports activity and wakes or restarts the idle timeout.
static SLEEP_INPUT: Signal<crate::RawMutex, bool> = Signal::new();

/// Mirrored activity/suspend input for the host BLE power policy. Keeping a
/// separate signal lets the persistent sleep manager and a connected host
/// session observe every state change independently.
static HOST_POWER_INPUT: Signal<crate::RawMutex, bool> = Signal::new();

pub(crate) fn is_sleeping() -> bool {
    SLEEPING_STATE.lock(Cell::get)
}

fn set_sleeping(next: bool) {
    SLEEPING_STATE.lock(|state| state.set(next));
}

/// Report keyboard activity: wake the keyboard or restart its idle timeout.
pub(crate) fn report_activity() {
    SLEEP_INPUT.signal(false);
    HOST_POWER_INPUT.signal(false);
}

/// Report pointing activity only when the event represents real user input.
pub(crate) fn report_pointing_activity(event: &PointingEvent) {
    if event.is_user_activity() {
        report_activity();
    }
}

/// Input subscriptions created before a host disconnect or advertising
/// attempt so no wake event is lost in the transition window.
pub(crate) struct InputActivityWaiter {
    key_wake: <KeyboardEvent as SubscribableEvent>::Subscriber,
    pointing_wake: <PointingEvent as SubscribableEvent>::Subscriber,
}

impl InputActivityWaiter {
    pub(crate) fn new() -> Self {
        Self {
            key_wake: KeyboardEvent::subscriber(),
            pointing_wake: PointingEvent::subscriber(),
        }
    }

    pub(crate) async fn wait(mut self) {
        loop {
            match select(self.key_wake.next_event(), self.pointing_wake.next_event()).await {
                Either::First(_) => return,
                Either::Second(event) if event.is_user_activity() => return,
                Either::Second(_) => {}
            }
        }
    }
}

/// Wait for a key or a meaningful pointing report, ignoring sensor noise.
pub(crate) async fn wait_for_input_activity() {
    InputActivityWaiter::new().wait().await;
}

pub(crate) fn reset_host_power_input() {
    HOST_POWER_INPUT.reset();
}

pub(crate) async fn wait_for_host_power_input() -> bool {
    HOST_POWER_INPUT.wait().await
}

pub(crate) fn take_host_power_input() -> Option<bool> {
    HOST_POWER_INPUT.try_take()
}

/// Request sleep immediately instead of waiting for the idle timeout.
pub(crate) fn request_local_sleep() {
    SLEEP_INPUT.signal(true);
}

/// Request keyboard sleep and terminate the connected host session.
pub(crate) fn request_sleep() {
    request_local_sleep();
    HOST_POWER_INPUT.signal(true);
}

/// Run the single persistent sleep manager alongside the BLE transport.
pub(crate) async fn run_sleep_manager() {
    if SPLIT_CENTRAL_SLEEP_TIMEOUT_SECONDS == 0 {
        info!("Sleep management disabled (timeout = 0)");
        core::future::pending::<()>().await;
        return;
    }

    info!(
        "Sleep manager started with {}s timeout",
        SPLIT_CENTRAL_SLEEP_TIMEOUT_SECONDS
    );
    manage_sleep_state(Duration::from_secs(SPLIT_CENTRAL_SLEEP_TIMEOUT_SECONDS.into())).await
}

async fn manage_sleep_state(idle_timeout: Duration) -> ! {
    loop {
        // Poll activity first so it wins a race with the idle timeout.
        loop {
            match select(SLEEP_INPUT.wait(), Timer::after(idle_timeout)).await {
                Either::First(true) | Either::Second(_) => break,
                Either::First(false) => debug!("Activity detected, resetting sleep timeout"),
            }
        }

        info!("Entering sleep mode");
        set_sleeping(true);
        publish_event(SleepStateEvent::new(true));

        // Further sleep requests are idempotent; only activity wakes us.
        while SLEEP_INPUT.wait().await {}

        info!("Waking up from sleep mode due to activity");
        set_sleeping(false);
        publish_event(SleepStateEvent::new(false));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::test_block_on as block_on;

    fn with_sleep_manager(script: impl core::future::Future<Output = ()>) {
        SLEEP_INPUT.reset();
        HOST_POWER_INPUT.reset();
        set_sleeping(false);
        block_on(async {
            select(manage_sleep_state(Duration::from_secs(1)), script).await;
        });
    }

    #[test]
    fn sleeps_when_idle_and_wakes_on_activity() {
        with_sleep_manager(async {
            Timer::after_millis(900).await;
            assert!(!is_sleeping(), "still inside the idle timeout");

            Timer::after_millis(200).await;
            assert!(is_sleeping(), "idle timeout elapsed");

            report_activity();
            Timer::after_millis(10).await;
            assert!(!is_sleeping(), "activity wakes the keyboard");
        });
    }

    #[test]
    fn activity_restarts_the_idle_timeout() {
        with_sleep_manager(async {
            for _ in 0..3 {
                Timer::after_millis(600).await;
                assert!(!is_sleeping(), "activity must restart the idle timeout");
                report_activity();
            }
        });
    }

    #[test]
    fn sleep_request_skips_the_idle_timeout() {
        with_sleep_manager(async {
            request_sleep();
            Timer::after_millis(10).await;
            assert!(is_sleeping(), "a sleep request must not wait for the timeout");

            request_sleep();
            Timer::after_millis(10).await;
            assert!(is_sleeping(), "repeated sleep requests are idempotent");

            report_activity();
            Timer::after_millis(10).await;
            assert!(!is_sleeping());
        });
    }

    #[test]
    fn local_sleep_request_keeps_the_host_session_alive() {
        SLEEP_INPUT.reset();
        HOST_POWER_INPUT.reset();

        request_local_sleep();

        assert_eq!(SLEEP_INPUT.try_take(), Some(true));
        assert_eq!(take_host_power_input(), None);
    }

    #[test]
    fn full_sleep_request_notifies_both_managers() {
        SLEEP_INPUT.reset();
        HOST_POWER_INPUT.reset();

        request_sleep();

        assert_eq!(SLEEP_INPUT.try_take(), Some(true));
        assert_eq!(take_host_power_input(), Some(true));
    }
}

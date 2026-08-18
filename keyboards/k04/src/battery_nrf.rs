//! Ergohaven split battery (AIN7/P0_31, 1564k/2370k).
//!
//! Stock RMK `battery_adc_pin` is not used: its NrfAdc path calls
//! `calibrate().await`, which can hang under SoftDevice and leave the dongle
//! on "??" forever. This reader skips calibrate, times out samples, and
//! re-publishes BatteryStatusEvent periodically so the split loop can forward.

use embassy_futures::select::select;
use embassy_nrf::interrupt;
use embassy_nrf::interrupt::InterruptExt;
use embassy_nrf::peripherals::{P0_31, SAADC};
use embassy_nrf::saadc::{self, Input as _, Saadc};
use embassy_nrf::{bind_interrupts, Peri};
use embassy_time::{with_timeout, Duration, Timer};
use rmk::core_traits::Runnable;
use rmk::event::{
    publish_event, BatteryStatusEvent, EventSubscriber, PeripheralBatteryRefreshEvent, SubscribableEvent,
};
use rmk::input_device::battery::publish_battery_status;
use rmk::processor::Processor;
use rmk::types::battery::{BatteryStatus, ChargeState};

use crate::battery_level::BatteryLevelTracker;

bind_interrupts!(struct SaadcIrqs {
    SAADC => saadc::InterruptHandler;
});

const EMPTY_MV: i32 = 3300;
const FULL_MV: i32 = 4100;
// Calibrated from K:04 Qube halves: raw ~= 2520 at 3.150 V and 2600 at 3.281 V.
const RAW_PER_MV_NUM: i32 = 4;
const RAW_PER_MV_DEN: i32 = 5;
const CHARGING_SAMPLE_INTERVAL: Duration = Duration::from_secs(2);
const DISCHARGING_SAMPLE_INTERVAL: Duration = Duration::from_secs(15);

fn percent(val: u16) -> u8 {
    let val = val as i32;
    let empty = EMPTY_MV * RAW_PER_MV_NUM / RAW_PER_MV_DEN;
    let full = FULL_MV * RAW_PER_MV_NUM / RAW_PER_MV_DEN;

    if val >= full {
        100
    } else if val <= empty {
        0
    } else {
        (((val - empty) * 100) / (full - empty)) as u8
    }
}

#[allow(dead_code)]
pub type K04Battery = BatteryReader<true>;
#[allow(dead_code)]
pub type K04QubeBattery = BatteryReader<false>;

pub struct BatteryReader<const CACHE_HOST_STATUS: bool> {
    saadc: Saadc<'static, 1>,
    level: BatteryLevelTracker,
}

impl<const CACHE_HOST_STATUS: bool> BatteryReader<CACHE_HOST_STATUS> {
    pub fn new(saadc: Peri<'static, SAADC>, pin: Peri<'static, P0_31>) -> Self {
        interrupt::SAADC.set_priority(interrupt::Priority::P3);
        let channel = saadc::ChannelConfig::single_ended(pin.degrade_saadc());
        Self {
            saadc: Saadc::new(saadc, SaadcIrqs, saadc::Config::default(), [channel]),
            level: BatteryLevelTracker::default(),
        }
    }

    async fn read_raw(&mut self) -> Option<u16> {
        let mut buf = [0i16; 1];
        with_timeout(Duration::from_millis(200), self.saadc.sample(&mut buf))
            .await
            .ok()?;
        Some(buf[0].max(0) as u16)
    }

    async fn sample_raw(&mut self) -> Option<u16> {
        // The first conversion after idle can be biased by the SAADC sample
        // capacitor. Discard it, then let the high-impedance divider settle.
        self.read_raw().await?;
        Timer::after(Duration::from_millis(30)).await;

        let mut sum = 0u32;
        let mut count = 0u32;

        for _ in 0..3 {
            if let Some(raw) = self.read_raw().await {
                sum += raw as u32;
                count += 1;
            }
            Timer::after(Duration::from_millis(30)).await;
        }

        (count > 0).then_some((sum / count) as u16)
    }

    async fn publish_sample(&mut self) {
        let charge_state = current_charge_state();
        let charging = charge_state == ChargeState::Charging;
        let measured = if charging {
            // VBUS only tells us that charging power is present. The charger
            // raises terminal voltage toward 4.1 V before the cell is full,
            // so publishing that sample would create a false 100% reading.
            None
        } else {
            self.sample_raw().await.map(percent)
        };
        let level = self.level.observe(measured, charging);
        let status = if charging || measured.is_some() {
            BatteryStatus::Available { charge_state, level }
        } else {
            BatteryStatus::Unavailable
        };
        if CACHE_HOST_STATUS {
            publish_battery_status(status);
        } else {
            publish_event(BatteryStatusEvent(status));
        }
    }
}

pub(crate) fn usb_powered() -> bool {
    embassy_nrf::pac::POWER.usbregstatus().read().vbusdetect()
}

fn current_charge_state() -> ChargeState {
    if usb_powered() {
        ChargeState::Charging
    } else {
        ChargeState::Discharging
    }
}

struct NeverSub;
pub struct NeverEvent;

impl EventSubscriber for NeverSub {
    type Event = NeverEvent;
    async fn next_event(&mut self) -> NeverEvent {
        core::future::pending().await
    }
}

impl<const CACHE_HOST_STATUS: bool> Runnable for BatteryReader<CACHE_HOST_STATUS> {
    async fn run(&mut self) -> ! {
        Timer::after(Duration::from_millis(1000)).await;
        let mut refresh_sub = PeripheralBatteryRefreshEvent::subscriber();
        loop {
            self.publish_sample().await;
            let interval = match current_charge_state() {
                ChargeState::Charging => CHARGING_SAMPLE_INTERVAL,
                _ => DISCHARGING_SAMPLE_INTERVAL,
            };
            let _ = select(Timer::after(interval), refresh_sub.next_event()).await;
        }
    }
}

impl<const CACHE_HOST_STATUS: bool> Processor for BatteryReader<CACHE_HOST_STATUS> {
    type Event = NeverEvent;
    fn subscriber() -> impl EventSubscriber<Event = NeverEvent> {
        NeverSub
    }
    async fn process(&mut self, _: NeverEvent) {}
    async fn process_loop(&mut self) -> ! {
        self.run().await
    }
}

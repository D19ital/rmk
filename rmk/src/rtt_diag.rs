//! Low-rate RTT diagnostics for the K:04 pointing path.
//!
//! Counters are updated in hot paths with relaxed atomics and emitted at most
//! once per second. This avoids changing scheduler timing with one log record
//! per motion packet.

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};

use embassy_time::Instant;

use crate::event::{Axis, AxisValType, PointingEvent};
use crate::hid::Report;
use crate::split::SplitMessage;

const REPORT_INTERVAL_MS: u32 = 1_000;
const MAX_RELEVANT_GAP_MS: u32 = 2_000;
const SLOW_GATT_US: u32 = 20_000;
const WAITING_GATT_US: u32 = 10_000;
const WAITING_SPLIT_US: u32 = 20_000;
const RAW_SIGN_MIN_ABS: u16 = 8;
const RAW_SIGN_CONTINUITY_MS: u32 = 750;
const RAW_NEAR_SATURATION_ABS: u16 = 1_900;

static NEXT_RIGHT_LOG_MS: AtomicU32 = AtomicU32::new(0);
static PMW_READS: AtomicU32 = AtomicU32::new(0);
static PMW_PUBLISHES: AtomicU32 = AtomicU32::new(0);
static PMW_ERRORS: AtomicU32 = AtomicU32::new(0);
static PMW_DX: AtomicI32 = AtomicI32::new(0);
static PMW_DY: AtomicI32 = AtomicI32::new(0);
static PMW_ACCUM_DROPPED_X: AtomicU32 = AtomicU32::new(0);
static PMW_ACCUM_DROPPED_Y: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_SAMPLES: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_MOTION_SET: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_X_POS: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_X_NEG: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_X_ZERO: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_X_POS_SUM: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_X_NEG_ABS_SUM: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_X_ABS_MAX: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_X_NEAR_SAT: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_X_FLIPS: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_LAST_X_SIGN: AtomicI32 = AtomicI32::new(0);
static PMW_RAW_LAST_X_SIGN_MS: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_Y_POS: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_Y_NEG: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_Y_ZERO: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_Y_POS_SUM: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_Y_NEG_ABS_SUM: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_Y_ABS_MAX: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_Y_NEAR_SAT: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_Y_FLIPS: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_LAST_Y_SIGN: AtomicI32 = AtomicI32::new(0);
static PMW_RAW_LAST_Y_SIGN_MS: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_XYH_OR: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_XYH_AND: AtomicU32 = AtomicU32::new(0xff);
static PMW_RAW_XYH_LAST: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_LAST_FLIP_MS: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_LAST_FLIP_AXIS: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_LAST_FLIP_MOTION: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_LAST_FLIP_X_L: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_LAST_FLIP_Y_L: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_LAST_FLIP_XY_H: AtomicU32 = AtomicU32::new(0);
static PMW_RAW_LAST_FLIP_DX: AtomicI32 = AtomicI32::new(0);
static PMW_RAW_LAST_FLIP_DY: AtomicI32 = AtomicI32::new(0);
static PMW_AXIS_SAMPLES: AtomicU32 = AtomicU32::new(0);
static PMW_AXIS_PRE_X_POS: AtomicU32 = AtomicU32::new(0);
static PMW_AXIS_PRE_X_NEG: AtomicU32 = AtomicU32::new(0);
static PMW_AXIS_PRE_Y_POS: AtomicU32 = AtomicU32::new(0);
static PMW_AXIS_PRE_Y_NEG: AtomicU32 = AtomicU32::new(0);
static PMW_AXIS_POST_X_POS: AtomicU32 = AtomicU32::new(0);
static PMW_AXIS_POST_X_NEG: AtomicU32 = AtomicU32::new(0);
static PMW_AXIS_POST_Y_POS: AtomicU32 = AtomicU32::new(0);
static PMW_AXIS_POST_Y_NEG: AtomicU32 = AtomicU32::new(0);
static PMW_AXIS_PRE_X_SUM: AtomicI32 = AtomicI32::new(0);
static PMW_AXIS_PRE_Y_SUM: AtomicI32 = AtomicI32::new(0);
static PMW_AXIS_POST_X_SUM: AtomicI32 = AtomicI32::new(0);
static PMW_AXIS_POST_Y_SUM: AtomicI32 = AtomicI32::new(0);
static PMW_AXIS_MISMATCH: AtomicU32 = AtomicU32::new(0);
static PMW_AXIS_LAST_PRE_X: AtomicI32 = AtomicI32::new(0);
static PMW_AXIS_LAST_PRE_Y: AtomicI32 = AtomicI32::new(0);
static PMW_AXIS_LAST_POST_X: AtomicI32 = AtomicI32::new(0);
static PMW_AXIS_LAST_POST_Y: AtomicI32 = AtomicI32::new(0);
static PMW_READ_MAX_US: AtomicU32 = AtomicU32::new(0);
static LAST_PMW_READ_MS: AtomicU32 = AtomicU32::new(0);
static PMW_MAX_GAP_MS: AtomicU32 = AtomicU32::new(0);
static LAST_PMW_PUBLISH_MS: AtomicU32 = AtomicU32::new(0);
static PMW_PUBLISH_MAX_GAP_MS: AtomicU32 = AtomicU32::new(0);
static MOTION_WAKE_COUNT: AtomicU32 = AtomicU32::new(0);
static MOTION_WAIT_MAX_US: AtomicU32 = AtomicU32::new(0);
static MOTION_PENDING_WAIT_MAX_US: AtomicU32 = AtomicU32::new(0);
static PMW_FALLBACK_READS: AtomicU32 = AtomicU32::new(0);
static PMW_FALLBACK_HITS: AtomicU32 = AtomicU32::new(0);
static PMW_GPIO_LOW_READS: AtomicU32 = AtomicU32::new(0);
static PMW_GPIO_LOW_ZERO: AtomicU32 = AtomicU32::new(0);
static PMW_OPTICS_SAMPLES: AtomicU32 = AtomicU32::new(0);
static PMW_OPTICS_MOTION_SET: AtomicU32 = AtomicU32::new(0);
static PMW_OPTICS_MOTION_CLEAR: AtomicU32 = AtomicU32::new(0);
static PMW_SQUAL_SUM: AtomicU32 = AtomicU32::new(0);
static PMW_SQUAL_MIN: AtomicU32 = AtomicU32::new(u32::MAX);
static PMW_SQUAL_MAX: AtomicU32 = AtomicU32::new(0);
static PMW_SHUTTER_SUM: AtomicU32 = AtomicU32::new(0);
static PMW_SHUTTER_MIN: AtomicU32 = AtomicU32::new(u32::MAX);
static PMW_SHUTTER_MAX: AtomicU32 = AtomicU32::new(0);
static PMW_SHUTTER_BELOW: AtomicU32 = AtomicU32::new(0);
static PMW_SHUTTER_EQUAL: AtomicU32 = AtomicU32::new(0);
static PMW_SHUTTER_ABOVE: AtomicU32 = AtomicU32::new(0);
static PMW_SMART_ENABLES: AtomicU32 = AtomicU32::new(0);
static PMW_SMART_DISABLES: AtomicU32 = AtomicU32::new(0);
static PMW_SMART_DISABLED: AtomicBool = AtomicBool::new(false);
static PMW_SMART_LAST_SQUAL: AtomicU32 = AtomicU32::new(0);
static PMW_SMART_LAST_SHUTTER: AtomicU32 = AtomicU32::new(0);
static SCHEDULER_TICKS: AtomicU32 = AtomicU32::new(0);
static SCHEDULER_LATE_MAX_US: AtomicU32 = AtomicU32::new(0);
static SCHEDULER_LATE_OVER_5_MS: AtomicU32 = AtomicU32::new(0);
static SPLIT_TX_POINTING: AtomicU32 = AtomicU32::new(0);
static SPLIT_TX_ERRORS: AtomicU32 = AtomicU32::new(0);
static SPLIT_TX_DX: AtomicI32 = AtomicI32::new(0);
static SPLIT_TX_DY: AtomicI32 = AtomicI32::new(0);
static SPLIT_TX_MAX_US: AtomicU32 = AtomicU32::new(0);
static LAST_SPLIT_TX_MS: AtomicU32 = AtomicU32::new(0);
static SPLIT_TX_MAX_GAP_MS: AtomicU32 = AtomicU32::new(0);
static SPLIT_TX_TOTAL_US: AtomicU32 = AtomicU32::new(0);
static SPLIT_TX_WAIT_OVER_20_MS: AtomicU32 = AtomicU32::new(0);
static SPLIT_TX_WAIT_STREAK_CURRENT: AtomicU32 = AtomicU32::new(0);
static SPLIT_TX_WAIT_STREAK_MAX: AtomicU32 = AtomicU32::new(0);

static NEXT_LEFT_LOG_MS: AtomicU32 = AtomicU32::new(0);
static SPLIT_RX_POINTING: AtomicU32 = AtomicU32::new(0);
static SPLIT_RX_DX: AtomicI32 = AtomicI32::new(0);
static SPLIT_RX_DY: AtomicI32 = AtomicI32::new(0);
static LAST_SPLIT_RX_MS: AtomicU32 = AtomicU32::new(0);
static SPLIT_RX_MAX_GAP_MS: AtomicU32 = AtomicU32::new(0);
static HID_ENQUEUE_ALL: AtomicU32 = AtomicU32::new(0);
static HID_ENQUEUE_MOUSE: AtomicU32 = AtomicU32::new(0);
static HID_ENQUEUE_FULL: AtomicU32 = AtomicU32::new(0);
static HID_ENQUEUE_ABORTED: AtomicU32 = AtomicU32::new(0);
static HID_QUEUE_HIGH_WATER: AtomicU32 = AtomicU32::new(0);
static HID_ENQUEUE_MAX_WAIT_US: AtomicU32 = AtomicU32::new(0);
static HID_WRITE_ALL: AtomicU32 = AtomicU32::new(0);
static HID_WRITE_MOUSE: AtomicU32 = AtomicU32::new(0);
static HID_WRITE_ERRORS: AtomicU32 = AtomicU32::new(0);
static HID_WRITE_SLOW: AtomicU32 = AtomicU32::new(0);
static HID_WRITE_MAX_US: AtomicU32 = AtomicU32::new(0);
static HID_WRITE_DX: AtomicI32 = AtomicI32::new(0);
static HID_WRITE_DY: AtomicI32 = AtomicI32::new(0);
static HID_MOUSE_MERGED: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_SPLIT_CHUNKS: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_CLIP_X_POS: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_CLIP_X_NEG: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_CLIP_Y_POS: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_CLIP_Y_NEG: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_RESIDUAL_X_MAX: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_RESIDUAL_Y_MAX: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_SOURCE_REPORTS: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_AGE_SAMPLES: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_AGE_TOTAL_US: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_AGE_MAX_US: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_AGE_OVER_15_MS: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_AGE_OVER_30_MS: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_AGE_OVER_100_MS: AtomicU32 = AtomicU32::new(0);
static LAST_MOUSE_WRITE_US: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_WRITE_GAP_MAX_US: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_SLOT_WAITS: AtomicU32 = AtomicU32::new(0);
static HID_MOUSE_SLOT_WAIT_MAX_US: AtomicU32 = AtomicU32::new(0);
static HID_GATT_TOTAL_US: AtomicU32 = AtomicU32::new(0);
static HID_GATT_WAIT_OVER_10_MS: AtomicU32 = AtomicU32::new(0);
static HID_GATT_WAIT_STREAK_CURRENT: AtomicU32 = AtomicU32::new(0);
static HID_GATT_WAIT_STREAK_MAX: AtomicU32 = AtomicU32::new(0);

#[inline]
fn now_ms() -> u32 {
    Instant::now().as_millis() as u32
}

#[inline]
fn now_us() -> u32 {
    Instant::now().as_micros() as u32
}

#[inline]
fn add_i32(counter: &AtomicI32, value: i16) {
    counter.fetch_add(i32::from(value), Ordering::Relaxed);
}

fn axis_totals(event: &PointingEvent) -> (i16, i16) {
    let mut x = 0i16;
    let mut y = 0i16;
    for axis in event.axes {
        match (axis.typ, axis.axis) {
            (AxisValType::Rel, Axis::X) => x = x.saturating_add(axis.value),
            (AxisValType::Rel, Axis::Y) => y = y.saturating_add(axis.value),
            _ => {}
        }
    }
    (x, y)
}

fn record_bounded_gap(last: &AtomicU32, max: &AtomicU32, now: u32) {
    let previous = last.swap(now, Ordering::Relaxed);
    if previous == 0 {
        return;
    }
    let gap = now.saturating_sub(previous);
    if gap <= MAX_RELEVANT_GAP_MS {
        max.fetch_max(gap, Ordering::Relaxed);
    }
}

fn take_log_slot(next: &AtomicU32, now: u32) -> bool {
    let deadline = next.load(Ordering::Relaxed);
    if deadline == 0 {
        let _ = next.compare_exchange(
            0,
            now.saturating_add(REPORT_INTERVAL_MS),
            Ordering::Relaxed,
            Ordering::Relaxed,
        );
        return false;
    }
    if now < deadline {
        return false;
    }
    next.compare_exchange(
        deadline,
        now.saturating_add(REPORT_INTERVAL_MS),
        Ordering::Relaxed,
        Ordering::Relaxed,
    )
    .is_ok()
}

/// Record one successful PMW3610 read on a K:04 half.
pub fn record_pmw_read(dx: i16, dy: i16, elapsed_us: u32, gpio_low: bool) {
    let now = now_ms();
    PMW_READS.fetch_add(1, Ordering::Relaxed);
    add_i32(&PMW_DX, dx);
    add_i32(&PMW_DY, dy);
    PMW_READ_MAX_US.fetch_max(elapsed_us, Ordering::Relaxed);
    record_bounded_gap(&LAST_PMW_READ_MS, &PMW_MAX_GAP_MS, now);
    if gpio_low {
        PMW_GPIO_LOW_READS.fetch_add(1, Ordering::Relaxed);
        if dx == 0 && dy == 0 {
            PMW_GPIO_LOW_ZERO.fetch_add(1, Ordering::Relaxed);
        }
    }
    maybe_log_right(now);
}

#[allow(clippy::too_many_arguments)]
fn record_raw_axis(
    value: i16,
    now: u32,
    positive: &AtomicU32,
    negative: &AtomicU32,
    zero: &AtomicU32,
    positive_sum: &AtomicU32,
    negative_abs_sum: &AtomicU32,
    abs_max: &AtomicU32,
    near_saturation: &AtomicU32,
    flips: &AtomicU32,
    last_sign: &AtomicI32,
    last_sign_ms: &AtomicU32,
) -> bool {
    if value == 0 {
        zero.fetch_add(1, Ordering::Relaxed);
        return false;
    }

    let magnitude = u32::from(value.unsigned_abs());
    abs_max.fetch_max(magnitude, Ordering::Relaxed);
    if value > 0 {
        positive.fetch_add(1, Ordering::Relaxed);
        positive_sum.fetch_add(magnitude, Ordering::Relaxed);
    } else {
        negative.fetch_add(1, Ordering::Relaxed);
        negative_abs_sum.fetch_add(magnitude, Ordering::Relaxed);
    }
    if magnitude >= u32::from(RAW_NEAR_SATURATION_ABS) {
        near_saturation.fetch_add(1, Ordering::Relaxed);
    }

    if magnitude < u32::from(RAW_SIGN_MIN_ABS) {
        return false;
    }

    let sign = if value < 0 { -1 } else { 1 };
    let previous_sign = last_sign.swap(sign, Ordering::Relaxed);
    let previous_ms = last_sign_ms.swap(now, Ordering::Relaxed);
    let changed =
        previous_sign != 0 && previous_sign != sign && now.saturating_sub(previous_ms) <= RAW_SIGN_CONTINUITY_MS;
    if changed {
        flips.fetch_add(1, Ordering::Relaxed);
    }
    changed
}

/// Record the exact PMW3610 motion-burst delta bytes and decoded signs.
///
/// The stage-10 test uses this to distinguish optical direction reversal from
/// a lost sign bit in the shared DELTA_XY_H byte. Only strong deltas contribute
/// to sign-flip counters, so small cross-axis noise does not dominate them.
pub fn record_pmw_raw(motion: u8, x_l: u8, y_l: u8, xy_h: u8, dx: i16, dy: i16) {
    let now = now_ms();
    PMW_RAW_SAMPLES.fetch_add(1, Ordering::Relaxed);
    if motion & 0x80 != 0 {
        PMW_RAW_MOTION_SET.fetch_add(1, Ordering::Relaxed);
    }
    PMW_RAW_XYH_OR.fetch_or(u32::from(xy_h), Ordering::Relaxed);
    PMW_RAW_XYH_AND.fetch_and(u32::from(xy_h), Ordering::Relaxed);
    PMW_RAW_XYH_LAST.store(u32::from(xy_h), Ordering::Relaxed);

    let x_flipped = record_raw_axis(
        dx,
        now,
        &PMW_RAW_X_POS,
        &PMW_RAW_X_NEG,
        &PMW_RAW_X_ZERO,
        &PMW_RAW_X_POS_SUM,
        &PMW_RAW_X_NEG_ABS_SUM,
        &PMW_RAW_X_ABS_MAX,
        &PMW_RAW_X_NEAR_SAT,
        &PMW_RAW_X_FLIPS,
        &PMW_RAW_LAST_X_SIGN,
        &PMW_RAW_LAST_X_SIGN_MS,
    );
    let y_flipped = record_raw_axis(
        dy,
        now,
        &PMW_RAW_Y_POS,
        &PMW_RAW_Y_NEG,
        &PMW_RAW_Y_ZERO,
        &PMW_RAW_Y_POS_SUM,
        &PMW_RAW_Y_NEG_ABS_SUM,
        &PMW_RAW_Y_ABS_MAX,
        &PMW_RAW_Y_NEAR_SAT,
        &PMW_RAW_Y_FLIPS,
        &PMW_RAW_LAST_Y_SIGN,
        &PMW_RAW_LAST_Y_SIGN_MS,
    );

    if x_flipped || y_flipped {
        PMW_RAW_LAST_FLIP_MS.store(now, Ordering::Relaxed);
        PMW_RAW_LAST_FLIP_AXIS.store(u32::from(x_flipped) | (u32::from(y_flipped) << 1), Ordering::Relaxed);
        PMW_RAW_LAST_FLIP_MOTION.store(u32::from(motion), Ordering::Relaxed);
        PMW_RAW_LAST_FLIP_X_L.store(u32::from(x_l), Ordering::Relaxed);
        PMW_RAW_LAST_FLIP_Y_L.store(u32::from(y_l), Ordering::Relaxed);
        PMW_RAW_LAST_FLIP_XY_H.store(u32::from(xy_h), Ordering::Relaxed);
        PMW_RAW_LAST_FLIP_DX.store(i32::from(dx), Ordering::Relaxed);
        PMW_RAW_LAST_FLIP_DY.store(i32::from(dy), Ordering::Relaxed);
    }
}

fn record_axis_sign(value: i16, positive: &AtomicU32, negative: &AtomicU32) {
    if value > 0 {
        positive.fetch_add(1, Ordering::Relaxed);
    } else if value < 0 {
        negative.fetch_add(1, Ordering::Relaxed);
    }
}

/// Record the native PMW deltas and the exact values sent into the K:04
/// pointing pipeline after the software X/Y swap.
pub fn record_pmw_axis_transform(pre_x: i16, pre_y: i16, post_x: i16, post_y: i16) {
    PMW_AXIS_SAMPLES.fetch_add(1, Ordering::Relaxed);
    record_axis_sign(pre_x, &PMW_AXIS_PRE_X_POS, &PMW_AXIS_PRE_X_NEG);
    record_axis_sign(pre_y, &PMW_AXIS_PRE_Y_POS, &PMW_AXIS_PRE_Y_NEG);
    record_axis_sign(post_x, &PMW_AXIS_POST_X_POS, &PMW_AXIS_POST_X_NEG);
    record_axis_sign(post_y, &PMW_AXIS_POST_Y_POS, &PMW_AXIS_POST_Y_NEG);
    add_i32(&PMW_AXIS_PRE_X_SUM, pre_x);
    add_i32(&PMW_AXIS_PRE_Y_SUM, pre_y);
    add_i32(&PMW_AXIS_POST_X_SUM, post_x);
    add_i32(&PMW_AXIS_POST_Y_SUM, post_y);
    if post_x != pre_y || post_y != pre_x {
        PMW_AXIS_MISMATCH.fetch_add(1, Ordering::Relaxed);
    }
    PMW_AXIS_LAST_PRE_X.store(i32::from(pre_x), Ordering::Relaxed);
    PMW_AXIS_LAST_PRE_Y.store(i32::from(pre_y), Ordering::Relaxed);
    PMW_AXIS_LAST_POST_X.store(i32::from(post_x), Ordering::Relaxed);
    PMW_AXIS_LAST_POST_Y.store(i32::from(post_y), Ordering::Relaxed);
}

/// Record raw optical-quality fields from every smart-mode motion burst,
/// including bursts whose MOTION bit is clear. This makes shiny-surface
/// tracking loss visible before the driver returns a zero delta.
pub fn record_pmw_optics(motion_set: bool, squal: u8, shutter: u16, threshold: u16, smart_disabled: bool) {
    PMW_OPTICS_SAMPLES.fetch_add(1, Ordering::Relaxed);
    if motion_set {
        PMW_OPTICS_MOTION_SET.fetch_add(1, Ordering::Relaxed);
    } else {
        PMW_OPTICS_MOTION_CLEAR.fetch_add(1, Ordering::Relaxed);
    }

    let squal = u32::from(squal);
    let shutter = u32::from(shutter);
    PMW_SQUAL_SUM.fetch_add(squal, Ordering::Relaxed);
    PMW_SQUAL_MIN.fetch_min(squal, Ordering::Relaxed);
    PMW_SQUAL_MAX.fetch_max(squal, Ordering::Relaxed);
    PMW_SHUTTER_SUM.fetch_add(shutter, Ordering::Relaxed);
    PMW_SHUTTER_MIN.fetch_min(shutter, Ordering::Relaxed);
    PMW_SHUTTER_MAX.fetch_max(shutter, Ordering::Relaxed);
    match shutter.cmp(&u32::from(threshold)) {
        core::cmp::Ordering::Less => PMW_SHUTTER_BELOW.fetch_add(1, Ordering::Relaxed),
        core::cmp::Ordering::Equal => PMW_SHUTTER_EQUAL.fetch_add(1, Ordering::Relaxed),
        core::cmp::Ordering::Greater => PMW_SHUTTER_ABOVE.fetch_add(1, Ordering::Relaxed),
    };
    PMW_SMART_DISABLED.store(smart_disabled, Ordering::Relaxed);
}

/// Record a successful write that changed the PMW3610 smart-mode register.
pub fn record_pmw_smart_transition(disabled: bool, squal: u8, shutter: u16) {
    if disabled {
        PMW_SMART_DISABLES.fetch_add(1, Ordering::Relaxed);
    } else {
        PMW_SMART_ENABLES.fetch_add(1, Ordering::Relaxed);
    }
    PMW_SMART_DISABLED.store(disabled, Ordering::Relaxed);
    PMW_SMART_LAST_SQUAL.store(u32::from(squal), Ordering::Relaxed);
    PMW_SMART_LAST_SHUTTER.store(u32::from(shutter), Ordering::Relaxed);
}

/// Record how long the trackball task waited for MOTION/deadline selection.
/// `pending_at_arm` distinguishes an already-low MOTION pin from a sensor that
/// had not asserted motion when the wait began.
pub fn record_motion_wake(wait_us: u32, pending_at_arm: bool) {
    MOTION_WAKE_COUNT.fetch_add(1, Ordering::Relaxed);
    MOTION_WAIT_MAX_US.fetch_max(wait_us, Ordering::Relaxed);
    if pending_at_arm {
        MOTION_PENDING_WAIT_MAX_US.fetch_max(wait_us, Ordering::Relaxed);
    }
}

/// Record a timer-forced sensor read while MOTION was not asserted. A hit
/// means polling recovered non-zero movement that the GPIO path did not wake.
pub fn record_pmw_fallback(dx: i16, dy: i16) {
    PMW_FALLBACK_READS.fetch_add(1, Ordering::Relaxed);
    if dx != 0 || dy != 0 {
        PMW_FALLBACK_HITS.fetch_add(1, Ordering::Relaxed);
    }
}

/// Record lateness of an independent timer task used as an executor heartbeat.
pub fn record_scheduler_tick(late_us: u32) {
    SCHEDULER_TICKS.fetch_add(1, Ordering::Relaxed);
    SCHEDULER_LATE_MAX_US.fetch_max(late_us, Ordering::Relaxed);
    if late_us >= 5_000 {
        SCHEDULER_LATE_OVER_5_MS.fetch_add(1, Ordering::Relaxed);
    }
    maybe_log_right(now_ms());
}

/// Record one PMW3610 read failure.
pub fn record_pmw_error() {
    PMW_ERRORS.fetch_add(1, Ordering::Relaxed);
    maybe_log_right(now_ms());
}

/// Record one PointingEvent published by the K:04 trackball task.
pub fn record_pmw_publish() {
    let now = now_ms();
    PMW_PUBLISHES.fetch_add(1, Ordering::Relaxed);
    record_bounded_gap(&LAST_PMW_PUBLISH_MS, &PMW_PUBLISH_MAX_GAP_MS, now);
    maybe_log_right(now);
}

/// Record motion that could not fit in the widened trackball accumulator.
/// These counters are exact discarded relative counts, not HID chunking.
pub fn record_pmw_accum_drop(dropped_x: u32, dropped_y: u32) {
    PMW_ACCUM_DROPPED_X.fetch_add(dropped_x, Ordering::Relaxed);
    PMW_ACCUM_DROPPED_Y.fetch_add(dropped_y, Ordering::Relaxed);
}

/// Record completion of a peripheral-to-central BLE split notification.
pub(crate) fn record_split_tx(message: &SplitMessage, elapsed_us: u32, ok: bool) {
    let now = now_ms();
    if let SplitMessage::Pointing(event) = message {
        let (x, y) = axis_totals(event);
        SPLIT_TX_POINTING.fetch_add(1, Ordering::Relaxed);
        add_i32(&SPLIT_TX_DX, x);
        add_i32(&SPLIT_TX_DY, y);
        SPLIT_TX_MAX_US.fetch_max(elapsed_us, Ordering::Relaxed);
        SPLIT_TX_TOTAL_US.fetch_add(elapsed_us, Ordering::Relaxed);
        record_bounded_gap(&LAST_SPLIT_TX_MS, &SPLIT_TX_MAX_GAP_MS, now);
        if elapsed_us >= WAITING_SPLIT_US {
            SPLIT_TX_WAIT_OVER_20_MS.fetch_add(1, Ordering::Relaxed);
            let streak = SPLIT_TX_WAIT_STREAK_CURRENT
                .fetch_add(1, Ordering::Relaxed)
                .saturating_add(1);
            SPLIT_TX_WAIT_STREAK_MAX.fetch_max(streak, Ordering::Relaxed);
        } else {
            SPLIT_TX_WAIT_STREAK_CURRENT.store(0, Ordering::Relaxed);
        }
    }
    if !ok {
        SPLIT_TX_ERRORS.fetch_add(1, Ordering::Relaxed);
    }
    maybe_log_right(now);
}

/// Record a pointing notification received by the split central.
pub(crate) fn record_split_rx(event: &PointingEvent) {
    let now = now_ms();
    let (x, y) = axis_totals(event);
    SPLIT_RX_POINTING.fetch_add(1, Ordering::Relaxed);
    add_i32(&SPLIT_RX_DX, x);
    add_i32(&SPLIT_RX_DY, y);
    record_bounded_gap(&LAST_SPLIT_RX_MS, &SPLIT_RX_MAX_GAP_MS, now);
    maybe_log_left(now);
}

/// Record a report's attempt to enter the BLE HID queue.
pub(crate) fn record_hid_enqueue(
    is_mouse: bool,
    queue_len: usize,
    full_retries: u32,
    elapsed_us: u32,
    delivered: bool,
) {
    if delivered {
        HID_ENQUEUE_ALL.fetch_add(1, Ordering::Relaxed);
        if is_mouse {
            HID_ENQUEUE_MOUSE.fetch_add(1, Ordering::Relaxed);
        }
    } else {
        HID_ENQUEUE_ABORTED.fetch_add(1, Ordering::Relaxed);
    }
    HID_ENQUEUE_FULL.fetch_add(full_retries, Ordering::Relaxed);
    HID_QUEUE_HIGH_WATER.fetch_max(queue_len as u32, Ordering::Relaxed);
    HID_ENQUEUE_MAX_WAIT_US.fetch_max(elapsed_us, Ordering::Relaxed);
    maybe_log_left(now_ms());
}

/// Record completion of a BLE HID GATT notification.
pub(crate) fn record_hid_write(
    report: &Report,
    elapsed_us: u32,
    ok: bool,
    queue_len: usize,
    motion_age_us: u32,
    source_reports: u32,
) {
    HID_WRITE_ALL.fetch_add(1, Ordering::Relaxed);
    if let Report::MouseReport(mouse) = report {
        HID_WRITE_MOUSE.fetch_add(1, Ordering::Relaxed);
        add_i32(&HID_WRITE_DX, i16::from(mouse.x));
        add_i32(&HID_WRITE_DY, i16::from(mouse.y));
        HID_MOUSE_SOURCE_REPORTS.fetch_add(source_reports, Ordering::Relaxed);
        HID_MOUSE_AGE_SAMPLES.fetch_add(1, Ordering::Relaxed);
        HID_MOUSE_AGE_TOTAL_US.fetch_add(motion_age_us, Ordering::Relaxed);
        HID_MOUSE_AGE_MAX_US.fetch_max(motion_age_us, Ordering::Relaxed);
        if motion_age_us >= 15_000 {
            HID_MOUSE_AGE_OVER_15_MS.fetch_add(1, Ordering::Relaxed);
        }
        if motion_age_us >= 30_000 {
            HID_MOUSE_AGE_OVER_30_MS.fetch_add(1, Ordering::Relaxed);
        }
        if motion_age_us >= 100_000 {
            HID_MOUSE_AGE_OVER_100_MS.fetch_add(1, Ordering::Relaxed);
        }

        let now = now_us();
        let previous = LAST_MOUSE_WRITE_US.swap(now, Ordering::Relaxed);
        if previous != 0 {
            let gap = now.wrapping_sub(previous);
            if gap <= MAX_RELEVANT_GAP_MS.saturating_mul(1_000) {
                HID_MOUSE_WRITE_GAP_MAX_US.fetch_max(gap, Ordering::Relaxed);
            }
        }
    }
    if !ok {
        HID_WRITE_ERRORS.fetch_add(1, Ordering::Relaxed);
    }
    if elapsed_us >= SLOW_GATT_US {
        HID_WRITE_SLOW.fetch_add(1, Ordering::Relaxed);
    }
    HID_WRITE_MAX_US.fetch_max(elapsed_us, Ordering::Relaxed);
    HID_GATT_TOTAL_US.fetch_add(elapsed_us, Ordering::Relaxed);
    if elapsed_us >= WAITING_GATT_US {
        HID_GATT_WAIT_OVER_10_MS.fetch_add(1, Ordering::Relaxed);
        let streak = HID_GATT_WAIT_STREAK_CURRENT
            .fetch_add(1, Ordering::Relaxed)
            .saturating_add(1);
        HID_GATT_WAIT_STREAK_MAX.fetch_max(streak, Ordering::Relaxed);
    } else {
        HID_GATT_WAIT_STREAK_CURRENT.store(0, Ordering::Relaxed);
    }
    HID_QUEUE_HIGH_WATER.fetch_max(queue_len as u32, Ordering::Relaxed);
    maybe_log_left(now_ms());
}

/// Record how many adjacent mouse samples were folded into one BLE report and
/// whether either pointer axis exceeded the signed 8-bit HID range. The
/// residual maxima make directional backlog visible without per-report RTT.
pub(crate) fn record_mouse_coalesce(
    merged_reports: u32,
    has_residual: bool,
    input_x: i32,
    input_y: i32,
    residual_x: i32,
    residual_y: i32,
) {
    HID_MOUSE_MERGED.fetch_add(merged_reports, Ordering::Relaxed);
    if has_residual {
        HID_MOUSE_SPLIT_CHUNKS.fetch_add(1, Ordering::Relaxed);
    }
    if input_x > i32::from(i8::MAX) {
        HID_MOUSE_CLIP_X_POS.fetch_add(1, Ordering::Relaxed);
    } else if input_x < i32::from(i8::MIN) {
        HID_MOUSE_CLIP_X_NEG.fetch_add(1, Ordering::Relaxed);
    }
    if input_y > i32::from(i8::MAX) {
        HID_MOUSE_CLIP_Y_POS.fetch_add(1, Ordering::Relaxed);
    } else if input_y < i32::from(i8::MIN) {
        HID_MOUSE_CLIP_Y_NEG.fetch_add(1, Ordering::Relaxed);
    }
    HID_MOUSE_RESIDUAL_X_MAX.fetch_max(residual_x.unsigned_abs(), Ordering::Relaxed);
    HID_MOUSE_RESIDUAL_Y_MAX.fetch_max(residual_y.unsigned_abs(), Ordering::Relaxed);
}

/// Record the intentional pacing delay in the 15 ms control build. Baseline
/// diagnostics leave both values at zero.
pub(crate) fn record_mouse_slot_wait(elapsed_us: u32) {
    if elapsed_us == 0 {
        return;
    }
    HID_MOUSE_SLOT_WAITS.fetch_add(1, Ordering::Relaxed);
    HID_MOUSE_SLOT_WAIT_MAX_US.fetch_max(elapsed_us, Ordering::Relaxed);
}

fn maybe_log_right(now: u32) {
    if !take_log_slot(&NEXT_RIGHT_LOG_MS, now) {
        return;
    }
    let optics_samples = PMW_OPTICS_SAMPLES.swap(0, Ordering::Relaxed);
    let squal_min = PMW_SQUAL_MIN.swap(u32::MAX, Ordering::Relaxed);
    let shutter_min = PMW_SHUTTER_MIN.swap(u32::MAX, Ordering::Relaxed);
    let squal_avg = if optics_samples == 0 {
        0
    } else {
        PMW_SQUAL_SUM.swap(0, Ordering::Relaxed) / optics_samples
    };
    let shutter_avg = if optics_samples == 0 {
        0
    } else {
        PMW_SHUTTER_SUM.swap(0, Ordering::Relaxed) / optics_samples
    };
    if optics_samples == 0 {
        PMW_SQUAL_SUM.store(0, Ordering::Relaxed);
        PMW_SHUTTER_SUM.store(0, Ordering::Relaxed);
    }
    let raw_samples = PMW_RAW_SAMPLES.swap(0, Ordering::Relaxed);
    let raw_xyh_and = PMW_RAW_XYH_AND.swap(0xff, Ordering::Relaxed);
    defmt::info!(
        "[PMW_NATIVE_V14] samples={} motion1={} xp={} xn={} x0={} xpsum={} xnabs={} xmax={} xsat={} xflip={} yp={} yn={} y0={} ypsum={} ynabs={} ymax={} ysat={} yflip={} xyh_or={:#04x} xyh_and={:#04x} xyh_last={:#04x} flip_t_ms={} flip_axis={} flip_motion={:#04x} flip_xl={:#04x} flip_yl={:#04x} flip_xyh={:#04x} flip_dx={} flip_dy={}",
        raw_samples,
        PMW_RAW_MOTION_SET.swap(0, Ordering::Relaxed),
        PMW_RAW_X_POS.swap(0, Ordering::Relaxed),
        PMW_RAW_X_NEG.swap(0, Ordering::Relaxed),
        PMW_RAW_X_ZERO.swap(0, Ordering::Relaxed),
        PMW_RAW_X_POS_SUM.swap(0, Ordering::Relaxed),
        PMW_RAW_X_NEG_ABS_SUM.swap(0, Ordering::Relaxed),
        PMW_RAW_X_ABS_MAX.swap(0, Ordering::Relaxed),
        PMW_RAW_X_NEAR_SAT.swap(0, Ordering::Relaxed),
        PMW_RAW_X_FLIPS.swap(0, Ordering::Relaxed),
        PMW_RAW_Y_POS.swap(0, Ordering::Relaxed),
        PMW_RAW_Y_NEG.swap(0, Ordering::Relaxed),
        PMW_RAW_Y_ZERO.swap(0, Ordering::Relaxed),
        PMW_RAW_Y_POS_SUM.swap(0, Ordering::Relaxed),
        PMW_RAW_Y_NEG_ABS_SUM.swap(0, Ordering::Relaxed),
        PMW_RAW_Y_ABS_MAX.swap(0, Ordering::Relaxed),
        PMW_RAW_Y_NEAR_SAT.swap(0, Ordering::Relaxed),
        PMW_RAW_Y_FLIPS.swap(0, Ordering::Relaxed),
        PMW_RAW_XYH_OR.swap(0, Ordering::Relaxed),
        if raw_samples == 0 { 0 } else { raw_xyh_and },
        PMW_RAW_XYH_LAST.load(Ordering::Relaxed),
        PMW_RAW_LAST_FLIP_MS.swap(0, Ordering::Relaxed),
        PMW_RAW_LAST_FLIP_AXIS.load(Ordering::Relaxed),
        PMW_RAW_LAST_FLIP_MOTION.load(Ordering::Relaxed),
        PMW_RAW_LAST_FLIP_X_L.load(Ordering::Relaxed),
        PMW_RAW_LAST_FLIP_Y_L.load(Ordering::Relaxed),
        PMW_RAW_LAST_FLIP_XY_H.load(Ordering::Relaxed),
        PMW_RAW_LAST_FLIP_DX.load(Ordering::Relaxed),
        PMW_RAW_LAST_FLIP_DY.load(Ordering::Relaxed),
    );
    let axis_samples = PMW_AXIS_SAMPLES.swap(0, Ordering::Relaxed);
    let pre_x_pos = PMW_AXIS_PRE_X_POS.swap(0, Ordering::Relaxed);
    let pre_x_neg = PMW_AXIS_PRE_X_NEG.swap(0, Ordering::Relaxed);
    let pre_y_pos = PMW_AXIS_PRE_Y_POS.swap(0, Ordering::Relaxed);
    let pre_y_neg = PMW_AXIS_PRE_Y_NEG.swap(0, Ordering::Relaxed);
    let post_x_pos = PMW_AXIS_POST_X_POS.swap(0, Ordering::Relaxed);
    let post_x_neg = PMW_AXIS_POST_X_NEG.swap(0, Ordering::Relaxed);
    let post_y_pos = PMW_AXIS_POST_Y_POS.swap(0, Ordering::Relaxed);
    let post_y_neg = PMW_AXIS_POST_Y_NEG.swap(0, Ordering::Relaxed);
    defmt::info!(
        "[PMW_AXES_V14] samples={} pre_xp={} pre_xn={} pre_x0={} pre_yp={} pre_yn={} pre_y0={} pre_xsum={} pre_ysum={} post_xp={} post_xn={} post_x0={} post_yp={} post_yn={} post_y0={} post_xsum={} post_ysum={} mismatch={} last_pre_x={} last_pre_y={} last_post_x={} last_post_y={}",
        axis_samples,
        pre_x_pos,
        pre_x_neg,
        axis_samples.saturating_sub(pre_x_pos.saturating_add(pre_x_neg)),
        pre_y_pos,
        pre_y_neg,
        axis_samples.saturating_sub(pre_y_pos.saturating_add(pre_y_neg)),
        PMW_AXIS_PRE_X_SUM.swap(0, Ordering::Relaxed),
        PMW_AXIS_PRE_Y_SUM.swap(0, Ordering::Relaxed),
        post_x_pos,
        post_x_neg,
        axis_samples.saturating_sub(post_x_pos.saturating_add(post_x_neg)),
        post_y_pos,
        post_y_neg,
        axis_samples.saturating_sub(post_y_pos.saturating_add(post_y_neg)),
        PMW_AXIS_POST_X_SUM.swap(0, Ordering::Relaxed),
        PMW_AXIS_POST_Y_SUM.swap(0, Ordering::Relaxed),
        PMW_AXIS_MISMATCH.swap(0, Ordering::Relaxed),
        PMW_AXIS_LAST_PRE_X.load(Ordering::Relaxed),
        PMW_AXIS_LAST_PRE_Y.load(Ordering::Relaxed),
        PMW_AXIS_LAST_POST_X.load(Ordering::Relaxed),
        PMW_AXIS_LAST_POST_Y.load(Ordering::Relaxed),
    );
    defmt::info!(
        "[PMW_OPTICS_V10] samples={} motion1={} motion0={} gpio_low={} gpio_low_zero={} squal_min={} squal_avg={} squal_max={} shutter_min={} shutter_avg={} shutter_max={} shutter_lt45={} shutter_eq45={} shutter_gt45={} smart_en={} smart_dis={} smart_disabled={} smart_last_squal={} smart_last_shutter={}",
        optics_samples,
        PMW_OPTICS_MOTION_SET.swap(0, Ordering::Relaxed),
        PMW_OPTICS_MOTION_CLEAR.swap(0, Ordering::Relaxed),
        PMW_GPIO_LOW_READS.swap(0, Ordering::Relaxed),
        PMW_GPIO_LOW_ZERO.swap(0, Ordering::Relaxed),
        if squal_min == u32::MAX { 0 } else { squal_min },
        squal_avg,
        PMW_SQUAL_MAX.swap(0, Ordering::Relaxed),
        if shutter_min == u32::MAX { 0 } else { shutter_min },
        shutter_avg,
        PMW_SHUTTER_MAX.swap(0, Ordering::Relaxed),
        PMW_SHUTTER_BELOW.swap(0, Ordering::Relaxed),
        PMW_SHUTTER_EQUAL.swap(0, Ordering::Relaxed),
        PMW_SHUTTER_ABOVE.swap(0, Ordering::Relaxed),
        PMW_SMART_ENABLES.swap(0, Ordering::Relaxed),
        PMW_SMART_DISABLES.swap(0, Ordering::Relaxed),
        PMW_SMART_DISABLED.load(Ordering::Relaxed),
        PMW_SMART_LAST_SQUAL.load(Ordering::Relaxed),
        PMW_SMART_LAST_SHUTTER.load(Ordering::Relaxed),
    );
    defmt::info!(
        "[DIAG_R] pmw_read={} pmw_pub={} pmw_err={} pmw_dx={} pmw_dy={} accum_drop_x={} accum_drop_y={} pmw_read_max_us={} pmw_gap_max_ms={} pmw_pub_gap_max_ms={} motion_wake={} motion_wait_max_us={} motion_pending_wait_max_us={} fallback_read={} fallback_hit={} sched_ticks={} sched_late_max_us={} sched_late_5ms={} split_tx={} split_dx={} split_dy={} split_err={} split_max_us={} split_gap_max_ms={} split_total_us={} split_wait20={} split_streak={} split_streak_max={}",
        PMW_READS.swap(0, Ordering::Relaxed),
        PMW_PUBLISHES.swap(0, Ordering::Relaxed),
        PMW_ERRORS.swap(0, Ordering::Relaxed),
        PMW_DX.swap(0, Ordering::Relaxed),
        PMW_DY.swap(0, Ordering::Relaxed),
        PMW_ACCUM_DROPPED_X.swap(0, Ordering::Relaxed),
        PMW_ACCUM_DROPPED_Y.swap(0, Ordering::Relaxed),
        PMW_READ_MAX_US.swap(0, Ordering::Relaxed),
        PMW_MAX_GAP_MS.swap(0, Ordering::Relaxed),
        PMW_PUBLISH_MAX_GAP_MS.swap(0, Ordering::Relaxed),
        MOTION_WAKE_COUNT.swap(0, Ordering::Relaxed),
        MOTION_WAIT_MAX_US.swap(0, Ordering::Relaxed),
        MOTION_PENDING_WAIT_MAX_US.swap(0, Ordering::Relaxed),
        PMW_FALLBACK_READS.swap(0, Ordering::Relaxed),
        PMW_FALLBACK_HITS.swap(0, Ordering::Relaxed),
        SCHEDULER_TICKS.swap(0, Ordering::Relaxed),
        SCHEDULER_LATE_MAX_US.swap(0, Ordering::Relaxed),
        SCHEDULER_LATE_OVER_5_MS.swap(0, Ordering::Relaxed),
        SPLIT_TX_POINTING.swap(0, Ordering::Relaxed),
        SPLIT_TX_DX.swap(0, Ordering::Relaxed),
        SPLIT_TX_DY.swap(0, Ordering::Relaxed),
        SPLIT_TX_ERRORS.swap(0, Ordering::Relaxed),
        SPLIT_TX_MAX_US.swap(0, Ordering::Relaxed),
        SPLIT_TX_MAX_GAP_MS.swap(0, Ordering::Relaxed),
        SPLIT_TX_TOTAL_US.swap(0, Ordering::Relaxed),
        SPLIT_TX_WAIT_OVER_20_MS.swap(0, Ordering::Relaxed),
        SPLIT_TX_WAIT_STREAK_CURRENT.load(Ordering::Relaxed),
        SPLIT_TX_WAIT_STREAK_MAX.swap(0, Ordering::Relaxed),
    );
}

fn maybe_log_left(now: u32) {
    if !take_log_slot(&NEXT_LEFT_LOG_MS, now) {
        return;
    }
    let mouse_age_samples = HID_MOUSE_AGE_SAMPLES.swap(0, Ordering::Relaxed);
    let mouse_age_total_us = HID_MOUSE_AGE_TOTAL_US.swap(0, Ordering::Relaxed);
    let mouse_age_avg_us = if mouse_age_samples == 0 {
        0
    } else {
        mouse_age_total_us / mouse_age_samples
    };
    defmt::info!(
        "[DIAG_L] split_rx={} split_dx={} split_dy={} split_gap_max_ms={} hid_enq={} mouse_enq={} q_now={} q_hi={} q_full={} q_abort={} q_wait_max_us={} hid_write={} mouse_write={} mouse_merge={} mouse_split={} clip_xp={} clip_xn={} clip_yp={} clip_yn={} res_x_max={} res_y_max={} mouse_src={} mouse_dx={} mouse_dy={} mouse_age_avg_us={} mouse_age_max_us={} age15={} age30={} age100={} mouse_gap_max_us={} slot_wait={} slot_wait_max_us={} gatt_max_us={} gatt_slow={} gatt_wait10={} gatt_streak={} gatt_streak_max={} gatt_total_us={} gatt_err={}",
        SPLIT_RX_POINTING.swap(0, Ordering::Relaxed),
        SPLIT_RX_DX.swap(0, Ordering::Relaxed),
        SPLIT_RX_DY.swap(0, Ordering::Relaxed),
        SPLIT_RX_MAX_GAP_MS.swap(0, Ordering::Relaxed),
        HID_ENQUEUE_ALL.swap(0, Ordering::Relaxed),
        HID_ENQUEUE_MOUSE.swap(0, Ordering::Relaxed),
        crate::channel::BLE_REPORT_CHANNEL.len() as u32,
        HID_QUEUE_HIGH_WATER.swap(0, Ordering::Relaxed),
        HID_ENQUEUE_FULL.swap(0, Ordering::Relaxed),
        HID_ENQUEUE_ABORTED.swap(0, Ordering::Relaxed),
        HID_ENQUEUE_MAX_WAIT_US.swap(0, Ordering::Relaxed),
        HID_WRITE_ALL.swap(0, Ordering::Relaxed),
        HID_WRITE_MOUSE.swap(0, Ordering::Relaxed),
        HID_MOUSE_MERGED.swap(0, Ordering::Relaxed),
        HID_MOUSE_SPLIT_CHUNKS.swap(0, Ordering::Relaxed),
        HID_MOUSE_CLIP_X_POS.swap(0, Ordering::Relaxed),
        HID_MOUSE_CLIP_X_NEG.swap(0, Ordering::Relaxed),
        HID_MOUSE_CLIP_Y_POS.swap(0, Ordering::Relaxed),
        HID_MOUSE_CLIP_Y_NEG.swap(0, Ordering::Relaxed),
        HID_MOUSE_RESIDUAL_X_MAX.swap(0, Ordering::Relaxed),
        HID_MOUSE_RESIDUAL_Y_MAX.swap(0, Ordering::Relaxed),
        HID_MOUSE_SOURCE_REPORTS.swap(0, Ordering::Relaxed),
        HID_WRITE_DX.swap(0, Ordering::Relaxed),
        HID_WRITE_DY.swap(0, Ordering::Relaxed),
        mouse_age_avg_us,
        HID_MOUSE_AGE_MAX_US.swap(0, Ordering::Relaxed),
        HID_MOUSE_AGE_OVER_15_MS.swap(0, Ordering::Relaxed),
        HID_MOUSE_AGE_OVER_30_MS.swap(0, Ordering::Relaxed),
        HID_MOUSE_AGE_OVER_100_MS.swap(0, Ordering::Relaxed),
        HID_MOUSE_WRITE_GAP_MAX_US.swap(0, Ordering::Relaxed),
        HID_MOUSE_SLOT_WAITS.swap(0, Ordering::Relaxed),
        HID_MOUSE_SLOT_WAIT_MAX_US.swap(0, Ordering::Relaxed),
        HID_WRITE_MAX_US.swap(0, Ordering::Relaxed),
        HID_WRITE_SLOW.swap(0, Ordering::Relaxed),
        HID_GATT_WAIT_OVER_10_MS.swap(0, Ordering::Relaxed),
        HID_GATT_WAIT_STREAK_CURRENT.load(Ordering::Relaxed),
        HID_GATT_WAIT_STREAK_MAX.swap(0, Ordering::Relaxed),
        HID_GATT_TOTAL_US.swap(0, Ordering::Relaxed),
        HID_WRITE_ERRORS.swap(0, Ordering::Relaxed),
    );
}

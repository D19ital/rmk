pub(crate) const LOCAL_REPORT_INTERVAL_US: u64 = 4_000;
pub(crate) const SPLIT_REPORT_INTERVAL_US: u64 = 15_000;
pub(crate) const SPLIT_POINTING_CONN_INTERVAL_US: u64 = 7_500;
pub(crate) const LEGACY_TRANSPORT_REPORT_INTERVAL_US: u64 = 8_000;

// A split producer must never be faster than the BLE connection can drain one
// notification. This guard prevents the v19 persistent-latency regression.
const _: () = assert!(SPLIT_REPORT_INTERVAL_US >= SPLIT_POINTING_CONN_INTERVAL_US);

/// Take one report-sized portion from an accumulated relative-motion axis.
///
/// The central trackball still feeds the HID path in native i8-sized portions.
/// The split peripheral can use the full PointingEvent i16 range so reducing
/// its radio report rate does not discard or distort fast motion.
pub(crate) fn take_report_axis(is_central: bool, accumulated: i32) -> i16 {
    if cfg!(any(feature = "production_v22", feature = "pmw_axes_600_diag")) && !is_central {
        accumulated.clamp(i16::MIN as i32, i16::MAX as i32) as i16
    } else {
        accumulated.clamp(i8::MIN as i32, i8::MAX as i32) as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn production_split_interval_is_15ms_and_two_link_intervals() {
        assert_eq!(LOCAL_REPORT_INTERVAL_US, 4_000);
        assert_eq!(SPLIT_REPORT_INTERVAL_US, 15_000);
        assert!(SPLIT_REPORT_INTERVAL_US >= SPLIT_POINTING_CONN_INTERVAL_US);
        assert_eq!(SPLIT_REPORT_INTERVAL_US, SPLIT_POINTING_CONN_INTERVAL_US * 2);
    }

    #[test]
    fn central_keeps_native_hid_sized_chunks() {
        assert_eq!(take_report_axis(true, 400), i8::MAX as i16);
        assert_eq!(take_report_axis(true, -400), i8::MIN as i16);
    }

    #[test]
    #[cfg(any(feature = "production_v22", feature = "pmw_axes_600_diag"))]
    fn production_split_peripheral_preserves_full_i16_delta() {
        assert_eq!(take_report_axis(false, 32_000), 32_000);
        assert_eq!(take_report_axis(false, -32_000), -32_000);
    }

    #[test]
    #[cfg(any(feature = "production_v22", feature = "pmw_axes_600_diag"))]
    fn production_split_peripheral_saturates_only_at_i16_bounds() {
        assert_eq!(take_report_axis(false, 40_000), i16::MAX);
        assert_eq!(take_report_axis(false, -40_000), i16::MIN);
    }

    #[cfg(not(any(feature = "production_v22", feature = "pmw_axes_600_diag")))]
    #[test]
    fn qube_split_peripheral_keeps_legacy_i8_chunks() {
        assert_eq!(LEGACY_TRANSPORT_REPORT_INTERVAL_US, 8_000);
        assert_eq!(take_report_axis(false, 400), i8::MAX as i16);
        assert_eq!(take_report_axis(false, -400), i8::MIN as i16);
    }
}

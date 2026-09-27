//! Vector-preserving chunking for signed 8-bit relative HID reports.
//!
//! The standard mouse descriptor used by `usbd-hid` declares a logical range
//! of -127..=127 for every relative axis. The number of required reports is
//! selected from the dominant axis, then every component is divided across
//! that same number of reports. This keeps the pointer direction stable while
//! preserving the exact accumulated sum.

const HID_RELATIVE_LIMIT: i64 = 127;

#[cfg(feature = "mouse_usb_16bit_report")]
const USB16_XY_MIN: i64 = -(i16::MAX as i64);
#[cfg(feature = "mouse_usb_16bit_report")]
const USB16_XY_MAX: i64 = i16::MAX as i64;

fn chunks_needed(value: i32) -> u32 {
    if value == 0 {
        return 0;
    }
    let magnitude = u64::from(value.unsigned_abs());
    magnitude.div_ceil(HID_RELATIVE_LIMIT as u64).min(u64::from(u32::MAX)) as u32
}

fn take_even_chunk(value: &mut i32, chunks: u32) -> i8 {
    debug_assert!(chunks > 0);
    let denominator = i64::from(chunks);
    let source = i64::from(*value);
    let quotient = source / denominator;
    let remainder = source % denominator;
    let rounded = if remainder.unsigned_abs().saturating_mul(2) >= denominator as u64 {
        quotient + remainder.signum()
    } else {
        quotient
    };
    let chunk = rounded.clamp(-HID_RELATIVE_LIMIT, HID_RELATIVE_LIMIT) as i8;
    *value -= i32::from(chunk);
    chunk
}

pub(crate) fn take_vector_chunk(x: &mut i32, y: &mut i32, wheel: &mut i32, pan: &mut i32) -> (i8, i8, i8, i8) {
    let chunks = chunks_needed(*x)
        .max(chunks_needed(*y))
        .max(chunks_needed(*wheel))
        .max(chunks_needed(*pan))
        .max(1);
    (
        take_even_chunk(x, chunks),
        take_even_chunk(y, chunks),
        take_even_chunk(wheel, chunks),
        take_even_chunk(pan, chunks),
    )
}

/// One immutable B11 write plan. The residual is committed only after the
/// endpoint confirms success, so an error can retry byte-for-byte unchanged.
#[cfg(feature = "mouse_usb_16bit_report")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct Usb16Chunk {
    pub(crate) x: i16,
    pub(crate) y: i16,
    pub(crate) wheel: i8,
    pub(crate) pan: i8,
    pub(crate) residual: [i32; 4],
}

#[cfg(feature = "mouse_usb_16bit_report")]
fn chunks_for_range(value: i32, min: i64, max: i64) -> u32 {
    if value == 0 {
        return 0;
    }
    let limit = if value < 0 { min.unsigned_abs() } else { max as u64 };
    u64::from(value.unsigned_abs()).div_ceil(limit).min(u64::from(u32::MAX)) as u32
}

#[cfg(feature = "mouse_usb_16bit_report")]
fn even_i32(value: i32, chunks: u32, min: i64, max: i64) -> i32 {
    if chunks == 0 {
        return 0;
    }
    let value = i64::from(value);
    let chunks = i64::from(chunks);
    let quotient = value / chunks;
    let remainder = value % chunks;
    let rounded = quotient + i64::from(remainder != 0 && remainder.signum() == value.signum());
    rounded.clamp(min, max) as i32
}

/// Plan a vector-preserving USB16 chunk without changing the source state.
#[cfg(feature = "mouse_usb_16bit_report")]
pub(crate) fn plan_usb16_chunk(values: [i32; 4]) -> Usb16Chunk {
    let chunks = chunks_for_range(values[0], USB16_XY_MIN, USB16_XY_MAX)
        .max(chunks_for_range(values[1], USB16_XY_MIN, USB16_XY_MAX))
        .max(chunks_for_range(values[2], -HID_RELATIVE_LIMIT, HID_RELATIVE_LIMIT))
        .max(chunks_for_range(values[3], -HID_RELATIVE_LIMIT, HID_RELATIVE_LIMIT))
        .max(1);
    let x = even_i32(values[0], chunks, USB16_XY_MIN, USB16_XY_MAX);
    let y = even_i32(values[1], chunks, USB16_XY_MIN, USB16_XY_MAX);
    let wheel = even_i32(values[2], chunks, -HID_RELATIVE_LIMIT, HID_RELATIVE_LIMIT);
    let pan = even_i32(values[3], chunks, -HID_RELATIVE_LIMIT, HID_RELATIVE_LIMIT);
    Usb16Chunk {
        x: x as i16,
        y: y as i16,
        wheel: wheel as i8,
        pan: pan as i8,
        residual: [values[0] - x, values[1] - y, values[2] - wheel, values[3] - pan],
    }
}

#[cfg(test)]
mod tests {
    use super::take_vector_chunk;

    fn drain4(mut values: [i32; 4]) -> Vec<[i8; 4]> {
        let source = values;
        let mut chunks = Vec::new();
        while values.iter().any(|value| *value != 0) {
            let [x, y, wheel, pan] = &mut values;
            let chunk = take_vector_chunk(x, y, wheel, pan);
            let chunk = [chunk.0, chunk.1, chunk.2, chunk.3];
            assert!(chunk.iter().all(|value| (-127..=127).contains(value)));
            assert!(chunk.iter().all(|value| *value != i8::MIN));
            chunks.push(chunk);
        }
        assert_eq!(values, [0; 4]);
        for axis in 0..4 {
            assert_eq!(
                chunks.iter().map(|chunk| i32::from(chunk[axis])).sum::<i32>(),
                source[axis]
            );
        }
        chunks
    }

    fn drain(x: i32, y: i32) -> Vec<(i8, i8)> {
        drain4([x, y, 0, 0])
            .into_iter()
            .map(|chunk| (chunk[0], chunk[1]))
            .collect()
    }

    #[test]
    fn asymmetric_vector_keeps_the_same_direction() {
        assert_eq!(drain(-250, 20), vec![(-125, 10), (-125, 10)]);
    }

    #[test]
    fn all_quadrants_are_distributed_without_loss_or_sign_reversal() {
        for (x, y) in [(300, 90), (300, -90), (-300, 90), (-300, -90)] {
            let chunks = drain(x, y);
            assert_eq!(chunks.len(), 3);
            for (chunk_x, chunk_y) in chunks {
                assert!(chunk_x == 0 || i32::from(chunk_x.signum()) == x.signum());
                assert!(chunk_y == 0 || i32::from(chunk_y.signum()) == y.signum());
            }
        }
    }

    #[test]
    fn descriptor_boundaries_fit_but_minus_128_is_split() {
        assert_eq!(drain(127, -127), vec![(127, -127)]);
        assert_eq!(drain(-128, 0), vec![(-64, 0), (-64, 0)]);
        assert_eq!(drain(0, -128), vec![(0, -64), (0, -64)]);
        assert_eq!(drain(-128, 127), vec![(-64, 64), (-64, 63)]);
        assert_eq!(drain(128, -128), vec![(64, -64), (64, -64)]);
    }

    #[test]
    fn every_axis_preserves_boundary_and_large_values_without_i8_min() {
        for axis in 0..4 {
            for value in [-32_000, -255, -129, -128, -127, -1, 0, 1, 127, 128, 129, 255, 32_000] {
                let mut source = [0; 4];
                source[axis] = value;
                let chunks = drain4(source);
                let expected_len = if value == 0 {
                    0
                } else {
                    value.unsigned_abs().div_ceil(127) as usize
                };
                assert_eq!(chunks.len(), expected_len, "axis={axis} value={value}");
            }
        }
    }

    #[test]
    fn mixed_four_axis_vectors_are_lossless_and_terminate() {
        for source in [
            [-400, 400, -128, 128],
            [i16::MIN as i32, i16::MAX as i32, -511, 509],
            [-1_000_000, 999_999, -127, 127],
        ] {
            let chunks = drain4(source);
            let dominant = source.iter().map(|value| value.unsigned_abs()).max().unwrap();
            assert_eq!(chunks.len(), dominant.div_ceil(127) as usize);
        }
    }

    #[test]
    fn representative_range_never_emits_i8_min() {
        for x in -512..=512 {
            for y in [-512, -255, -129, -128, -127, -1, 0, 1, 127, 128, 129, 255, 512] {
                let _ = drain(x, y);
            }
        }
    }
}

#[cfg(all(test, feature = "mouse_usb_16bit_report"))]
mod usb16_tests {
    use super::plan_usb16_chunk;

    fn drain(source: [i32; 4]) -> Vec<[i32; 4]> {
        let mut residual = source;
        let mut reports = Vec::new();
        while residual.iter().any(|v| *v != 0) {
            let plan = plan_usb16_chunk(residual);
            let report = [
                i32::from(plan.x),
                i32::from(plan.y),
                i32::from(plan.wheel),
                i32::from(plan.pan),
            ];
            for axis in 0..4 {
                assert!(report[axis] == 0 || report[axis].signum() == source[axis].signum());
            }
            reports.push(report);
            residual = plan.residual;
        }
        for axis in 0..4 {
            assert_eq!(reports.iter().map(|r| r[axis]).sum::<i32>(), source[axis]);
        }
        reports
    }

    #[test]
    fn b11_typical_xy_uses_one_report_not_b9_b10_i8_chunks() {
        let reports = drain([3_200, -2_400, 7, -9]);
        assert_eq!(reports, vec![[3_200, -2_400, 7, -9]]);
    }

    #[test]
    fn b11_boundaries_large_residuals_and_quadrants_are_lossless() {
        for source in [
            [i16::MAX as i32, i16::MIN as i32 + 1, 127, -127],
            [i16::MIN as i32, i16::MAX as i32 + 1, -128, 128],
            [100_000, 70_001, 511, -509],
            [100_000, -70_001, -511, 509],
            [-100_000, 70_001, 511, 509],
            [-100_000, -70_001, -511, -509],
            [i32::MAX, i32::MIN, 1_000_000, -1_000_000],
        ] {
            let reports = drain(source);
            assert!(
                reports
                    .iter()
                    .all(|r| (-(i16::MAX as i32)..=i16::MAX as i32).contains(&r[0]))
            );
            assert!(
                reports
                    .iter()
                    .all(|r| (-(i16::MAX as i32)..=i16::MAX as i32).contains(&r[1]))
            );
            assert!(reports.iter().all(|r| r[2].abs() <= 127 && r[3].abs() <= 127));
        }
    }

    #[test]
    fn b11_plan_is_pure_for_exact_retry_then_commits_known_residual() {
        let source = [90_000, -45_000, 300, -301];
        let first = plan_usb16_chunk(source);
        assert_eq!(first, plan_usb16_chunk(source));
        assert_ne!(first.residual, source);
        assert_eq!(plan_usb16_chunk(first.residual), plan_usb16_chunk(first.residual));
    }
}

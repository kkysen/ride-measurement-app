//! What the app's graph and map draw that isn't an estimate: the clock times on the graph's time axis,
//! and where on a path the rider tapped.
//! Here rather than in `ride_fusion`, since it doesn't change what a replay gives, so it keeps cached replays.

/// The tick spacings tried (minutes): each a whole number of minutes, up to a few hours.
const STEPS_MIN: [i64; 12] = [1, 2, 5, 10, 15, 20, 30, 60, 120, 180, 360, 720];

/// Round clock times (unix ms) in `start_ms..=end_ms` to label the time axis with, at most `max_ticks` of them,
/// the closest together that fit. `utc_offset_ms` is the local zone's offset from UTC (at the start, from `RideClock`),
/// so ticks hours apart fall on local round hours. Empty if the span is empty or `max_ticks` is 0.
pub fn clock_ticks(start_ms: i64, end_ms: i64, utc_offset_ms: i64, max_ticks: u32) -> Vec<i64> {
    if end_ms < start_ms || max_ticks == 0 {
        return Vec::new();
    }
    let span = end_ms - start_ms;
    let step = STEPS_MIN
        .iter()
        .map(|m| m * 60_000)
        .find(|step| span / step < i64::from(max_ticks))
        .unwrap_or(STEPS_MIN[STEPS_MIN.len() - 1] * 60_000);
    // The first multiple of the step, in local time, at or after the start.
    let first = (start_ms + utc_offset_ms).div_euclid(step) * step - utc_offset_ms;
    let first = if first < start_ms {
        first + step
    } else {
        first
    };
    (0..)
        .map(|i| first + i * step)
        .take_while(|t| *t <= end_ms)
        .take(max_ticks as usize)
        .collect()
}

/// Where a path was at each of `times_ns`: interpolated between the path's two points around it, by time.
/// `path` is `(elapsed_realtime_ns, lat, lon)` in time order. `None` before the path's first point,
/// after its last, or between two points more than `max_gap_ns` apart (no telling where it went between).
pub fn positions_at(
    times_ns: &[i64],
    path: &[(i64, f64, f64)],
    max_gap_ns: i64,
) -> Vec<Option<(f64, f64)>> {
    times_ns
        .iter()
        .map(|&t| {
            let i = path.partition_point(|p| p.0 < t);
            let b = path.get(i)?;
            if b.0 == t {
                return Some((b.1, b.2));
            }
            let a = path.get(i.checked_sub(1)?)?;
            if b.0 - a.0 > max_gap_ns {
                return None;
            }
            let f = (t - a.0) as f64 / (b.0 - a.0) as f64;
            Some((a.1 + (b.1 - a.1) * f, a.2 + (b.2 - a.2) * f))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const MIN: i64 = 60_000;
    /// New York in winter: UTC-5.
    const EST: i64 = -5 * 60 * MIN;

    #[test]
    fn ticks_on_round_minutes() {
        // 6:18:30 pm to 6:31 pm: at most 5 ticks, so 5 min apart, from 6:20.
        let start = 1000 * 60 * MIN + 18 * MIN + 30_000;
        let end = 1000 * 60 * MIN + 31 * MIN;
        let ticks = clock_ticks(start, end, 0, 5);
        assert_eq!(
            ticks,
            vec![
                1000 * 60 * MIN + 20 * MIN,
                1000 * 60 * MIN + 25 * MIN,
                1000 * 60 * MIN + 30 * MIN
            ]
        );
    }

    #[test]
    fn ticks_include_ends_on_a_tick() {
        let ticks = clock_ticks(0, 4 * MIN, 0, 5);
        assert_eq!(ticks, vec![0, MIN, 2 * MIN, 3 * MIN, 4 * MIN]);
    }

    #[test]
    fn hour_ticks_are_local_hours() {
        // Six hours from local midnight with 3 ticks: 3 h apart, on local 3:00, not UTC's.
        let midnight = 20_000 * 24 * 60 * MIN - EST;
        let ticks = clock_ticks(midnight + MIN, midnight + 9 * 60 * MIN, EST, 3);
        assert_eq!(
            ticks,
            vec![
                midnight + 3 * 60 * MIN,
                midnight + 6 * 60 * MIN,
                midnight + 9 * 60 * MIN
            ]
        );
    }

    #[test]
    fn never_more_than_asked() {
        for span in [
            0,
            30_000,
            7 * MIN,
            95 * MIN,
            3 * 60 * MIN + 14 * MIN,
            30 * 60 * MIN,
        ] {
            for max in 1..8 {
                let ticks = clock_ticks(123_456, 123_456 + span, EST, max);
                assert!(ticks.len() <= max as usize, "{span} {max} {ticks:?}");
                assert!(ticks.iter().all(|t| (123_456..=123_456 + span).contains(t)));
            }
        }
        assert!(clock_ticks(10, 5, 0, 5).is_empty());
        assert!(clock_ticks(0, 10, 0, 0).is_empty());
    }

    #[test]
    fn positions_between_points() {
        let path = [(0, 40.0, -74.0), (10, 41.0, -73.0), (100, 42.0, -72.0)];
        let at = positions_at(&[-1, 0, 5, 10, 50, 101], &path, 20);
        assert_eq!(
            at,
            vec![
                None,
                Some((40.0, -74.0)),
                Some((40.5, -73.5)),
                Some((41.0, -73.0)),
                None,
                None
            ]
        );
        let (lat, lon) = positions_at(&[50], &path, 100)[0].unwrap();
        assert!(
            (lat - (41.0 + 4.0 / 9.0)).abs() < 1e-12 && (lon - (-73.0 + 4.0 / 9.0)).abs() < 1e-12
        );
        assert_eq!(positions_at(&[5], &[], 100), vec![None]);
    }
}

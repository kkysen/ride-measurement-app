//! When each line runs, from the MTA's schedule (`data/service_hours.csv`, from `python/experiments/service_hours.py`):
//! the B and W only on weekdays, not late at night; the C not late at night; the Z only at rush hours.
//! What a line guess falls back on when the feeds don't say what's running (see [`crate::trips::Trips`]).

use std::collections::HashMap;
use std::sync::OnceLock;

const TABLE: &str = include_str!("../data/service_hours.csv");

/// Each line's hours by day of the week, Monday first: bit `h` for the hour from `h`:00.
fn table() -> &'static HashMap<String, [u32; 7]> {
    static HOURS: OnceLock<HashMap<String, [u32; 7]>> = OnceLock::new();
    HOURS.get_or_init(|| {
        TABLE
            .lines()
            .skip(1)
            .filter_map(|row| {
                let mut f = row.split(',');
                let line = f.next()?.to_string();
                let mut days = [0; 7];
                for d in &mut days {
                    *d = f.next()?.parse().ok()?;
                }
                Some((line, days))
            })
            .collect()
    })
}

/// The zone the schedule's hours are in: the feed's `agency_timezone` (`data/service_zone.txt`).
const ZONE: &str = include_str!("../data/service_zone.txt");

/// The schedule's zone as a clock, for tools comparing times with the feed's (never a literal zone).
pub fn zone() -> &'static ride_session::clock::Clock {
    static CLOCK: OnceLock<ride_session::clock::Clock> = OnceLock::new();
    // GTFS requires the zone; one the database doesn't know (it never is) leaves times in UTC.
    CLOCK.get_or_init(|| ride_session::clock::Clock::named_or_fixed(ZONE.trim(), 0))
}

/// The schedule's day of the week (0 for Monday) and hour at `unix_s`, in its zone with its daylight saving rules.
pub fn schedule_time(unix_s: i64) -> (usize, u32) {
    let local = zone()
        .local_ms(unix_s.saturating_mul(1000))
        .div_euclid(1000);
    let days = local.div_euclid(86_400);
    // 1970-01-01 was a Thursday: day 3 counting from Monday.
    let weekday = (days + 3).rem_euclid(7) as usize;
    let hour = (local.rem_euclid(86_400) / 3600) as u32;
    (weekday, hour)
}

/// Whether `line` runs at `unix_s` by the schedule; true for a line it doesn't list.
pub fn runs(line: &str, unix_s: i64) -> bool {
    let (day, hour) = schedule_time(unix_s);
    table()
        .get(line)
        .is_none_or(|days| days[day] >> hour & 1 == 1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn schedule_time_across_daylight_time() {
        assert_eq!(ZONE.trim(), "America/New_York");
        // Sun Nov 8 2026, 2:52 pm EST.
        assert_eq!(schedule_time(1_794_167_520), (6, 14));
        // Sun Nov 1 2026: 1:30 am EDT, then 1:30 am EST an hour later.
        assert_eq!(schedule_time(1_793_511_000), (6, 1));
        assert_eq!(schedule_time(1_793_514_600), (6, 1));
        assert_eq!(schedule_time(1_793_518_200), (6, 2));
        // Sun Mar 8 2026, 1:59 am EST, then 3 am EDT.
        assert_eq!(schedule_time(1_772_953_140), (6, 1));
        assert_eq!(schedule_time(1_772_953_200), (6, 3));
    }

    #[test]
    fn part_time_lines_run_only_at_their_hours() {
        // Sun Nov 8 2026, 2:52 pm: the A and C, not the B or W.
        let sunday = 1_794_167_520;
        assert!(runs("A", sunday) && runs("C", sunday));
        assert!(!runs("B", sunday) && !runs("W", sunday));
        // Mon Nov 9 2026, 2:52 pm: the B too; at 3:30 am, not the B or C.
        let monday = sunday + 86_400;
        assert!(runs("B", monday));
        let late = 1_794_213_000;
        assert_eq!(schedule_time(late), (0, 3));
        assert!(!runs("B", late) && !runs("C", late) && runs("A", late));
        assert!(runs("unlisted", late));
    }
}

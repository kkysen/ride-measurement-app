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

/// The day since 1970-01-01 of a civil date (proleptic Gregorian), as in Howard Hinnant's `days_from_civil`.
fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let doy = (153 * (m + if m > 2 { -3 } else { 9 }) + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

/// The year of a day since 1970-01-01.
fn year_of(days: i64) -> i64 {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    yoe + era * 400 + i64::from(m <= 2)
}

/// The `n`th Sunday (from 1) of a month, as a day since 1970-01-01.
fn nth_sunday(y: i64, m: i64, n: i64) -> i64 {
    let first = days_from_civil(y, m, 1);
    // 1970-01-01 was a Thursday: day 3 counting from Monday.
    let weekday = (first + 3).rem_euclid(7);
    first + (6 - weekday) + 7 * (n - 1)
}

/// New York's day of the week (0 for Monday) and hour at `unix_s`: daylight time (UTC-4) from the second
/// Sunday of March at 2 am to the first Sunday of November at 2 am, else UTC-5.
pub fn new_york(unix_s: i64) -> (usize, u32) {
    let year = year_of(unix_s.div_euclid(86_400));
    // 2 am standard time is 7:00 UTC; 2 am daylight time is 6:00 UTC.
    let starts = nth_sunday(year, 3, 2) * 86_400 + 7 * 3600;
    let ends = nth_sunday(year, 11, 1) * 86_400 + 6 * 3600;
    let offset = if (starts..ends).contains(&unix_s) {
        -4
    } else {
        -5
    };
    let local = unix_s + offset * 3600;
    let days = local.div_euclid(86_400);
    let weekday = (days + 3).rem_euclid(7) as usize;
    let hour = (local.rem_euclid(86_400) / 3600) as u32;
    (weekday, hour)
}

/// Whether `line` runs at `unix_s` by the schedule; true for a line it doesn't list.
pub fn runs(line: &str, unix_s: i64) -> bool {
    let (day, hour) = new_york(unix_s);
    table()
        .get(line)
        .is_none_or(|days| days[day] >> hour & 1 == 1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_york_time_across_daylight_time() {
        // Sun Nov 8 2026, 2:52 pm EST.
        assert_eq!(new_york(1_794_167_520), (6, 14));
        // Sun Nov 1 2026: 1:30 am EDT, then 1:30 am EST an hour later.
        assert_eq!(new_york(1_793_511_000), (6, 1));
        assert_eq!(new_york(1_793_514_600), (6, 1));
        assert_eq!(new_york(1_793_518_200), (6, 2));
        // Sun Mar 8 2026, 1:59 am EST, then 3 am EDT.
        assert_eq!(new_york(1_772_953_140), (6, 1));
        assert_eq!(new_york(1_772_953_200), (6, 3));
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
        assert_eq!(new_york(late), (0, 3));
        assert!(!runs("B", late) && !runs("C", late) && runs("A", late));
        assert!(runs("unlisted", late));
    }
}

//! New York local time for clock times in reports, with daylight saving time.
//!
//! The United States rule since 2007: daylight saving time (UTC-4) from 2 am on the second Sunday in March
//! to 2 am on the first Sunday in November, standard time (UTC-5) otherwise.
//! The app itself uses the platform's zone rules; this is for the CLI and analysis tools.

const HOUR_MS: i64 = 3_600_000;
const DAY_MS: i64 = 24 * HOUR_MS;

/// Days since 1970-01-01 of a proleptic Gregorian date (Howard Hinnant's `days_from_civil`).
fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

/// The year of a Unix time in UTC (`civil_from_days`).
fn year_of(unix_ms: i64) -> i64 {
    let z = unix_ms.div_euclid(DAY_MS) + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    yoe + era * 400 + i64::from(m <= 2)
}

/// Days since 1970-01-01 of the `n`th Sunday (1-based) of a month.
fn nth_sunday(y: i64, m: i64, n: i64) -> i64 {
    let first = days_from_civil(y, m, 1);
    // 1970-01-01 was a Thursday: weekday 0 is Sunday.
    let weekday = (first + 4).rem_euclid(7);
    first + (7 - weekday) % 7 + 7 * (n - 1)
}

/// New York's offset from UTC at a Unix time, in milliseconds (-4 or -5 hours).
pub fn new_york_offset_ms(unix_ms: i64) -> i64 {
    let y = year_of(unix_ms);
    // 2 am standard time is 7 am UTC; 2 am daylight time is 6 am UTC.
    let start = nth_sunday(y, 3, 2) * DAY_MS + 7 * HOUR_MS;
    let end = nth_sunday(y, 11, 1) * DAY_MS + 6 * HOUR_MS;
    if (start..end).contains(&unix_ms) {
        -4 * HOUR_MS
    } else {
        -5 * HOUR_MS
    }
}

/// A Unix time as New York local milliseconds (for splitting into days, hours, minutes).
pub fn to_new_york_ms(unix_ms: i64) -> i64 {
    unix_ms + new_york_offset_ms(unix_ms)
}

/// A Unix time as a 12-hour New York clock time, like `6:19:30 pm`.
pub fn new_york_clock(unix_ms: i64) -> String {
    let secs = to_new_york_ms(unix_ms).div_euclid(1000).rem_euclid(86_400);
    let (h, m, s) = (secs / 3600, secs / 60 % 60, secs % 60);
    format!(
        "{}:{m:02}:{s:02} {}",
        (h + 11) % 12 + 1,
        if h < 12 { "am" } else { "pm" }
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transitions_2026() {
        // 2026-03-08 7:00 UTC and 2026-11-01 6:00 UTC (as in `examples/passes.rs`).
        assert_eq!(new_york_offset_ms(1_772_953_200_000 - 1), -5 * HOUR_MS);
        assert_eq!(new_york_offset_ms(1_772_953_200_000), -4 * HOUR_MS);
        assert_eq!(new_york_offset_ms(1_793_512_800_000 - 1), -4 * HOUR_MS);
        assert_eq!(new_york_offset_ms(1_793_512_800_000), -5 * HOUR_MS);
    }

    #[test]
    fn clock() {
        // 2026-11-03 23:19:30 UTC is 6:19:30 pm in New York (standard time).
        let t = days_from_civil(2026, 11, 3) * DAY_MS + (23 * 3600 + 19 * 60 + 30) * 1000;
        assert_eq!(new_york_clock(t), "6:19:30 pm");
        // 2026-10-26 00:08:00 UTC is 8:08:00 pm on Oct 25 (daylight time).
        let t = days_from_civil(2026, 10, 26) * DAY_MS + 8 * 60 * 1000;
        assert_eq!(new_york_clock(t), "8:08:00 pm");
        assert_eq!(
            new_york_clock(days_from_civil(2027, 1, 1) * DAY_MS),
            "7:00:00 pm"
        );
    }
}

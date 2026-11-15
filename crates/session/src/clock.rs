//! Clock times in the ride's own time zone, with that zone's daylight saving rules.
//!
//! A session's zone comes from up to three sources, kept apart so tools can show where they disagree:
//! - the phone's zone: `SessionInfo.time_zone` at the start and `TimeZoneChange` records in `events`
//!   (the zone the phone's clock, and so the rider's taps, used);
//! - the location's zone: the zone each location fix falls in (`location` stream, any provider),
//!   from timezone-boundary-builder's boundaries (`tzf-rs`, lite data, borders within about 111 m);
//! - the zone implied by the session id: the phone's local time at the start, so the start's fixed offset.
//!
//! [`SessionClock::shown`] picks the one clock times are shown in: the phone's zone if recorded,
//! else the location's, else the id's fixed offset (UTC if none).
//! Zone rules come from `jiff`'s bundled copy of the IANA database, the same on the phone as on a computer.

use std::sync::LazyLock;

use jiff::Timestamp;
use jiff::civil::DateTime;
use jiff::tz::{Offset, TimeZone, TimeZoneDatabase};

use crate::RecordChunk;
use crate::SessionInfo;
use crate::proto::record::Payload;
use crate::read::Session;

static DATABASE: LazyLock<TimeZoneDatabase> = LazyLock::new(TimeZoneDatabase::bundled);
static FINDER: LazyLock<tzf_rs::EmbeddedFinder> = LazyLock::new(tzf_rs::EmbeddedFinder::new);

/// Location fixes looked up at most this often (ms): zones are hundreds of kilometers across.
const LOOKUP_EVERY_MS: i64 = 60_000;
/// Fixes less accurate than this (m) say too little about which side of a border the phone is on.
const MAX_FIX_ACCURACY_M: f32 = 5_000.0;
/// A session id's start time is to the second, and offsets are whole quarter hours.
const QUARTER_HOUR_MS: i64 = 15 * 60_000;

/// An IANA zone by id from the bundled database, if it knows it.
pub fn named_zone(id: &str) -> Option<TimeZone> {
    if id.is_empty() {
        return None;
    }
    DATABASE.get(id).ok()
}

/// A fixed offset from UTC, as a zone.
pub fn fixed_zone(utc_offset_ms: i64) -> TimeZone {
    let seconds = i32::try_from(utc_offset_ms.div_euclid(1000)).unwrap_or(0);
    TimeZone::fixed(Offset::from_seconds(seconds).unwrap_or(Offset::UTC))
}

/// The IANA zone a point falls in (`None` if the boundaries name none).
pub fn zone_at_location(lat: f64, lon: f64) -> Option<String> {
    if !lat.is_finite() || !lon.is_finite() {
        return None;
    }
    let name = FINDER.get_tz_name(lon, lat);
    (!name.is_empty()).then(|| name.to_owned())
}

fn timestamp(unix_ms: i64) -> Timestamp {
    Timestamp::from_millisecond(unix_ms).unwrap_or(Timestamp::UNIX_EPOCH)
}

/// One zone, in effect from a time on.
#[derive(Clone, Debug)]
pub struct ZoneSpan {
    /// Unix ms it takes effect; the first span's is `i64::MIN`.
    pub from_unix_ms: i64,
    /// Its IANA id, or the offset like `UTC-05:00` for a fixed one.
    pub name: String,
    pub zone: TimeZone,
}

/// The zones of a ride in time order, each in effect until the next; never empty.
#[derive(Clone, Debug)]
pub struct Clock {
    spans: Vec<ZoneSpan>,
}

impl Clock {
    /// One zone throughout.
    pub fn new(name: impl Into<String>, zone: TimeZone) -> Self {
        Self {
            spans: vec![ZoneSpan {
                from_unix_ms: i64::MIN,
                name: name.into(),
                zone,
            }],
        }
    }

    /// The IANA zone `id`, or the fixed `fallback_offset_ms` if the database doesn't know it.
    pub fn named_or_fixed(id: &str, fallback_offset_ms: i64) -> Self {
        match named_zone(id) {
            Some(zone) => Self::new(id, zone),
            None => Self::fixed(fallback_offset_ms),
        }
    }

    /// A fixed offset from UTC throughout.
    pub fn fixed(utc_offset_ms: i64) -> Self {
        let zone = fixed_zone(utc_offset_ms);
        let offset = zone.to_offset(Timestamp::UNIX_EPOCH);
        Self::new(format!("UTC{offset}"), zone)
    }

    /// From `from_unix_ms` on, `zone`; a change to the zone already in effect is kept out.
    pub fn change(&mut self, from_unix_ms: i64, name: impl Into<String>, zone: TimeZone) {
        let name = name.into();
        // Changes come in time order; one out of order replaces those after it.
        while self.spans.len() > 1 && self.spans.last().unwrap().from_unix_ms >= from_unix_ms {
            self.spans.pop();
        }
        if self.spans.last().unwrap().name != name {
            self.spans.push(ZoneSpan {
                from_unix_ms,
                name,
                zone,
            });
        }
    }

    pub fn spans(&self) -> &[ZoneSpan] {
        &self.spans
    }

    /// The zone in effect at a Unix time.
    pub fn span_at(&self, unix_ms: i64) -> &ZoneSpan {
        let i = self.spans.partition_point(|s| s.from_unix_ms <= unix_ms);
        &self.spans[i.saturating_sub(1)]
    }

    /// The offset from UTC at a Unix time, in milliseconds.
    pub fn utc_offset_ms(&self, unix_ms: i64) -> i64 {
        i64::from(
            self.span_at(unix_ms)
                .zone
                .to_offset(timestamp(unix_ms))
                .seconds(),
        ) * 1000
    }

    /// A Unix time as local milliseconds (for splitting into days, hours, minutes).
    pub fn local_ms(&self, unix_ms: i64) -> i64 {
        unix_ms + self.utc_offset_ms(unix_ms)
    }

    fn format(&self, unix_ms: i64, pattern: &str) -> String {
        let zone = self.span_at(unix_ms).zone.clone();
        timestamp(unix_ms)
            .to_zoned(zone)
            .strftime(pattern)
            .to_string()
    }

    /// A 12-hour clock time, like `6:19:30 pm`.
    pub fn time(&self, unix_ms: i64) -> String {
        self.format(unix_ms, "%-I:%M:%S %P")
    }

    /// A 12-hour clock time to the minute, like `6:19 pm`.
    pub fn time_hm(&self, unix_ms: i64) -> String {
        self.format(unix_ms, "%-I:%M %P")
    }

    /// The date, like `Sat Nov 14, 2026`.
    pub fn date(&self, unix_ms: i64) -> String {
        self.format(unix_ms, "%a %b %-d, %Y")
    }

    /// The zone's abbreviation then, like `EST` (or the offset, like `+0530`, where the database has none).
    pub fn abbreviation(&self, unix_ms: i64) -> String {
        self.format(unix_ms, "%Z")
    }
}

/// The offset from UTC (ms) a session id implies: the id is the phone's local time at the start
/// (`yyyyMMdd-HHmmss`, maybe with a `-2` suffix), so the offset is that minus `start_unix_ms`,
/// to the nearest quarter hour. `None` if the id isn't one.
pub fn offset_implied_by_id(session_id: &str, start_unix_ms: i64) -> Option<i64> {
    let stamp = session_id.get(..15)?;
    if session_id.len() > 15 && !session_id[15..].starts_with('-') {
        return None;
    }
    let local: DateTime = DateTime::strptime("%Y%m%d-%H%M%S", stamp).ok()?;
    let local_ms = local
        .to_zoned(TimeZone::UTC)
        .ok()?
        .timestamp()
        .as_millisecond();
    let offset = local_ms - start_unix_ms;
    // The id drops the start's milliseconds, so the difference is just under the offset.
    let rounded = (offset as f64 / QUARTER_HOUR_MS as f64).round() as i64 * QUARTER_HOUR_MS;
    // Real offsets are within -12 and +14 hours.
    (-12 * 3_600_000..=14 * 3_600_000)
        .contains(&rounded)
        .then_some(rounded)
}

/// The zones the phone's clock was in: the recorded zone at the start, then each recorded change.
/// `None` for a session from before zones were recorded.
pub fn phone_clock<'a>(
    info: &SessionInfo,
    events: impl IntoIterator<Item = &'a RecordChunk>,
) -> Option<Clock> {
    if info.time_zone.is_empty() {
        return None;
    }
    let implied = offset_implied_by_id(&info.session_id, info.start_unix_ms).unwrap_or(0);
    let mut clock = Clock::named_or_fixed(&info.time_zone, implied);
    let mut changes: Vec<_> = events
        .into_iter()
        .flat_map(|c| &c.records)
        .filter_map(|r| match &r.payload {
            Some(Payload::TimeZoneChange(z)) => Some(z),
            _ => None,
        })
        .collect();
    changes.sort_by_key(|z| z.unix_ms);
    for z in changes {
        let fallback = Clock::fixed(i64::from(z.utc_offset_s) * 1000);
        match named_zone(&z.time_zone) {
            Some(zone) => clock.change(z.unix_ms, &z.time_zone, zone),
            None => {
                let span = &fallback.spans[0];
                clock.change(z.unix_ms, span.name.clone(), span.zone.clone());
            }
        }
    }
    Some(clock)
}

/// The zones the ride's location fixes fall in (any provider, so network fixes count too),
/// looked up about once a minute. A new zone takes effect only once two lookups in a row agree,
/// so a path along a border doesn't flicker between zones. `None` if no fix places it.
pub fn location_clock<'a>(locations: impl IntoIterator<Item = &'a RecordChunk>) -> Option<Clock> {
    let mut fixes: Vec<(i64, f64, f64)> = locations
        .into_iter()
        .flat_map(|c| &c.records)
        .filter_map(|r| match &r.payload {
            Some(Payload::Location(l))
                if !l.is_mock
                    && l.time_ms > 0
                    && l.horizontal_accuracy_meters
                        .is_none_or(|a| a <= MAX_FIX_ACCURACY_M) =>
            {
                Some((l.time_ms, l.latitude_degrees, l.longitude_degrees))
            }
            _ => None,
        })
        .collect();
    fixes.sort_by_key(|f| f.0);
    let mut clock: Option<Clock> = None;
    let mut last_lookup = i64::MIN;
    // A zone seen once, waiting for a second lookup to agree: when, and which.
    let mut pending: Option<(i64, String)> = None;
    for (t, lat, lon) in fixes {
        if last_lookup != i64::MIN && t - last_lookup < LOOKUP_EVERY_MS {
            continue;
        }
        last_lookup = t;
        let Some(name) = zone_at_location(lat, lon) else {
            continue;
        };
        let Some(zone) = named_zone(&name) else {
            continue;
        };
        let Some(c) = clock.as_mut() else {
            clock = Some(Clock::new(name, zone));
            continue;
        };
        if c.spans.last().unwrap().name == name {
            pending = None;
            continue;
        }
        match pending.take() {
            Some((from, p)) if p == name => c.change(from, name, zone),
            _ => pending = Some((t, name)),
        }
    }
    clock
}

/// A session's clock sources (see the module docs).
#[derive(Clone, Debug)]
pub struct SessionClock {
    pub phone: Option<Clock>,
    pub location: Option<Clock>,
    /// The offset (ms) the session id implies at the start.
    pub implied_offset_ms: Option<i64>,
    start_unix_ms: i64,
    shown: Clock,
}

impl SessionClock {
    /// From the session's info, its `events` (zone changes) and its `location` fixes.
    pub fn from_parts<'a>(
        info: &SessionInfo,
        events: impl IntoIterator<Item = &'a RecordChunk>,
        locations: impl IntoIterator<Item = &'a RecordChunk>,
    ) -> Self {
        let phone = phone_clock(info, events);
        let location = location_clock(locations);
        let implied_offset_ms = offset_implied_by_id(&info.session_id, info.start_unix_ms);
        let shown = phone
            .clone()
            .or_else(|| location.clone())
            .unwrap_or_else(|| Clock::fixed(implied_offset_ms.unwrap_or(0)));
        Self {
            phone,
            location,
            implied_offset_ms,
            start_unix_ms: info.start_unix_ms,
            shown,
        }
    }

    /// Reads the streams it needs; a missing or damaged stream only leaves its source out.
    pub fn for_session(session: &Session) -> Self {
        let has = |name: &str| session.info.streams.iter().any(|s| s.name == name);
        let read = |name: &str| {
            if has(name) {
                session
                    .read_records(name)
                    .map(|c| c.chunks)
                    .unwrap_or_default()
            } else {
                Vec::new()
            }
        };
        let events = read("events");
        let locations = read("location");
        Self::from_parts(&session.info, &events, &locations)
    }

    /// The clock times are shown in: the phone's zone if recorded (its clock timed the rider's taps),
    /// else the location's, else the session id's fixed offset.
    pub fn shown(&self) -> &Clock {
        &self.shown
    }

    pub fn utc_offset_ms(&self, unix_ms: i64) -> i64 {
        self.shown.utc_offset_ms(unix_ms)
    }

    pub fn local_ms(&self, unix_ms: i64) -> i64 {
        self.shown.local_ms(unix_ms)
    }

    pub fn time(&self, unix_ms: i64) -> String {
        self.shown.time(unix_ms)
    }

    /// The zones, one line per source, for tools to print: each zone with when it took effect,
    /// and whether the phone's and the location's disagree on the clock.
    pub fn describe(&self) -> String {
        let list = |c: &Clock| {
            c.spans
                .iter()
                .map(|s| {
                    if s.from_unix_ms == i64::MIN {
                        s.name.clone()
                    } else {
                        format!("{} from {}", s.name, c.time(s.from_unix_ms))
                    }
                })
                .collect::<Vec<_>>()
                .join(", then ")
        };
        let mut lines = Vec::new();
        match &self.phone {
            Some(c) => lines.push(format!("phone zone: {}", list(c))),
            None => lines.push("phone zone: not recorded".to_owned()),
        }
        match &self.location {
            Some(c) => lines.push(format!("location zone: {}", list(c))),
            None => lines.push("location zone: no fixes".to_owned()),
        }
        if let Some(o) = self.implied_offset_ms {
            lines.push(format!("id's offset: {}", Clock::fixed(o).spans[0].name));
        }
        if let (Some(p), Some(l)) = (&self.phone, &self.location)
            && let Some(t) = disagreement(p, l, self.start_unix_ms)
        {
            lines.push(format!(
                "the phone's clock and the location's differ from {} (phone) on; times shown in the phone's",
                p.time(t)
            ));
        }
        lines.join("\n")
    }
}

/// The first time from the ride's start that two clocks give different offsets:
/// at the start, or at a zone change of either after it.
fn disagreement(a: &Clock, b: &Clock, start_unix_ms: i64) -> Option<i64> {
    let mut times: Vec<i64> = a
        .spans
        .iter()
        .chain(&b.spans)
        .map(|s| s.from_unix_ms)
        .filter(|t| *t > start_unix_ms)
        .collect();
    times.sort_unstable();
    std::iter::once(start_unix_ms)
        .chain(times)
        .find(|t| a.utc_offset_ms(*t) != b.utc_offset_ms(*t))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{Location, Record, TimeZoneChange};

    const HOUR_MS: i64 = 3_600_000;

    fn unix_ms(y: i16, m: i8, d: i8, h: i8, min: i8, s: i8) -> i64 {
        jiff::civil::date(y, m, d)
            .at(h, min, s, 0)
            .to_zoned(TimeZone::UTC)
            .unwrap()
            .timestamp()
            .as_millisecond()
    }

    #[test]
    fn new_york_across_2026_11_01() {
        let c = Clock::named_or_fixed("America/New_York", 0);
        // 2 am daylight time is 6 am UTC.
        let change = unix_ms(2026, 11, 1, 6, 0, 0);
        assert_eq!(c.utc_offset_ms(change - 1), -4 * HOUR_MS);
        assert_eq!(c.utc_offset_ms(change), -5 * HOUR_MS);
        assert_eq!(c.time(change - 1000), "1:59:59 am");
        assert_eq!(c.time(change), "1:00:00 am");
        assert_eq!(c.abbreviation(change - 1), "EDT");
        assert_eq!(c.abbreviation(change), "EST");
        // And in March: 2 am standard time is 7 am UTC.
        let spring = unix_ms(2026, 3, 8, 7, 0, 0);
        assert_eq!(c.utc_offset_ms(spring - 1), -5 * HOUR_MS);
        assert_eq!(c.utc_offset_ms(spring), -4 * HOUR_MS);
        assert_eq!(c.time(unix_ms(2026, 11, 3, 23, 19, 30)), "6:19:30 pm");
        assert_eq!(c.time_hm(unix_ms(2026, 10, 26, 0, 8, 0)), "8:08 pm");
        assert_eq!(c.date(unix_ms(2026, 10, 26, 0, 8, 0)), "Sun Oct 25, 2026");
    }

    #[test]
    fn london_and_kolkata() {
        let london = Clock::named_or_fixed("Europe/London", 0);
        // British Summer Time ends at 1 am UTC on the last Sunday in October.
        let end = unix_ms(2026, 10, 25, 1, 0, 0);
        assert_eq!(london.utc_offset_ms(end - 1), HOUR_MS);
        assert_eq!(london.utc_offset_ms(end), 0);
        assert_eq!(london.time(unix_ms(2026, 11, 3, 23, 19, 30)), "11:19:30 pm");
        let kolkata = Clock::named_or_fixed("Asia/Kolkata", 0);
        assert_eq!(kolkata.utc_offset_ms(end), 5 * HOUR_MS + HOUR_MS / 2);
        assert_eq!(kolkata.time(unix_ms(2026, 11, 3, 23, 19, 30)), "4:49:30 am");
        assert_eq!(
            kolkata.date(unix_ms(2026, 11, 3, 23, 19, 30)),
            "Wed Nov 4, 2026"
        );
    }

    #[test]
    fn unknown_zone_falls_back_to_the_offset() {
        let c = Clock::named_or_fixed("Mars/Olympus_Mons", -5 * HOUR_MS);
        assert_eq!(c.spans()[0].name, "UTC-05");
        assert_eq!(c.time(unix_ms(2026, 7, 1, 12, 0, 0)), "7:00:00 am");
    }

    #[test]
    fn offset_from_old_session_ids() {
        // Sat Nov 14, 7:01:48 pm in New York (standard time), started 0.4 s into the second.
        let start = unix_ms(2026, 11, 15, 0, 1, 48) + 400;
        assert_eq!(
            offset_implied_by_id("20261114-190148", start),
            Some(-5 * HOUR_MS)
        );
        assert_eq!(
            offset_implied_by_id("20261114-190148-2", start),
            Some(-5 * HOUR_MS)
        );
        // Kolkata's half hour.
        let start = unix_ms(2026, 11, 14, 13, 31, 48) + 999;
        assert_eq!(
            offset_implied_by_id("20261114-190148", start),
            Some(5 * HOUR_MS + HOUR_MS / 2)
        );
        assert_eq!(offset_implied_by_id("ride", start), None);
        assert_eq!(offset_implied_by_id("20261114-190148x", start), None);
    }

    fn info(id: &str, zone: &str, start: i64) -> SessionInfo {
        SessionInfo {
            session_id: id.into(),
            start_unix_ms: start,
            time_zone: zone.into(),
            ..Default::default()
        }
    }

    fn chunk(payloads: Vec<Payload>) -> RecordChunk {
        RecordChunk {
            records: payloads
                .into_iter()
                .map(|p| Record {
                    payload: Some(p),
                    ..Default::default()
                })
                .collect(),
            ..Default::default()
        }
    }

    fn fix(time_ms: i64, lat: f64, lon: f64) -> Payload {
        Payload::Location(Location {
            provider: "gps".into(),
            time_ms,
            latitude_degrees: lat,
            longitude_degrees: lon,
            horizontal_accuracy_meters: Some(10.0),
            ..Default::default()
        })
    }

    #[test]
    fn old_session_falls_back_to_location_then_id() {
        let start = unix_ms(2026, 11, 15, 0, 1, 48) + 400;
        let i = info("20261114-190148", "", start);
        // No zone, no fixes: the id's fixed offset.
        let c = SessionClock::from_parts(&i, &[], &[]);
        assert!(c.phone.is_none() && c.location.is_none());
        assert_eq!(c.time(start), "7:01:48 pm");
        assert_eq!(c.shown().spans()[0].name, "UTC-05");
        // A fix in Manhattan: the location's zone.
        let locations = [chunk(vec![fix(start, 40.7527, -73.9772)])];
        let c = SessionClock::from_parts(&i, &[], &locations);
        assert_eq!(c.shown().spans()[0].name, "America/New_York");
        assert_eq!(c.time(start), "7:01:48 pm");
    }

    #[test]
    fn phone_zone_changes_mid_ride() {
        let start = unix_ms(2026, 11, 14, 12, 0, 0);
        let i = info("20261114-070000", "America/New_York", start);
        let landed = start + 3 * HOUR_MS;
        let events = [chunk(vec![
            Payload::TimeZoneChange(TimeZoneChange {
                time_zone: "Europe/London".into(),
                utc_offset_s: 0,
                unix_ms: landed,
            }),
            Payload::TimeZoneChange(TimeZoneChange {
                time_zone: "Nowhere/Unknown".into(),
                utc_offset_s: 19_800,
                unix_ms: landed + HOUR_MS,
            }),
        ])];
        let c = SessionClock::from_parts(&i, &events, &[]);
        assert_eq!(c.time(start), "7:00:00 am");
        assert_eq!(c.time(landed - 1000), "9:59:59 am");
        assert_eq!(c.time(landed), "3:00:00 pm");
        // An id the database doesn't know: the offset recorded with it.
        assert_eq!(c.time(landed + HOUR_MS), "9:30:00 pm");
        assert_eq!(c.shown().spans().len(), 3);
    }

    #[test]
    fn location_follows_a_flight_and_disagrees_with_the_phone() {
        let start = unix_ms(2026, 11, 14, 12, 0, 0);
        // The phone stayed on New York's zone (airplane mode); the fixes go from New York to Chicago.
        let i = info("20261114-070000", "America/New_York", start);
        let mut fixes = Vec::new();
        for k in 0..10 {
            fixes.push(fix(start + k * 120_000, 40.64, -73.78));
        }
        let arrived = start + 3 * HOUR_MS;
        for k in 0..10 {
            fixes.push(fix(arrived + k * 120_000, 41.98, -87.90));
        }
        let locations = [chunk(fixes)];
        let c = SessionClock::from_parts(&i, &[], &locations);
        let location = c.location.as_ref().unwrap();
        let names: Vec<_> = location.spans().iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, ["America/New_York", "America/Chicago"]);
        assert_eq!(location.spans()[1].from_unix_ms, arrived);
        // Shown in the phone's zone, both listed.
        assert_eq!(c.time(arrived), "10:00:00 am");
        assert_eq!(location.time(arrived), "9:00:00 am");
        let d = c.describe();
        assert!(d.contains("phone zone: America/New_York"), "{d}");
        assert!(d.contains("America/Chicago from 9:00:00 am"), "{d}");
        assert!(d.contains("differ from 10:00:00 am"), "{d}");
    }

    #[test]
    fn single_zones_compared_at_the_start() {
        // Phoenix keeps standard time; Denver doesn't: an hour apart in July, the same in January.
        let fixes = |start: i64| [chunk(vec![fix(start, 39.74, -104.99)])];
        let july = unix_ms(2026, 7, 1, 18, 0, 0);
        let c = SessionClock::from_parts(
            &info("20260701-110000", "America/Phoenix", july),
            &[],
            &fixes(july),
        );
        assert_eq!(
            c.location.as_ref().unwrap().spans()[0].name,
            "America/Denver"
        );
        let d = c.describe();
        assert!(d.contains("differ from 11:00:00 am"), "{d}");
        let january = unix_ms(2026, 1, 15, 18, 0, 0);
        let c = SessionClock::from_parts(
            &info("20260115-110000", "America/Phoenix", january),
            &[],
            &fixes(january),
        );
        assert!(!c.describe().contains("differ"), "{}", c.describe());
    }

    #[test]
    fn one_stray_lookup_is_no_change() {
        let start = unix_ms(2026, 11, 14, 12, 0, 0);
        let mut fixes: Vec<_> = (0..5)
            .map(|k| fix(start + k * 120_000, 40.75, -73.98))
            .collect();
        fixes.push(fix(start + 5 * 120_000, 41.98, -87.90));
        fixes.extend((6..9).map(|k| fix(start + k * 120_000, 40.75, -73.98)));
        let c = location_clock(&[chunk(fixes)]).unwrap();
        assert_eq!(c.spans().len(), 1);
    }
}

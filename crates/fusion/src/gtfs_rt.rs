//! Which of the MTA's subway GTFS-realtime feeds to record while riding: the ones for lines with a station near
//! the rider, so a later analysis knows which trip (and, with its vehicle positions, which train) the rider was on.
//! The feeds are public and need no key (checked 2026-10-22, and for v2 on 2026-11-08).
//! These are the MTA's v2 feeds (GTFS-realtime 2.0, launched before daylight saving time ended on 2026-11-01):
//! the same line groups as v1, but they handle the repeated 1 am hour, drop non-revenue stops, include past stops
//! (so a trip's stops behind the train are kept), reflect closed platforms, and relabel tracks.
//! v1 lacked the 2's reroute past a fire on Tue Oct 27; the MTA plans to retire it in 2027.
//! With past stops they're bigger: the numbered lines' feed was 91 KB against v1's 51 KB at 4 am on 2026-11-08
//! (all eight v1 feeds were about 680 KB a poll in the evening of 2026-10-22).

use crate::stations::{all, meters};

const BASE: &str = "https://api-endpoint.mta.info/Dataservice/mtagtfsfeeds/nyct-v2%2Fgtfs";
/// Every subway feed, by its URL suffix.
pub const FEEDS: [&str; 8] = ["", "-ace", "-bdfm", "-g", "-jz", "-nqrw", "-l", "-si"];
/// How often to poll while recording (ms); the feeds update about every 30 s.
pub const POLL_MS: i64 = 30_000;
/// The subway's service alerts (delays, reroutes, planned work), recorded alongside the feeds near the rider
/// for why a train was held or ran differently. About 450 KB a poll, 73 KB gzipped as fetched (2026-11-09),
/// so polled less often than the trains' feeds.
pub const ALERTS_URL: &str =
    "https://api-endpoint.mta.info/Dataservice/mtagtfsfeeds/camsys%2Fsubway-alerts";
/// How often to poll [`ALERTS_URL`] while any subway feed is polled (ms).
pub const ALERTS_POLL_MS: i64 = 60_000;
/// Lines with a station this close (m) are recorded: a few stations either way, and a transfer's other lines.
pub const NEAR_M: f64 = 1_500.0;

/// The feeds (URL suffixes) carrying a line, as the MTA's station list names them.
/// `S` is any of the three shuttles: 42 St's is in the numbered lines' feed, Franklin Av's and Rockaway's in the A's.
fn feeds_of(line: &str) -> &'static [&'static str] {
    match line {
        "1" | "2" | "3" | "4" | "5" | "6" | "7" => &[""],
        "A" | "C" | "E" => &["-ace"],
        "S" => &["", "-ace"],
        "B" | "D" | "F" | "M" => &["-bdfm"],
        "G" => &["-g"],
        "J" | "Z" => &["-jz"],
        "N" | "Q" | "R" | "W" => &["-nqrw"],
        "L" => &["-l"],
        "SIR" => &["-si"],
        _ => &[],
    }
}

/// The feed URLs to poll now, nearest first: those for lines with a station within [`NEAR_M`] of the last known
/// position, ordered by their nearest station (a brief signal in a station gets the rider's own line first),
/// or every feed without a position (a ride starting underground). Outside the city, none.
pub fn feeds_near(position: Option<(f64, f64)>) -> Vec<String> {
    let suffixes: Vec<&str> = match position {
        None => FEEDS.to_vec(),
        Some(p) => {
            let mut near: Vec<(f64, &str)> = FEEDS
                .into_iter()
                .filter_map(|f| {
                    all()
                        .iter()
                        .filter(|s| s.lines.iter().any(|l| feeds_of(l).contains(&f)))
                        .map(|s| meters(p, (s.lat, s.lon)))
                        .filter(|&d| d <= NEAR_M)
                        .min_by(f64::total_cmp)
                        .map(|d| (d, f))
                })
                .collect();
            near.sort_by(|a, b| a.0.total_cmp(&b.0));
            near.into_iter().map(|(_, f)| f).collect()
        }
    };
    suffixes.into_iter().map(|f| format!("{BASE}{f}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn suffixes(position: Option<(f64, f64)>) -> Vec<String> {
        feeds_near(position)
            .into_iter()
            .map(|u| u.trim_start_matches(BASE).to_string())
            .collect()
    }

    #[test]
    fn every_line_has_a_feed() {
        for s in all() {
            for l in &s.lines {
                assert!(!feeds_of(l).is_empty(), "{l} at {}", s.name);
            }
        }
    }

    #[test]
    fn feeds_follow_the_nearby_lines() {
        assert_eq!(suffixes(None).len(), FEEDS.len());
        // Times Sq: the numbered lines, the A/C/E at 42 St-Port Authority, B/D/F/M at 42 St-Bryant Pk, N/Q/R/W.
        let times_sq = suffixes(Some((40.7556, -73.9870)));
        for f in ["", "-ace", "-bdfm", "-nqrw"] {
            assert!(times_sq.contains(&f.to_string()), "{f} in {times_sq:?}");
        }
        assert!(!times_sq.contains(&"-si".to_string()));
        // 110 St on the 1, near the B/C on Central Park West.
        let home = suffixes(Some((40.8040, -73.9668)));
        assert_eq!(home[0], "", "the 1's feed first: {home:?}");
        assert!(home.contains(&"-ace".to_string()), "{home:?}");
        assert!(!home.contains(&"-l".to_string()));
        // Newark: no subway within reach.
        assert!(suffixes(Some((40.7357, -74.1724))).is_empty());
    }
}

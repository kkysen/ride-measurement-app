//! Which route the rider's train is: its recent stops matched against the trips the MTA's GTFS-realtime feeds
//! (the `transit` stream, see [`crate::gtfs_rt`]) say stopped at those stations then, as in
//! `python/experiments/actual_trips.py`, but live. The feeds only list a trip's stops still ahead of it,
//! so each stop's last prediction is kept once it drops out: by then it's within a poll (30 s) of what happened.

use std::collections::HashMap;
use std::sync::OnceLock;

use ride_session::transit_realtime::FeedMessage;

use crate::stations::{Station, meters};

/// The GTFS parent stations (`data/mta/gtfs/stops.txt`, `location_type` 1): stop ID and position.
const GTFS_STOPS: &str = include_str!("../data/gtfs_stops.csv");
/// GTFS stations this close (m) to the MTA's station are its platforms; positions, not names,
/// since names repeat (several 96 Sts and 7 Avs).
const NEAR_M: f64 = 250.0;
/// How far (s) a trip's time at a stop can be outside the rider's stop and still match it.
pub const SLACK_S: i64 = 45;
/// Trips not in a feed for this long (s) are dropped.
const KEEP_S: i64 = 7_200;
/// Another route's best trip within this (s) of the best's mean error leaves the route undecided.
const AMBIGUOUS_S: f64 = 15.0;

fn gtfs_stops() -> &'static [(String, f64, f64)] {
    static STOPS: OnceLock<Vec<(String, f64, f64)>> = OnceLock::new();
    STOPS.get_or_init(|| {
        GTFS_STOPS
            .lines()
            .skip(1)
            .filter_map(|line| {
                let f: Vec<&str> = line.split(',').collect();
                let [id, lat, lon] = f[..] else { return None };
                Some((id.to_string(), lat.parse().ok()?, lon.parse().ok()?))
            })
            .collect()
    })
}

/// The GTFS parent stop IDs of a station.
pub fn stop_ids(station: &Station) -> Vec<&'static str> {
    gtfs_stops()
        .iter()
        .filter(|(_, lat, lon)| meters((station.lat, station.lon), (*lat, *lon)) <= NEAR_M)
        .map(|(id, _, _)| id.as_str())
        .collect()
}

/// How close (m) a GTFS stop is to the MTA's station to be that platform itself (they're listed at one spot).
const OWN_M: f64 = 1.0;

/// The GTFS stop IDs of the station's own platform, not the rest of its complex.
fn own_stop_ids(station: &Station) -> Vec<&'static str> {
    gtfs_stops()
        .iter()
        .filter(|(_, lat, lon)| meters((station.lat, station.lon), (*lat, *lon)) <= OWN_M)
        .map(|(id, _, _)| id.as_str())
        .collect()
}

/// A route ID as the rider names the line: express variants (`7X`, `6X`, `FX`) are their line.
pub fn line_of_route(route: &str) -> String {
    match route.strip_suffix('X') {
        Some(line) if !line.is_empty() => line.to_string(),
        _ => route.to_string(),
    }
}

/// One stop of a trip in a feed: trip ID, route, parent stop ID, arrival and departure (unix s).
#[derive(Clone, Debug, PartialEq)]
pub struct TripStop {
    pub trip_id: String,
    pub route: String,
    pub stop_id: String,
    pub arrival: i64,
    pub departure: i64,
}

/// A feed's stops, compact: what's kept of a poll.
pub fn trip_stops(feed: &FeedMessage) -> Vec<TripStop> {
    let mut out = Vec::new();
    for e in &feed.entity {
        let Some(t) = &e.trip_update else { continue };
        let Some(trip_id) = t.trip.trip_id.clone() else {
            continue;
        };
        let route = t.trip.route_id.clone().unwrap_or_default();
        for s in &t.stop_time_update {
            let Some(id) = &s.stop_id else { continue };
            let time = |e: &Option<ride_session::transit_realtime::trip_update::StopTimeEvent>| {
                e.as_ref().and_then(|e| e.time)
            };
            let (Some(arrival), Some(departure)) = (
                time(&s.arrival).or(time(&s.departure)),
                time(&s.departure).or(time(&s.arrival)),
            ) else {
                continue;
            };
            // Platform IDs end in their direction (`120N`); the station is the rest.
            let stop_id = id
                .strip_suffix(['N', 'S'])
                .unwrap_or(id.as_str())
                .to_string();
            out.push(TripStop {
                trip_id: trip_id.clone(),
                route: route.clone(),
                stop_id,
                arrival,
                departure,
            });
        }
    }
    out
}

/// A rider's stop as matched: its complex's and its own platform's stop IDs, stopped and departed (unix s),
/// and the lines listed at its spot.
type Stop<'a> = (Vec<&'a str>, Vec<&'a str>, i64, i64, Vec<String>);

#[derive(Clone, Debug, Default)]
struct Trip {
    route: String,
    last_seen: i64,
    stops: HashMap<String, (i64, i64)>,
}

/// One of the rider's stops: when the train stopped and left (elapsed realtime ns; still there if none),
/// and the station.
#[derive(Clone, Copy, Debug)]
pub struct RiderStop<'a> {
    pub stopped_ns: i64,
    pub departed_ns: Option<i64>,
    pub station: &'a Station,
}

/// Every trip the feeds have listed, with each stop's latest time.
#[derive(Clone, Debug, Default)]
pub struct Trips {
    trips: HashMap<String, Trip>,
    /// Unix ns minus elapsed realtime ns, from the latest poll.
    offset_ns: Option<i64>,
}

impl Trips {
    /// A poll's stops, received at `received_ns` for a request at `request_unix_ms`.
    pub fn on_poll(&mut self, received_ns: i64, request_unix_ms: i64, stops: Vec<TripStop>) {
        let now = request_unix_ms / 1000;
        self.offset_ns = Some(request_unix_ms * 1_000_000 - received_ns);
        for s in stops {
            let trip = self.trips.entry(s.trip_id).or_default();
            trip.route = s.route;
            trip.last_seen = now;
            trip.stops.insert(s.stop_id, (s.arrival, s.departure));
        }
        self.trips.retain(|_, t| now - t.last_seen <= KEEP_S);
    }

    /// The route of the trip matching the most of the rider's stops (the last of which must match), then the
    /// smallest mean error; none without feed data, or when another route matches as well.
    /// A trip only matches a stop at a station its line serves: [`stop_ids`] takes in a complex's other
    /// platforms, so a Q at Union Sq would otherwise match the L stopping there; platforms listed at one spot
    /// count as one (see [`crate::stations::same_spot_lines`]): the N/W at Queensboro Plaza named at its 7 one.
    /// Only a line stopping at each stop's complex can match, or a trip the feeds list at the stop's own platform:
    /// the A, running local on weekends, at 50 St, which the MTA lists for the C and E (Sat Oct 31, 5:13 pm).
    /// `now_ns` stands in for the departure of a stop the train hasn't left.
    pub fn route(&self, stops: &[RiderStop], now_ns: i64) -> Option<String> {
        self.best_trip(stops, &[], now_ns).map(|(route, ..)| route)
    }

    /// [`Self::route`], else the route of the best trip for the stops up to an older one, if it matched each of
    /// them and the newer stops don't rule it out (see [`Self::contradicts`]). On Tue Nov 3 the W's stop named
    /// Queensboro Plaza at 6:10:43 pm no longer matched once the feeds settled the W's time there at 6:12:10 pm
    /// (that stop's cell names the next station early: Broadway at 30 Av, Queensboro Plaza at 39 Av), and the
    /// stops after, underground through the 60 St tunnel to 57 St-7 Av, had no station, so nothing matched again.
    /// Only the best trip of older stops is kept, never one found from the newer stops' times alone:
    /// an N a station ahead lists 5 Av, 57 St, 49 St and Times Sq within seconds of the rider's stops then.
    pub fn held_route(&self, stops: &[RiderStop], now_ns: i64) -> Option<String> {
        self.held_route_passing(stops, &[], now_ns)
    }

    /// [`Self::held_route`] with no trip that stops at a station the train passed without stopping
    /// (see [`Self::stops_at`]): on Tue Nov 3 the rider's train passed 49 St after 57 St-7 Av, so it wasn't the W
    /// the feeds still had it on, but the Q, express there.
    pub fn held_route_passing(
        &self,
        stops: &[RiderStop],
        passed: &[&Station],
        now_ns: i64,
    ) -> Option<String> {
        if let Some((route, ..)) = self.best_trip(stops, passed, now_ns) {
            return Some(route);
        }
        let offset = self.offset_ns?;
        let unix_s = |ns: i64| (ns + offset).div_euclid(1_000_000_000);
        (1..stops.len()).rev().find_map(|k| {
            // Kept only when it matched each of those stops: a W the other way, at Queensboro Plaza as the
            // rider's train got there, matches it but not Broadway before.
            let (route, trip, matched) = self.best_trip(&stops[..k], passed, now_ns)?;
            if matched < k {
                return None;
            }
            let ruled_out = stops[k..].iter().any(|s| {
                let departed = unix_s(s.departed_ns.unwrap_or(now_ns));
                Self::contradicts(trip, s.station, departed)
            });
            (!ruled_out).then_some(route)
        })
    }

    /// The trip's earliest time (unix s) at the station's complex, if the feeds list it there.
    fn arrival(trip: &Trip, station: &Station) -> Option<i64> {
        stop_ids(station)
            .iter()
            .filter_map(|id| trip.stops.get(*id))
            .map(|&(a, _)| a)
            .min()
    }

    /// Whether a later stop of the rider at `station`, left at `departed` (unix s), rules out the trip:
    /// its line doesn't stop at the station's platform (or one listed at its spot) nor do the feeds list it there,
    /// or it reached a stop before this one, in its own order, after the rider left here.
    /// The platform, not the complex: the rider who rode the Q to Times Sq and walked to the 1 unseen
    /// (Sat Nov 7, 6:25 pm) stops at the 1's platform, where the Q doesn't.
    fn contradicts(trip: &Trip, station: &Station, departed: i64) -> bool {
        let line = line_of_route(&trip.route);
        let own = own_stop_ids(station);
        if !crate::stations::same_spot_lines(station).contains(&line)
            && !own.iter().any(|id| trip.stops.contains_key(*id))
        {
            return true;
        }
        let Some(here) = Self::arrival(trip, station) else {
            return false;
        };
        trip.stops
            .values()
            .any(|&(a, _)| a < here && a > departed + SLACK_S)
    }

    /// Whether the trip stops at `station`'s platform (or one listed at its spot): the feeds list it there,
    /// or its route does in the MTA's list. By route, not line: an express variant (`6X`) passes the 6's locals.
    fn stops_at(trip: &Trip, station: &Station) -> bool {
        crate::stations::same_spot_lines(station).contains(&trip.route)
            || own_stop_ids(station)
                .iter()
                .any(|id| trip.stops.contains_key(*id))
    }

    /// The route of [`Self::route`], its best trip, and how many of the stops that trip matches;
    /// no trip stopping at a `passed` station counts.
    fn best_trip(
        &self,
        stops: &[RiderStop],
        passed: &[&Station],
        now_ns: i64,
    ) -> Option<(String, &Trip, usize)> {
        let offset = self.offset_ns?;
        let unix_s = |ns: i64| (ns + offset).div_euclid(1_000_000_000);
        // The feeds often no longer list a stop passed before the first poll, so a line that doesn't stop at one
        // of the rider's stations at all could match the rest: the 7, at Queensboro Plaza then too,
        // with the N/W boarded at Broadway before (Tue Nov 3, 6:10 pm).
        let own: Vec<Vec<&str>> = stops.iter().map(|s| own_stop_ids(s.station)).collect();
        let lists_own =
            |trip: &Trip, own: &[&str]| own.iter().any(|id| trip.stops.contains_key(*id));
        let serves_all = |trip: &Trip, line: &str| {
            stops
                .iter()
                .zip(&own)
                .all(|(s, own)| crate::stations::serves(s.station, line) || lists_own(trip, own))
        };
        let stops: Vec<Stop> = stops
            .iter()
            .zip(&own)
            .map(|(s, own)| {
                (
                    stop_ids(s.station),
                    own.clone(),
                    unix_s(s.stopped_ns),
                    unix_s(s.departed_ns.unwrap_or(now_ns)),
                    crate::stations::same_spot_lines(s.station),
                )
            })
            .collect();
        let (last, _) = stops.split_last()?;
        // Per route: its best trip's (matches, mean error), and that trip.
        let mut best: HashMap<&str, ((usize, f64), &Trip)> = HashMap::new();
        for trip in self.trips.values() {
            let line = line_of_route(&trip.route);
            let error = |(ids, own, stopped, departed, lines): &Stop| {
                // Another line's trip only at the stop's own platform, never the rest of its complex.
                let ids = if !lines.is_empty() && !lines.contains(&line) {
                    own
                } else {
                    ids
                };
                ids.iter()
                    .filter_map(|id| trip.stops.get(*id))
                    .filter(|&&(a, d)| a <= departed + SLACK_S && d >= stopped - SLACK_S)
                    .map(|&(a, _)| (a - stopped).abs())
                    .min()
            };
            if !serves_all(trip, &line)
                || error(last).is_none()
                || passed.iter().any(|p| Self::stops_at(trip, p))
            {
                continue;
            }
            let errors: Vec<i64> = stops.iter().filter_map(error).collect();
            let score = (
                errors.len(),
                errors.iter().sum::<i64>() as f64 / errors.len() as f64,
            );
            let b = best.entry(trip.route.as_str()).or_insert((score, trip));
            if (score.0, -score.1) > (b.0.0, -b.0.1) {
                *b = (score, trip);
            }
        }
        let mut ranked: Vec<_> = best.into_iter().collect();
        ranked.sort_by(|a, b| b.1.0.0.cmp(&a.1.0.0).then(a.1.0.1.total_cmp(&b.1.0.1)));
        match ranked[..] {
            [] => None,
            [(r, (a, trip))] => Some((line_of_route(r), trip, a.0)),
            [(r, (a, trip)), (r2, (b, _)), ..] => {
                let tied = a.0 == b.0 && b.1 - a.1 < AMBIGUOUS_S;
                (!tied || line_of_route(r) == line_of_route(r2))
                    .then(|| (line_of_route(r), trip, a.0))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn station(name: &str) -> &'static Station {
        crate::stations::all()
            .iter()
            .find(|s| s.name == name && (s.lines.iter().any(|l| l == "2") || name.contains("110")))
            .unwrap()
    }

    fn stop(trip: &str, route: &str, id: &str, at: i64) -> TripStop {
        TripStop {
            trip_id: trip.into(),
            route: route.into(),
            stop_id: id.into(),
            arrival: at,
            departure: at + 30,
        }
    }

    #[test]
    fn a_trip_only_matches_at_its_own_lines_platform() {
        // A Q at Union Sq as the rider's L stops there: the Q's platform is near enough to count, its line isn't.
        let t0 = 1_793_000_000;
        let offset = 1_000_000_000_000i64;
        let mut trips = Trips::default();
        trips.on_poll(offset, t0 * 1000, vec![stop("q", "Q", "R20", t0)]);
        let union_sq_l = crate::stations::all()
            .iter()
            .find(|s| s.name == "14 St-Union Sq" && s.lines == ["L"])
            .unwrap();
        assert!(stop_ids(union_sq_l).contains(&"R20"));
        let at = |s: i64| offset + s * 1_000_000_000;
        let stops = [RiderStop {
            stopped_ns: at(0),
            departed_ns: Some(at(30)),
            station: union_sq_l,
        }];
        assert_eq!(trips.route(&stops, at(40)), None);
        trips.on_poll(offset, t0 * 1000, vec![stop("l", "L", "L03", t0)]);
        assert_eq!(trips.route(&stops, at(40)).as_deref(), Some("L"));
    }

    #[test]
    fn a_line_not_at_every_stop_does_not_match() {
        // Tue Nov 3, 6:10 pm: the N/W from Broadway stops at Queensboro Plaza, named at its 7 platform
        // (listed at the same spot), as a 7 stops there too; the feeds no longer list the W at Broadway.
        let t0 = 1_793_000_000;
        let offset = 1_000_000_000_000i64;
        let named = |name: &str, line: &str| {
            crate::stations::all()
                .iter()
                .find(|s| s.name == name && s.lines.iter().any(|l| l == line))
                .unwrap()
        };
        let mut trips = Trips::default();
        trips.on_poll(
            offset,
            t0 * 1000,
            vec![
                stop("7", "7", "718", t0 + 270),
                stop("w", "W", "R09", t0 + 300),
            ],
        );
        let at = |s: i64| offset + s * 1_000_000_000;
        let stops = [
            RiderStop {
                stopped_ns: at(0),
                departed_ns: Some(at(30)),
                station: named("Broadway", "W"),
            },
            RiderStop {
                stopped_ns: at(275),
                departed_ns: Some(at(320)),
                station: named("Queensboro Plaza", "7"),
            },
        ];
        assert_eq!(trips.route(&stops, at(330)).as_deref(), Some("W"));
        // Once the W's stop there is no longer listed either, the 7 still isn't named.
        let mut seven = Trips::default();
        seven.on_poll(offset, t0 * 1000, vec![stop("7", "7", "718", t0 + 270)]);
        assert_eq!(seven.route(&stops, at(330)), None);
        assert_eq!(seven.route(&stops[1..], at(330)).as_deref(), Some("7"));
    }

    #[test]
    fn a_station_passed_rules_out_the_trips_stopping_there() {
        // Tue Nov 3, 6:19 pm: from 57 St-7 Av the W and the Q both list Times Sq near the rider's stop there,
        // the W nearer, but the rider's train passed 49 St, where the W stops and the Q, express, doesn't.
        let t0 = 1_793_747_140;
        let offset = 1_000_000_000_000i64;
        let named = |name: &str, line: &str| {
            crate::stations::all()
                .iter()
                .find(|s| s.name == name && s.lines.iter().any(|l| l == line))
                .unwrap()
        };
        let mut trips = Trips::default();
        trips.on_poll(
            offset,
            t0 * 1000,
            vec![
                stop("w", "W", "R14", t0 - 5),
                stop("w", "W", "R15", t0 + 70),
                stop("w", "W", "R16", t0 + 150),
                stop("q", "Q", "R14", t0 + 10),
                stop("q", "Q", "R16", t0 + 180),
            ],
        );
        let at = |s: i64| offset + s * 1_000_000_000;
        let stops = [
            RiderStop {
                stopped_ns: at(0),
                departed_ns: Some(at(15)),
                station: named("57 St-7 Av", "Q"),
            },
            RiderStop {
                stopped_ns: at(150),
                departed_ns: None,
                station: named("Times Sq-42 St", "Q"),
            },
        ];
        let passed = [named("49 St", "W")];
        assert_eq!(trips.held_route(&stops, at(170)).as_deref(), Some("W"));
        assert_eq!(
            trips
                .held_route_passing(&stops, &passed, at(170))
                .as_deref(),
            Some("Q")
        );
        // The W stops there by the MTA's list even when the feeds no longer list it there; an express variant
        // of a local doesn't, unless the feeds list it at that platform.
        let trip = |route: &str, ids: &[&str]| Trip {
            route: route.into(),
            last_seen: t0,
            stops: ids.iter().map(|id| (id.to_string(), (t0, t0))).collect(),
        };
        assert!(Trips::stops_at(&trip("W", &[]), passed[0]));
        assert!(!Trips::stops_at(&trip("Q", &["R14", "R16"]), passed[0]));
        assert!(!Trips::stops_at(&trip("WX", &[]), passed[0]));
        assert!(Trips::stops_at(&trip("Q", &["R15"]), passed[0]));
    }

    #[test]
    fn a_route_is_kept_past_a_stop_named_early() {
        // Tue Nov 3, 6:05 pm: the W from 30 Av, its stops named a station early by the cell (Broadway at 30 Av,
        // Queensboro Plaza at 39 Av); the feeds settle the W at Queensboro Plaza 1:27 after that stop.
        // An N a station ahead and a W the other way are in the same polls.
        let t0 = 1_793_747_140;
        let offset = 1_000_000_000_000i64;
        let named = |name: &str, line: &str| {
            crate::stations::all()
                .iter()
                .find(|s| s.name == name && s.lines.iter().any(|l| l == line))
                .unwrap()
        };
        let mut trips = Trips::default();
        trips.on_poll(
            offset,
            t0 * 1000,
            vec![
                stop("w", "W", "R04", t0 - 20),
                stop("w", "W", "R05", t0 + 65),
                stop("w", "W", "R08", t0 + 276),
                stop("w", "W", "R09", t0 + 390),
                stop("w", "W", "R11", t0 + 615),
                stop("n", "N", "R09", t0 + 50),
                stop("n", "N", "R11", t0 + 280),
                stop("w-north", "W", "R09", t0 + 220),
                stop("w-north", "W", "R08", t0 + 400),
            ],
        );
        let at = |s: i64| offset + s * 1_000_000_000;
        let stops = [
            RiderStop {
                stopped_ns: at(0),
                departed_ns: Some(at(30)),
                station: named("Broadway", "W"),
            },
            RiderStop {
                stopped_ns: at(303),
                departed_ns: Some(at(331)),
                station: named("Queensboro Plaza", "7"),
            },
        ];
        // No trip matches the newest.
        assert_eq!(trips.route(&stops, at(500)), None);
        assert_eq!(trips.held_route(&stops, at(500)).as_deref(), Some("W"));
        // A newer stop at a station the W doesn't serve rules it out.
        let off_line = [
            stops[0],
            stops[1],
            RiderStop {
                stopped_ns: at(700),
                departed_ns: Some(at(730)),
                station: named("Grand Central-42 St", "7"),
            },
        ];
        assert_eq!(trips.held_route(&off_line, at(740)), None);
        // So does one the W reached a stop before only after the rider left it.
        let behind = [
            stops[0],
            RiderStop {
                stopped_ns: at(150),
                departed_ns: Some(at(170)),
                station: named("Queensboro Plaza", "N"),
            },
        ];
        assert_eq!(trips.held_route(&behind, at(500)), None);
    }

    #[test]
    fn a_line_off_its_usual_stops_matches_where_the_feeds_list_it() {
        // Sat Oct 31, 5:12 pm: the A, running local, from 42 St-Port Authority stops at 50 St, which the MTA
        // lists for the C and E only, then runs express from 59 St to 125 St; the C starts at 59 St, ahead.
        let t0 = 1_793_481_174;
        let offset = 1_000_000_000_000i64;
        let named = |name: &str, line: &str| {
            crate::stations::all()
                .iter()
                .find(|s| s.name == name && s.lines.iter().any(|l| l == line))
                .unwrap()
        };
        let mut trips = Trips::default();
        trips.on_poll(
            offset,
            t0 * 1000,
            vec![
                stop("a", "A", "A27", t0),
                stop("a", "A", "A25", t0 + 85),
                stop("a", "A", "A24", t0 + 350),
                stop("a", "A", "A15", t0 + 770),
                stop("c", "C", "A24", t0 - 2),
                stop("c", "C", "A15", t0 + 770),
            ],
        );
        let at = |s: i64| offset + s * 1_000_000_000;
        let stops = [
            RiderStop {
                stopped_ns: at(70),
                departed_ns: Some(at(100)),
                station: named("50 St", "C"),
            },
            RiderStop {
                stopped_ns: at(734),
                departed_ns: Some(at(810)),
                station: named("125 St", "A"),
            },
        ];
        assert_eq!(trips.route(&stops, at(820)).as_deref(), Some("A"));
        // Without the A's trip, the C, which doesn't list 50 St, still matches 125 St alone but not 50 St.
        let mut c = Trips::default();
        c.on_poll(offset, t0 * 1000, vec![stop("c", "C", "A15", t0 + 770)]);
        assert_eq!(c.route(&stops[1..], at(820)).as_deref(), Some("C"));
        // And the 1's 50 St, a block away, doesn't take an A's stop for its own.
        let one = named("50 St", "1");
        assert!(!own_stop_ids(one).contains(&"A25"));
    }

    #[test]
    fn station_has_its_gtfs_stops() {
        assert_eq!(stop_ids(station("Cathedral Pkwy (110 St)")), ["118"]);
        assert_eq!(line_of_route("7X"), "7");
        assert_eq!(line_of_route("GS"), "GS");
    }

    #[test]
    fn shared_stops_are_told_apart_by_time() {
        // 96 St (120) then 72 St (123) on the 2 and 3, a few minutes apart; the rider stopped with the 2.
        let t0 = 1_793_000_000;
        let mut trips = Trips::default();
        // Unix time t0 is elapsed 1000 s.
        let offset = 1_000_000_000_000i64;
        trips.on_poll(
            offset,
            t0 * 1000,
            vec![
                stop("a", "2", "120", t0),
                stop("a", "2", "123", t0 + 120),
                stop("b", "3", "120", t0 + 240),
                stop("b", "3", "123", t0 + 360),
            ],
        );
        // A later poll no longer lists the stops passed.
        trips.on_poll(offset + 60_000_000_000, (t0 + 60) * 1000, vec![]);
        let at = |s: i64| offset + s * 1_000_000_000;
        let stops = [
            RiderStop {
                stopped_ns: at(5),
                departed_ns: Some(at(35)),
                station: station("96 St"),
            },
            RiderStop {
                stopped_ns: at(125),
                departed_ns: None,
                station: station("72 St"),
            },
        ];
        let s96 = stop_ids(stops[0].station);
        assert!(s96.contains(&"120"), "{s96:?}");
        assert_eq!(trips.route(&stops, at(140)).as_deref(), Some("2"));
        // The 3's times: the 3.
        let later: Vec<RiderStop> = stops
            .iter()
            .map(|s| RiderStop {
                stopped_ns: s.stopped_ns + 240_000_000_000,
                departed_ns: s.departed_ns.map(|d| d + 240_000_000_000),
                ..*s
            })
            .collect();
        assert_eq!(trips.route(&later, at(380)).as_deref(), Some("3"));
        // Between them, nothing matches.
        let between: Vec<RiderStop> = stops
            .iter()
            .map(|s| RiderStop {
                stopped_ns: s.stopped_ns + 120_000_000_000,
                departed_ns: Some(s.stopped_ns + 125_000_000_000),
                ..*s
            })
            .collect();
        assert_eq!(trips.route(&between[..1], at(140)), None);
        assert_eq!(Trips::default().route(&stops, at(140)), None);
    }
}

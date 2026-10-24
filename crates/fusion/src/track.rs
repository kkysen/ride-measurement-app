//! The subway's track between stations, from `python/experiments/subway_track.py` (the MTA's GTFS shapes, each with
//! its stops in order): stops at known stations are joined along it, so the path underground follows the track
//! instead of the dead reckoning (a median 429 to 2066 m off at stations, `crates/cli/examples/stop_positions.rs`).
//! Each sample between two stops is placed along the track by the share of the distance its speeds cover,
//! so the speeds still set when the train was where.

use std::sync::OnceLock;

use crate::fusion::{SmoothedSpeed, StopRecord};
use crate::live_speed::SUBWAY_MAX_SPEED_MPS;
use crate::position::PositionEstimate;
use crate::stations::{Station, meters};
use crate::trips::{line_of_route, stop_ids};

const TRACK: &[u8] = include_bytes!("../data/subway_track.bin");

struct Shape {
    route: String,
    points: Vec<(f64, f64)>,
    /// Meters along the shape at each point.
    along: Vec<f64>,
    /// Parent stop IDs and their point's index.
    stops: Vec<(String, usize)>,
}

fn shapes() -> &'static [Shape] {
    static SHAPES: OnceLock<Vec<Shape>> = OnceLock::new();
    SHAPES.get_or_init(|| {
        let mut b = TRACK;
        let mut out = Vec::new();
        while !b.is_empty() {
            let mut take = |n: usize| {
                let (head, rest) = b.split_at(n);
                b = rest;
                head
            };
            let n = take(1)[0] as usize;
            let route = String::from_utf8_lossy(take(n)).into_owned();
            let n = u16::from_le_bytes(take(2).try_into().unwrap()) as usize;
            let points: Vec<(f64, f64)> = (0..n)
                .map(|_| {
                    let p = take(8);
                    let lat = i32::from_le_bytes(p[0..4].try_into().unwrap());
                    let lon = i32::from_le_bytes(p[4..8].try_into().unwrap());
                    (f64::from(lat) * 1e-5, f64::from(lon) * 1e-5)
                })
                .collect();
            let mut along = vec![0.0];
            for w in points.windows(2) {
                along.push(along.last().unwrap() + meters(w[0], w[1]));
            }
            let n = u16::from_le_bytes(take(2).try_into().unwrap()) as usize;
            let stops = (0..n)
                .map(|_| {
                    let k = take(1)[0] as usize;
                    let id = String::from_utf8_lossy(take(k)).into_owned();
                    let i = u16::from_le_bytes(take(2).try_into().unwrap()) as usize;
                    (id, i)
                })
                .collect();
            out.push(Shape {
                route,
                points,
                along,
                stops,
            });
        }
        out
    })
}

/// A stretch of track from one station to another.
#[derive(Clone, Debug, PartialEq)]
pub struct Track {
    pub points: Vec<(f64, f64)>,
    /// Meters along the stretch at each point.
    pub along: Vec<f64>,
}

impl Track {
    pub fn length_m(&self) -> f64 {
        *self.along.last().unwrap_or(&0.0)
    }

    /// The point `m` meters along, clamped to the ends.
    pub fn at(&self, m: f64) -> (f64, f64) {
        let i = self.along.partition_point(|&a| a <= m);
        if i == 0 {
            return self.points[0];
        }
        if i == self.points.len() {
            return *self.points.last().unwrap();
        }
        let (a, b) = (self.along[i - 1], self.along[i]);
        let f = if b > a { (m - a) / (b - a) } else { 0.0 };
        let (p, q) = (self.points[i - 1], self.points[i]);
        (p.0 + (q.0 - p.0) * f, p.1 + (q.1 - p.1) * f)
    }
}

/// The track from station `a` to `b`: along a shape that stops at both, preferring a route that serves both
/// as named, then the shortest. None when no shape joins them.
pub fn between(a: &Station, b: &Station) -> Option<Track> {
    let (ids_a, ids_b) = (stop_ids(a), stop_ids(b));
    let serves = |route: &str| {
        let line = line_of_route(route);
        a.lines.contains(&line) && b.lines.contains(&line)
    };
    let mut best: Option<(bool, f64, &Shape, usize, usize)> = None;
    for shape in shapes() {
        let find = |ids: &[&str]| {
            shape
                .stops
                .iter()
                .find(|(id, _)| ids.contains(&id.as_str()))
                .map(|&(_, i)| i)
        };
        let (Some(i), Some(j)) = (find(&ids_a), find(&ids_b)) else {
            continue;
        };
        if i == j {
            continue;
        }
        let key = (
            !serves(&shape.route),
            (shape.along[j] - shape.along[i]).abs(),
        );
        if best.is_none_or(|(s, l, ..)| key < (s, l)) {
            best = Some((key.0, key.1, shape, i, j));
        }
    }
    let (_, _, shape, i, j) = best?;
    let mut points: Vec<(f64, f64)> = if i < j {
        shape.points[i..=j].to_vec()
    } else {
        shape.points[j..=i].iter().rev().copied().collect()
    };
    // The ends at the stations' own points, which the shapes pass within a few meters of.
    points[0] = (a.lat, a.lon);
    *points.last_mut().unwrap() = (b.lat, b.lon);
    let mut along = vec![0.0];
    for w in points.windows(2) {
        along.push(along.last().unwrap() + meters(w[0], w[1]));
    }
    Some(Track { points, along })
}

/// What [`join`] did.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Joined {
    pub pairs: usize,
    /// Pairs of stops at stations no shape joins; farther apart along it than the subway's top speed covers
    /// in the time between them (a station misnamed, as a cell a station off); or with a good fix between them.
    pub no_track: usize,
    pub too_far: usize,
    pub with_fix: usize,
}

/// Whether a stop anchors the path: underground at a named station, not a hold between stations.
fn anchor(r: &StopRecord) -> bool {
    r.in_subway && r.station.is_some() && !r.between_stations
}

/// Moves `positions` underground onto the track: from each stop at a station to the next on the same train,
/// along the track between them by the share of the distance the smoothed `speeds` cover, and through each joined stop's
/// dwell at its station. `good_fix_ns` are the times of fixes good enough that the path there is GNSS's own;
/// a pair with one between is left alone. `skip` leaves one stop out as an anchor (for scoring).
pub fn join(
    positions: &mut [PositionEstimate],
    speeds: &[SmoothedSpeed],
    records: &[StopRecord],
    good_fix_ns: &[i64],
    skip: Option<usize>,
) -> Joined {
    let mut out = Joined::default();
    let mut last: Option<&StopRecord> = None;
    for (k, r) in records.iter().enumerate() {
        if r.on_platform {
            last = None;
        }
        let is_anchor = anchor(r) && skip != Some(k);
        if is_anchor && let Some(a) = last {
            let (sa, sb) = (a.station.unwrap(), r.station.unwrap());
            let from = a.departed_ns.unwrap_or(a.stopped_ns);
            let to = r.stopped_ns;
            out.pairs += 1;
            if sa.name != sb.name && to > from {
                match between(sa, sb) {
                    None => out.no_track += 1,
                    Some(_) if good_fix_ns.iter().any(|&t| t > from && t < to) => out.with_fix += 1,
                    Some(track) => {
                        if !place(positions, speeds, &track, from, to) {
                            out.too_far += 1;
                        }
                    }
                }
            }
        }
        if is_anchor {
            let s = r.station.unwrap();
            let end = r.departed_ns.or(r.alighted_ns).unwrap_or(r.stopped_ns);
            for p in positions.iter_mut() {
                if p.elapsed_realtime_ns >= r.stopped_ns && p.elapsed_realtime_ns <= end {
                    (p.lat, p.lon) = (s.lat, s.lon);
                }
            }
            last = Some(r);
        }
        if r.alighted_ns.is_some() {
            last = None;
        }
    }
    out
}

/// Places the samples from `from` to `to` along `track`, by the share of the distance `speeds` cover to each
/// (by time when they cover none): the speeds' shape, not their scale, which is often far off underground
/// (a third of the track on the L, `20261023-193658`). False (nothing moved) when the track is too long
/// for the time (see [`Joined::too_far`]).
fn place(
    positions: &mut [PositionEstimate],
    speeds: &[SmoothedSpeed],
    track: &Track,
    from: i64,
    to: i64,
) -> bool {
    let length = track.length_m();
    if length > SUBWAY_MAX_SPEED_MPS * (to - from) as f64 * 1e-9 {
        return false;
    }
    let total = covered_m(speeds, from, to);
    for p in positions
        .iter_mut()
        .filter(|p| p.elapsed_realtime_ns >= from && p.elapsed_realtime_ns <= to)
    {
        let t = p.elapsed_realtime_ns;
        let share = if total > 0.0 {
            covered_m(speeds, from, t) / total
        } else {
            (t - from) as f64 / (to - from) as f64
        };
        (p.lat, p.lon) = track.at(share * length);
    }
    true
}

/// The distance (m) the smoothed `speeds` cover from `from` to `to`, by the trapezoid rule.
pub fn covered_m(speeds: &[SmoothedSpeed], from: i64, to: i64) -> f64 {
    let lo = speeds.partition_point(|s| s.elapsed_realtime_ns < from);
    let hi = speeds.partition_point(|s| s.elapsed_realtime_ns <= to);
    speeds[lo..hi]
        .windows(2)
        .map(|w| {
            let dt = (w[1].elapsed_realtime_ns - w[0].elapsed_realtime_ns) as f64 * 1e-9;
            (w[0].speed_mps + w[1].speed_mps) / 2.0 * dt
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stations::all;

    fn station(name: &str, line: &str) -> &'static Station {
        all()
            .iter()
            .find(|s| s.name == name && s.lines.iter().any(|l| l == line))
            .unwrap()
    }

    #[test]
    fn the_l_under_the_east_river() {
        // Bedford Av to 1 Av: about 2.6 km of track under the river, longer than the straight line.
        let t = between(station("Bedford Av", "L"), station("1 Av", "L")).unwrap();
        let straight = meters(t.points[0], *t.points.last().unwrap());
        assert!(t.length_m() > straight, "{} {straight}", t.length_m());
        assert!((2300.0..2900.0).contains(&t.length_m()), "{}", t.length_m());
        // Halfway is under the river, not at either end.
        let mid = t.at(t.length_m() / 2.0);
        assert!(meters(mid, t.points[0]) > 500.0 && meters(mid, *t.points.last().unwrap()) > 500.0);
    }

    #[test]
    fn an_express_run_joins_stations_it_skips_between() {
        // 96 St to 72 St on the 1/2/3: two local stops between, on the same track.
        let t = between(station("96 St", "1"), station("72 St", "1")).unwrap();
        assert!((1700.0..2600.0).contains(&t.length_m()), "{}", t.length_m());
    }
}

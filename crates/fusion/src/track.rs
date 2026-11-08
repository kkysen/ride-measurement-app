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

    /// The same track, the other way.
    pub fn reversed(&self) -> Track {
        let points: Vec<(f64, f64)> = self.points.iter().rev().copied().collect();
        let length = self.length_m();
        let along = self.along.iter().rev().map(|a| length - a).collect();
        Track { points, along }
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

/// A shape's points from index `i` to `j` (either way), as a [`Track`].
fn slice(shape: &Shape, i: usize, j: usize) -> Track {
    let points: Vec<(f64, f64)> = if i <= j {
        shape.points[i..=j].to_vec()
    } else {
        shape.points[j..=i].iter().rev().copied().collect()
    };
    let mut along = vec![0.0];
    for w in points.windows(2) {
        along.push(along.last().unwrap() + meters(w[0], w[1]));
    }
    Track { points, along }
}

/// A fix is on a shape's track within this (m): where the path leaves GNSS for the track.
const ON_SHAPE_M: f64 = 60.0;

/// The track from where a train was at a good fix (`p`) to the station it next stopped at,
/// along a shape of one of `lines` (any, if none) that stops there and passes within [`ON_SHAPE_M`] of `p`;
/// the nearest such. From the fix's nearest point on it, so the path meets GNSS's.
/// For entering a tunnel, as from Queensboro Plaza's elevated platform to Lexington Av/59 St.
pub fn to_station(p: (f64, f64), station: &Station, lines: &[String]) -> Option<Track> {
    let ids = stop_ids(station);
    // The fix's distance, the shape, its segment there, the station's point, and the fix's point on it.
    #[allow(clippy::type_complexity)]
    let mut best: Option<(f64, &Shape, usize, usize, (f64, f64))> = None;
    for shape in shapes() {
        if !lines.is_empty() && !lines.contains(&line_of_route(&shape.route)) {
            continue;
        }
        let Some(&(_, k)) = shape
            .stops
            .iter()
            .find(|(id, _)| ids.contains(&id.as_str()))
        else {
            continue;
        };
        let Some((d, i, q)) = nearest_on(shape, p) else {
            continue;
        };
        if d <= ON_SHAPE_M && best.is_none_or(|(b, ..)| d < b) {
            best = Some((d, shape, i, k, q));
        }
    }
    let (_, shape, i, k, q) = best?;
    // From the fix's point on segment `i` (points `i` to `i + 1`) to the station's point `k`.
    let mut track = if i < k {
        slice(shape, i + 1, k)
    } else {
        slice(shape, i, k)
    };
    track.points.insert(0, q);
    *track.points.last_mut().unwrap() = (station.lat, station.lon);
    track.along = vec![0.0];
    for w in track.points.windows(2) {
        track
            .along
            .push(track.along.last().unwrap() + meters(w[0], w[1]));
    }
    Some(track)
}

/// The point of a shape's track nearest `p`: its distance (m), its segment's first point, and the point.
fn nearest_on(shape: &Shape, p: (f64, f64)) -> Option<(f64, usize, (f64, f64))> {
    let m_lat = 111_195.0;
    let m_lon = m_lat * p.0.to_radians().cos();
    let mut best: Option<(f64, usize, (f64, f64))> = None;
    for (i, w) in shape.points.windows(2).enumerate() {
        // In meters about `p`, flat over a segment's length.
        let (ax, ay) = ((w[0].1 - p.1) * m_lon, (w[0].0 - p.0) * m_lat);
        let (bx, by) = ((w[1].1 - p.1) * m_lon, (w[1].0 - p.0) * m_lat);
        let (dx, dy) = (bx - ax, by - ay);
        let len2 = dx * dx + dy * dy;
        let f = if len2 > 0.0 {
            (-(ax * dx + ay * dy) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (x, y) = (ax + dx * f, ay + dy * f);
        let d = x.hypot(y);
        if best.is_none_or(|(b, ..)| d < b) {
            let q = (
                w[0].0 + (w[1].0 - w[0].0) * f,
                w[0].1 + (w[1].1 - w[0].1) * f,
            );
            best = Some((d, i, q));
        }
    }
    best
}

/// How many stations on [`ahead`] goes.
const AHEAD_STOPS: usize = 4;

/// The track from a station on through the next [`AHEAD_STOPS`] a shape of one of `lines` (any, if none) stops at,
/// away from `prev` where the shape stops there too: where a train leaving `from` is headed. None if no shape
/// knows the way, or the shapes disagree (no `prev`, or lines going different ways): the way isn't known.
pub fn ahead(from: &Station, prev: Option<&Station>, lines: &[String]) -> Option<Track> {
    let ids_from = stop_ids(from);
    let ids_prev = prev.map(stop_ids)?;
    // The shape, the station's point, the farthest stop's point, and the next station's position.
    let mut found: Option<(&Shape, usize, usize, (f64, f64))> = None;
    for shape in shapes() {
        if !lines.is_empty() && !lines.contains(&line_of_route(&shape.route)) {
            continue;
        }
        let find = |ids: &[&str]| {
            shape
                .stops
                .iter()
                .position(|(id, _)| ids.contains(&id.as_str()))
        };
        let (Some(k), Some(p)) = (find(&ids_from), find(&ids_prev)) else {
            continue;
        };
        let j = if p < k {
            k.checked_add(1)
        } else {
            k.checked_sub(1)
        };
        let Some(&(_, i)) = j.and_then(|j| shape.stops.get(j)) else {
            continue;
        };
        // On past the next station, up to [`AHEAD_STOPS`] (or the line's end): a train can pass one without a
        // stop found there (an express, a closed station, a stop missed), and is placed by distance.
        let far = if p < k {
            (k + AHEAD_STOPS).min(shape.stops.len() - 1)
        } else {
            k.saturating_sub(AHEAD_STOPS)
        };
        let to = shape.points[i];
        match found {
            // Another shape to the same next station (within a platform's length): the same way.
            Some((.., q)) if meters(q, to) <= 200.0 => {}
            Some(_) => return None,
            None => found = Some((shape, shape.stops[k].1, shape.stops[far].1, to)),
        }
    }
    let (shape, k, i, _) = found?;
    let mut track = slice(shape, k, i);
    track.points[0] = (from.lat, from.lon);
    Some(track)
}

/// The shortest track from `from` to the next station a shape of one of `lines` (any, if none) stops at,
/// away from `prev` where that shape stops there too (else either way): what a train leaving `from`
/// must cover before it can stand at a station again.
pub fn next_station_m(from: &Station, prev: Option<&Station>, lines: &[String]) -> Option<f64> {
    let ids_from = stop_ids(from);
    let ids_prev = prev.map(stop_ids).unwrap_or_default();
    let mut best: Option<f64> = None;
    for shape in shapes() {
        if !lines.is_empty() && !lines.contains(&line_of_route(&shape.route)) {
            continue;
        }
        let find = |ids: &[&str]| {
            shape
                .stops
                .iter()
                .position(|(id, _)| ids.contains(&id.as_str()))
        };
        let Some(k) = find(&ids_from) else {
            continue;
        };
        let next: &[Option<usize>] = match find(&ids_prev) {
            Some(p) if p < k => &[k.checked_add(1)],
            Some(p) if p > k => &[k.checked_sub(1)],
            _ => &[k.checked_add(1), k.checked_sub(1)],
        };
        let here = shape.along[shape.stops[k].1];
        for &j in next {
            if let Some(&(_, i)) = j.and_then(|j| shape.stops.get(j)) {
                let d = (shape.along[i] - here).abs();
                best = Some(best.map_or(d, |b| b.min(d)));
            }
        }
    }
    best
}

/// The least time (s) a train takes over `length_m` of track from a stand to a stand (see [`fastest_s`]).
pub fn least_s(length_m: f64) -> f64 {
    fastest_s(length_m)
}

/// What [`join`] did.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Joined {
    pub pairs: usize,
    /// Pairs of stops at stations no shape joins; farther apart along it than a train goes in the time between
    /// them (see [`fastest_s`]: a station misnamed, as a cell a station off); or with a good fix between them.
    pub no_track: usize,
    pub too_far: usize,
    pub with_fix: usize,
}

/// The subway's highest rate of acceleration and braking (m/s^2): 3 mph/s.
const SUBWAY_MAX_ACCEL_MPS2: f64 = 3.0 * 0.44704;

/// The least time (s) a train takes from a stop to a stop `length` (m) along: speeding up and braking at
/// [`SUBWAY_MAX_ACCEL_MPS2`], at most at [`SUBWAY_MAX_SPEED_MPS`] between.
fn fastest_s(length: f64) -> f64 {
    let (a, v) = (SUBWAY_MAX_ACCEL_MPS2, SUBWAY_MAX_SPEED_MPS);
    if length < v * v / a {
        2.0 * (length / a).sqrt()
    } else {
        length / v + v / a
    }
}

/// Whether a stop anchors the path: underground at a named station, not a hold between stations.
pub(crate) fn anchor(r: &StopRecord) -> bool {
    r.in_subway && r.station.is_some() && !r.between_stations
}

/// A train's run between two stops at stations, along the track.
#[derive(Clone, Debug, PartialEq)]
pub struct Leg {
    /// The departure from the first stop (its stop, if no departure was found) and the stop at the second.
    pub from_ns: i64,
    pub to_ns: i64,
    pub track: Track,
}

/// The legs between consecutive stops at stations on the same train (no walk off or platform wait between),
/// with no good fix between them (`good_fix_ns`: where the path is GNSS's own), a shape joining the stations,
/// and a track a train can cover in the time (see [`fastest_s`]). `skip` leaves one stop out (for scoring).
pub fn legs(
    records: &[StopRecord],
    good_fix_ns: &[i64],
    skip: Option<usize>,
) -> (Vec<Leg>, Joined) {
    legs_by(records, good_fix_ns, skip, between)
}

/// [`legs`], with the track between two stations from `between` (see [`between`]): one that remembers them.
pub(crate) fn legs_by(
    records: &[StopRecord],
    good_fix_ns: &[i64],
    skip: Option<usize>,
    mut between: impl FnMut(&'static Station, &'static Station) -> Option<Track>,
) -> (Vec<Leg>, Joined) {
    let mut out = Vec::new();
    let mut stats = Joined::default();
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
            stats.pairs += 1;
            if sa.name != sb.name && to > from {
                match between(sa, sb) {
                    None => stats.no_track += 1,
                    Some(_) if good_fix_ns.iter().any(|&t| t > from && t < to) => {
                        stats.with_fix += 1
                    }
                    Some(track) if (to - from) as f64 * 1e-9 < fastest_s(track.length_m()) => {
                        stats.too_far += 1
                    }
                    Some(track) => out.push(Leg {
                        from_ns: from,
                        to_ns: to,
                        track,
                    }),
                }
            }
        }
        if is_anchor {
            last = Some(r);
        }
        if r.alighted_ns.is_some() {
            last = None;
        }
    }
    (out, stats)
}

/// Moves `positions` underground onto the track: along each of the [`legs`] by the share of the distance the
/// smoothed `speeds` cover (their shape, not their scale, which is often far off underground: a third of the
/// track on the L, `20261023-193658`; by time when they cover none), and through each stop's dwell at its
/// station. Then shifts the rest of the path so it meets the joined points and the good fixes next to it
/// (see [`blend`]): no jump where the joined path hands over to dead reckoning.
pub fn join(
    positions: &mut [PositionEstimate],
    speeds: &[SmoothedSpeed],
    records: &[StopRecord],
    good_fix_ns: &[i64],
    skip: Option<usize>,
) -> Joined {
    let before: Vec<(f64, f64)> = positions.iter().map(|p| (p.lat, p.lon)).collect();
    let (stats, placed) = place(positions, speeds, records, good_fix_ns, skip);
    blend(positions, &before, &placed, good_fix_ns);
    stats
}

/// [`join`] without [`blend`]: only the samples it places on the track or at a station move,
/// and which ones those are.
pub fn place(
    positions: &mut [PositionEstimate],
    speeds: &[SmoothedSpeed],
    records: &[StopRecord],
    good_fix_ns: &[i64],
    skip: Option<usize>,
) -> (Joined, Vec<bool>) {
    let mut placed = vec![false; positions.len()];
    let (legs, stats) = legs(records, good_fix_ns, skip);
    for (k, r) in records.iter().enumerate() {
        if !anchor(r) || skip == Some(k) {
            continue;
        }
        let s = r.station.unwrap();
        let end = r.departed_ns.or(r.alighted_ns).unwrap_or(r.stopped_ns);
        for (p, placed) in positions.iter_mut().zip(&mut placed) {
            if p.elapsed_realtime_ns >= r.stopped_ns && p.elapsed_realtime_ns <= end {
                (p.lat, p.lon) = (s.lat, s.lon);
                *placed = true;
            }
        }
    }
    for leg in &legs {
        let along = Along::new(leg, speeds);
        for (p, placed) in positions.iter_mut().zip(&mut placed).filter(|(p, _)| {
            p.elapsed_realtime_ns >= leg.from_ns && p.elapsed_realtime_ns <= leg.to_ns
        }) {
            *placed = true;
            (p.lat, p.lon) = along.at(p.elapsed_realtime_ns);
        }
    }
    (stats, placed)
}

/// Where a train is along a [`Leg`]'s track at each time in it: by the share of the leg's distance the
/// smoothed speeds cover by then (by time, if they cover none). Each speed's distance is summed once,
/// so placing every sample of a leg costs its samples, not their square.
pub struct Along<'a> {
    leg: &'a Leg,
    /// The leg's speeds, and the distance (m) they cover from the leg's start to each.
    speeds: &'a [SmoothedSpeed],
    covered: Vec<f64>,
}

impl<'a> Along<'a> {
    pub fn new(leg: &'a Leg, speeds: &'a [SmoothedSpeed]) -> Self {
        let lo = speeds.partition_point(|s| s.elapsed_realtime_ns < leg.from_ns);
        let hi = speeds.partition_point(|s| s.elapsed_realtime_ns <= leg.to_ns);
        let speeds = &speeds[lo..hi];
        let mut covered = Vec::with_capacity(speeds.len());
        let mut m = 0.0;
        for (i, s) in speeds.iter().enumerate() {
            if i > 0 {
                let p = &speeds[i - 1];
                let dt = (s.elapsed_realtime_ns - p.elapsed_realtime_ns) as f64 * 1e-9;
                m += (p.speed_mps + s.speed_mps) / 2.0 * dt;
            }
            covered.push(m);
        }
        Along {
            leg,
            speeds,
            covered,
        }
    }

    /// The point on the track at `t_ns`, within the leg.
    pub fn at(&self, t_ns: i64) -> (f64, f64) {
        let total = self.covered.last().copied().unwrap_or(0.0);
        let share = if total > 0.0 {
            // As `covered_m(speeds, from, t)`: the speeds at or before `t`.
            let k = self
                .speeds
                .partition_point(|s| s.elapsed_realtime_ns <= t_ns);
            k.checked_sub(1).map_or(0.0, |k| self.covered[k]) / total
        } else {
            (t_ns - self.leg.from_ns) as f64 / (self.leg.to_ns - self.leg.from_ns) as f64
        };
        self.leg.track.at(share * self.leg.track.length_m())
    }

    /// The leg's speeds, each with its point on the track.
    pub fn path(&self) -> impl Iterator<Item = (&'a SmoothedSpeed, (f64, f64))> + '_ {
        self.speeds
            .iter()
            .map(|s| (s, self.at(s.elapsed_realtime_ns)))
    }
}

/// Shifts each stretch of `positions` that [`place`] left alone by an offset blended, over time,
/// from the shift of the pinned sample before it to that of the one after:
/// the shift [`place`] made to a placed sample, none at a good fix (the path there is GNSS's own).
/// A stretch pinned on one side only (the ride's start or end) takes that side's shift whole.
/// `before` is each position before [`place`] moved it.
pub fn blend(
    positions: &mut [PositionEstimate],
    before: &[(f64, f64)],
    placed: &[bool],
    good_fix_ns: &[i64],
) {
    if !placed.iter().any(|&p| p) {
        return;
    }
    let mut fixes = good_fix_ns.to_vec();
    fixes.sort_unstable();
    let shift = |i: usize, p: &PositionEstimate| -> Option<(f64, f64)> {
        if placed[i] {
            Some((p.lat - before[i].0, p.lon - before[i].1))
        } else if fixes.binary_search(&p.elapsed_realtime_ns).is_ok() {
            Some((0.0, 0.0))
        } else {
            None
        }
    };
    let pins: Vec<(usize, (f64, f64))> = positions
        .iter()
        .enumerate()
        .filter_map(|(i, p)| Some((i, shift(i, p)?)))
        .collect();
    let mut next = 0;
    let mut prev: Option<(usize, (f64, f64))> = None;
    for i in 0..positions.len() {
        if next < pins.len() && pins[next].0 == i {
            prev = Some(pins[next]);
            next += 1;
            continue;
        }
        let after = pins.get(next).copied();
        let (dlat, dlon) = match (prev, after) {
            (Some((_, a)), None) => a,
            (None, Some((_, b))) => b,
            (Some((ia, a)), Some((ib, b))) => {
                let ta = positions[ia].elapsed_realtime_ns;
                let tb = positions[ib].elapsed_realtime_ns;
                let f = if tb > ta {
                    (positions[i].elapsed_realtime_ns - ta) as f64 / (tb - ta) as f64
                } else {
                    0.5
                };
                (a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f)
            }
            (None, None) => (0.0, 0.0),
        };
        positions[i].lat += dlat;
        positions[i].lon += dlon;
    }
}

/// Holds the smoothed `speeds` over each of the [`legs`] to its track's length: scaled to cover it
/// (speed and acceleration, as [`crate::fusion`]'s gap bound does), at most to the subway's top speed;
/// spread evenly over the leg if they cover nothing. Underground they cover a median 0.46 of the track
/// (`crates/cli/examples/legs.rs`, 90 legs on 15 rides; a tenth under 0.14).
/// Only legs the speeds cover whole: live, the smoothing window can start partway through one.
pub fn hold_to_track(speeds: &mut [SmoothedSpeed], legs: &[Leg]) {
    let (Some(first), Some(last)) = (speeds.first(), speeds.last()) else {
        return;
    };
    let (first, last) = (first.elapsed_realtime_ns, last.elapsed_realtime_ns);
    for leg in legs {
        let (from, to) = (leg.from_ns, leg.to_ns);
        if from < first || to > last {
            continue;
        }
        let length = leg.track.length_m();
        let total = covered_m(speeds, from, to);
        let lo = speeds.partition_point(|s| s.elapsed_realtime_ns < from);
        let hi = speeds.partition_point(|s| s.elapsed_realtime_ns <= to);
        for s in &mut speeds[lo..hi] {
            if total > 0.0 {
                let k = length / total;
                s.speed_mps *= k;
                s.accel_mps2 *= k;
            } else {
                s.speed_mps = length / ((to - from) as f64 * 1e-9);
            }
            s.speed_mps = s.speed_mps.min(SUBWAY_MAX_SPEED_MPS);
        }
    }
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

    fn speeds(from_s: i64, to_s: i64, v: f64) -> Vec<SmoothedSpeed> {
        (from_s..=to_s)
            .map(|t| SmoothedSpeed {
                elapsed_realtime_ns: t * 1_000_000_000,
                speed_mps: v,
                sigma_mps: 1.0,
                forward_mps: v,
                backward_mps: v,
                accel_mps2: 0.0,
            })
            .collect()
    }

    #[test]
    fn a_leg_is_placed_by_the_share_of_its_speeds_distance() {
        let track = between(station("Bedford Av", "L"), station("1 Av", "L")).unwrap();
        let leg = Leg {
            from_ns: 100_000_000_000,
            to_ns: 300_000_000_000,
            track,
        };
        // Faster in the leg's second half: a quarter of the way at half time.
        let mut v = speeds(0, 400, 2.0);
        v.iter_mut()
            .filter(|s| s.elapsed_realtime_ns > 200_000_000_000)
            .for_each(|s| s.speed_mps = 6.0);
        let along = Along::new(&leg, &v);
        for t in [100, 150, 200, 250, 300].map(|s: i64| s * 1_000_000_000) {
            let share = covered_m(&v, leg.from_ns, t) / covered_m(&v, leg.from_ns, leg.to_ns);
            let want = leg.track.at(share * leg.track.length_m());
            assert!(meters(along.at(t), want) < 0.01, "{t}");
        }
        let quarter = leg.track.at(leg.track.length_m() / 4.0);
        assert!(meters(along.at(200_000_000_000), quarter) < 20.0);
        assert_eq!(along.path().count(), 201);
    }

    #[test]
    fn speeds_are_held_to_the_track_over_whole_legs_only() {
        let track = between(station("Bedford Av", "L"), station("1 Av", "L")).unwrap();
        let length = track.length_m();
        let leg = Leg {
            from_ns: 100_000_000_000,
            to_ns: 300_000_000_000,
            track,
        };
        // The whole leg: 5 m/s over 200 s is 1000 m, scaled up to the track's length.
        let mut whole = speeds(0, 400, 5.0);
        hold_to_track(&mut whole, std::slice::from_ref(&leg));
        let held = covered_m(&whole, leg.from_ns, leg.to_ns);
        assert!((held - length).abs() < 1.0, "{held} {length}");
        assert_eq!(whole[50].speed_mps, 5.0);
        // A live window starting partway through the leg leaves it alone.
        let mut window = speeds(200, 400, 5.0);
        hold_to_track(&mut window, &[leg]);
        assert!(window.iter().all(|s| s.speed_mps == 5.0));
    }

    #[test]
    fn a_train_needs_time_to_speed_up() {
        // 528 m in 22 s would average 52 mph: a station misnamed (`20261024-003357`).
        assert!(fastest_s(528.0) > 35.0);
        // 2.2 km at 55 mph with 18 s each to speed up and brake.
        assert!((100.0..115.0).contains(&fastest_s(2217.0)));
    }

    #[test]
    fn an_express_run_joins_stations_it_skips_between() {
        // 96 St to 72 St on the 1/2/3: two local stops between, on the same track.
        let t = between(station("96 St", "1"), station("72 St", "1")).unwrap();
        assert!((1700.0..2600.0).contains(&t.length_m()), "{}", t.length_m());
    }

    #[test]
    fn the_path_is_shifted_to_meet_what_was_placed() {
        let pos = |t: i64, lat: f64| PositionEstimate {
            elapsed_realtime_ns: t * 1_000_000_000,
            lat,
            lon: 0.0,
            sigma_m: 1.0,
            velocity: [0.0; 2],
        };
        // A good fix at 0 s, dead reckoning from 1 to 3 s, a sample placed 0.04 degrees north at 4 s.
        let before = [0.0, 0.0, 0.0, 0.0, 0.0];
        let mut positions: Vec<_> = (0..5).map(|t| pos(t, before[t as usize])).collect();
        positions[4].lat = 0.04;
        let placed = [false, false, false, false, true];
        let before: Vec<_> = before.iter().map(|&l| (l, 0.0)).collect();
        blend(&mut positions, &before, &placed, &[0]);
        let lats: Vec<f64> = positions.iter().map(|p| (p.lat * 100.0).round()).collect();
        assert_eq!(lats, [0.0, 1.0, 2.0, 3.0, 4.0]);
    }
}

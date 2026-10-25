//! Going up or down on foot or riding: stairs, and elevators or escalators, from the barometer's height and the
//! step rhythm (`python/experiments/elevator.py`). The rider's home elevator and stairs both went about 12 m in
//! 25 s; the stairs had steps throughout, the elevator none, but walking just before it. The barometer can't tell
//! an elevator from an escalator (both about 0.5 m/s, both walked onto), so those are one kind.
//! A height change with no walking around it is a train on a grade or a tunnel's pressure wave, so it's left out.

/// Meters of height per hPa near sea level.
const M_PER_HPA: f64 = 8.3;
/// A move is at least this height change (m) over [`SPAN_S`]: 0.16 m/s, slower than stairs or an escalator, and
/// several times the barometer's noise (about 0.6 m).
const MIN_CHANGE_M: f64 = 4.0;
const SPAN_S: i64 = 25;
const MAX_CHANGE_M: f64 = 60.0;
/// Steps in at least this share of a move's step windows make it stairs.
const STAIRS_SHARE: f64 = 0.5;
/// Riding up or down (no steps) needs walking within this (s) before it: walked onto.
const WALKED_BEFORE_S: f64 = 60.0;

/// Heights (m, from the first) once a second: each second's median pressure, so a single odd reading can't move it.
#[derive(Clone, Debug, Default)]
pub struct Altimeter {
    second: Option<i64>,
    readings: Vec<f64>,
    first_hpa: Option<f64>,
    heights: Vec<(i64, f64)>,
}

impl Altimeter {
    /// A pressure reading (hPa).
    pub fn on_pressure(&mut self, t_ns: i64, hpa: f64) {
        let second = t_ns.div_euclid(1_000_000_000);
        if self.second.is_some_and(|s| s != second) {
            self.flush();
        }
        self.second = Some(second);
        self.readings.push(hpa);
    }

    fn flush(&mut self) {
        let (Some(second), false) = (self.second, self.readings.is_empty()) else {
            return;
        };
        self.readings.sort_by(f64::total_cmp);
        let median = self.readings[self.readings.len() / 2];
        let first = *self.first_hpa.get_or_insert(median);
        self.heights
            .push((second * 1_000_000_000, -M_PER_HPA * (median - first)));
        self.readings.clear();
    }

    /// The heights so far, one per second (ns, m).
    pub fn heights(&self) -> &[(i64, f64)] {
        &self.heights
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Stairs,
    /// An elevator or an escalator, stood on.
    Riding,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Move {
    pub start_ns: i64,
    pub end_ns: i64,
    /// Height change (m), positive going up.
    pub change_m: f64,
    pub kind: Kind,
}

/// The moves in `heights` (see [`Altimeter::heights`]), given each step window's end and whether it had steps.
/// A second is moving when the height changed by [`MIN_CHANGE_M`] over the [`SPAN_S`] centered on it;
/// each run of them, widened by the half span, is a move.
pub fn moves(heights: &[(i64, f64)], step_windows: &[(i64, bool)], window_s: f64) -> Vec<Move> {
    let half = (SPAN_S / 2) as usize;
    let mut runs: Vec<(usize, usize)> = Vec::new();
    for i in half..heights.len().saturating_sub(half) {
        if !moving(heights, i) {
            continue;
        }
        match runs.last_mut() {
            Some(r) if r.1 + 1 == i => r.1 = i,
            _ => runs.push((i, i)),
        }
    }
    let window_ns = (window_s * 1e9) as i64;
    let found: Vec<Move> = runs
        .into_iter()
        .filter_map(|(i, j)| {
            let (start_ns, from) = heights[i - half];
            let (end_ns, to) = heights[j + half];
            // Step windows are in time order: those ending from the move's start on, then those within it.
            let first = step_windows.partition_point(|&(at, _)| at < start_ns);
            let during: Vec<bool> = step_windows[first..]
                .iter()
                .take_while(|&&(at, _)| at <= end_ns)
                .filter(|&&(at, _)| at - window_ns >= start_ns)
                .map(|&(_, steps)| steps)
                .collect();
            let stepping = !during.is_empty()
                && during.iter().filter(|&&s| s).count() as f64
                    >= STAIRS_SHARE * during.len() as f64;
            let lo = step_windows
                .partition_point(|&(at, _)| at < start_ns - (WALKED_BEFORE_S * 1e9) as i64 - 1);
            let walked_before = step_windows[lo..first].iter().any(|&(at, steps)| {
                steps && at <= start_ns && (start_ns - at) as f64 * 1e-9 <= WALKED_BEFORE_S
            }) || step_windows[first..]
                .iter()
                .take_while(|&&(at, _)| at <= start_ns)
                .any(|&(_, steps)| steps);
            let kind = if stepping {
                Kind::Stairs
            } else if walked_before {
                Kind::Riding
            } else {
                return None;
            };
            Some(Move {
                start_ns,
                end_ns,
                change_m: to - from,
                kind,
            })
        })
        .collect();
    // A climb's runs can break where it eases (a landing); overlapping ones the same way are one move.
    let mut joined: Vec<Move> = Vec::new();
    for m in found {
        match joined.last_mut() {
            Some(last)
                if m.start_ns <= last.end_ns
                    && last.kind == m.kind
                    && last.change_m.signum() == m.change_m.signum() =>
            {
                let end = heights
                    .partition_point(|h| h.0 <= m.end_ns)
                    .checked_sub(1)
                    .map_or(0.0, |i| heights[i].1);
                let start = heights
                    .get(heights.partition_point(|h| h.0 < last.start_ns))
                    .map_or(0.0, |h| h.1);
                last.end_ns = m.end_ns;
                last.change_m = end - start;
            }
            _ => joined.push(m),
        }
    }
    // Joined runs that came back to about where they started, or more than any station's depth (the deepest
    // are about 55 m), are the weather or ventilation moving the pressure.
    joined.retain(|m| m.change_m.abs() >= MIN_CHANGE_M && m.change_m.abs() <= MAX_CHANGE_M);
    joined
}

/// Whether the second at `i` is moving: the height changed by [`MIN_CHANGE_M`] over the [`SPAN_S`] centered on it.
fn moving(heights: &[(i64, f64)], i: usize) -> bool {
    let half = (SPAN_S / 2) as usize;
    // Seconds can be missing; only a full span counts.
    let (a, b) = (heights[i - half], heights[i + half]);
    b.0 - a.0 <= (SPAN_S + 2) * 1_000_000_000 && (b.1 - a.1).abs() >= MIN_CHANGE_M
}

/// The [`moves`] that end after `since_ns`, from the heights since a still stretch before it:
/// none of them joins one before that, so they're found the same, at a cost that doesn't grow with the ride.
pub fn moves_since(
    heights: &[(i64, f64)],
    step_windows: &[(i64, bool)],
    window_s: f64,
    since_ns: i64,
) -> Vec<Move> {
    let half = (SPAN_S / 2) as usize;
    // A move ends half a span after its last moving second.
    let mut i = heights.partition_point(|h| h.0 < since_ns.saturating_sub(SPAN_S * 1_000_000_000));
    i = i.min(heights.len().saturating_sub(half));
    // Back to a stretch still for longer than two spans: no move before it reaches past it.
    let mut still = 0;
    while i > half && still <= 2 * SPAN_S as usize + 2 {
        i -= 1;
        if moving(heights, i) {
            still = 0;
        } else {
            still += 1;
        }
    }
    let from = i.saturating_sub(half);
    let mut found = moves(&heights[from..], step_windows, window_s);
    found.retain(|m| m.end_ns > since_ns);
    found
}

#[cfg(test)]
mod tests {
    use super::*;

    const S: i64 = 1_000_000_000;

    /// Heights each second: flat, then `rate` m/s for `secs` seconds from 100 s, then flat.
    fn climb(rate: f64, secs: i64) -> Vec<(i64, f64)> {
        (0..300)
            .map(|t| (t * S, rate * (t.clamp(100, 100 + secs) - 100) as f64))
            .collect()
    }

    fn windows(steps: impl Fn(i64) -> bool) -> Vec<(i64, bool)> {
        (1..75).map(|k| (k * 4 * S, steps(k * 4))).collect()
    }

    #[test]
    fn elevator_walked_onto_is_riding() {
        let m = moves(&climb(0.5, 25), &windows(|t| t < 100), 4.0);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].kind, Kind::Riding);
        assert!((m[0].change_m - 12.5).abs() < 0.1);
    }

    #[test]
    fn stairs_have_steps() {
        let m = moves(&climb(-0.4, 30), &windows(|_| true), 4.0);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].kind, Kind::Stairs);
        assert!(m[0].change_m < -11.0);
    }

    #[test]
    fn no_walking_around_it_is_a_vehicle() {
        assert!(moves(&climb(0.5, 25), &windows(|_| false), 4.0).is_empty());
    }

    #[test]
    fn small_changes_are_noise() {
        assert!(moves(&climb(0.1, 25), &windows(|_| true), 4.0).is_empty());
    }

    #[test]
    fn recent_moves_are_found_the_same() {
        // Two climbs, the second joined from two runs around a landing.
        let heights: Vec<(i64, f64)> = (0..600)
            .map(|t| {
                let h = 0.5 * (t.clamp(100, 125) - 100) as f64
                    + 0.5 * (t.clamp(400, 420) - 400) as f64
                    + 0.5 * (t.clamp(424, 444) - 424) as f64;
                (t * S, h)
            })
            .collect();
        let w: Vec<(i64, bool)> = (1..150).map(|k| (k * 4 * S, k % 3 == 0)).collect();
        let all = moves(&heights, &w, 4.0);
        assert_eq!(all.len(), 2);
        for since in [0, 200 * S, 430 * S, 460 * S] {
            let want: Vec<Move> = all.iter().copied().filter(|m| m.end_ns > since).collect();
            assert_eq!(moves_since(&heights, &w, 4.0, since), want);
        }
    }

    #[test]
    fn altimeter_takes_each_seconds_median() {
        let mut a = Altimeter::default();
        a.on_pressure(0, 1000.0);
        a.on_pressure(S / 2, 1000.0);
        a.on_pressure(S, 999.0);
        a.on_pressure(S + 1, 999.0);
        a.on_pressure(S + 2, 950.0);
        a.on_pressure(2 * S, 999.0);
        assert_eq!(a.heights().len(), 2);
        assert!((a.heights()[1].1 - M_PER_HPA).abs() < 1e-9);
    }
}

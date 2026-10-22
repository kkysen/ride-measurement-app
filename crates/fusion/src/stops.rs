//! The train's stops from its motion, a port of `python/experiments/stopping_lurch.py` (with `stops.lurches`,
//! `stops.surges`, and `startstop.world_horizontal_50hz`): of the braking lurches, the one the train stands
//! after, and the start surge that ends the stop.
//!
//! Horizontal acceleration is taken in a world frame carried by the gyro and pulled toward gravity over
//! [`FRAME_TAU_S`], at 50 Hz, smoothed over 0.2 s, so tilting the phone in hand doesn't leak gravity in.
//! A lurch is a collapse of sustained braking (over [`SUSTAINED_MPS2`] for 3 s, falling by half within 0.6 s),
//! after braking along that direction for half of the 10 s before. It stops the train when, along its braking
//! direction, the 1 to 5 s after average under [`AFTER_MAX_MPS2`] (stopped, the car is still or settles back),
//! its score (the share of the 20 s before spent braking, times the braking just before, less twice any braking
//! after) is at least [`MIN_SCORE`], and no lurch within [`SEP_S`] either side scores higher.
//! So a stop is known [`SEP_S`] and a bit after it happened.
//! On four rides with precise "train stopped" taps, 20 of 28 within 1 s and 21 within 2 s.

use std::collections::VecDeque;

const RATE_HZ: f64 = 50.0;
const BIN_NS: i64 = 20_000_000;
const FRAME_TAU_S: f64 = 60.0;
const SMOOTH_N: usize = 10;

const SUSTAINED_S: f64 = 3.0;
const SUSTAINED_MPS2: f64 = 0.3;
const COLLAPSE: f64 = 0.5;
const COLLAPSE_MIN_MPS2: f64 = 0.3;
const COLLAPSE_WITHIN_S: f64 = 0.6;
const LURCH_EVERY_S: f64 = 3.0;
const BRAKING_BEFORE_S: f64 = 10.0;
const BRAKING_SHARE: f64 = 0.5;
const BRAKING_MPS2: f64 = 0.2;

const AFTER_S: (f64, f64) = (1.0, 5.0);
const BRAKED_S: f64 = 20.0;
const AFTER_MAX_MPS2: f64 = 0.1;
const MIN_SCORE: f64 = 0.2;
pub const SEP_S: f64 = 30.0;

/// A start surge: from a steady level (spread under [`STEADY_MPS2`] over [`STEADY_S`]), the 1 s mean steps by over
/// [`STRONG_MPS2`] and holds along that direction for [`STRONG_S`]. At most one per [`SURGE_EVERY_S`].
const STEADY_S: f64 = 4.0;
const STEADY_MPS2: f64 = 0.06;
const STRONG_MPS2: f64 = 0.5;
const STRONG_S: f64 = 3.0;
const SURGE_EVERY_S: f64 = 20.0;

/// How much history is kept (s): the 20 s of braking before a lurch, and the [`SEP_S`] after it.
const KEEP_S: f64 = 2.0 * SEP_S + BRAKED_S + 10.0;

/// A stop: its stopping lurch, dated (as in Python) at the start of the [`COLLAPSE_WITHIN_S`] the braking
/// collapses within, so the train stands from [`Stop::standing_ns`]; and the lurch's score (higher is surer).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stop {
    pub t_ns: i64,
    pub score: f64,
}

/// What the detector found as of the latest sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Event {
    Stopped(Stop),
    /// A start surge's beginning: the train pulling away.
    Departed(i64),
}

/// A 50 Hz bin being filled: its index, the sum of acceleration and its count, and of rotation rate and its count.
type Bin = (i64, [f64; 3], usize, [f64; 3], usize);

pub struct StopDetector {
    bin: Option<Bin>,
    gyro: [f64; 3],
    /// Rows: the world axes in the phone's frame, once the first second has set up.
    frame: Option<[[f64; 3]; 3]>,
    first: Vec<[f64; 3]>,
    raw: VecDeque<[f64; 2]>,
    /// Smoothed horizontal acceleration, its time, and the running sum for 1 s means.
    t: VecDeque<i64>,
    h: VecDeque<[f64; 2]>,
    /// Index (in samples since the start) of `h[0]`.
    base: usize,
    next_end: usize,
    last_end_ns: Option<i64>,
    /// Lurches waiting for their score, and scored ones waiting [`SEP_S`] to see if a better one comes.
    unscored: VecDeque<usize>,
    scored: VecDeque<(i64, f64)>,
    last_stop: Option<Stop>,
    next_surge: usize,
    last_surge_ns: Option<i64>,
}

impl Stop {
    /// When the train surely stands: once the braking has collapsed.
    pub fn standing_ns(&self) -> i64 {
        self.t_ns + (COLLAPSE_WITHIN_S * 1e9) as i64
    }
}

impl Default for StopDetector {
    fn default() -> Self {
        Self::new()
    }
}

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn norm3(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn unit(v: [f64; 3]) -> [f64; 3] {
    let n = norm3(v).max(1e-12);
    [v[0] / n, v[1] / n, v[2] / n]
}

impl StopDetector {
    pub fn new() -> Self {
        Self {
            bin: None,
            gyro: [0.0; 3],
            frame: None,
            first: Vec::new(),
            raw: VecDeque::new(),
            t: VecDeque::new(),
            h: VecDeque::new(),
            base: 0,
            next_end: 0,
            last_end_ns: None,
            unscored: VecDeque::new(),
            scored: VecDeque::new(),
            last_stop: None,
            next_surge: 0,
            last_surge_ns: None,
        }
    }

    pub fn on_gyro(&mut self, t_ns: i64, w: [f64; 3]) -> Vec<Event> {
        let events = self.advance(t_ns);
        if let Some((_, _, _, g, n)) = &mut self.bin {
            for k in 0..3 {
                g[k] += w[k];
            }
            *n += 1;
        }
        self.gyro = w;
        events
    }

    pub fn on_accel(&mut self, t_ns: i64, a: [f64; 3]) -> Vec<Event> {
        let events = self.advance(t_ns);
        let (_, sum, n, _, _) =
            self.bin
                .get_or_insert((t_ns.div_euclid(BIN_NS), [0.0; 3], 0, [0.0; 3], 0));
        for k in 0..3 {
            sum[k] += a[k];
        }
        *n += 1;
        events
    }

    /// Closes the bin being filled once a sample of a later one comes.
    fn advance(&mut self, t_ns: i64) -> Vec<Event> {
        let k = t_ns.div_euclid(BIN_NS);
        match self.bin {
            Some((b, ..)) if b < k => {}
            None => {
                self.bin = Some((k, [0.0; 3], 0, [0.0; 3], 0));
                return Vec::new();
            }
            _ => return Vec::new(),
        }
        let (b, sum, n, gsum, gn) = self.bin.take().unwrap();
        self.bin = Some((k, [0.0; 3], 0, [0.0; 3], 0));
        if n == 0 {
            return Vec::new();
        }
        let acc = [sum[0] / n as f64, sum[1] / n as f64, sum[2] / n as f64];
        let w = if gn > 0 {
            [
                gsum[0] / gn as f64,
                gsum[1] / gn as f64,
                gsum[2] / gn as f64,
            ]
        } else {
            self.gyro
        };
        self.on_bin(b * BIN_NS + BIN_NS / 2, acc, w)
    }

    fn on_bin(&mut self, t_ns: i64, acc: [f64; 3], w: [f64; 3]) -> Vec<Event> {
        let Some(r) = &mut self.frame else {
            self.first.push(acc);
            if self.first.len() >= RATE_HZ as usize {
                let n = self.first.len() as f64;
                let z = unit(self.first.iter().fold([0.0; 3], |s, a| {
                    [s[0] + a[0] / n, s[1] + a[1] / n, s[2] + a[2] / n]
                }));
                let x = unit(if z[1].abs() < 0.9 {
                    cross([0.0, 1.0, 0.0], z)
                } else {
                    cross([1.0, 0.0, 0.0], z)
                });
                self.frame = Some([x, cross(z, x), z]);
                self.first = Vec::new();
            }
            return Vec::new();
        };
        // The phone turned by w dt, so the world axes, in its frame, turn the other way (`r @ rot`).
        let dt = 1.0 / RATE_HZ;
        let wv = [w[0] * dt, w[1] * dt, w[2] * dt];
        let angle = norm3(wv);
        if angle > 0.0 {
            let ax = [wv[0] / angle, wv[1] / angle, wv[2] / angle];
            let kx = [
                [0.0, -ax[2], ax[1]],
                [ax[2], 0.0, -ax[0]],
                [-ax[1], ax[0], 0.0],
            ];
            let mut kk = [[0.0; 3]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    kk[i][j] = (0..3).map(|m| kx[i][m] * kx[m][j]).sum();
                }
            }
            let (s, c) = angle.sin_cos();
            let mut rot = [[0.0; 3]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    rot[i][j] = f64::from(u8::from(i == j)) + s * kx[i][j] + (1.0 - c) * kk[i][j];
                }
            }
            let old = *r;
            for i in 0..3 {
                for j in 0..3 {
                    r[i][j] = (0..3).map(|m| old[i][m] * rot[m][j]).sum();
                }
            }
        }
        // Pull world z toward the measured acceleration, slowly.
        let alpha = 1.0 / (RATE_HZ * FRAME_TAU_S);
        let m = unit(acc);
        let zc = unit([
            r[2][0] + alpha * (m[0] - r[2][0]),
            r[2][1] + alpha * (m[1] - r[2][1]),
            r[2][2] + alpha * (m[2] - r[2][2]),
        ]);
        let p = r[0][0] * zc[0] + r[0][1] * zc[1] + r[0][2] * zc[2];
        let xc = unit([
            r[0][0] - p * zc[0],
            r[0][1] - p * zc[1],
            r[0][2] - p * zc[2],
        ]);
        *r = [xc, cross(zc, xc), zc];
        let world = |row: [f64; 3]| row[0] * acc[0] + row[1] * acc[1] + row[2] * acc[2];
        self.raw.push_back([world(r[0]), world(r[1])]);
        if self.raw.len() > SMOOTH_N {
            self.raw.pop_front();
        }
        if self.raw.len() < SMOOTH_N {
            return Vec::new();
        }
        // Centered over 0.2 s: this mean belongs half the window back.
        let n = SMOOTH_N as f64;
        let mean = self
            .raw
            .iter()
            .fold([0.0; 2], |s, v| [s[0] + v[0] / n, s[1] + v[1] / n]);
        self.t.push_back(t_ns - (SMOOTH_N as i64 / 2) * BIN_NS);
        self.h.push_back(mean);
        let mut events = self.find_lurches();
        events.extend(self.find_surges());
        self.trim();
        events
    }

    fn end(&self) -> usize {
        self.base + self.h.len()
    }

    fn at(&self, i: usize) -> [f64; 2] {
        self.h[i - self.base]
    }

    fn time(&self, i: usize) -> i64 {
        self.t[i - self.base]
    }

    /// The mean of `h` over samples `a..b` (clamped to what's kept).
    fn mean(&self, a: usize, b: usize) -> [f64; 2] {
        let a = a.max(self.base);
        if b <= a {
            return [0.0; 2];
        }
        let n = (b - a) as f64;
        (a..b).fold([0.0; 2], |s, i| {
            let v = self.at(i);
            [s[0] + v[0] / n, s[1] + v[1] / n]
        })
    }

    fn samples(s: f64) -> usize {
        (s * RATE_HZ) as usize
    }

    fn find_lurches(&mut self) -> Vec<Event> {
        let nb = Self::samples(SUSTAINED_S);
        let nw = Self::samples(COLLAPSE_WITHIN_S);
        let mut events = Vec::new();
        self.next_end = self.next_end.max(self.base + nb);
        while self.next_end + nw <= self.end() {
            let i = self.next_end;
            self.next_end += 1;
            let mean = self.mean(i - nb, i);
            let size = dot(mean, mean).sqrt();
            if size < SUSTAINED_MPS2 {
                continue;
            }
            let u = [mean[0] / size, mean[1] / size];
            let after = (i..i + nw)
                .map(|k| dot(self.at(k), u))
                .fold(f64::INFINITY, f64::min);
            let drop = size - after;
            let t = self.time(i);
            if !(drop > COLLAPSE * size && drop > COLLAPSE_MIN_MPS2)
                || self
                    .last_end_ns
                    .is_some_and(|l| (t - l) as f64 * 1e-9 <= LURCH_EVERY_S)
            {
                continue;
            }
            self.last_end_ns = Some(t);
            // Braking before it: along the 3 s before's direction, for half the 10 s before (but the last 0.5 s).
            let u3 = self.mean(i.saturating_sub(Self::samples(SUSTAINED_S)), i);
            let n3 = dot(u3, u3).sqrt().max(1e-12);
            let u3 = [u3[0] / n3, u3[1] / n3];
            let a = i
                .saturating_sub(Self::samples(BRAKING_BEFORE_S))
                .max(self.base);
            let b = i.saturating_sub(Self::samples(0.5));
            if b > a {
                let braking = (a..b)
                    .filter(|&k| dot(self.at(k), u3) > BRAKING_MPS2)
                    .count();
                if braking as f64 / (b - a) as f64 >= BRAKING_SHARE {
                    self.unscored.push_back(i);
                }
            }
        }
        // Score lurches once the 5 s after them have come.
        let after_end = Self::samples(AFTER_S.1) + Self::samples(1.0);
        while let Some(&i) = self.unscored.front() {
            if i + after_end > self.end() {
                break;
            }
            self.unscored.pop_front();
            if let Some(score) = self.score(i) {
                self.scored.push_back((self.time(i), score));
            }
        }
        // A scored lurch stops the train once nothing within `SEP_S` after it can outscore it.
        let Some(&now) = self.t.back() else {
            return events;
        };
        let sep = (SEP_S * 1e9) as i64;
        let wait = sep + ((AFTER_S.1 + 2.0 + SUSTAINED_S) * 1e9) as i64;
        while let Some(&(t, score)) = self.scored.front() {
            if now - t < wait {
                break;
            }
            self.scored.pop_front();
            let beaten_later = self.scored.iter().any(|&(u, s)| u - t <= sep && s > score);
            let beaten_before = self
                .last_stop
                .is_some_and(|s| t - s.t_ns <= sep && s.score >= score);
            if beaten_later || beaten_before {
                continue;
            }
            let stop = Stop { t_ns: t, score };
            self.last_stop = Some(stop);
            events.push(Event::Stopped(stop));
        }
        events
    }

    fn score(&self, i: usize) -> Option<f64> {
        let u = self.mean(i.saturating_sub(Self::samples(SUSTAINED_S)), i);
        let size = dot(u, u).sqrt();
        let u = [u[0] / size.max(1e-12), u[1] / size.max(1e-12)];
        // 1 s means over the 1 to 5 s after, along the braking direction.
        let n1 = Self::samples(1.0);
        let (a, b) = (i + Self::samples(AFTER_S.0), i + Self::samples(AFTER_S.1));
        let after = (a..b)
            .map(|k| dot(self.mean(k + 1 - n1, k + 1), u))
            .sum::<f64>()
            / (b - a) as f64;
        if after >= AFTER_MAX_MPS2 {
            return None;
        }
        let a = i.saturating_sub(Self::samples(BRAKED_S)).max(self.base);
        let braked = if i > a {
            (a..i)
                .filter(|&k| dot(self.at(k), u) > BRAKING_MPS2)
                .count() as f64
                / (i - a) as f64
        } else {
            0.0
        };
        let score = braked * size - 2.0 * after.max(0.0);
        (score >= MIN_SCORE).then_some(score)
    }

    fn find_surges(&mut self) -> Vec<Event> {
        let n1 = Self::samples(1.0);
        let nq = Self::samples(STEADY_S);
        let ns = Self::samples(STRONG_S);
        let mut events = Vec::new();
        self.next_surge = self.next_surge.max(self.base + nq + 2 * n1);
        while self.next_surge + ns <= self.end() {
            let i = self.next_surge;
            self.next_surge += 5;
            let t = self.time(i);
            if self
                .last_surge_ns
                .is_some_and(|l| (t - l) as f64 * 1e-9 < SURGE_EVERY_S)
            {
                continue;
            }
            let m = |k: usize| self.mean(k + 1 - n1, k + 1);
            let before: Vec<[f64; 2]> = (i - nq - n1..i - n1).map(m).collect();
            let n = before.len() as f64;
            let base = before
                .iter()
                .fold([0.0; 2], |s, v| [s[0] + v[0] / n, s[1] + v[1] / n]);
            let mi = m(i);
            let step = [mi[0] - base[0], mi[1] - base[1]];
            let size = dot(step, step).sqrt();
            if size < STRONG_MPS2 {
                continue;
            }
            let spread_ok = before.iter().all(|v| {
                let d = [v[0] - base[0], v[1] - base[1]];
                dot(d, d).sqrt() <= 2.5 * STEADY_MPS2
            });
            let std_ok = (0..2).all(|k| {
                let var = before.iter().map(|v| (v[k] - base[k]).powi(2)).sum::<f64>() / n;
                var.sqrt() <= STEADY_MPS2
            });
            if !spread_ok || !std_ok {
                continue;
            }
            let u = [step[0] / size, step[1] / size];
            let held = (i..i + ns).all(|k| {
                let v = m(k);
                dot([v[0] - base[0], v[1] - base[1]], u) > STRONG_MPS2
            });
            if held {
                self.last_surge_ns = Some(t);
                events.push(Event::Departed(t));
            }
        }
        events
    }

    fn trim(&mut self) {
        let keep = Self::samples(KEEP_S);
        let oldest_needed = [
            self.unscored.front().copied(),
            Some(self.next_end),
            Some(self.next_surge),
        ]
        .into_iter()
        .flatten()
        .min()
        .unwrap_or(self.end())
        .saturating_sub(Self::samples(BRAKED_S + STEADY_S + 2.0));
        while self.h.len() > keep && self.base < oldest_needed {
            self.h.pop_front();
            self.t.pop_front();
            self.base += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GRAVITY_MPS2: f64 = 9.80665;

    /// A phone lying flat on a train: `accel(t)` along x (m/s^2, positive pulling forward), 100 Hz, no rotation.
    fn ride(secs: f64, accel: impl Fn(f64) -> f64) -> Vec<Event> {
        let mut d = StopDetector::new();
        let mut out = Vec::new();
        let n = (secs * 100.0) as i64;
        for k in 0..n {
            let t_ns = k * 10_000_000;
            let a = accel(k as f64 / 100.0);
            out.extend(d.on_gyro(t_ns, [0.0; 3]));
            out.extend(d.on_accel(t_ns, [a, 0.0, GRAVITY_MPS2]));
        }
        out
    }

    fn stops(events: &[Event]) -> Vec<f64> {
        events
            .iter()
            .filter_map(|e| match e {
                Event::Stopped(s) => Some(s.t_ns as f64 * 1e-9),
                _ => None,
            })
            .collect()
    }

    fn departures(events: &[Event]) -> Vec<f64> {
        events
            .iter()
            .filter_map(|e| match e {
                Event::Departed(t) => Some(*t as f64 * 1e-9),
                _ => None,
            })
            .collect()
    }

    /// Standing, pulling away (the motors' pull fading as the train speeds up), cruising, then braking for 30 s
    /// with a lurch partway that braking goes on after, and the final lurch at 100 s; standing 30 s, and away again.
    fn profile(t: f64) -> f64 {
        match t {
            t if t < 10.0 => 0.0,
            t if t < 30.0 => 1.0 - (t - 10.0) / 20.0,
            t if t < 70.0 => 0.0,
            t if t < 85.0 => -0.9,
            // A brief easing in mid-braking: a lurch, but the braking resumes.
            t if t < 86.0 => -0.1,
            t if t < 100.0 => -0.9,
            // Stopped: the car settles back a little, then stands.
            t if t < 101.0 => 0.15,
            t if t < 130.0 => 0.0,
            t if t < 150.0 => 1.0 - (t - 130.0) / 20.0,
            _ => 0.0,
        }
    }

    #[test]
    fn the_stop_is_the_final_lurch() {
        let events = ride(200.0, profile);
        let s = stops(&events);
        assert_eq!(s.len(), 1, "{events:?}");
        // Dated at the start of the 0.6 s the braking collapses within, as in Python.
        assert!(s[0] > 99.2 && s[0] <= 100.0, "{s:?}");
    }

    #[test]
    fn departures_are_the_surges() {
        let events = ride(200.0, profile);
        // Surges have no direction, so braking's onset and the stop itself step from a steady level too;
        // what follows a stop by more than a few seconds is the departure.
        let d = departures(&events);
        assert!(d.iter().any(|&t| (t - 10.0).abs() < 1.5), "{d:?}");
        let after_stop: Vec<f64> = d.iter().copied().filter(|&t| t > 105.0).collect();
        assert!((after_stop[0] - 130.0).abs() < 1.5, "{d:?}");
    }

    #[test]
    fn braking_that_goes_on_is_no_stop() {
        // Easing, then braking again, never standing.
        let events = ride(200.0, |t| match t {
            t if t < 70.0 => 0.0,
            t if t < 85.0 => -0.9,
            t if t < 86.0 => -0.1,
            t if t < 110.0 => -0.9,
            _ => 0.0,
        });
        // Only the end of braking at 110 s could be a stop; nothing at 85 s.
        assert!(
            stops(&events).iter().all(|&s| (s - 85.0).abs() > 2.0),
            "{events:?}"
        );
    }
}

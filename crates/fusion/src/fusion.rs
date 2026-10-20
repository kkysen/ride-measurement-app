//! Everything the app and offline replay feed: IMU, GNSS fixes, raw GNSS, and navigation messages,
//! fused into a live speed and, over a window of recent inputs, a smoothed one.
//!
//! The one entry point for both, so live and offline can't differ:
//! - Raw GNSS measurements are solved for velocity (`ride_gnss`), which feeds the estimator with its real uncertainty;
//!   the phone's own fix speed is used only when there's no such solution, since both come from the same Doppler.
//! - Inputs settle into the forward pass in time order after [`SETTLE_NS`], whatever order they arrive in,
//!   and the live display gets a provisional estimate through the newest ones.
//! - Smoothing runs the same estimator backward in time over the buffered inputs (gyro negated, as time reversal
//!   runs rotation backward) and combines both directions at each fix and second by their uncertainties.
//!   Offline, it runs once over the whole ride. Live, it re-runs every [`SMOOTH_EVERY_NS`] of settled input
//!   that brought a new GNSS measurement, and a value freezes once smoothing has included [`SMOOTH_WINDOW_NS`]
//!   of GNSS after it: so the fixes after a tunnel, however long, correct all of it, up to [`MAX_UNMEASURED_NS`].
//!   The backward pass carries the first good fix after a tunnel back through it, which a forward filter can't.
//!   It starts from the forward pass's frame (gravity, travel axis, bias), which constant speed can't teach it.

use std::collections::{BTreeMap, VecDeque};

use ride_gnss::solve::{Navigator, geodetic_to_ecef};
use ride_session::proto::{GnssMeasurementsEvent, GnssNavigationMessage};

use crate::live_speed::{Estimate, LiveSpeedEstimator};

/// Own velocity solutions this recent suppress the phone's fix speed (which comes from the same Doppler).
const OWN_SOLUTION_HOLD_NS: i64 = 3_000_000_000;

/// One buffered input, for re-running the estimator.
#[derive(Clone, Debug)]
enum Input {
    Accel([f64; 3]),
    Gyro([f64; 3]),
    /// A fix, with its speed only if it was used. Boxed: fixes are rare, and IMU samples (most of the window,
    /// which on the phone is about half a million inputs) stay small.
    Fix(Box<Fix>),
    /// Our own velocity solution's speed and standard deviation.
    Speed(f64, f64),
}

/// Inputs settle into the forward pass, in time order, once input this much newer has arrived: Android delivers
/// sensor samples up to 1 s late (`SensorStreamRecorder.kt`'s batching), and GNSS a little after its epoch.
/// Live and offline feed the same buffer, so both see exactly the same order; until then, the live display
/// gets a provisional estimate through the unsettled inputs (see [`Fusion::estimate`]).
pub const SETTLE_NS: i64 = 1_500_000_000;

/// An input waiting to settle.
#[derive(Clone, Debug)]
enum Pending {
    Accel([f64; 3]),
    Gyro([f64; 3]),
    Fix(Fix),
    Measurements(Box<GnssMeasurementsEvent>),
}

impl Pending {
    /// Order among inputs at the same time: IMU first, as offline used to sort them.
    fn rank(&self) -> u8 {
        match self {
            Self::Accel(_) => 0,
            Self::Gyro(_) => 1,
            Self::Fix(_) => 2,
            Self::Measurements(_) => 3,
        }
    }
}

/// Feeds one settled (or provisional) input to the estimator; returns what smoothing keeps of it.
/// `own` is the speed and standard deviation of the velocity solved from `Measurements`, if any.
fn step(
    estimator: &mut LiveSpeedEstimator,
    last_own_solution_ns: &mut Option<i64>,
    t_ns: i64,
    input: &Pending,
    own: Option<(f64, f64)>,
) -> Option<Input> {
    match input {
        Pending::Accel(a) => {
            estimator.on_accel(t_ns, a[0], a[1], a[2]);
            Some(Input::Accel(*a))
        }
        Pending::Gyro(w) => {
            estimator.on_gyro(w[0], w[1], w[2]);
            Some(Input::Gyro(*w))
        }
        Pending::Fix(fix) => {
            let own_recent =
                last_own_solution_ns.is_some_and(|t| (t_ns - t).abs() <= OWN_SOLUTION_HOLD_NS);
            let (speed, speed_sigma) = if own_recent {
                (None, None)
            } else {
                (fix.speed, fix.speed_sigma)
            };
            estimator.on_gnss_fix(t_ns, fix.lat, fix.lon, fix.accuracy, speed, speed_sigma);
            Some(Input::Fix(Box::new(Fix {
                speed,
                speed_sigma,
                ..*fix
            })))
        }
        Pending::Measurements(_) => {
            let (speed, sigma) = own?;
            estimator.on_gnss_speed(t_ns, speed, sigma);
            *last_own_solution_ns = Some(t_ns);
            Some(Input::Speed(speed, sigma))
        }
    }
}

/// Forward estimates are kept at every fix and at least this often, so smoothing covers gaps without fixes.
const SAMPLE_EVERY_NS: i64 = 1_000_000_000;

/// Live smoothing re-runs at most this often, when settled input crosses a multiple of it with a new GNSS
/// measurement, so the map's recent path is recolored about as soon as a fix after a tunnel arrives.
/// Without one it wouldn't change: the backward pass has no weight until a measurement initializes it.
pub const SMOOTH_EVERY_NS: i64 = 1_000_000_000;
/// How much GNSS after a value live smoothing includes before freezing it, so how far back it reaches while GNSS
/// is available: on the phone, about 25 MB of IMU samples, and about 12 ms per re-run on a desktop.
pub const SMOOTH_WINDOW_NS: i64 = 600_000_000_000;
/// Live, values without a GNSS measurement after them freeze anyway once this old,
/// which bounds the phone's memory (about 150 MB of IMU samples) and the re-run on leaving a tunnel.
pub const MAX_UNMEASURED_NS: i64 = 3_600_000_000_000;

/// `android.hardware.Sensor` types.
pub const TYPE_ACCELEROMETER: i32 = 1;
pub const TYPE_GYROSCOPE: i32 = 4;
pub const TYPE_GYROSCOPE_UNCALIBRATED: i32 = 16;
pub const TYPE_ACCELEROMETER_UNCALIBRATED: i32 = 35;

pub enum ImuSample {
    Accel([f64; 3]),
    Gyro([f64; 3]),
}

/// An accelerometer or gyro sample from Android's values: uncalibrated sensors carry their bias estimate in
/// `values[3..6]`, which is subtracted (in `f32`, as Android computes the calibrated values).
pub fn imu_sample(sensor_type: i32, v: &[f32]) -> Option<ImuSample> {
    let uncalibrated = matches!(
        sensor_type,
        TYPE_ACCELEROMETER_UNCALIBRATED | TYPE_GYROSCOPE_UNCALIBRATED
    );
    if v.len() < 3 || (uncalibrated && v.len() < 6) {
        return None;
    }
    let xyz = std::array::from_fn(|k| f64::from(if uncalibrated { v[k] - v[k + 3] } else { v[k] }));
    match sensor_type {
        TYPE_ACCELEROMETER | TYPE_ACCELEROMETER_UNCALIBRATED => Some(ImuSample::Accel(xyz)),
        TYPE_GYROSCOPE | TYPE_GYROSCOPE_UNCALIBRATED => Some(ImuSample::Gyro(xyz)),
        _ => None,
    }
}

/// A GPS fix, as `android.location.Location` reports it (degrees, meters, m/s).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Fix {
    pub elapsed_realtime_ns: i64,
    pub lat: f64,
    pub lon: f64,
    pub altitude: Option<f64>,
    pub accuracy: Option<f64>,
    pub speed: Option<f64>,
    pub speed_sigma: Option<f64>,
}

/// A smoothed speed at a fix or a sample time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SmoothedSpeed {
    pub elapsed_realtime_ns: i64,
    pub speed_mps: f64,
    pub sigma_mps: f64,
    pub forward_mps: f64,
    pub backward_mps: f64,
    /// Acceleration in the direction of travel (m/s^2): the passes' low-passed rates of change of the fused speed,
    /// weighted like the speed. Their lags run opposite ways in time, so the combination is roughly centered.
    pub accel_mps2: f64,
}

pub struct Fusion {
    estimator: LiveSpeedEstimator,
    navigator: Navigator,
    last_own_solution_ns: Option<i64>,
    /// Inputs not yet settled, by time, rank, and arrival.
    pending: BTreeMap<(i64, u8, u64), Pending>,
    arrivals: u64,
    /// The newest input time seen, and the newest settled.
    latest_ns: Option<i64>,
    settled_ns: Option<i64>,
    /// Inputs that arrived after newer ones had already settled, so went into the forward pass out of order.
    pub late_inputs: u64,
    /// Settled inputs with their times, and the forward estimate at each fix and sample time, within `window_ns`.
    inputs: VecDeque<(i64, Input)>,
    forward: VecDeque<(i64, Estimate)>,
    window_ns: Option<i64>,
    /// With a window: the latest smoothing of it, the values frozen as they left it,
    /// and the settled time at or after which smoothing re-runs.
    recent: VecDeque<SmoothedSpeed>,
    frozen: Vec<SmoothedSpeed>,
    next_smooth_ns: Option<i64>,
    /// The times of settled GNSS measurements (fixes and own velocities) in the window,
    /// and the newest one the latest smoothing included.
    measured: VecDeque<i64>,
    measured_ns: Option<i64>,
    smoothed_measured_ns: Option<i64>,
    /// The distance (m) covered by the frozen values (trapezoidally), from the first one's time
    /// to the last one's, whose time and speed the window's distance continues from.
    frozen_distance: Distance,
}

impl Default for Fusion {
    fn default() -> Self {
        Self::new()
    }
}

impl Fusion {
    /// Live: smoothing re-runs as GNSS measurements arrive, over [`SMOOTH_WINDOW_NS`] or since GNSS was lost.
    pub fn new() -> Self {
        Self::with_window(Some(SMOOTH_WINDOW_NS))
    }

    /// Offline: everything is kept and smoothed at once, when asked, so every tunnel gets the fix after it.
    pub fn whole_ride() -> Self {
        Self::with_window(None)
    }

    /// `window_ns`: how long values stay unfrozen once a GNSS measurement after them has been smoothed;
    /// `None` keeps everything and smooths it all at once.
    pub fn with_window(window_ns: Option<i64>) -> Self {
        Self {
            estimator: LiveSpeedEstimator::new(),
            navigator: Navigator::default(),
            last_own_solution_ns: None,
            pending: BTreeMap::new(),
            arrivals: 0,
            latest_ns: None,
            settled_ns: None,
            late_inputs: 0,
            inputs: VecDeque::new(),
            forward: VecDeque::new(),
            window_ns,
            recent: VecDeque::new(),
            frozen: Vec::new(),
            next_smooth_ns: None,
            measured: VecDeque::new(),
            measured_ns: None,
            smoothed_measured_ns: None,
            frozen_distance: Distance::default(),
        }
    }

    pub fn navigator(&self) -> &Navigator {
        &self.navigator
    }

    pub fn navigator_mut(&mut self) -> &mut Navigator {
        &mut self.navigator
    }

    /// An Android sensor sample, as delivered (`SensorEvent.values`); IMU samples are used, others ignored.
    pub fn on_sensor(&mut self, sensor_type: i32, t_ns: i64, values: &[f32]) {
        match imu_sample(sensor_type, values) {
            Some(ImuSample::Accel(a)) => self.on_accel(t_ns, a),
            Some(ImuSample::Gyro(w)) => self.on_gyro(t_ns, w),
            None => {}
        }
    }

    pub fn on_accel(&mut self, t_ns: i64, a: [f64; 3]) {
        self.arrive(t_ns, Pending::Accel(a));
    }

    pub fn on_gyro(&mut self, t_ns: i64, w: [f64; 3]) {
        self.arrive(t_ns, Pending::Gyro(w));
    }

    /// A GPS-provider fix (never fused or network ones, which can be the wrong station underground).
    pub fn on_fix(&mut self, fix: Fix) {
        self.arrive(fix.elapsed_realtime_ns, Pending::Fix(fix));
    }

    /// Navigation messages only add orbits, so they take effect on arrival, live and offline alike.
    pub fn on_navigation_message(&mut self, msg: &GnssNavigationMessage) {
        self.navigator.on_navigation_message(msg);
    }

    /// Raw measurements received at `received_ns`, solved once they settle.
    pub fn on_gnss_measurements(&mut self, received_ns: i64, event: &GnssMeasurementsEvent) {
        let t = event
            .clock
            .as_ref()
            .and_then(|c| c.elapsed_realtime_nanos)
            .unwrap_or(received_ns);
        self.arrive(t, Pending::Measurements(Box::new(event.clone())));
    }

    /// Settles everything still pending, as at the end of a recording, and smooths the window once more.
    pub fn finish(&mut self) {
        while let Some(((t, _, _), input)) = self.pending.pop_first() {
            self.settle(t, input);
        }
        if self.window_ns.is_some() {
            self.recent = self.smooth_window().into();
            self.smoothed_measured_ns = self.measured_ns;
        }
    }

    fn arrive(&mut self, t_ns: i64, input: Pending) {
        if self.settled_ns.is_some_and(|s| t_ns < s) {
            // Newer inputs already went forward; apply it now rather than lose it, and count it.
            self.late_inputs += 1;
            self.settle(t_ns, input);
            return;
        }
        self.arrivals += 1;
        self.pending
            .insert((t_ns, input.rank(), self.arrivals), input);
        let latest = self.latest_ns.map_or(t_ns, |l| l.max(t_ns));
        self.latest_ns = Some(latest);
        while let Some(entry) = self.pending.first_entry() {
            if entry.key().0 > latest - SETTLE_NS {
                break;
            }
            let ((t, _, _), input) = entry.remove_entry();
            self.settle(t, input);
        }
    }

    fn settle(&mut self, t_ns: i64, input: Pending) {
        self.settled_ns = Some(self.settled_ns.map_or(t_ns, |s| s.max(t_ns)));
        let own = match &input {
            Pending::Measurements(event) => self.navigator.on_measurements(event).ok(),
            Pending::Fix(f) => {
                self.navigator.set_approximate_position(geodetic_to_ecef(
                    f.lat,
                    f.lon,
                    f.altitude.unwrap_or(0.0),
                ));
                None
            }
            _ => None,
        }
        .and_then(|sol| sol.velocity)
        .map(|v| (v.speed(), v.speed_sigma()));
        let Some(kept) = step(
            &mut self.estimator,
            &mut self.last_own_solution_ns,
            t_ns,
            &input,
            own,
        ) else {
            return;
        };
        let sample = match kept {
            Input::Fix(_) => true,
            Input::Accel(_) => self
                .forward
                .back()
                .is_none_or(|&(t, _)| t_ns - t >= SAMPLE_EVERY_NS),
            _ => false,
        };
        if matches!(kept, Input::Fix(_) | Input::Speed(..)) {
            self.measured_ns = Some(self.measured_ns.map_or(t_ns, |m| m.max(t_ns)));
            self.measured.push_back(t_ns);
        }
        self.buffer(t_ns, kept);
        if sample {
            self.forward
                .push_back((t_ns, self.estimator.estimate(t_ns)));
        }
        if self.window_ns.is_some() {
            let next = *self.next_smooth_ns.get_or_insert(t_ns);
            if t_ns >= next && self.measured_ns != self.smoothed_measured_ns {
                self.recent = self.smooth_window().into();
                self.smoothed_measured_ns = self.measured_ns;
                self.next_smooth_ns =
                    Some(t_ns - t_ns.rem_euclid(SMOOTH_EVERY_NS) + SMOOTH_EVERY_NS);
            }
        }
    }

    /// The estimate at `now_ns`: the settled forward pass carried on, in a copy, through the inputs still
    /// settling, with measurements solved without changing the navigator. Nothing of it is kept.
    pub fn estimate(&self, now_ns: i64) -> Estimate {
        if self.pending.is_empty() {
            return self.estimator.estimate(now_ns);
        }
        let mut estimator = self.estimator.clone();
        let mut last_own = self.last_own_solution_ns;
        for (&(t, _, _), input) in &self.pending {
            let own = match input {
                Pending::Measurements(event) => self
                    .navigator
                    .solve(event)
                    .ok()
                    .and_then(|sol| sol.velocity)
                    .map(|v| (v.speed(), v.speed_sigma())),
                _ => None,
            };
            step(&mut estimator, &mut last_own, t, input, own);
        }
        estimator.estimate(now_ns)
    }

    fn buffer(&mut self, t_ns: i64, input: Input) {
        self.inputs.push_back((t_ns, input));
        if let Some(window) = self.window_ns {
            // Smoothed with a window's worth of GNSS after it, or too old to wait any longer.
            let measured = &self.measured;
            let smoothed = self.smoothed_measured_ns;
            let done = |t: i64| {
                t_ns - t > MAX_UNMEASURED_NS
                    || measured
                        .iter()
                        .find(|&&m| m > t)
                        .is_some_and(|&first| smoothed.is_some_and(|s| s - first >= window))
            };
            while self.inputs.front().is_some_and(|(t, _)| done(*t)) {
                self.inputs.pop_front();
            }
            while let Some(&(t, ref f)) = self.forward.front() {
                if !done(t) {
                    break;
                }
                // Freeze its latest smoothing, or the forward estimate alone if smoothing never reached it.
                while self
                    .recent
                    .front()
                    .is_some_and(|s| s.elapsed_realtime_ns < t)
                {
                    self.recent.pop_front();
                }
                let value = match self.recent.front() {
                    Some(s) if s.elapsed_realtime_ns == t => self.recent.pop_front().unwrap(),
                    _ => combine(
                        t,
                        f,
                        &Estimate {
                            initialized: false,
                            ..f.clone()
                        },
                    ),
                };
                self.frozen_distance.add(&value);
                self.frozen.push(value);
                self.forward.pop_front();
            }
            let oldest = self.inputs.front().map_or(t_ns, |&(t, _)| t);
            while self.measured.len() > 1 && self.measured[1] <= oldest {
                self.measured.pop_front();
            }
        }
    }

    /// The whole ride's average smoothed speed (m/s), stops included, once it covers any time:
    /// live, the window's part changes as smoothing re-runs, so a tunnel's is corrected by the fix after it.
    pub fn average_speed_mps(&self) -> Option<f64> {
        let mut distance = self.frozen_distance;
        if self.window_ns.is_none() {
            self.smooth_window().iter().for_each(|s| distance.add(s));
        } else {
            self.recent.iter().for_each(|s| distance.add(s));
        }
        distance.average_mps()
    }

    /// Smoothed speeds at every fix and sample time so far: those frozen as they left the window, then the
    /// window's latest smoothing (or, without a window, everything smoothed at once).
    pub fn smoothed(&self) -> Vec<SmoothedSpeed> {
        if self.window_ns.is_none() {
            return self.smooth_window();
        }
        self.frozen.iter().chain(&self.recent).copied().collect()
    }

    /// The window's latest smoothing, which the live map recolors its recent path with.
    pub fn recent_smoothed(&self) -> Vec<SmoothedSpeed> {
        if self.window_ns.is_none() {
            return self.smooth_window();
        }
        self.recent.iter().copied().collect()
    }

    /// Smoothed speeds at every fix and sample time in the window: the forward estimate there combined with
    /// a backward pass over the window, each weighted by its variance. The backward pass gets no weight until a
    /// measurement initializes it, since it starts from nothing at the window's end.
    fn smooth_window(&self) -> Vec<SmoothedSpeed> {
        let Some(&(end, _)) = self.inputs.back() else {
            return Vec::new();
        };
        // Settled inputs are in time order, except any that arrived late; sort a copy for those.
        let mut inputs: Vec<&(i64, Input)> = self.inputs.iter().collect();
        inputs.sort_by_key(|(t, _)| *t);
        let mut backward = self.estimator.reversed();
        // Forward samples from the latest back, each paired with the backward estimate once the pass reaches it.
        let mut pending = self.forward.iter().rev().peekable();
        let mut out = Vec::with_capacity(self.forward.len());
        for &&(t, ref input) in inputs.iter().rev() {
            // Reversed time runs forward from the end; rotation changes sign, acceleration doesn't
            // (see `LiveSpeedEstimator::reversed`).
            let tr = end - t + 1;
            match *input {
                Input::Accel(a) => backward.on_accel(tr, a[0], a[1], a[2]),
                Input::Gyro(w) => backward.on_gyro(-w[0], -w[1], -w[2]),
                Input::Speed(speed, sigma) => backward.on_gnss_speed(tr, speed, sigma),
                Input::Fix(ref f) => {
                    backward.on_gnss_fix(tr, f.lat, f.lon, f.accuracy, f.speed, f.speed_sigma)
                }
            }
            while let Some(&&(ts, ref f)) = pending.peek() {
                if ts < t {
                    break;
                }
                out.push(combine(ts, f, &backward.estimate(end - ts + 1)));
                pending.next();
            }
        }
        out.reverse();
        out
    }
}

/// Distance (m) along smoothed speeds, trapezoidally between their times; values out of time order are skipped.
#[derive(Clone, Copy, Debug, Default)]
struct Distance {
    meters: f64,
    first_ns: Option<i64>,
    last: Option<(i64, f64)>,
}

impl Distance {
    fn add(&mut self, s: &SmoothedSpeed) {
        let t = s.elapsed_realtime_ns;
        let v = s.speed_mps.max(0.0);
        match self.last {
            Some((last_t, _)) if t <= last_t => return,
            Some((last_t, last_v)) => {
                self.meters += (last_v + v) / 2.0 * (t - last_t) as f64 * 1e-9
            }
            None => self.first_ns = Some(t),
        }
        self.last = Some((t, v));
    }

    fn average_mps(&self) -> Option<f64> {
        let first = self.first_ns?;
        let (last, _) = self.last?;
        (last > first).then(|| self.meters / ((last - first) as f64 * 1e-9))
    }
}

fn combine(t: i64, f: &Estimate, b: &Estimate) -> SmoothedSpeed {
    let wf = 1.0 / f.sigma_mps.max(1e-3).powi(2);
    let wb = if b.initialized {
        1.0 / b.sigma_mps.max(1e-3).powi(2)
    } else {
        0.0
    };
    SmoothedSpeed {
        elapsed_realtime_ns: t,
        speed_mps: (f.speed_mps * wf + b.speed_mps * wb) / (wf + wb),
        sigma_mps: (1.0 / (wf + wb)).sqrt(),
        forward_mps: f.speed_mps,
        backward_mps: b.speed_mps,
        // In reversed time, speeding up is slowing down.
        accel_mps2: (f.accel_mps2 * wf - b.accel_mps2 * wb) / (wf + wb),
    }
}

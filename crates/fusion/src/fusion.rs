//! Everything the app and offline replay feed: IMU, GNSS fixes, raw GNSS, and navigation messages,
//! fused into a live speed and, over a window of recent inputs, a smoothed one.
//!
//! The one entry point for both, so live and offline can't differ:
//! - Raw GNSS measurements are solved for velocity (`ride_gnss`), which feeds the estimator with its real uncertainty;
//!   the phone's own fix speed is used only when there's no such solution, since both come from the same Doppler.
//! - Smoothing runs the same estimator backward in time over the buffered inputs (gyro negated, as time reversal
//!   runs rotation backward) and combines both directions at each fix and second by their uncertainties.
//!   The backward pass carries the first good fix after a tunnel back through it, which a forward filter can't.
//!   It starts from the forward pass's frame (gravity, travel axis, bias), which constant speed can't teach it.

use std::collections::VecDeque;

use ride_gnss::solve::{Navigator, Solution, geodetic_to_ecef};
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

/// Forward estimates are kept at every fix and at least this often, so smoothing covers gaps without fixes.
const SAMPLE_EVERY_NS: i64 = 1_000_000_000;

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
    /// Inputs in arrival order with their times, and the forward estimate at each fix and sample time,
    /// within `window_ns`.
    inputs: VecDeque<(i64, Input)>,
    forward: VecDeque<(i64, Estimate)>,
    window_ns: Option<i64>,
}

impl Fusion {
    /// `window_ns`: how much input to keep for smoothing; `None` keeps everything (offline).
    pub fn new(window_ns: Option<i64>) -> Self {
        Self {
            estimator: LiveSpeedEstimator::new(),
            navigator: Navigator::default(),
            last_own_solution_ns: None,
            inputs: VecDeque::new(),
            forward: VecDeque::new(),
            window_ns,
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
        self.estimator.on_accel(t_ns, a[0], a[1], a[2]);
        self.buffer(t_ns, Input::Accel(a));
        if self
            .forward
            .back()
            .is_none_or(|&(t, _)| t_ns - t >= SAMPLE_EVERY_NS)
        {
            self.forward
                .push_back((t_ns, self.estimator.estimate(t_ns)));
        }
    }

    pub fn on_gyro(&mut self, t_ns: i64, w: [f64; 3]) {
        self.estimator.on_gyro(w[0], w[1], w[2]);
        self.buffer(t_ns, Input::Gyro(w));
    }

    /// A GPS-provider fix (never fused or network ones, which can be the wrong station underground).
    pub fn on_fix(&mut self, fix: Fix) {
        let Fix {
            elapsed_realtime_ns: t_ns,
            lat,
            lon,
            altitude,
            accuracy,
            speed,
            speed_sigma,
        } = fix;
        self.navigator.set_approximate_position(geodetic_to_ecef(
            lat,
            lon,
            altitude.unwrap_or(0.0),
        ));
        let own_recent = self
            .last_own_solution_ns
            .is_some_and(|t| (t_ns - t).abs() <= OWN_SOLUTION_HOLD_NS);
        let (speed, speed_sigma) = if own_recent {
            (None, None)
        } else {
            (speed, speed_sigma)
        };
        self.estimator
            .on_gnss_fix(t_ns, lat, lon, accuracy, speed, speed_sigma);
        self.buffer(
            t_ns,
            Input::Fix(Box::new(Fix {
                speed,
                speed_sigma,
                ..fix
            })),
        );
        self.forward
            .push_back((t_ns, self.estimator.estimate(t_ns)));
    }

    pub fn on_navigation_message(&mut self, msg: &GnssNavigationMessage) {
        self.navigator.on_navigation_message(msg);
    }

    /// Raw measurements received at `received_ns`; returns the velocity solution if there was one.
    pub fn on_gnss_measurements(
        &mut self,
        received_ns: i64,
        event: &GnssMeasurementsEvent,
    ) -> Option<Solution> {
        let sol = self.navigator.on_measurements(event).ok()?;
        let t = sol.elapsed_realtime_ns.unwrap_or(received_ns);
        let (speed, sigma) = (sol.speed(), sol.speed_sigma());
        self.estimator.on_gnss_speed(t, speed, sigma);
        self.last_own_solution_ns = Some(t);
        self.buffer(t, Input::Speed(speed, sigma));
        Some(sol)
    }

    pub fn estimate(&self, now_ns: i64) -> Estimate {
        self.estimator.estimate(now_ns)
    }

    fn buffer(&mut self, t_ns: i64, input: Input) {
        self.inputs.push_back((t_ns, input));
        if let Some(window) = self.window_ns {
            while self.inputs.front().is_some_and(|(t, _)| t_ns - t > window) {
                self.inputs.pop_front();
            }
            while self.forward.front().is_some_and(|(t, _)| t_ns - t > window) {
                self.forward.pop_front();
            }
        }
    }

    /// Smoothed speeds at every fix and sample time in the window: the forward estimate there combined with
    /// a backward pass over the window, each weighted by its variance. The backward pass gets no weight until a
    /// measurement initializes it, since it starts from nothing at the window's end.
    pub fn smoothed(&self) -> Vec<SmoothedSpeed> {
        let Some(&(end, _)) = self.inputs.back() else {
            return Vec::new();
        };
        // Inputs are in arrival order, which can be slightly out of time order across sensors; sort a copy.
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

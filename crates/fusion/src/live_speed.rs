//! Live speed: a port of the app's `LiveSpeedEstimator.kt`, so recorded rides can be replayed
//! through exactly what the phone showed, and so the app can later share this code.
//!
//! A 1-D Kalman filter along the direction of travel, with state (speed, accelerometer bias along that direction):
//! - Prediction: acceleration along the travel axis from the IMU. With GNSS, the axis is the horizontal direction
//!   whose acceleration follows GNSS's speed changes, which gives its sign too; before any, it's the dominant
//!   horizontal acceleration direction in the phone's frame. Gravity and the axis are carried through phone rotations
//!   by the gyro, and gravity is re-learned from the accelerometer, so sustained acceleration isn't taken for tilt.
//! - Measurements, each weighted by its own uncertainty:
//!   GNSS Doppler speed, speed from successive GNSS positions, and stops (speed 0, from a quiet accelerometer).
//! - Every measurement is gated: one more than [`GATE_SIGMA`] sigma from the prediction is rejected and counted,
//!   so a single wrong sensor can't drag the estimate. If GNSS is rejected [`GNSS_REINIT_REJECTS`] times in a row,
//!   the IMU side is what's wrong, and the filter re-initializes from GNSS.
//! - The travel axis's sign (which way is forward) comes from GNSS speed changes, or from a departure from a stop
//!   (a train or bus accelerates forward when leaving). Until it's known, speed is the magnitude of the integral.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::sync::OnceLock;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Sensor {
    Imu,
    GnssDoppler,
    GnssPosition,
    Stop,
    /// A door cycle heard in the sound (see [`crate::doors`]): the train stood still at its closing.
    Doors,
    /// Steps in the accelerometer (see [`crate::walking`]): the rider is on foot.
    Walking,
    /// The mode's top speed, applied when the estimate goes over it (see [`LiveSpeedEstimator::set_max_speed`]),
    /// and on the subway, how fast a train can speed up or slow down (see [`subway_accel_limits`]).
    SpeedLimit,
    /// The car's own accelerometer, from its beacon (see [`crate::car_beacons`]): the phone's bias along the axis.
    Car,
    /// A known place on a leg underground: standing at a station, or passing one the radio names
    /// (see [`LiveSpeedEstimator::anchor`]).
    Anchor,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Estimate {
    pub speed_mps: f64,
    pub sigma_mps: f64,
    /// Sensors that contributed in the last [`RECENT_S`] seconds.
    pub sensors: BTreeSet<Sensor>,
    pub stopped: bool,
    pub since_gnss_s: Option<f64>,
    /// Measurements rejected as outliers so far, per sensor.
    pub rejected: BTreeMap<Sensor, u32>,
    /// Whether a measurement (GNSS or a stop) has set the speed yet; before that it's integrated from 0.
    pub initialized: bool,
    /// Acceleration in the direction of travel (m/s^2, negative when braking): the rate of change of the fused
    /// speed (IMU prediction and GNSS and stop corrections alike), averaged over about [`ACCEL_TAU_S`].
    pub accel_mps2: f64,
    /// The fastest the vehicle can go now (m/s), if limited (see [`LiveSpeedEstimator::set_max_speed`]):
    /// the speed is held under it, but its sigma isn't, so the speed and sigma together can exceed it.
    pub max_speed_mps: Option<f64>,
}

/// A leg between stations underground: when the train left, and for each station ahead, nearest first,
/// the soonest a train could stand there (s since the departure) and the fastest it goes on the way;
/// and how far the estimate has gone since the departure, or since a known place ([`LiveSpeedEstimator::anchor`]),
/// only for measuring its error at the next known place, never for the cap.
#[derive(Clone, Debug)]
struct Leg {
    since_ns: i64,
    stations: VecDeque<(f64, f64)>,
    covered_m: f64,
    /// The last known place: when, and how far along the track from the departure (m).
    known: (i64, f64),
}

#[derive(Clone, Copy, Debug)]
struct SpeedRef {
    t_ns: i64,
    speed: f64,
    sigma: f64,
    /// Implausible speeds since, and the last of them.
    implausible: u32,
    last_implausible: Option<(i64, f64, f64)>,
}

#[derive(Clone, Copy, Debug)]
struct Fix {
    t_ns: i64,
    lat: f64,
    lon: f64,
    accuracy_m: f64,
}

const MAX_DT_S: f64 = 0.1;
const STILL_WINDOW_S: f64 = 0.5;
const STILL_STD_MPS2: f64 = 0.04;
const STILL_S: f64 = 2.0;
const GRAVITY_TAU_STILL_S: f64 = 0.5;
/// Moving with recent GNSS, which gives the vehicle's own acceleration to subtract: short enough to follow the gyro's
/// drift (at 600 s, gravity drifted far enough on a ride to leak 1.5 to 3.5 m/s^2 into the along-track acceleration).
const GRAVITY_TAU_MOVING_S: f64 = 30.0;
/// Moving without GNSS: the IMU can't tell tilt from sustained acceleration, so a short time constant would take
/// the vehicle's acceleration for tilt and lose the speed it gained; until a stop, the gyro alone carries gravity.
const GRAVITY_TAU_UNAIDED_S: f64 = 600.0;
/// Before any measurement has set the speed, while the phone is being handled (turning faster than
/// [`HANDLED_RAD_S`] on average): gravity from the first sample, taken with the phone moving in a hand, was 3 m/s^2
/// off at the start of `20261020-184134`, and with no GNSS to re-learn it the speed ran to 221 mph. Handled, the speed
/// is mostly the hand's anyway, so gravity follows the accelerometer closely. A phone lying in a train that starts
/// underground barely turns, so the train's first acceleration isn't taken for tilt.
const GRAVITY_TAU_HANDLED_S: f64 = 2.0;
const HANDLED_RAD_S: f64 = 0.15;
const HANDLED_TAU_S: f64 = 2.0;
const AXIS_TAU_S: f64 = 30.0;
/// How recent GNSS must be for the fused acceleration to be the vehicle's, so gravity can follow the rest.
const GNSS_RECENT_S: f64 = 5.0;
const AXIS_MIN_VARIANCE: f64 = 0.002;
const DEPARTURE_EVIDENCE_MPS: f64 = 1.0;
const SIGN_EVIDENCE: f64 = 2.0;
/// The GNSS-learned axis: how long its correlation averages over, the most acceleration a GNSS speed change can
/// show and still be used (no train exceeds 1.4 m/s^2 outside an emergency; more is a bad speed), the speed sigma
/// at and under which a change gets full weight (noisier ones are weighted down by their variance: noise in a speed
/// change doesn't bias the correlation, it only scatters it), the largest spacing to use, and the correlation
/// ((m/s^2)^2) it takes to replace the spread axis.
const AXIS_GNSS_TAU_S: f64 = 30.0;
const AXIS_GNSS_MAX_ACCEL_MPS2: f64 = 1.5;
/// Tuned on the 2026-10-20 elevated 7 ride with GNSS dropped for 30 s of every 90 s, and checked with 25 s of every 77 s.
const AXIS_GNSS_FULL_WEIGHT_SIGMA_MPS: f64 = 1.0;
const AXIS_GNSS_MAX_DT_S: f64 = 1.5;
const AXIS_GNSS_MIN_CROSS: f64 = 0.01;
/// Both GNSS speeds of a change must be at least this (m/s): near a standstill, a speed's magnitude is mostly noise
/// (on that ride, 2 to 7 km/h jitter at stations), and its changes would turn the axis at every stop.
const AXIS_GNSS_MIN_SPEED_MPS: f64 = 3.0;
/// Centripetal acceleration is only taken out while turning slower than this about the vertical (rad/s, about 6 deg/s):
/// a subway curve of 300 m radius at 25 mph turns at 0.04, and faster is the rider turning the phone, where the
/// yaw rate times a wrong speed would push the speed further off (35 mph stopped in a cut on `20261023-011209`).
const CENTRIPETAL_MAX_YAW_RAD_S: f64 = 0.1;
/// No train turns faster than about 0.18 rad/s about the vertical (sqrt of 1.5 m/s^2 sideways over a 45 m radius,
/// at any speed), so turning faster than this is the phone in a hand, which turns the travel axis in its frame
/// (once GNSS has given the axis, nothing else re-learns it underground: 58 deg at 1.2 rad/s braking into a stop
/// at 6:13 pm on `20261107-175954` left the axis 60 to 80 deg off, so departures barely registered).
const HANDLED_YAW_RAD_S: f64 = 0.25;
/// Process noise: unmodeled along-axis acceleration (m/s^2 per sqrt(s)), and bias random walk.
const ACCEL_NOISE: f64 = 0.3;
const BIAS_WALK: f64 = 0.005;
const BIAS_SIGMA0: f64 = 0.1;
/// Time constant of the reported acceleration's low-pass, to steady it against vibration.
const ACCEL_TAU_S: f64 = 1.0;
const STOP_SIGMA_MPS: f64 = 0.05;
/// A door cycle's closing: the train is standing, but the time is only good to a step's sharpest point.
const DOOR_STOP_SIGMA_MPS: f64 = 0.2;
/// Walking speed and its spread (m/s): people walk at 1 to 2 m/s, slower in crowds and on stairs.
const WALKING_MPS: f64 = 1.4;
const WALKING_SIGMA_MPS: f64 = 0.6;
/// Running: 2 to 4 m/s for most people, jogging for a train included.
const RUNNING_MPS: f64 = 3.0;
const RUNNING_SIGMA_MPS: f64 = 1.0;
/// How often the speed is kept for a door cycle's closing to be compared with (s), and for how long.
const HISTORY_EVERY_S: f64 = 0.1;
const HISTORY_S: f64 = 30.0;
const DEFAULT_DOPPLER_SIGMA_MPS: f64 = 0.5;
const MIN_DOPPLER_SIGMA_MPS: f64 = 0.1;
const POSITION_MAX_DT_S: f64 = 5.0;
const POSITION_MAX_SIGMA_MPS: f64 = 3.0;
/// Mach 1 at sea level: nothing this records goes faster, so a faster measurement is always wrong,
/// even as the first one or during a re-initialization, when gating can't catch it.
pub const MAX_SPEED_MPS: f64 = 343.0;
/// NYC subway trains cruise at 40 to 45 mph and may reach 50, where power cuts off; they never exceed 55 mph.
/// Other rail (LIRR, Metro-North) is faster, so this is only for subway rides.
pub const SUBWAY_MAX_SPEED_MPS: f64 = 55.0 * 0.44704;
/// How fast a subway train can speed up and slow down (m/s^2) at a speed (m/s), as (motoring, braking).
/// Motoring is the traction motors' three regions: constant force, then constant power (1/v),
/// then their natural region (1/v^2). Fit by `data/work/track/fit_curves.py` to the 90th percentile per 2 mph
/// of GNSS acceleration (speeds over 6 to 8 s) on open track, from `crates/cli/examples/accelcurves.rs`
/// (14 runs on 9 rides; `data/work/track/fit_curves.txt`), rms 10 to 15% in log space:
/// the fleet (R142, R160, R68A, R46, R62A) `min(1.00, 6.5 / v, 75 / v^2)`;
/// the 7's R188s (CBTC) the highest, `min(1.24, 9.5 / v, 85 / v^2)`, which this is with 15% margin.
/// Every model ridden is rated 2.5 mph/s motoring (1.12), 3.0 full service braking (1.34), 3.2 emergency
/// (Wikipedia); only the R188s went over the motoring rate beyond GNSS's own noise (2 sigma):
/// at 1.4 to 1.56 m/s^2 between 10 and 18 mph, on 7 of their 13 departures from a stop.
/// Braking's 90th percentile is about 0.76 to 0.8 m/s^2 to 30 mph, then lower; no sample went over
/// the full service rate, which is the bound.
pub fn subway_accel_limits(speed_mps: f64) -> (f64, f64) {
    let v = speed_mps.max(1e-3);
    ((10.9 / v).min(98.0 / (v * v)).min(1.43), 3.0 * 0.44704)
}
/// A train speeding up from a stand as fast as [`subway_accel_limits`] allows (at most [`SUBWAY_MAX_SPEED_MPS`]):
/// its speed every [`REACH_STEP_M`] along, and every [`REACH_STEP_S`] since it started.
fn reach_tables() -> &'static (Vec<f64>, Vec<f64>, Vec<f64>) {
    static TABLES: OnceLock<(Vec<f64>, Vec<f64>, Vec<f64>)> = OnceLock::new();
    TABLES.get_or_init(|| {
        let dt = 0.01;
        let (mut v, mut x, mut t) = (0.0_f64, 0.0_f64, 0.0_f64);
        let (mut by_m, mut by_s, mut m_by_s) = (vec![0.0], vec![0.0], vec![0.0]);
        while v < SUBWAY_MAX_SPEED_MPS || by_m.len() < 2 {
            let a = subway_accel_limits(v).0;
            x += (v + 0.5 * a * dt) * dt;
            v = (v + a * dt).min(SUBWAY_MAX_SPEED_MPS);
            t += dt;
            while x >= by_m.len() as f64 * REACH_STEP_M {
                by_m.push(v);
            }
            while t >= by_s.len() as f64 * REACH_STEP_S {
                by_s.push(v);
                m_by_s.push(x);
            }
        }
        (by_m, by_s, m_by_s)
    })
}

fn lookup(table: &[f64], x: f64) -> f64 {
    let i = x.max(0.0) as usize;
    match (table.get(i), table.get(i + 1)) {
        (Some(a), Some(b)) => a + (b - a) * (x.max(0.0) - i as f64),
        _ => SUBWAY_MAX_SPEED_MPS,
    }
}

/// The fastest a train can be going (m/s) `distance_m` from a stand (see [`reach_tables`]).
pub fn subway_reach_mps(distance_m: f64) -> f64 {
    lookup(&reach_tables().0, distance_m / REACH_STEP_M)
}

/// The fastest a train can be going (m/s) `seconds` after starting from a stand (see [`reach_tables`]).
pub fn subway_reach_by_time_mps(seconds: f64) -> f64 {
    lookup(&reach_tables().1, seconds / REACH_STEP_S)
}

/// The farthest a train can be (m) `seconds` after starting from a stand (see [`reach_tables`]),
/// going on at [`SUBWAY_MAX_SPEED_MPS`] once it reaches it.
pub fn subway_reach_by_time_m(seconds: f64) -> f64 {
    let table = &reach_tables().2;
    let x = seconds.max(0.0) / REACH_STEP_S;
    let last = (table.len() - 1) as f64;
    if x <= last {
        lookup(table, x)
    } else {
        table[table.len() - 1] + (x - last) * REACH_STEP_S * SUBWAY_MAX_SPEED_MPS
    }
}

/// The fastest a train goes (m/s) between two stops `length_m` apart along the track: where speeding up as fast
/// as it can ([`subway_reach_mps`]) meets braking as hard as it can into the far stop.
pub fn subway_peak_mps(length_m: f64) -> f64 {
    let braking = subway_accel_limits(0.0).1;
    (0..=(length_m / REACH_STEP_M) as usize)
        .map(|i| {
            let x = i as f64 * REACH_STEP_M;
            subway_reach_mps(x).min((2.0 * braking * (length_m - x).max(0.0)).sqrt())
        })
        .fold(0.0, f64::max)
}
const REACH_STEP_M: f64 = 5.0;
const REACH_STEP_S: f64 = 0.5;
/// On a leg (see [`LiveSpeedEstimator::start_leg`]), the cap never goes under this (m/s), for the departure's
/// time found a moment late.
const LEG_CAP_FLOOR_MPS: f64 = 2.0;
/// An anchor ([`LiveSpeedEstimator::anchor`]) this soon after the last known place (s) says too little,
/// and the bias moves by at most this (m/s^2) for one.
const ANCHOR_MIN_S: f64 = 15.0;
/// Over a recorded peak ([`LiveSpeedEstimator::start_leg`]), the cap allows the more of this (m/s, 5 mph) and
/// [`RECORDED_MARGIN_SHARE`] of it: the study's own speedometer error (2 mph) and overspeed allowance (1.5 mph),
/// and speed limits changed since its 2018 runs (some raised), somewhat but not massively.
const RECORDED_MARGIN_MPS: f64 = 5.0 * 0.44704;
const RECORDED_MARGIN_SHARE: f64 = 0.15;
const ANCHOR_MAX_DB_MPS2: f64 = 0.3;

/// The slack (m/s) over [`subway_accel_limits`]'s ceiling, for the phone's noise.
const ACCEL_LIMIT_SLACK_MPS: f64 = 0.3;
/// Trains, buses, and planes in normal operation stay well under 1 g along the track,
/// so a GNSS speed that would take more than this to reach from the previous plausible one is wrong.
pub const MAX_ACCEL_MPS2: f64 = 9.80665;
/// After this many implausible GNSS speeds in a row that agree with each other,
/// the earlier reference was the wrong one (e.g. a glitched first fix): start over from them.
const IMPLAUSIBLE_RESET: u32 = 5;
/// The car's accelerometer: how long its means and its correlation with the phone's average over (s),
/// the readings and correlation it takes to trust an axis and its sign, how long the phone's and the car's
/// accelerations are averaged to compare (s; the beacon's reading lags by up to a second), and the comparison's
/// sigma (m/s^2): a count is 0.15, and grades (to 4.6%, 0.45) tilt the car's reading but not the phone's.
const CAR_MEAN_TAU_S: f64 = 60.0;
const CAR_CORR_TAU_S: f64 = 300.0;
const CAR_MIN_READINGS: u32 = 60;
const CAR_MIN_CORR: f64 = 0.5;
const CAR_WINDOW_S: f64 = 4.0;
const CAR_SIGMA_MPS2: f64 = 0.6;
/// The cars heard steadily at once are averaged over each window: their own noise averages down, but the phone's
/// doesn't, so the comparison's variance is this share of [`CAR_SIGMA_MPS2`]'s plus the rest divided by the cars.
/// (`python/experiments/car_accel_average.py`: on `184134`, two cars agree with each other about as well as
/// with GNSS, 0.86 against 0.81 and 0.82, so their noise is mostly their own; averaged, the residual against GNSS
/// falls from 0.32 to 0.25 m/s^2 where both are heard.) Cars not heard for [`CAR_STALE_S`] are dropped.
/// With GNSS cut 30 s of every 90 s, outage rms (at outage end) against the rider's car alone:
/// `154951` 3.41 (6.54) to 3.25 (6.22) km/h, `184134` 6.84 (10.97) to 6.39 (9.82); `214427`, without beacons, unchanged.
const CAR_SHARED: f64 = 0.5;
const CAR_STALE_S: f64 = 10.0;
const CARS_MAX: usize = 6;
/// Whether the car's accelerometer corrects the phone's bias, without recent GNSS only.
/// On since 2026-10-22 at the rider's call: it changed little, but helped at underground stops.
/// It follows GNSS's acceleration (correlation 0.7, no lag), but 0.46 m/s^2 of it is noise against a 0.61 signal.
/// With GNSS cut 30 s of every 90 s, outage rms 3.33 to 3.41 km/h on `154951` and 6.84 on `184134` either way
/// (always on, at 0.3: 4.57 and 6.75); at underground stop taps, the 90th percentile forward speed fell from
/// 46 to 38 mph on `184134` and was unchanged elsewhere, still at the subway's 55 mph cap.
const USE_CAR_ACCEL: bool = true;
/// While stopped, the car's level reading (its mounting and the station's grade) is learned over this (s).
const CAR_LEVEL_TAU_S: f64 = 3.0;

/// One car's accelerometer, as it relates to the phone's travel axis.
#[derive(Clone, Debug, Default)]
struct CarAxis {
    car: u16,
    readings: u32,
    last_ns: i64,
    /// Means of the car's axes and the phone's along-axis acceleration, and their covariances and variances.
    mean: [f64; 3],
    along_mean: f64,
    cross: [f64; 3],
    var: [f64; 3],
    along_var: f64,
    /// The car's reading at rest, per axis, once a stop has given it; until then, its mean over
    /// [`CAR_CORR_TAU_S`], since a train speeds up about as much as it slows down.
    level: Option<[f64; 3]>,
    slow_mean: [f64; 3],
    /// The phone's along-axis acceleration integrated since this car's last reading.
    pending_sum: f64,
    pending_dt: f64,
    /// This car's forward readings in the current window.
    car_sum: f64,
    car_n: u32,
}

impl CarAxis {
    /// The car's forward axis and its sign along the phone's travel axis, once they correlate well enough.
    fn forward(&self) -> Option<(usize, f64)> {
        if self.readings < CAR_MIN_READINGS {
            return None;
        }
        // The vertical axis carries gravity; the forward one is the horizontal axis that follows the phone's.
        let vertical = (0..3).max_by(|&a, &b| self.mean[a].abs().total_cmp(&self.mean[b].abs()))?;
        (0..3)
            .filter(|&k| k != vertical)
            .map(|k| {
                (
                    k,
                    self.cross[k] / (self.var[k] * self.along_var).sqrt().max(1e-9),
                )
            })
            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
            .filter(|&(_, c)| c.abs() >= CAR_MIN_CORR)
            .map(|(k, c)| (k, c.signum()))
    }
}
pub const GATE_SIGMA: f64 = 3.0;
pub const GNSS_REINIT_REJECTS: u32 = 5;
pub const RECENT_S: f64 = 3.0;

/// Not thread-safe; feed it from one thread, in time order.
#[derive(Clone, Debug)]
pub struct LiveSpeedEstimator {
    // IMU front end.
    gravity: [f64; 3],
    gravity_init: bool,
    /// The phone's rotation rate (rad/s), low-passed over [`HANDLED_TAU_S`].
    turning: f64,
    /// Mean and covariance of horizontal acceleration: centered, so a constant leak of gravity isn't the axis.
    axis_mean: [f64; 3],
    cov: [f64; 9],
    axis: [f64; 3],
    axis_known: bool,
    axis_sign_known: bool,
    /// Correlation of horizontal acceleration with GNSS speed changes, and whether the axis comes from it.
    axis_cross: [f64; 3],
    axis_from_gnss: bool,
    /// Horizontal acceleration integrated since the last GNSS speed (m/s, in the phone's frame) and over how long.
    h_since_gnss: [f64; 3],
    h_since_gnss_s: f64,
    omega: [f64; 3],
    last_ns: i64,
    mag_mean: f64,
    mag_var: f64,
    still_since_ns: i64,
    stopped: bool,
    since_stop_along: f64,

    // Filter state: signed speed along the axis, and along-axis accelerometer bias.
    v: f64,
    b: f64,
    p00: f64,
    p01: f64,
    p11: f64,
    initialized: bool,
    /// Low-passed rate of change of `|v|`, and the speed and time it was last updated at.
    accel: f64,
    accel_speed: f64,
    accel_ns: i64,
    /// Whether `initialized` was set when the rate was last updated, and whether the speed was reset since:
    /// the speed jumps then, which isn't acceleration.
    accel_initialized: bool,
    accel_reset: bool,

    // GNSS.
    last_gnss_ns: i64,
    /// The last plausible GNSS speed and its time.
    last_gnss_speed: Option<(i64, f64)>,
    /// Recent (time, signed speed), every [`HISTORY_EVERY_S`], for measurements of a past moment.
    history: std::collections::VecDeque<(i64, f64)>,
    /// How far the covariance has been propagated, by the IMU or [`Self::inflate_to`].
    predicted_ns: i64,
    /// The last GNSS speed: time, speed, and sigma.
    prev_doppler: Option<(i64, f64, f64)>,
    along_since_gnss: f64,
    sign_evidence: f64,
    gnss_reject_streak: u32,
    /// The signed speeds rejected in the current streak, to re-initialize from their median.
    gnss_rejected: Vec<f64>,
    prev_fix: Option<Fix>,

    last_used_ns: BTreeMap<Sensor, i64>,
    rejected: BTreeMap<Sensor, u32>,
    /// Per GNSS sensor, the last plausible speed, to check the next against [`MAX_ACCEL_MPS2`].
    speed_refs: BTreeMap<Sensor, SpeedRef>,
    /// The fastest the vehicle can go, if the mode is known to have a limit.
    max_speed_mps: Option<f64>,
    /// Whether time runs backward (the smoother's backward pass), where speeding up is braking.
    backward: bool,
    /// The most the train could be going (m/s), from the last stop or GNSS speed (see [`Self::limit_accel`]).
    ceiling_mps: Option<f64>,
    /// The leg in progress since a departure from a station (see [`Self::start_leg`]).
    leg: Option<Leg>,
    /// The steady cars' accelerometers, and the current comparison window: since when,
    /// and the phone's along-axis acceleration integrated over it.
    cars: Vec<CarAxis>,
    window_ns: i64,
    window_along_sum: f64,
    window_along_dt: f64,
}

impl Default for LiveSpeedEstimator {
    fn default() -> Self {
        Self {
            gravity: [0.0; 3],
            gravity_init: false,
            turning: 0.0,
            axis_mean: [0.0; 3],
            cov: [0.0; 9],
            axis: [1.0, 0.0, 0.0],
            axis_known: false,
            axis_sign_known: false,
            axis_cross: [0.0; 3],
            axis_from_gnss: false,
            h_since_gnss: [0.0; 3],
            h_since_gnss_s: 0.0,
            omega: [0.0; 3],
            last_ns: 0,
            mag_mean: 0.0,
            mag_var: 0.0,
            still_since_ns: 0,
            stopped: false,
            since_stop_along: 0.0,
            v: 0.0,
            b: 0.0,
            p00: 1.0,
            p01: 0.0,
            p11: BIAS_SIGMA0 * BIAS_SIGMA0,
            initialized: false,
            accel: 0.0,
            accel_speed: 0.0,
            accel_ns: 0,
            accel_initialized: false,
            accel_reset: false,
            last_gnss_ns: 0,
            last_gnss_speed: None,
            history: std::collections::VecDeque::new(),
            predicted_ns: 0,
            prev_doppler: None,
            along_since_gnss: 0.0,
            sign_evidence: 0.0,
            gnss_reject_streak: 0,
            gnss_rejected: Vec::new(),
            prev_fix: None,
            last_used_ns: BTreeMap::new(),
            rejected: BTreeMap::new(),
            speed_refs: BTreeMap::new(),
            max_speed_mps: None,
            backward: false,
            ceiling_mps: None,
            leg: None,
            cars: Vec::new(),
            window_ns: 0,
            window_along_sum: 0.0,
            window_along_dt: 0.0,
        }
    }
}

impl LiveSpeedEstimator {
    pub fn new() -> Self {
        Self::default()
    }

    /// A fresh estimator for running backward in time from this one's latest state, for smoothing:
    /// it keeps the phone's frame (gravity, travel axis and its sign, the accelerometer bias along it), which need
    /// acceleration to learn and so can't be learned again at constant speed, adjusted for reversed time.
    /// It starts without a speed, so the two directions stay independent where they're combined.
    ///
    /// Feed it the accelerometer as recorded and the gyro negated. In reversed time velocity reverses
    /// and rotation runs backward, but acceleration is the same vector, so along the flipped axis the speed changes
    /// at minus the forward rate, as it should; the bias along the flipped axis changes sign.
    pub fn reversed(&self) -> Self {
        Self {
            gravity: self.gravity,
            gravity_init: self.gravity_init,
            axis_mean: self.axis_mean,
            cov: self.cov,
            axis: self.axis.map(|a| -a),
            axis_known: self.axis_known,
            axis_sign_known: self.axis_sign_known,
            // Speed changes reverse with time, and the acceleration doesn't, so the correlation flips like the axis.
            axis_cross: self.axis_cross.map(|c| -c),
            axis_from_gnss: self.axis_from_gnss,
            b: -self.b,
            p11: self.p11,
            max_speed_mps: self.max_speed_mps,
            backward: !self.backward,
            // The phone's along-axis acceleration flips with the axis, so the car's correlation with it does too.
            cars: self
                .cars
                .iter()
                .map(|c| CarAxis {
                    cross: c.cross.map(|x| -x),
                    along_mean: -c.along_mean,
                    last_ns: 0,
                    pending_sum: 0.0,
                    pending_dt: 0.0,
                    car_sum: 0.0,
                    car_n: 0,
                    ..c.clone()
                })
                .collect(),
            ..Self::default()
        }
    }

    /// Up in the phone's frame (a unit vector, opposite to gravity), once the accelerometer has given one:
    /// the gyro's rate about it is the turn rate of the direction of travel.
    pub fn up(&self) -> Option<[f64; 3]> {
        let g = norm(&self.gravity);
        (self.gravity_init && g > 0.0).then(|| self.gravity.map(|x| x / g))
    }

    /// The fastest the vehicle can go (m/s), e.g. [`SUBWAY_MAX_SPEED_MPS`] on the subway, or `None` for no limit.
    /// Underground, with no GNSS, a small error in the accelerometer bias otherwise integrates without bound;
    /// a speed over the limit is projected back onto it, and the bias moves with it by their correlation.
    /// The projection leaves the covariance alone: a bound says the speed is at most the limit, not that it's there,
    /// so it mustn't make the estimate more certain (it would then outweigh the backward pass when smoothing).
    pub fn set_max_speed(&mut self, max_speed_mps: Option<f64>) {
        self.max_speed_mps = max_speed_mps;
    }

    /// Gyro sample (rad/s), bias already removed.
    pub fn on_gyro(&mut self, wx: f64, wy: f64, wz: f64) {
        self.omega = [wx, wy, wz];
    }

    /// Accelerometer sample with gravity included (m/s^2), bias already removed, on the elapsed realtime clock.
    pub fn on_accel(&mut self, t_ns: i64, ax: f64, ay: f64, az: f64) {
        if !self.gravity_init {
            self.gravity = [ax, ay, az];
            self.gravity_init = true;
        }
        if self.last_ns == 0 || t_ns <= self.last_ns {
            self.last_ns = self.last_ns.max(t_ns);
            return;
        }
        let dt = ((t_ns - self.last_ns) as f64 * 1e-9).min(MAX_DT_S);
        self.last_ns = t_ns;

        // Stillness from the variance of |a|.
        let mag = (ax * ax + ay * ay + az * az).sqrt();
        let k_mag = 1.0 - (-dt / STILL_WINDOW_S).exp();
        let d = mag - self.mag_mean;
        self.mag_mean += k_mag * d;
        self.mag_var = (1.0 - k_mag) * (self.mag_var + k_mag * d * d);
        let still = self.mag_var.sqrt() < STILL_STD_MPS2;
        if !still {
            self.still_since_ns = 0;
        } else if self.still_since_ns == 0 {
            self.still_since_ns = t_ns;
        }

        // World-fixed vectors seen from a rotating phone turn the other way: v' = v - (omega x v) dt.
        rotate_world_fixed(&self.omega, &mut self.gravity, dt);
        // The travel axis is fixed to the vehicle, not the world: a curve turns the vehicle about the vertical with
        // the phone, so only the phone's tilting (rotation about horizontal axes) moves the axis in its frame.
        // A turn of the phone in hand about the vertical looks the same as a curve; GNSS re-learns the axis then.
        let g_norm = norm(&self.gravity);
        // Turning about the vertical faster than any train turns is the phone turning in a hand: the axis turns
        // with it then, or it would stay where the phone was pointing.
        let tilting = if g_norm > 1e-6 {
            let up = self.gravity.map(|x| x / g_norm);
            let about_up = dot(&self.omega, &up);
            if about_up.abs() > HANDLED_YAW_RAD_S {
                self.omega
            } else {
                std::array::from_fn(|i| self.omega[i] - about_up * up[i])
            }
        } else {
            self.omega
        };
        rotate_world_fixed(&tilting, &mut self.axis, dt);
        rotate_world_fixed(&tilting, &mut self.axis_mean, dt);
        rotate_world_fixed(&tilting, &mut self.axis_cross, dt);
        rotate_world_fixed(&tilting, &mut self.h_since_gnss, dt);
        self.rotate_cov(&tilting, dt);
        let gnss_recent =
            self.last_gnss_ns != 0 && (t_ns - self.last_gnss_ns) as f64 * 1e-9 < GNSS_RECENT_S;
        let w = norm(&self.omega);
        self.turning += (1.0 - (-dt / HANDLED_TAU_S).exp()) * (w - self.turning);
        let tau = if still {
            GRAVITY_TAU_STILL_S
        } else if !self.initialized && self.turning > HANDLED_RAD_S {
            GRAVITY_TAU_HANDLED_S
        } else if gnss_recent {
            GRAVITY_TAU_MOVING_S
        } else {
            GRAVITY_TAU_UNAIDED_S
        };
        let k_g = 1.0 - (-dt / tau).exp();
        // The vehicle's own acceleration (the fused speed's rate, signed along the axis) isn't tilt.
        let linear = if self.axis_known && !still && gnss_recent {
            self.accel * self.v.signum()
        } else {
            0.0
        };
        for ((g, a), u) in self.gravity.iter_mut().zip([ax, ay, az]).zip(self.axis) {
            *g += k_g * (a - linear * u - *g);
        }
        let g_norm = norm(&self.gravity);
        if g_norm < 1e-6 {
            return;
        }
        let g = self.gravity.map(|x| x / g_norm);
        let l = [
            ax - self.gravity[0],
            ay - self.gravity[1],
            az - self.gravity[2],
        ];
        let vertical = dot(&l, &g);
        // On a curve, the vehicle's centripetal acceleration: its turn rate about the vertical crossed with its
        // velocity, sideways to the travel axis (about 1.5 m/s^2 at 40 mph on a subway curve). Taken out, so it
        // neither turns the axis learned from the spread nor leaks into the travel direction through the axis's
        // error (40 to 55 mph on the curve before Jefferson St, `20261024-003357`).
        let yaw = dot(&self.omega, &g);
        let centripetal = if self.axis_known
            && self.initialized
            && self.axis_sign_known
            && yaw.abs() < CENTRIPETAL_MAX_YAW_RAD_S
        {
            let up_x_axis = cross(&g, &self.axis);
            up_x_axis.map(|x| yaw * self.v * x)
        } else {
            [0.0; 3]
        };
        let h = [
            l[0] - vertical * g[0] - centripetal[0],
            l[1] - vertical * g[1] - centripetal[1],
            l[2] - vertical * g[2] - centripetal[2],
        ];
        // Once GNSS has given the axis, the gyro carries it: the spread would drift to sway while cruising.
        if !still && !self.axis_from_gnss {
            self.update_axis(&h, dt);
        }
        for (s, x) in self.h_since_gnss.iter_mut().zip(h) {
            *s += x * dt;
        }
        self.h_since_gnss_s += dt;
        let along = if self.axis_known {
            dot(&h, &self.axis)
        } else {
            0.0
        };

        self.predict(along, dt);
        self.predicted_ns = t_ns;
        self.mark_used(Sensor::Imu, t_ns);
        // The bounds are for the IMU alone: with recent GNSS, its speed stands (a train above 55 mph on a downgrade,
        // or speeding up faster than the curves, is what GNSS measured).
        let gnss_recent =
            self.last_gnss_ns != 0 && (t_ns - self.last_gnss_ns) as f64 * 1e-9 < GNSS_RECENT_S;
        if let Some(max) = self.max_speed_mps
            && self.v.abs() > max
            && !gnss_recent
        {
            let y = self.v.signum() * max - self.v;
            if self.p00 > 0.0 {
                self.b += self.p01 / self.p00 * y;
            }
            self.v += y;
            self.mark_used(Sensor::SpeedLimit, t_ns);
        }
        if !gnss_recent {
            self.limit_accel(t_ns, dt);
        }
        self.limit_on_leg(t_ns, dt);
        self.along_since_gnss += along * dt;
        for c in &mut self.cars {
            c.pending_sum += along * dt;
            c.pending_dt += dt;
        }
        if self.window_ns != 0 {
            self.window_along_sum += along * dt;
            self.window_along_dt += dt;
        }

        let was_stopped = self.stopped;
        self.stopped =
            self.still_since_ns != 0 && (t_ns - self.still_since_ns) as f64 * 1e-9 >= STILL_S;
        if self.stopped {
            // Like a door cycle, not gated without recent GNSS: underground the speed's sigma grows to a few m/s,
            // so the gate rejected the quiet phone's zero exactly when the speed had run away
            // (`rejected as outliers: stop`, standing on the platforms at 6 Av and 14 St on `20261023-193658`).
            let gnss_recent =
                self.last_gnss_ns != 0 && (t_ns - self.last_gnss_ns) as f64 * 1e-9 < GNSS_RECENT_S;
            let r = STOP_SIGMA_MPS * STOP_SIGMA_MPS;
            if gnss_recent {
                self.update(0.0, r, Sensor::Stop, t_ns);
            } else {
                self.update_ungated(0.0, r, Sensor::Stop, t_ns);
            }
            self.since_stop_along = 0.0;
        } else {
            if was_stopped {
                self.since_stop_along = 0.0;
            }
            self.since_stop_along += along * dt;
            // A departure accelerates forward, which fixes the axis sign if GNSS hasn't.
            if !self.axis_sign_known
                && self.axis_known
                && self.since_stop_along.abs() > DEPARTURE_EVIDENCE_MPS
            {
                if self.since_stop_along < 0.0 {
                    self.flip_axis();
                }
                self.axis_sign_known = true;
            }
        }
        if self
            .history
            .back()
            .is_none_or(|&(t, _)| (t_ns - t) as f64 * 1e-9 >= HISTORY_EVERY_S)
        {
            self.history.push_back((t_ns, self.v));
            while self
                .history
                .front()
                .is_some_and(|&(t, _)| (t_ns - t) as f64 * 1e-9 > HISTORY_S)
            {
                self.history.pop_front();
            }
        }
        self.track_accel(t_ns);
    }

    /// A reading of a steady car's own accelerometer (m/s^2, the beacon's frame, gravity included), at `t_ns`.
    /// Each car's forward axis and its sign are learned from how it follows the phone's along-axis acceleration;
    /// then, over each [`CAR_WINDOW_S`], the phone's mean along-axis acceleration minus the cars' mean forward one
    /// (from each car's level at the last stop) measures the phone's bias, which no tilt of the phone's gravity
    /// can fool. In reversed time, feed it the same readings: the learned signs flip with the axis.
    pub fn on_car_accel(&mut self, t_ns: i64, car: u16, a: [f64; 3]) {
        self.cars.retain(|c| {
            c.last_ns == 0 || ((t_ns - c.last_ns).unsigned_abs() as f64) * 1e-9 < CAR_STALE_S
        });
        let i = match self.cars.iter().position(|c| c.car == car) {
            Some(i) => i,
            None => {
                if self.cars.len() >= CARS_MAX {
                    let oldest = (0..self.cars.len())
                        .min_by_key(|&i| self.cars[i].last_ns)
                        .expect("not empty");
                    self.cars.swap_remove(oldest);
                }
                self.cars.push(CarAxis {
                    car,
                    mean: a,
                    slow_mean: a,
                    ..CarAxis::default()
                });
                self.cars.len() - 1
            }
        };
        let stopped = self.stopped;
        let c = &mut self.cars[i];
        let along = (c.pending_dt > 0.0).then(|| c.pending_sum / c.pending_dt);
        c.pending_sum = 0.0;
        c.pending_dt = 0.0;
        let dt = if c.last_ns == 0 {
            0.0
        } else {
            (t_ns - c.last_ns).unsigned_abs() as f64 * 1e-9
        };
        c.last_ns = t_ns;
        let Some(along) = along.filter(|_| dt > 0.0 && dt < 2.0) else {
            return;
        };
        let km = 1.0 - (-dt / CAR_MEAN_TAU_S).exp();
        let kc = 1.0 - (-dt / CAR_CORR_TAU_S).exp();
        for (m, x) in c.mean.iter_mut().zip(a) {
            *m += km * (x - *m);
        }
        c.along_mean += km * (along - c.along_mean);
        let da = along - c.along_mean;
        for (k, &x) in a.iter().enumerate() {
            let dk = x - c.mean[k];
            c.cross[k] += kc * (dk * da - c.cross[k]);
            c.var[k] += kc * (dk * dk - c.var[k]);
            c.slow_mean[k] += kc * (x - c.slow_mean[k]);
        }
        c.along_var += kc * (da * da - c.along_var);
        c.readings += 1;
        if stopped {
            let kl = 1.0 - (-dt / CAR_LEVEL_TAU_S).exp();
            let level = c.level.get_or_insert(a);
            for (l, x) in level.iter_mut().zip(a) {
                *l += kl * (x - *l);
            }
        }
        let level = c.level.unwrap_or(c.slow_mean);
        let Some((axis, sign)) = c.forward() else {
            return;
        };
        c.car_sum += sign * (a[axis] - level[axis]);
        c.car_n += 1;
        if self.window_ns == 0 {
            self.window_ns = t_ns;
            return;
        }
        if ((t_ns - self.window_ns).unsigned_abs() as f64) * 1e-9 < CAR_WINDOW_S
            || self.window_along_dt <= 0.0
        {
            return;
        }
        let means: Vec<f64> = self
            .cars
            .iter()
            .filter(|c| c.car_n > 0)
            .map(|c| c.car_sum / f64::from(c.car_n))
            .collect();
        let n = means.len() as f64;
        let z = self.window_along_sum / self.window_along_dt - means.iter().sum::<f64>() / n;
        self.window_ns = 0;
        self.window_along_sum = 0.0;
        self.window_along_dt = 0.0;
        for c in &mut self.cars {
            c.car_sum = 0.0;
            c.car_n = 0;
        }
        let gnss_recent =
            self.last_gnss_ns != 0 && (t_ns - self.last_gnss_ns) as f64 * 1e-9 < GNSS_RECENT_S;
        if USE_CAR_ACCEL && !stopped && !gnss_recent {
            let r = CAR_SIGMA_MPS2 * CAR_SIGMA_MPS2 * (CAR_SHARED + (1.0 - CAR_SHARED) / n);
            self.update_bias(z, r, Sensor::Car, t_ns);
        }
    }

    /// A gated measurement of the along-axis accelerometer bias.
    fn update_bias(&mut self, z: f64, r: f64, sensor: Sensor, t_ns: i64) {
        let y = z - self.b;
        let s = self.p11 + r;
        if y * y > GATE_SIGMA * GATE_SIGMA * s {
            *self.rejected.entry(sensor).or_default() += 1;
            return;
        }
        let k0 = self.p01 / s;
        let k1 = self.p11 / s;
        self.v += k0 * y;
        self.b += k1 * y;
        let n00 = self.p00 - k0 * self.p01;
        let n01 = (1.0 - k1) * self.p01;
        let n11 = (1.0 - k1) * self.p11;
        self.p00 = n00;
        self.p01 = n01;
        self.p11 = n11;
        self.mark_used(sensor, t_ns);
    }

    /// The train stood still at `stopped_ns`, a few seconds ago (a door cycle's closing, found once the
    /// closing step is complete). The speed since then is the IMU's integral from there, so the zero applies
    /// as a measurement that the speed now is its change since then.
    /// Underground it isn't gated: after a minute without GNSS the speed's sigma is a couple of m/s, so the gate
    /// would reject exactly the stops that end a runaway. With recent GNSS it's gated like any other.
    pub fn on_stopped_at(&mut self, stopped_ns: i64, now_ns: i64) {
        let Some(&(_, then)) = self.history.iter().rev().find(|&&(t, _)| t <= stopped_ns) else {
            return;
        };
        self.inflate_to(now_ns);
        let z = self.v - then;
        let r = DOOR_STOP_SIGMA_MPS * DOOR_STOP_SIGMA_MPS;
        let gnss_recent =
            self.last_gnss_ns != 0 && (now_ns - self.last_gnss_ns) as f64 * 1e-9 < GNSS_RECENT_S;
        let before = self.v;
        if gnss_recent {
            self.update(z, r, Sensor::Doors, now_ns);
        } else {
            self.update_ungated(z, r, Sensor::Doors, now_ns);
        }
        // The speeds kept since were the same integral, so they move with the correction.
        let shift = self.v - before;
        for (t, v) in self.history.iter_mut() {
            if *t >= stopped_ns {
                *v += shift;
            }
        }
        self.track_accel(now_ns);
    }

    /// The last plausible GNSS speed (m/s) and its time.
    pub fn last_gnss_speed(&self) -> Option<(i64, f64)> {
        self.last_gnss_speed
    }

    /// The last time a measurement (GNSS, a stop, or a door cycle) set the speed, not just the IMU.
    pub fn last_measured_ns(&self) -> Option<i64> {
        [
            Sensor::GnssDoppler,
            Sensor::GnssPosition,
            Sensor::Stop,
            Sensor::Doors,
            Sensor::Walking,
        ]
        .iter()
        .filter_map(|s| self.last_used_ns.get(s).copied())
        .max()
    }

    /// The rider is walking or running (see [`crate::walking`]), so moving at that pace, whichever way.
    /// Like a door cycle, it isn't gated without recent GNSS, which is when the IMU alone runs away.
    pub fn on_walking(&mut self, t_ns: i64, running: bool) {
        self.inflate_to(t_ns);
        let (pace, sigma) = if running {
            (RUNNING_MPS, RUNNING_SIGMA_MPS)
        } else {
            (WALKING_MPS, WALKING_SIGMA_MPS)
        };
        let z = if self.v < 0.0 { -pace } else { pace };
        let r = sigma * sigma;
        let gnss_recent =
            self.last_gnss_ns != 0 && (t_ns - self.last_gnss_ns) as f64 * 1e-9 < GNSS_RECENT_S;
        // A walking pace says little about the accelerometer's bias (the phone swings with each step), so it keeps it.
        let (b, p11) = (self.b, self.p11);
        if gnss_recent {
            self.update(z, r, Sensor::Walking, t_ns);
        } else {
            self.update_ungated(z, r, Sensor::Walking, t_ns);
        }
        self.b = b;
        self.p11 = p11;
        self.track_accel(t_ns);
    }

    /// The train is standing now (in the smoother's backward pass, across a door cycle), not gated.
    pub fn on_known_stop(&mut self, t_ns: i64) {
        self.inflate_to(t_ns);
        self.update_ungated(
            0.0,
            DOOR_STOP_SIGMA_MPS * DOOR_STOP_SIGMA_MPS,
            Sensor::Doors,
            t_ns,
        );
        self.track_accel(t_ns);
    }

    /// A GNSS fix: Doppler speed if the fix has one, and position for position-differenced speed.
    /// A speed (m/s) and its standard deviation from a velocity solution of our own (e.g. `ride_gnss`'s from
    /// raw Doppler), used the same way as a fix's Doppler speed.
    pub fn on_gnss_speed(&mut self, t_ns: i64, speed_mps: f64, sigma_mps: f64) {
        self.last_gnss_ns = t_ns;
        self.inflate_to(t_ns);
        self.on_doppler_speed(t_ns, speed_mps, sigma_mps);
        self.track_accel(t_ns);
    }

    pub fn on_gnss_fix(
        &mut self,
        t_ns: i64,
        lat_deg: f64,
        lon_deg: f64,
        horizontal_accuracy_m: Option<f64>,
        speed_mps: Option<f64>,
        speed_accuracy_mps: Option<f64>,
    ) {
        self.last_gnss_ns = t_ns;
        self.inflate_to(t_ns);
        if let Some(speed) = speed_mps {
            self.on_doppler_speed(
                t_ns,
                speed,
                speed_accuracy_mps.unwrap_or(DEFAULT_DOPPLER_SIGMA_MPS),
            );
        }
        if let Some(accuracy_m) = horizontal_accuracy_m {
            let fix = Fix {
                t_ns,
                lat: lat_deg,
                lon: lon_deg,
                accuracy_m,
            };
            if let Some(prev) = self.prev_fix {
                let dt = (t_ns - prev.t_ns) as f64 * 1e-9;
                if (0.5..=POSITION_MAX_DT_S).contains(&dt) {
                    let speed = distance_m(&prev, &fix) / dt;
                    let sigma = prev.accuracy_m.hypot(fix.accuracy_m) / dt;
                    if sigma < POSITION_MAX_SIGMA_MPS {
                        self.measure_speed_magnitude(speed, sigma, Sensor::GnssPosition, t_ns);
                    }
                }
            }
            self.prev_fix = Some(fix);
        }
        self.track_accel(t_ns);
    }

    pub fn estimate(&self, now_ns: i64) -> Estimate {
        Estimate {
            speed_mps: self.v.abs(),
            sigma_mps: self.p00.max(0.0).sqrt(),
            sensors: self
                .last_used_ns
                .iter()
                .filter(|&(_, &t)| ((now_ns - t) as f64 * 1e-9) < RECENT_S)
                .map(|(&s, _)| s)
                .collect(),
            stopped: self.stopped,
            since_gnss_s: (self.last_gnss_ns != 0)
                .then(|| (now_ns - self.last_gnss_ns) as f64 * 1e-9),
            rejected: self.rejected.clone(),
            initialized: self.initialized,
            accel_mps2: self.accel,
            max_speed_mps: self.max_speed_mps,
        }
    }

    fn on_doppler_speed(&mut self, t_ns: i64, speed: f64, sigma: f64) {
        if let Some((t0, v0, s0)) = self.prev_doppler
            && v0.min(speed) >= AXIS_GNSS_MIN_SPEED_MPS
        {
            self.learn_axis_from_gnss(t_ns - t0, speed - v0, s0.max(sigma));
        }
        // Speed change between fixes vs integrated along-axis acceleration fixes the axis sign.
        if let Some((_, prev, _)) = self.prev_doppler
            && self.axis_known
            && !self.axis_sign_known
        {
            let dv = speed - prev;
            if dv.abs() > 0.3 {
                self.sign_evidence += dv * self.along_since_gnss;
                if self.sign_evidence.abs() > SIGN_EVIDENCE {
                    if self.sign_evidence < 0.0 {
                        self.flip_axis();
                    }
                    self.axis_sign_known = true;
                }
            }
        }
        self.prev_doppler = Some((t_ns, speed, sigma));
        self.along_since_gnss = 0.0;
        self.h_since_gnss = [0.0; 3];
        self.h_since_gnss_s = 0.0;
        self.measure_speed_magnitude(
            speed,
            sigma.max(MIN_DOPPLER_SIGMA_MPS),
            Sensor::GnssDoppler,
            t_ns,
        );
    }

    /// GNSS measures speed magnitude; the filter's speed is signed along the axis, so match the sign.
    fn measure_speed_magnitude(&mut self, speed: f64, sigma: f64, sensor: Sensor, t_ns: i64) {
        if !(0.0..=MAX_SPEED_MPS).contains(&speed) || !sigma.is_finite() {
            *self.rejected.entry(sensor).or_default() += 1;
            return;
        }
        if !self.plausible_change(sensor, t_ns, speed, sigma) {
            *self.rejected.entry(sensor).or_default() += 1;
            return;
        }
        self.last_gnss_speed = Some((t_ns, speed));
        let signed = if self.v < 0.0 && !self.axis_sign_known {
            -speed
        } else {
            speed
        };
        if !self.initialized {
            self.v = signed;
            self.p00 = sigma * sigma;
            self.initialized = true;
            self.mark_used(sensor, t_ns);
            return;
        }
        if self.update(signed, sigma * sigma, sensor, t_ns) {
            self.gnss_reject_streak = 0;
            self.gnss_rejected.clear();
        } else {
            self.gnss_reject_streak += 1;
            self.gnss_rejected.push(signed);
            if self.gnss_reject_streak >= GNSS_REINIT_REJECTS {
                // Several GNSS fixes in a row disagree with the IMU-driven prediction: trust GNSS, by their median,
                // so one bad speed among them (it happens, with a small claimed sigma) isn't what the filter restarts on.
                let mut rejected = std::mem::take(&mut self.gnss_rejected);
                rejected.sort_by(f64::total_cmp);
                let median = rejected[rejected.len() / 2];
                let spread = rejected
                    .iter()
                    .map(|v| (v - median).abs())
                    .fold(0.0, f64::max);
                self.v = median;
                self.b = 0.0;
                self.p00 = sigma.max(spread).powi(2);
                self.p01 = 0.0;
                self.p11 = BIAS_SIGMA0 * BIAS_SIGMA0;
                self.accel_reset = true;
                self.gnss_reject_streak = 0;
                self.mark_used(sensor, t_ns);
            }
        }
    }

    /// Whether reaching `speed` from this sensor's last plausible speed takes at most [`MAX_ACCEL_MPS2`],
    /// allowing for both speeds' uncertainty; records it as the new reference if so.
    fn plausible_change(&mut self, sensor: Sensor, t_ns: i64, speed: f64, sigma: f64) -> bool {
        let reachable = |t0: i64, v0: f64, s0: f64| {
            let dt = (t_ns - t0).max(0) as f64 * 1e-9;
            (speed - v0).abs() <= MAX_ACCEL_MPS2 * dt + GATE_SIGMA * s0.hypot(sigma)
        };
        let fresh = SpeedRef {
            t_ns,
            speed,
            sigma,
            implausible: 0,
            last_implausible: None,
        };
        let Some(r) = self.speed_refs.get_mut(&sensor) else {
            self.speed_refs.insert(sensor, fresh);
            return true;
        };
        if reachable(r.t_ns, r.speed, r.sigma) {
            *r = fresh;
            return true;
        }
        // Implausible; but a run of them consistent with each other means the reference was wrong.
        let consistent = r
            .last_implausible
            .is_none_or(|(t0, v0, s0)| reachable(t0, v0, s0));
        r.implausible = if consistent { r.implausible + 1 } else { 1 };
        r.last_implausible = Some((t_ns, speed, sigma));
        if r.implausible >= IMPLAUSIBLE_RESET {
            *r = fresh;
            return true;
        }
        false
    }

    /// A train left a station underground at `since_ns`, the next stations its line stops at `stations_m` along
    /// the track ([`crate::track::stations_ahead_m`]): until [`Self::end_leg`], the speed is capped as on
    /// [`Self::limit_on_leg`].
    /// `recorded_mps` is the most a recorded train went to the next station
    /// ([`crate::track::recorded_peak_mph`]), if known: the cap to it is no higher, with [`RECORDED_MARGIN_MPS`]
    /// or [`RECORDED_MARGIN_SHARE`].
    pub fn start_leg(&mut self, since_ns: i64, stations_m: &[f64], recorded_mps: Option<f64>) {
        self.leg = (!stations_m.is_empty()).then(|| Leg {
            since_ns,
            stations: stations_m
                .iter()
                .enumerate()
                .map(|(i, &d)| {
                    let peak = subway_peak_mps(d);
                    let peak = match recorded_mps {
                        Some(r) if i == 0 => {
                            peak.min(r + RECORDED_MARGIN_MPS.max(r * RECORDED_MARGIN_SHARE))
                        }
                        _ => peak,
                    };
                    (crate::track::least_s(d), peak)
                })
                .collect(),
            covered_m: 0.0,
            known: (since_ns, 0.0),
        });
    }

    /// On a leg, the train is known to be `along_m` along the track from the departure at `t_ns`
    /// (standing at a station, or passing one the radio names), with `weight` (0 to 1) for how sure that is.
    /// Since the last known place, where the speed was right (0 at the departure), a steady bias error grows the
    /// speed error as `db t` and the distance's as `db t^2 / 2`: so the estimate's distance over the known one
    /// gives the bias, corrected by half of it times `weight` (at most [`ANCHOR_MAX_DB_MPS2`]), and the speed by
    /// the error it made since. The distance counts from the known place on.
    pub fn anchor(&mut self, t_ns: i64, along_m: f64, weight: f64) {
        let Some(leg) = self.leg.as_mut() else {
            return;
        };
        let (known_ns, known_m) = leg.known;
        let t = (t_ns - known_ns) as f64 * 1e-9;
        let (estimated, known) = (leg.covered_m - known_m, along_m - known_m);
        leg.covered_m = along_m;
        leg.known = (t_ns, along_m);
        if t < ANCHOR_MIN_S || known <= 0.0 {
            return;
        }
        let gain = 0.5 * weight.clamp(0.0, 1.0);
        let db =
            (2.0 * (estimated - known) / (t * t)).clamp(-ANCHOR_MAX_DB_MPS2, ANCHOR_MAX_DB_MPS2);
        // The speed is signed along the axis; the distance is its magnitude's.
        let sign = if self.v < 0.0 { -1.0 } else { 1.0 };
        self.b += sign * gain * db;
        let dv = gain * db * t;
        if self.v.abs() > dv.abs() || dv < 0.0 {
            self.v -= sign * dv;
        } else {
            self.v = 0.0;
        }
        self.mark_used(Sensor::Anchor, t_ns);
    }

    /// The train stopped (or the way ahead isn't known): the leg's cap is off.
    pub fn end_leg(&mut self) {
        self.leg = None;
    }

    /// On a leg underground without recent GNSS, from known inputs only (the departure's time, the stations'
    /// places on the track, the learned curves), never the estimate itself: the speed is at most what a train
    /// reaches in the time since the departure ([`subway_reach_by_time_mps`]), and at most the fastest a train goes
    /// on the way to the next station it stops at ([`subway_peak_mps`]). Once a train could have stood at that
    /// station and no stop was found, it may have run through, so the one after counts, which only loosens the cap;
    /// past the last station known, the leg ends. The cap is projected like [`Self::set_max_speed`]'s bound.
    fn limit_on_leg(&mut self, t_ns: i64, dt: f64) {
        let gnss_recent =
            self.last_gnss_ns != 0 && (t_ns - self.last_gnss_ns) as f64 * 1e-9 < GNSS_RECENT_S;
        let Some(leg) = self.leg.as_mut() else {
            return;
        };
        leg.covered_m += self.v.abs() * dt;
        if self.backward || self.max_speed_mps != Some(SUBWAY_MAX_SPEED_MPS) || gnss_recent {
            return;
        }
        let since_s = (t_ns - leg.since_ns) as f64 * 1e-9;
        while leg
            .stations
            .front()
            .is_some_and(|&(least, _)| least < since_s)
        {
            leg.stations.pop_front();
        }
        let Some(&(_, peak)) = leg.stations.front() else {
            self.leg = None;
            return;
        };
        let cap = subway_reach_by_time_mps(since_s)
            .min(peak)
            .max(LEG_CAP_FLOOR_MPS);
        if self.v.abs() > cap {
            let y = self.v.signum() * cap - self.v;
            if self.p00 > 0.0 {
                self.b += self.p01 / self.p00 * y;
            }
            self.v += y;
            self.mark_used(Sensor::SpeedLimit, t_ns);
        }
    }

    /// On the subway (its top speed is the bound), keeps the speed under the most a train could reach
    /// since the last stop or GNSS speed: the ceiling starts there and rises along [`subway_accel_limits`],
    /// and a speed over it is projected back like [`Self::set_max_speed`]'s bound.
    /// In the backward pass, rising is braking, so it's the most the train could have braked from before a stop.
    /// Only rising is bounded: an estimate that has run too high must stay free to come back down
    /// (bounding the fall too held the backward pass's estimate at 55 mph on `154951` with GNSS cut).
    fn limit_accel(&mut self, t_ns: i64, dt: f64) {
        if self.max_speed_mps != Some(SUBWAY_MAX_SPEED_MPS) {
            self.ceiling_mps = None;
            return;
        }
        let Some(c) = self.ceiling_mps.as_mut() else {
            return;
        };
        let (motoring, braking) = subway_accel_limits(*c);
        *c += if self.backward { braking } else { motoring } * dt;
        let bounded = self.v.abs().min(*c + ACCEL_LIMIT_SLACK_MPS);
        if bounded != self.v.abs() {
            let y = self.v.signum() * bounded - self.v;
            if self.p00 > 0.0 {
                self.b += self.p01 / self.p00 * y;
            }
            self.v += y;
            self.mark_used(Sensor::SpeedLimit, t_ns);
        }
    }

    fn predict(&mut self, along: f64, dt: f64) {
        // Nothing this records sustains more than 1 g along the track; more is the phone being knocked or shaken.
        let along = along.clamp(-MAX_ACCEL_MPS2, MAX_ACCEL_MPS2);
        // The bias is along the axis, like the signed speed, so it's subtracted whichever way is forward.
        self.v += (along - self.b) * dt;
        // P = F P F^T + Q with F = [[1, -dt], [0, 1]].
        let n00 =
            self.p00 - 2.0 * dt * self.p01 + dt * dt * self.p11 + ACCEL_NOISE * ACCEL_NOISE * dt;
        let n01 = self.p01 - dt * self.p11;
        let n11 = self.p11 + BIAS_WALK * BIAS_WALK * dt;
        self.p00 = n00;
        self.p01 = n01;
        self.p11 = n11;
    }

    /// Grows the covariance over time the IMU didn't cover (no IMU data, or before its first sample),
    /// so speed can change between fixes without every fix being rejected as an outlier.
    /// Not in the app's Kotlin version, which always has the IMU running.
    fn inflate_to(&mut self, t_ns: i64) {
        if self.predicted_ns != 0 && t_ns > self.predicted_ns {
            let dt = (t_ns - self.predicted_ns) as f64 * 1e-9;
            let n00 = self.p00 - 2.0 * dt * self.p01
                + dt * dt * self.p11
                + ACCEL_NOISE * ACCEL_NOISE * dt;
            let n01 = self.p01 - dt * self.p11;
            self.p00 = n00;
            self.p01 = n01;
            self.p11 += BIAS_WALK * BIAS_WALK * dt;
        }
        self.predicted_ns = self.predicted_ns.max(t_ns);
    }

    /// Scalar speed measurement with gating; returns whether it was accepted.
    fn update(&mut self, z: f64, r: f64, sensor: Sensor, t_ns: i64) -> bool {
        let y = z - self.v;
        if y * y > GATE_SIGMA * GATE_SIGMA * (self.p00 + r) {
            *self.rejected.entry(sensor).or_default() += 1;
            return false;
        }
        self.update_ungated(z, r, sensor, t_ns);
        true
    }

    fn update_ungated(&mut self, z: f64, r: f64, sensor: Sensor, t_ns: i64) {
        let y = z - self.v;
        let s = self.p00 + r;
        let k0 = self.p00 / s;
        let k1 = self.p01 / s;
        self.v += k0 * y;
        self.b += k1 * y;
        let n00 = (1.0 - k0) * self.p00;
        let n01 = (1.0 - k0) * self.p01;
        let n11 = self.p11 - k1 * self.p01;
        self.p00 = n00;
        self.p01 = n01;
        self.p11 = n11;
        self.initialized = true;
        self.mark_used(sensor, t_ns);
    }

    /// Folds the change in fused speed since the last call into the low-passed acceleration.
    /// A correction adds its whole step at once; the low-pass spreads it over [`ACCEL_TAU_S`],
    /// the same as if it had come over many IMU samples.
    fn track_accel(&mut self, t_ns: i64) {
        let speed = self.v.abs();
        let jumped = self.accel_reset || self.initialized != self.accel_initialized;
        // A change at the same time as the last update (or earlier, out of order) waits for the next one.
        if t_ns <= self.accel_ns && !jumped {
            return;
        }
        if self.accel_ns != 0 && !jumped {
            let dt = (t_ns - self.accel_ns) as f64 * 1e-9;
            let k = 1.0 - (-dt / ACCEL_TAU_S).exp();
            self.accel += k * ((speed - self.accel_speed) / dt - self.accel);
        }
        self.accel_ns = self.accel_ns.max(t_ns);
        self.accel_speed = speed;
        self.accel_initialized = self.initialized;
        self.accel_reset = false;
    }

    fn mark_used(&mut self, sensor: Sensor, t_ns: i64) {
        self.last_used_ns.insert(sensor, t_ns);
        // A stop or a GNSS speed (a GNSS restart sets it outright) says where the speed is,
        // so the train's ceiling starts again from there; walking can happen in a moving car, so it doesn't.
        match sensor {
            Sensor::Stop | Sensor::Doors => self.ceiling_mps = Some(0.0),
            Sensor::GnssDoppler | Sensor::GnssPosition => self.ceiling_mps = Some(self.v.abs()),
            _ => {}
        }
    }

    fn flip_axis(&mut self) {
        self.axis = self.axis.map(|x| -x);
        self.v = -self.v;
        self.b = -self.b;
        self.since_stop_along = -self.since_stop_along;
        self.along_since_gnss = -self.along_since_gnss;
    }

    /// C' = (I - W dt) C (I - W dt)^T, to first order C - (W C - C W) dt, with W v = omega x v.
    fn rotate_cov(&mut self, o: &[f64; 3], dt: f64) {
        let w = [0.0, -o[2], o[1], o[2], 0.0, -o[0], -o[1], o[0], 0.0];
        let c = self.cov;
        for i in 0..3 {
            for j in 0..3 {
                let mut wc = 0.0;
                let mut cw = 0.0;
                for k in 0..3 {
                    wc += w[i * 3 + k] * c[k * 3 + j];
                    cw += c[i * 3 + k] * w[k * 3 + j];
                }
                self.cov[i * 3 + j] = c[i * 3 + j] - (wc - cw) * dt;
            }
        }
    }

    /// Folds one GNSS speed change (over `dt_ns`, worse sigma `sigma`) into the correlation with horizontal
    /// acceleration over the same time, and takes the axis from it once it's strong enough. Lateral sway doesn't
    /// follow speed changes, so it doesn't pull the axis; the sign comes with it, as GNSS speeding up is forward.
    fn learn_axis_from_gnss(&mut self, dt_ns: i64, dv: f64, sigma: f64) {
        let dt = dt_ns as f64 * 1e-9;
        if !(0.0..=AXIS_GNSS_MAX_DT_S).contains(&dt) || self.h_since_gnss_s < 0.5 * dt {
            return;
        }
        let a = dv / dt;
        if a.abs() > AXIS_GNSS_MAX_ACCEL_MPS2 {
            return;
        }
        let weight = (AXIS_GNSS_FULL_WEIGHT_SIGMA_MPS / sigma).powi(2).min(1.0);
        let k = weight * (1.0 - (-dt / AXIS_GNSS_TAU_S).exp());
        for (c, h) in self.axis_cross.iter_mut().zip(self.h_since_gnss) {
            *c += k * (h / self.h_since_gnss_s * a - *c);
        }
        let n = norm(&self.axis_cross);
        if n < AXIS_GNSS_MIN_CROSS {
            return;
        }
        let axis = self.axis_cross.map(|c| c / n);
        // The speed is signed along the axis; keep it (and the bias) meaning the same if the axis turns around.
        if self.axis_known && dot(&axis, &self.axis) < 0.0 {
            self.flip_axis();
        }
        self.axis = axis;
        self.axis_known = true;
        self.axis_sign_known = true;
        self.axis_from_gnss = true;
    }

    /// Exponentially weighted covariance of horizontal acceleration, and its principal axis by power iteration.
    fn update_axis(&mut self, h: &[f64; 3], dt: f64) {
        let k = 1.0 - (-dt / AXIS_TAU_S).exp();
        for (m, x) in self.axis_mean.iter_mut().zip(h) {
            *m += k * (x - *m);
        }
        let d: [f64; 3] = std::array::from_fn(|i| h[i] - self.axis_mean[i]);
        for i in 0..3 {
            for j in 0..3 {
                self.cov[i * 3 + j] += k * (d[i] * d[j] - self.cov[i * 3 + j]);
            }
        }
        let c = &self.cov;
        let mut x = self.axis;
        for _ in 0..3 {
            let next = [
                c[0] * x[0] + c[1] * x[1] + c[2] * x[2],
                c[3] * x[0] + c[4] * x[1] + c[5] * x[2],
                c[6] * x[0] + c[7] * x[1] + c[8] * x[2],
            ];
            let n = norm(&next);
            if n < 1e-12 {
                return;
            }
            x = next.map(|v| v / n);
        }
        // Keep the sign continuous with the previous axis, so a known sign stays known.
        if dot(&x, &self.axis) < 0.0 {
            x = x.map(|v| -v);
        }
        self.axis = x;
        if c[0] + c[4] + c[8] > AXIS_MIN_VARIANCE {
            self.axis_known = true;
        }
    }
}

fn rotate_world_fixed(omega: &[f64; 3], v: &mut [f64; 3], dt: f64) {
    let c = [
        omega[1] * v[2] - omega[2] * v[1],
        omega[2] * v[0] - omega[0] * v[2],
        omega[0] * v[1] - omega[1] * v[0],
    ];
    for i in 0..3 {
        v[i] -= c[i] * dt;
    }
}

fn cross(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(a: &[f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn distance_m(a: &Fix, b: &Fix) -> f64 {
    let r = 6_371_008.8;
    let p1 = a.lat.to_radians();
    let p2 = b.lat.to_radians();
    let dp = p2 - p1;
    let dl = (b.lon - a.lon).to_radians();
    let h = (dp / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.0).sin().powi(2);
    2.0 * r * h.min(1.0).sqrt().asin()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_train_reaches_speed_along_the_motoring_curve() {
        let mph = |m: f64| subway_reach_mps(m) / 0.44704;
        assert_eq!(subway_reach_mps(0.0), 0.0);
        // About 1.4 m/s^2 at first: 20 mph (8.9 m/s) takes about 28 m.
        assert!((18.0..23.0).contains(&mph(28.0)), "{}", mph(28.0));
        // A station spacing of 600 m allows under 50 mph, and it only rises.
        assert!((35.0..50.0).contains(&mph(600.0)), "{}", mph(600.0));
        assert!(
            (0..300)
                .all(|i| subway_reach_mps(i as f64 * 10.0)
                    <= subway_reach_mps(i as f64 * 10.0 + 10.0))
        );
        assert_eq!(subway_reach_mps(1e6), SUBWAY_MAX_SPEED_MPS);
        // About 6 s to 20 mph.
        let s20 = (0..200)
            .find(|&i| subway_reach_by_time_mps(i as f64 * 0.1) >= 20.0 * 0.44704)
            .unwrap();
        assert!((50..80).contains(&s20), "{s20}");
        // 600 m between stations: no train passes about 45 mph; 2.2 km, about 55.
        let peak = subway_peak_mps(600.0) / 0.44704;
        assert!((40.0..48.0).contains(&peak), "{peak}");
        assert!(subway_peak_mps(2200.0) / 0.44704 > 50.0);
    }
}

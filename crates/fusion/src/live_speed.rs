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

use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Sensor {
    Imu,
    GnssDoppler,
    GnssPosition,
    Stop,
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
/// Process noise: unmodeled along-axis acceleration (m/s^2 per sqrt(s)), and bias random walk.
const ACCEL_NOISE: f64 = 0.3;
const BIAS_WALK: f64 = 0.005;
const BIAS_SIGMA0: f64 = 0.1;
/// Time constant of the reported acceleration's low-pass, to steady it against vibration.
const ACCEL_TAU_S: f64 = 1.0;
const STOP_SIGMA_MPS: f64 = 0.05;
const DEFAULT_DOPPLER_SIGMA_MPS: f64 = 0.5;
const MIN_DOPPLER_SIGMA_MPS: f64 = 0.1;
const POSITION_MAX_DT_S: f64 = 5.0;
const POSITION_MAX_SIGMA_MPS: f64 = 3.0;
/// Mach 1 at sea level: nothing this records goes faster, so a faster measurement is always wrong,
/// even as the first one or during a re-initialization, when gating can't catch it.
pub const MAX_SPEED_MPS: f64 = 343.0;
/// Trains, buses, and planes in normal operation stay well under 1 g along the track,
/// so a GNSS speed that would take more than this to reach from the previous plausible one is wrong.
pub const MAX_ACCEL_MPS2: f64 = 9.80665;
/// After this many implausible GNSS speeds in a row that agree with each other,
/// the earlier reference was the wrong one (e.g. a glitched first fix): start over from them.
const IMPLAUSIBLE_RESET: u32 = 5;
pub const GATE_SIGMA: f64 = 3.0;
pub const GNSS_REINIT_REJECTS: u32 = 5;
pub const RECENT_S: f64 = 3.0;

/// Not thread-safe; feed it from one thread, in time order.
#[derive(Clone, Debug)]
pub struct LiveSpeedEstimator {
    // IMU front end.
    gravity: [f64; 3],
    gravity_init: bool,
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
}

impl Default for LiveSpeedEstimator {
    fn default() -> Self {
        Self {
            gravity: [0.0; 3],
            gravity_init: false,
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
            ..Self::default()
        }
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
        let tilting = if g_norm > 1e-6 {
            let up = self.gravity.map(|x| x / g_norm);
            let about_up = dot(&self.omega, &up);
            std::array::from_fn(|i| self.omega[i] - about_up * up[i])
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
        let tau = if still {
            GRAVITY_TAU_STILL_S
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
        let h = [
            l[0] - vertical * g[0],
            l[1] - vertical * g[1],
            l[2] - vertical * g[2],
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
        self.along_since_gnss += along * dt;

        let was_stopped = self.stopped;
        self.stopped =
            self.still_since_ns != 0 && (t_ns - self.still_since_ns) as f64 * 1e-9 >= STILL_S;
        if self.stopped {
            self.update(0.0, STOP_SIGMA_MPS * STOP_SIGMA_MPS, Sensor::Stop, t_ns);
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
        let s = self.p00 + r;
        if y * y > GATE_SIGMA * GATE_SIGMA * s {
            *self.rejected.entry(sensor).or_default() += 1;
            return false;
        }
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
        true
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

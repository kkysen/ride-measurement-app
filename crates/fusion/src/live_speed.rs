//! Live speed: a port of the app's `LiveSpeedEstimator.kt`, so recorded rides can be replayed
//! through exactly what the phone showed, and so the app can later share this code.
//!
//! A 1-D Kalman filter along the direction of travel, with state (speed, accelerometer bias along that direction):
//! - Prediction: acceleration along the travel axis from the IMU. The axis is the dominant horizontal acceleration
//!   direction in the phone's frame; gravity and the axis are carried through phone rotations by the gyro,
//!   and gravity is re-learned from the accelerometer mostly while stopped, so sustained acceleration isn't taken for tilt.
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
const GRAVITY_TAU_MOVING_S: f64 = 600.0;
const AXIS_TAU_S: f64 = 30.0;
const AXIS_MIN_VARIANCE: f64 = 0.002;
const DEPARTURE_EVIDENCE_MPS: f64 = 1.0;
const SIGN_EVIDENCE: f64 = 2.0;
/// Process noise: unmodeled along-axis acceleration (m/s^2 per sqrt(s)), and bias random walk.
const ACCEL_NOISE: f64 = 0.3;
const BIAS_WALK: f64 = 0.005;
const BIAS_SIGMA0: f64 = 0.1;
const STOP_SIGMA_MPS: f64 = 0.05;
const DEFAULT_DOPPLER_SIGMA_MPS: f64 = 0.5;
const MIN_DOPPLER_SIGMA_MPS: f64 = 0.1;
const POSITION_MAX_DT_S: f64 = 5.0;
const POSITION_MAX_SIGMA_MPS: f64 = 3.0;
/// Mach 1 at sea level: nothing this records goes faster, so a faster measurement is always wrong,
/// even as the first one or during a re-initialization, when gating can't catch it.
pub const MAX_SPEED_MPS: f64 = 343.0;
pub const GATE_SIGMA: f64 = 3.0;
pub const GNSS_REINIT_REJECTS: u32 = 5;
pub const RECENT_S: f64 = 3.0;

/// Not thread-safe; feed it from one thread, in time order.
#[derive(Clone, Debug)]
pub struct LiveSpeedEstimator {
    // IMU front end.
    gravity: [f64; 3],
    gravity_init: bool,
    cov: [f64; 9],
    axis: [f64; 3],
    axis_known: bool,
    axis_sign_known: bool,
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

    // GNSS.
    last_gnss_ns: i64,
    /// How far the covariance has been propagated, by the IMU or [`Self::inflate_to`].
    predicted_ns: i64,
    prev_doppler_speed: Option<f64>,
    along_since_gnss: f64,
    sign_evidence: f64,
    gnss_reject_streak: u32,
    prev_fix: Option<Fix>,

    last_used_ns: BTreeMap<Sensor, i64>,
    rejected: BTreeMap<Sensor, u32>,
}

impl Default for LiveSpeedEstimator {
    fn default() -> Self {
        Self {
            gravity: [0.0; 3],
            gravity_init: false,
            cov: [0.0; 9],
            axis: [1.0, 0.0, 0.0],
            axis_known: false,
            axis_sign_known: false,
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
            last_gnss_ns: 0,
            predicted_ns: 0,
            prev_doppler_speed: None,
            along_since_gnss: 0.0,
            sign_evidence: 0.0,
            gnss_reject_streak: 0,
            prev_fix: None,
            last_used_ns: BTreeMap::new(),
            rejected: BTreeMap::new(),
        }
    }
}

impl LiveSpeedEstimator {
    pub fn new() -> Self {
        Self::default()
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
        rotate_world_fixed(&self.omega, &mut self.axis, dt);
        self.rotate_cov(dt);
        let tau = if still {
            GRAVITY_TAU_STILL_S
        } else {
            GRAVITY_TAU_MOVING_S
        };
        let k_g = 1.0 - (-dt / tau).exp();
        for (g, a) in self.gravity.iter_mut().zip([ax, ay, az]) {
            *g += k_g * (a - *g);
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
        if !still {
            self.update_axis(&h, dt);
        }
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
    }

    /// A GNSS fix: Doppler speed if the fix has one, and position for position-differenced speed.
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
        }
    }

    fn on_doppler_speed(&mut self, t_ns: i64, speed: f64, sigma: f64) {
        // Speed change between fixes vs integrated along-axis acceleration fixes the axis sign.
        if let Some(prev) = self.prev_doppler_speed
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
        self.prev_doppler_speed = Some(speed);
        self.along_since_gnss = 0.0;
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
        } else {
            self.gnss_reject_streak += 1;
            if self.gnss_reject_streak >= GNSS_REINIT_REJECTS {
                // Several GNSS fixes in a row disagree with the IMU-driven prediction: trust GNSS.
                self.v = signed;
                self.b = 0.0;
                self.p00 = sigma * sigma;
                self.p01 = 0.0;
                self.p11 = BIAS_SIGMA0 * BIAS_SIGMA0;
                self.gnss_reject_streak = 0;
                self.mark_used(sensor, t_ns);
            }
        }
    }

    fn predict(&mut self, along: f64, dt: f64) {
        if self.axis_sign_known || self.stopped {
            self.v += (along - self.b) * dt;
        } else {
            self.v += along * dt;
        }
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
    fn rotate_cov(&mut self, dt: f64) {
        let o = &self.omega;
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

    /// Exponentially weighted covariance of horizontal acceleration, and its principal axis by power iteration.
    fn update_axis(&mut self, h: &[f64; 3], dt: f64) {
        let k = 1.0 - (-dt / AXIS_TAU_S).exp();
        for i in 0..3 {
            for j in 0..3 {
                self.cov[i * 3 + j] += k * (h[i] * h[j] - self.cov[i * 3 + j]);
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

//! Horizontal position and velocity, from our own GNSS solutions, the phone's fixes, and the IMU between them,
//! smoothed forward and backward in time so the fixes after a tunnel place the path through it.
//!
//! A Kalman filter on east and north position and velocity in a plane tangent at the first position:
//! - Our own GNSS position and velocity update it with their covariances (an exactly determined position's is wide).
//!   The phone's fix updates position only when we have no position of our own from about then,
//!   since both come from the same satellites.
//! - Between them, the gyro's rate about up turns the velocity, and the speed estimator's speed sets its size,
//!   so position follows the track through tunnels with honestly growing uncertainty.
//!   The speed estimator's error is a state of its own: underground it's mostly a slowly changing bias,
//!   so treating each speed as independent would let position drift far beyond its stated uncertainty.
//! - Every update is checked against the prediction first, and an outlier is skipped and counted.
//! - Smoothing runs the same filter backward over the recorded inputs (velocities and turns negated) and combines
//!   the two at each sample by their covariances. Its inputs are light (a few per second), so it covers the whole
//!   ride, live and offline alike.
//!
//! The plane is fine for a city: over 30 km, the earth's curvature moves it by centimeters horizontally.

use ride_gnss::solve::geodetic_to_ecef;

/// Horizontal acceleration the motion model allows (m/s^2, one sigma, per axis).
const ACCEL_SIGMA: f64 = 0.7;
/// Gyro bias left after the phone's calibration (rad/s): how fast the direction of travel's uncertainty grows.
const TURN_SIGMA_RAD_PER_S: f64 = 0.005;
/// Below this speed (m/s), the speed estimator's value says the vehicle is about stopped, rather than giving a size
/// for a direction the filter may not know.
const SLOW_MPS: f64 = 0.5;
/// Innovations beyond this (chi-squared, 2 degrees of freedom: 1 in 3000 by chance) are outliers.
const GATE_CHI2: f64 = 16.0;
/// This many position updates rejected in a row means the filter, not the measurements, went wrong
/// (e.g. dead reckoning off a phone in a pocket): it restarts at the next one.
const RESET_AFTER_REJECTED: u32 = 5;
/// Android's horizontal accuracy is a 68% radius; a per-axis sigma is about this fraction of it.
const ACCURACY_TO_SIGMA: f64 = 1.0 / 1.515;
/// How long the speed estimator's error stays correlated (s): about the time between stops,
/// where its zero-velocity updates reset it.
const SPEED_ERROR_TAU_S: f64 = 90.0;
/// The part of each speed's error that is independent of the next (m/s).
const SPEED_WHITE_SIGMA: f64 = 0.1;
/// Our own GNSS velocity this recent makes the speed estimator's speed redundant (it uses the same Doppler).
const OWN_VELOCITY_HOLD_NS: i64 = 3_000_000_000;
/// Our own GNSS position this recent makes the phone's fix redundant.
const OWN_POSITION_HOLD_NS: i64 = 3_000_000_000;

/// One input to the position filter, in ECEF or geodetic coordinates as it arrives.
#[derive(Clone, Debug, PartialEq)]
pub enum PositionInput {
    /// Our own GNSS position (ECEF, m) and its covariance (row-major).
    OwnPosition {
        ecef: [f64; 3],
        covariance: [f64; 9],
    },
    /// Our own GNSS velocity (ECEF, m/s) and its covariance.
    OwnVelocity {
        ecef: [f64; 3],
        covariance: [f64; 9],
    },
    /// The phone's fix (degrees) and its 68% horizontal accuracy (m).
    Fix { lat: f64, lon: f64, accuracy_m: f64 },
    /// The direction of travel turned this much (rad, counterclockwise seen from above) since the last turn.
    Turn(f64),
    /// The speed estimator's speed and sigma (m/s).
    Speed(f64, f64),
    /// A point to report the position at.
    Sample,
}

/// A position estimate at a sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PositionEstimate {
    pub elapsed_realtime_ns: i64,
    pub lat: f64,
    pub lon: f64,
    /// One sigma, along the worse horizontal axis (m).
    pub sigma_m: f64,
    /// East and north velocity (m/s).
    pub velocity: [f64; 2],
}

/// The tangent plane at the first position.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane {
    lat0: f64,
    lon0: f64,
    ecef0: [f64; 3],
    /// Rows: east and north unit vectors in ECEF.
    east: [f64; 3],
    north: [f64; 3],
    /// Meters per radian of latitude and of longitude there.
    m_per_rad_lat: f64,
    m_per_rad_lon: f64,
}

impl Plane {
    fn new(lat: f64, lon: f64) -> Self {
        const A: f64 = 6_378_137.0;
        const E2: f64 = 6.694_379_990_14e-3;
        let (sl, cl) = lat.to_radians().sin_cos();
        let (so, co) = lon.to_radians().sin_cos();
        let w = 1.0 - E2 * sl * sl;
        Self {
            lat0: lat,
            lon0: lon,
            ecef0: geodetic_to_ecef(lat, lon, 0.0),
            east: [-so, co, 0.0],
            north: [-sl * co, -sl * so, cl],
            m_per_rad_lat: A * (1.0 - E2) / w.powf(1.5),
            m_per_rad_lon: A / w.sqrt() * cl,
        }
    }

    fn from_ecef(ecef: [f64; 3]) -> Self {
        let p = ecef[0].hypot(ecef[1]);
        // Latitude by a few fixed-point steps (converges to well under a meter in three).
        const E2: f64 = 6.694_379_990_14e-3;
        let mut lat = ecef[2].atan2(p * (1.0 - E2));
        for _ in 0..3 {
            let n = 6_378_137.0 / (1.0 - E2 * lat.sin().powi(2)).sqrt();
            let h = p / lat.cos() - n;
            lat = ecef[2].atan2(p * (1.0 - E2 * n / (n + h)));
        }
        Self::new(lat.to_degrees(), ecef[1].atan2(ecef[0]).to_degrees())
    }

    fn en(&self, v: [f64; 3]) -> [f64; 2] {
        [dot3(&self.east, &v), dot3(&self.north, &v)]
    }

    fn en_of_ecef(&self, ecef: [f64; 3]) -> [f64; 2] {
        self.en(std::array::from_fn(|k| ecef[k] - self.ecef0[k]))
    }

    fn en_of_geodetic(&self, lat: f64, lon: f64) -> [f64; 2] {
        [
            (lon - self.lon0).to_radians() * self.m_per_rad_lon,
            (lat - self.lat0).to_radians() * self.m_per_rad_lat,
        ]
    }

    fn geodetic(&self, en: [f64; 2]) -> (f64, f64) {
        (
            self.lat0 + (en[1] / self.m_per_rad_lat).to_degrees(),
            self.lon0 + (en[0] / self.m_per_rad_lon).to_degrees(),
        )
    }

    /// A 3x3 ECEF covariance's east-north block.
    fn en_covariance(&self, c: &[f64; 9]) -> [[f64; 2]; 2] {
        let rows = [self.east, self.north];
        std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                (0..3)
                    .map(|a| {
                        (0..3)
                            .map(|b| rows[i][a] * c[3 * a + b] * rows[j][b])
                            .sum::<f64>()
                    })
                    .sum()
            })
        })
    }
}

/// East, north, east velocity, north velocity, and the speed estimator's error.
const N: usize = 5;
type M = [[f64; N]; N];
type V = [f64; N];
/// A forward pass's estimate at a sample, with its east-north position and covariance for combining.
type Forward = (PositionEstimate, [f64; 2], [[f64; 2]; 2]);

/// The filter's state: see [`N`].
#[derive(Clone, Debug)]
pub struct PositionFilter {
    plane: Option<Plane>,
    x: V,
    p: M,
    /// The speed estimator's latest sigma (m/s): what its error decays toward.
    speed_sigma: f64,
    /// The direction of travel (east, north unit vector) when last moving, turned by the gyro since:
    /// where a speed puts the velocity when the vehicle pulls away from a stop with no fixes.
    heading: Option<[f64; 2]>,
    last_ns: Option<i64>,
    last_own_position_ns: Option<i64>,
    last_own_velocity_ns: Option<i64>,
    /// Whether this pass runs backward in time (velocities and turns negated).
    reversed: bool,
    pub rejected: u32,
    /// Position updates rejected since the last one accepted, and how many times that forced a restart.
    rejected_in_row: u32,
    pub resets: u32,
}

impl PositionFilter {
    pub fn new() -> Self {
        Self::with_plane(None, false)
    }

    fn with_plane(plane: Option<Plane>, reversed: bool) -> Self {
        Self {
            plane,
            x: [0.0; N],
            p: [[0.0; N]; N],
            speed_sigma: 0.0,
            heading: None,
            last_ns: None,
            last_own_position_ns: None,
            last_own_velocity_ns: None,
            reversed,
            rejected: 0,
            rejected_in_row: 0,
            resets: 0,
        }
    }

    fn initialized(&self) -> bool {
        self.last_ns.is_some()
    }

    fn predict(&mut self, t_ns: i64) {
        let Some(last) = self.last_ns else { return };
        let dt = ((t_ns - last) as f64 * 1e-9).abs();
        self.last_ns = Some(t_ns);
        if dt == 0.0 {
            return;
        }
        // The speed error decays toward its stationary spread, the speed estimator's own sigma.
        let phi = (-dt / SPEED_ERROR_TAU_S).exp();
        self.x = [
            self.x[0] + dt * self.x[2],
            self.x[1] + dt * self.x[3],
            self.x[2],
            self.x[3],
            phi * self.x[4],
        ];
        let mut fm: M = identity();
        fm[0][2] = dt;
        fm[1][3] = dt;
        fm[4][4] = phi;
        self.p = mul(&mul(&fm, &self.p), &transpose(&fm));
        // Acceleration along the direction of travel once moving: a vehicle doesn't slide sideways,
        // so its sideways uncertainty comes only from the direction's, below. Either way when slow.
        let v = self.x[2].hypot(self.x[3]);
        let q = ACCEL_SIGMA * ACCEL_SIGMA;
        let axes: [[f64; 2]; 2] = if v > SLOW_MPS {
            [[self.x[2] / v, self.x[3] / v], [0.0, 0.0]]
        } else {
            [[1.0, 0.0], [0.0, 1.0]]
        };
        for i in 0..2 {
            for j in 0..2 {
                let a: f64 = axes.iter().map(|u| u[i] * u[j]).sum();
                self.p[i][j] += q * a * dt.powi(3) / 3.0;
                self.p[i][j + 2] += q * a * dt.powi(2) / 2.0;
                self.p[i + 2][j] += q * a * dt.powi(2) / 2.0;
                self.p[i + 2][j + 2] += q * a * dt;
            }
        }
        self.p[4][4] += self.speed_sigma.powi(2) * (1.0 - phi * phi);
        // The direction of travel's uncertainty from the gyro's leftover bias.
        let v2 = v * v;
        if v2 > 0.0 {
            let s = TURN_SIGMA_RAD_PER_S * dt;
            let perp = [-self.x[3], self.x[2]];
            for i in 0..2 {
                for j in 0..2 {
                    self.p[i + 2][j + 2] += s * s * perp[i] * perp[j];
                }
            }
        }
    }

    /// A linear measurement `z = H x` with covariance `r`, if it passes the gate.
    #[allow(clippy::needless_range_loop)] // `i` and `j` index `x`, `k`, `h`, and `p` together.
    fn update(&mut self, h: &[V; 2], z: [f64; 2], r: [[f64; 2]; 2]) -> bool {
        let y = [
            z[0] - row_dot(&h[0], &self.x),
            z[1] - row_dot(&h[1], &self.x),
        ];
        let ph: [[f64; 2]; N] =
            std::array::from_fn(|i| std::array::from_fn(|k| row_dot(&self.p[i], &h[k])));
        let s: [[f64; 2]; 2] = std::array::from_fn(|k| {
            std::array::from_fn(|l| row_dot(&h[k], &column(&ph, l)) + r[k][l])
        });
        let Some(si) = inv2(&s) else { return false };
        let chi2 =
            y[0] * (si[0][0] * y[0] + si[0][1] * y[1]) + y[1] * (si[1][0] * y[0] + si[1][1] * y[1]);
        if chi2 > GATE_CHI2 {
            self.rejected += 1;
            return false;
        }
        let k: [[f64; 2]; N] = std::array::from_fn(|i| {
            std::array::from_fn(|l| ph[i][0] * si[0][l] + ph[i][1] * si[1][l])
        });
        for i in 0..N {
            self.x[i] += k[i][0] * y[0] + k[i][1] * y[1];
        }
        // Joseph form keeps P symmetric and positive.
        let mut ikh: M = identity();
        for i in 0..N {
            for j in 0..N {
                ikh[i][j] -= k[i][0] * h[0][j] + k[i][1] * h[1][j];
            }
        }
        let mut p = mul(&mul(&ikh, &self.p), &transpose(&ikh));
        for i in 0..N {
            for j in 0..N {
                p[i][j] += (0..2)
                    .map(|a| (0..2).map(|b| k[i][a] * r[a][b] * k[j][b]).sum::<f64>())
                    .sum::<f64>();
            }
        }
        self.p = p;
        true
    }

    fn update_position(&mut self, t_ns: i64, en: [f64; 2], r: [[f64; 2]; 2]) {
        let restart = self.rejected_in_row >= RESET_AFTER_REJECTED;
        if restart {
            self.resets += 1;
        }
        if !self.initialized() || restart {
            self.rejected_in_row = 0;
            self.x = [en[0], en[1], 0.0, 0.0, 0.0];
            self.p = [[0.0; N]; N];
            self.p[0][0] = r[0][0];
            self.p[0][1] = r[0][1];
            self.p[1][0] = r[1][0];
            self.p[1][1] = r[1][1];
            // Velocity unknown: up to about 100 km/h either way.
            self.p[2][2] = 30.0 * 30.0;
            self.p[3][3] = 30.0 * 30.0;
            self.p[4][4] = self.speed_sigma.powi(2);
            self.last_ns = Some(t_ns);
            return;
        }
        if self.update(
            &[[1.0, 0.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0, 0.0]],
            en,
            r,
        ) {
            self.rejected_in_row = 0;
        } else {
            self.rejected_in_row += 1;
        }
    }

    fn update_velocity(&mut self, v: [f64; 2], r: [[f64; 2]; 2]) {
        self.update(
            &[[0.0, 0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0, 0.0]],
            v,
            r,
        );
    }

    /// Feeds one input at `t_ns`; returns the estimate if it's a sample and the filter has a position.
    pub fn on_input(&mut self, t_ns: i64, input: &PositionInput) -> Option<PositionEstimate> {
        let sign = if self.reversed { -1.0 } else { 1.0 };
        if self.plane.is_none() {
            self.plane = match input {
                PositionInput::OwnPosition { ecef, .. } => Some(Plane::from_ecef(*ecef)),
                PositionInput::Fix { lat, lon, .. } => Some(Plane::new(*lat, *lon)),
                _ => None,
            };
        }
        let plane = self.plane?;
        self.predict(t_ns);
        match *input {
            PositionInput::OwnPosition { ecef, covariance } => {
                self.update_position(
                    t_ns,
                    plane.en_of_ecef(ecef),
                    plane.en_covariance(&covariance),
                );
                self.last_own_position_ns = Some(t_ns);
            }
            PositionInput::Fix {
                lat,
                lon,
                accuracy_m,
            } => {
                let own_recent = self
                    .last_own_position_ns
                    .is_some_and(|t| (t_ns - t).abs() <= OWN_POSITION_HOLD_NS);
                if !own_recent {
                    let s2 = (accuracy_m * ACCURACY_TO_SIGMA).powi(2);
                    self.update_position(
                        t_ns,
                        plane.en_of_geodetic(lat, lon),
                        [[s2, 0.0], [0.0, s2]],
                    );
                }
            }
            _ if !self.initialized() => {}
            PositionInput::OwnVelocity { ecef, covariance } => {
                let v = plane.en(ecef);
                self.update_velocity([sign * v[0], sign * v[1]], plane.en_covariance(&covariance));
                self.last_own_velocity_ns = Some(t_ns);
            }
            PositionInput::Turn(theta) => {
                let (s, c) = (sign * theta).sin_cos();
                let mut t: M = identity();
                t[2][2] = c;
                t[2][3] = -s;
                t[3][2] = s;
                t[3][3] = c;
                self.x = [
                    self.x[0],
                    self.x[1],
                    c * self.x[2] - s * self.x[3],
                    s * self.x[2] + c * self.x[3],
                    self.x[4],
                ];
                self.p = mul(&mul(&t, &self.p), &transpose(&t));
                self.heading = self
                    .heading
                    .map(|h| [c * h[0] - s * h[1], s * h[0] + c * h[1]]);
            }
            PositionInput::Speed(speed, sigma) => {
                // The error is in the speed's size, so it's the same either way in time.
                self.speed_sigma = sigma;
                let own_recent = self
                    .last_own_velocity_ns
                    .is_some_and(|t| (t_ns - t).abs() <= OWN_VELOCITY_HOLD_NS);
                let v = self.x[2].hypot(self.x[3]);
                if v > SLOW_MPS {
                    self.heading = Some([self.x[2] / v, self.x[3] / v]);
                }
                if own_recent {
                } else if speed < SLOW_MPS {
                    // Stopped: and the speed estimator's own zero-velocity updates leave little error in it.
                    let s2 = sigma * sigma + SLOW_MPS * SLOW_MPS;
                    self.update_velocity([0.0, 0.0], [[s2, 0.0], [0.0, s2]]);
                    let w = SPEED_WHITE_SIGMA * SPEED_WHITE_SIGMA;
                    let h = [[0.0, 0.0, 0.0, 0.0, 1.0], [0.0; N]];
                    self.update(&h, [0.0, 0.0], [[sigma * sigma + w, 0.0], [0.0, 1.0]]);
                } else if let Some(u) = self.heading {
                    // The size of the velocity, in its own direction (or the last one, setting off from a stop),
                    // plus the speed's error (a scalar update, as one row with a loose other).
                    let h = [[0.0, 0.0, u[0], u[1], 1.0], [0.0, 0.0, -u[1], u[0], 0.0]];
                    let loose = 1e6;
                    let w = SPEED_WHITE_SIGMA * SPEED_WHITE_SIGMA;
                    self.update(&h, [speed, 0.0], [[w, 0.0], [0.0, loose]]);
                }
            }
            PositionInput::Sample => return self.estimate(t_ns),
        }
        None
    }

    fn estimate(&self, t_ns: i64) -> Option<PositionEstimate> {
        let plane = self.plane?;
        self.initialized().then(|| {
            let (lat, lon) = plane.geodetic([self.x[0], self.x[1]]);
            let sign = if self.reversed { -1.0 } else { 1.0 };
            PositionEstimate {
                elapsed_realtime_ns: t_ns,
                lat,
                lon,
                sigma_m: worst_sigma(&[[self.p[0][0], self.p[0][1]], [self.p[1][0], self.p[1][1]]]),
                velocity: [sign * self.x[2], sign * self.x[3]],
            }
        })
    }

    fn position_block(&self) -> ([f64; 2], [[f64; 2]; 2]) {
        (
            [self.x[0], self.x[1]],
            [[self.p[0][0], self.p[0][1]], [self.p[1][0], self.p[1][1]]],
        )
    }
}

impl Default for PositionFilter {
    fn default() -> Self {
        Self::new()
    }
}

/// Positions at every sample in `inputs` (in time order), forward and backward passes combined by their covariances.
pub fn smooth(inputs: &[(i64, PositionInput)]) -> Vec<PositionEstimate> {
    let mut forward = PositionFilter::new();
    let mut fwd: Vec<Option<Forward>> = Vec::new();
    for (t, input) in inputs {
        if forward.on_input(*t, input).is_some() || matches!(input, PositionInput::Sample) {
            fwd.push(forward.estimate(*t).map(|e| {
                let (x, p) = forward.position_block();
                (e, x, p)
            }));
        }
    }
    // The backward pass shares the forward pass's plane, so the two combine in the same coordinates.
    let mut backward = PositionFilter::with_plane(forward.plane, true);
    let mut out = Vec::with_capacity(fwd.len());
    let mut i = fwd.len();
    for (t, input) in inputs.iter().rev() {
        backward.on_input(*t, input);
        if !matches!(input, PositionInput::Sample) {
            continue;
        }
        i -= 1;
        let Some((f, xf, pf)) = &fwd[i] else { continue };
        let combined = if backward.initialized() {
            let (xb, pb) = backward.position_block();
            match (inv2(pf), inv2(&pb)) {
                (Some(wf), Some(wb)) => {
                    let w = add2(&wf, &wb);
                    inv2(&w).map(|p| {
                        let m = [
                            wf[0][0] * xf[0]
                                + wf[0][1] * xf[1]
                                + wb[0][0] * xb[0]
                                + wb[0][1] * xb[1],
                            wf[1][0] * xf[0]
                                + wf[1][1] * xf[1]
                                + wb[1][0] * xb[0]
                                + wb[1][1] * xb[1],
                        ];
                        (
                            [
                                p[0][0] * m[0] + p[0][1] * m[1],
                                p[1][0] * m[0] + p[1][1] * m[1],
                            ],
                            p,
                        )
                    })
                }
                _ => None,
            }
        } else {
            None
        };
        out.push(match (combined, forward.plane) {
            (Some((x, p)), Some(plane)) => {
                let (lat, lon) = plane.geodetic(x);
                PositionEstimate {
                    lat,
                    lon,
                    sigma_m: worst_sigma(&p),
                    ..*f
                }
            }
            _ => *f,
        });
    }
    out.reverse();
    out
}

fn worst_sigma(p: &[[f64; 2]; 2]) -> f64 {
    // The larger eigenvalue of a symmetric 2x2.
    let (a, b, d) = (p[0][0], p[0][1], p[1][1]);
    let m = (a + d) / 2.0;
    (m + (((a - d) / 2.0).powi(2) + b * b).sqrt())
        .max(0.0)
        .sqrt()
}

fn dot3(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn identity() -> M {
    std::array::from_fn(|i| std::array::from_fn(|j| f64::from(u8::from(i == j))))
}

fn mul(a: &M, b: &M) -> M {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..N).map(|k| a[i][k] * b[k][j]).sum()))
}

fn transpose(a: &M) -> M {
    std::array::from_fn(|i| std::array::from_fn(|j| a[j][i]))
}

fn row_dot(a: &V, b: &V) -> f64 {
    (0..N).map(|k| a[k] * b[k]).sum()
}

fn column(m: &[[f64; 2]; N], l: usize) -> V {
    std::array::from_fn(|i| m[i][l])
}

fn inv2(m: &[[f64; 2]; 2]) -> Option<[[f64; 2]; 2]> {
    let det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    (det.abs() > 1e-12).then(|| {
        [
            [m[1][1] / det, -m[0][1] / det],
            [-m[1][0] / det, m[0][0] / det],
        ]
    })
}

fn add2(a: &[[f64; 2]; 2], b: &[[f64; 2]; 2]) -> [[f64; 2]; 2] {
    std::array::from_fn(|i| std::array::from_fn(|j| a[i][j] + b[i][j]))
}

#[cfg(test)]
mod tests {
    use super::*;

    const LAT0: f64 = 40.75;
    const LON0: f64 = -73.99;
    const S: i64 = 1_000_000_000;

    /// A path at 10 m/s, east for `turn_at_s` seconds, then north: (t, east, north, velocity).
    fn path(t_s: f64, turn_at_s: f64) -> ([f64; 2], [f64; 2]) {
        if t_s <= turn_at_s {
            ([10.0 * t_s, 0.0], [10.0, 0.0])
        } else {
            ([10.0 * turn_at_s, 10.0 * (t_s - turn_at_s)], [0.0, 10.0])
        }
    }

    fn fix(plane: &Plane, en: [f64; 2]) -> PositionInput {
        let (lat, lon) = plane.geodetic(en);
        PositionInput::Fix {
            lat,
            lon,
            accuracy_m: 5.0,
        }
    }

    /// Fixes each second except in `gap` (seconds), speed and a sample each second, and the turn as a `Turn`.
    fn ride(
        seconds: i64,
        turn_at_s: f64,
        gap: std::ops::Range<i64>,
    ) -> (Plane, Vec<(i64, PositionInput)>) {
        let plane = Plane::new(LAT0, LON0);
        let mut inputs = Vec::new();
        for t in 0..=seconds {
            let (en, _) = path(t as f64, turn_at_s);
            if !gap.contains(&t) {
                inputs.push((t * S, fix(&plane, en)));
            }
            if t as f64 == turn_at_s {
                inputs.push((t * S, PositionInput::Turn(std::f64::consts::FRAC_PI_2)));
            }
            inputs.push((t * S, PositionInput::Speed(10.0, 0.5)));
            inputs.push((t * S, PositionInput::Sample));
        }
        (plane, inputs)
    }

    fn error_m(plane: &Plane, e: &PositionEstimate, truth: [f64; 2]) -> f64 {
        let en = plane.en_of_geodetic(e.lat, e.lon);
        (en[0] - truth[0]).hypot(en[1] - truth[1])
    }

    fn forward(inputs: &[(i64, PositionInput)]) -> Vec<PositionEstimate> {
        let mut f = PositionFilter::new();
        inputs
            .iter()
            .filter_map(|(t, i)| f.on_input(*t, i))
            .collect()
    }

    #[test]
    fn plane_round_trips() {
        let plane = Plane::new(LAT0, LON0);
        let (lat, lon) = plane.geodetic([1234.0, -567.0]);
        let en = plane.en_of_geodetic(lat, lon);
        assert!((en[0] - 1234.0).abs() < 1e-6 && (en[1] + 567.0).abs() < 1e-6);
        // ECEF and geodetic agree on the same point, to well under a meter.
        let en = plane.en_of_ecef(geodetic_to_ecef(lat, lon, 0.0));
        assert!(
            (en[0] - 1234.0).abs() < 0.5 && (en[1] + 567.0).abs() < 0.5,
            "{en:?}"
        );
    }

    #[test]
    fn follows_a_straight_line() {
        let (plane, inputs) = ride(60, 1e9, 0..0);
        let estimates = smooth(&inputs);
        assert_eq!(estimates.len(), 61);
        for (t, e) in estimates.iter().enumerate().skip(5) {
            let (truth, v) = path(t as f64, 1e9);
            assert!(error_m(&plane, e, truth) < 3.0, "at {t} s: {e:?}");
            assert!(
                (e.velocity[0] - v[0]).abs() < 1.0 && e.velocity[1].abs() < 1.0,
                "at {t} s: {e:?}"
            );
        }
    }

    /// Forward-only estimates and smoothed ones, through a gap in the fixes with the speed 5% high,
    /// as an accelerometer bias would leave it (within the speed's stated sigma).
    fn biased_gap(turn_at_s: f64) -> (Plane, Vec<PositionEstimate>, Vec<PositionEstimate>) {
        let (plane, mut inputs) = ride(90, turn_at_s, 30..70);
        for (_, input) in &mut inputs {
            if let PositionInput::Speed(v, _) = input {
                *v *= 1.05;
            }
        }
        let fwd = forward(&inputs);
        let smoothed = smooth(&inputs);
        (plane, fwd, smoothed)
    }

    /// The mean error over the gap, after checking every error there is within 3 sigma.
    fn gap_error(plane: &Plane, es: &[PositionEstimate], turn_at_s: f64) -> f64 {
        let errors: Vec<f64> = (30..70)
            .map(|t| {
                let e = error_m(plane, &es[t], path(t as f64, turn_at_s).0);
                assert!(
                    e <= 3.0 * es[t].sigma_m,
                    "at {t} s: {e:.1} m off, sigma {:.1} m",
                    es[t].sigma_m
                );
                e
            })
            .collect();
        errors.iter().sum::<f64>() / errors.len() as f64
    }

    #[test]
    fn smoothing_through_a_straight_gap_cancels_the_drift() {
        let (plane, fwd, smoothed) = biased_gap(1e9);
        let (f, s) = (
            gap_error(&plane, &fwd, 1e9),
            gap_error(&plane, &smoothed, 1e9),
        );
        assert!(s < f / 2.0, "smoothed {s:.1} m, forward {f:.1} m");
    }

    #[test]
    fn turns_with_the_gyro_through_a_gap() {
        let (plane, fwd, smoothed) = biased_gap(40.0);
        let (f, s) = (
            gap_error(&plane, &fwd, 40.0),
            gap_error(&plane, &smoothed, 40.0),
        );
        assert!(s < f, "smoothed {s:.1} m, forward {f:.1} m");
        // Uncertainty grows into the gap going forward, and smoothing pulls it back in the middle.
        assert!(fwd[69].sigma_m > fwd[31].sigma_m);
        assert!(smoothed[50].sigma_m < fwd[69].sigma_m);
    }

    #[test]
    fn skips_an_outlying_fix() {
        let (plane, mut inputs) = ride(60, 1e9, 0..0);
        // A fix 300 m off to the north at 30 s.
        let at = inputs.iter().position(|(t, _)| *t == 30 * S).unwrap();
        inputs.insert(at, (30 * S, fix(&plane, [300.0, 300.0])));
        let mut f = PositionFilter::new();
        let estimates: Vec<_> = inputs
            .iter()
            .filter_map(|(t, i)| f.on_input(*t, i))
            .collect();
        assert_eq!(f.rejected, 1);
        assert!(
            error_m(&plane, &estimates[30], [300.0, 0.0]) < 3.0,
            "{:?}",
            estimates[30]
        );
    }

    #[test]
    fn restarts_when_it_rejects_fix_after_fix() {
        // Dead reckoning carried it 290 m east, but the fixes resume 210 m west: 500 m from where it thinks.
        let plane = Plane::new(LAT0, LON0);
        let mut f = PositionFilter::new();
        f.on_input(0, &fix(&plane, [0.0, 0.0]));
        f.on_input(S, &fix(&plane, [10.0, 0.0]));
        for t in 2..60 {
            f.on_input(
                t * S,
                &PositionInput::Speed(if t < 30 { 10.0 } else { 0.0 }, 0.1),
            );
        }
        let mut last = None;
        for t in 60..80 {
            f.on_input(t * S, &fix(&plane, [-210.0, 0.0]));
            last = f.on_input(t * S, &PositionInput::Sample);
        }
        assert_eq!(f.resets, 1);
        assert!(
            error_m(&plane, &last.unwrap(), [-210.0, 0.0]) < 5.0,
            "{last:?}"
        );
    }

    #[test]
    fn stops_when_the_speed_says_so() {
        let plane = Plane::new(LAT0, LON0);
        let mut f = PositionFilter::new();
        f.on_input(0, &fix(&plane, [0.0, 0.0]));
        let mut last = None;
        for t in 1..=60 {
            f.on_input(t * S, &PositionInput::Speed(0.0, 0.1));
            last = f.on_input(t * S, &PositionInput::Sample);
        }
        let e = last.unwrap();
        assert!(error_m(&plane, &e, [0.0, 0.0]) < 10.0, "{e:?}");
        assert!(e.velocity[0].hypot(e.velocity[1]) < 0.5, "{e:?}");
    }

    #[test]
    fn sets_off_from_a_stop_the_way_it_last_went() {
        // East with fixes, then with none: a stop, a left turn, and away at 1 m/s^2. It should go north.
        let (_, mut inputs) = ride(20, 1e9, 20..21);
        for t in 21..51 {
            if (31..41).contains(&t) {
                inputs.push((
                    t * S,
                    PositionInput::Turn(std::f64::consts::FRAC_PI_2 / 10.0),
                ));
            }
            let speed = (t - 40).max(0) as f64;
            inputs.push((t * S, PositionInput::Speed(speed, 0.1)));
            inputs.push((t * S, PositionInput::Sample));
        }
        let e = forward(&inputs).pop().unwrap();
        assert!(e.velocity[1] > 9.0 && e.velocity[0].abs() < 1.0, "{e:?}");
    }
}

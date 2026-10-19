//! Position and velocity from one epoch of Android's raw GNSS measurements (GPS so far).
//!
//! Velocity comes from each satellite's Doppler (`pseudorange_rate`), which measures speed directly and to a few
//! cm/s per satellite, by weighted least squares with outlier rejection. Position, from pseudoranges, is only needed
//! for the directions to the satellites, so meters of error don't matter for velocity.

use std::collections::BTreeMap;

use ride_session::proto::{
    GnssClock, GnssMeasurement, GnssMeasurementsEvent, GnssNavigationMessage,
};

use crate::ephemeris::{C, GpsEphemeris, OMEGA_E, SECONDS_PER_WEEK, SatState};
use crate::lnav::{EphemerisAssembler, TYPE_GPS_L1CA, decode_subframe};

/// `GnssStatus.CONSTELLATION_GPS`.
pub const CONSTELLATION_GPS: i32 = 1;
/// `GnssMeasurement.STATE_CODE_LOCK`, `STATE_TOW_DECODED`, `STATE_TOW_KNOWN`.
const STATE_CODE_LOCK: i32 = 0x1;
const STATE_TOW_DECODED: i32 = 0x8;
const STATE_TOW_KNOWN: i32 = 0x4000;
/// `GnssMeasurement.STATE_MSEC_AMBIGUOUS`: the transmit time may be off by whole milliseconds (300 km each).
const STATE_MSEC_AMBIGUOUS: i32 = 0x10;
/// Pseudorange residuals beyond this (m) reject the worst satellite from the position solution: uncorrected
/// ionosphere and troposphere are tens of meters at most, so more is multipath or a bad measurement.
const POSITION_OUTLIER_M: f64 = 60.0;
const NANOS_PER_WEEK: i64 = 604_800_000_000_000;
/// Doppler measurements are trusted no more than this (m/s), whatever Android says.
const MIN_PRR_SIGMA: f64 = 0.05;
/// Residuals beyond this many sigma reject the worst satellite and solve again.
const OUTLIER_SIGMA: f64 = 4.0;
/// Weaker signals than this are too noisy and multipath-prone to use.
const MIN_CN0_DBHZ: f64 = 20.0;

/// One epoch's solution.
#[derive(Clone, Debug, PartialEq)]
pub struct Solution {
    /// The epoch on the elapsed realtime clock, if Android gave it.
    pub elapsed_realtime_ns: Option<i64>,
    /// ECEF (m).
    pub position: [f64; 3],
    /// ECEF (m/s).
    pub velocity: [f64; 3],
    /// Covariance of `velocity` (m^2/s^2), row-major.
    pub velocity_covariance: [f64; 9],
    /// Receiver clock drift (m/s).
    pub clock_drift: f64,
    pub satellites_used: usize,
    pub satellites_rejected: usize,
    /// Root mean square of the Doppler residuals (m/s).
    pub residual_rms: f64,
    /// Root mean square of the pseudorange residuals of the position solution (m).
    pub position_residual_rms: f64,
}

impl Solution {
    pub fn speed(&self) -> f64 {
        norm(&self.velocity)
    }

    /// Standard deviation of the speed, along the direction of travel (or the largest one when nearly stopped).
    pub fn speed_sigma(&self) -> f64 {
        let s = self.speed();
        let c = &self.velocity_covariance;
        if s < 0.1 {
            return (c[0].max(c[4]).max(c[8])).sqrt();
        }
        let u = self.velocity.map(|v| v / s);
        let mut var = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                var += u[i] * c[3 * i + j] * u[j];
            }
        }
        var.max(0.0).sqrt()
    }
}

/// Why an epoch has no solution.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NoSolution {
    NoClock,
    TooFewSatellites { usable: usize },
    Singular,
}

/// Counters for diagnostics (`ride gnss`).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    pub subframes_ok: u64,
    pub subframes_bad: u64,
    pub ephemerides: u64,
    pub epochs: u64,
    pub epochs_solved: u64,
}

/// Keeps ephemerides from navigation messages and solves epochs of measurements.
#[derive(Default)]
pub struct Navigator {
    assembler: EphemerisAssembler,
    ephemerides: BTreeMap<u8, Vec<GpsEphemeris>>,
    /// Full GPS week from the latest clock, to resolve navigation messages' 10-bit weeks.
    week: Option<u32>,
    /// Subframes that arrived before any clock.
    waiting: Vec<(u8, crate::lnav::Subframe)>,
    last_position: Option<[f64; 3]>,
    pub stats: Stats,
}

impl Navigator {
    /// Adds an ephemeris from elsewhere (e.g. a downloaded broadcast ephemeris file).
    pub fn add_ephemeris(&mut self, eph: GpsEphemeris) {
        let list = self.ephemerides.entry(eph.prn).or_default();
        if !list
            .iter()
            .any(|e| e.week == eph.week && e.toe == eph.toe && e.iode == eph.iode)
        {
            list.push(eph);
            self.stats.ephemerides += 1;
        }
    }

    pub fn on_navigation_message(&mut self, msg: &GnssNavigationMessage) {
        if msg.r#type != TYPE_GPS_L1CA {
            return;
        }
        let Ok(prn) = u8::try_from(msg.svid) else {
            return;
        };
        match decode_subframe(&msg.data) {
            Ok(sf) => {
                self.stats.subframes_ok += 1;
                match self.week {
                    Some(week) => {
                        if let Some(eph) = self.assembler.add(prn, sf, week) {
                            self.add_ephemeris(eph);
                        }
                    }
                    None if self.waiting.len() < 1000 => self.waiting.push((prn, sf)),
                    None => {}
                }
            }
            Err(_) => self.stats.subframes_bad += 1,
        }
    }

    /// A position near the receiver (e.g. the phone's last fix), used until a pseudorange solution exists.
    pub fn set_approximate_position(&mut self, ecef: [f64; 3]) {
        if self.last_position.is_none() {
            self.last_position = Some(ecef);
        }
    }

    pub fn on_measurements(
        &mut self,
        event: &GnssMeasurementsEvent,
    ) -> Result<Solution, NoSolution> {
        self.stats.epochs += 1;
        let clock = event.clock.as_ref().ok_or(NoSolution::NoClock)?;
        let full_bias = clock.full_bias_nanos.ok_or(NoSolution::NoClock)?;
        let rx_gps_ns = clock.time_nanos - full_bias;
        let week =
            u32::try_from(rx_gps_ns.div_euclid(NANOS_PER_WEEK)).map_err(|_| NoSolution::NoClock)?;
        if self.week != Some(week) {
            self.week = Some(week);
            for (prn, sf) in std::mem::take(&mut self.waiting) {
                if let Some(eph) = self.assembler.add(prn, sf, week) {
                    self.add_ephemeris(eph);
                }
            }
        }
        let sats = self.satellites(clock, &event.measurements, rx_gps_ns, week);
        if sats.len() < 5 {
            return Err(NoSolution::TooFewSatellites { usable: sats.len() });
        }
        let (position, position_residual_rms) =
            solve_position(&sats, self.last_position).ok_or(NoSolution::Singular)?;
        self.last_position = Some(position);
        let mut sol = solve_velocity(&sats, position).ok_or(NoSolution::Singular)?;
        sol.position_residual_rms = position_residual_rms;
        sol.elapsed_realtime_ns = clock.elapsed_realtime_nanos;
        self.stats.epochs_solved += 1;
        Ok(sol)
    }

    /// Usable GPS measurements with their satellites' states, one per satellite (the strongest signal).
    fn satellites(
        &self,
        clock: &GnssClock,
        measurements: &[GnssMeasurement],
        rx_gps_ns: i64,
        week: u32,
    ) -> Vec<Sat> {
        let bias = clock.bias_nanos.unwrap_or(0.0);
        let mut best: BTreeMap<u8, Sat> = BTreeMap::new();
        for m in measurements {
            let tow_known = m.state & (STATE_TOW_DECODED | STATE_TOW_KNOWN) != 0;
            if m.constellation_type != CONSTELLATION_GPS
                || m.state & STATE_CODE_LOCK == 0
                || !tow_known
                || m.state & STATE_MSEC_AMBIGUOUS != 0
                || m.cn0_dbhz < MIN_CN0_DBHZ
                || !m.pseudorange_rate_meters_per_second.is_finite()
            {
                continue;
            }
            let Ok(prn) = u8::try_from(m.svid) else {
                continue;
            };
            // Receive time of week (ns, with sub-ns parts kept as f64), and transmit time in the satellite's clock.
            let rx_tow_ns =
                rx_gps_ns.rem_euclid(NANOS_PER_WEEK) as f64 + m.time_offset_nanos - bias;
            let tx_tow_ns = m.received_sv_time_nanos as f64;
            let mut travel_ns = rx_tow_ns - tx_tow_ns;
            if travel_ns < -(NANOS_PER_WEEK as f64) / 2.0 {
                travel_ns += NANOS_PER_WEEK as f64;
            }
            // GPS signals take 64 to 90 ms; anything else is a bad time.
            if !(0.06e9..0.1e9).contains(&travel_ns) {
                continue;
            }
            let t_sv = tx_tow_ns * 1e-9;
            let Some(eph) = self.ephemeris(prn, week, t_sv) else {
                continue;
            };
            // Satellite clock time to GPS time, then the satellite's state at transmission.
            let dt = eph.state_at(t_sv).clock_bias;
            let state = eph.state_at(t_sv - dt);
            let sat = Sat {
                state,
                pseudorange: travel_ns * 1e-9 * C + C * (state.clock_bias - eph.tgd),
                prr: m.pseudorange_rate_meters_per_second + C * state.clock_drift,
                prr_sigma: m
                    .pseudorange_rate_uncertainty_meters_per_second
                    .max(MIN_PRR_SIGMA),
                cn0: m.cn0_dbhz,
            };
            if best.get(&prn).is_none_or(|b| b.cn0 < sat.cn0) {
                best.insert(prn, sat);
            }
        }
        best.into_values().collect()
    }

    /// The ephemeris for `prn` valid at `t` (seconds of `week`) with the closest `toe`.
    fn ephemeris(&self, prn: u8, week: u32, t: f64) -> Option<&GpsEphemeris> {
        let at = f64::from(week) * SECONDS_PER_WEEK + t;
        self.ephemerides
            .get(&prn)?
            .iter()
            .filter(|e| e.sv_health == 0)
            .map(|e| {
                (
                    e,
                    (at - (f64::from(e.week) * SECONDS_PER_WEEK + e.toe)).abs(),
                )
            })
            .filter(|&(_, d)| d <= 4.0 * 3600.0)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(e, _)| e)
    }
}

#[derive(Clone, Copy, Debug)]
struct Sat {
    state: SatState,
    /// Corrected for the satellite clock and group delay (m).
    pseudorange: f64,
    /// Corrected for the satellite clock drift (m/s).
    prr: f64,
    prr_sigma: f64,
    cn0: f64,
}

/// The satellite's position and velocity rotated into the ECEF frame at reception (the Sagnac correction).
fn rotated(state: &SatState, receiver: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let range = norm(&sub(&state.position, &receiver));
    let theta = OMEGA_E * range / C;
    let (s, c) = theta.sin_cos();
    let rot = |v: [f64; 3]| [c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2]];
    (rot(state.position), rot(state.velocity))
}

/// Least-squares position (and clock bias) from pseudoranges, by Gauss-Newton from `start` or the earth's center,
/// dropping the worst satellite while its residual is beyond [`POSITION_OUTLIER_M`] and a redundant one remains.
/// Returns the position and the residual RMS (m).
fn solve_position(sats: &[Sat], start: Option<[f64; 3]>) -> Option<([f64; 3], f64)> {
    let mut used: Vec<&Sat> = sats.iter().collect();
    loop {
        let (x, residuals) = gauss_newton_position(&used, start)?;
        let (worst, worst_value) = residuals
            .iter()
            .enumerate()
            .map(|(i, r)| (i, r.abs()))
            .max_by(|a, b| a.1.total_cmp(&b.1))?;
        if worst_value > POSITION_OUTLIER_M && used.len() > 5 {
            used.remove(worst);
            continue;
        }
        let rms = (residuals.iter().map(|r| r * r).sum::<f64>() / residuals.len() as f64).sqrt();
        return Some((x, rms));
    }
}

fn gauss_newton_position(sats: &[&Sat], start: Option<[f64; 3]>) -> Option<([f64; 3], Vec<f64>)> {
    let mut x = [0.0; 4];
    if let Some(p) = start {
        x[..3].copy_from_slice(&p);
    }
    let rows_at = |x: &[f64; 4]| -> Vec<([f64; 4], f64, f64)> {
        sats.iter()
            .map(|s| {
                let rx = [x[0], x[1], x[2]];
                let (sp, _) = rotated(&s.state, rx);
                let d = sub(&sp, &rx);
                let r = norm(&d);
                let u = d.map(|v| v / r);
                ([-u[0], -u[1], -u[2], 1.0], s.pseudorange - (r + x[3]), 1.0)
            })
            .collect()
    };
    for _ in 0..10 {
        let (dx, _) = weighted_least_squares(&rows_at(&x))?;
        for (xi, d) in x.iter_mut().zip(dx) {
            *xi += d;
        }
        if norm(&[dx[0], dx[1], dx[2]]) < 1e-3 {
            break;
        }
    }
    let residuals = rows_at(&x).iter().map(|(_, b, _)| *b).collect();
    Some(([x[0], x[1], x[2]], residuals))
}

/// Weighted least-squares velocity and clock drift from Doppler, dropping the worst satellite while its residual
/// is beyond [`OUTLIER_SIGMA`], as long as one more satellite than unknowns remains to check the rest.
fn solve_velocity(sats: &[Sat], position: [f64; 3]) -> Option<Solution> {
    let rows: Vec<([f64; 4], f64, f64)> = sats
        .iter()
        .map(|s| {
            let (sp, sv) = rotated(&s.state, position);
            let d = sub(&sp, &position);
            let r = norm(&d);
            let u = d.map(|v| v / r);
            // prr = (v_sat - v_rx) . u + drift, so prr - v_sat . u = -u . v_rx + drift.
            (
                [-u[0], -u[1], -u[2], 1.0],
                s.prr - dot(&sv, &u),
                1.0 / (s.prr_sigma * s.prr_sigma),
            )
        })
        .collect();
    let mut used: Vec<usize> = (0..rows.len()).collect();
    loop {
        let subset: Vec<_> = used.iter().map(|&i| rows[i]).collect();
        let (x, cov) = weighted_least_squares(&subset)?;
        let normalized: Vec<f64> = subset
            .iter()
            .map(|(a, b, w)| (b - (0..4).map(|k| a[k] * x[k]).sum::<f64>()) * w.sqrt())
            .collect();
        let (worst, worst_value) = normalized
            .iter()
            .enumerate()
            .map(|(i, v)| (i, v.abs()))
            .max_by(|a, b| a.1.total_cmp(&b.1))?;
        if worst_value > OUTLIER_SIGMA && used.len() > 5 {
            used.remove(worst);
            continue;
        }
        let residuals: Vec<f64> = normalized
            .iter()
            .zip(&subset)
            .map(|(n, (_, _, w))| n / w.sqrt())
            .collect();
        return Some(Solution {
            elapsed_realtime_ns: None,
            position,
            velocity: [x[0], x[1], x[2]],
            velocity_covariance: std::array::from_fn(|k| cov[(k / 3) * 4 + k % 3]),
            clock_drift: x[3],
            satellites_used: used.len(),
            satellites_rejected: rows.len() - used.len(),
            position_residual_rms: 0.0,
            residual_rms: (residuals.iter().map(|r| r * r).sum::<f64>() / residuals.len() as f64)
                .sqrt(),
        });
    }
}

/// Solves min sum w (a.x - b)^2 for 4 unknowns; returns x and its covariance (A^T W A)^-1.
fn weighted_least_squares(rows: &[([f64; 4], f64, f64)]) -> Option<([f64; 4], [f64; 16])> {
    let mut n = [0.0; 16];
    let mut rhs = [0.0; 4];
    for (a, b, w) in rows {
        for i in 0..4 {
            rhs[i] += w * a[i] * b;
            for j in 0..4 {
                n[4 * i + j] += w * a[i] * a[j];
            }
        }
    }
    let inv = invert4(&n)?;
    let x = std::array::from_fn(|i| (0..4).map(|j| inv[4 * i + j] * rhs[j]).sum());
    Some((x, inv))
}

/// Inverse of a 4x4 matrix by Gauss-Jordan elimination with partial pivoting.
fn invert4(m: &[f64; 16]) -> Option<[f64; 16]> {
    let mut a = *m;
    let mut inv = [0.0; 16];
    for i in 0..4 {
        inv[5 * i] = 1.0;
    }
    for col in 0..4 {
        let pivot =
            (col..4).max_by(|&i, &j| a[4 * i + col].abs().total_cmp(&a[4 * j + col].abs()))?;
        if a[4 * pivot + col].abs() < 1e-12 {
            return None;
        }
        for k in 0..4 {
            a.swap(4 * col + k, 4 * pivot + k);
            inv.swap(4 * col + k, 4 * pivot + k);
        }
        let p = a[4 * col + col];
        for k in 0..4 {
            a[4 * col + k] /= p;
            inv[4 * col + k] /= p;
        }
        for row in (0..4).filter(|&r| r != col) {
            let f = a[4 * row + col];
            for k in 0..4 {
                a[4 * row + k] -= f * a[4 * col + k];
                inv[4 * row + k] -= f * inv[4 * col + k];
            }
        }
    }
    Some(inv)
}

fn sub(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(a: &[f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// WGS 84 geodetic coordinates (degrees, meters) to ECEF (m).
pub fn geodetic_to_ecef(lat_deg: f64, lon_deg: f64, alt_m: f64) -> [f64; 3] {
    const A: f64 = 6_378_137.0;
    const E2: f64 = 6.694_379_990_14e-3;
    let (sl, cl) = lat_deg.to_radians().sin_cos();
    let (so, co) = lon_deg.to_radians().sin_cos();
    let n = A / (1.0 - E2 * sl * sl).sqrt();
    [
        (n + alt_m) * cl * co,
        (n + alt_m) * cl * so,
        (n * (1.0 - E2) + alt_m) * sl,
    ]
}

//! Position and velocity from one epoch of Android's raw GNSS measurements, from every constellation with orbits.
//!
//! Velocity comes from each satellite's Doppler (`pseudorange_rate`), which measures speed directly and to a few
//! cm/s per satellite, by weighted least squares with outlier rejection. Position, from pseudoranges, is only needed
//! for the directions to the satellites, so meters of error don't matter for velocity.

use std::collections::BTreeMap;

use ride_session::proto::{
    GnssClock, GnssMeasurement, GnssMeasurementsEvent, GnssNavigationMessage,
};

use crate::ephemeris::{C, Ephemeris, OMEGA_E, SECONDS_PER_WEEK, SatState};
use crate::inav::{self, TYPE_GAL_I};
use crate::lnav::{EphemerisAssembler, TYPE_GPS_L1CA, TYPE_QZS_L1CA, decode_subframe};
use crate::system::{DEFAULT_LEAP_SECONDS, NANOS_PER_WEEK, SatId, System};

/// Signal travel times (ns) outside this are bad times: medium orbits take 64 to 100 ms,
/// geostationary and inclined geosynchronous ones (BeiDou, QZSS) up to about 140 ms.
const TRAVEL_NS: std::ops::Range<f64> = 0.06e9..0.15e9;
/// Carrier frequencies (Hz) of the L1 band (GPS L1, Galileo E1, BeiDou B1, GLONASS G1).
/// Only these are used: broadcast clocks and group delays refer to them, and other bands need other corrections.
const L1_BAND_HZ: std::ops::Range<f32> = 1.55e9..1.61e9;
/// Pseudorange residuals beyond this (m) reject the worst satellite from the position solution: uncorrected
/// ionosphere and troposphere are tens of meters at most, so more is multipath or a bad measurement.
const POSITION_OUTLIER_M: f64 = 60.0;
/// Added (m) to each axis's sigma, in quadrature, for a position with no satellite to spare:
/// nothing checks it, so one bad satellite as far off as [`POSITION_OUTLIER_M`] would go unnoticed.
const UNCHECKED_POSITION_SIGMA_M: f64 = POSITION_OUTLIER_M;
/// Pseudoranges are trusted no more than this (m), whatever Android says.
const MIN_PSEUDORANGE_SIGMA: f64 = 1.0;
/// Doppler measurements are trusted no more than this (m/s), whatever Android says.
const MIN_PRR_SIGMA: f64 = 0.05;
/// Residuals beyond this many sigma reject the worst satellite and solve again.
const OUTLIER_SIGMA: f64 = 4.0;
/// Weaker signals than this are too noisy and multipath-prone to use.
const MIN_CN0_DBHZ: f64 = 20.0;

/// One epoch's solution: position and velocity, each solved on its own, so either can fail without the other.
#[derive(Clone, Debug, PartialEq)]
pub struct Solution {
    /// The epoch on the elapsed realtime clock, if Android gave it.
    pub elapsed_realtime_ns: Option<i64>,
    /// From this epoch's pseudoranges, if there were enough.
    pub position: Option<PositionSolution>,
    /// From this epoch's Doppler, if there were enough, with the directions to the satellites from this epoch's
    /// position or else the latest one (meters or even kilometers off barely change them).
    pub velocity: Option<VelocitySolution>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PositionSolution {
    /// ECEF (m).
    pub position: [f64; 3],
    /// Receiver clock bias (m) against each constellation.
    pub clock_biases: Vec<(System, f64)>,
    /// Covariance of `position` (m^2), row-major,
    /// widened by [`UNCHECKED_POSITION_SIGMA_M`] when nothing checked it.
    pub position_covariance: [f64; 9],
    pub satellites_used: usize,
    /// Root mean square of the pseudorange residuals (m): zero when nothing checked the position.
    pub residual_rms: f64,
    /// Whether a satellite to spare checked the position for outliers.
    pub checked: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct VelocitySolution {
    /// ECEF (m/s).
    pub velocity: [f64; 3],
    /// Covariance of `velocity` (m^2/s^2), row-major.
    pub velocity_covariance: [f64; 9],
    /// Receiver clock drift (m/s).
    pub clock_drift: f64,
    pub satellites_used: usize,
    pub satellites_rejected: usize,
    /// How many of the satellites used each constellation gave.
    pub satellites_by_system: Vec<(System, usize)>,
    /// Root mean square of the Doppler residuals (m/s).
    pub residual_rms: f64,
    /// Whether the directions to the satellites came from this epoch's own position.
    pub with_this_position: bool,
}

impl VelocitySolution {
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

/// The full GPS week of an epoch's receive time.
fn epoch_week(event: &GnssMeasurementsEvent) -> Result<u32, NoSolution> {
    let clock = event.clock.as_ref().ok_or(NoSolution::NoClock)?;
    let rx_gps_ns = clock.time_nanos - clock.full_bias_nanos.ok_or(NoSolution::NoClock)?;
    u32::try_from(rx_gps_ns.div_euclid(NANOS_PER_WEEK)).map_err(|_| NoSolution::NoClock)
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
    /// Of those, how many came from the phone's navigation messages.
    pub ephemerides_decoded: u64,
    pub epochs: u64,
    /// Epochs with a position, a velocity, or both.
    pub epochs_solved: u64,
    pub positions_solved: u64,
    pub velocities_solved: u64,
}

/// Keeps ephemerides from navigation messages and solves epochs of measurements.
#[derive(Default)]
pub struct Navigator {
    assembler: EphemerisAssembler,
    galileo: inav::EphemerisAssembler,
    /// Each with whether it was decoded from the phone's navigation messages (or downloaded).
    ephemerides: BTreeMap<SatId, Vec<(Ephemeris, bool)>>,
    /// Whether to ignore downloaded ephemerides, to check the phone's navigation message decoding alone.
    pub decoded_only: bool,
    /// Full GPS week from the latest clock, to resolve navigation messages' 10-bit weeks.
    week: Option<u32>,
    /// Subframes that arrived before any clock.
    waiting: Vec<(SatId, crate::lnav::Subframe)>,
    last_position: Option<[f64; 3]>,
    pub stats: Stats,
}

impl Navigator {
    /// Adds an ephemeris from elsewhere (e.g. a downloaded broadcast ephemeris file).
    pub fn add_ephemeris(&mut self, eph: impl Into<Ephemeris>) {
        self.add(eph.into(), false);
    }

    /// Adds one decoded from the phone's navigation messages: used only where no downloaded one is valid,
    /// since a decoder bug in Android's bit format would otherwise compete with good orbits.
    fn add_decoded(&mut self, eph: impl Into<Ephemeris>) {
        self.add(eph.into(), true);
    }

    fn add(&mut self, eph: Ephemeris, decoded: bool) {
        let list = self.ephemerides.entry(eph.sat()).or_default();
        if !list.iter().any(|(e, d)| *d == decoded && e.same_as(&eph)) {
            list.push((eph, decoded));
            self.stats.ephemerides += 1;
            if decoded {
                self.stats.ephemerides_decoded += 1;
            }
        }
    }

    pub fn on_navigation_message(&mut self, msg: &GnssNavigationMessage) {
        let Ok(svid) = u8::try_from(msg.svid) else {
            return;
        };
        let system = match msg.r#type {
            TYPE_GPS_L1CA => System::Gps,
            // QZSS L1 C/A has GPS's LNAV format and time.
            TYPE_QZS_L1CA => System::Qzss,
            TYPE_GAL_I => {
                match inav::decode_page(&msg.data) {
                    Ok(word) => {
                        self.stats.subframes_ok += 1;
                        if let Some(eph) = self.galileo.add(svid, word) {
                            self.add_decoded(eph);
                        }
                    }
                    Err(_) => self.stats.subframes_bad += 1,
                }
                return;
            }
            _ => return,
        };
        let sat = SatId { system, svid };
        match decode_subframe(&msg.data) {
            Ok(sf) => {
                self.stats.subframes_ok += 1;
                match self.week {
                    Some(week) => {
                        if let Some(eph) = self.assembler.add(sat, sf, week) {
                            self.add_decoded(eph);
                        }
                    }
                    None if self.waiting.len() < 1000 => self.waiting.push((sat, sf)),
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
        let week = epoch_week(event)?;
        if self.week != Some(week) {
            self.week = Some(week);
            for (sat, sf) in std::mem::take(&mut self.waiting) {
                if let Some(eph) = self.assembler.add(sat, sf, week) {
                    self.add_decoded(eph);
                }
            }
        }
        let sol = self.solve(event)?;
        if let Some(p) = &sol.position {
            self.last_position = Some(p.position);
            self.stats.positions_solved += 1;
        }
        if sol.velocity.is_some() {
            self.stats.velocities_solved += 1;
        }
        self.stats.epochs_solved += 1;
        Ok(sol)
    }

    /// The solution [`Self::on_measurements`] would give, changing nothing: for a provisional estimate from
    /// measurements that haven't settled into time order yet.
    pub fn solve(&self, event: &GnssMeasurementsEvent) -> Result<Solution, NoSolution> {
        let clock = event.clock.as_ref().ok_or(NoSolution::NoClock)?;
        let week = epoch_week(event)?;
        let rx_gps_ns = clock.time_nanos - clock.full_bias_nanos.ok_or(NoSolution::NoClock)?;
        let sats = self.satellites(clock, &event.measurements, rx_gps_ns, week);
        // Position needs as many satellites as its unknowns (position, and a clock bias per constellation),
        // velocity one more than its own (velocity and clock drift): each is tried whatever the other did.
        let position = solve_position(&sats, self.last_position);
        let velocity = position
            .as_ref()
            .map(|p| p.position)
            .or(self.last_position)
            .and_then(|at| {
                let mut v = solve_velocity(&sats, at)?;
                v.with_this_position = position.is_some();
                Some(v)
            });
        if position.is_none() && velocity.is_none() {
            return Err(NoSolution::TooFewSatellites { usable: sats.len() });
        }
        Ok(Solution {
            elapsed_realtime_ns: clock.elapsed_realtime_nanos,
            position,
            velocity,
        })
    }

    /// Usable measurements with their satellites' states, one per satellite (the strongest L1-band signal).
    fn satellites(
        &self,
        clock: &GnssClock,
        measurements: &[GnssMeasurement],
        rx_gps_ns: i64,
        week: u32,
    ) -> Vec<Sat> {
        let bias = clock.bias_nanos.unwrap_or(0.0);
        let leap_seconds = clock.leap_second.unwrap_or(DEFAULT_LEAP_SECONDS);
        let mut best: BTreeMap<SatId, Sat> = BTreeMap::new();
        for m in measurements {
            let Some(system) = System::from_android(m.constellation_type) else {
                continue;
            };
            if !system.time_known(m.state)
                || m.cn0_dbhz < MIN_CN0_DBHZ
                || !m.pseudorange_rate_meters_per_second.is_finite()
                || m.carrier_frequency_hz
                    .is_some_and(|f| !L1_BAND_HZ.contains(&f))
            {
                continue;
            }
            let Ok(svid) = u8::try_from(m.svid) else {
                continue;
            };
            // GLONASS reports its frequency channel (93 to 106) instead when it doesn't know the slot.
            if system == System::Glonass && svid > 27 {
                continue;
            }
            let id = SatId { system, svid };
            // Receive time as the system counts it (ns, with sub-ns parts kept as f64),
            // less the transmit time in the satellite's clock, wrapped into the system's week (or day).
            let period = system.period_ns() as f64;
            let rx_ns = system.time_of_period_ns(rx_gps_ns, leap_seconds) as f64
                + m.time_offset_nanos
                - bias;
            let travel_ns = (rx_ns - m.received_sv_time_nanos as f64 + period / 2.0)
                .rem_euclid(period)
                - period / 2.0;
            if !TRAVEL_NS.contains(&travel_ns) {
                continue;
            }
            // Transmit time in the satellite's clock, as GPS seconds of week.
            let rx_tow_ns =
                rx_gps_ns.rem_euclid(NANOS_PER_WEEK) as f64 + m.time_offset_nanos - bias;
            let t_sv = (rx_tow_ns - travel_ns) * 1e-9;
            let Some(eph) = self.ephemeris(id, week, t_sv) else {
                continue;
            };
            // Satellite clock time to GPS time, then the satellite's state at transmission.
            let dt = eph.state_at(t_sv).clock_bias;
            let state = eph.state_at(t_sv - dt);
            let sat = Sat {
                system,
                state,
                pseudorange: travel_ns * 1e-9 * C + C * (state.clock_bias - eph.tgd()),
                pseudorange_sigma: (m.received_sv_time_uncertainty_nanos as f64 * 1e-9 * C)
                    .max(MIN_PSEUDORANGE_SIGMA),
                prr: m.pseudorange_rate_meters_per_second + C * state.clock_drift,
                prr_sigma: m
                    .pseudorange_rate_uncertainty_meters_per_second
                    .max(MIN_PRR_SIGMA),
                cn0: m.cn0_dbhz,
            };
            if best.get(&id).is_none_or(|b| b.cn0 < sat.cn0) {
                best.insert(id, sat);
            }
        }
        best.into_values().collect()
    }

    /// The ephemeris for `id` valid at `t` (GPS seconds of `week`) with the closest `toe`,
    /// preferring downloaded ones to decoded ones.
    fn ephemeris(&self, id: SatId, week: u32, t: f64) -> Option<&Ephemeris> {
        let at = f64::from(week) * SECONDS_PER_WEEK + t;
        self.ephemerides
            .get(&id)?
            .iter()
            .filter(|(e, decoded)| e.healthy() && (*decoded || !self.decoded_only))
            .map(|(e, decoded)| (e, *decoded, (at - e.toe_gps_seconds()).abs()))
            .filter(|&(e, _, d)| d <= e.valid_for())
            .min_by(|a, b| a.1.cmp(&b.1).then(a.2.total_cmp(&b.2)))
            .map(|(e, _, _)| e)
    }
}

#[derive(Clone, Copy, Debug)]
struct Sat {
    system: System,
    state: SatState,
    /// Corrected for the satellite clock and group delay (m).
    pseudorange: f64,
    pseudorange_sigma: f64,
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

/// The constellations among `sats`, in order: each one's clock bias is an unknown of the position solution.
fn systems_of(sats: &[&Sat]) -> Vec<System> {
    let mut systems: Vec<System> = sats.iter().map(|s| s.system).collect();
    systems.sort();
    systems.dedup();
    systems
}

/// Weighted least-squares position (and a clock bias per constellation) from pseudoranges, by Gauss-Newton from `start`
/// or the earth's center, dropping the worst satellite while its residual is beyond [`POSITION_OUTLIER_M`]
/// and a redundant one remains. With exactly as many satellites as unknowns, nothing can be checked,
/// so the position comes with a wide uncertainty instead of not at all.
fn solve_position(sats: &[Sat], start: Option<[f64; 3]>) -> Option<PositionSolution> {
    let mut used: Vec<&Sat> = sats.iter().collect();
    loop {
        let systems = systems_of(&used);
        let unknowns = 3 + systems.len();
        if used.len() < unknowns {
            return None;
        }
        let checked = used.len() > unknowns;
        let (x, cov, residuals) = gauss_newton_position(&used, &systems, start)?;
        let (worst, worst_value) = residuals
            .iter()
            .enumerate()
            .map(|(i, r)| (i, r.abs()))
            .max_by(|a, b| a.1.total_cmp(&b.1))?;
        if worst_value > POSITION_OUTLIER_M && used.len() > unknowns + 1 {
            used.remove(worst);
            continue;
        }
        let rms = (residuals.iter().map(|r| r * r).sum::<f64>() / residuals.len() as f64).sqrt();
        let n = x.len();
        let mut position_covariance: [f64; 9] = std::array::from_fn(|k| cov[(k / 3) * n + k % 3]);
        if !checked {
            for k in [0, 4, 8] {
                position_covariance[k] += UNCHECKED_POSITION_SIGMA_M * UNCHECKED_POSITION_SIGMA_M;
            }
        }
        return Some(PositionSolution {
            position: [x[0], x[1], x[2]],
            clock_biases: systems.into_iter().zip(x[3..].iter().copied()).collect(),
            position_covariance,
            satellites_used: used.len(),
            residual_rms: rms,
            checked,
        });
    }
}

/// Returns the unknowns (position, then each of `systems`' clock bias), their covariance (row-major),
/// and the residuals.
fn gauss_newton_position(
    sats: &[&Sat],
    systems: &[System],
    start: Option<[f64; 3]>,
) -> Option<(Vec<f64>, Vec<f64>, Vec<f64>)> {
    let mut x = vec![0.0; 3 + systems.len()];
    if let Some(p) = start {
        x[..3].copy_from_slice(&p);
    }
    let rows_at = |x: &[f64]| -> Vec<(Vec<f64>, f64, f64)> {
        sats.iter()
            .map(|s| {
                let rx = [x[0], x[1], x[2]];
                let (sp, _) = rotated(&s.state, rx);
                let d = sub(&sp, &rx);
                let r = norm(&d);
                let u = d.map(|v| v / r);
                let k = 3 + systems.iter().position(|&sys| sys == s.system).unwrap();
                let mut a = vec![0.0; x.len()];
                a[..3].copy_from_slice(&[-u[0], -u[1], -u[2]]);
                a[k] = 1.0;
                (
                    a,
                    s.pseudorange - (r + x[k]),
                    1.0 / (s.pseudorange_sigma * s.pseudorange_sigma),
                )
            })
            .collect()
    };
    for _ in 0..10 {
        let (dx, _) = weighted_least_squares(&rows_at(&x))?;
        for (xi, d) in x.iter_mut().zip(&dx) {
            *xi += d;
        }
        if norm(&[dx[0], dx[1], dx[2]]) < 1e-3 {
            break;
        }
    }
    let rows = rows_at(&x);
    let (_, cov) = weighted_least_squares(&rows)?;
    let residuals = rows.iter().map(|(_, b, _)| *b).collect();
    Some((x, cov, residuals))
}

/// Weighted least-squares velocity and clock drift from Doppler, dropping the worst satellite while its residual
/// is beyond [`OUTLIER_SIGMA`], as long as one more satellite than unknowns remains to check the rest.
fn solve_velocity(sats: &[Sat], position: [f64; 3]) -> Option<VelocitySolution> {
    // Velocity and clock drift, and one more to check them.
    if sats.len() < 5 {
        return None;
    }
    let rows: Vec<(Vec<f64>, f64, f64)> = sats
        .iter()
        .map(|s| {
            let (sp, sv) = rotated(&s.state, position);
            let d = sub(&sp, &position);
            let r = norm(&d);
            let u = d.map(|v| v / r);
            // prr = (v_sat - v_rx) . u + drift, so prr - v_sat . u = -u . v_rx + drift.
            (
                vec![-u[0], -u[1], -u[2], 1.0],
                s.prr - dot(&sv, &u),
                1.0 / (s.prr_sigma * s.prr_sigma),
            )
        })
        .collect();
    let mut used: Vec<usize> = (0..rows.len()).collect();
    loop {
        let subset: Vec<_> = used.iter().map(|&i| rows[i].clone()).collect();
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
        return Some(VelocitySolution {
            velocity: [x[0], x[1], x[2]],
            velocity_covariance: std::array::from_fn(|k| cov[(k / 3) * 4 + k % 3]),
            clock_drift: x[3],
            satellites_by_system: systems_of(&used.iter().map(|&i| &sats[i]).collect::<Vec<_>>())
                .into_iter()
                .map(|sys| (sys, used.iter().filter(|&&i| sats[i].system == sys).count()))
                .collect(),
            satellites_used: used.len(),
            satellites_rejected: rows.len() - used.len(),
            with_this_position: false,
            residual_rms: (residuals.iter().map(|r| r * r).sum::<f64>() / residuals.len() as f64)
                .sqrt(),
        });
    }
}

/// Solves min sum w (a.x - b)^2; returns x and its covariance (A^T W A)^-1, row-major.
fn weighted_least_squares(rows: &[(Vec<f64>, f64, f64)]) -> Option<(Vec<f64>, Vec<f64>)> {
    let n = rows.first()?.0.len();
    let mut normal = vec![0.0; n * n];
    let mut rhs = vec![0.0; n];
    for (a, b, w) in rows {
        for i in 0..n {
            rhs[i] += w * a[i] * b;
            for j in 0..n {
                normal[n * i + j] += w * a[i] * a[j];
            }
        }
    }
    let inv = invert(&normal, n)?;
    let x = (0..n)
        .map(|i| (0..n).map(|j| inv[n * i + j] * rhs[j]).sum())
        .collect();
    Some((x, inv))
}

/// Inverse of an n x n matrix by Gauss-Jordan elimination with partial pivoting.
fn invert(m: &[f64], n: usize) -> Option<Vec<f64>> {
    let mut a = m.to_vec();
    let mut inv = vec![0.0; n * n];
    for i in 0..n {
        inv[(n + 1) * i] = 1.0;
    }
    for col in 0..n {
        let pivot =
            (col..n).max_by(|&i, &j| a[n * i + col].abs().total_cmp(&a[n * j + col].abs()))?;
        if a[n * pivot + col].abs() < 1e-12 {
            return None;
        }
        for k in 0..n {
            a.swap(n * col + k, n * pivot + k);
            inv.swap(n * col + k, n * pivot + k);
        }
        let p = a[n * col + col];
        for k in 0..n {
            a[n * col + k] /= p;
            inv[n * col + k] /= p;
        }
        for row in (0..n).filter(|&r| r != col) {
            let f = a[n * row + col];
            for k in 0..n {
                a[n * row + k] -= f * a[n * col + k];
                inv[n * row + k] -= f * inv[n * col + k];
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A downloaded ephemeris wins over a decoded one for the same time, unless only decoded ones are wanted.
    #[test]
    fn prefers_downloaded_ephemerides() {
        let rinex = crate::rinex::read_nav(include_str!("../tests/data/brdc1820.10n"));
        let downloaded = rinex.into_iter().find(|e| e.healthy()).unwrap();
        let Ephemeris::Kepler(mut decoded) = downloaded.clone() else {
            panic!("GPS ephemeris expected");
        };
        decoded.af0 += 1e-3;
        let decoded = Ephemeris::from(decoded);
        let sat = downloaded.sat();
        let toe = downloaded.toe_gps_seconds();
        let (week, t) = (
            (toe / SECONDS_PER_WEEK) as u32,
            toe.rem_euclid(SECONDS_PER_WEEK),
        );

        let mut nav = Navigator::default();
        nav.add_decoded(decoded.clone());
        nav.add_ephemeris(downloaded.clone());
        assert_eq!(nav.stats.ephemerides_decoded, 1);
        assert_eq!(nav.ephemeris(sat, week, t), Some(&downloaded));
        nav.decoded_only = true;
        assert_eq!(nav.ephemeris(sat, week, t), Some(&decoded));
    }
}

//! Keplerian broadcast ephemerides (GPS and QZSS LNAV, Galileo I/NAV, BeiDou D1/D2),
//! and satellite position, velocity, and clock from them, per IS-GPS-200, the Galileo OS SIS ICD,
//! and the BDS SIS ICD.

use crate::system::{BDT_BEHIND_GPS_NS, SatId, System};

/// WGS 84 gravitational constant (m^3/s^2), as IS-GPS-200 specifies.
pub const MU: f64 = 3.986_005e14;
/// Galileo's (GTRF) and BeiDou's (CGCS2000) gravitational constant (m^3/s^2).
pub const MU_GALILEO_BEIDOU: f64 = 3.986_004_418e14;
/// WGS 84 earth rotation rate (rad/s), also Galileo's.
pub const OMEGA_E: f64 = 7.292_115_146_7e-5;
/// CGCS2000 earth rotation rate (rad/s), BeiDou's.
pub const OMEGA_E_BEIDOU: f64 = 7.292_115_0e-5;
/// Speed of light (m/s).
pub const C: f64 = 299_792_458.0;
/// The value of pi IS-GPS-200 uses to convert semicircles.
#[allow(
    clippy::approx_constant,
    reason = "IS-GPS-200 specifies this exact value, not the true pi"
)]
pub const GPS_PI: f64 = 3.141_592_653_589_8;
/// Relativistic clock correction constant (s/m^(1/2)) for GPS; the others compute it from their own `MU`.
const F: f64 = -4.442_807_633e-10;
pub const SECONDS_PER_WEEK: f64 = 604_800.0;

/// One satellite's Keplerian broadcast ephemeris and clock, in SI units (angles in radians).
/// Times are in the satellite's own system time: GPS time for GPS, QZSS, and Galileo (whose weeks are counted
/// like GPS's here, as in RINEX), BeiDou time for BeiDou.
#[derive(Clone, Debug, PartialEq)]
pub struct KeplerEphemeris {
    pub sat: SatId,
    /// Full week of `toe` (rollovers resolved).
    pub week: u32,
    pub sv_accuracy_index: u8,
    pub sv_health: u8,
    pub iodc: u16,
    /// Issue of data (IODE for GPS and QZSS, IODnav for Galileo, AODE for BeiDou).
    pub iode: u16,
    /// Group delay (s) of the L1-band signal, to subtract from the clock: GPS's TGD, Galileo's BGD for the
    /// clock's frequency pair, BeiDou's TGD1.
    pub tgd: f64,
    pub toc: f64,
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub crs: f64,
    pub delta_n: f64,
    pub m0: f64,
    pub cuc: f64,
    pub e: f64,
    pub cus: f64,
    pub sqrt_a: f64,
    pub toe: f64,
    pub cic: f64,
    pub omega0: f64,
    pub cis: f64,
    pub i0: f64,
    pub crc: f64,
    pub omega: f64,
    pub omega_dot: f64,
    pub idot: f64,
}

/// A satellite's state at one transmit time, in ECEF (WGS 84) at that time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SatState {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    /// Satellite clock offset (s), including the relativistic term, excluding the group delay.
    pub clock_bias: f64,
    /// Satellite clock drift (s/s).
    pub clock_drift: f64,
}

/// Seconds from `b` to `a` within a week, wrapped into [-half a week, half a week).
fn week_diff(a: f64, b: f64) -> f64 {
    let mut d = a - b;
    if d > SECONDS_PER_WEEK / 2.0 {
        d -= SECONDS_PER_WEEK;
    } else if d < -SECONDS_PER_WEEK / 2.0 {
        d += SECONDS_PER_WEEK;
    }
    d
}

impl KeplerEphemeris {
    fn mu(&self) -> f64 {
        match self.sat.system {
            System::Gps | System::Qzss => MU,
            _ => MU_GALILEO_BEIDOU,
        }
    }

    fn omega_e(&self) -> f64 {
        match self.sat.system {
            System::Beidou => OMEGA_E_BEIDOU,
            _ => OMEGA_E,
        }
    }

    fn relativistic_f(&self) -> f64 {
        match self.sat.system {
            System::Gps | System::Qzss => F,
            _ => -2.0 * self.mu().sqrt() / (C * C),
        }
    }

    /// BeiDou's geostationary satellites, whose orbits are broadcast in a frame tilted by 5 degrees.
    fn is_beidou_geo(&self) -> bool {
        self.sat.system == System::Beidou && (self.sat.svid <= 5 || self.sat.svid >= 59)
    }

    /// GPS seconds of week as seconds of week in the satellite's system time.
    fn system_time(&self, t_gps: f64) -> f64 {
        match self.sat.system {
            System::Beidou => t_gps - BDT_BEHIND_GPS_NS as f64 * 1e-9,
            _ => t_gps,
        }
    }

    /// `toe` as GPS seconds since the GPS epoch, to compare ephemerides across systems.
    pub fn toe_gps_seconds(&self) -> f64 {
        let t = f64::from(self.week) * SECONDS_PER_WEEK + self.toe;
        match self.sat.system {
            // BeiDou weeks count from 2006-01-01, 1356 GPS weeks later.
            System::Beidou => t + 1356.0 * SECONDS_PER_WEEK + BDT_BEHIND_GPS_NS as f64 * 1e-9,
            _ => t,
        }
    }

    /// Whether the ephemeris is usable at GPS seconds-of-week `t` (healthy and within 4 h of `toe`).
    pub fn valid_at(&self, t: f64) -> bool {
        self.sv_health == 0 && week_diff(self.system_time(t), self.toe).abs() <= 4.0 * 3600.0
    }

    /// Position and eccentric anomaly at seconds-of-week `t` in the satellite's system time.
    fn position_at(&self, t: f64) -> ([f64; 3], f64) {
        let a = self.sqrt_a * self.sqrt_a;
        let tk = week_diff(t, self.toe);
        let omega_e = self.omega_e();
        let n = (self.mu() / (a * a * a)).sqrt() + self.delta_n;
        let m = self.m0 + n * tk;
        // Kepler's equation by Newton's method.
        let mut ecc = m;
        for _ in 0..20 {
            let d = (ecc - self.e * ecc.sin() - m) / (1.0 - self.e * ecc.cos());
            ecc -= d;
            if d.abs() < 1e-15 {
                break;
            }
        }
        let nu = ((1.0 - self.e * self.e).sqrt() * ecc.sin()).atan2(ecc.cos() - self.e);
        let phi = nu + self.omega;
        let (s2, c2) = (2.0 * phi).sin_cos();
        let u = phi + self.cus * s2 + self.cuc * c2;
        let r = a * (1.0 - self.e * ecc.cos()) + self.crs * s2 + self.crc * c2;
        let i = self.i0 + self.idot * tk + self.cis * s2 + self.cic * c2;
        let (x, y) = (r * u.cos(), r * u.sin());
        let (si, ci) = i.sin_cos();
        if self.is_beidou_geo() {
            // BDS SIS ICD section 5.2.4.12: in an inertial-like frame, then tilted by -5 degrees about x
            // and rotated by the earth's turn since toe.
            let big_omega = self.omega0 + self.omega_dot * tk - omega_e * self.toe;
            let (so, co) = big_omega.sin_cos();
            let g = [x * co - y * ci * so, x * so + y * ci * co, y * si];
            let (sf, cf) = (-5f64).to_radians().sin_cos();
            let tilted = [g[0], cf * g[1] + sf * g[2], -sf * g[1] + cf * g[2]];
            let (sz, cz) = (omega_e * tk).sin_cos();
            return (
                [
                    cz * tilted[0] + sz * tilted[1],
                    -sz * tilted[0] + cz * tilted[1],
                    tilted[2],
                ],
                ecc,
            );
        }
        let big_omega = self.omega0 + (self.omega_dot - omega_e) * tk - omega_e * self.toe;
        let (so, co) = big_omega.sin_cos();
        ([x * co - y * ci * so, x * so + y * ci * co, y * si], ecc)
    }

    fn clock_at(&self, t: f64, ecc: f64) -> f64 {
        let dt = week_diff(t, self.toc);
        self.af0
            + self.af1 * dt
            + self.af2 * dt * dt
            + self.relativistic_f() * self.e * self.sqrt_a * ecc.sin()
    }

    /// The satellite's state at GPS seconds-of-week `t` (a transmit time, in GPS time).
    /// Velocity and clock drift are central differences over 10 ms, far more precise than the measurements.
    pub fn state_at(&self, t: f64) -> SatState {
        const H: f64 = 0.005;
        let t = self.system_time(t);
        let (position, ecc) = self.position_at(t);
        let (before, ecc_before) = self.position_at(t - H);
        let (after, ecc_after) = self.position_at(t + H);
        SatState {
            position,
            velocity: std::array::from_fn(|k| (after[k] - before[k]) / (2.0 * H)),
            clock_bias: self.clock_at(t, ecc),
            clock_drift: (self.clock_at(t + H, ecc_after) - self.clock_at(t - H, ecc_before))
                / (2.0 * H),
        }
    }
}

/// PZ-90 gravitational constant (m^3/s^2), equatorial radius (m), second zonal harmonic, and earth rotation
/// rate (rad/s), as the GLONASS ICD gives them for orbit integration.
const MU_GLONASS: f64 = 3.986_004_4e14;
const A_E_GLONASS: f64 = 6_378_136.0;
const J2_GLONASS: f64 = 1.082_625_7e-3;
const OMEGA_E_GLONASS: f64 = 7.292_115e-5;
/// Integration step (s): RK4's error at this step is far below a millimeter over 15 minutes.
const GLONASS_STEP: f64 = 30.0;
/// A GLONASS ephemeris is broadcast every 30 minutes, for 15 minutes either side, but a downloaded file can lag
/// the newest broadcast by most of an hour. Integrated further, its error grows about with the square of the time:
/// against the next broadcasts on 2026-10-20 (936 pairs), median orbit and clock error 2.0 m at 30 minutes,
/// 7.5 m at 60, 18 m at 90, 35 m at 120. So it's used for an hour, weighted by [`GLONASS_SIGMA_AT_30_MIN_M`].
const GLONASS_VALID_S: f64 = 60.0 * 60.0;
/// GLONASS orbit and clock error (m) 30 minutes from the reference time (between the median and 95th percentile
/// above), growing with the square of the time.
const GLONASS_SIGMA_AT_30_MIN_M: f64 = 3.0;

/// One GLONASS satellite's broadcast state and clock: position, velocity, and the moon's and sun's acceleration
/// in PZ-90 (taken as WGS 84, within centimeters) at `tb`.
#[derive(Clone, Debug, PartialEq)]
pub struct GlonassEphemeris {
    pub sat: SatId,
    /// Frequency channel, -7 to 6, if known (RINEX has it; strings 1 to 4 don't).
    pub frequency_channel: Option<i8>,
    pub healthy: bool,
    /// GPS week and seconds of week of the reference time `tb`.
    pub week: u32,
    pub tb: f64,
    /// Clock offset at `tb` (s, RINEX's -TauN) and its relative rate (GammaN).
    pub clock_bias: f64,
    pub clock_rate: f64,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub acceleration: [f64; 3],
}

impl GlonassEphemeris {
    fn derivative(&self, s: &[f64; 6]) -> [f64; 6] {
        let [x, y, z, vx, vy, vz] = *s;
        let r2 = x * x + y * y + z * z;
        let r = r2.sqrt();
        let mu_r3 = MU_GLONASS / (r2 * r);
        let j = 1.5 * J2_GLONASS * MU_GLONASS * A_E_GLONASS * A_E_GLONASS / (r2 * r2 * r);
        let z2 = 5.0 * z * z / r2;
        let w2 = OMEGA_E_GLONASS * OMEGA_E_GLONASS;
        let a = self.acceleration;
        [
            vx,
            vy,
            vz,
            -mu_r3 * x - j * x * (1.0 - z2) + w2 * x + 2.0 * OMEGA_E_GLONASS * vy + a[0],
            -mu_r3 * y - j * y * (1.0 - z2) + w2 * y - 2.0 * OMEGA_E_GLONASS * vx + a[1],
            -mu_r3 * z - j * z * (3.0 - z2) + a[2],
        ]
    }

    /// Position and velocity at `dt` seconds from `tb`, by fourth-order Runge-Kutta.
    fn integrate(&self, dt: f64) -> [f64; 6] {
        let mut s = [
            self.position[0],
            self.position[1],
            self.position[2],
            self.velocity[0],
            self.velocity[1],
            self.velocity[2],
        ];
        let steps = (dt.abs() / GLONASS_STEP).ceil().max(1.0);
        let h = dt / steps;
        let add = |s: &[f64; 6], k: &[f64; 6], f: f64| -> [f64; 6] {
            std::array::from_fn(|i| s[i] + f * k[i])
        };
        for _ in 0..steps as usize {
            let k1 = self.derivative(&s);
            let k2 = self.derivative(&add(&s, &k1, h / 2.0));
            let k3 = self.derivative(&add(&s, &k2, h / 2.0));
            let k4 = self.derivative(&add(&s, &k3, h));
            s = std::array::from_fn(|i| {
                s[i] + h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
            });
        }
        s
    }

    pub fn toe_gps_seconds(&self) -> f64 {
        f64::from(self.week) * SECONDS_PER_WEEK + self.tb
    }

    /// The satellite's state at GPS seconds-of-week `t` (a transmit time, in GPS time).
    pub fn state_at(&self, t: f64) -> SatState {
        let dt = week_diff(t, self.tb);
        let s = self.integrate(dt);
        SatState {
            position: [s[0], s[1], s[2]],
            velocity: [s[3], s[4], s[5]],
            clock_bias: self.clock_bias + self.clock_rate * dt,
            clock_drift: self.clock_rate,
        }
    }
}

/// A broadcast ephemeris of either kind.
#[derive(Clone, Debug, PartialEq)]
pub enum Ephemeris {
    Kepler(KeplerEphemeris),
    Glonass(GlonassEphemeris),
}

impl From<KeplerEphemeris> for Ephemeris {
    fn from(e: KeplerEphemeris) -> Self {
        Self::Kepler(e)
    }
}

impl From<GlonassEphemeris> for Ephemeris {
    fn from(e: GlonassEphemeris) -> Self {
        Self::Glonass(e)
    }
}

impl Ephemeris {
    pub fn sat(&self) -> SatId {
        match self {
            Self::Kepler(e) => e.sat,
            Self::Glonass(e) => e.sat,
        }
    }

    pub fn healthy(&self) -> bool {
        match self {
            Self::Kepler(e) => e.sv_health == 0,
            Self::Glonass(e) => e.healthy,
        }
    }

    /// The reference time, as GPS seconds since the GPS epoch.
    pub fn toe_gps_seconds(&self) -> f64 {
        match self {
            Self::Kepler(e) => e.toe_gps_seconds(),
            Self::Glonass(e) => e.toe_gps_seconds(),
        }
    }

    /// How far from the reference time the ephemeris may be used (s).
    pub fn valid_for(&self) -> f64 {
        match self {
            Self::Kepler(_) => 4.0 * 3600.0,
            Self::Glonass(_) => GLONASS_VALID_S,
        }
    }

    /// The orbit and clock's own uncertainty (m) at `at` (GPS seconds since the GPS epoch): GLONASS's grows as it's
    /// integrated away from its reference time; broadcast Kepler orbits stay within a few meters while valid.
    pub fn sigma_m(&self, at: f64) -> f64 {
        match self {
            Self::Kepler(_) => 0.0,
            Self::Glonass(_) => {
                let r = (at - self.toe_gps_seconds()).abs() / 1800.0;
                GLONASS_SIGMA_AT_30_MIN_M * r * r
            }
        }
    }

    /// Group delay (s) of the L1-band signal; GLONASS's clock already refers to L1.
    pub fn tgd(&self) -> f64 {
        match self {
            Self::Kepler(e) => e.tgd,
            Self::Glonass(_) => 0.0,
        }
    }

    pub fn state_at(&self, t: f64) -> SatState {
        match self {
            Self::Kepler(e) => e.state_at(t),
            Self::Glonass(e) => e.state_at(t),
        }
    }

    /// Whether two ephemerides are the same broadcast (repeated in a file or by several messages).
    pub fn same_as(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Kepler(a), Self::Kepler(b)) => {
                a.week == b.week && a.toe == b.toe && a.iode == b.iode
            }
            (Self::Glonass(a), Self::Glonass(b)) => a.week == b.week && a.tb == b.tb,
            _ => false,
        }
    }
}

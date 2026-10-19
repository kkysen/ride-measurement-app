//! GPS broadcast ephemeris (LNAV), and satellite position, velocity, and clock from it, per IS-GPS-200.

/// WGS 84 gravitational constant (m^3/s^2), as IS-GPS-200 specifies.
pub const MU: f64 = 3.986_005e14;
/// WGS 84 earth rotation rate (rad/s).
pub const OMEGA_E: f64 = 7.292_115_146_7e-5;
/// Speed of light (m/s).
pub const C: f64 = 299_792_458.0;
/// The value of pi IS-GPS-200 uses to convert semicircles.
#[allow(
    clippy::approx_constant,
    reason = "IS-GPS-200 specifies this exact value, not the true pi"
)]
pub const GPS_PI: f64 = 3.141_592_653_589_8;
/// Relativistic clock correction constant (s/m^(1/2)).
const F: f64 = -4.442_807_633e-10;
pub const SECONDS_PER_WEEK: f64 = 604_800.0;

/// One GPS satellite's broadcast ephemeris and clock, in SI units (angles in radians).
#[derive(Clone, Debug, PartialEq)]
pub struct GpsEphemeris {
    pub prn: u8,
    /// Full GPS week of `toe` (rollovers resolved).
    pub week: u32,
    pub sv_accuracy_index: u8,
    pub sv_health: u8,
    pub iodc: u16,
    pub iode: u8,
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

impl GpsEphemeris {
    /// Whether the ephemeris is usable at GPS seconds-of-week `t` (healthy and within 4 h of `toe`).
    pub fn valid_at(&self, t: f64) -> bool {
        self.sv_health == 0 && week_diff(t, self.toe).abs() <= 4.0 * 3600.0
    }

    /// Position and eccentric anomaly at GPS seconds-of-week `t`.
    fn position_at(&self, t: f64) -> ([f64; 3], f64) {
        let a = self.sqrt_a * self.sqrt_a;
        let tk = week_diff(t, self.toe);
        let n = (MU / (a * a * a)).sqrt() + self.delta_n;
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
        let big_omega = self.omega0 + (self.omega_dot - OMEGA_E) * tk - OMEGA_E * self.toe;
        let (so, co) = big_omega.sin_cos();
        let (si, ci) = i.sin_cos();
        ([x * co - y * ci * so, x * so + y * ci * co, y * si], ecc)
    }

    fn clock_at(&self, t: f64, ecc: f64) -> f64 {
        let dt = week_diff(t, self.toc);
        self.af0 + self.af1 * dt + self.af2 * dt * dt + F * self.e * self.sqrt_a * ecc.sin()
    }

    /// The satellite's state at GPS seconds-of-week `t` (a transmit time, in GPS time).
    /// Velocity and clock drift are central differences over 10 ms, far more precise than the measurements.
    pub fn state_at(&self, t: f64) -> SatState {
        const H: f64 = 0.005;
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

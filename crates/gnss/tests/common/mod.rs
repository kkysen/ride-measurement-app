//! Shared test helpers.

#![allow(dead_code, reason = "each test uses only some")]

use ride_session::proto::GnssMeasurementsEvent;

/// A GnssLogger log's raw measurements, one event per epoch.
pub fn raw_epochs(log: &str) -> Vec<GnssMeasurementsEvent> {
    ride_gnss::gnsslogger::read(log).epochs
}

pub fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// East, north, up unit vectors at an ECEF position.
pub fn enu(p: [f64; 3]) -> [[f64; 3]; 3] {
    let lat = p[2].atan2(p[0].hypot(p[1]));
    let lon = p[1].atan2(p[0]);
    let (sl, cl) = lat.sin_cos();
    let (so, co) = lon.sin_cos();
    [
        [-so, co, 0.0],
        [-sl * co, -sl * so, cl],
        [cl * co, cl * so, sl],
    ]
}

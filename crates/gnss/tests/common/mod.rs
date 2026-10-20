//! Shared test helpers.

#![allow(dead_code, reason = "each test uses only some")]

use ride_session::proto::{GnssClock, GnssMeasurement, GnssMeasurementsEvent};

pub fn opt(s: &str) -> Option<f64> {
    (!s.is_empty()).then(|| s.parse().unwrap())
}

/// A GnssLogger log's `Raw` lines, one event per epoch. `elapsed_realtime_nanos` is the log's first column,
/// in ns: elapsed realtime in old logs, UTC (`utcTimeMillis`) in newer ones.
pub fn raw_epochs(log: &str) -> Vec<GnssMeasurementsEvent> {
    let mut events: Vec<GnssMeasurementsEvent> = Vec::new();
    for line in log.lines().filter(|l| l.starts_with("Raw,")) {
        let f: Vec<&str> = line.split(',').collect();
        let time_nanos: i64 = f[2].parse().unwrap();
        let measurement = GnssMeasurement {
            svid: f[11].parse().unwrap(),
            time_offset_nanos: f[12].parse().unwrap(),
            state: f[13].parse().unwrap(),
            received_sv_time_nanos: f[14].parse().unwrap(),
            received_sv_time_uncertainty_nanos: f[15].parse().unwrap(),
            cn0_dbhz: f[16].parse().unwrap(),
            pseudorange_rate_meters_per_second: f[17].parse().unwrap(),
            pseudorange_rate_uncertainty_meters_per_second: f[18].parse().unwrap(),
            carrier_frequency_hz: opt(f[22]).map(|f| f as f32),
            constellation_type: f[28].parse().unwrap(),
            ..Default::default()
        };
        match events.last_mut() {
            Some(e) if e.clock.as_ref().unwrap().time_nanos == time_nanos => {
                e.measurements.push(measurement);
            }
            _ => events.push(GnssMeasurementsEvent {
                clock: Some(GnssClock {
                    time_nanos,
                    leap_second: opt(f[3]).map(|s| s as i32),
                    full_bias_nanos: Some(f[5].parse().unwrap()),
                    bias_nanos: opt(f[6]),
                    bias_uncertainty_nanos: opt(f[7]),
                    drift_nanos_per_second: opt(f[8]),
                    hardware_clock_discontinuity_count: f[10].parse().unwrap(),
                    elapsed_realtime_nanos: Some(f[1].parse::<i64>().unwrap() * 1_000_000),
                    ..Default::default()
                }),
                measurements: vec![measurement],
                ..Default::default()
            }),
        }
    }
    events
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

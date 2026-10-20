//! Position and velocity from a real phone's raw measurements (Google's GnssLogger demo log, a phone at rest),
//! checked against the phone's own fixes. Unlike `solve.rs`, the measurements don't come from our own model,
//! so a sign or unit mistake shared by the solver and that model shows up here.

mod common;

use ride_gnss::solve::{Navigator, geodetic_to_ecef};
use ride_session::proto::{GnssClock, GnssMeasurement, GnssMeasurementsEvent};

const LOG: &str = include_str!("data/gnsslogger_2016_06_30_21_26_07.txt");

fn opt(s: &str) -> Option<f64> {
    (!s.is_empty()).then(|| s.parse().unwrap())
}

/// The log's `Raw` lines, one event per epoch.
fn epochs() -> Vec<GnssMeasurementsEvent> {
    let mut events: Vec<GnssMeasurementsEvent> = Vec::new();
    for line in LOG.lines().filter(|l| l.starts_with("Raw,")) {
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

/// The phone's own fix (its first one; it stays within a few meters).
fn phone_fix() -> [f64; 3] {
    let f: Vec<f64> = LOG
        .lines()
        .find(|l| l.starts_with("Fix,"))
        .unwrap()
        .split(',')
        .skip(2)
        .take(3)
        .map(|s| s.parse().unwrap())
        .collect();
    geodetic_to_ecef(f[0], f[1], f[2])
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// East, north, up unit vectors at an ECEF position.
fn enu(p: [f64; 3]) -> [[f64; 3]; 3] {
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

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

#[test]
fn real_phone_at_rest() {
    let mut nav = Navigator::default();
    for e in common::read_rinex_nav(include_str!("data/hour1820-20h-22h.16n")) {
        nav.add_ephemeris(e);
    }
    let events = epochs();
    let solutions: Vec<_> = events
        .iter()
        .filter_map(|e| nav.on_measurements(e).ok())
        .collect();
    assert!(
        solutions.len() * 10 >= events.len() * 9,
        "solved {} of {} epochs",
        solutions.len(),
        events.len()
    );

    // Averaged over the epochs, the noise goes and any bias stays: a wrong Doppler or clock drift sign
    // gives hundreds of m/s, a wrong week or time scale km. What's left is mostly the atmosphere,
    // which we don't correct for (it delays the signals, so mostly up).
    let fix = phone_fix();
    let n = solutions.len() as f64;
    let mean = |f: &dyn Fn(&ride_gnss::solve::Solution, usize) -> f64| -> [f64; 3] {
        std::array::from_fn(|k| solutions.iter().map(|s| f(s, k)).sum::<f64>() / n)
    };
    let [east, north, up] = enu(fix);
    let offset = mean(&|s, k| s.position[k] - fix[k]);
    let horizontal = dot(offset, east).hypot(dot(offset, north));
    let vertical = dot(offset, up);
    let velocity = mean(&|s, k| s.velocity[k]);
    let drift = dot(velocity, velocity).sqrt();
    println!(
        "mean offset {horizontal:.1} m horizontal, {vertical:.1} m up; mean velocity {drift:.3} m/s"
    );
    assert!(
        horizontal < 10.0,
        "mean horizontal offset {horizontal:.1} m"
    );
    assert!(
        vertical.abs() < 25.0,
        "mean vertical offset {vertical:.1} m"
    );
    assert!(drift < 0.05, "mean velocity {drift:.3} m/s");

    // Each epoch alone is noisy (this 2016 phone's pseudoranges scatter by tens of meters).
    let errors: Vec<f64> = solutions
        .iter()
        .map(|s| {
            (0..3)
                .map(|k| (s.position[k] - fix[k]).powi(2))
                .sum::<f64>()
                .sqrt()
        })
        .collect();
    let speeds: Vec<f64> = solutions.iter().map(|s| s.speed()).collect();
    let worst = speeds.iter().copied().fold(0.0, f64::max);
    let (error, speed) = (median(errors), median(speeds));
    println!(
        "median position error {error:.1} m, median speed {speed:.3} m/s, worst speed {worst:.3} m/s"
    );
    assert!(error < 40.0, "median position error {error:.1} m");
    assert!(speed < 0.5, "median speed {speed:.3} m/s");
    assert!(worst < 2.0, "worst speed {worst:.3} m/s");
}

//! Position and velocity from a real phone's raw measurements (Google's GnssLogger demo log, a phone at rest),
//! checked against the phone's own fixes. Unlike `solve.rs`, the measurements don't come from our own model,
//! so a sign or unit mistake shared by the solver and that model shows up here.

mod common;

use common::{dot, enu, raw_epochs};
use ride_gnss::solve::{Navigator, geodetic_to_ecef};

const LOG: &str = include_str!("data/gnsslogger_2016_06_30_21_26_07.txt");

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

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

#[test]
fn real_phone_at_rest() {
    let mut nav = Navigator::default();
    for e in ride_gnss::rinex::read_nav(include_str!("data/hour1820-20h-22h.16n")) {
        nav.add_ephemeris(e);
    }
    let events = raw_epochs(LOG);
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

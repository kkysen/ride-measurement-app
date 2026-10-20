//! A real drive from Google's Smartphone Decimeter Challenge, solved with each set of constellations
//! and checked against its ground truth (a survey-grade receiver's positions and speeds, to centimeters).
//! The data isn't committed (Kaggle's rules): `just test-sdc` downloads it and sets `RIDE_SDC_DRIVE`;
//! without it, this passes without checking anything.

mod common;

use std::collections::BTreeMap;
use std::path::PathBuf;

use common::{dot, enu, raw_epochs};
use ride_gnss::rinex::read_nav;
use ride_gnss::solve::{Navigator, geodetic_to_ecef};
use ride_gnss::system::System;
use ride_session::proto::GnssMeasurementsEvent;

struct Truth {
    ecef: [f64; 3],
    /// East, north (m/s).
    velocity: [f64; 2],
}

/// Ground truth by UTC millisecond.
fn ground_truth(csv: &str) -> BTreeMap<i64, Truth> {
    let mut lines = csv.lines();
    let header: Vec<&str> = lines.next().unwrap().split(',').collect();
    let col = |name: &str| header.iter().position(|h| *h == name).unwrap();
    let (lat, lon, alt, speed, bearing, time) = (
        col("LatitudeDegrees"),
        col("LongitudeDegrees"),
        col("AltitudeMeters"),
        col("SpeedMps"),
        col("BearingDegrees"),
        col("UnixTimeMillis"),
    );
    lines
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let f: Vec<&str> = l.split(',').collect();
            let n = |k: usize| f[k].parse::<f64>().unwrap();
            let (s, b) = (n(speed), n(bearing).to_radians());
            (
                f[time].parse().unwrap(),
                Truth {
                    ecef: geodetic_to_ecef(n(lat), n(lon), n(alt)),
                    velocity: [s * b.sin(), s * b.cos()],
                },
            )
        })
        .collect()
}

fn percentile(mut v: Vec<f64>, p: f64) -> f64 {
    v.sort_by(f64::total_cmp);
    v[((v.len() - 1) as f64 * p).round() as usize]
}

struct Result {
    solved: usize,
    horizontal_median: f64,
    horizontal_p95: f64,
    velocity_median: f64,
    velocity_p95: f64,
    satellites: f64,
}

fn solve(
    systems: &[System],
    orbits: &str,
    events: &[GnssMeasurementsEvent],
    truth: &BTreeMap<i64, Truth>,
) -> Result {
    let mut nav = Navigator::default();
    for e in read_nav(orbits) {
        nav.add_ephemeris(e);
    }
    let (mut horizontal, mut velocity, mut satellites) = (Vec::new(), Vec::new(), 0usize);
    for event in events {
        let mut event = event.clone();
        event.measurements.retain(|m| {
            System::from_android(m.constellation_type).is_some_and(|s| systems.contains(&s))
        });
        let Ok(sol) = nav.on_measurements(&event) else {
            continue;
        };
        let utc_ms = event
            .clock
            .as_ref()
            .unwrap()
            .elapsed_realtime_nanos
            .unwrap()
            / 1_000_000;
        let Some(t) = truth.get(&utc_ms) else {
            continue;
        };
        let [east, north, _] = enu(t.ecef);
        if let Some(p) = &sol.position {
            let d: [f64; 3] = std::array::from_fn(|k| p.position[k] - t.ecef[k]);
            horizontal.push(dot(d, east).hypot(dot(d, north)));
        }
        if let Some(v) = &sol.velocity {
            let en = [dot(v.velocity, east), dot(v.velocity, north)];
            velocity.push((en[0] - t.velocity[0]).hypot(en[1] - t.velocity[1]));
            satellites += v.satellites_used;
        }
    }
    // Epochs with both, at least: position and velocity solve independently.
    let solved = horizontal.len().min(velocity.len());
    let satellites = satellites as f64 / velocity.len() as f64;
    Result {
        solved,
        horizontal_median: percentile(horizontal.clone(), 0.5),
        horizontal_p95: percentile(horizontal, 0.95),
        velocity_median: percentile(velocity.clone(), 0.5),
        velocity_p95: percentile(velocity, 0.95),
        satellites,
    }
}

#[test]
fn drive_with_ground_truth() {
    let Some(dir) = std::env::var_os("RIDE_SDC_DRIVE").map(PathBuf::from) else {
        eprintln!("RIDE_SDC_DRIVE not set (`just test-sdc` sets it); skipping");
        return;
    };
    let read = |name: &str| std::fs::read_to_string(dir.join(name)).unwrap();
    let (log, truth, orbits) = (
        read("gnss_log.txt"),
        ground_truth(&read("ground_truth.csv")),
        read("orbits.rnx"),
    );
    let events = raw_epochs(&log);
    use System::*;
    let sets: [&[System]; 5] = [
        &[Gps],
        &[Gps, Galileo],
        &[Gps, Beidou],
        &[Gps, Glonass],
        &[Gps, Glonass, Qzss, Beidou, Galileo],
    ];
    println!("{} epochs, {} with ground truth", events.len(), truth.len());
    println!(
        "systems                          solved  sats  horizontal m (median, 95%)  velocity km/h (median, 95%)"
    );
    let mut results = Vec::new();
    for set in sets {
        let r = solve(set, &orbits, &events, &truth);
        let names: Vec<&str> = set.iter().map(|s| s.name()).collect();
        println!(
            "{:32} {:6} {:5.1}  {:8.1} {:8.1}               {:8.2} {:8.2}",
            names.join("+"),
            r.solved,
            r.satellites,
            r.horizontal_median,
            r.horizontal_p95,
            r.velocity_median * 3.6,
            r.velocity_p95 * 3.6
        );
        results.push(r);
    }

    // Measured on this drive (GPS alone: 4.3 m and 0.092 m/s, 0.33 km/h, at the median), with some margin.
    let gps = &results[0];
    assert!(gps.solved * 10 >= events.len() * 9);
    assert!(gps.horizontal_median < 5.5 && gps.velocity_median < 0.11);
    for (set, r) in sets.iter().zip(&results).skip(1) {
        // Each constellation adds satellites and makes nothing worse.
        assert!(r.solved >= gps.solved, "{set:?}");
        assert!(
            r.satellites > gps.satellites + 3.0,
            "{set:?}: {} satellites",
            r.satellites
        );
        assert!(
            r.horizontal_median <= gps.horizontal_median + 0.3,
            "{set:?}"
        );
        assert!(r.velocity_median <= gps.velocity_median + 0.005, "{set:?}");
    }
    let all = results.last().unwrap();
    assert!(all.horizontal_median < 4.0 && all.velocity_median < 0.08);
}

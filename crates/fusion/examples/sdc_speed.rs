//! The live speed estimator alone on the SDC drive, fixes from the truth cut out as in `tests/sdc_position.rs`:
//! once a second, the truth's speed and acceleration against the estimator's, to see where its speed goes wrong.
//! `cargo run --release -p ride-fusion --example sdc_speed -- <drive dir>`

use ride_fusion::live_speed::LiveSpeedEstimator;

fn main() {
    let dir = std::path::PathBuf::from(std::env::args().nth(1).expect("drive dir"));
    let csv = std::fs::read_to_string(dir.join("ground_truth.csv")).unwrap();
    let mut lines = csv.lines();
    let header: Vec<&str> = lines.next().unwrap().split(',').collect();
    let col = |name: &str| header.iter().position(|h| *h == name).unwrap();
    let (lat, lon, speed, time) = (
        col("LatitudeDegrees"),
        col("LongitudeDegrees"),
        col("SpeedMps"),
        col("UnixTimeMillis"),
    );
    let truth: Vec<(i64, f64, f64, f64)> = lines
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let f: Vec<&str> = l.split(',').collect();
            let n = |k: usize| f[k].parse::<f64>().unwrap();
            (
                f[time].parse::<i64>().unwrap() * 1_000_000,
                n(lat),
                n(lon),
                n(speed),
            )
        })
        .collect();
    let t0 = truth[0].0;
    let in_outage = |t: i64| {
        let s = (t - t0) / 1_000_000_000;
        s >= 300 && s % 300 < 120
    };
    let imu = std::fs::read_to_string(dir.join("device_imu.csv")).unwrap();
    let mut events: Vec<(i64, bool, [f64; 3])> = imu
        .lines()
        .skip(1)
        .filter_map(|l| {
            let f: Vec<&str> = l.split(',').collect();
            let gyro = match f[0] {
                "UncalAccel" => false,
                "UncalGyro" => true,
                _ => return None,
            };
            let n = |k: usize| f[k].parse::<f64>().unwrap();
            let t = f[1].parse::<i64>().unwrap() * 1_000_000;
            Some((t, gyro, [n(3) - n(6), n(4) - n(7), n(5) - n(8)]))
        })
        .collect();
    events.sort_by_key(|e| e.0);

    let mut est = LiveSpeedEstimator::new();
    let mut fixes = truth.iter().filter(|f| !in_outage(f.0)).peekable();
    let mut next_print = t0;
    let mut prev: Option<(f64, f64)> = None;
    for (t, gyro, v) in events {
        while let Some(f) = fixes.next_if(|f| f.0 <= t) {
            est.on_gnss_fix(f.0, f.1, f.2, Some(5.0), Some(f.3), Some(0.3));
        }
        if gyro {
            est.on_gyro(v[0], v[1], v[2]);
        } else {
            est.on_accel(t, v[0], v[1], v[2]);
        }
        if t >= next_print {
            next_print += 1_000_000_000;
            let i = truth.partition_point(|f| f.0 < t).min(truth.len() - 1);
            let e = est.estimate(t);
            let tv = truth[i].3;
            let (ta, ea) = prev.map_or((0.0, 0.0), |(pt, pe)| (tv - pt, e.speed_mps - pe));
            prev = Some((tv, e.speed_mps));
            println!(
                "{:5} {} truth {tv:5.1} est {:6.1} sigma {:5.1} | accel truth {ta:+5.2} est {ea:+5.2} err {:+5.2}{}",
                (t - t0) / 1_000_000_000,
                if in_outage(t) { "OUT" } else { "   " },
                e.speed_mps,
                e.sigma_mps,
                ea - ta,
                if e.stopped { " stopped" } else { "" }
            );
        }
    }
}

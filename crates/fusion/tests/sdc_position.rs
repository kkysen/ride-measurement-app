//! The position filter on a real drive from Google's Smartphone Decimeter Challenge, with GNSS cut out:
//! the phone's IMU as recorded, and fixes made from the ground truth (a survey-grade receiver) with noise,
//! removed for tunnel-length stretches. Within each, the smoothed position is checked against the truth,
//! for its error and for whether its sigma owns up to it.
//! The data isn't committed (Kaggle's rules): `just test-sdc` downloads it and sets `RIDE_SDC_DRIVE`;
//! without it, this passes without checking anything. `SDC_DEBUG=1` prints every tenth second in the outages.

use std::path::PathBuf;

use ride_fusion::fusion::{Fix, Fusion};

/// Phone-like fixes: the truth plus this much noise per axis (m), and the accuracy they claim (m, 68%).
const FIX_NOISE_M: f64 = 3.0;
const FIX_ACCURACY_M: f64 = 5.0;
/// Fixes are cut out for this long, every this often (s), as on a subway line with stations underground.
const OUTAGE_S: i64 = 120;
const OUTAGE_EVERY_S: i64 = 300;

struct TruthFix {
    t_ns: i64,
    lat: f64,
    lon: f64,
    speed: f64,
}

fn drive() -> Option<PathBuf> {
    std::env::var_os("RIDE_SDC_DRIVE").map(PathBuf::from)
}

fn truth(dir: &std::path::Path) -> Vec<TruthFix> {
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
    lines
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let f: Vec<&str> = l.split(',').collect();
            let n = |k: usize| f[k].parse::<f64>().unwrap();
            TruthFix {
                t_ns: f[time].parse::<i64>().unwrap() * 1_000_000,
                lat: n(lat),
                lon: n(lon),
                speed: n(speed),
            }
        })
        .collect()
}

/// A repeatable standard normal (Box-Muller over a 64-bit LCG), so the test never flakes.
struct Noise(u64);

impl Noise {
    fn uniform(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((self.0 >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    fn normal(&mut self) -> f64 {
        (-2.0 * self.uniform().ln()).sqrt() * (std::f64::consts::TAU * self.uniform()).cos()
    }
}

fn meters(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dn = (a.0 - b.0).to_radians() * 6_371_000.0;
    let de = (a.1 - b.1).to_radians() * 6_371_000.0 * a.0.to_radians().cos();
    dn.hypot(de)
}

#[test]
fn position_through_gnss_outages() {
    let Some(dir) = drive() else {
        eprintln!("RIDE_SDC_DRIVE not set: skipping");
        return;
    };
    let truth = truth(&dir);
    let t0 = truth[0].t_ns;
    let in_outage = |t_ns: i64| {
        let s = (t_ns - t0) / 1_000_000_000;
        // The first stretch has fixes, so the filter starts from them.
        s >= OUTAGE_EVERY_S && s % OUTAGE_EVERY_S < OUTAGE_S
    };

    // The IMU as recorded, by UTC time (this drive has no elapsed realtime for it).
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

    let mut fusion = Fusion::whole_ride();
    let mut noise = Noise(1);
    let mut fixes = truth.iter().filter(|f| !in_outage(f.t_ns)).peekable();
    let deg = 111_320.0;
    for (t, gyro, v) in events {
        while let Some(f) = fixes.next_if(|f| f.t_ns <= t) {
            fusion.on_fix(Fix {
                elapsed_realtime_ns: f.t_ns,
                lat: f.lat + noise.normal() * FIX_NOISE_M / deg,
                lon: f.lon + noise.normal() * FIX_NOISE_M / (deg * f.lat.to_radians().cos()),
                altitude: None,
                accuracy: Some(FIX_ACCURACY_M),
                speed: Some(f.speed + noise.normal() * 0.2),
                speed_sigma: Some(0.3),
            });
        }
        if gyro {
            fusion.on_gyro(t, v);
        } else {
            fusion.on_accel(t, v);
        }
    }
    fusion.finish();
    let positions = fusion.positions();
    assert!(!positions.is_empty());
    let speeds = fusion.smoothed();

    // Each truth point in an outage against the position at the nearest sample.
    let mut errors = Vec::new();
    let mut within_3_sigma = 0;
    for tr in truth.iter().filter(|f| in_outage(f.t_ns)) {
        let i = positions.partition_point(|p| p.elapsed_realtime_ns < tr.t_ns);
        let Some(p) = [i.checked_sub(1), Some(i)]
            .into_iter()
            .flatten()
            .filter_map(|i| positions.get(i))
            .min_by_key(|p| (p.elapsed_realtime_ns - tr.t_ns).abs())
        else {
            continue;
        };
        if (p.elapsed_realtime_ns - tr.t_ns).abs() > 1_000_000_000 {
            continue;
        }
        let e = meters((p.lat, p.lon), (tr.lat, tr.lon));
        if std::env::var_os("SDC_DEBUG").is_some() && (tr.t_ns - t0) / 1_000_000_000 % 10 == 0 {
            let hd = p.velocity[0].atan2(p.velocity[1]).to_degrees();
            let j = speeds
                .partition_point(|s| s.elapsed_realtime_ns < tr.t_ns)
                .min(speeds.len() - 1);
            let s = speeds[j];
            eprintln!(
                "{:5} s truth {:.5},{:.5} {:5.1} m/s | est {:.5},{:.5} {:5.1} m/s hdg {hd:6.1} sigma {:6.1} err {e:6.1} \
                 | speed fwd {:5.1} bwd {:5.1} sigma {:5.1}",
                (tr.t_ns - t0) / 1_000_000_000,
                tr.lat,
                tr.lon,
                tr.speed,
                p.lat,
                p.lon,
                p.velocity[0].hypot(p.velocity[1]),
                p.sigma_m,
                s.forward_mps,
                s.backward_mps,
                s.sigma_mps
            );
        }
        if e <= 3.0 * p.sigma_m {
            within_3_sigma += 1;
        }
        errors.push(e);
    }
    errors.sort_by(f64::total_cmp);
    let n = errors.len();
    assert!(n > 100, "only {n} truth points in outages");
    let (median, p95, max) = (errors[n / 2], errors[n * 95 / 100], errors[n - 1]);
    let honest = within_3_sigma as f64 / n as f64;
    eprintln!(
        "{OUTAGE_S} s outages: {n} truth points, error median {median:.1} m, 95% {p95:.1} m, max {max:.1} m; \
         {:.0}% within 3 sigma",
        honest * 100.0
    );
    // Dead reckoning for up to two minutes at a car's speeds: tens of meters, and owned up to.
    assert!(median < 30.0, "median {median:.1} m");
    assert!(p95 < 100.0, "95% {p95:.1} m");
    assert!(honest > 0.9, "only {:.0}% within 3 sigma", honest * 100.0);
}

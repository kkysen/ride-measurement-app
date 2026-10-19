//! The fusion layer: smoothing through tunnels.

use ride_fusion::fusion::{Fix, Fusion, SmoothedSpeed};

const G: f64 = 9.80665;
const RATE: f64 = 400.0;

/// Forward acceleration (m/s^2): depart, cruise, slow down at 70 s, cruise.
fn profile(t: f64) -> f64 {
    match t {
        t if t < 5.0 => 0.0,
        t if t < 20.0 => 1.0,
        t if (70.0..80.0).contains(&t) => -0.5,
        _ => 0.0,
    }
}

/// A 140 s ride with an uncorrected accelerometer bias along the direction of travel and no GNSS while
/// `in_tunnel`; returns the smoothed speeds and the true speed each second.
fn ride(in_tunnel: impl Fn(f64) -> bool) -> (Vec<SmoothedSpeed>, Vec<(i64, f64)>) {
    let mut fusion = Fusion::new(None);
    let tilt = 30f64.to_radians();
    let bias = 0.05;
    let mut v: f64 = 0.0;
    let mut truth = Vec::new();
    let mut state = 1u64;
    let mut noise = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1u64 << 52) as f64 - 1.0
    };
    for i in 0..(140.0 * RATE) as usize {
        let t = i as f64 / RATE;
        let a = profile(t);
        v = (v + a / RATE).max(0.0);
        let t_ns = (t * 1e9) as i64 + 1;
        let amp = if v > 0.1 { 0.15 } else { 0.005 };
        fusion.on_gyro(t_ns, [0.0; 3]);
        fusion.on_accel(
            t_ns,
            [
                a + bias + amp * noise(),
                G * tilt.sin() + amp * noise(),
                G * tilt.cos() + amp * noise(),
            ],
        );
        if i % RATE as usize == 0 {
            if !in_tunnel(t) {
                fusion.on_fix(Fix {
                    elapsed_realtime_ns: t_ns,
                    lat: 40.0,
                    lon: -74.0,
                    speed: Some(v),
                    speed_sigma: Some(0.2),
                    ..Default::default()
                });
            }
            truth.push((t_ns, v));
        }
    }
    (fusion.smoothed(), truth)
}

/// RMS error (forward-only, smoothed) over the seconds `in_tunnel`.
fn tunnel_errors(in_tunnel: impl Fn(f64) -> bool + Copy) -> (f64, f64) {
    let (smoothed, truth) = ride(in_tunnel);
    let (mut fwd, mut smo, mut n) = (0.0, 0.0, 0.0);
    for &(t_ns, v) in truth.iter().filter(|(t, _)| in_tunnel(*t as f64 * 1e-9)) {
        let s = smoothed
            .iter()
            .find(|s| s.elapsed_realtime_ns == t_ns)
            .unwrap();
        fwd += (s.forward_mps - v).powi(2);
        smo += (s.speed_mps - v).powi(2);
        n += 1.0;
    }
    ((fwd / n).sqrt(), (smo / n).sqrt())
}

#[test]
fn smoothing_fixes_a_ride_that_starts_underground() {
    // No GNSS for the first minute: forward-only drifts with the bias it can't learn yet;
    // the backward pass carries the first fix back through the tunnel.
    let (fwd, smo) = tunnel_errors(|t| t < 60.0);
    assert!(
        smo < 0.5 * fwd,
        "smoothed RMS {smo:.3} m/s vs forward {fwd:.3} m/s"
    );
    assert!(smo < 0.3, "smoothed RMS {smo:.3} m/s");
}

#[test]
fn smoothing_is_no_worse_through_a_mid_ride_tunnel() {
    // Forward-only already learned the bias before the tunnel, so smoothing can only add a little.
    let (fwd, smo) = tunnel_errors(|t| (50.0..110.0).contains(&t));
    assert!(
        smo <= fwd + 0.01,
        "smoothed RMS {smo:.3} m/s vs forward {fwd:.3} m/s"
    );
    assert!(smo < 0.2, "smoothed RMS {smo:.3} m/s");
}

#[test]
fn smoothing_matches_gnss_outside_tunnels() {
    let (smoothed, truth) = ride(|t| (50.0..110.0).contains(&t));
    for &(t_ns, v) in &truth {
        let t = t_ns as f64 * 1e-9;
        if t > 25.0 && !(50.0..115.0).contains(&t) {
            let s = smoothed
                .iter()
                .find(|s| s.elapsed_realtime_ns == t_ns)
                .unwrap();
            assert!(
                (s.speed_mps - v).abs() < 0.3,
                "at {t} s: {} vs {v}",
                s.speed_mps
            );
        }
    }
}

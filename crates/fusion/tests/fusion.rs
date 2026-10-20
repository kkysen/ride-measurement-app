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
/// `in_tunnel`; returns the speeds smoothed all at once and the true speed each second.
fn ride(in_tunnel: impl Fn(f64) -> bool) -> (Vec<SmoothedSpeed>, Vec<(i64, f64)>) {
    ride_in(Fusion::whole_ride(), in_tunnel)
}

/// [`ride`], through `fusion`.
fn ride_in(
    mut fusion: Fusion,
    in_tunnel: impl Fn(f64) -> bool,
) -> (Vec<SmoothedSpeed>, Vec<(i64, f64)>) {
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
    fusion.finish();
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

/// Live smoothing, with a 20 s window and a 60 s tunnel, gives about what smoothing the whole ride at once does:
/// nothing in the tunnel freezes until the fix after it has been carried back through it.
#[test]
fn live_smoothing_matches_whole_ride_through_a_longer_tunnel() {
    let tunnel = |t: f64| (50.0..110.0).contains(&t);
    let (whole, _) = ride(tunnel);
    let (windowed, _) = ride_in(Fusion::with_window(Some(20_000_000_000)), tunnel);
    assert_eq!(
        windowed
            .iter()
            .map(|s| s.elapsed_realtime_ns)
            .collect::<Vec<_>>(),
        whole
            .iter()
            .map(|s| s.elapsed_realtime_ns)
            .collect::<Vec<_>>()
    );
    let worst = whole
        .iter()
        .zip(&windowed)
        .map(|(w, f)| {
            (
                w.elapsed_realtime_ns as f64 * 1e-9,
                (w.speed_mps - f.speed_mps).abs(),
            )
        })
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .unwrap();
    println!("worst difference {:.4} m/s at {:.0} s", worst.1, worst.0);
    assert!(
        worst.1 < 0.01,
        "{:.3} m/s apart at {:.0} s",
        worst.1,
        worst.0
    );
}

enum Event {
    Accel([f64; 3]),
    Gyro,
    Fix(Fix),
}

/// A 60 s ride (depart, cruise) with fixes every second: the events in time order, and the true speed each second.
/// Times (ns) with what happened then, or with the true speed (m/s).
type Timed<T> = Vec<(i64, T)>;

fn ride_events() -> (Timed<Event>, Timed<f64>) {
    let tilt = 30f64.to_radians();
    let (mut v, mut events, mut truth) = (0.0f64, Vec::new(), Vec::new());
    for i in 0..(60.0 * RATE) as usize {
        let t = i as f64 / RATE;
        let a = profile(t);
        v = (v + a / RATE).max(0.0);
        let t_ns = (t * 1e9) as i64 + 1;
        events.push((t_ns, Event::Gyro));
        events.push((t_ns, Event::Accel([a, G * tilt.sin(), G * tilt.cos()])));
        if i % RATE as usize == 0 {
            let fix = Fix {
                elapsed_realtime_ns: t_ns,
                lat: 40.0,
                lon: -74.0,
                speed: Some(v),
                speed_sigma: Some(0.2),
                ..Default::default()
            };
            events.push((t_ns, Event::Fix(fix)));
            truth.push((t_ns, v));
        }
    }
    (events, truth)
}

fn feed(fusion: &mut Fusion, t: i64, e: &Event) {
    match e {
        Event::Accel(a) => fusion.on_accel(t, *a),
        Event::Gyro => fusion.on_gyro(t, [0.0; 3]),
        Event::Fix(f) => fusion.on_fix(*f),
    }
}

#[test]
fn delivery_order_changes_nothing() {
    let (events, _) = ride_events();
    let mut in_order = Fusion::whole_ride();
    for (t, e) in &events {
        feed(&mut in_order, *t, e);
    }
    in_order.finish();

    // As Android delivers them: the IMU in 1 s batches, gyro batches before accel ones, and each fix
    // 0.4 s after its time, so arrival order is far from time order (but within the settle delay).
    let mut arrivals: Vec<(i64, usize)> = events
        .iter()
        .enumerate()
        .map(|(i, (t, e))| {
            let batch_end = (t / 1_000_000_000 + 1) * 1_000_000_000;
            let arrive = match e {
                Event::Gyro => batch_end,
                Event::Accel(_) => batch_end + 1,
                Event::Fix(_) => t + 400_000_000,
            };
            (arrive, i)
        })
        .collect();
    arrivals.sort();
    let mut delivered = Fusion::whole_ride();
    for &(_, i) in &arrivals {
        feed(&mut delivered, events[i].0, &events[i].1);
    }
    delivered.finish();

    assert_eq!(delivered.late_inputs, 0);
    assert_eq!(delivered.smoothed(), in_order.smoothed());
}

#[test]
fn provisional_estimate_tracks_the_newest_inputs() {
    let (events, truth) = ride_events();
    let mut fusion = Fusion::whole_ride();
    let mut checked = 0;
    for (t, e) in &events {
        feed(&mut fusion, *t, e);
        if let Event::Fix(_) = e {
            // During departure, the settled pass is 1.5 s behind (1.5 m/s at 1 m/s^2); the provisional one isn't.
            let true_v = truth.iter().find(|(tt, _)| tt == t).unwrap().1;
            let provisional = fusion.estimate(*t).speed_mps;
            if (8.0..20.0).contains(&(*t as f64 * 1e-9)) {
                assert!(
                    (provisional - true_v).abs() < 0.5,
                    "at {} s: {provisional} vs {true_v}",
                    *t as f64 * 1e-9
                );
                checked += 1;
            }
        }
    }
    assert!(checked >= 10);
    // Once everything settles, the estimate is the settled one.
    let end = events.last().unwrap().0;
    let provisional = fusion.estimate(end);
    fusion.finish();
    assert_eq!(fusion.estimate(end).speed_mps, provisional.speed_mps);
}

#[test]
fn late_inputs_are_counted_not_lost() {
    let (events, _) = ride_events();
    let mut fusion = Fusion::whole_ride();
    for (t, e) in &events {
        if *t > 10_000_000_000 {
            break;
        }
        feed(&mut fusion, *t, e);
    }
    // 5 s after its time: newer inputs have long settled.
    fusion.on_accel(5_000_000_000, [0.0, 0.0, G]);
    assert_eq!(fusion.late_inputs, 1);
}

#[test]
fn average_speed_covers_the_whole_ride() {
    let mut fusion = Fusion::new();
    assert_eq!(fusion.average_speed_mps(), None);
    // 10 m/s for a minute, then a minute standing still.
    for s in 0..=120 {
        let v = if s < 60 { 10.0 } else { 0.0 };
        fusion.on_fix(Fix {
            elapsed_realtime_ns: s * 1_000_000_000 + 1,
            lat: 40.0,
            lon: -74.0,
            speed: Some(v),
            speed_sigma: Some(0.2),
            ..Default::default()
        });
    }
    fusion.finish();
    let average = fusion.average_speed_mps().unwrap();
    assert!((average - 5.0).abs() < 0.5, "average {average} m/s");
}

//! The same scenarios as the app's `LiveSpeedEstimatorTest.kt`.

use std::collections::BTreeSet;
use std::f64::consts::PI;

use ride_fusion::live_speed::{LiveSpeedEstimator, Sensor};

const G: f64 = 9.80665;
const RATE: f64 = 400.0;

/// Deterministic uniform noise in [-1, 1) (xorshift), so tests don't need a `rand` dependency.
struct Noise(u64);

impl Noise {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 52) as f64 - 1.0
    }
}

/// Simulates a phone at a fixed tilt in a vehicle: gravity along a tilted phone axis,
/// forward acceleration along a horizontal axis, plus vibration noise while moving.
/// `profile` gives forward acceleration (m/s^2) at time t (s). Returns (true speed, end time s).
fn simulate(
    est: &mut LiveSpeedEstimator,
    duration_s: f64,
    profile: impl Fn(f64) -> f64,
    forward_sign: f64,
    mut gnss: Option<&mut dyn FnMut(f64) -> Option<f64>>,
) -> (f64, f64) {
    let mut noise = Noise(1);
    // Phone tilted 30 degrees about its x axis: gravity has y and z components; forward is phone x.
    let tilt = 30f64.to_radians();
    let down = [0.0, G * tilt.sin(), G * tilt.cos()];
    let mut v: f64 = 0.0;
    let mut t = 0.0;
    let mut i = 0u64;
    while t < duration_s {
        let a = profile(t);
        v = (v + a / RATE).max(0.0);
        let n = if v > 0.1 { 0.15 } else { 0.005 };
        let t_ns = (t * 1e9) as i64;
        est.on_gyro(0.0, 0.0, 0.0);
        est.on_accel(
            t_ns,
            down[0] + forward_sign * a + n * noise.next(),
            down[1] + n * noise.next(),
            down[2] + n * noise.next(),
        );
        if let Some(gnss) = gnss.as_deref_mut()
            && i.is_multiple_of(RATE as u64)
            && let Some(speed) = gnss(t)
        {
            est.on_gnss_fix(t_ns, 0.0, 0.0, None, Some(speed), Some(0.3));
        }
        i += 1;
        t += 1.0 / RATE;
    }
    (v, t)
}

fn ns(t: f64) -> i64 {
    (t * 1e9) as i64
}

/// Stop 5 s, accelerate 1 m/s^2 for 15 s, cruise 20 s: true speed 15 m/s.
fn depart_and_cruise(t: f64) -> f64 {
    if (5.0..20.0).contains(&t) { 1.0 } else { 0.0 }
}

/// GNSS from integrating `profile`, to know the true speed at t.
fn true_speed(profile: impl Fn(f64) -> f64) -> impl FnMut(f64) -> f64 {
    let mut v = 0.0;
    let mut last = 0.0;
    move |t| {
        while last < t {
            v += profile(last) / RATE;
            last += 1.0 / RATE;
        }
        v
    }
}

fn assert_close(expected: f64, actual: f64, tolerance: f64) {
    assert!(
        (expected - actual).abs() <= tolerance,
        "expected {expected} ± {tolerance}, got {actual}"
    );
}

#[test]
fn imu_only_departure_from_stop() {
    for sign in [1.0, -1.0] {
        let mut est = LiveSpeedEstimator::new();
        let (speed, end) = simulate(&mut est, 40.0, depart_and_cruise, sign, None);
        let e = est.estimate(ns(end));
        assert_eq!(e.sensors, BTreeSet::from([Sensor::Imu]));
        assert_close(speed, e.speed_mps, 0.1 * speed);
    }
}

#[test]
fn stop_resets_to_zero() {
    let mut est = LiveSpeedEstimator::new();
    // Depart, cruise, brake to a stop, then stand still.
    let profile = |t: f64| match t {
        t if t < 5.0 => 0.0,
        t if t < 20.0 => 1.0,
        t if t < 30.0 => 0.0,
        t if t < 45.0 => -1.0,
        _ => 0.0,
    };
    let (_, end) = simulate(&mut est, 55.0, profile, 1.0, None);
    let e = est.estimate(ns(end));
    assert!(e.stopped);
    assert!(e.sensors.contains(&Sensor::Stop));
    assert_close(0.0, e.speed_mps, 0.1);
}

#[test]
fn gnss_used_when_fresh_then_imu_carries_on() {
    let mut est = LiveSpeedEstimator::new();
    // GNSS for the first 30 s (true speed), then lost; keep cruising, then accelerate again.
    let profile = |t: f64| match t {
        t if t < 5.0 => 0.0,
        t if t < 15.0 => 1.0,
        t if t < 40.0 => 0.0,
        t if t < 45.0 => 1.0,
        _ => 0.0,
    };
    let mut truth = true_speed(profile);
    let mut gnss = |t: f64| {
        let v = truth(t);
        (t < 30.0).then_some(v)
    };
    let (speed, end) = simulate(&mut est, 50.0, profile, -1.0, Some(&mut gnss));
    let e = est.estimate(ns(end));
    assert_eq!(e.sensors, BTreeSet::from([Sensor::Imu]));
    assert!(e.since_gnss_s.unwrap() > 15.0);
    assert_close(speed, e.speed_mps, 0.1 * speed);
}

#[test]
fn rotation_of_the_phone_is_tracked_by_the_gyro() {
    let mut est = LiveSpeedEstimator::new();
    let (_, end) = simulate(&mut est, 30.0, depart_and_cruise, 1.0, None);
    let before = est.estimate(ns(end)).speed_mps;
    // Rotate the phone 90 degrees about the (tilted) down axis over 2 s.
    let tilt = 30f64.to_radians();
    let axis = [0.0, tilt.sin(), tilt.cos()];
    let w = PI / 2.0 / 2.0;
    let mut angle = 0.0;
    let mut t = end;
    while angle < PI / 2.0 {
        angle += w / RATE;
        t += 1.0 / RATE;
        est.on_gyro(axis[0] * w, axis[1] * w, axis[2] * w);
        est.on_accel(
            ns(t),
            0.1 * (t * 50.0).sin(),
            G * tilt.sin(),
            G * tilt.cos(),
        );
    }
    let after = est.estimate(ns(t)).speed_mps;
    assert_close(before, after, 0.1 * before);
}

#[test]
fn wrong_gnss_spike_is_rejected() {
    let mut est = LiveSpeedEstimator::new();
    let profile = |t: f64| if (5.0..=15.0).contains(&t) { 1.0 } else { 0.0 };
    let mut truth = true_speed(profile);
    // One wildly wrong fix at 25 s, as from multipath in a station.
    let mut gnss = |t: f64| {
        let v = truth(t);
        Some(if t as i64 == 25 { v + 15.0 } else { v })
    };
    let (speed, end) = simulate(&mut est, 26.5, profile, 1.0, Some(&mut gnss));
    let e = est.estimate(ns(end));
    assert_eq!(e.rejected.get(&Sensor::GnssDoppler), Some(&1));
    assert_close(speed, e.speed_mps, 0.5);
}

#[test]
fn sustained_gnss_disagreement_reinitializes_from_gnss() {
    let mut est = LiveSpeedEstimator::new();
    simulate(
        &mut est,
        20.0,
        |t| if (5.0..=10.0).contains(&t) { 1.0 } else { 0.0 },
        1.0,
        None,
    );
    // The IMU thinks ~5 m/s; GNSS consistently says 20 (e.g. the IMU estimate drifted). GNSS wins.
    for i in 0..8 {
        est.on_gnss_fix(ns(21.0 + i as f64), 0.0, 0.0, None, Some(20.0), Some(0.2));
    }
    assert_close(20.0, est.estimate(29_000_000_000).speed_mps, 1.0);
}

#[test]
fn position_differenced_speed_is_fused() {
    let mut est = LiveSpeedEstimator::new();
    // 10 m/s east along the equator: 1 degree of longitude is ~111.2 km.
    for i in 0..10 {
        let lon = i as f64 * 10.0 / 111_195.0;
        est.on_gnss_fix(ns(i as f64), 0.0, lon, Some(1.0), None, None);
    }
    let e = est.estimate(9_000_000_000);
    assert_close(10.0, e.speed_mps, 0.3);
    assert!(e.sensors.contains(&Sensor::GnssPosition));
}

#[test]
fn gnss_only_follows_changing_speed() {
    // No IMU at all (as in a recording without it): GNSS alone, accelerating 0.5 m/s^2.
    let mut est = LiveSpeedEstimator::new();
    for i in 0..60 {
        est.on_gnss_fix(
            ns(i as f64),
            0.0,
            0.0,
            None,
            Some(0.5 * i as f64),
            Some(0.15),
        );
    }
    let e = est.estimate(ns(59.0));
    assert!(e.rejected.is_empty(), "rejected {:?}", e.rejected);
    assert_close(29.5, e.speed_mps, 0.3);
}

#[test]
fn impossible_speeds_are_rejected_even_first() {
    let mut est = LiveSpeedEstimator::new();
    // A glitched first fix, then a run of them, as could otherwise force a re-initialization.
    est.on_gnss_fix(0, 0.0, 0.0, None, Some(500.0), Some(0.3));
    for i in 1..10 {
        est.on_gnss_fix(
            ns(i as f64),
            0.0,
            0.0,
            None,
            Some(if i < 3 { 10.0 } else { 900.0 }),
            Some(0.3),
        );
    }
    let e = est.estimate(ns(9.0));
    assert_eq!(e.rejected.get(&Sensor::GnssDoppler), Some(&8));
    assert_close(10.0, e.speed_mps, 1.0);
}

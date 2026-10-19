//! Broadcast orbits computed by `ride-gnss` against IGS precise orbits for the same satellites and day.

mod common;
use common::read_rinex_nav;

fn d(s: &str) -> f64 {
    s.trim().replace('D', "E").parse().unwrap()
}

/// SP3 positions (m) by PRN, at 15-minute epochs from 00:00.
fn read_sp3(text: &str) -> std::collections::BTreeMap<u8, Vec<[f64; 3]>> {
    let mut out = std::collections::BTreeMap::<u8, Vec<[f64; 3]>>::new();
    for l in text.lines().filter(|l| l.starts_with("PG")) {
        let prn = l[2..4].parse().unwrap();
        let p = [d(&l[4..18]), d(&l[18..32]), d(&l[32..46])].map(|km| km * 1000.0);
        out.entry(prn).or_default().push(p);
    }
    out
}

/// Derivative at the middle node of the Lagrange polynomial through equally spaced samples.
fn lagrange_derivative_at_middle(y: &[[f64; 3]], h: f64) -> [f64; 3] {
    let n = y.len();
    let m = n / 2;
    let x = |i: usize| (i as f64 - m as f64) * h;
    let mut out = [0.0; 3];
    for (j, yj) in y.iter().enumerate() {
        // d/dx of the j-th basis polynomial, at x = 0 (the middle node).
        let mut dl = 0.0;
        for k in (0..n).filter(|&k| k != j) {
            let mut term = 1.0 / (x(j) - x(k));
            for i in (0..n).filter(|&i| i != j && i != k) {
                term *= (0.0 - x(i)) / (x(j) - x(i));
            }
            dl += term;
        }
        for (o, v) in out.iter_mut().zip(yj) {
            *o += v * dl;
        }
    }
    out
}

fn norm(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

#[test]
fn broadcast_orbits_match_precise_orbits() {
    let ephemerides = read_rinex_nav(include_str!("data/brdc1820.10n"));
    let precise = read_sp3(include_str!("data/igs15904-first-2h.sp3"));
    assert_eq!(ephemerides.len(), 11);
    // 00:00 on Thursday, in GPS seconds of week.
    let t0 = 4.0 * 86_400.0;
    for eph in &ephemerides {
        let sp3 = &precise[&eph.prn];
        assert_eq!(sp3.len(), 9);
        for (k, p) in sp3.iter().enumerate() {
            let s = eph.state_at(t0 + 900.0 * k as f64);
            let err = norm(std::array::from_fn(|c| s.position[c] - p[c]));
            // Broadcast orbits are good to a few meters, and they're for the antenna phase center
            // while SP3 is the center of mass (1 to 2.5 m apart on these satellite blocks); 0.1 to 6.7 m here.
            // A math error would be kilometers off.
            assert!(err < 8.0, "G{:02} at {} min: {err:.2} m", eph.prn, 15 * k);
        }
        // Velocity at 01:00 against the derivative of the precise orbit there.
        let v = eph.state_at(t0 + 3600.0).velocity;
        let v_ref = lagrange_derivative_at_middle(sp3, 900.0);
        let err = norm(std::array::from_fn(|c| v[c] - v_ref[c]));
        assert!(err < 0.01, "G{:02} velocity off by {err:.4} m/s", eph.prn);
        assert!(
            (2_000.0..4_500.0).contains(&norm(v)),
            "G{:02} speed {}",
            eph.prn,
            norm(v)
        );
    }
}

//! GLONASS strings: real broadcast values (from an IGS RINEX file) encoded into strings 1 to 4 as Android delivers
//! them, decoded back, and checked against the originals to within each field's resolution.
//! The encoder writes through the decoder's own field table, so this checks scales, signs, the Moscow-time and
//! four-year-interval dating, the Hamming check, and assembly, but not the table's bit positions:
//! those need a real phone's messages.

use ride_gnss::ephemeris::{Ephemeris, GlonassEphemeris};
use ride_gnss::glonass::{
    ACCELERATION, BN_UNHEALTHY, EphemerisAssembler, Field, GAMMA_N, GlonassError, HAMMING_MASKS,
    N_T, POSITION, SLOT, STRING_BITS, TAU_N, TB, VELOCITY, decode_string, hamming_ok,
};

const ORBITS: &str = include_str!("data/BRDM00DLR_S_20210980000-21h.rnx");
const LEAP_SECONDS: i32 = 18;
const WEEK_S: f64 = 604_800.0;

fn glonass() -> Vec<GlonassEphemeris> {
    ride_gnss::rinex::read_nav(ORBITS)
        .into_iter()
        .filter_map(|e| match e {
            Ephemeris::Glonass(g) => Some(g),
            _ => None,
        })
        .collect()
}

/// Strings 1 to 4, bit by bit.
struct Writer([bool; 4 * STRING_BITS]);

impl Writer {
    fn set(&mut self, start: usize, len: usize, v: u64) {
        for k in 0..len {
            self.0[start + k] = v >> (len - 1 - k) & 1 == 1;
        }
    }

    fn field(&mut self, f: &Field, x: f64) {
        let raw = (x / f.scale).round() as i64;
        let v = if f.signed {
            let magnitude = raw.unsigned_abs();
            assert!(magnitude < 1 << (f.len - 1), "{x} doesn't fit");
            u64::from(raw < 0) << (f.len - 1) | magnitude
        } else {
            raw as u64
        };
        self.set(f.start, f.len, v);
    }

    /// String `k`'s 11 bytes, with its number and Hamming check bits.
    fn string(&self, k: usize) -> Vec<u8> {
        let mut b = [0u8; 11];
        let bits = &self.0[k * STRING_BITS..(k + 1) * STRING_BITS];
        for (i, &bit) in bits.iter().enumerate() {
            b[i / 8] |= u8::from(bit) << (7 - i % 8);
        }
        // The string number.
        b[0] = (b[0] & 0x87) | ((k as u8 + 1) << 3);
        // Check bits: masks 1 to 7 each set the bit 84 - i, the last (overall) mask bit 77.
        let positions: Vec<usize> = (0..7).map(|i| 84 - i).chain([77]).collect();
        for (mask, &pos) in HAMMING_MASKS.iter().zip(&positions) {
            let parity = mask
                .iter()
                .zip(&b)
                .fold(0, |p, (m, d)| p ^ (m & d).count_ones())
                % 2;
            if parity == 1 {
                b[pos / 8] ^= 1 << (7 - pos % 8);
            }
        }
        b.to_vec()
    }
}

/// Strings 1 to 4 carrying `e`, and its Moscow quarter hour and day of the four-year interval.
fn encode(e: &GlonassEphemeris) -> Vec<Vec<u8>> {
    let gps_s = f64::from(e.week) * WEEK_S + e.tb;
    let moscow_s = (gps_s as i64) - i64::from(LEAP_SECONDS) + 3 * 3600;
    let day = moscow_s.div_euclid(86_400);
    let second = moscow_s.rem_euclid(86_400);
    assert_eq!(second % 900, 0, "tb isn't on a quarter hour");
    let n_t = (day - 5839).rem_euclid(1461) + 1;

    let mut w = Writer([false; 4 * STRING_BITS]);
    for k in 0..3 {
        w.field(&POSITION[k], e.position[k] / 1e3);
        w.field(&VELOCITY[k], e.velocity[k] / 1e3);
        w.field(&ACCELERATION[k], e.acceleration[k] / 1e3);
    }
    w.field(&BN_UNHEALTHY, f64::from(u8::from(!e.healthy)));
    w.field(&TB, (second / 900) as f64);
    w.field(&GAMMA_N, e.clock_rate);
    w.field(&TAU_N, -e.clock_bias);
    w.field(&N_T, n_t as f64);
    w.field(&SLOT, f64::from(e.sat.svid));
    (0..4).map(|k| w.string(k)).collect()
}

#[test]
fn round_trips_real_ephemerides() {
    let ephs = glonass();
    assert!(ephs.len() > 10);
    for e in &ephs {
        // The receiver's clock, half a day off, still dates it.
        let rx = f64::from(e.week) * WEEK_S + e.tb + 43_000.0;
        let mut asm = EphemerisAssembler::default();
        let mut out = None;
        for s in encode(e) {
            let s = decode_string(&s).unwrap();
            out = asm.add(e.sat.svid, s, rx, LEAP_SECONDS);
        }
        let d = out.expect("no ephemeris from strings 1 to 4");
        assert_eq!(
            (d.sat, d.week, d.tb, d.healthy),
            (e.sat, e.week, e.tb, e.healthy)
        );
        let check = |name: &str, a: f64, b: f64, resolution: f64| {
            assert!(
                (a - b).abs() <= resolution / 2.0 * 1.000_001,
                "{:?} {name}: {a:e} vs {b:e}",
                e.sat
            );
        };
        let p = |n: i32| 2f64.powi(n);
        check("clock_bias", d.clock_bias, e.clock_bias, p(-30));
        check("clock_rate", d.clock_rate, e.clock_rate, p(-40));
        for k in 0..3 {
            check("position", d.position[k], e.position[k], p(-11) * 1e3);
            check("velocity", d.velocity[k], e.velocity[k], p(-20) * 1e3);
            check(
                "acceleration",
                d.acceleration[k],
                e.acceleration[k],
                p(-30) * 1e3,
            );
        }
        // 15 minutes on, the orbits agree to within what rounding the fields allows.
        let t = e.tb + 900.0;
        let (a, b) = (
            Ephemeris::from(d.clone()).state_at(t),
            Ephemeris::from(e.clone()).state_at(t),
        );
        let dist = (0..3)
            .map(|k| (a.position[k] - b.position[k]).powi(2))
            .sum::<f64>()
            .sqrt();
        assert!(dist < 1.0, "{:?}: {dist} m apart", e.sat);
    }
}

#[test]
fn needs_strings_in_order() {
    let e = &glonass()[0];
    let strings: Vec<_> = encode(e)
        .iter()
        .map(|s| decode_string(s).unwrap())
        .collect();
    let rx = f64::from(e.week) * WEEK_S + e.tb;
    let mut asm = EphemerisAssembler::default();
    for s in [strings[0], strings[1], strings[3]] {
        assert!(asm.add(e.sat.svid, s, rx, LEAP_SECONDS).is_none());
    }
    // A missed string 3: nothing until the next string 1.
    assert!(asm.add(e.sat.svid, strings[2], rx, LEAP_SECONDS).is_none());
    let mut out = None;
    for &s in &strings {
        out = asm.add(e.sat.svid, s, rx, LEAP_SECONDS);
    }
    assert!(out.is_some());
}

#[test]
fn hamming_catches_single_bit_errors() {
    let e = &glonass()[0];
    let s = &encode(e)[0];
    assert!(hamming_ok(s));
    for i in 0..85 {
        let mut b = s.clone();
        b[i / 8] ^= 1 << (7 - i % 8);
        assert_eq!(decode_string(&b), Err(GlonassError::Hamming), "bit {i}");
    }
    assert_eq!(decode_string(&s[..10]), Err(GlonassError::TooShort));
}

/// Through the navigator, as the app feeds it: strings that arrive before any clock wait for one to date them.
#[test]
fn navigator_dates_strings_from_the_first_clock() {
    use ride_gnss::glonass::TYPE_GLO_L1CA;
    use ride_gnss::solve::Navigator;
    use ride_gnss::system::System;
    use ride_session::proto::{GnssClock, GnssMeasurementsEvent, GnssNavigationMessage};

    let e = &glonass()[0];
    let mut nav = Navigator::default();
    for s in encode(e) {
        nav.on_navigation_message(&GnssNavigationMessage {
            svid: i32::from(e.sat.svid),
            r#type: TYPE_GLO_L1CA,
            data: s,
            ..Default::default()
        });
    }
    assert_eq!(nav.stats.decoding[&System::Glonass].ephemerides, 0);
    let rx_ns = ((f64::from(e.week) * WEEK_S + e.tb) * 1e9) as i64;
    let time_nanos = 1_000_000_000_000;
    let _ = nav.on_measurements(&GnssMeasurementsEvent {
        clock: Some(GnssClock {
            time_nanos,
            full_bias_nanos: Some(time_nanos - rx_ns),
            leap_second: Some(LEAP_SECONDS),
            ..Default::default()
        }),
        ..Default::default()
    });
    let d = &nav.stats.decoding[&System::Glonass];
    assert_eq!((d.ok, d.bad, d.ephemerides), (4, 0, 1));
}

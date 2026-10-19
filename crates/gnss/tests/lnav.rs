//! Navigation message decoding against real receiver data, and parity through a round trip.

use ride_gnss::ephemeris::GpsEphemeris;
use ride_gnss::lnav::{EphemerisAssembler, Subframe, decode_subframe, parity};

/// GPS week of 2008-05-26, when the u-blox log was recorded.
const WEEK_2008_05_26: u32 = 1481;

/// Ephemerides from a real u-blox 5 log's `RXM-SFRB` messages (subframes with parity removed).
fn ublox_ephemerides() -> Vec<GpsEphemeris> {
    let bytes = include_bytes!("data/ubx_20080526-sfrb.bin");
    let mut assembler = EphemerisAssembler::default();
    let mut out = Vec::new();
    for msg in bytes.as_chunks::<42>().0 {
        let prn = msg[1];
        let words: [u32; 10] = std::array::from_fn(|i| {
            u32::from_le_bytes(msg[2 + 4 * i..6 + 4 * i].try_into().unwrap())
        });
        let Ok(sf) = Subframe::from_data_words(words) else {
            continue;
        };
        // A reference a few weeks off: only the low 10 bits are broadcast, and the nearest full week is resolved.
        if let Some(eph) = assembler.add(prn, sf, WEEK_2008_05_26 - 3) {
            out.push(eph);
        }
    }
    out
}

#[test]
fn real_subframes_decode_to_physical_gps_orbits() {
    let ephemerides = ublox_ephemerides();
    assert!(
        ephemerides.len() >= 5,
        "only {} ephemerides",
        ephemerides.len()
    );
    for e in &ephemerides {
        // GPS orbits: semi-major axis about 26,560 km, inclination about 55 degrees, nearly circular.
        let a_km = e.sqrt_a * e.sqrt_a / 1000.0;
        assert!(
            (26_400.0..26_700.0).contains(&a_km),
            "G{:02} a = {a_km} km",
            e.prn
        );
        assert!(
            (50.0..60.0).contains(&e.i0.to_degrees()),
            "G{:02} i = {}",
            e.prn,
            e.i0.to_degrees()
        );
        assert!(e.e < 0.03, "G{:02} e = {}", e.prn, e.e);
        assert_eq!(e.week, WEEK_2008_05_26);
        assert_eq!(e.iode, e.iodc as u8);
        assert!(e.af0.abs() < 1e-3, "G{:02} clock offset {}", e.prn, e.af0);
        let r = e.state_at(e.toe).position;
        let r_km = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt() / 1000.0;
        assert!(
            (25_500.0..27_500.0).contains(&r_km),
            "G{:02} at {r_km} km",
            e.prn
        );
    }
}

/// Encodes 240 data bits as Android delivers them: 10 words of 30 bits (data with polarity, then parity)
/// in the low bits of 4 big-endian bytes, choosing words 2 and 10's last two data bits so they end in parity 0, 0.
fn encode_android(sf: &Subframe) -> Vec<u8> {
    let mut out = Vec::new();
    let (mut d29, mut d30) = (0, 0);
    for w in 0..10 {
        let mut d = sf.u(24 * w, 24) as u32;
        if w == 1 || w == 9 {
            d = (0..4)
                .map(|t| (d & !3) | t)
                .find(|&d| parity(d, d29, d30) & 3 == 0)
                .unwrap();
        }
        let p = parity(d, d29, d30);
        let word = ((if d30 == 1 { d ^ 0xff_ffff } else { d }) << 6) | p;
        out.extend_from_slice(&word.to_be_bytes());
        d29 = (p >> 1) & 1;
        d30 = p & 1;
    }
    out
}

#[test]
fn android_layout_round_trips_and_parity_catches_bit_flips() {
    let bytes = include_bytes!("data/ubx_20080526-sfrb.bin");
    let mut checked = 0;
    for msg in bytes.as_chunks::<42>().0 {
        let words: [u32; 10] = std::array::from_fn(|i| {
            u32::from_le_bytes(msg[2 + 4 * i..6 + 4 * i].try_into().unwrap())
        });
        let Ok(sf) = Subframe::from_data_words(words) else {
            continue;
        };
        let android = encode_android(&sf);
        let decoded = decode_subframe(&android).unwrap();
        // Everything but the two bits chosen for parity in words 2 and 10.
        for w in 0..10 {
            let keep = if w == 1 || w == 9 { 22 } else { 24 };
            assert_eq!(decoded.u(24 * w, keep), sf.u(24 * w, keep));
        }
        // Any single flipped bit is caught.
        for bit in [8 * 4 + 7, 8 * 13 + 2, 8 * 39 + 1] {
            let mut bad = android.clone();
            bad[bit / 8] ^= 1 << (7 - bit % 8);
            assert!(decode_subframe(&bad).is_err());
        }
        checked += 1;
    }
    assert!(checked > 100);
}

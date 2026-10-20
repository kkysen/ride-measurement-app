//! BeiDou D1 and D2 subframes: real broadcast values (from an IGS RINEX file) encoded into subframes as Android
//! delivers them, decoded back, and checked against the originals to within each field's resolution.
//! The encoder writes through the decoder's own field table, so this checks scales, signs, times, parity,
//! interleaving, and assembly, but not the table's bit positions: those need a real phone's messages.

use ride_gnss::bds::{
    BdsError, D1, D2, EphemerisAssembler, Field, Layout, SUBFRAME_BITS, Subframe, bch_ok,
    bch_parity, decode_subframe, interleave,
};
use ride_gnss::ephemeris::{Ephemeris, GPS_PI, KeplerEphemeris};
use ride_gnss::system::System;

const ORBITS: &str = include_str!("data/BRDM00DLR_S_20210980000-21h.rnx");

fn beidou() -> Vec<KeplerEphemeris> {
    ride_gnss::rinex::read_nav(ORBITS)
        .into_iter()
        .filter_map(|e| match e {
            Ephemeris::Kepler(k) if k.sat.system == System::Beidou => Some(k),
            _ => None,
        })
        .collect()
}

fn is_geo(svid: u8) -> bool {
    svid <= 5 || svid >= 59
}

/// Subframes' data bits, written field by field over them back to back.
struct Writer {
    words: Vec<[u32; 10]>,
}

impl Writer {
    fn new(n: usize) -> Self {
        Self {
            words: vec![[0; 10]; n],
        }
    }

    fn set(&mut self, start: usize, len: usize, v: u64) {
        for k in 0..len {
            let i = start + k;
            let bit = (v >> (len - 1 - k) & 1) as u32;
            let w = &mut self.words[i / SUBFRAME_BITS][i % SUBFRAME_BITS / 30];
            let shift = 29 - i % 30;
            *w = (*w & !(1 << shift)) | bit << shift;
        }
    }

    fn field(&mut self, f: &Field, x: f64) {
        let raw = (x / f.scale).round() as i64;
        let n = f.bits();
        let mut v = (raw as u64) & ((1u64 << n) - 1);
        let mut left = n;
        for &(start, len) in f.parts {
            left -= len;
            self.set(start, len, v >> left);
            v &= (1u64 << left) - 1;
        }
    }

    /// Preamble, subframe ID, and seconds of week in each subframe, then parity for every word.
    fn finish(mut self, id: impl Fn(usize) -> u64, sow: impl Fn(usize) -> u64) -> Vec<Subframe> {
        for k in 0..self.words.len() {
            let base = k * SUBFRAME_BITS;
            self.set(base, 11, 0b111_0001_0010);
            self.set(base + 15, 3, id(k));
            self.set(base + 18, 8, sow(k) >> 12);
            self.set(base + 30, 12, sow(k) & 0xfff);
        }
        self.words
            .into_iter()
            .map(|mut ws| {
                ws[0] = (ws[0] & !0xf) | bch_parity(ws[0] >> 4 & 0x7ff);
                for w in &mut ws[1..] {
                    let a = *w >> 19 & 0x7ff;
                    let b = *w >> 8 & 0x7ff;
                    *w = a << 19 | b << 8 | bch_parity(a) << 4 | bch_parity(b);
                }
                Subframe { words: ws }
            })
            .collect()
    }
}

fn write_layout(w: &mut Writer, l: &Layout, e: &KeplerEphemeris) {
    let sc = GPS_PI;
    w.field(&l.sat_h1, f64::from(e.sv_health));
    w.field(&l.aodc, f64::from(e.iodc & 0x1f));
    w.field(&l.urai, f64::from(e.sv_accuracy_index & 0xf));
    w.field(&l.week, f64::from(e.week));
    w.field(&l.toc, e.toc);
    w.field(&l.tgd1, e.tgd);
    w.field(&l.a0, e.af0);
    w.field(&l.a1, e.af1);
    w.field(&l.a2, e.af2);
    w.field(&l.aode, f64::from(e.iode & 0x1f));
    w.field(&l.delta_n, e.delta_n / sc);
    w.field(&l.cuc, e.cuc);
    w.field(&l.m0, e.m0 / sc);
    w.field(&l.e, e.e);
    w.field(&l.cus, e.cus);
    w.field(&l.crc, e.crc);
    w.field(&l.crs, e.crs);
    w.field(&l.sqrt_a, e.sqrt_a);
    w.field(&l.toe, e.toe);
    w.field(&l.i0, e.i0 / sc);
    w.field(&l.cic, e.cic);
    w.field(&l.omega_dot, e.omega_dot / sc);
    w.field(&l.cis, e.cis);
    w.field(&l.idot, e.idot / sc);
    w.field(&l.omega0, e.omega0 / sc);
    w.field(&l.omega, e.omega / sc);
}

/// The subframes carrying `e`, as D1 (subframes 1 to 3) or D2 (subframe 1, pages 1 to 10),
/// starting at seconds of week `sow`.
fn encode(e: &KeplerEphemeris, sow: u64) -> Vec<Subframe> {
    if is_geo(e.sat.svid) {
        let mut w = Writer::new(10);
        write_layout(&mut w, &D2, e);
        for p in 0..10 {
            w.set(p * SUBFRAME_BITS + 42, 4, p as u64 + 1);
        }
        w.finish(|_| 1, |k| sow + 3 * k as u64)
    } else {
        let mut w = Writer::new(3);
        write_layout(&mut w, &D1, e);
        w.finish(|k| k as u64 + 1, |k| sow + 6 * k as u64)
    }
}

/// As Android delivers it: each word in the low 30 bits of 4 bytes.
fn android_bytes(sf: &Subframe, interleaved: bool, inverted: bool) -> Vec<u8> {
    sf.words
        .iter()
        .enumerate()
        .flat_map(|(k, &w)| {
            let w = if interleaved && k > 0 {
                interleave(w)
            } else {
                w
            };
            let w = if inverted { w ^ 0x3fff_ffff } else { w };
            w.to_be_bytes()
        })
        .collect()
}

fn decode(e: &KeplerEphemeris, interleaved: bool, inverted: bool) -> KeplerEphemeris {
    let mut asm = EphemerisAssembler::default();
    let sow = (e.toe as u64).saturating_sub(1800);
    let mut out = None;
    for sf in encode(e, sow) {
        let decoded = decode_subframe(
            &android_bytes(&sf, interleaved, inverted),
            &mut Some(interleaved),
        )
        .unwrap();
        assert_eq!(decoded, sf);
        out = if is_geo(e.sat.svid) {
            asm.add_d2(e.sat.svid, decoded)
        } else {
            asm.add_d1(e.sat.svid, decoded)
        };
    }
    out.expect("no ephemeris from a full set")
}

#[test]
fn round_trips_real_ephemerides() {
    let ephs = beidou();
    assert!(
        ephs.iter().any(|e| is_geo(e.sat.svid)),
        "no geostationary satellites in the file"
    );
    assert!(ephs.iter().any(|e| !is_geo(e.sat.svid)));
    for e in &ephs {
        for (interleaved, inverted) in [(false, false), (true, false), (false, true), (true, true)]
        {
            let d = decode(e, interleaved, inverted);
            assert_eq!((d.sat, d.week, d.toe, d.toc), (e.sat, e.week, e.toe, e.toc));
            let check = |name: &str, a: f64, b: f64, resolution: f64| {
                assert!(
                    (a - b).abs() <= resolution / 2.0 * 1.000_001,
                    "{:?} {name}: {a:e} vs {b:e}",
                    e.sat
                );
            };
            let p = |n: i32| 2f64.powi(n);
            let sc = GPS_PI;
            check("af0", d.af0, e.af0, p(-33));
            check("af1", d.af1, e.af1, p(-50));
            check("af2", d.af2, e.af2, p(-66));
            check("tgd", d.tgd, e.tgd, 1e-10);
            check("sqrt_a", d.sqrt_a, e.sqrt_a, p(-19));
            check("e", d.e, e.e, p(-33));
            check("m0", d.m0, e.m0, p(-31) * sc);
            check("omega0", d.omega0, e.omega0, p(-31) * sc);
            check("omega", d.omega, e.omega, p(-31) * sc);
            check("i0", d.i0, e.i0, p(-31) * sc);
            check("omega_dot", d.omega_dot, e.omega_dot, p(-43) * sc);
            check("idot", d.idot, e.idot, p(-43) * sc);
            check("delta_n", d.delta_n, e.delta_n, p(-43) * sc);
            check("cuc", d.cuc, e.cuc, p(-31));
            check("cus", d.cus, e.cus, p(-31));
            check("cic", d.cic, e.cic, p(-31));
            check("cis", d.cis, e.cis, p(-31));
            check("crc", d.crc, e.crc, p(-6));
            check("crs", d.crs, e.crs, p(-6));
            // An hour on, the orbits agree to within what rounding the fields allows.
            let t = e.toe_gps_seconds().rem_euclid(604_800.0) + 3600.0;
            let (a, b) = (
                Ephemeris::from(d.clone()).state_at(t),
                Ephemeris::from(e.clone()).state_at(t),
            );
            let dist = (0..3)
                .map(|k| (a.position[k] - b.position[k]).powi(2))
                .sum::<f64>()
                .sqrt();
            assert!(dist < 0.05, "{:?}: {dist} m apart", e.sat);
        }
    }
}

/// Which way Android delivers words is settled by the first subframe that passes only one way.
#[test]
fn learns_interleaving() {
    let e = beidou().into_iter().find(|e| !is_geo(e.sat.svid)).unwrap();
    for interleaved in [false, true] {
        let mut known = None;
        for sf in encode(&e, 1000) {
            let bytes = android_bytes(&sf, interleaved, false);
            match decode_subframe(&bytes, &mut known) {
                Ok(d) => assert_eq!(d, sf),
                Err(BdsError::Ambiguous) => assert_eq!(known, None),
                Err(other) => panic!("{other:?}"),
            }
        }
        assert_eq!(known, Some(interleaved));
    }
}

#[test]
fn rejects_corrupted_subframes() {
    let e = beidou().into_iter().find(|e| !is_geo(e.sat.svid)).unwrap();
    let sf = &encode(&e, 1000)[0];
    let mut bytes = android_bytes(sf, false, false);
    bytes[13] ^= 0x10;
    assert_eq!(
        decode_subframe(&bytes, &mut None),
        Err(BdsError::Parity { word: 3 })
    );
    let mut bytes = android_bytes(sf, false, false);
    bytes[0] ^= 0x01;
    assert_eq!(decode_subframe(&bytes, &mut None), Err(BdsError::Preamble));
    assert_eq!(
        decode_subframe(&bytes[..38], &mut None),
        Err(BdsError::WrongLength(38))
    );
}

/// BCH(15,11) against the ICD's generator x^4 + x + 1: every single-bit error is caught.
#[test]
fn bch_catches_single_bit_errors() {
    for data in [0, 1, 0x555, 0x7ff, 0x123] {
        let cw = data << 4 | bch_parity(data);
        assert!(bch_ok(cw));
        for b in 0..15 {
            assert!(!bch_ok(cw ^ 1 << b));
        }
    }
}

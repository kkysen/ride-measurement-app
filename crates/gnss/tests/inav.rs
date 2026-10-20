//! Galileo I/NAV pages: real broadcast values (from an IGS RINEX file) encoded into pages as Android delivers
//! them, decoded back, and checked against the originals to within each field's resolution.

use ride_gnss::ephemeris::{Ephemeris, GPS_PI, KeplerEphemeris};
use ride_gnss::inav::{EphemerisAssembler, InavError, crc24q, decode_page};
use ride_gnss::system::System;

const ORBITS: &str = include_str!("data/BRDM00DLR_S_20210980000-21h.rnx");

fn galileo() -> Vec<KeplerEphemeris> {
    ride_gnss::rinex::read_nav(ORBITS)
        .into_iter()
        .filter_map(|e| match e {
            Ephemeris::Kepler(k) if k.sat.system == System::Galileo => Some(k),
            _ => None,
        })
        .collect()
}

/// A 128-bit word built field by field, most significant first.
#[derive(Default)]
struct Bits {
    value: u128,
    len: usize,
}

impl Bits {
    fn push(&mut self, len: usize, v: i64) -> &mut Self {
        self.value = self.value << len | (v as u128 & ((1u128 << len) - 1));
        self.len += len;
        self
    }

    /// `x / scale`, rounded, as a `len`-bit field.
    fn scaled(&mut self, len: usize, x: f64, scale: f64) -> &mut Self {
        self.push(len, (x / scale).round() as i64)
    }

    fn word(&self) -> u128 {
        assert_eq!(self.len, 128);
        self.value
    }
}

fn words(e: &KeplerEphemeris) -> [u128; 5] {
    let p2 = |n: i32| 2f64.powi(n);
    let sc = GPS_PI;
    let iod = i64::from(e.iode);
    [
        Bits::default()
            .push(6, 1)
            .push(10, iod)
            .scaled(14, e.toe, 60.0)
            .scaled(32, e.m0, p2(-31) * sc)
            .scaled(32, e.e, p2(-33))
            .scaled(32, e.sqrt_a, p2(-19))
            .push(2, 0)
            .word(),
        Bits::default()
            .push(6, 2)
            .push(10, iod)
            .scaled(32, e.omega0, p2(-31) * sc)
            .scaled(32, e.i0, p2(-31) * sc)
            .scaled(32, e.omega, p2(-31) * sc)
            .scaled(14, e.idot, p2(-43) * sc)
            .push(2, 0)
            .word(),
        Bits::default()
            .push(6, 3)
            .push(10, iod)
            .scaled(24, e.omega_dot, p2(-43) * sc)
            .scaled(16, e.delta_n, p2(-43) * sc)
            .scaled(16, e.cuc, p2(-29))
            .scaled(16, e.cus, p2(-29))
            .scaled(16, e.crc, p2(-5))
            .scaled(16, e.crs, p2(-5))
            .push(8, 107)
            .word(),
        Bits::default()
            .push(6, 4)
            .push(10, iod)
            .push(6, i64::from(e.sat.svid))
            .scaled(16, e.cic, p2(-29))
            .scaled(16, e.cis, p2(-29))
            .scaled(14, e.toc, 60.0)
            .scaled(31, e.af0, p2(-34))
            .scaled(21, e.af1, p2(-46))
            .scaled(6, e.af2, p2(-59))
            .push(2, 0)
            .word(),
        Bits::default()
            .push(6, 5)
            .push(11 + 11 + 14 + 5, 0)
            .push(10, 0)
            .scaled(10, e.tgd, p2(-32))
            .push(2 + 2 + 1 + 1, 0)
            .push(12, i64::from(e.week) - 1024)
            .push(20, 0)
            .push(23, 0)
            .word(),
    ]
}

/// One page as Android delivers it: even part (0, nominal, word bits 0 to 111), odd part (1, nominal,
/// word bits 112 to 127, 64 other bits, CRC-24Q), 228 bits in 29 bytes.
fn page(word: u128) -> Vec<u8> {
    let mut bits = vec![0u32; 232];
    for i in 0..112 {
        bits[2 + i] = (word >> (127 - i) & 1) as u32;
    }
    bits[114] = 1;
    for i in 0..16 {
        bits[116 + i] = (word >> (15 - i) & 1) as u32;
    }
    for (i, b) in bits[132..196].iter_mut().enumerate() {
        // Other services' bits, here an arbitrary pattern the CRC must cover.
        *b = (i % 3 == 0) as u32;
    }
    let crc = crc24q(196, |i| bits[i]);
    for i in 0..24 {
        bits[196 + i] = crc >> (23 - i) & 1;
    }
    bits.chunks(8)
        .map(|c| c.iter().fold(0u8, |v, &b| v << 1 | b as u8))
        .collect()
}

#[test]
fn crc24q_check_value() {
    let data = b"123456789";
    let crc = crc24q(72, |i| u32::from(data[i / 8] >> (7 - i % 8) & 1));
    assert_eq!(crc, 0xcd_e703);
}

#[test]
fn pages_decode_to_the_broadcast_values() {
    let ephemerides = galileo();
    assert!(ephemerides.len() > 50, "{}", ephemerides.len());
    let p2 = |n: i32| 2f64.powi(n);
    let sc = GPS_PI;
    for e in &ephemerides {
        let mut assembler = EphemerisAssembler::default();
        let decoded = words(e)
            .iter()
            .filter_map(|&w| assembler.add(e.sat.svid, decode_page(&page(w)).unwrap()))
            .last()
            .unwrap_or_else(|| panic!("{}: no ephemeris", e.sat));
        assert_eq!(
            (decoded.sat, decoded.week, decoded.iode),
            (e.sat, e.week, e.iode)
        );
        assert_eq!(decoded.sv_health, 0);
        let close = |name: &str, a: f64, b: f64, lsb: f64| {
            assert!(
                (a - b).abs() <= lsb / 2.0 + 1e-15,
                "{} {name}: {a} vs {b}",
                e.sat
            );
        };
        close("toe", decoded.toe, e.toe, 60.0);
        close("toc", decoded.toc, e.toc, 60.0);
        close("m0", decoded.m0, e.m0, p2(-31) * sc);
        close("e", decoded.e, e.e, p2(-33));
        close("sqrt_a", decoded.sqrt_a, e.sqrt_a, p2(-19));
        close("omega0", decoded.omega0, e.omega0, p2(-31) * sc);
        close("i0", decoded.i0, e.i0, p2(-31) * sc);
        close("omega", decoded.omega, e.omega, p2(-31) * sc);
        close("idot", decoded.idot, e.idot, p2(-43) * sc);
        close("omega_dot", decoded.omega_dot, e.omega_dot, p2(-43) * sc);
        close("delta_n", decoded.delta_n, e.delta_n, p2(-43) * sc);
        close("cuc", decoded.cuc, e.cuc, p2(-29));
        close("crs", decoded.crs, e.crs, p2(-5));
        close("cic", decoded.cic, e.cic, p2(-29));
        close("af0", decoded.af0, e.af0, p2(-34));
        close("af1", decoded.af1, e.af1, p2(-46));
        close("tgd", decoded.tgd, e.tgd, p2(-32));
        // And the satellite ends up where the original puts it, to a few millimeters.
        let (a, b) = (decoded.state_at(e.toe + 600.0), e.state_at(e.toe + 600.0));
        let d: f64 = (0..3)
            .map(|k| (a.position[k] - b.position[k]).powi(2))
            .sum();
        assert!(d.sqrt() < 0.05, "{}: {} m", e.sat, d.sqrt());
    }
}

#[test]
fn damaged_and_alert_pages_are_rejected() {
    let e = &galileo()[0];
    let good = page(words(e)[0]);
    for bit in [3, 60, 113, 120, 150, 200] {
        let mut bad = good.clone();
        bad[bit / 8] ^= 1 << (7 - bit % 8);
        assert_eq!(decode_page(&bad), Err(InavError::Crc), "bit {bit}");
    }
    let mut alert = good.clone();
    alert[0] |= 0x40;
    assert_eq!(decode_page(&alert), Err(InavError::NotNominal));
    assert_eq!(decode_page(&good[..20]), Err(InavError::TooShort));
}

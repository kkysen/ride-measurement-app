//! BeiDou D1 (medium and inclined geosynchronous orbits) and D2 (geostationary) navigation messages
//! as Android delivers them, decoded into ephemerides (BeiDou SIS ICD open service 2.1, section 5).
//!
//! `GnssNavigationMessage.getData()` for `TYPE_BDS_D1` and `TYPE_BDS_D2`: one subframe of 10 words of 30 bits,
//! each in the low 30 bits of 4 bytes, most significant bit first: 40 bytes, as for GPS.
//! Word 1 is 15 plain bits, then 11 data bits and 4 parity bits (BCH(15,11)); words 2 to 10 are two
//! such codewords, which the satellite interleaves bit by bit. Android doesn't say whether it delivers them
//! interleaved, so each subframe is checked both ways and taken in whichever passes. A sparse subframe
//! can pass both ways; it's taken in the way an earlier subframe settled, or dropped until one has.
//!
//! Field positions count over the de-interleaved subframe (each word 22 data bits, then 8 parity bits),
//! with D1's subframes 1 to 3 or D2's subframe 1 pages 1 to 10 back to back, 300 bits each.
//! They follow the ICD's figures 5-5 to 5-7 and 5-16 to 5-25, cross-checked against RTKLIB's `rcvraw.c`.

use std::collections::BTreeMap;

use crate::ephemeris::{GPS_PI, KeplerEphemeris};
use crate::system::{SatId, System};

/// `GnssNavigationMessage.TYPE_BDS_D1`, `TYPE_BDS_D2`.
pub const TYPE_BDS_D1: i32 = 0x0501;
pub const TYPE_BDS_D2: i32 = 0x0502;

const PREAMBLE: u32 = 0b111_0001_0010;
const WORD_MASK: u32 = 0x3fff_ffff;
pub const SUBFRAME_BITS: usize = 300;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BdsError {
    WrongLength(usize),
    Preamble,
    Parity {
        word: usize,
    },
    /// It passes both interleaved and not, and no subframe has settled which Android delivers.
    Ambiguous,
}

/// One subframe's 10 words, de-interleaved and with polarity corrected.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Subframe {
    pub words: [u32; 10],
}

impl Subframe {
    /// `len` bits from bit `start` of the subframe, most significant first.
    pub fn u(&self, start: usize, len: usize) -> u64 {
        bits(std::slice::from_ref(self), start, len)
    }

    /// Subframe ID (FraID): 1 to 5.
    pub fn id(&self) -> u8 {
        self.u(15, 3) as u8
    }

    /// Seconds of BeiDou week at the start of the subframe.
    pub fn sow(&self) -> u32 {
        (self.u(18, 8) << 12 | self.u(30, 12)) as u32
    }

    /// D2 subframe 1's page number: 1 to 10.
    pub fn page(&self) -> u8 {
        self.u(42, 4) as u8
    }
}

/// `len` bits from bit `start` of `subframes` back to back.
fn bits(subframes: &[Subframe], start: usize, len: usize) -> u64 {
    (start..start + len).fold(0, |v, i| {
        let w = subframes[i / SUBFRAME_BITS].words[i % SUBFRAME_BITS / 30];
        v << 1 | u64::from(w >> (29 - i % 30) & 1)
    })
}

/// Whether a 15-bit BCH(15,11) codeword (11 data bits, then 4 parity) is divisible by x^4 + x + 1.
pub fn bch_ok(codeword: u32) -> bool {
    let mut r = codeword & 0x7fff;
    for i in (4..15).rev() {
        if r >> i & 1 == 1 {
            r ^= 0b10011 << (i - 4);
        }
    }
    r == 0
}

/// The parity bits of 11 data bits.
pub fn bch_parity(data: u32) -> u32 {
    let mut r = (data & 0x7ff) << 4;
    for i in (4..15).rev() {
        if r >> i & 1 == 1 {
            r ^= 0b10011 << (i - 4);
        }
    }
    r
}

/// A word in transmitted order (codewords' bits alternating, first codeword first) to de-interleaved order.
pub fn deinterleave(w: u32) -> u32 {
    let (mut a, mut b) = (0, 0);
    for j in 0..15 {
        a = a << 1 | (w >> (29 - 2 * j) & 1);
        b = b << 1 | (w >> (28 - 2 * j) & 1);
    }
    join(a, b)
}

/// The inverse of [`deinterleave`].
pub fn interleave(w: u32) -> u32 {
    let (a, b) = split(w);
    (0..15).fold(0, |v, j| {
        v << 2 | (a >> (14 - j) & 1) << 1 | (b >> (14 - j) & 1)
    })
}

/// A de-interleaved word's two codewords.
fn split(w: u32) -> (u32, u32) {
    let a = (w >> 19 & 0x7ff) << 4 | (w >> 4 & 0xf);
    let b = (w >> 8 & 0x7ff) << 4 | (w & 0xf);
    (a, b)
}

fn join(a: u32, b: u32) -> u32 {
    (a >> 4) << 19 | (b >> 4) << 8 | (a & 0xf) << 4 | (b & 0xf)
}

fn word_ok(w: u32) -> bool {
    let (a, b) = split(w);
    bch_ok(a) && bch_ok(b)
}

/// Checks and unpacks one subframe; `interleaved` is whether Android delivers words 2 to 10 interleaved,
/// if known, and is set once a subframe passes only one way.
pub fn decode_subframe(data: &[u8], interleaved: &mut Option<bool>) -> Result<Subframe, BdsError> {
    if data.len() != 40 {
        return Err(BdsError::WrongLength(data.len()));
    }
    let mut words: [u32; 10] = std::array::from_fn(|k| {
        u32::from_be_bytes([
            data[4 * k],
            data[4 * k + 1],
            data[4 * k + 2],
            data[4 * k + 3],
        ]) & WORD_MASK
    });
    match words[0] >> 19 {
        PREAMBLE => {}
        p if p == !PREAMBLE & 0x7ff => words.iter_mut().for_each(|w| *w ^= WORD_MASK),
        _ => return Err(BdsError::Preamble),
    }
    if !bch_ok(words[0] & 0x7fff) {
        return Err(BdsError::Parity { word: 0 });
    }
    let as_is = (1..10).find(|&k| !word_ok(words[k]));
    let deinterleaved = words.map(deinterleave);
    let after = (1..10).find(|&k| !word_ok(deinterleaved[k]));
    let use_deinterleaved = match (as_is, after) {
        (None, None) if words[1..] == deinterleaved[1..] => false,
        (None, None) => interleaved.ok_or(BdsError::Ambiguous)?,
        (None, Some(_)) => false,
        (Some(_), None) => true,
        (Some(k), Some(_)) => return Err(BdsError::Parity { word: k }),
    };
    if as_is.is_none() != after.is_none() {
        *interleaved = Some(use_deinterleaved);
    }
    if use_deinterleaved {
        words[1..].copy_from_slice(&deinterleaved[1..]);
    }
    Ok(Subframe { words })
}

/// One field: its bit ranges (most significant first) over the subframes back to back,
/// whether it's two's complement, and its scale.
pub struct Field {
    pub parts: &'static [(usize, usize)],
    pub signed: bool,
    pub scale: f64,
}

impl Field {
    pub fn bits(&self) -> usize {
        self.parts.iter().map(|p| p.1).sum()
    }

    pub fn raw(&self, subframes: &[Subframe]) -> i64 {
        let v = self.parts.iter().fold(0u64, |v, &(start, len)| {
            v << len | bits(subframes, start, len)
        });
        let n = self.bits();
        if self.signed && v >> (n - 1) & 1 == 1 {
            v as i64 - (1i64 << n)
        } else {
            v as i64
        }
    }

    pub fn value(&self, subframes: &[Subframe]) -> f64 {
        self.raw(subframes) as f64 * self.scale
    }
}

const fn u(parts: &'static [(usize, usize)], scale: f64) -> Field {
    Field {
        parts,
        signed: false,
        scale,
    }
}

const fn s(parts: &'static [(usize, usize)], scale: f64) -> Field {
    Field {
        parts,
        signed: true,
        scale,
    }
}

const P2_6: f64 = 1.0 / 64.0;
const P2_19: f64 = 1.0 / 524_288.0;
const P2_31: f64 = 1.0 / 2_147_483_648.0;
const P2_33: f64 = P2_31 / 4.0;
const P2_43: f64 = P2_33 / 1024.0;
const P2_50: f64 = P2_43 / 128.0;
const P2_66: f64 = P2_50 / 65_536.0;
const NS_TENTH: f64 = 1e-10;

/// An ephemeris's fields, wherever its message puts them. Angles are in semicircles.
pub struct Layout {
    pub sat_h1: Field,
    pub aodc: Field,
    pub urai: Field,
    pub week: Field,
    pub toc: Field,
    pub tgd1: Field,
    pub a0: Field,
    pub a1: Field,
    pub a2: Field,
    pub aode: Field,
    pub delta_n: Field,
    pub cuc: Field,
    pub m0: Field,
    pub e: Field,
    pub cus: Field,
    pub crc: Field,
    pub crs: Field,
    pub sqrt_a: Field,
    pub toe: Field,
    pub i0: Field,
    pub cic: Field,
    pub omega_dot: Field,
    pub cis: Field,
    pub idot: Field,
    pub omega0: Field,
    pub omega: Field,
}

/// D1: subframes 1, 2, 3.
pub const D1: Layout = {
    const S2: usize = SUBFRAME_BITS;
    const S3: usize = 2 * SUBFRAME_BITS;
    Layout {
        sat_h1: u(&[(42, 1)], 1.0),
        aodc: u(&[(43, 5)], 1.0),
        urai: u(&[(48, 4)], 1.0),
        week: u(&[(60, 13)], 1.0),
        toc: u(&[(73, 9), (90, 8)], 8.0),
        tgd1: s(&[(98, 10)], NS_TENTH),
        a2: s(&[(214, 11)], P2_66),
        a0: s(&[(225, 7), (240, 17)], P2_33),
        a1: s(&[(257, 5), (270, 17)], P2_50),
        aode: u(&[(287, 5)], 1.0),
        delta_n: s(&[(S2 + 42, 10), (S2 + 60, 6)], P2_43),
        cuc: s(&[(S2 + 66, 16), (S2 + 90, 2)], P2_31),
        m0: s(&[(S2 + 92, 20), (S2 + 120, 12)], P2_31),
        e: u(&[(S2 + 132, 10), (S2 + 150, 22)], P2_33),
        cus: s(&[(S2 + 180, 18)], P2_31),
        crc: s(&[(S2 + 198, 4), (S2 + 210, 14)], P2_6),
        crs: s(&[(S2 + 224, 8), (S2 + 240, 10)], P2_6),
        sqrt_a: u(&[(S2 + 250, 12), (S2 + 270, 20)], P2_19),
        toe: u(&[(S2 + 290, 2), (S3 + 42, 10), (S3 + 60, 5)], 8.0),
        i0: s(&[(S3 + 65, 17), (S3 + 90, 15)], P2_31),
        cic: s(&[(S3 + 105, 7), (S3 + 120, 11)], P2_31),
        omega_dot: s(&[(S3 + 131, 11), (S3 + 150, 13)], P2_43),
        cis: s(&[(S3 + 163, 9), (S3 + 180, 9)], P2_31),
        idot: s(&[(S3 + 189, 13), (S3 + 210, 1)], P2_43),
        omega0: s(&[(S3 + 211, 21), (S3 + 240, 11)], P2_31),
        omega: s(&[(S3 + 251, 11), (S3 + 270, 21)], P2_31),
    }
};

/// D2: subframe 1, pages 1 to 10 (page 2 has none of it).
pub const D2: Layout = {
    const P: usize = SUBFRAME_BITS;
    Layout {
        sat_h1: u(&[(46, 1)], 1.0),
        aodc: u(&[(47, 5)], 1.0),
        urai: u(&[(60, 4)], 1.0),
        week: u(&[(64, 13)], 1.0),
        toc: u(&[(77, 5), (90, 12)], 8.0),
        tgd1: s(&[(102, 10)], NS_TENTH),
        a0: s(&[(2 * P + 100, 12), (2 * P + 120, 12)], P2_33),
        a1: s(
            &[(2 * P + 132, 4), (3 * P + 46, 6), (3 * P + 60, 12)],
            P2_50,
        ),
        a2: s(&[(3 * P + 72, 10), (3 * P + 90, 1)], P2_66),
        aode: u(&[(3 * P + 91, 5)], 1.0),
        delta_n: s(&[(3 * P + 96, 16)], P2_43),
        cuc: s(&[(3 * P + 120, 14), (4 * P + 46, 4)], P2_31),
        m0: s(&[(4 * P + 50, 2), (4 * P + 60, 22), (4 * P + 90, 8)], P2_31),
        cus: s(&[(4 * P + 98, 14), (4 * P + 120, 4)], P2_31),
        e: u(
            &[(4 * P + 124, 10), (5 * P + 46, 6), (5 * P + 60, 16)],
            P2_33,
        ),
        sqrt_a: u(
            &[(5 * P + 76, 6), (5 * P + 90, 22), (5 * P + 120, 4)],
            P2_19,
        ),
        cic: s(
            &[(5 * P + 124, 10), (6 * P + 46, 6), (6 * P + 60, 2)],
            P2_31,
        ),
        cis: s(&[(6 * P + 62, 18)], P2_31),
        toe: u(&[(6 * P + 80, 2), (6 * P + 90, 15)], 8.0),
        i0: s(
            &[
                (6 * P + 105, 7),
                (6 * P + 120, 14),
                (7 * P + 46, 6),
                (7 * P + 60, 5),
            ],
            P2_31,
        ),
        crc: s(&[(7 * P + 65, 17), (7 * P + 90, 1)], P2_6),
        crs: s(&[(7 * P + 91, 18)], P2_6),
        omega_dot: s(
            &[(7 * P + 109, 3), (7 * P + 120, 16), (8 * P + 46, 5)],
            P2_43,
        ),
        omega0: s(&[(8 * P + 51, 1), (8 * P + 60, 22), (8 * P + 90, 9)], P2_31),
        omega: s(
            &[(8 * P + 99, 13), (8 * P + 120, 14), (9 * P + 46, 5)],
            P2_31,
        ),
        idot: s(&[(9 * P + 51, 1), (9 * P + 60, 13)], P2_43),
    }
};

/// Collects D1 subframes 1 to 3 or D2 subframe 1's pages per satellite, and yields an ephemeris
/// once a consecutive set agrees on its reference time.
#[derive(Default)]
pub struct EphemerisAssembler {
    d1: BTreeMap<u8, [Option<Subframe>; 3]>,
    d2: BTreeMap<u8, [Option<Subframe>; 10]>,
}

impl EphemerisAssembler {
    pub fn add_d1(&mut self, svid: u8, sf: Subframe) -> Option<KeplerEphemeris> {
        let id = sf.id();
        if !(1..=3).contains(&id) {
            return None;
        }
        let set = self.d1.entry(svid).or_default();
        set[usize::from(id - 1)] = Some(sf);
        let [Some(a), Some(b), Some(c)] = set else {
            return None;
        };
        // One frame: subframes 6 s apart.
        if b.sow() != a.sow() + 6 || c.sow() != b.sow() + 6 {
            return None;
        }
        let sfs = [a.clone(), b.clone(), c.clone()];
        set.fill(None);
        ephemeris(svid, &D1, &sfs)
    }

    pub fn add_d2(&mut self, svid: u8, sf: Subframe) -> Option<KeplerEphemeris> {
        let page = sf.page();
        if sf.id() != 1 || !(1..=10).contains(&page) {
            return None;
        }
        let set = self.d2.entry(svid).or_default();
        set[usize::from(page - 1)] = Some(sf);
        if set.iter().any(Option::is_none) {
            return None;
        }
        let sfs: Vec<Subframe> = set.iter().flatten().cloned().collect();
        // Consecutive pages, 3 s apart.
        if sfs.windows(2).any(|w| w[1].sow() != w[0].sow() + 3) {
            return None;
        }
        set.iter_mut().for_each(|p| *p = None);
        ephemeris(svid, &D2, &sfs)
    }
}

fn ephemeris(svid: u8, l: &Layout, sfs: &[Subframe]) -> Option<KeplerEphemeris> {
    let v = |f: &Field| f.value(sfs);
    let toe = v(&l.toe);
    let toc = v(&l.toc);
    // Clock and orbit from different issues, mid-update.
    if toe != toc {
        return None;
    }
    // The week is the first subframe's: the reference time may be in the one before or after.
    let sow = f64::from(sfs[0].sow());
    let mut week = l.week.raw(sfs);
    if toe - sow > 302_400.0 {
        week -= 1;
    } else if sow - toe > 302_400.0 {
        week += 1;
    }
    let sc = GPS_PI;
    Some(KeplerEphemeris {
        sat: SatId {
            system: System::Beidou,
            svid,
        },
        week: u32::try_from(week).ok()?,
        sv_accuracy_index: l.urai.raw(sfs) as u8,
        sv_health: l.sat_h1.raw(sfs) as u8,
        iodc: l.aodc.raw(sfs) as u16,
        iode: l.aode.raw(sfs) as u16,
        tgd: v(&l.tgd1),
        toc,
        af0: v(&l.a0),
        af1: v(&l.a1),
        af2: v(&l.a2),
        crs: v(&l.crs),
        delta_n: v(&l.delta_n) * sc,
        m0: v(&l.m0) * sc,
        cuc: v(&l.cuc),
        e: v(&l.e),
        cus: v(&l.cus),
        sqrt_a: v(&l.sqrt_a),
        toe,
        cic: v(&l.cic),
        omega0: v(&l.omega0) * sc,
        cis: v(&l.cis),
        i0: v(&l.i0) * sc,
        crc: v(&l.crc),
        omega: v(&l.omega) * sc,
        omega_dot: v(&l.omega_dot) * sc,
        idot: v(&l.idot) * sc,
    })
}

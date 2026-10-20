//! GPS L1 C/A navigation messages (LNAV) as Android delivers them, decoded into ephemerides.
//!
//! `GnssNavigationMessage.getData()` for GPS L1 C/A: one subframe of 10 words of 30 bits,
//! each in the low 30 bits of 4 bytes, most significant bit first: 40 bytes.
//! Bits are as transmitted, so each word's data bits are inverted when the previous word's last parity bit is 1,
//! and every word is checked against its 6 parity bits (IS-GPS-200 section 20.3.5).

use std::collections::BTreeMap;

use crate::ephemeris::{GPS_PI, KeplerEphemeris};
use crate::system::SatId;

/// `GnssNavigationMessage.TYPE_GPS_L1CA`.
pub const TYPE_GPS_L1CA: i32 = 0x0101;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LnavError {
    WrongLength(usize),
    Parity { word: usize },
    Preamble,
}

impl std::fmt::Display for LnavError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LnavError::WrongLength(n) => write!(f, "{n} bytes, expected 40"),
            LnavError::Parity { word } => write!(f, "parity error in word {}", word + 1),
            LnavError::Preamble => write!(f, "no preamble"),
        }
    }
}

/// One subframe's 240 data bits (10 words of 24, parity removed and polarity corrected).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Subframe {
    bits: [u8; 30],
}

impl Subframe {
    /// From 10 words of 24 data bits each (low bits), parity already checked and removed, as some receivers deliver.
    pub fn from_data_words(words: [u32; 10]) -> Result<Self, LnavError> {
        let mut bits = [0u8; 30];
        for (w, d) in words.iter().enumerate() {
            for b in 0..3 {
                bits[3 * w + b] = (d >> (16 - 8 * b)) as u8;
            }
        }
        let sf = Subframe { bits };
        if sf.u(0, 8) != 0x8b {
            return Err(LnavError::Preamble);
        }
        Ok(sf)
    }

    /// Unsigned field of `len` bits starting at data bit `start` (0-based).
    pub fn u(&self, start: usize, len: usize) -> u64 {
        (start..start + len).fold(0, |v, i| {
            (v << 1) | u64::from((self.bits[i / 8] >> (7 - i % 8)) & 1)
        })
    }

    /// Two's-complement signed field.
    pub fn s(&self, start: usize, len: usize) -> i64 {
        let v = self.u(start, len) as i64;
        if v >> (len - 1) & 1 == 1 {
            v - (1 << len)
        } else {
            v
        }
    }

    /// Subframe ID from the handover word: 1 to 5.
    pub fn id(&self) -> u8 {
        self.u(43, 3) as u8
    }

    /// Time of week (s) of the start of the next subframe, from the handover word.
    pub fn tow(&self) -> f64 {
        self.u(24, 17) as f64 * 6.0
    }
}

/// Bits of `d1..d24` (as bit 23 = d1 .. bit 0 = d24) that feed each of the 6 parity bits.
const PARITY_MASKS: [[u8; 15]; 6] = [
    [1, 2, 3, 5, 6, 10, 11, 12, 13, 14, 17, 18, 20, 23, 0],
    [2, 3, 4, 6, 7, 11, 12, 13, 14, 15, 18, 19, 21, 24, 0],
    [1, 3, 4, 5, 7, 8, 12, 13, 14, 15, 16, 19, 20, 22, 0],
    [2, 4, 5, 6, 8, 9, 13, 14, 15, 16, 17, 20, 21, 23, 0],
    [1, 3, 5, 6, 7, 9, 10, 14, 15, 16, 17, 18, 21, 22, 24],
    [3, 5, 6, 8, 9, 10, 11, 13, 15, 19, 22, 23, 24, 0, 0],
];

/// The 6 parity bits for 24 source data bits, given the previous word's last two parity bits.
pub fn parity(data: u32, d29_prev: u32, d30_prev: u32) -> u32 {
    let bit = |i: u8| (data >> (24 - i)) & 1;
    let mut p = 0;
    for (k, mask) in PARITY_MASKS.iter().enumerate() {
        let mut v = if matches!(k, 0 | 2 | 5) {
            d29_prev
        } else {
            d30_prev
        };
        for &i in mask.iter().filter(|&&i| i != 0) {
            v ^= bit(i);
        }
        p = (p << 1) | v;
    }
    p
}

/// Checks parity and removes it, for one subframe of Android's 40 bytes.
pub fn decode_subframe(data: &[u8]) -> Result<Subframe, LnavError> {
    if data.len() != 40 {
        return Err(LnavError::WrongLength(data.len()));
    }
    let mut bits = [0u8; 30];
    // The word before the TLM word (the previous subframe's word 10) always ends in parity bits 0, 0.
    let (mut d29, mut d30) = (0, 0);
    for w in 0..10 {
        let word = u32::from_be_bytes(data[4 * w..4 * w + 4].try_into().unwrap()) & 0x3fff_ffff;
        let mut d = word >> 6;
        if d30 == 1 {
            d ^= 0x00ff_ffff;
        }
        if parity(d, d29, d30) != word & 0x3f {
            return Err(LnavError::Parity { word: w });
        }
        for b in 0..3 {
            bits[3 * w + b] = (d >> (16 - 8 * b)) as u8;
        }
        d29 = (word >> 1) & 1;
        d30 = word & 1;
    }
    let sf = Subframe { bits };
    if sf.u(0, 8) != 0x8b {
        return Err(LnavError::Preamble);
    }
    Ok(sf)
}

/// Collects subframes 1 to 3 per satellite and yields an ephemeris once a matching set is complete.
#[derive(Default)]
pub struct EphemerisAssembler {
    partial: BTreeMap<SatId, [Option<Subframe>; 3]>,
}

impl EphemerisAssembler {
    /// Adds a decoded subframe for `sat` (GPS or QZSS); `reference_week` is a full GPS week near the subframe's,
    /// to resolve the 10-bit week. Returns a new ephemeris when subframes 1 to 3 agree on their issue of data.
    pub fn add(
        &mut self,
        sat: SatId,
        sf: Subframe,
        reference_week: u32,
    ) -> Option<KeplerEphemeris> {
        let id = sf.id();
        if !(1..=3).contains(&id) {
            return None;
        }
        let set = self.partial.entry(sat).or_default();
        set[usize::from(id - 1)] = Some(sf);
        let [Some(sf1), Some(sf2), Some(sf3)] = set else {
            return None;
        };
        let iodc = (sf1.u(70, 2) << 8 | sf1.u(168, 8)) as u16;
        let (iode2, iode3) = (sf2.u(48, 8) as u8, sf3.u(216, 8) as u8);
        if iode2 != iode3 || iode2 != iodc as u8 {
            // Mid-update: wait for the rest of the new set.
            return None;
        }
        let eph = ephemeris(sat, sf1, sf2, sf3, reference_week);
        self.partial.remove(&sat);
        Some(eph)
    }
}

/// Field layout and scale factors from IS-GPS-200 tables 20-I and 20-III, as offsets into the 240 data bits.
fn ephemeris(
    sat: SatId,
    sf1: &Subframe,
    sf2: &Subframe,
    sf3: &Subframe,
    reference_week: u32,
) -> KeplerEphemeris {
    let p2 = |n: i32| 2f64.powi(n);
    let sc = GPS_PI;
    let week10 = sf1.u(48, 10) as u32;
    // The full week closest to the reference with these low 10 bits.
    let week = reference_week - ((reference_week + 1024 - week10 + 512) % 1024) + 512;
    KeplerEphemeris {
        sat,
        week,
        sv_accuracy_index: sf1.u(60, 4) as u8,
        sv_health: sf1.u(64, 6) as u8,
        iodc: (sf1.u(70, 2) << 8 | sf1.u(168, 8)) as u16,
        tgd: sf1.s(160, 8) as f64 * p2(-31),
        toc: sf1.u(176, 16) as f64 * 16.0,
        af2: sf1.s(192, 8) as f64 * p2(-55),
        af1: sf1.s(200, 16) as f64 * p2(-43),
        af0: sf1.s(216, 22) as f64 * p2(-31),
        iode: sf2.u(48, 8) as u16,
        crs: sf2.s(56, 16) as f64 * p2(-5),
        delta_n: sf2.s(72, 16) as f64 * p2(-43) * sc,
        m0: sf2.s(88, 32) as f64 * p2(-31) * sc,
        cuc: sf2.s(120, 16) as f64 * p2(-29),
        e: sf2.u(136, 32) as f64 * p2(-33),
        cus: sf2.s(168, 16) as f64 * p2(-29),
        sqrt_a: sf2.u(184, 32) as f64 * p2(-19),
        toe: sf2.u(216, 16) as f64 * 16.0,
        cic: sf3.s(48, 16) as f64 * p2(-29),
        omega0: sf3.s(64, 32) as f64 * p2(-31) * sc,
        cis: sf3.s(96, 16) as f64 * p2(-29),
        i0: sf3.s(112, 32) as f64 * p2(-31) * sc,
        crc: sf3.s(144, 16) as f64 * p2(-5),
        omega: sf3.s(160, 32) as f64 * p2(-31) * sc,
        omega_dot: sf3.s(192, 24) as f64 * p2(-43) * sc,
        idot: sf3.s(224, 14) as f64 * p2(-43) * sc,
    }
}

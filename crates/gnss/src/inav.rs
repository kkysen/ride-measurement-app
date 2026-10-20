//! Galileo E1-B I/NAV navigation messages as Android delivers them, decoded into ephemerides
//! (Galileo OS SIS ICD issue 2.0, sections 4.3 and 5.1).
//!
//! `GnssNavigationMessage.getData()` for `TYPE_GAL_I`: one nominal page, its even and odd parts of 114 bits each
//! (sync and tail removed), most significant bit first in 29 bytes.
//! The even part is even/odd (0), page type, and the word's first 112 bits; the odd part is even/odd (1),
//! page type, the word's last 16 bits, then 64 bits of other services, and a CRC-24Q over both parts up to it.

use std::collections::BTreeMap;

use crate::ephemeris::{GPS_PI, KeplerEphemeris};
use crate::system::{SatId, System};

/// `GnssNavigationMessage.TYPE_GAL_I`.
pub const TYPE_GAL_I: i32 = 0x0601;
/// Galileo System Time weeks start with GPS week 1024 (1999-08-22).
const GST_WEEK_OFFSET: u32 = 1024;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum InavError {
    TooShort,
    /// The parts aren't even then odd, or the page is an alert page.
    NotNominal,
    Crc,
}

/// One 128-bit I/NAV word.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Word(u128);

impl Word {
    /// `len` bits starting at bit `start`, counting from the word's first (most significant) bit.
    pub fn u(&self, start: usize, len: usize) -> u64 {
        ((self.0 >> (128 - start - len)) & ((1u128 << len) - 1)) as u64
    }

    /// The same bits, as two's complement.
    pub fn s(&self, start: usize, len: usize) -> i64 {
        let v = self.u(start, len) as i64;
        if v >> (len - 1) & 1 == 1 {
            v - (1 << len)
        } else {
            v
        }
    }

    pub fn word_type(&self) -> u8 {
        self.u(0, 6) as u8
    }

    fn iodnav(&self) -> u16 {
        self.u(6, 10) as u16
    }
}

/// Bit `i` of `data`, most significant first.
fn bit(data: &[u8], i: usize) -> u32 {
    u32::from(data[i / 8] >> (7 - i % 8) & 1)
}

fn bits(data: &[u8], start: usize, len: usize) -> u128 {
    (start..start + len).fold(0, |v, i| v << 1 | u128::from(bit(data, i)))
}

/// CRC-24Q (generator 0x1864CFB), as Galileo, SBAS, and RTCM 3 use, over the first `len` bits of `bits`.
pub fn crc24q(len: usize, bit_at: impl Fn(usize) -> u32) -> u32 {
    let mut crc: u32 = 0;
    for i in 0..len {
        crc ^= bit_at(i) << 23;
        crc = if crc & 0x80_0000 != 0 {
            (crc << 1) ^ 0x186_4cfb
        } else {
            crc << 1
        };
        crc &= 0xff_ffff;
    }
    crc
}

/// Checks and unpacks one page into its word.
pub fn decode_page(data: &[u8]) -> Result<Word, InavError> {
    if data.len() < 29 {
        return Err(InavError::TooShort);
    }
    // Even/odd flags and page types (1 marks an alert page).
    if bit(data, 0) != 0 || bit(data, 1) != 0 || bit(data, 114) != 1 || bit(data, 115) != 0 {
        return Err(InavError::NotNominal);
    }
    // The CRC covers the even part and the odd part's first 82 bits; it follows them.
    let crc = crc24q(114 + 82, |i| bit(data, i));
    if crc != bits(data, 114 + 82, 24) as u32 {
        return Err(InavError::Crc);
    }
    Ok(Word(bits(data, 2, 112) << 16 | bits(data, 114 + 2, 16)))
}

/// Collects word types 1 to 5 per satellite and yields an ephemeris once words 1 to 4 share an issue of data.
#[derive(Default)]
pub struct EphemerisAssembler {
    partial: BTreeMap<u8, [Option<Word>; 5]>,
}

impl EphemerisAssembler {
    pub fn add(&mut self, svid: u8, word: Word) -> Option<KeplerEphemeris> {
        let t = word.word_type();
        if !(1..=5).contains(&t) {
            return None;
        }
        let set = self.partial.entry(svid).or_default();
        set[usize::from(t - 1)] = Some(word);
        let [Some(w1), Some(w2), Some(w3), Some(w4), Some(w5)] = &*set else {
            return None;
        };
        let iod = w1.iodnav();
        if [w2, w3, w4].into_iter().any(|w| w.iodnav() != iod) {
            // Mid-update: wait for the rest of the new set.
            return None;
        }
        let eph = ephemeris(svid, w1, w2, w3, w4, w5);
        // Keep word 5 (clock-independent) for the next set.
        set[..4].fill(None);
        Some(eph)
    }
}

/// Field layout and scale factors from the ICD's tables 60 to 64 (word types 1 to 5), as offsets into the word.
fn ephemeris(svid: u8, w1: &Word, w2: &Word, w3: &Word, w4: &Word, w5: &Word) -> KeplerEphemeris {
    let p2 = |n: i32| 2f64.powi(n);
    let sc = GPS_PI;
    // Word 5: ionosphere (6 + 11 + 11 + 14 + 5 bits), then BGDs, health, week, and time of week.
    let bgd_e1_e5b = w5.s(57, 10) as f64 * p2(-32);
    let e1b_hs = w5.u(69, 2) as u8;
    let e1b_dvs = w5.u(72, 1) as u8;
    let week = w5.u(73, 12) as u32 + GST_WEEK_OFFSET;
    KeplerEphemeris {
        sat: SatId {
            system: System::Galileo,
            svid,
        },
        week,
        sv_accuracy_index: w3.u(120, 8) as u8,
        sv_health: e1b_hs << 1 | e1b_dvs,
        iodc: 0,
        iode: w1.iodnav(),
        tgd: bgd_e1_e5b,
        toe: w1.u(16, 14) as f64 * 60.0,
        m0: w1.s(30, 32) as f64 * p2(-31) * sc,
        e: w1.u(62, 32) as f64 * p2(-33),
        sqrt_a: w1.u(94, 32) as f64 * p2(-19),
        omega0: w2.s(16, 32) as f64 * p2(-31) * sc,
        i0: w2.s(48, 32) as f64 * p2(-31) * sc,
        omega: w2.s(80, 32) as f64 * p2(-31) * sc,
        idot: w2.s(112, 14) as f64 * p2(-43) * sc,
        omega_dot: w3.s(16, 24) as f64 * p2(-43) * sc,
        delta_n: w3.s(40, 16) as f64 * p2(-43) * sc,
        cuc: w3.s(56, 16) as f64 * p2(-29),
        cus: w3.s(72, 16) as f64 * p2(-29),
        crc: w3.s(88, 16) as f64 * p2(-5),
        crs: w3.s(104, 16) as f64 * p2(-5),
        cic: w4.s(22, 16) as f64 * p2(-29),
        cis: w4.s(38, 16) as f64 * p2(-29),
        toc: w4.u(54, 14) as f64 * 60.0,
        af0: w4.s(68, 31) as f64 * p2(-34),
        af1: w4.s(99, 21) as f64 * p2(-46),
        af2: w4.s(120, 6) as f64 * p2(-59),
    }
}

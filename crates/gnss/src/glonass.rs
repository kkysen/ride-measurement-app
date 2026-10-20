//! GLONASS L1 C/A navigation strings as Android delivers them, decoded into ephemerides
//! (GLONASS ICD edition 5.1, section 4.4).
//!
//! `GnssNavigationMessage.getData()` for `TYPE_GLO_L1CA`: one string of 85 bits (its idle bit, the string number,
//! 72 data bits, and 8 Hamming check bits), most significant first in 11 bytes.
//! Strings 1 to 4 of a frame carry a satellite's ephemeris and clock. Their reference time `tb` counts
//! quarter hours of the Moscow day, and the day comes from string 4's day of the four-year interval,
//! whose interval this takes from the receiver's own time.
//!
//! Field positions count from a string's first (idle) bit, with strings 1 to 4 back to back, 85 bits each.
//! They follow the ICD's table 4.5, cross-checked against RTKLIB's `rcvraw.c`.
//! Numbers are sign and magnitude, not two's complement.

use std::collections::BTreeMap;

use crate::ephemeris::{GlonassEphemeris, SECONDS_PER_WEEK};
use crate::system::{GLONASS_AHEAD_OF_UTC_NS, SatId, System};

/// `GnssNavigationMessage.TYPE_GLO_L1CA`.
pub const TYPE_GLO_L1CA: i32 = 0x0301;
pub const STRING_BITS: usize = 85;
/// Days from the GPS epoch (1980-01-06) to 1996-01-01, when GLONASS's first four-year interval starts.
const GPS_DAYS_TO_1996: i64 = 5839;
const DAYS_PER_INTERVAL: i64 = 1461;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GlonassError {
    TooShort,
    Hamming,
}

/// One string's 85 bits, the first in bit 84.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GloString(pub u128);

impl GloString {
    pub fn number(&self) -> u8 {
        bits(std::slice::from_ref(self), 1, 4) as u8
    }
}

fn bits(strings: &[GloString], start: usize, len: usize) -> u64 {
    (start..start + len).fold(0, |v, i| {
        let s = strings[i / STRING_BITS].0;
        v << 1 | (s >> (STRING_BITS - 1 - i % STRING_BITS) & 1) as u64
    })
}

/// Bytes of the 11-byte string that each of the 8 Hamming checks covers (the last over all of them);
/// a valid string has even parity under each. From RTKLIB's `test_glostr` in `src/rcvraw.c`
/// (version 2.4.3), which takes the string in this same layout, under this license:
//
// Copyright (c) 2007-2020, T. Takasu, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this list
// of conditions and the following disclaimer. Redistributions in binary form must
// reproduce the above copyright notice, this list of conditions and the following
// disclaimer in the documentation and/or other materials provided with the
// distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
pub const HAMMING_MASKS: [[u8; 11]; 8] = [
    [
        0x55, 0x55, 0x5A, 0xAA, 0xAA, 0xAA, 0xB5, 0x55, 0x6A, 0xD8, 0x08,
    ],
    [
        0x66, 0x66, 0x6C, 0xCC, 0xCC, 0xCC, 0xD9, 0x99, 0xB3, 0x68, 0x10,
    ],
    [
        0x87, 0x87, 0x8F, 0x0F, 0x0F, 0x0F, 0x1E, 0x1E, 0x3C, 0x70, 0x20,
    ],
    [
        0x07, 0xF8, 0x0F, 0xF0, 0x0F, 0xF0, 0x1F, 0xE0, 0x3F, 0x80, 0x40,
    ],
    [
        0xF8, 0x00, 0x0F, 0xFF, 0xF0, 0x00, 0x1F, 0xFF, 0xC0, 0x00, 0x80,
    ],
    [
        0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x01, 0x00,
    ],
    [
        0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
    ],
    [
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8,
    ],
];

pub fn hamming_ok(data: &[u8]) -> bool {
    HAMMING_MASKS.iter().all(|mask| {
        mask.iter()
            .zip(data)
            .fold(0, |p, (m, d)| p ^ (m & d).count_ones())
            % 2
            == 0
    })
}

/// Checks and unpacks one string.
pub fn decode_string(data: &[u8]) -> Result<GloString, GlonassError> {
    if data.len() < 11 {
        return Err(GlonassError::TooShort);
    }
    if !hamming_ok(&data[..11]) {
        return Err(GlonassError::Hamming);
    }
    let v = data[..11]
        .iter()
        .fold(0u128, |v, &b| v << 8 | u128::from(b));
    Ok(GloString(v >> 3))
}

/// One field: its bit range over strings 1 to 4 back to back, whether it's sign and magnitude, and its scale.
pub struct Field {
    pub start: usize,
    pub len: usize,
    pub signed: bool,
    pub scale: f64,
}

impl Field {
    pub fn raw(&self, strings: &[GloString]) -> i64 {
        let v = bits(strings, self.start, self.len) as i64;
        let magnitude = v & ((1 << (self.len - 1)) - 1);
        if self.signed && v >> (self.len - 1) == 1 {
            -magnitude
        } else if self.signed {
            magnitude
        } else {
            v
        }
    }

    pub fn value(&self, strings: &[GloString]) -> f64 {
        self.raw(strings) as f64 * self.scale
    }
}

const fn u(start: usize, len: usize) -> Field {
    Field {
        start,
        len,
        signed: false,
        scale: 1.0,
    }
}

const fn g(start: usize, len: usize, scale: f64) -> Field {
    Field {
        start,
        len,
        signed: true,
        scale,
    }
}

const P2_11: f64 = 1.0 / 2048.0;
const P2_20: f64 = 1.0 / 1_048_576.0;
const P2_30: f64 = P2_20 / 1024.0;
const P2_40: f64 = P2_30 / 1024.0;
const S2: usize = STRING_BITS;
const S3: usize = 2 * STRING_BITS;
const S4: usize = 3 * STRING_BITS;

/// Position (km), velocity (km/s), and lunisolar acceleration (km/s^2) per axis, in strings 1 to 3.
pub const POSITION: [Field; 3] = [
    g(50, 27, P2_11),
    g(S2 + 50, 27, P2_11),
    g(S3 + 50, 27, P2_11),
];
pub const VELOCITY: [Field; 3] = [
    g(21, 24, P2_20),
    g(S2 + 21, 24, P2_20),
    g(S3 + 21, 24, P2_20),
];
pub const ACCELERATION: [Field; 3] = [g(45, 5, P2_30), g(S2 + 45, 5, P2_30), g(S3 + 45, 5, P2_30)];
/// The most significant bit of B_n: set when the satellite is unhealthy.
pub const BN_UNHEALTHY: Field = u(S2 + 5, 1);
/// Quarter hours of the Moscow day.
pub const TB: Field = u(S2 + 9, 7);
pub const GAMMA_N: Field = g(S3 + 6, 11, P2_40);
pub const L_N: Field = u(S3 + 20, 1);
pub const TAU_N: Field = g(S4 + 5, 22, P2_30);
/// Day of the four-year interval, from 1 (1 January of its leap year).
pub const N_T: Field = u(S4 + 59, 11);
pub const SLOT: Field = u(S4 + 70, 5);

/// Collects strings 1 to 4 per satellite as they arrive in order, and yields an ephemeris from each full set.
#[derive(Default)]
pub struct EphemerisAssembler {
    partial: BTreeMap<u8, Vec<GloString>>,
}

impl EphemerisAssembler {
    /// `gps_s`: the receiver's time (GPS seconds since the GPS epoch) to within a year or so, to tell which
    /// four-year interval the strings' day is in; `leap_seconds`: GPS minus UTC.
    pub fn add(
        &mut self,
        svid: u8,
        s: GloString,
        gps_s: f64,
        leap_seconds: i32,
    ) -> Option<GlonassEphemeris> {
        let n = s.number();
        let set = self.partial.entry(svid).or_default();
        if n == 1 {
            set.clear();
        }
        if !(1..=4).contains(&n) || set.len() + 1 != usize::from(n) {
            // Not the next of a frame's first four: start over at the next string 1.
            if n != 1 {
                set.clear();
            }
            return None;
        }
        set.push(s);
        if set.len() < 4 {
            return None;
        }
        let strings = std::mem::take(set);
        ephemeris(svid, &strings, gps_s, leap_seconds)
    }
}

/// `tb` on day `n_t` of the four-year interval nearest `gps_s`, in GPS seconds since the GPS epoch.
fn reference_time(n_t: i64, tb_s: i64, gps_s: f64, leap_seconds: i32) -> f64 {
    let ahead_s = GLONASS_AHEAD_OF_UTC_NS / 1_000_000_000;
    let at = |n4: i64| {
        let day = GPS_DAYS_TO_1996 + DAYS_PER_INTERVAL * (n4 - 1) + n_t - 1;
        (day * 86_400 + tb_s - ahead_s + i64::from(leap_seconds)) as f64
    };
    let interval_s = (DAYS_PER_INTERVAL * 86_400) as f64;
    let guess = ((gps_s - at(1)) / interval_s).round() as i64 + 1;
    (guess - 1..=guess + 1)
        .map(at)
        .min_by(|a, b| (a - gps_s).abs().total_cmp(&(b - gps_s).abs()))
        .unwrap()
}

fn ephemeris(
    svid: u8,
    strings: &[GloString],
    gps_s: f64,
    leap_seconds: i32,
) -> Option<GlonassEphemeris> {
    let n_t = N_T.raw(strings);
    if !(1..=DAYS_PER_INTERVAL).contains(&n_t) {
        return None;
    }
    let t = reference_time(n_t, TB.raw(strings) * 900, gps_s, leap_seconds);
    let week = (t / SECONDS_PER_WEEK).floor();
    let slot = match SLOT.raw(strings) as u8 {
        s @ 1..=24 => s,
        _ => svid,
    };
    let km = |f: &[Field; 3]| -> [f64; 3] { std::array::from_fn(|k| f[k].value(strings) * 1e3) };
    Some(GlonassEphemeris {
        sat: SatId {
            system: System::Glonass,
            svid: slot,
        },
        frequency_channel: None,
        healthy: BN_UNHEALTHY.raw(strings) == 0 && L_N.raw(strings) == 0,
        week: week as u32,
        tb: t - week * SECONDS_PER_WEEK,
        clock_bias: -TAU_N.value(strings),
        clock_rate: GAMMA_N.value(strings),
        position: km(&POSITION),
        velocity: km(&VELOCITY),
        acceleration: km(&ACCELERATION),
    })
}

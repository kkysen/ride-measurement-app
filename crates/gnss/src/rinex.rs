//! Broadcast ephemerides from RINEX navigation files (version 2 for GPS, version 3 for any constellation),
//! e.g. the IGS daily merged `BRDM00DLR_S_*_MN.rnx`, to solve without navigation messages from the phone.

use crate::ephemeris::{Ephemeris, GlonassEphemeris, KeplerEphemeris, SECONDS_PER_WEEK};
use crate::system::{DEFAULT_LEAP_SECONDS, SatId, System};

/// Galileo `Data sources` bit 9: the clock is for E5b and E1, as I/NAV (the E1 navigation message) broadcasts.
const GALILEO_INAV_CLOCK: u32 = 1 << 9;
/// Galileo `SV health` bits 0 to 2: E1B data validity and signal health.
const GALILEO_E1B_HEALTH: u8 = 0b111;
/// Android numbers QZSS satellites from 193; RINEX from 1.
const QZSS_FIRST_SVID: u8 = 193;

/// GLONASS slots are 1 to 24, with a few more for spares.
const GLONASS_MAX_SLOT: u8 = 27;

/// Every ephemeris in a RINEX navigation file (GPS, QZSS, Galileo I/NAV, BeiDou, GLONASS).
/// Records of other constellations, and ones that don't parse, are skipped.
pub fn read_nav(text: &str) -> Vec<Ephemeris> {
    let version: f64 = text
        .lines()
        .next()
        .and_then(|l| l.get(..9))
        .and_then(|v| v.trim().parse().ok())
        .unwrap_or(0.0);
    let leap_seconds = text
        .lines()
        .take_while(|l| !l.contains("END OF HEADER"))
        .find(|l| {
            l.get(60..)
                .is_some_and(|label| label.starts_with("LEAP SECONDS"))
        })
        .and_then(|l| l.get(..6)?.trim().parse().ok())
        .unwrap_or(DEFAULT_LEAP_SECONDS);
    let body: Vec<&str> = text
        .lines()
        .skip_while(|l| !l.contains("END OF HEADER"))
        .skip(1)
        .collect();
    // A record starts with its satellite in the first 3 columns and continues with lines indented further.
    let mut records: Vec<Vec<&str>> = Vec::new();
    for line in body {
        if line.get(..3).unwrap_or(line).trim().is_empty() {
            if let Some(r) = records.last_mut() {
                r.push(line);
            }
        } else if !line.trim().is_empty() {
            records.push(vec![line]);
        }
    }
    records
        .iter()
        .filter_map(|r| {
            if version >= 3.0 {
                record_v3(r, leap_seconds)
            } else {
                record_v2(r).map(Ephemeris::from)
            }
        })
        .collect()
}

/// A field: Fortran `D` exponents allowed, blank as zero.
fn number(s: &str) -> Option<f64> {
    let s = s.trim();
    if s.is_empty() {
        return Some(0.0);
    }
    s.replace(['D', 'd'], "E").parse().ok()
}

/// The record's values in order: three on the first line from `first_column`, four per line after `indent`.
fn values(lines: &[&str], first_column: usize, indent: usize) -> Option<Vec<f64>> {
    let mut v = Vec::new();
    let mut take = |line: &str, start: usize, count: usize| -> Option<()> {
        for k in 0..count {
            let from = start + 19 * k;
            let field = line.get(from..(from + 19).min(line.len())).unwrap_or("");
            v.push(number(field)?);
        }
        Some(())
    };
    take(lines.first()?, first_column, 3)?;
    for line in &lines[1..] {
        take(line, indent, 4)?;
    }
    Some(v)
}

/// Seconds into the week (from Sunday 00:00, as GPS and BeiDou weeks start) of a calendar date and time.
#[cfg(test)]
fn seconds_of_week(year: i64, month: i64, day: i64, h: i64, m: i64, s: f64) -> f64 {
    week_and_seconds(year, month, day, h, m, s).1
}

/// The week since 1980-01-06 (GPS's week 0) and seconds into it of a calendar date and time.
fn week_and_seconds(year: i64, month: i64, day: i64, h: i64, m: i64, s: f64) -> (i64, f64) {
    // Days since 1970-01-01 (Howard Hinnant's days_from_civil); 1970-01-04 was a Sunday.
    let y = if month <= 2 { year - 1 } else { year };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let doy = (153 * (month + if month > 2 { -3 } else { 9 }) + 2) / 5 + day - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    // 1980-01-06 is day 3657.
    let days = era * 146_097 + doe - 719_468 - 3657;
    (
        days.div_euclid(7),
        days.rem_euclid(7) as f64 * 86_400.0 + (h * 3600 + m * 60) as f64 + s,
    )
}

fn epoch(fields: &str) -> Option<f64> {
    Some(epoch_week(fields)?.1)
}

/// A record's epoch as a week since 1980-01-06 and seconds into it.
fn epoch_week(fields: &str) -> Option<(i64, f64)> {
    let p: Vec<&str> = fields.split_whitespace().collect();
    if p.len() < 6 {
        return None;
    }
    let mut year: i64 = p[0].parse().ok()?;
    if year < 100 {
        year += if year >= 80 { 1900 } else { 2000 };
    }
    let i = |k: usize| p[k].parse::<i64>().ok();
    Some(week_and_seconds(
        year,
        i(1)?,
        i(2)?,
        i(3)?,
        i(4)?,
        p[5].parse().ok()?,
    ))
}

fn record_v2(lines: &[&str]) -> Option<KeplerEphemeris> {
    let first = lines.first()?;
    let svid: u8 = first.get(..2)?.trim().parse().ok()?;
    let toc = epoch(first.get(2..22)?)?;
    kepler(
        SatId {
            system: System::Gps,
            svid,
        },
        toc,
        &values(lines, 22, 3)?,
    )
}

fn record_v3(lines: &[&str], leap_seconds: i32) -> Option<Ephemeris> {
    let first = lines.first()?;
    let system = match first.get(..1)? {
        "R" => return glonass(lines, leap_seconds).map(Ephemeris::from),
        "G" => System::Gps,
        "J" => System::Qzss,
        "E" => System::Galileo,
        "C" => System::Beidou,
        _ => return None,
    };
    let number: u8 = first.get(1..3)?.trim().parse().ok()?;
    let svid = match system {
        System::Qzss => QZSS_FIRST_SVID + number - 1,
        _ => number,
    };
    let toc = epoch(first.get(3..23)?)?;
    kepler(SatId { system, svid }, toc, &values(lines, 23, 4)?).map(Ephemeris::from)
}

/// A GLONASS record (RINEX 3.04 table A10): its epoch is `tb` in UTC; positions are in km.
fn glonass(lines: &[&str], leap_seconds: i32) -> Option<GlonassEphemeris> {
    let first = lines.first()?;
    let slot: u8 = first.get(1..3)?.trim().parse().ok()?;
    if !(1..=GLONASS_MAX_SLOT).contains(&slot) || lines.len() < 4 {
        return None;
    }
    let (mut week, utc) = epoch_week(first.get(3..23)?)?;
    let v = values(&lines[..4], 23, 4)?;
    let mut tb = utc + f64::from(leap_seconds);
    if tb >= SECONDS_PER_WEEK {
        tb -= SECONDS_PER_WEEK;
        week += 1;
    }
    let km = |k: usize| v[k] * 1000.0;
    Some(GlonassEphemeris {
        sat: SatId {
            system: System::Glonass,
            svid: slot,
        },
        frequency_channel: Some(v[10] as i8),
        healthy: v[6] == 0.0,
        week: u32::try_from(week).ok()?,
        tb,
        clock_bias: v[0],
        clock_rate: v[1],
        position: [km(3), km(7), km(11)],
        velocity: [km(4), km(8), km(12)],
        acceleration: [km(5), km(9), km(13)],
    })
}

/// The broadcast orbit fields, in the order every Keplerian record has them (RINEX 3.04 table A6 and on).
fn kepler(sat: SatId, toc: f64, v: &[f64]) -> Option<KeplerEphemeris> {
    if v.len() < 27 {
        return None;
    }
    let (sv_health, tgd) = match sat.system {
        System::Galileo => {
            if (v[20] as u32) & GALILEO_INAV_CLOCK == 0 {
                // F/NAV: its clock is for E5a, and I/NAV repeats the same orbits for E1.
                return None;
            }
            (v[24] as u8 & GALILEO_E1B_HEALTH, v[26])
        }
        _ => (v[24] as u8, v[25]),
    };
    Some(KeplerEphemeris {
        sat,
        week: v[21] as u32,
        sv_accuracy_index: 0,
        sv_health,
        iodc: if matches!(sat.system, System::Gps | System::Qzss) {
            v[26] as u16
        } else {
            0
        },
        iode: v[3] as u16,
        tgd,
        toc,
        af0: v[0],
        af1: v[1],
        af2: v[2],
        crs: v[4],
        delta_n: v[5],
        m0: v[6],
        cuc: v[7],
        e: v[8],
        cus: v[9],
        sqrt_a: v[10],
        toe: v[11],
        cic: v[12],
        omega0: v[13],
        cis: v[14],
        i0: v[15],
        crc: v[16],
        omega: v[17],
        omega_dot: v[18],
        idot: v[19],
    })
}

#[cfg(test)]
mod tests {
    use super::seconds_of_week;

    #[test]
    fn weeks_start_on_sunday() {
        // 2021-04-08 was a Thursday; 2010-07-01 too.
        assert_eq!(seconds_of_week(2021, 4, 8, 0, 0, 0.0), 4.0 * 86_400.0);
        assert_eq!(
            seconds_of_week(2010, 7, 1, 1, 0, 0.0),
            4.0 * 86_400.0 + 3600.0
        );
        assert_eq!(seconds_of_week(1980, 1, 6, 0, 0, 0.0), 0.0);
    }
}

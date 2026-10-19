//! Shared test helpers.

use ride_gnss::ephemeris::GpsEphemeris;

fn d(s: &str) -> f64 {
    s.trim().replace('D', "E").parse().unwrap()
}

/// The 00:00 records of a RINEX 2 GPS navigation file.
pub fn read_rinex_nav(text: &str) -> Vec<GpsEphemeris> {
    let lines: Vec<&str> = text
        .lines()
        .skip_while(|l| !l.contains("END OF HEADER"))
        .skip(1)
        .collect();
    lines
        .chunks(8)
        .filter(|c| c.len() == 8)
        .map(|c| {
            let f = |line: usize, k: usize| {
                let l = c[line];
                if line == 0 {
                    d(&l[22 + 19 * k..22 + 19 * (k + 1)])
                } else {
                    d(&l[3 + 19 * k..(3 + 19 * (k + 1)).min(l.len())])
                }
            };
            GpsEphemeris {
                prn: c[0][..2].trim().parse().unwrap(),
                af0: f(0, 0),
                af1: f(0, 1),
                af2: f(0, 2),
                iode: f(1, 0) as u8,
                crs: f(1, 1),
                delta_n: f(1, 2),
                m0: f(1, 3),
                cuc: f(2, 0),
                e: f(2, 1),
                cus: f(2, 2),
                sqrt_a: f(2, 3),
                toe: f(3, 0),
                cic: f(3, 1),
                omega0: f(3, 2),
                cis: f(3, 3),
                i0: f(4, 0),
                crc: f(4, 1),
                omega: f(4, 2),
                omega_dot: f(4, 3),
                idot: f(5, 0),
                week: f(5, 2) as u32,
                sv_accuracy_index: 0,
                sv_health: f(6, 1) as u8,
                tgd: f(6, 2),
                iodc: f(6, 3) as u16,
                toc: f(3, 0),
            }
        })
        .collect()
}

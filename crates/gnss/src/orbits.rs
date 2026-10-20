//! Downloaded broadcast orbits for a session: which files to fetch, where they go, and reading them back.
//! The app (live) and `just orbits` (offline) both follow [`fetch_plan`] and only do the HTTP requests,
//! and the app's fusion and `ride gnss`/`ride map` read the same files with [`read_session_orbits`].

use std::io::Read;
use std::path::{Path, PathBuf};

use crate::ephemeris::Ephemeris;
use crate::rinex::read_nav;

/// Where a session keeps downloaded orbits, relative to its directory.
pub const SESSION_ORBITS_DIR: &str = "orbits";
const BKG_BRDC: &str = "https://igs.bkg.bund.de/root_ftp/IGS/BRDC";
const DAY_MS: i64 = 86_400_000;
/// How far back from now the live fusion needs orbits: broadcast ephemerides are good for up to 4 hours.
pub const LIVE_LOOKBACK_MS: i64 = 4 * 3_600_000;

/// How often the app follows [`fetch_plan`] again while recording: BKG updates `BRDC00WRD` every 15 minutes.
pub const LIVE_REFRESH_MS: i64 = 15 * 60_000;

/// One file to try: download `url` to `path` (via a temporary file, then a rename).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OrbitFile {
    pub url: String,
    pub path: PathBuf,
}

/// For each UTC day overlapping `start_ms..=end_ms` (Unix ms), the files to try in order, stopping at the first
/// that downloads: the day's complete merged file (`BRDM00DLR`, published the day after), else BKG's file
/// updated every 15 minutes (`BRDC00WRD`), which is fetched again each time, since it only grows.
/// Days whose complete file is already there are left out.
pub fn fetch_plan(session_dir: &Path, start_ms: i64, end_ms: i64) -> Vec<Vec<OrbitFile>> {
    let dir = session_dir.join(SESSION_ORBITS_DIR);
    (start_ms.div_euclid(DAY_MS)..=end_ms.div_euclid(DAY_MS))
        .filter_map(|day| {
            let (year, doy) = year_and_day(day);
            let file = |name: &str| OrbitFile {
                url: format!("{BKG_BRDC}/{year}/{doy:03}/{name}"),
                path: dir.join(name),
            };
            let complete = file(&format!("BRDM00DLR_S_{year}{doy:03}0000_01D_MN.rnx.gz"));
            if complete.path.exists() {
                return None;
            }
            Some(vec![
                complete,
                file(&format!("BRDC00WRD_S_{year}{doy:03}0000_01D_MN.rnx.gz")),
            ])
        })
        .collect()
}

/// Every ephemeris in a session's [`SESSION_ORBITS_DIR`] (`*.rnx` and `*.rnx.gz`), if it has one.
pub fn read_session_orbits(session_dir: &Path) -> std::io::Result<Vec<Ephemeris>> {
    let dir = session_dir.join(SESSION_ORBITS_DIR);
    if !dir.is_dir() {
        return Ok(Vec::new());
    }
    let mut paths: Vec<_> = std::fs::read_dir(&dir)?
        .filter_map(|e| Some(e.ok()?.path()))
        .filter(|p| is_orbit_file(p))
        .collect();
    paths.sort();
    let mut out = Vec::new();
    for p in paths {
        out.extend(read_orbit_file(&p)?);
    }
    Ok(out)
}

fn is_orbit_file(p: &Path) -> bool {
    let name = p.file_name().and_then(|n| n.to_str()).unwrap_or("");
    name.ends_with(".rnx") || name.ends_with(".rnx.gz")
}

/// The ephemerides in one RINEX navigation file, gzipped (as downloaded) or not.
pub fn read_orbit_file(path: &Path) -> std::io::Result<Vec<Ephemeris>> {
    let bytes = std::fs::read(path)?;
    let text = if bytes.starts_with(&[0x1f, 0x8b]) {
        let mut s = String::new();
        flate2::read::MultiGzDecoder::new(&bytes[..]).read_to_string(&mut s)?;
        s
    } else {
        String::from_utf8(bytes).map_err(std::io::Error::other)?
    };
    Ok(read_nav(&text))
}

/// Days since 1970-01-01 as a year and day of the year (1 to 366).
pub fn year_and_day(days: i64) -> (i64, i64) {
    let mut year = 1970;
    let mut d = days;
    loop {
        let len = if (year % 4 == 0 && year % 100 != 0) || year % 400 == 0 {
            366
        } else {
            365
        };
        if d < len {
            return (year, d + 1);
        }
        d -= len;
        year += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn days_of_year() {
        assert_eq!(year_and_day(0), (1970, 1));
        // 2021-04-08, and the last day of leap year 2024.
        assert_eq!(year_and_day(18_725), (2021, 98));
        assert_eq!(year_and_day(20_088), (2024, 366));
    }

    #[test]
    fn plan_covers_each_day_and_skips_complete_ones() {
        let dir = tempfile::tempdir().unwrap();
        // 2021-04-08 23:00 to 2021-04-09 01:00 UTC.
        let start = 18_725 * DAY_MS + 23 * 3_600_000;
        let plan = fetch_plan(dir.path(), start, start + 2 * 3_600_000);
        assert_eq!(plan.len(), 2);
        assert_eq!(
            plan[0][0].url,
            "https://igs.bkg.bund.de/root_ftp/IGS/BRDC/2021/098/BRDM00DLR_S_20210980000_01D_MN.rnx.gz"
        );
        assert!(
            plan[0][1]
                .url
                .ends_with("/2021/098/BRDC00WRD_S_20210980000_01D_MN.rnx.gz")
        );
        assert!(plan[1][0].url.contains("/2021/099/"));
        std::fs::create_dir_all(plan[0][0].path.parent().unwrap()).unwrap();
        std::fs::write(&plan[0][0].path, b"").unwrap();
        let plan = fetch_plan(dir.path(), start, start + 2 * 3_600_000);
        assert_eq!(plan.len(), 1);
        assert!(plan[0][0].url.contains("/2021/099/"));
    }

    #[test]
    fn reads_gzipped_and_plain_files() {
        use std::io::Write;
        let text = include_str!("../tests/data/brdc1820.10n");
        let dir = tempfile::tempdir().unwrap();
        let orbits = dir.path().join(SESSION_ORBITS_DIR);
        std::fs::create_dir_all(&orbits).unwrap();
        std::fs::write(orbits.join("a.rnx"), text).unwrap();
        let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::default());
        gz.write_all(text.as_bytes()).unwrap();
        std::fs::write(orbits.join("b.rnx.gz"), gz.finish().unwrap()).unwrap();
        std::fs::write(orbits.join("b.rnx.gz.part"), b"partial download").unwrap();
        let n = read_nav(text).len();
        assert_eq!(read_session_orbits(dir.path()).unwrap().len(), 2 * n);
    }
}

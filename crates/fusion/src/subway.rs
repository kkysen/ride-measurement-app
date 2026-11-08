//! Whether the phone is in the subway (on a train, or in a station), from what it senses:
//! the subway's own Wi-Fi, or no GNSS fix with few networks around, as underground.
//! Never from the rider's marks, which only score it (`python/experiments/subway.py`).
//!
//! On the first five rides (2026-10-20 and 21) this is on for 97 to 100% of the time on underground trains,
//! and mostly off on the street; where it's on with a good fix, it's nearly always on a platform or in a station.
//! Elevated lines keep GNSS and see street Wi-Fi, so it misses them (61% on the 7), which matters less:
//! there, GNSS keeps the speed from running away.
//!
//! Two guards, from an NJ Transit bus out of Port Authority on 2026-10-22 (`20261022-002914`): the MTA Wi-Fi seen in
//! the terminal (the 42 St subway is under it) held "subway" onto the ramps outside, and the Lincoln Tunnel had no fix
//! and few networks. So a good fix while moving ends the Wi-Fi's hold, and no fix counts as underground only with the
//! trains' magnetic field: the traction current swings |B| by over [`MAG_BURST_UT`] (std over 10 s) within every
//! 90 s on every subway ride so far, and never did on the bus in the tunnel (at most 7.3 uT).

/// The subway's Wi-Fi (Boldyn, formerly Transit Wireless, and the MTA's own), by exact SSID or prefix.
/// Not `BoldynPasspoint`: despite the name it's outdoors, up to -31 dBm on the street and never seen on a train.
const SUBWAY_SSIDS: &[&str] = &["BoldynWiFi", "PasspointBoldyn", "NYCTA", "FINDSystem"];
const SUBWAY_SSID_PREFIXES: &[&str] = &["OFMTA", "TW_Survey_"];

/// Subway Wi-Fi this recent means still in the subway (scans come every few seconds to a minute).
const SUBWAY_WIFI_HOLD_NS: i64 = 60_000_000_000;
/// A fix this recent and this good means GNSS is working, so not underground.
const FIX_HOLD_NS: i64 = 10_000_000_000;
const GOOD_FIX_ACCURACY_M: f64 = 50.0;
/// The subway's track above ground, from `python/experiments/open_track.py` (MTA GTFS shapes between stations that
/// are elevated, on a viaduct or embankment, at grade, or in an open cut, and OpenStreetMap's subway track not in a
/// tunnel, for bridges and viaducts between underground stations): little-endian i32 pairs, latitude and
/// longitude in 1e-5 degrees, every 10 m. Underground track is left out: it runs under streets.
const OPEN_TRACK: &[u8] = include_bytes!("../data/open_track.bin");
/// A good fix this close to open track (m), moving at least [`ON_TRACK_MPS`], is a train on it;
/// it counts for [`ON_TRACK_HOLD_NS`]. Elevated lines see street Wi-Fi and keep GNSS, so nothing else catches them.
const ON_TRACK_M: f64 = 25.0;
const ON_TRACK_MPS: f64 = 5.0;
const ON_TRACK_HOLD_NS: i64 = 30_000_000_000;
/// Open track points (1e-5 degrees) by grid cell (1e-3 degrees), for nearest-point lookups.
type TrackGrid = std::collections::HashMap<(i32, i32), Vec<(i32, i32)>>;

fn open_track() -> &'static TrackGrid {
    static GRID: std::sync::OnceLock<TrackGrid> = std::sync::OnceLock::new();
    GRID.get_or_init(|| {
        let mut grid = TrackGrid::default();
        for p in OPEN_TRACK.as_chunks::<8>().0 {
            let lat = i32::from_le_bytes([p[0], p[1], p[2], p[3]]);
            let lon = i32::from_le_bytes([p[4], p[5], p[6], p[7]]);
            grid.entry((lat.div_euclid(100), lon.div_euclid(100)))
                .or_default()
                .push((lat, lon));
        }
        grid
    })
}

/// Distance (m) from a point to the nearest open track, if within about 100 m.
pub fn open_track_m(lat: f64, lon: f64) -> Option<f64> {
    let (la, lo) = ((lat * 1e5).round() as i32, (lon * 1e5).round() as i32);
    let (ca, co) = (la.div_euclid(100), lo.div_euclid(100));
    let m_lat = 1e-5_f64.to_radians() * 6_371_000.0;
    let m_lon = m_lat * lat.to_radians().cos();
    let mut best: Option<f64> = None;
    for da in -1..=1 {
        for dn in -1..=1 {
            for &(pa, po) in open_track().get(&(ca + da, co + dn)).into_iter().flatten() {
                let d = (f64::from(pa - la) * m_lat).hypot(f64::from(po - lo) * m_lon);
                best = Some(best.map_or(d, |b: f64| b.min(d)));
            }
        }
    }
    best
}

/// A fix while moving at least this fast (m/s) is outdoors on the move, not on a platform.
const MOVING_MPS: f64 = 3.0;
/// The magnetometer: windows its spread is measured over, the spread that means a train's traction current,
/// and how long one keeps counting.
const MAG_WINDOW_NS: i64 = 10_000_000_000;
const MAG_BURST_UT: f64 = 10.0;
const MAG_HOLD_NS: i64 = 90_000_000_000;
/// Fewer networks than this in the latest scan, without a fix, looks like underground rather than indoors.
const FEW_NETWORKS: usize = 20;

/// An SSID as recorded: Android's `WifiSsid.toString()` quotes UTF-8 names (`"BoldynWiFi"`, quotes included).
pub fn is_subway_ssid(ssid: &str) -> bool {
    let ssid = ssid.trim();
    let ssid = ssid
        .strip_prefix('"')
        .and_then(|s| s.strip_suffix('"'))
        .unwrap_or(ssid)
        .trim();
    SUBWAY_SSIDS.contains(&ssid) || SUBWAY_SSID_PREFIXES.iter().any(|p| ssid.starts_with(p))
}

#[derive(Clone, Debug, Default)]
pub struct SubwayDetector {
    last_subway_wifi_ns: Option<i64>,
    last_good_fix_ns: Option<i64>,
    /// How many networks the latest scan saw, once there's been one.
    last_scan_networks: Option<usize>,
    /// The latest good fix while moving, and the latest on open track (see [`ON_TRACK_M`]).
    last_moving_fix_ns: Option<i64>,
    last_on_track_ns: Option<i64>,
    /// The current magnetometer window: its start, and the count, sum, and sum of squares of |B| (uT).
    mag_window: Option<(i64, f64, f64, f64)>,
    last_mag_burst_ns: Option<i64>,
}

impl SubwayDetector {
    /// A Wi-Fi scan's networks, at the time it was recorded.
    pub fn on_wifi_scan<'a>(&mut self, t_ns: i64, ssids: impl IntoIterator<Item = &'a str>) {
        let mut n = 0;
        for ssid in ssids {
            n += 1;
            if is_subway_ssid(ssid) {
                self.last_subway_wifi_ns = Some(t_ns);
            }
        }
        self.last_scan_networks = Some(n);
    }

    pub fn on_fix(
        &mut self,
        t_ns: i64,
        lat: f64,
        lon: f64,
        accuracy_m: Option<f64>,
        speed_mps: Option<f64>,
    ) {
        if accuracy_m.is_some_and(|a| a < GOOD_FIX_ACCURACY_M) {
            self.last_good_fix_ns = Some(t_ns);
            if speed_mps.is_some_and(|v| v >= MOVING_MPS) {
                self.last_moving_fix_ns = Some(t_ns);
            }
            if speed_mps.is_some_and(|v| v >= ON_TRACK_MPS)
                && accuracy_m.is_some_and(|a| a <= ON_TRACK_M)
                && open_track_m(lat, lon).is_some_and(|d| d <= ON_TRACK_M)
            {
                self.last_on_track_ns = Some(t_ns);
            }
        }
    }

    /// A magnetometer sample's magnitude (uT).
    pub fn on_magnetometer(&mut self, t_ns: i64, magnitude_ut: f64) {
        let (start, n, sum, sq) = self.mag_window.get_or_insert((t_ns, 0.0, 0.0, 0.0));
        if t_ns - *start >= MAG_WINDOW_NS {
            if *n > 20.0 {
                let mean = *sum / *n;
                let std = (*sq / *n - mean * mean).max(0.0).sqrt();
                // Only without GNSS: steel structures swing it too (13 uT on the ramps out of Port Authority),
                // and they're outdoors.
                let fix_recent = self
                    .last_good_fix_ns
                    .is_some_and(|f| t_ns - f <= FIX_HOLD_NS);
                if std > MAG_BURST_UT && !fix_recent {
                    self.last_mag_burst_ns = Some(t_ns);
                }
            }
            self.mag_window = Some((t_ns, 1.0, magnitude_ut, magnitude_ut * magnitude_ut));
            return;
        }
        *n += 1.0;
        *sum += magnitude_ut;
        *sq += magnitude_ut * magnitude_ut;
    }

    pub fn in_subway(&self, t_ns: i64) -> bool {
        let recent = |last: Option<i64>, hold: i64| last.is_some_and(|l| t_ns - l <= hold);
        let wifi = recent(self.last_subway_wifi_ns, SUBWAY_WIFI_HOLD_NS)
            && !self
                .last_moving_fix_ns
                .is_some_and(|m| self.last_subway_wifi_ns.is_some_and(|w| m > w));
        wifi || recent(self.last_on_track_ns, ON_TRACK_HOLD_NS)
            || (!recent(self.last_good_fix_ns, FIX_HOLD_NS)
                && self.last_scan_networks.is_some_and(|n| n < FEW_NETWORKS)
                && recent(self.last_mag_burst_ns, MAG_HOLD_NS))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const S: i64 = 1_000_000_000;

    #[test]
    fn subway_wifi_but_not_the_street_one() {
        assert!(is_subway_ssid("BoldynWiFi"));
        assert!(is_subway_ssid("\"PasspointBoldyn\""));
        assert!(!is_subway_ssid("\"BoldynPasspoint\""));
        assert!(is_subway_ssid(" OFMTA1XTLA1YT "));
        assert!(is_subway_ssid("TW_Survey_58"));
        assert!(!is_subway_ssid("BoldynPasspoint"));
        assert!(!is_subway_ssid("LinkNYC Free Wi-Fi"));
    }

    #[test]
    fn in_while_subway_wifi_is_recent() {
        let mut d = SubwayDetector::default();
        let street: Vec<String> = (0..40).map(|i| format!("Home-{i}")).collect();
        d.on_fix(0, 40.0, -74.5, Some(5.0), None);
        d.on_wifi_scan(0, street.iter().map(String::as_str));
        assert!(!d.in_subway(S));
        d.on_wifi_scan(
            10 * S,
            ["BoldynWiFi"]
                .into_iter()
                .chain(street.iter().map(String::as_str)),
        );
        d.on_fix(10 * S, 40.0, -74.5, Some(5.0), None);
        assert!(d.in_subway(10 * S));
        assert!(d.in_subway(70 * S));
        d.on_fix(80 * S, 40.0, -74.5, Some(5.0), None);
        assert!(!d.in_subway(80 * S));
    }

    #[test]
    fn in_without_a_fix_and_few_networks() {
        let mut d = SubwayDetector::default();
        // A train's traction current swinging |B|.
        for i in 0..=200 {
            d.on_magnetometer(i * S / 20 - 10 * S, if i % 2 == 0 { 30.0 } else { 60.0 });
        }
        // No scans yet: no fix alone isn't enough (Wi-Fi scanning may be off).
        assert!(!d.in_subway(0));
        d.on_wifi_scan(0, ["A", "B"]);
        assert!(d.in_subway(S));
        d.on_fix(2 * S, 40.0, -74.5, Some(8.0), None);
        assert!(!d.in_subway(3 * S));
        // A poor fix doesn't count.
        assert!(d.in_subway(13 * S));
        d.on_fix(14 * S, 40.0, -74.5, Some(500.0), None);
        assert!(d.in_subway(14 * S));
    }

    #[test]
    fn a_road_tunnel_is_not_the_subway() {
        let mut d = SubwayDetector::default();
        for i in 0..=2400 {
            d.on_magnetometer(i * S / 20, 45.0 + if i % 2 == 0 { 2.0 } else { -2.0 });
        }
        d.on_wifi_scan(0, ["A", "B"]);
        assert!(!d.in_subway(100 * S));
    }

    #[test]
    fn a_moving_fix_ends_the_subway_wifi() {
        let mut d = SubwayDetector::default();
        d.on_wifi_scan(0, ["\"OFMTA1XWIFI\""]);
        d.on_fix(5 * S, 40.0, -74.5, Some(30.0), Some(1.0));
        assert!(d.in_subway(10 * S));
        d.on_fix(20 * S, 40.0, -74.5, Some(5.0), Some(8.0));
        assert!(!d.in_subway(20 * S));
    }

    #[test]
    fn a_fix_moving_on_elevated_track() {
        // The 7 over Roosevelt Av at 74 St, and a block south of it.
        assert!(open_track_m(40.74685, -73.89135).is_some_and(|d| d < 25.0));
        assert!(open_track_m(40.7455, -73.8913).is_none_or(|d| d > 50.0));
        // Under Broadway in Manhattan, the 1 is underground.
        assert!(open_track_m(40.7990, -73.9682).is_none());
        // The Manhattan Bridge and the 1's viaduct at 125 St, between underground stations,
        // but not Chrystie St over the B and D at Grand St, where the shuttle bus runs.
        assert!(open_track_m(40.7074, -73.9907).is_some_and(|d| d < 25.0));
        assert!(open_track_m(40.8155, -73.9586).is_some_and(|d| d < 25.0));
        assert!(open_track_m(40.7190, -73.9935).is_none_or(|d| d > 50.0));
        let mut d = SubwayDetector::default();
        d.on_fix(0, 40.74685, -73.89135, Some(5.0), Some(12.0));
        assert!(d.in_subway(S));
        assert!(!d.in_subway(40 * S));
    }
}

//! Whether the phone is in the subway (on a train, or in a station), from what it senses:
//! the subway's own Wi-Fi, or no GNSS fix with few networks around, as underground.
//! Never from the rider's marks, which only score it (`python/experiments/subway.py`).
//!
//! On the first five rides (2026-10-20 and 21) this is on for 97 to 100% of the time on underground trains,
//! and mostly off on the street; where it's on with a good fix, it's nearly always on a platform or in a station.
//! Elevated lines keep GNSS and see street Wi-Fi, so it misses them (61% on the 7), which matters less:
//! there, GNSS keeps the speed from running away.

/// The subway's Wi-Fi (Boldyn, formerly Transit Wireless, and the MTA's own), by exact SSID or prefix.
/// Not `BoldynPasspoint`: despite the name it's outdoors, up to -31 dBm on the street and never seen on a train.
const SUBWAY_SSIDS: &[&str] = &["BoldynWiFi", "PasspointBoldyn", "NYCTA", "FINDSystem"];
const SUBWAY_SSID_PREFIXES: &[&str] = &["OFMTA", "TW_Survey_"];

/// Subway Wi-Fi this recent means still in the subway (scans come every few seconds to a minute).
const SUBWAY_WIFI_HOLD_NS: i64 = 60_000_000_000;
/// A fix this recent and this good means GNSS is working, so not underground.
const FIX_HOLD_NS: i64 = 10_000_000_000;
const GOOD_FIX_ACCURACY_M: f64 = 50.0;
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

    pub fn on_fix(&mut self, t_ns: i64, accuracy_m: Option<f64>) {
        if accuracy_m.is_some_and(|a| a < GOOD_FIX_ACCURACY_M) {
            self.last_good_fix_ns = Some(t_ns);
        }
    }

    pub fn in_subway(&self, t_ns: i64) -> bool {
        let recent = |last: Option<i64>, hold: i64| last.is_some_and(|l| t_ns - l <= hold);
        recent(self.last_subway_wifi_ns, SUBWAY_WIFI_HOLD_NS)
            || (!recent(self.last_good_fix_ns, FIX_HOLD_NS)
                && self.last_scan_networks.is_some_and(|n| n < FEW_NETWORKS))
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
        d.on_fix(0, Some(5.0));
        d.on_wifi_scan(0, street.iter().map(String::as_str));
        assert!(!d.in_subway(S));
        d.on_wifi_scan(
            10 * S,
            ["BoldynWiFi"]
                .into_iter()
                .chain(street.iter().map(String::as_str)),
        );
        d.on_fix(10 * S, Some(5.0));
        assert!(d.in_subway(10 * S));
        assert!(d.in_subway(70 * S));
        d.on_fix(80 * S, Some(5.0));
        assert!(!d.in_subway(80 * S));
    }

    #[test]
    fn in_without_a_fix_and_few_networks() {
        let mut d = SubwayDetector::default();
        // No scans yet: no fix alone isn't enough (Wi-Fi scanning may be off).
        assert!(!d.in_subway(0));
        d.on_wifi_scan(0, ["A", "B"]);
        assert!(d.in_subway(S));
        d.on_fix(2 * S, Some(8.0));
        assert!(!d.in_subway(3 * S));
        // A poor fix doesn't count.
        assert!(d.in_subway(13 * S));
        d.on_fix(14 * S, Some(500.0));
        assert!(d.in_subway(14 * S));
    }
}

//! Stations from the registered LTE cell: `data/station_cells.csv`, from `python/experiments/station_cells.py`
//! (the cell registered as the rider tagged each station across rides, with the MTA's position for it).
//! A station keeps its cell across rides while stopped; at speed, tunnel handovers can shift it by a station.

use std::collections::HashMap;
use std::sync::OnceLock;

const TABLE: &str = include_str!("../data/station_cells.csv");
/// Every station platform the MTA lists (`data/mta/stations.csv` from data.ny.gov): name, daytime routes, position.
const ALL: &str = include_str!("../data/stations.csv");

#[derive(Clone, Debug, PartialEq)]
pub struct Station {
    pub name: String,
    /// The lines the rider was on when they tagged it there.
    pub lines: Vec<String>,
    pub lat: f64,
    pub lon: f64,
}

fn table() -> &'static HashMap<i64, Station> {
    static TABLE_BY_CELL: OnceLock<HashMap<i64, Station>> = OnceLock::new();
    TABLE_BY_CELL.get_or_init(|| {
        TABLE
            .lines()
            .skip(1)
            .filter_map(|line| {
                let f: Vec<&str> = line.split(',').collect();
                let [cell, name, lines, lat, lon] = f[..] else {
                    return None;
                };
                Some((
                    cell.parse().ok()?,
                    Station {
                        name: name.to_string(),
                        lines: lines.split_whitespace().map(str::to_uppercase).collect(),
                        lat: lat.parse().ok()?,
                        lon: lon.parse().ok()?,
                    },
                ))
            })
            .collect()
    })
}

/// The distance (m) between two points (degrees), flat over a city's scale.
pub fn meters(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dn = (a.0 - b.0).to_radians() * 6_371_000.0;
    let de = (a.1 - b.1).to_radians() * 6_371_000.0 * a.0.to_radians().cos();
    dn.hypot(de)
}

/// A car model: whether its door chime is the NTT one (else the SMEE fleet's: R46, R62, R62A, R68, R68A), and whether its cars have the
/// Bluetooth beacons of [`crate::car_beacons`] (CBTC-equipped or ready cars); none when not known.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CarModel {
    pub name: &'static str,
    pub ntt: Option<bool>,
    pub beacons: Option<bool>,
}

const fn model(name: &'static str, ntt: Option<bool>, beacons: Option<bool>) -> CarModel {
    CarModel { name, ntt, beacons }
}

const R62: CarModel = model("R62", Some(false), Some(false));
const R62A: CarModel = model("R62A", Some(false), Some(false));
const R142: CarModel = model("R142", Some(true), Some(false));
const R188: CarModel = model("R188", Some(true), Some(true));
const R160: CarModel = model("R160", Some(true), Some(true));
const R143: CarModel = model("R143", Some(true), Some(true));
const R211: CarModel = model("R211", Some(true), Some(true));
/// Not NTT-chimed; whether they have beacons isn't known.
const R68: CarModel = model("R68", Some(false), None);

/// The car models a line runs, as the rider knows them; none for a line whose fleet isn't known.
pub fn car_models(line: &str) -> Option<&'static [CarModel]> {
    Some(match line {
        "1" => &[R62A],
        "3" => &[R62],
        "2" => &[R142],
        "7" => &[R188],
        "F" | "M" | "R" => &[R160],
        "L" => &[R143, R160],
        // Mostly R211s, with a few R179s.
        "C" => &[R211],
        "B" => &[R68, R211],
        _ => return None,
    })
}

/// What the cars are, as far as known: NTT-chimed or not, and the model.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CarType {
    pub ntt: Option<bool>,
    pub model: Option<&'static str>,
}

/// The car type from what the phone detects, the door chime's family (`ntt`) and whether car beacons are
/// heard, narrowed by the lines the train could be (its route, else the station's).
/// A line whose fleet isn't known could be the one ridden, so then no model is named.
/// Beacons not heard say nothing: the phone may not be scanning.
pub fn car_type(ntt: Option<bool>, beacons: bool, lines: &[String]) -> Option<CarType> {
    if ntt.is_none() && !beacons {
        return None;
    }
    let fits = |m: &CarModel| {
        ntt.is_none_or(|n| m.ntt != Some(!n)) && (!beacons || m.beacons != Some(false))
    };
    let known: Option<Vec<&[CarModel]>> = lines.iter().map(|l| car_models(l)).collect();
    let fleet: Vec<&CarModel> = known
        .filter(|_| !lines.is_empty())
        .unwrap_or_default()
        .into_iter()
        .flatten()
        .collect();
    let mut models: Vec<&CarModel> = fleet.iter().copied().filter(|m| fits(m)).collect();
    // A chime no car of the lines' fleets makes is another train's (a 2 across the platform from the 1):
    // then the fleets say what the cars are.
    if models.is_empty() && !fleet.is_empty() && ntt.is_some() {
        return car_type(None, beacons, lines);
    }
    models.sort_by_key(|m| m.name);
    models.dedup();
    let agreed = |f: fn(&CarModel) -> Option<bool>| {
        let first = f(models.first()?);
        models
            .iter()
            .all(|m| f(m) == first)
            .then_some(first)
            .flatten()
    };
    Some(CarType {
        ntt: ntt.or_else(|| agreed(|m| m.ntt)),
        model: match models[..] {
            [m] => Some(m.name),
            _ => None,
        },
    })
}

pub(crate) fn all() -> &'static [Station] {
    static ALL_STATIONS: OnceLock<Vec<Station>> = OnceLock::new();
    ALL_STATIONS.get_or_init(|| {
        ALL.lines()
            .skip(1)
            .filter_map(|line| {
                let f: Vec<&str> = line.split(',').collect();
                let [name, lines, lat, lon] = f[..] else {
                    return None;
                };
                Some(Station {
                    name: name.to_string(),
                    lines: lines.split_whitespace().map(str::to_string).collect(),
                    lat: lat.parse().ok()?,
                    lon: lon.parse().ok()?,
                })
            })
            .collect()
    })
}

/// The station nearest a point, and how far it is (m).
pub fn nearest(lat: f64, lon: f64) -> Option<(&'static Station, f64)> {
    all()
        .iter()
        .map(|s| (s, meters((lat, lon), (s.lat, s.lon))))
        .min_by(|a, b| a.1.total_cmp(&b.1))
}

/// The part of `station`'s complex the rider's `lines` stop at: itself if it serves one of them (or `lines` is empty),
/// else the nearest station within [`COMPLEX_M`] that does (59 St-Columbus Circle's 1 platforms for a rider on the 1,
/// not its A/B/C/D ones), else itself.
pub fn on_lines(station: &'static Station, lines: &[String]) -> &'static Station {
    let serves = |s: &Station| s.lines.iter().any(|l| lines.contains(l));
    if lines.is_empty() || serves(station) {
        return station;
    }
    all()
        .iter()
        .filter(|s| serves(s))
        .map(|s| (s, meters((station.lat, station.lon), (s.lat, s.lon))))
        .filter(|&(_, d)| d <= COMPLEX_M)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map_or(station, |(s, _)| s)
}

/// Every line of a station's complex: what the rider may take next from it.
pub fn complex_lines(station: &Station) -> Vec<String> {
    let mut lines: Vec<String> = all()
        .iter()
        .filter(|s| meters((station.lat, station.lon), (s.lat, s.lon)) <= COMPLEX_M)
        .flat_map(|s| s.lines.iter().cloned())
        .chain(station.lines.iter().cloned())
        .collect();
    lines.sort();
    lines.dedup();
    lines
}

/// Whether `line` stops at `station` or another part of its complex.
pub fn serves(station: &Station, line: &str) -> bool {
    station.lines.iter().any(|l| l == line) || complex_lines(station).iter().any(|l| l == line)
}

/// How far apart (m) the parts of one station complex can be (Times Sq to 42 St-Port Authority is about 250).
const COMPLEX_M: f64 = 350.0;

/// Whether two stations are of one complex (within [`COMPLEX_M`]): a train standing at one may have either named.
pub fn same_complex(a: &Station, b: &Station) -> bool {
    meters((a.lat, a.lon), (b.lat, b.lon)) <= COMPLEX_M
}

/// The station a registered LTE cell serves, if it's one the rider has tagged, by the MTA's name for it.
pub fn of_cell(cell_id: i64) -> Option<&'static Station> {
    let tagged = table().get(&cell_id)?;
    Some(nearest(tagged.lat, tagged.lon).map_or(tagged, |(s, _)| s))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_complex_is_named_for_the_line_ridden() {
        let abcd = all()
            .iter()
            .find(|s| s.name == "59 St-Columbus Circle" && s.lines.iter().any(|l| l == "A"))
            .unwrap();
        let on_2 = on_lines(abcd, &["2".into(), "1".into()]);
        assert_eq!(on_2.lines, ["1"]);
        assert!(std::ptr::eq(on_lines(abcd, &["C".into()]), abcd));
        assert!(std::ptr::eq(on_lines(abcd, &[]), abcd));
        // No part of it serves the 7: as found.
        assert!(std::ptr::eq(on_lines(abcd, &["7".into()]), abcd));
    }

    #[test]
    fn known_cells_name_their_stations() {
        let s = of_cell(29646219).expect("110 St on the 1");
        assert_eq!(s.name, "Cathedral Pkwy (110 St)");
        assert_eq!(s.lines, ["1"]);
        assert!((s.lat - 40.804).abs() < 0.001 && (s.lon + 73.967).abs() < 0.001);
        assert!(of_cell(1).is_none());
    }

    #[test]
    fn nearest_station_to_a_point() {
        let (s, d) = nearest(40.7556, -73.9870).unwrap();
        assert!(s.name.starts_with("Times Sq"), "{s:?}");
        assert!(d < 100.0);
    }

    #[test]
    fn a_line_serves_only_its_stations_complexes() {
        let named = |n: &str| all().iter().find(|s| s.name == n).unwrap();
        assert!(!serves(named("Astoria Blvd"), "6"));
        assert!(serves(named("Astoria Blvd"), "N"));
        // 59 St-Columbus Circle's A/B/C/D platforms, and its 1 platform, but no E.
        let columbus = all()
            .iter()
            .find(|s| s.name == "59 St-Columbus Circle" && s.lines.contains(&"A".to_string()))
            .unwrap();
        assert!(serves(columbus, "1"));
        assert!(!serves(columbus, "E"));
    }

    #[test]
    fn a_complex_has_all_its_lines() {
        let l = all()
            .iter()
            .find(|s| s.name == "14 St-Union Sq" && s.lines == ["L"])
            .unwrap();
        assert_eq!(complex_lines(l), ["4", "5", "6", "L", "N", "Q", "R", "W"]);
    }

    #[test]
    fn car_type_from_chime_beacons_and_lines() {
        let lines = |l: &[&str]| l.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        let ct = |ntt, model| Some(CarType { ntt, model });
        // 96 St on the 1, 2, 3: the chime tells the 2 from the 1 and 3, not the 1 from the 3.
        let s96 = lines(&["1", "2", "3"]);
        assert_eq!(
            car_type(Some(true), false, &s96),
            ct(Some(true), Some("R142"))
        );
        assert_eq!(car_type(Some(false), false, &s96), ct(Some(false), None));
        // The route decides.
        assert_eq!(
            car_type(Some(false), false, &lines(&["1"])),
            ct(Some(false), Some("R62A"))
        );
        // Beacons rule out the R142s: on the 2 and 7, the 7's R188s, NTT.
        assert_eq!(
            car_type(None, true, &lines(&["2", "7"])),
            ct(Some(true), Some("R188"))
        );
        // On the B: NTT means R211; the SMEE chime, the R68s.
        let b = lines(&["B"]);
        assert_eq!(
            car_type(Some(true), false, &b),
            ct(Some(true), Some("R211"))
        );
        assert_eq!(
            car_type(Some(false), false, &b),
            ct(Some(false), Some("R68"))
        );
        // Beacons alone on the B: the R68s' aren't known, so either.
        assert_eq!(car_type(None, true, &b), ct(None, None));
        // An NTT chime on the 1 is another train's (a 2 across the platform at 96 St): it says nothing,
        // and nothing else is heard.
        assert_eq!(car_type(Some(true), false, &lines(&["1"])), None);
        // The L: NTT either way, R143s or R160s.
        assert_eq!(car_type(None, true, &lines(&["L"])), ct(Some(true), None));
        // A line whose fleet isn't known: no model.
        assert_eq!(
            car_type(Some(true), false, &lines(&["2", "Q"])),
            ct(Some(true), None)
        );
        assert_eq!(car_type(Some(true), false, &[]), ct(Some(true), None));
        assert_eq!(car_type(None, false, &s96), None);
    }
}

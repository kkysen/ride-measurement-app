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

/// The car model a line runs, as the rider knows them (the chime only tells R62 from NTT).
pub fn car_model(line: &str) -> Option<&'static str> {
    Some(match line {
        "1" => "R62A",
        "3" => "R62",
        "2" => "R142",
        "7" => "R188",
        "F" | "M" | "R" => "R160",
        // Mostly R211s, with a few R179s.
        "C" => "R211",
        _ => return None,
    })
}

fn all() -> &'static [Station] {
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

/// The station a registered LTE cell serves, if it's one the rider has tagged, by the MTA's name for it.
pub fn of_cell(cell_id: i64) -> Option<&'static Station> {
    let tagged = table().get(&cell_id)?;
    Some(nearest(tagged.lat, tagged.lon).map_or(tagged, |(s, _)| s))
}

#[cfg(test)]
mod tests {
    use super::*;

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
}

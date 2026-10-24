// Scratch: at each subway stop at a named station, how far the smoothed position is from the station (m),
// and its stated sigma; and the same for the last GPS fix before it, for scale.
use ride_fusion::replay::replay;
use ride_fusion::stations::meters;
use ride_session::read::Session;
fn main() -> anyhow::Result<()> {
    for dir in std::env::args().skip(1) {
        let r = replay(&Session::open(&dir)?)?;
        let mut errors = Vec::new();
        for s in r.stops.iter().filter(|s| s.in_subway) {
            let Some(station) = s.station else { continue };
            let Some(p) = r
                .positions
                .iter()
                .min_by_key(|p| (p.elapsed_realtime_ns - s.stopped_ns).abs())
            else {
                continue;
            };
            let d = meters((p.lat, p.lon), (station.lat, station.lon));
            errors.push(d);
            println!(
                "{dir} {:<28} {d:6.0} m  sigma {:6.0} m",
                station.name, p.sigma_m
            );
        }
        errors.sort_by(f64::total_cmp);
        if !errors.is_empty() {
            println!(
                "{dir}: {} stops, median {:.0} m, 90th {:.0} m",
                errors.len(),
                errors[errors.len() / 2],
                errors[errors.len() * 9 / 10]
            );
        }
    }
    Ok(())
}

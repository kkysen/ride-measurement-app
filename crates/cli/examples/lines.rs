// Scratch: what the status showed (mode, station, route, car type, car beacon) whenever it changed, with the taps,
// in New York time, to check the line and car shown against the rider's.
use ride_fusion::replay::replay;
use ride_session::marks::marks;
use ride_session::read::Session;
fn main() -> anyhow::Result<()> {
    let a: Vec<String> = std::env::args().collect();
    let s = Session::open(&a[1])?;
    let offset_ms = s.info.start_unix_ms - s.info.start_elapsed_realtime_ns / 1_000_000;
    let clock = |t_ns: i64| {
        // New York is UTC-4 in October.
        let ms = offset_ms + t_ns / 1_000_000 - 4 * 3_600_000;
        let secs = ms.div_euclid(1000).rem_euclid(86_400);
        let (h, m, s) = (secs / 3600, secs / 60 % 60, secs % 60);
        format!(
            "{}:{m:02}:{s:02} {}",
            (h + 11) % 12 + 1,
            if h < 12 { "am" } else { "pm" }
        )
    };
    let mut rows: Vec<(i64, String)> = Vec::new();
    for m in marks(&s)?.into_iter().filter(|m| !m.deleted) {
        rows.push((m.t_ns, format!("  TAP {:?} {}", m.kind, m.text)));
    }
    let r = replay(&s)?;
    let mut last = String::new();
    for (t, st) in &r.statuses {
        let line = format!(
            "{:?} | {} | route {} | cars {} | beacon {} | transfer {:?}",
            st.mode,
            st.station.as_ref().map_or("-".into(), |s| format!(
                "{} ({})",
                s.name,
                s.lines.join(" ")
            )),
            st.route.as_deref().unwrap_or("-"),
            st.car_type.map_or("-".into(), |c| format!(
                "{:?} {}",
                c.ntt,
                c.model.unwrap_or("?")
            )),
            st.car.map_or("-".into(), |c| format!("{c:04x}")),
            st.transfer_s.map(|s| s as i64),
        );
        if line != last {
            rows.push((*t, line.clone()));
            last = line;
        }
    }
    for (stop, ..) in r.stops.iter().map(|x| (x, ())) {
        rows.push((
            stop.stopped_ns,
            format!(
                "  STOP {} dwell {:?} left {}",
                stop.station.map_or("-", |s| s.name.as_str()),
                stop.departed_ns
                    .map(|d| (d - stop.stopped_ns) / 1_000_000_000),
                stop.rider_left()
            ),
        ));
    }
    rows.sort_by_key(|r| r.0);
    for (t, row) in rows {
        println!("{} {row}", clock(t));
    }
    Ok(())
}

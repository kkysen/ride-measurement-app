// Scratch: each ride's runs at train speed on or next to the subway's track with GPS (above ground),
// with the lines whose track they follow, the stations at each end, and the car numbers and notes tapped.
use ride_fusion::stations::{meters, nearest};
use ride_fusion::subway::open_track_m;
use ride_fusion::track::lines_near;
use ride_session::proto::manual_event::Kind;
use ride_session::proto::record::Payload;
use ride_session::read::Session;
use ride_session::track::gps_segments;
use std::collections::BTreeMap;

/// A fix counts at this speed (m/s, 11 mph) or more, within this of a track (m), with this accuracy or better (m).
const MIN_MPS: f32 = 5.0;
const NEAR_M: f64 = 30.0;
const ACCURACY_M: f32 = 25.0;
/// Fixes this far apart still belong to one run (s: a station stop between them), and a run lasts at least this.
const JOIN_S: i64 = 120;
const MIN_RUN_S: i64 = 60;

/// A fix in a run: its time, where, its speed, and the lines near it.
type RunFix = (i64, (f64, f64), f32, Vec<(String, f64)>);

fn main() -> anyhow::Result<()> {
    for dir in std::env::args().skip(1) {
        let s = Session::open(&dir)?;
        let Ok(loc) = s.read_records("location") else {
            continue;
        };
        let fixes: Vec<_> = gps_segments(&loc.chunks).into_iter().flatten().collect();
        let mut taps = Vec::new();
        if let Ok(ev) = s.read_records("events") {
            for r in ev.chunks.iter().flat_map(|c| &c.records) {
                if let Some(Payload::ManualEvent(e)) = &r.payload
                    && matches!(Kind::try_from(e.kind), Ok(Kind::CarNumber | Kind::Note))
                    && !e.text.is_empty()
                {
                    taps.push((r.received_elapsed_realtime_ns, e.text.clone()));
                }
            }
        }
        let t0 = fixes.first().map_or(0, |f| f.elapsed_realtime_ns);
        let mmss = |t: i64| {
            let s = (t - t0) / 1_000_000_000;
            format!("{}:{:02}", s / 60, s % 60)
        };
        // Runs of fast fixes on the track.
        let mut runs: Vec<Vec<RunFix>> = Vec::new();
        for f in &fixes {
            let (Some(v), Some(a)) = (f.speed_meters_per_second, f.horizontal_accuracy_meters)
            else {
                continue;
            };
            if v < MIN_MPS || a > ACCURACY_M {
                continue;
            }
            // Open track only: underground lines' shapes run under streets, where buses go too.
            if open_track_m(f.latitude_degrees, f.longitude_degrees).is_none_or(|d| d > NEAR_M) {
                continue;
            }
            let near = lines_near(f.latitude_degrees, f.longitude_degrees, NEAR_M);
            if near.is_empty() {
                continue;
            }
            let p = (f.latitude_degrees, f.longitude_degrees);
            match runs.last_mut() {
                Some(r)
                    if f.elapsed_realtime_ns - r.last().unwrap().0 <= JOIN_S * 1_000_000_000 =>
                {
                    r.push((f.elapsed_realtime_ns, p, v, near))
                }
                _ => runs.push(vec![(f.elapsed_realtime_ns, p, v, near)]),
            }
        }
        runs.retain(|r| r.last().unwrap().0 - r[0].0 >= MIN_RUN_S * 1_000_000_000);
        if runs.is_empty() {
            continue;
        }
        println!("== {}", s.dir.file_name().unwrap().to_string_lossy());
        for r in &runs {
            let (a, b) = (r[0].0, r.last().unwrap().0);
            // The lines near most of the run's fixes.
            let mut count: BTreeMap<&str, usize> = BTreeMap::new();
            for (.., near) in r {
                for (l, _) in near {
                    *count.entry(l).or_default() += 1;
                }
            }
            let mut lines: Vec<(&str, usize)> = count
                .into_iter()
                .filter(|&(_, n)| n * 10 >= r.len() * 8)
                .collect();
            lines.sort();
            let at = |p: (f64, f64)| {
                nearest(p.0, p.1).map_or("?".into(), |(s, d)| format!("{} ({d:.0} m)", s.name))
            };
            let km: f64 = r.windows(2).map(|w| meters(w[0].1, w[1].1)).sum::<f64>() / 1e3;
            let top = r.iter().map(|x| x.2).fold(0.0, f32::max) / 0.44704;
            let tapped: Vec<&str> = taps
                .iter()
                .filter(|(t, _)| *t >= a - 300_000_000_000 && *t <= b + 60_000_000_000)
                .map(|(_, x)| x.as_str())
                .collect();
            println!(
                "  {} to {} ({}): {} to {}, {km:.1} km, top {top:.0} mph, lines {:?}, taps {:?}",
                mmss(a),
                mmss(b),
                mmss(t0 + b - a),
                at(r[0].1),
                at(r.last().unwrap().1),
                lines.iter().map(|x| x.0).collect::<Vec<_>>(),
                tapped
            );
        }
    }
    Ok(())
}

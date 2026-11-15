// Scratch: scores the walk offs found (`StopRecord::alighted_ns`, the rider leaving the train) against the rider's
// "alighted" taps (`PLAN.md` item 9 of the 2026-10-23 to 24 session).
// - found: a tap with a record walked off from within 20 s before to 45 s after it; its lag (s).
// - missed: a tap with none; printed with the stops, departures and step windows around it.
// - false: a walk off (left, not stepped out) with no tap within that span.
// Only sessions with "alighted" taps are scored.
use ride_fusion::live_speed::SUBWAY_MAX_SPEED_MPS;
use ride_fusion::replay::{Options, replay_with};
use ride_session::marks::marks;
use ride_session::proto::manual_event::Kind;
use ride_session::read::Session;

fn main() -> anyhow::Result<()> {
    let s = Session::open(std::env::args().nth(1).unwrap())?;
    let ms: Vec<_> = marks(&s)?.into_iter().filter(|m| !m.deleted).collect();
    let taps: Vec<i64> = ms
        .iter()
        .filter(|m| m.kind == Kind::Alighted)
        .map(|m| m.t_ns)
        .collect();
    if taps.is_empty() {
        println!("no alighted taps");
        return Ok(());
    }
    let options = Options {
        max_speed_mps: Some(SUBWAY_MAX_SPEED_MPS),
        no_statuses: true,
        ..Options::default()
    };
    let r = replay_with(&s, &options)?;
    let offset_ms = ms.first().map(|m| m.unix_ms - m.t_ns / 1_000_000);
    let clock = |t: i64| match offset_ms {
        Some(o) => {
            let secs = ride_session::clock::to_new_york_ms(t / 1_000_000 + o) / 1000;
            let d = secs.rem_euclid(86400);
            let (h, m, s) = (d / 3600, d / 60 % 60, d % 60);
            let h12 = if h % 12 == 0 { 12 } else { h % 12 };
            format!("{h12}:{m:02}:{s:02} {}", if h < 12 { "am" } else { "pm" })
        }
        None => format!("{t}"),
    };
    let s_ns = |x: f64| (x * 1e9) as i64;
    let sec = |d: i64| d as f64 * 1e-9;
    let walk_offs: Vec<(i64, bool)> = r
        .stops
        .iter()
        .filter_map(|x| x.alighted_ns.map(|a| (a, x.stepped_out())))
        .collect();
    let near = |a: i64, t: i64| a >= t - s_ns(20.0) && a <= t + s_ns(45.0);
    let (mut found, mut lag_sum) = (0, 0.0);
    for &t in &taps {
        match walk_offs.iter().find(|&&(a, _)| near(a, t)) {
            Some(&(a, out)) => {
                found += 1;
                lag_sum += sec(a - t);
                println!(
                    "  found {} lag {:+.0} s{}",
                    clock(t),
                    sec(a - t),
                    if out { " (stepped out)" } else { "" }
                );
            }
            None => {
                println!("  MISSED {}", clock(t));
                for x in r
                    .stops
                    .iter()
                    .filter(|x| (x.stopped_ns - t).abs() < s_ns(120.0))
                {
                    println!(
                        "    stop {:+.0} s subway {} platform {} closed {:?} departed {:?} boarded {:?}",
                        sec(x.stopped_ns - t),
                        x.in_subway,
                        x.on_platform,
                        x.doors_closed_ns.map(|c| (sec(c - t)).round()),
                        x.departed_ns.map(|c| (sec(c - t)).round()),
                        x.boarded_ns.map(|c| (sec(c - t)).round()),
                    );
                }
                let d: Vec<String> = r
                    .departures
                    .iter()
                    .filter(|&&d| (d - t).abs() < s_ns(120.0))
                    .map(|&d| format!("{:+.0}", sec(d - t)))
                    .collect();
                println!("    departures {}", d.join(" "));
                let w: String = r
                    .step_windows
                    .iter()
                    .filter(|&&(w, _)| w > t - s_ns(60.0) && w < t + s_ns(90.0))
                    .map(|&(_, st)| if st { 'S' } else { '.' })
                    .collect();
                println!("    steps -60..+90 s: {w}");
                let m: Vec<String> = ms
                    .iter()
                    .filter(|m| (m.t_ns - t).abs() < s_ns(120.0))
                    .map(|m| format!("{:?}{:+.0}", m.kind, sec(m.t_ns - t)))
                    .collect();
                println!("    taps {}", m.join(" "));
            }
        }
    }
    let mut false_offs = 0;
    for x in &r.stops {
        let Some(a) = x.alighted_ns else { continue };
        if !x.stepped_out() && !taps.iter().any(|&t| near(a, t)) {
            false_offs += 1;
            println!(
                "  FALSE {} stop {:+.0} s subway {} closed {:?} departed {:?} boarded {:?}",
                clock(a),
                sec(x.stopped_ns - a),
                x.in_subway,
                x.doors_closed_ns.map(|c| (sec(c - a)).round()),
                x.departed_ns.map(|c| (sec(c - a)).round()),
                x.boarded_ns.map(|c| (sec(c - a)).round()),
            );
            let m: Vec<String> = ms
                .iter()
                .filter(|m| (m.t_ns - a).abs() < s_ns(120.0))
                .map(|m| format!("{:?}{:+.0}", m.kind, sec(m.t_ns - a)))
                .collect();
            println!("    taps {}", m.join(" "));
            let d: Vec<String> = r
                .departures
                .iter()
                .filter(|&&d| (d - a).abs() < s_ns(120.0))
                .map(|&d| format!("{:+.0}", sec(d - a)))
                .collect();
            println!("    departures {}", d.join(" "));
            let st: Vec<String> = r
                .stops
                .iter()
                .filter(|x| (x.stopped_ns - a).abs() < s_ns(120.0))
                .map(|x| format!("{:+.0}", sec(x.stopped_ns - a)))
                .collect();
            println!("    stops {}", st.join(" "));
            let w: String = r
                .step_windows
                .iter()
                .filter(|&&(w, _)| w > a - s_ns(60.0) && w < a + s_ns(90.0))
                .map(|&(_, st)| if st { 'S' } else { '.' })
                .collect();
            println!("    steps -60..+90 s: {w}");
        }
    }
    println!(
        "walk offs: {found}/{} taps found, mean lag {:+.1} s, {false_offs} false",
        taps.len(),
        if found > 0 {
            lag_sum / found as f64
        } else {
            0.0
        }
    );
    Ok(())
}

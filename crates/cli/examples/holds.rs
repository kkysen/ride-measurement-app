// Scratch: scores the live speed and the stops found underground against the rider's taps
// (`PLAN.md` items 1, 6 and 7 of the 2026-10-23 to 24 session).
// - dwell: from 3 s after a "train stopped" tap to 2 s before its "train started" (or 20 s after, without one):
//   the share of the live speed's seconds under 2 mph, and the stops with any second over 10 mph.
// - between trains: from an "alighted" tap to the next "boarded" (or 10 minutes): seconds over 6 mph.
// - moving: from 10 s after a "train started" tap to 10 s before the next "train stopped", riding:
//   seconds shown stopped (under 1 mph).
// - each stop found (`StopRecord`) in the subway: the nearest "train stopped" and "at station" taps, its station,
//   whether its cell matches the stop before's, and whether a closing was heard.
// `DUMP=1` prints each stop and dwell.
use ride_fusion::live_speed::SUBWAY_MAX_SPEED_MPS;
use ride_fusion::replay::{Options, replay_with};
use ride_session::marks::marks;
use ride_session::proto::manual_event::Kind;
use ride_session::proto::record::Payload;
use ride_session::read::Session;

fn main() -> anyhow::Result<()> {
    let s = Session::open(std::env::args().nth(1).unwrap())?;
    let zone = ride_session::clock::SessionClock::for_session(&s);
    let dump = std::env::var("DUMP").is_ok();
    let ms: Vec<_> = marks(&s)?.into_iter().filter(|m| !m.deleted).collect();
    let options = Options {
        max_speed_mps: Some(SUBWAY_MAX_SPEED_MPS),
        no_statuses: std::env::var("STATUS").is_err(),
        live_every_ns: Some(1_000_000_000),
        ..Options::default()
    };
    let r = replay_with(&s, &options)?;
    // Unix ms of an elapsed time, from the marks, for New York clock times (EDT, UTC-4, until November 1).
    let offset_ms = ms.first().map(|m| m.unix_ms - m.t_ns / 1_000_000);
    let clock = |t: i64| match offset_ms {
        Some(o) => {
            let secs = zone.local_ms(t / 1_000_000 + o) / 1000;
            let d = secs.rem_euclid(86400);
            let (h, m, s) = (d / 3600, d / 60 % 60, d % 60);
            let h12 = if h % 12 == 0 { 12 } else { h % 12 };
            format!("{h12}:{m:02}:{s:02} {}", if h < 12 { "am" } else { "pm" })
        }
        None => format!("{t}"),
    };
    let mph = |v: f64| v / 0.44704;
    let live_in = |a: i64, b: i64| {
        r.live
            .iter()
            .filter(move |(t, _)| *t >= a && *t <= b)
            .map(|(_, e)| mph(e.speed_mps))
    };
    let s_ns = |x: f64| (x * 1e9) as i64;
    let kind_at = |k: Kind| ms.iter().filter(move |m| m.kind == k).map(|m| m.t_ns);
    let next_of = |k: Kind, after: i64, within: f64| {
        kind_at(k).find(|&t| t > after && t - after <= s_ns(within))
    };

    // `TRACE=7:38:15,60`: each live second within 60 s of that clock time (New York, pm assumed past noon).
    if let Ok(trace) = std::env::var("TRACE") {
        let (at, span) = trace.split_once(',').unwrap();
        let span: f64 = span.parse()?;
        for (t, e) in &r.live {
            let c = clock(*t);
            let hms: Vec<i64> = c
                .split(' ')
                .next()
                .unwrap()
                .split(':')
                .map(|x| x.parse().unwrap())
                .collect();
            let want: Vec<i64> = at.split(':').map(|x| x.parse().unwrap()).collect();
            let sec = |v: &[i64]| (v[0] % 12) * 3600 + v[1] * 60 + v[2];
            if ((sec(&hms) - sec(&want)) as f64).abs() <= span {
                println!(
                    "  {c} {t} {:5.1} mph ±{:4.1} stopped {:5} {:?} rejected {:?}",
                    mph(e.speed_mps),
                    mph(e.sigma_mps),
                    e.stopped,
                    e.sensors,
                    e.rejected
                );
            }
        }
    }

    if std::env::var("SUBWAY").is_ok() {
        for (t, on) in &r.subway_changes {
            println!("  subway {on} at {}", clock(*t));
        }
        for d in &r.departures {
            println!("  departure {}", clock(*d));
        }
    }

    if std::env::var("STATUS").is_ok() {
        for (t, st) in &r.statuses {
            println!(
                "  status {} dwell {:?} transfer {:?}",
                clock(*t),
                st.dwell,
                st.transfer_s
            );
        }
    }

    // Dwells.
    let (mut held, mut total, mut fast_stops, mut stops) = (0usize, 0usize, 0usize, 0usize);
    for t in kind_at(Kind::TrainStopped) {
        let end = next_of(Kind::TrainStarted, t, 300.0).map_or(t + s_ns(20.0), |e| e - s_ns(2.0));
        let v: Vec<f64> = live_in(t + s_ns(3.0), end).collect();
        if v.is_empty() {
            continue;
        }
        stops += 1;
        let under = v.iter().filter(|&&x| x < 2.0).count();
        let max = v.iter().copied().fold(0.0, f64::max);
        held += under;
        total += v.len();
        if max > 10.0 {
            fast_stops += 1;
        }
        if dump {
            println!(
                "  dwell {} {:3.0} s: {:3.0}% held, max {:4.1} mph",
                clock(t),
                v.len(),
                100.0 * under as f64 / v.len() as f64,
                max
            );
        }
    }

    // Between trains.
    let mut between_fast = 0usize;
    let mut between = 0usize;
    for t in kind_at(Kind::Alighted) {
        let end = next_of(Kind::Boarded, t, 600.0).unwrap_or(t + s_ns(600.0));
        let v: Vec<f64> = live_in(t, end).collect();
        between += v.len();
        let fast = v.iter().filter(|&&x| x > 6.0).count();
        between_fast += fast;
        if dump && fast > 0 {
            println!(
                "  between trains {} {:3.0} s: {} s over 6 mph, max {:4.1}",
                clock(t),
                v.len(),
                fast,
                v.iter().copied().fold(0.0, f64::max)
            );
        }
    }

    // Moving.
    let mut moving = 0usize;
    let mut moving_stopped = 0usize;
    for t in kind_at(Kind::TrainStarted) {
        let Some(next) = next_of(Kind::TrainStopped, t, 600.0) else {
            continue;
        };
        let v: Vec<f64> = live_in(t + s_ns(10.0), next - s_ns(10.0)).collect();
        moving += v.len();
        let st = v.iter().filter(|&&x| x < 1.0).count();
        moving_stopped += st;
        if dump && st > 0 {
            println!(
                "  moving {} {:3.0} s: {} s shown stopped",
                clock(t),
                v.len(),
                st
            );
        }
    }

    // The registered LTE cell over time, for the stops' cues.
    let mut cells: Vec<(i64, Option<i64>)> = Vec::new();
    if s.stream_path("radio").exists() {
        for rec in s
            .read_records("radio")?
            .chunks
            .into_iter()
            .flat_map(|c| c.records)
        {
            if let Some(Payload::CellInfo(c)) = rec.payload {
                cells.push((
                    rec.received_elapsed_realtime_ns,
                    c.cells
                        .iter()
                        .find(|x| x.registered && x.kind == "lte")
                        .and_then(|x| x.cell_id),
                ));
            }
        }
    }
    let cell_at = |t: i64| {
        cells
            .iter()
            .rev()
            .find(|(c, _)| *c <= t)
            .and_then(|(_, c)| *c)
    };

    // Stops found, against the taps: one within 15 s of a "train stopped" tap is a tapped stop;
    // with an "at station" tap within 60 s before it (pulling in), at a platform; else a hold between stations.
    let near = |k: Kind, t: i64, before: f64, after: f64| {
        kind_at(k).find(|&m| m >= t - s_ns(before) && m <= t + s_ns(after))
    };
    let tapped_any = !kind_at(Kind::TrainStopped).collect::<Vec<_>>().is_empty();
    let mut prev_cell: Option<i64> = None;
    let (mut found, mut matched, mut untapped) = (0, 0, 0);
    let (mut between_tapped, mut between_untapped) = (0, 0);
    for rec in r.stops.iter().filter(|x| x.in_subway) {
        found += 1;
        let tap = near(Kind::TrainStopped, rec.stopped_ns, 15.0, 15.0);
        let at_station = near(Kind::AtStation, rec.stopped_ns, 60.0, 15.0);
        if rec.between_stations {
            if tap.is_some() {
                between_tapped += 1;
            } else {
                between_untapped += 1;
            }
        }
        if tap.is_some() {
            matched += 1;
        } else {
            untapped += 1;
        }
        let cell = cell_at(rec.stopped_ns);
        let same = cell.is_some() && cell == prev_cell;
        prev_cell = cell;
        if dump && tapped_any {
            println!(
                "  stop {} tap {:>5} at-station {:>5} station {:<28} cell {:>10} same {:5} doors {:5} dwell {} between {}",
                clock(rec.stopped_ns),
                tap.map_or("-".into(), |m| format!(
                    "{:+.0}",
                    (m - rec.stopped_ns) as f64 * 1e-9
                )),
                at_station.map_or("-".into(), |m| format!(
                    "{:+.0}",
                    (m - rec.stopped_ns) as f64 * 1e-9
                )),
                rec.station.map_or("-", |x| &x.name),
                cell.map_or("-".into(), |c| c.to_string()),
                same,
                rec.doors_closed_ns.is_some(),
                rec.dwell_s().map_or("-".into(), |d| format!(
                    "{}:{:02}",
                    d as i64 / 60,
                    d as i64 % 60
                )),
                rec.between_stations,
            );
        }
    }
    let tapped_stops = kind_at(Kind::TrainStopped).count();
    let missed = kind_at(Kind::TrainStopped)
        .filter(|&t| {
            !r.stops
                .iter()
                .any(|x| x.in_subway && (x.stopped_ns - t).abs() <= s_ns(15.0))
        })
        .count();
    println!(
        "dwell held {held}/{total} s in {stops} tapped stops, {fast_stops} over 10 mph; \
         between trains {between_fast}/{between} s over 6 mph; moving shown stopped {moving_stopped}/{moving} s; \
         subway stops found {found}: {matched} tapped, {untapped} not; tapped stops {tapped_stops}, {missed} not found; between stations {between_tapped} tapped, {between_untapped} not"
    );
    Ok(())
}

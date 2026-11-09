// Stations passed without stopping (`Fusion::passes`), replayed live with `skip_still`, in New York time:
// each pass with the lowest live speed while named, then the status's route and lines whenever they change,
// with the taps, to check no local's stop is taken for a pass.
use ride_fusion::replay::{Options, replay_with};
use ride_session::marks::marks;
use ride_session::read::Session;
fn main() -> anyhow::Result<()> {
    let a: Vec<String> = std::env::args().collect();
    let s = Session::open(&a[1])?;
    let offset_ms = s.info.start_unix_ms - s.info.start_elapsed_realtime_ns / 1_000_000;
    let clock = |t_ns: i64| {
        let unix_ms = offset_ms + t_ns / 1_000_000;
        // New York: UTC-4 from 2026-03-08 to 2026-11-01 (7 am and 6 am UTC), else UTC-5.
        let dst = (1_772_953_200_000..1_793_512_800_000).contains(&unix_ms);
        let ms = unix_ms - if dst { 4 } else { 5 } * 3_600_000;
        let secs = ms.div_euclid(1000).rem_euclid(86_400);
        let (h, m, s) = (secs / 3600, secs / 60 % 60, secs % 60);
        format!(
            "{}:{m:02}:{s:02} {}",
            (h + 11) % 12 + 1,
            if h < 12 { "am" } else { "pm" }
        )
    };
    let r = replay_with(
        &s,
        &Options {
            live: true,
            skip_still: true,
            no_live_smoothing: true,
            ..Default::default()
        },
    )?;
    let name = a[1].trim_end_matches('/').rsplit('/').next().unwrap_or("");
    let taps: Vec<_> = marks(&s)?.into_iter().filter(|m| !m.deleted).collect();
    let s_ns = 1_000_000_000;
    for p in &r.passes {
        // The route shown in the 10 minutes before: a pass of one of its stations is a change of train,
        // or a local ruled out.
        let shown = r
            .statuses
            .iter()
            .rev()
            .filter(|(t, _)| *t < p.first_ns && *t > p.first_ns - 600 * s_ns)
            .find(|(_, st)| st.route.is_some())
            .and_then(|(_, st)| st.route.clone())
            .unwrap_or_default();
        println!(
            "{name} PASS {} to {} {} ({}) min {:.0} mph, route before {}{}",
            clock(p.first_ns),
            clock(p.last_ns),
            p.station.name,
            p.station.lines.join(" "),
            p.min_mps as f64 * 2.23694,
            if shown.is_empty() { "-" } else { &shown },
            if p.station.lines.contains(&shown) {
                " RULED OUT"
            } else {
                ""
            }
        );
        // The rider's taps then: a stop or the doors tapped while "passing" is a stop taken for a pass.
        for m in taps
            .iter()
            .filter(|m| (p.first_ns - 30 * s_ns..=p.last_ns + 30 * s_ns).contains(&m.t_ns))
        {
            println!("{name}     TAP {} {:?} {}", clock(m.t_ns), m.kind, m.text);
        }
    }
    if std::env::var("STATUS").is_err() {
        return Ok(());
    }
    let mut rows: Vec<(i64, String)> = Vec::new();
    for m in taps {
        rows.push((m.t_ns, format!("  TAP {:?} {}", m.kind, m.text)));
    }
    for &(first, last, st) in &r.named {
        rows.push((
            first,
            format!(
                "  NAMED {} ({}) to {}",
                st.name,
                st.lines.join(" "),
                clock(last)
            ),
        ));
    }
    let mut prev = String::new();
    for (t, st) in &r.statuses {
        let line = format!(
            "{:?} | {} | route {} | lines {}",
            st.mode,
            st.station.as_ref().map_or("-".into(), |s| s.name.clone()),
            st.route.as_deref().unwrap_or("-"),
            st.lines.join(" ")
        );
        if line != prev {
            rows.push((*t, line.clone()));
            prev = line;
        }
    }
    rows.sort_by_key(|r| r.0);
    for (t, row) in rows {
        println!("{} {row}", clock(t));
    }
    Ok(())
}

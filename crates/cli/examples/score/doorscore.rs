// Scratch: door cycles heard against the rider's taps.
// A cycle is "tapped" if a doors-open, doors-closed, "train stopped" or "at station" tap falls within it
// (from 15 s before the opening to 10 s after the closing); otherwise "untapped" (false, or an untapped stop).
// Also counts doors-closed taps with a cycle closing within 10 s. `DUMP=1` prints each cycle with
// the stopping lurch before it, any start surge inside it, and the live speed before the opening.
use ride_fusion::replay::Replay;
use ride_session::marks::marks;
use ride_session::proto::SensorChunk;
use ride_session::proto::manual_event::Kind;
use ride_session::read::Session;

pub fn run(s: &Session, r: &Replay, out: &mut String) -> anyhow::Result<()> {
    let dump = std::env::var("DUMP").is_ok();
    let ms: Vec<_> = marks(s)?.into_iter().filter(|m| !m.deleted).collect();
    if !ms
        .iter()
        .any(|m| m.kind == Kind::DoorsClosed || m.kind == Kind::TrainStopped)
    {
        out!(out, "doors: no door taps");
        return Ok(());
    }
    // Vibration: the std of |a| over each 1 s, by second.
    let mut acc: Vec<(i64, f64)> = Vec::new();
    if dump {
        for c in s.stream_chunks::<SensorChunk>("accel_uncal")? {
            let c = c?;
            let n = c.values_per_sample as usize;
            let v: Vec<f32> = c.values().collect();
            for (i, t) in c.timestamps_ns().into_iter().enumerate() {
                let a = &v[i * n..i * n + 3];
                acc.push((
                    t,
                    a.iter().map(|x| f64::from(*x).powi(2)).sum::<f64>().sqrt(),
                ));
            }
        }
    }
    let vib = |from: i64, to: i64| {
        let i = acc.partition_point(|(t, _)| *t < from);
        let j = acc.partition_point(|(t, _)| *t < to);
        let mut stds: Vec<f64> = acc[i..j]
            .chunk_by(|a, b| a.0 / 1_000_000_000 == b.0 / 1_000_000_000)
            .filter(|w| w.len() > 5)
            .map(|w| {
                let m = w.iter().map(|x| x.1).sum::<f64>() / w.len() as f64;
                (w.iter().map(|x| (x.1 - m).powi(2)).sum::<f64>() / w.len() as f64).sqrt()
            })
            .collect();
        stds.sort_by(f64::total_cmp);
        stds.get(stds.len() / 2).copied().unwrap_or(f64::NAN)
    };
    let offset_ms = ms.first().map(|m| m.unix_ms - m.t_ns / 1_000_000).unwrap();
    let clock = |t: i64| {
        let d = ((t / 1_000_000 + offset_ms) / 1000 - 4 * 3600).rem_euclid(86400);
        let (h, m, s) = (d / 3600, d / 60 % 60, d % 60);
        let h12 = if h % 12 == 0 { 12 } else { h % 12 };
        format!("{h12}:{m:02}:{s:02} {}", if h < 12 { "am" } else { "pm" })
    };
    let ns = |x: f64| (x * 1e9) as i64;
    let mph = |v: f64| v / 0.44704;
    let live_at = |t: i64| {
        r.live
            .iter()
            .min_by_key(|(lt, _)| (lt - t).abs())
            .map(|(_, e)| mph(e.speed_mps))
            .unwrap_or(f64::NAN)
    };
    let (mut tapped, mut untapped) = (0, 0);
    for c in &r.door_cycles {
        let hit = ms.iter().any(|m| {
            matches!(
                m.kind,
                Kind::DoorsOpen | Kind::DoorsClosed | Kind::TrainStopped | Kind::AtStation
            ) && m.t_ns >= c.open_ns - ns(15.0)
                && m.t_ns <= c.close_ns + ns(10.0)
        });
        if hit {
            tapped += 1;
        } else {
            untapped += 1;
        }
        if dump {
            let lurch = r
                .stops
                .iter()
                .filter(|st| st.stopped_ns <= c.close_ns && st.stopped_ns >= c.open_ns - ns(60.0))
                .map(|st| (c.open_ns - st.stopped_ns) as f64 * 1e-9)
                .next();
            let surge = r
                .departures
                .iter()
                .any(|&d| d > c.open_ns && d < c.close_ns);
            out!(
                out,
                "  cycle {} to {} ({:.0} s) {} lurch {:?} surge-inside {} live before {:.0} mph vib before {:.3} during {:.3} min5 {:.3}",
                clock(c.open_ns),
                clock(c.close_ns),
                (c.close_ns - c.open_ns) as f64 * 1e-9,
                if hit { "tapped" } else { "UNTAPPED" },
                lurch.map(|x| x.round()),
                surge,
                live_at(c.open_ns - ns(3.0)),
                vib(c.open_ns - ns(30.0), c.open_ns - ns(5.0)),
                vib(c.open_ns, c.close_ns),
                (0..((c.close_ns - c.open_ns) / ns(5.0)))
                    .map(|k| vib(c.open_ns + k * ns(5.0), c.open_ns + (k + 1) * ns(5.0)))
                    .fold(f64::INFINITY, f64::min),
            );
        }
    }
    let closes: Vec<_> = ms.iter().filter(|m| m.kind == Kind::DoorsClosed).collect();
    let found = closes
        .iter()
        .filter(|m| {
            r.door_cycles
                .iter()
                .any(|c| (c.close_ns - m.t_ns).abs() < ns(10.0))
        })
        .count();
    out!(
        out,
        "doors: cycles {} tapped {tapped} untapped {untapped}; closed taps {} with a cycle {found}",
        r.door_cycles.len(),
        closes.len()
    );
    Ok(())
}

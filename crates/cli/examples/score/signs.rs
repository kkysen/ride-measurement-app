// Scratch: whether the live speed's travel axis points the right way at tapped stops and starts.
// The live speed is |v| along the axis, so with the axis backward, braking raises it and motoring lowers it.
// - before each "train stopped" tap: the live speed 10 s before against 1 s before; it should fall.
// - after each "train started" tap: 1 s after against 10 s after; it should rise.
// Each counted as falls, rises, or flat (within 1 mph). `DUMP=1` prints each.
use ride_fusion::replay::Replay;
use ride_session::marks::marks;
use ride_session::proto::manual_event::Kind;
use ride_session::read::Session;

pub fn run(s: &Session, r: &Replay, out: &mut String) -> anyhow::Result<()> {
    let dump = std::env::var("DUMP").is_ok();
    let ms: Vec<_> = marks(s)?.into_iter().filter(|m| !m.deleted).collect();
    if !ms.iter().any(|m| m.kind == Kind::TrainStopped) {
        out!(out, "signs: no stop taps");
        return Ok(());
    }
    let offset_ms = ms.first().map(|m| m.unix_ms - m.t_ns / 1_000_000).unwrap();
    let clock = |t: i64| {
        let d = ((t / 1_000_000 + offset_ms) / 1000 - 4 * 3600).rem_euclid(86400);
        let (h, m, s) = (d / 3600, d / 60 % 60, d % 60);
        let h12 = if h % 12 == 0 { 12 } else { h % 12 };
        format!("{h12}:{m:02}:{s:02} {}", if h < 12 { "am" } else { "pm" })
    };
    let mph = |v: f64| v / 0.44704;
    let at = |t: i64| {
        r.live
            .iter()
            .min_by_key(|(lt, _)| (lt - t).abs())
            .filter(|(lt, _)| (lt - t).abs() < 1_500_000_000)
            .map(|(_, e)| mph(e.speed_mps))
    };
    let s_ns = |x: f64| (x * 1e9) as i64;
    for (kind, name, a, b, want_rise) in [
        (Kind::TrainStopped, "braking", -10.0, -1.0, false),
        (Kind::TrainStarted, "starting", 1.0, 10.0, true),
    ] {
        let (mut right, mut wrong, mut flat) = (0, 0, 0);
        for m in ms.iter().filter(|m| m.kind == kind) {
            let (Some(va), Some(vb)) = (at(m.t_ns + s_ns(a)), at(m.t_ns + s_ns(b))) else {
                continue;
            };
            let d = vb - va;
            let verdict = if d.abs() < 1.0 {
                flat += 1;
                "flat"
            } else if (d > 0.0) == want_rise {
                right += 1;
                "right"
            } else {
                wrong += 1;
                "WRONG"
            };
            if dump {
                out!(
                    out,
                    "  {name} {} {} {va:5.1} -> {vb:5.1} mph {verdict}",
                    clock(m.t_ns),
                    m.t_ns
                );
            }
        }
        out!(out, "signs {name}: right {right} wrong {wrong} flat {flat}");
    }
    Ok(())
}

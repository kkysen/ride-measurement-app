// The cost of what the app does every status and every smoothing tick, near a ride's start and near its end,
// replayed as the phone does (`Fusion::new`): a cost that grows with the ride rescans it, and shows here
// as the last 10 minutes costing more than the first. Flags any over 3x.
use ride_fusion::replay::Replay;

const MIN: f64 = 10.0;
const GROWTH: f64 = 3.0;

pub fn run(r: &Replay, out: &mut String) -> anyhow::Result<()> {
    let (Some(first), Some(last)) = (r.statuses.first(), r.statuses.last()) else {
        out!(out, "cost: no statuses");
        return Ok(());
    };
    let (t0, t1) = (first.0, last.0);
    let span = (t1 - t0) as f64 * 1e-9;
    let w = MIN * 60.0;
    // Mean us over the first and last `MIN` minutes.
    let ends = |v: &mut dyn Iterator<Item = (i64, u64)>| {
        let (mut a, mut b) = ((0u64, 0u64), (0u64, 0u64));
        for (t, c) in v {
            let x = (t - t0) as f64 * 1e-9;
            if x <= w {
                a = (a.0 + c, a.1 + 1);
            }
            if x >= span - w {
                b = (b.0 + c, b.1 + 1);
            }
        }
        let mean = |(s, n): (u64, u64)| s as f64 / n.max(1) as f64 * 1e-3;
        (mean(a), mean(b))
    };
    let times = || r.statuses.iter().map(|s| s.0);
    let rows = [
        (
            "status",
            ends(&mut times().zip(r.status_cost_ns.iter().copied())),
        ),
        (
            "feed per status",
            ends(&mut times().zip(r.feed_cost_ns.iter().copied())),
        ),
        ("tick", ends(&mut r.tick_cost_ns.iter().copied())),
    ];
    let mut flagged = Vec::new();
    for (what, (a, b)) in rows {
        let growth = b / a.max(1e-3);
        out!(
            out,
            "cost {what}: first {MIN} min {a:.0} us, last {b:.0} us, {growth:.1}x"
        );
        if growth > GROWTH {
            flagged.push(format!("{what} {growth:.1}x"));
        }
    }
    out!(
        out,
        "cost: over {:.0} min, {}",
        span / 60.0,
        if flagged.is_empty() {
            "nothing grew over 3x".to_string()
        } else {
            format!("GREW {}", flagged.join(", "))
        }
    );
    Ok(())
}

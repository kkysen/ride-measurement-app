// Scratch (item 28): the trains' acceleration curves from GNSS on open track, per line and car model.
// For each labeled run (session, minutes from the first fix, line, car model, grade), the GNSS speeds
// (accuracy 25 m or better, a speed accuracy of 1 m/s or better) are split at stops into legs;
// in each leg the acceleration is the change in speed over 6 to 8 s between fixes (shorter is GNSS noise).
// Printed per run: each leg's top speed, the time from a stop to 20 and 30 mph, and the acceleration
// while speeding up and braking by speed (5 mph bins: median and 90th percentile, m/s²).
// Then the same pooled per car model. `DUMP=1` prints every fix's speed and acceleration;
// `CSV=path` writes every sample (run, model, grade, speed, acceleration, and its sigma from the fixes' speed
// accuracies) for `data/work/track/fit_curves.py`.
use ride_session::read::Session;
use ride_session::track::gps_segments;
use std::collections::BTreeMap;
use std::io::Write;

const MPH: f64 = 0.44704;
const ACCURACY_M: f32 = 25.0;
const SPEED_ACCURACY_MPS: f32 = 1.5;
/// Under this (m/s) is stopped; a leg is from one stop to the next.
const STOPPED_MPS: f64 = 0.5;
/// Speeding up or braking at least this (m/s²); between is cruising.
const MOVING_MPS2: f64 = 0.1;
const BIN_MPH: f64 = 5.0;

/// The runs, from `crates/cli/examples/aboveground.rs` over every session, labeled from the rider's notes
/// and car numbers (models from Wikipedia's fleet listing, 2026-11-08): session, start and end (minutes
/// from the first fix), line, car model, and whether a steep grade (bridge) is in it.
const RUNS: &[(&str, f64, f64, &str, &str, bool)] = &[
    ("20261020-154951", 0.0, 2.6, "7", "R188", false),
    ("20261020-154951", 4.7, 21.0, "7", "R188", false),
    ("20261020-184134", 16.4, 26.8, "7", "R188", false),
    ("20261024-220437", 5.2, 8.4, "D/N/Q (car 2718)", "R68", true),
    ("20261027-220431", 0.0, 14.9, "5", "R142", false),
    ("20261027-220431", 20.7, 25.9, "2 express", "R142", false),
    ("20261027-220431", 51.1, 67.9, "4", "R142", false),
    ("20261031-215706", 16.8, 19.3, "1", "R62A", false),
    (
        "20261103-180153",
        2.8,
        10.8,
        "W (car 5546, noted wrong)",
        "R46?",
        false,
    ),
    ("20261103-185919", 6.5, 8.5, "1 (car 2387)", "R62A", false),
    ("20261104-030354", 3.5, 5.0, "1", "R62A", false),
    ("20261105-220621", 0.6, 4.3, "M (car 8582)", "R160", false),
    (
        "20261105-220621",
        6.7,
        18.0,
        "M, Williamsburg Bridge",
        "R160",
        true,
    ),
    ("20261107-175954", 3.5, 11.7, "N (car 5078)", "R68A", false),
];

#[derive(Default)]
struct Bins {
    up: BTreeMap<i64, Vec<f64>>,
    down: BTreeMap<i64, Vec<f64>>,
}

impl Bins {
    fn add(&mut self, v: f64, a: f64) {
        let bin = (v / MPH / BIN_MPH) as i64;
        if a >= MOVING_MPS2 {
            self.up.entry(bin).or_default().push(a);
        } else if a <= -MOVING_MPS2 {
            self.down.entry(bin).or_default().push(-a);
        }
    }

    fn print(&self) {
        let q = |v: &mut Vec<f64>, p: f64| {
            v.sort_by(f64::total_cmp);
            v[((v.len() - 1) as f64 * p).round() as usize]
        };
        for (name, m) in [("speeding up", &self.up), ("braking", &self.down)] {
            let cells: Vec<String> = m
                .iter()
                .filter(|(_, v)| v.len() >= 3)
                .map(|(b, v)| {
                    let mut v = v.clone();
                    format!(
                        "{}-{}: {:.2}/{:.2} (n {})",
                        b * BIN_MPH as i64,
                        (b + 1) * BIN_MPH as i64,
                        q(&mut v, 0.5),
                        q(&mut v, 0.9),
                        v.len()
                    )
                })
                .collect();
            println!("    {name} (mph: median/90th m/s²): {}", cells.join(", "));
        }
    }
}

fn mmss(s: f64) -> String {
    let s = s.round() as i64;
    format!("{}:{:02}", s / 60, s % 60)
}

fn main() -> anyhow::Result<()> {
    let dump = std::env::var("DUMP").is_ok();
    let mut csv = match std::env::var("CSV") {
        Ok(p) => {
            let mut f = std::fs::File::create(p)?;
            writeln!(f, "run,t,model,grade,v_mps,a_mps2,sigma_mps2")?;
            Some(f)
        }
        Err(_) => None,
    };
    let mut by_model: BTreeMap<(&str, bool), Bins> = BTreeMap::new();
    for &(session, from_min, to_min, line, model, grade) in RUNS {
        let s = Session::open(format!("data/sessions/{session}"))?;
        let fixes: Vec<_> = gps_segments(&s.read_records("location")?.chunks)
            .into_iter()
            .flatten()
            .collect();
        let t0 = fixes.first().map_or(0, |f| f.elapsed_realtime_ns);
        let secs = |t: i64| (t - t0) as f64 / 1e9;
        let pts: Vec<(f64, f64, f64)> = fixes
            .iter()
            .filter(|f| {
                let t = secs(f.elapsed_realtime_ns);
                t >= from_min * 60.0
                    && t <= to_min * 60.0
                    && f.horizontal_accuracy_meters
                        .is_some_and(|a| a <= ACCURACY_M)
                    && f.speed_accuracy_meters_per_second
                        .is_none_or(|a| a <= SPEED_ACCURACY_MPS)
            })
            .filter_map(|f| {
                let sigma = f
                    .speed_accuracy_meters_per_second
                    .unwrap_or(SPEED_ACCURACY_MPS) as f64;
                Some((
                    secs(f.elapsed_realtime_ns),
                    f.speed_meters_per_second? as f64,
                    sigma,
                ))
            })
            .collect();
        println!(
            "== {session} {}-{}: {line}, {model}{}, {} fixes",
            mmss(from_min * 60.0),
            mmss(to_min * 60.0),
            if grade { ", grade" } else { "" },
            pts.len()
        );
        let mut bins = Bins::default();
        // The acceleration at each fix: over the nearest fixes at least 3 s either side, 8 s apart at most.
        let mut accel = Vec::new();
        for (i, &(t, v, _)) in pts.iter().enumerate() {
            let before = pts[..i].iter().rev().find(|p| t - p.0 >= 3.0);
            let after = pts[i + 1..].iter().find(|p| p.0 - t >= 3.0);
            let a = match (before, after) {
                (Some(b), Some(c)) if c.0 - b.0 <= 8.0 => {
                    Some(((c.1 - b.1) / (c.0 - b.0), b.2.hypot(c.2) / (c.0 - b.0)))
                }
                _ => None,
            };
            if let (Some(f), Some((a, sigma))) = (csv.as_mut(), a) {
                writeln!(
                    f,
                    "{session} {from_min},{},{model},{grade},{v:.3},{a:.4},{sigma:.4}",
                    mmss(t)
                )?;
            }
            let a = a.map(|a| a.0);
            if dump {
                println!("    {} {:.1} mph {a:.2?}", mmss(t), v / MPH);
            }
            if let Some(a) = a {
                bins.add(v, a);
                by_model.entry((model, grade)).or_default().add(v, a);
            }
            accel.push(a);
        }
        // Legs: from one stop (or the run's start) to the next.
        let mut start = 0;
        for i in 1..=pts.len() {
            let end = i == pts.len() || (pts[i].1 < STOPPED_MPS && pts[i - 1].1 >= STOPPED_MPS);
            if !end {
                continue;
            }
            let leg = &pts[start..i];
            start = i;
            let Some(top) = leg.iter().map(|p| p.1).reduce(f64::max) else {
                continue;
            };
            if top < 5.0 * MPH || leg.len() < 10 {
                continue;
            }
            // From the last stopped fix before the first at 2 m/s (not the dwell) to the first at 20 and 30 mph.
            let moving = leg.iter().position(|p| p.1 >= 2.0);
            let from_stop = moving.and_then(|m| leg[..m].iter().rposition(|p| p.1 < STOPPED_MPS));
            let reach = |mph: f64| {
                let s = from_stop?;
                leg[s..]
                    .iter()
                    .find(|p| p.1 >= mph * MPH)
                    .map(|p| p.0 - leg[s].0)
            };
            let cruise: Vec<f64> = leg
                .iter()
                .zip(&accel[start - leg.len()..start])
                .filter(|(p, a)| p.1 >= 15.0 * MPH && a.is_some_and(|a| a.abs() < MOVING_MPS2))
                .map(|(p, _)| p.1 / MPH)
                .collect();
            println!(
                "  leg {} to {} ({}): top {:.0} mph, cruise {}, 0 to 20 mph {}, to 30 mph {}",
                mmss(leg[0].0),
                mmss(leg.last().unwrap().0),
                mmss(leg.last().unwrap().0 - leg[0].0),
                top / MPH,
                if cruise.is_empty() {
                    "none".into()
                } else {
                    format!(
                        "{:.0} to {:.0} mph ({} s)",
                        cruise.iter().copied().reduce(f64::min).unwrap(),
                        cruise.iter().copied().reduce(f64::max).unwrap(),
                        cruise.len()
                    )
                },
                reach(20.0).map_or("-".into(), |s| format!("{s:.0} s")),
                reach(30.0).map_or("-".into(), |s| format!("{s:.0} s")),
            );
        }
        bins.print();
    }
    println!("== by car model");
    for ((model, grade), bins) in &by_model {
        println!("  {model}{}", if *grade { ", grade" } else { "" });
        bins.print();
    }
    Ok(())
}

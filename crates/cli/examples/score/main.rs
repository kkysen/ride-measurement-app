// Every score of the examples `signs`, `holds`, `doorscore`, `curves`, `legs_gnss`, `jumps`, `legs`,
// `stop_positions`, and `vertical` for one session, and `cost` (see `cost.rs`), each replay done once and shared by the scores that use the same options,
// and only one replay held at a time.
// Writes everything to `data/work/score/<session>.txt` and prints each score's summary line.
// Leaves out where the phone sat still (`Options::skip_still`) unless `WHOLE=1`.
// Each replay's output is hashed, so a change meant to keep it the same can be checked.
// `ONLY=signs,holds` runs only those scores (and only the replays they need); `DUMP=1` as in each example.
use ride_fusion::live_speed::SUBWAY_MAX_SPEED_MPS;
use ride_fusion::replay::{Options, Replay, replay_with};
use ride_session::read::Session;
use std::time::Instant;

macro_rules! out {
    ($o:expr, $($a:tt)*) => {{
        use std::fmt::Write as _;
        let _ = writeln!($o, $($a)*);
    }};
}

mod cost;
mod curves;
mod doorscore;
mod holds;
mod jumps;
mod legs;
mod legs_gnss;
mod signs;
mod stop_positions;
mod vertical;

fn main() -> anyhow::Result<()> {
    let dir = std::env::args().nth(1).unwrap();
    let s = Session::open(&dir)?;
    let name = s.dir.file_name().unwrap().to_string_lossy().into_owned();
    let only: Option<Vec<String>> = std::env::var("ONLY")
        .ok()
        .map(|v| v.split(',').map(String::from).collect());
    let wants = |score: &str| only.as_ref().is_none_or(|o| o.iter().any(|x| x == score));
    let skip_still = std::env::var_os("WHOLE").is_none();
    let mut out = String::new();
    let hashes = std::cell::RefCell::new(String::new());
    let timed = |what: &str, options: &Options| -> anyhow::Result<Replay> {
        let start = Instant::now();
        let r = replay_with(&s, options)?;
        eprintln!("{name}: replay {what} {:.1?}", start.elapsed());
        out!(hashes.borrow_mut(), "hash {what}: {:016x}", hash(&r));
        Ok(r)
    };
    // The live speed's scores, the door cycles', and the subway's speed bound: what the app does on a train.
    let subway = Options {
        max_speed_mps: Some(SUBWAY_MAX_SPEED_MPS),
        no_statuses: true,
        live_every_ns: Some(1_000_000_000),
        skip_still,
        ..Options::default()
    };
    if ["signs", "holds", "doorscore", "curves", "legs_gnss"]
        .iter()
        .any(|x| wants(x))
    {
        let a = timed("subway", &subway)?;
        if wants("signs") {
            signs::run(&s, &a, &mut out)?;
        }
        if wants("holds") {
            holds::run(&s, &a, &mut out)?;
        }
        if wants("doorscore") {
            doorscore::run(&s, &a, &mut out)?;
        }
        if wants("curves") {
            let options = Options {
                gnss_outages: Some(curves::outages(&a)),
                live_every_ns: None,
                ..subway.clone()
            };
            let b = timed("subway with GNSS cut 30 s of 90", &options)?;
            curves::run(&s, &a, &b, &mut out)?;
        }
        if wants("legs_gnss") && s.stream_path("location").exists() {
            let options = Options {
                gnss_outages: Some(legs_gnss::outages()),
                live_every_ns: None,
                ..subway.clone()
            };
            let c = timed("subway without GNSS", &options)?;
            legs_gnss::run(&s, &c, &a, &mut out)?;
        }
    }
    // The path's scores, with the app's own options.
    if wants("jumps") || wants("legs") {
        let d = timed(
            "default",
            &Options {
                no_statuses: true,
                skip_still,
                ..Options::default()
            },
        )?;
        if wants("jumps") {
            jumps::run(&dir, &d, &mut out)?;
        }
        if wants("legs") {
            legs::run(&dir, &d, &mut out)?;
        }
    }
    // Going up and down, over the whole ride: an elevator's ride is where the phone sits still.
    if wants("vertical") {
        let v = timed(
            "whole",
            &Options {
                no_statuses: true,
                ..Options::default()
            },
        )?;
        vertical::run(&name, &v, &mut out)?;
    }
    if wants("stop_positions") {
        let e = timed(
            "unheld to the track",
            &Options {
                no_track_hold: true,
                no_statuses: true,
                skip_still,
                ..Options::default()
            },
        )?;
        stop_positions::run(&dir, &e, &mut out)?;
    }
    // The app's costs over the ride, replayed live with every status and smoothing tick.
    if wants("cost") {
        let r = timed(
            "live",
            &Options {
                live: true,
                tick_every_ns: Some(ride_fusion::fusion::SMOOTH_EVERY_NS),
                ..Options::default()
            },
        )?;
        cost::run(&r, &mut out)?;
    }
    out += &hashes.into_inner();
    let path = std::path::Path::new("data/work/score").join(format!("{name}.txt"));
    std::fs::create_dir_all(path.parent().unwrap())?;
    std::fs::write(&path, &out)?;
    // The summary: each score's own line, which starts with its name.
    let scores = [
        "signs ",
        "holds:",
        "doors:",
        "curves:",
        "legs_gnss:",
        "jumps:",
        "legs:",
        "stop_positions:",
        "vertical:",
        "cost",
        "hash ",
    ];
    for line in out
        .lines()
        .filter(|l| scores.iter().any(|s| l.starts_with(s)))
    {
        println!("{name} {line}");
    }
    Ok(())
}

/// A hash of what a replay produced: the same before and after a change means it changed nothing.
/// Each part's `Debug` text is hashed as it's written, never held whole.
fn hash(r: &Replay) -> u64 {
    use std::fmt::Write as _;
    use std::hash::Hasher;
    struct H(std::collections::hash_map::DefaultHasher);
    impl std::fmt::Write for H {
        fn write_str(&mut self, s: &str) -> std::fmt::Result {
            self.0.write(s.as_bytes());
            Ok(())
        }
    }
    let mut h = H(Default::default());
    let _ = write!(
        h,
        "{:?}{:?}{:?}{:?}{:?}{:?}{:?}",
        r.smoothed, r.positions, r.stops, r.statuses, r.timeline, r.vertical, r.live
    );
    h.0.finish()
}

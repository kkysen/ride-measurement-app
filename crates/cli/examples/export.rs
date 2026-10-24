// Scratch: fixes, manual events, and the replayed fusion as CSVs in `out`, for Python.
use ride_fusion::live_speed::SUBWAY_MAX_SPEED_MPS;
use ride_fusion::replay::{GnssOutages, Options, replay_with};
use ride_session::proto::record::Payload;
use ride_session::read::Session;
use ride_session::track::gps_segments;
use std::io::Write;
fn main() -> anyhow::Result<()> {
    let a: Vec<String> = std::env::args().collect();
    let s = Session::open(&a[1])?;
    let out = std::path::Path::new(&a[2]);
    std::fs::create_dir_all(out)?;
    let loc = s.read_records("location")?;
    let mut f = std::fs::File::create(out.join("fixes.csv"))?;
    writeln!(f, "t_ns,time_ms,lat,lon,speed,speed_acc,h_acc")?;
    for p in gps_segments(&loc.chunks).iter().flatten() {
        let o = |x: Option<f32>| x.map_or(String::new(), |x| x.to_string());
        writeln!(
            f,
            "{},{},{},{},{},{},{}",
            p.elapsed_realtime_ns,
            p.time_ms,
            p.latitude_degrees,
            p.longitude_degrees,
            o(p.speed_meters_per_second),
            o(p.speed_accuracy_meters_per_second),
            o(p.horizontal_accuracy_meters)
        )?;
    }
    let mut f = std::fs::File::create(out.join("events.csv"))?;
    writeln!(f, "t_ns,kind,text")?;
    // As the rider means them now: deleted ones left out, changed ones as changed.
    for m in ride_session::marks::marks(&s)?
        .into_iter()
        .filter(|m| !m.deleted)
    {
        writeln!(
            f,
            "{},{},\"{}\"",
            m.t_ns,
            m.kind.as_str_name(),
            m.text.replace('"', "'")
        )?;
    }
    // Our own raw-Doppler speeds, solved in time order (like the replay, but every epoch, never dropped).
    let mut nav = ride_gnss::solve::Navigator::default();
    for eph in ride_gnss::orbits::read_session_orbits(&s.dir)? {
        nav.add_ephemeris(eph);
    }
    let mut gnss: Vec<(i64, ride_session::proto::GnssMeasurementsEvent)> = Vec::new();
    for r in s
        .read_records("gnss")?
        .chunks
        .into_iter()
        .flat_map(|c| c.records)
    {
        if let Some(Payload::GnssMeasurements(m)) = r.payload {
            let t = m
                .clock
                .as_ref()
                .and_then(|c| c.elapsed_realtime_nanos)
                .unwrap_or(r.received_elapsed_realtime_ns);
            gnss.push((t, m));
        }
    }
    gnss.sort_by_key(|g| g.0);
    let mut f = std::fs::File::create(out.join("own.csv"))?;
    writeln!(f, "t_ns,speed,sigma")?;
    for (t, m) in &gnss {
        if let Some(v) = nav.on_measurements(m).ok().and_then(|sol| sol.velocity) {
            writeln!(f, "{t},{},{}", v.speed(), v.speed_sigma())?;
        }
    }
    // Audio frame positions on the elapsed realtime clock, to place `ride wav`'s samples.
    if s.stream_path("audio").exists() {
        let mut f = std::fs::File::create(out.join("audio_ts.csv"))?;
        writeln!(f, "frame,t_ns")?;
        for a in &s.read_audio("audio")?.chunks {
            for ts in &a.timestamps {
                writeln!(f, "{},{}", ts.frame_position, ts.boottime_ns)?;
            }
        }
    }
    let subway = std::env::var_os("SUBWAY").is_some();
    let r = replay_with(
        &s,
        &Options {
            max_speed_mps: subway.then_some(SUBWAY_MAX_SPEED_MPS),
            // `CUT_GNSS=<start ns>,<outage s>,<period s>`, to score dead reckoning (`python/experiments/score.py`).
            gnss_outages: std::env::var("CUT_GNSS").ok().map(|v| {
                let f: Vec<&str> = v.split(',').collect();
                GnssOutages {
                    start_ns: f[0].parse().unwrap(),
                    outage_s: f[1].parse().unwrap(),
                    period_s: f[2].parse().unwrap(),
                }
            }),
            ignore_audio: std::env::var_os("NO_AUDIO").is_some(),
            no_statuses: std::env::var_os("NO_STATUSES").is_some(),
            live_every_ns: None,
        },
    )?;
    let mut f = std::fs::File::create(out.join("chimes.csv"))?;
    writeln!(f, "t_ns,cars")?;
    for (t, cars) in &r.chimes {
        writeln!(f, "{t},{cars:?}")?;
    }
    let mut f = std::fs::File::create(out.join("subway.csv"))?;
    writeln!(f, "t_ns,in_subway")?;
    for (t, on) in &r.subway_changes {
        writeln!(f, "{t},{on}")?;
    }
    let mut f = std::fs::File::create(out.join("status.csv"))?;
    writeln!(
        f,
        "t_ns,mode,station,at_station,cars,staying_s,stopped_s,transfer_s"
    )?;
    for (t, st) in &r.statuses {
        writeln!(
            f,
            "{t},{:?},{},{},{},{:.0},{},{}",
            st.mode,
            st.station.as_ref().map_or("", |s| s.name.as_str()),
            st.at_station,
            st.cars.map_or(String::new(), |c| format!("{c:?}")),
            st.staying_s,
            st.dwell
                .as_ref()
                .map_or(String::new(), |d| format!("{:.0}", d.stopped_s)),
            st.transfer_s.map_or(String::new(), |v| format!("{v:.0}"))
        )?;
    }
    let mut f = std::fs::File::create(out.join("vertical.csv"))?;
    writeln!(f, "start_ns,end_ns,change_m,kind")?;
    for m in &r.vertical {
        writeln!(
            f,
            "{},{},{:.1},{:?}",
            m.start_ns, m.end_ns, m.change_m, m.kind
        )?;
    }
    let mut f = std::fs::File::create(out.join("timeline.csv"))?;
    writeln!(f, "start_ns,end_ns,doing")?;
    for s in &r.timeline {
        writeln!(f, "{},{},{:?}", s.start_ns, s.end_ns, s.doing)?;
    }
    let mut f = std::fs::File::create(out.join("door_cycles.csv"))?;
    writeln!(f, "open_ns,close_ns")?;
    for c in &r.door_cycles {
        writeln!(f, "{},{}", c.open_ns, c.close_ns)?;
    }
    // The train's stops, for dwell and door analyses; empty fields are unknown (no doors heard: a tunnel stop,
    // or doors missed).
    let mut f = std::fs::File::create(out.join("stops.csv"))?;
    writeln!(
        f,
        "stopped_ns,score,in_subway,station,doors_open_ns,doors_open_heard,doors_closed_ns,departed_ns,on_platform,alighted_ns,boarded_ns,dwell_s,doors_open_s,car_before,car_after,car_change,train_change"
    )?;
    let opt = |v: Option<i64>| v.map_or(String::new(), |v| v.to_string());
    let opt_s = |v: Option<f64>| v.map_or(String::new(), |v| format!("{v:.1}"));
    for s in &r.stops {
        writeln!(
            f,
            "{},{:.2},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            s.stopped_ns,
            s.score,
            s.in_subway,
            s.station.map_or(String::new(), |st| st.name.clone()),
            opt(s.doors_open_ns),
            s.doors_open_heard,
            opt(s.doors_closed_ns),
            opt(s.departed_ns),
            s.on_platform,
            opt(s.alighted_ns),
            opt(s.boarded_ns),
            opt_s(s.dwell_s()),
            opt_s(s.doors_open_s()),
            s.car_before.map_or(String::new(), |c| format!("{c:04x}")),
            s.car_after.map_or(String::new(), |c| format!("{c:04x}")),
            s.car_change(),
            s.train_change,
        )?;
    }
    write_tapped_stops(out, &s, &r.stops)?;
    let mut f = std::fs::File::create(out.join("departures.csv"))?;
    writeln!(f, "t_ns")?;
    for t in &r.departures {
        writeln!(f, "{t}")?;
    }
    let mut f = std::fs::File::create(out.join("steps.csv"))?;
    writeln!(f, "t_ns,steps")?;
    for (t, steps) in &r.step_windows {
        writeln!(f, "{t},{steps}")?;
    }
    let mut f = std::fs::File::create(out.join("fused.csv"))?;
    writeln!(f, "t_ns,speed,sigma,forward,backward,accel")?;
    for x in &r.smoothed {
        writeln!(
            f,
            "{},{},{},{},{},{}",
            x.elapsed_realtime_ns,
            x.speed_mps,
            x.sigma_mps,
            x.forward_mps,
            x.backward_mps,
            x.accel_mps2
        )?;
    }
    Ok(())
}

/// The stops as the rider's taps tell them, offline only (taps never feed fusion): each "train stopped" tap starts
/// one, filled from the taps up to the next (the last "doors closed" before "train started", since a door closing
/// and reopening was sometimes tapped twice); "alighted" then "boarded" is a transfer. Each tapped stop is matched
/// to the sensors' record within [`MATCH_S`], whose times fill what wasn't tapped (`source` says which), with how
/// far the sensors were from the taps; stops only the sensors found follow, as `sensors`.
fn write_tapped_stops(
    out: &std::path::Path,
    s: &Session,
    stops: &[ride_fusion::fusion::StopRecord],
) -> anyhow::Result<()> {
    use ride_session::proto::manual_event::Kind;
    const MATCH_S: f64 = 5.0;
    let marks: Vec<_> = ride_session::marks::marks(s)?
        .into_iter()
        .filter(|m| !m.deleted)
        .collect();
    #[derive(Default)]
    struct Tapped {
        stopped: Option<i64>,
        open: Option<i64>,
        closed: Option<i64>,
        started: Option<i64>,
        alighted: Option<i64>,
        boarded: Option<i64>,
    }
    let mut tapped: Vec<Tapped> = Vec::new();
    for m in &marks {
        if m.kind == Kind::TrainStopped {
            tapped.push(Tapped {
                stopped: Some(m.t_ns),
                ..Default::default()
            });
            continue;
        }
        let Some(cur) = tapped.last_mut() else {
            continue;
        };
        // After walking off, the doors and the departure are another train's until the rider boards it.
        let off_train = cur.alighted.is_some() && cur.boarded.is_none();
        match m.kind {
            Kind::DoorsOpen if cur.open.is_none() && cur.started.is_none() && !off_train => {
                cur.open = Some(m.t_ns)
            }
            Kind::DoorsClosed if cur.started.is_none() && !off_train => cur.closed = Some(m.t_ns),
            Kind::TrainStarted if cur.started.is_none() && !off_train => cur.started = Some(m.t_ns),
            Kind::Alighted if cur.alighted.is_none() && cur.started.is_none() => {
                cur.alighted = Some(m.t_ns)
            }
            Kind::Boarded if cur.alighted.is_some() && cur.boarded.is_none() => {
                cur.boarded = Some(m.t_ns)
            }
            _ => {}
        }
    }
    let mut f = std::fs::File::create(out.join("stops_tapped.csv"))?;
    writeln!(
        f,
        "source,stopped_ns,doors_open_ns,doors_closed_ns,departed_ns,alighted_ns,boarded_ns,transfer,dwell_s,doors_open_s,\
         sensor_stopped_err_s,sensor_dwell_err_s,sensor_rider_left"
    )?;
    let opt = |v: Option<i64>| v.map_or(String::new(), |v| v.to_string());
    let opt_s = |v: Option<f64>| v.map_or(String::new(), |v| format!("{v:.1}"));
    let mut used = vec![false; stops.len()];
    for t in &tapped {
        let stopped = t.stopped.unwrap();
        let k = stops
            .iter()
            .enumerate()
            .filter(|(k, r)| !used[*k] && ((r.stopped_ns - stopped) as f64 * 1e-9).abs() <= MATCH_S)
            .min_by_key(|(_, r)| (r.stopped_ns - stopped).abs())
            .map(|(k, _)| k);
        let sensed = k.map(|k| {
            used[k] = true;
            &stops[k]
        });
        let left = t.alighted.is_some();
        let fill = |tap: Option<i64>, sensor: Option<i64>| {
            (tap.or(sensor), tap.is_none() && sensor.is_some())
        };
        let (open, f1) = fill(t.open, sensed.and_then(|r| r.doors_open_ns));
        let (closed, f2) = fill(t.closed, sensed.and_then(|r| r.doors_closed_ns));
        let (departed, f3) = if left && t.started.is_none() {
            (None, false)
        } else {
            fill(t.started, sensed.and_then(|r| r.departed_ns))
        };
        let source = if f1 || f2 || f3 {
            "taps+sensors"
        } else {
            "taps"
        };
        let dwell = (!left || t.boarded.is_some())
            .then(|| departed.map(|d| (d - stopped) as f64 * 1e-9))
            .flatten()
            .filter(|_| !left);
        // A transfer's doors are two trains': the one left, opening, and the one boarded, closing.
        let doors = open
            .zip(closed)
            .filter(|_| t.boarded.is_none())
            .map(|(o, c)| (c - o) as f64 * 1e-9);
        writeln!(
            f,
            "{source},{stopped},{},{},{},{},{},{},{},{},{},{},{}",
            opt(open),
            opt(closed),
            opt(departed),
            opt(t.alighted),
            opt(t.boarded),
            left && t.boarded.is_some(),
            opt_s(dwell),
            opt_s(doors),
            opt_s(sensed.map(|r| (r.stopped_ns - stopped) as f64 * 1e-9)),
            opt_s(
                sensed
                    .and_then(|r| r.dwell_s())
                    .zip(dwell)
                    .map(|(a, b)| a - b)
            ),
            sensed.map_or(String::new(), |r| r.rider_left().to_string()),
        )?;
    }
    for (k, r) in stops.iter().enumerate() {
        if used[k] || !r.in_subway {
            continue;
        }
        writeln!(
            f,
            "sensors,{},{},{},{},{},{},{},{},{},,,{}",
            r.stopped_ns,
            opt(r.doors_open_ns),
            opt(r.doors_closed_ns),
            opt(r.departed_ns),
            opt(r.alighted_ns),
            opt(r.boarded_ns),
            r.alighted_ns.is_some() && r.boarded_ns.is_some(),
            opt_s(r.dwell_s()),
            opt_s(r.doors_open_s()),
            r.rider_left(),
        )?;
    }
    Ok(())
}

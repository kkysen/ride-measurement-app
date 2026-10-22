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
        },
    )?;
    let mut f = std::fs::File::create(out.join("subway.csv"))?;
    writeln!(f, "t_ns,in_subway")?;
    for (t, on) in &r.subway_changes {
        writeln!(f, "{t},{on}")?;
    }
    let mut f = std::fs::File::create(out.join("status.csv"))?;
    writeln!(f, "t_ns,mode,station,at_station,cars,staying_s")?;
    for (t, st) in &r.statuses {
        writeln!(
            f,
            "{t},{:?},{},{},{},{:.0}",
            st.mode,
            st.station.as_ref().map_or("", |s| s.name.as_str()),
            st.at_station,
            st.cars.map_or(String::new(), |c| format!("{c:?}")),
            st.staying_s
        )?;
    }
    let mut f = std::fs::File::create(out.join("door_cycles.csv"))?;
    writeln!(f, "open_ns,close_ns")?;
    for c in &r.door_cycles {
        writeln!(f, "{},{}", c.open_ns, c.close_ns)?;
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

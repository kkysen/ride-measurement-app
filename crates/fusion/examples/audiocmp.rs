// Scratch (PLAN.md item 42, lossy side measurement): what the app derives from a session's audio, to compare a session
// against a copy whose audio went through a lossy codec.
// `audiocmp doors <session>...`: door cycles and confirmed chimes from `DoorDetector` alone, fed as the app reads audio
//   (a tenth of a second at a time), streaming.
// `audiocmp replay <session>...`: the whole replay's door cycles, chimes and stops (`skip_still`, `no_live_smoothing`).
use ride_fusion::doors::DoorDetector;
use ride_fusion::replay::{Options, replay_with};
use ride_session::AudioChunk;
use ride_session::read::Session;

fn secs(s: &Session, t_ns: i64) -> f64 {
    (t_ns - s.info.start_elapsed_realtime_ns) as f64 * 1e-9
}

fn doors(path: &str) -> anyhow::Result<()> {
    let s = Session::open(path)?;
    let (rate, channels) = s
        .info
        .streams
        .iter()
        .find_map(|x| {
            x.audio
                .as_ref()
                .map(|a| (a.sample_rate_hz, a.channels.max(1) as usize))
        })
        .expect("an audio stream");
    let mut d = DoorDetector::new(rate as u32);
    let mut last = None;
    let read = rate as usize / 10;
    for c in s.stream_chunks::<AudioChunk>("audio")? {
        let mut c = c?;
        assert!(ride_session::audio_flac::restore(&mut c));
        let ts: Vec<(i64, i64)> = c
            .timestamps
            .iter()
            .map(|t| (t.frame_position, t.boottime_ns))
            .collect();
        let pcm = c.pcm();
        let frame_bytes = pcm.bytes_per_sample() * channels;
        for (i, part) in pcm.bytes().chunks(read * frame_bytes).enumerate() {
            let first = c.first_frame + (i * read) as i64;
            let part = match pcm {
                ride_session::Pcm::S16(_) => ride_session::Pcm::S16(part),
                ride_session::Pcm::F32(_) => ride_session::Pcm::F32(part),
            };
            let ts_i: Vec<(i64, i64)> = ts.get(i).copied().into_iter().collect();
            let a =
                ride_fusion::fusion::AudioSamples::from_pcm(first, part, channels, ts_i.clone());
            for cy in d.on_audio(first, &a.samples, &ts_i) {
                println!(
                    "cycle {:.1} {:.1}",
                    secs(&s, cy.open_ns),
                    secs(&s, cy.close_ns)
                );
            }
            if d.last_chime() != last {
                last = d.last_chime();
                if let Some((t, cars)) = last {
                    println!("chime {:.2} {cars:?}", secs(&s, t));
                }
            }
        }
    }
    Ok(())
}

fn replay(path: &str) -> anyhow::Result<()> {
    let s = Session::open(path)?;
    let r = replay_with(
        &s,
        &Options {
            skip_still: true,
            no_live_smoothing: true,
            ..Default::default()
        },
    )?;
    for c in &r.door_cycles {
        println!(
            "cycle {:.1} {:.1}",
            secs(&s, c.open_ns),
            secs(&s, c.close_ns)
        );
    }
    for (t, cars) in &r.chimes {
        println!("chime {:.2} {cars:?}", secs(&s, *t));
    }
    for st in &r.stops {
        let o = |t: Option<i64>| t.map_or("-".to_string(), |t| format!("{:.1}", secs(&s, t)));
        println!(
            "stop {:.1} {} open {} heard {} closed {} departed {}",
            secs(&s, st.stopped_ns),
            st.station.map_or("-", |x| &x.name),
            o(st.doors_open_ns),
            st.doors_open_heard,
            o(st.doors_closed_ns),
            o(st.departed_ns)
        );
    }
    println!("statuses {}", r.statuses.len());
    Ok(())
}

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    for p in &args[2..] {
        println!("== {p}");
        match args[1].as_str() {
            "doors" => doors(p)?,
            "replay" => replay(p)?,
            _ => anyhow::bail!("doors|replay"),
        }
    }
    Ok(())
}

use std::path::PathBuf;

use anyhow::Context;
use clap::{Parser, Subcommand};
use ride_session::proto::record::Payload;
use ride_session::proto::{AudioInfo, StreamKind, manual_event};
use ride_session::read::Session;

#[derive(Parser)]
#[command(about = "Inspect ride sessions")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Per-stream sample counts, rates, and gaps.
    Summary { session: PathBuf },
    /// Print one stream as CSV: timestamp_ns, accuracy, values...
    Csv { session: PathBuf, stream: String },
    /// Export an audio stream as a WAV file, bit-exact PCM.
    Wav {
        session: PathBuf,
        output: PathBuf,
        #[arg(long, default_value = "audio")]
        stream: String,
    },
}

fn main() -> anyhow::Result<()> {
    match Cli::parse().command {
        Command::Summary { session } => summary(&Session::open(session)?),
        Command::Csv { session, stream } => csv(&Session::open(session)?, &stream),
        Command::Wav {
            session,
            output,
            stream,
        } => wav(&Session::open(session)?, &stream, &output),
    }
}

fn summary(session: &Session) -> anyhow::Result<()> {
    let info = &session.info;
    println!(
        "session {} (format v{})",
        info.session_id, info.format_version
    );
    if !info.continues_session_id.is_empty() {
        println!(
            "continues session {} (Android killed recording and it restarted)",
            info.continues_session_id
        );
    }
    if let Some(d) = &info.device {
        println!("device {} {} (SDK {})", d.manufacturer, d.model, d.sdk_int);
    }
    for stream in &info.streams {
        match stream.kind() {
            StreamKind::Records => {
                record_summary(session, &stream.name)?;
                continue;
            }
            StreamKind::Audio => {
                audio_summary(session, &stream.name, stream.audio.as_ref())?;
                continue;
            }
            StreamKind::Sensor => {}
        }
        let contents = session
            .read_stream(&stream.name)
            .with_context(|| format!("stream {}", stream.name))?;
        let timestamps: Vec<i64> = contents
            .chunks
            .iter()
            .flat_map(|c| c.timestamps_ns())
            .collect();
        print!(
            "{}: {} chunks, {} samples",
            stream.name,
            contents.chunks.len(),
            timestamps.len()
        );
        if contents.truncated {
            print!(", TRUNCATED tail");
        }
        println!();
        if timestamps.len() < 2 {
            continue;
        }
        let mut dts: Vec<i64> = timestamps.windows(2).map(|w| w[1] - w[0]).collect();
        let non_monotonic = dts.iter().filter(|&&dt| dt <= 0).count();
        let duration_s = (timestamps[timestamps.len() - 1] - timestamps[0]) as f64 * 1e-9;
        dts.sort_unstable();
        let median = dts[dts.len() / 2];
        let max = dts[dts.len() - 1];
        let gaps = dts.iter().filter(|&&dt| dt > 2 * median).count();
        println!(
            "  {duration_s:.1} s, mean {:.1} Hz, median interval {:.3} ms, max {:.3} ms, {gaps} gaps (> 2x median), {non_monotonic} non-increasing",
            (timestamps.len() - 1) as f64 / duration_s,
            median as f64 * 1e-6,
            max as f64 * 1e-6,
        );
    }
    Ok(())
}

fn record_summary(session: &Session, name: &str) -> anyhow::Result<()> {
    let contents = session
        .read_records(name)
        .with_context(|| format!("stream {name}"))?;
    let records: Vec<_> = contents.chunks.iter().flat_map(|c| &c.records).collect();
    print!(
        "{name}: {} chunks, {} records",
        contents.chunks.len(),
        records.len()
    );
    if contents.truncated {
        print!(", TRUNCATED tail");
    }
    println!();
    let (Some(first), Some(last)) = (records.first(), records.last()) else {
        return Ok(());
    };
    let start = first.received_elapsed_realtime_ns;
    println!(
        "  {:.1} s",
        (last.received_elapsed_realtime_ns - start) as f64 * 1e-9
    );
    let mut counts = std::collections::BTreeMap::<&str, usize>::new();
    for r in &records {
        *counts.entry(payload_name(r.payload.as_ref())).or_default() += 1;
        if let Some(Payload::ManualEvent(e)) = &r.payload {
            let kind = manual_event::Kind::try_from(e.kind).map_or("unknown", |k| k.as_str_name());
            println!(
                "  at {:.1} s: {kind} {}",
                (r.received_elapsed_realtime_ns - start) as f64 * 1e-9,
                e.text
            );
        }
    }
    for (kind, n) in counts {
        println!("  {n} {kind}");
    }
    Ok(())
}

fn audio_summary(session: &Session, name: &str, info: Option<&AudioInfo>) -> anyhow::Result<()> {
    let contents = session
        .read_audio(name)
        .with_context(|| format!("stream {name}"))?;
    let rate = info.map_or(0, |i| i.sample_rate_hz);
    let frames: usize = contents.chunks.iter().map(|c| c.pcm_s16le.len() / 2).sum();
    print!("{name}: {} chunks, {frames} frames", contents.chunks.len());
    if contents.truncated {
        print!(", TRUNCATED tail");
    }
    println!();
    let mut expected = 0;
    let mut discontinuities = 0;
    for c in &contents.chunks {
        if c.first_frame != expected {
            discontinuities += 1;
        }
        expected = c.first_frame + (c.pcm_s16le.len() / 2) as i64;
    }
    let timestamps: Vec<_> = contents.chunks.iter().flat_map(|c| &c.timestamps).collect();
    if rate > 0 {
        print!(
            "  {:.1} s at {rate} Hz ({})",
            frames as f64 / f64::from(rate),
            info.map_or("", |i| i.audio_source_name.as_str())
        );
    }
    // Compare the audio clock to boot time across the stream: frames per second actually delivered.
    if let (Some(a), Some(b)) = (timestamps.first(), timestamps.last())
        && b.boottime_ns > a.boottime_ns
    {
        let measured = (b.frame_position - a.frame_position) as f64
            / ((b.boottime_ns - a.boottime_ns) as f64 * 1e-9);
        print!(", measured {measured:.2} Hz");
    }
    println!(", {discontinuities} frame discontinuities");
    Ok(())
}

fn wav(session: &Session, stream: &str, output: &std::path::Path) -> anyhow::Result<()> {
    use std::io::Write;
    let info = session
        .info
        .streams
        .iter()
        .find(|s| s.name == stream)
        .and_then(|s| s.audio.clone())
        .with_context(|| format!("no audio stream {stream}"))?;
    let contents = session.read_audio(stream)?;
    let data_len: usize = contents.chunks.iter().map(|c| c.pcm_s16le.len()).sum();
    let channels = u16::try_from(info.channels)?;
    let rate = u32::try_from(info.sample_rate_hz)?;
    let data_len = u32::try_from(data_len).context("audio too long for one WAV file")?;
    let mut out = std::io::BufWriter::new(std::fs::File::create(output)?);
    out.write_all(b"RIFF")?;
    out.write_all(&(36 + data_len).to_le_bytes())?;
    out.write_all(b"WAVEfmt ")?;
    out.write_all(&16u32.to_le_bytes())?;
    out.write_all(&1u16.to_le_bytes())?; // PCM
    out.write_all(&channels.to_le_bytes())?;
    out.write_all(&rate.to_le_bytes())?;
    out.write_all(&(rate * u32::from(channels) * 2).to_le_bytes())?;
    out.write_all(&(channels * 2).to_le_bytes())?;
    out.write_all(&16u16.to_le_bytes())?;
    out.write_all(b"data")?;
    out.write_all(&data_len.to_le_bytes())?;
    for c in &contents.chunks {
        out.write_all(&c.pcm_s16le)?;
    }
    out.flush()?;
    if contents.truncated {
        eprintln!("warning: {stream} has a truncated tail");
    }
    Ok(())
}

fn payload_name(payload: Option<&Payload>) -> &'static str {
    match payload {
        None => "empty",
        Some(Payload::GnssMeasurements(_)) => "gnss_measurements",
        Some(Payload::GnssStatus(_)) => "gnss_status",
        Some(Payload::GnssNavigationMessage(_)) => "gnss_navigation_message",
        Some(Payload::GnssMeasurementsStatus(_)) => "gnss_measurements_status",
        Some(Payload::Location(_)) => "location",
        Some(Payload::CellInfo(_)) => "cell_info",
        Some(Payload::ServiceState(_)) => "service_state",
        Some(Payload::SignalStrength(_)) => "signal_strength",
        Some(Payload::WifiScan(_)) => "wifi_scan",
        Some(Payload::ManualEvent(_)) => "manual_event",
        Some(Payload::Battery(_)) => "battery",
    }
}

fn csv(session: &Session, stream: &str) -> anyhow::Result<()> {
    let contents = session.read_stream(stream)?;
    for chunk in &contents.chunks {
        let n = chunk.values_per_sample as usize;
        let values: Vec<f32> = chunk.values().collect();
        for (i, t) in chunk.timestamps_ns().into_iter().enumerate() {
            print!("{t},{}", chunk.accuracy[i]);
            for v in &values[i * n..(i + 1) * n] {
                // `{:?}` prints the shortest representation that parses back to the same `f32`.
                print!(",{v:?}");
            }
            println!();
        }
    }
    if contents.truncated {
        eprintln!("warning: {stream} has a truncated tail");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use ride_session::proto::{AudioInfo, StreamInfo, StreamKind};
    use ride_session::read::Session;
    use ride_session::write::{StreamWriter, write_session_info};
    use ride_session::{AudioChunk, FORMAT_VERSION, SessionInfo};

    #[test]
    fn wav_has_header_and_exact_pcm() {
        let dir = tempfile::tempdir().unwrap();
        let info = SessionInfo {
            format_version: FORMAT_VERSION,
            streams: vec![StreamInfo {
                name: "audio".into(),
                kind: StreamKind::Audio.into(),
                audio: Some(AudioInfo {
                    sample_rate_hz: 16_000,
                    channels: 1,
                    ..Default::default()
                }),
                ..Default::default()
            }],
            ..Default::default()
        };
        write_session_info(dir.path(), &info).unwrap();
        let pcm: Vec<u8> = (0..1000u16)
            .flat_map(|i| (i.wrapping_mul(977)).to_le_bytes())
            .collect();
        let mut w = StreamWriter::create(dir.path(), "audio").unwrap();
        for c in 0..2 {
            w.append(&AudioChunk {
                first_frame: c * 1000,
                pcm_s16le: pcm.clone(),
                ..Default::default()
            })
            .unwrap();
        }
        let out = dir.path().join("out.wav");
        super::wav(&Session::open(dir.path()).unwrap(), "audio", &out).unwrap();
        let bytes = std::fs::read(out).unwrap();
        assert_eq!(&bytes[..4], b"RIFF");
        assert_eq!(&bytes[8..16], b"WAVEfmt ");
        assert_eq!(
            u32::from_le_bytes(bytes[24..28].try_into().unwrap()),
            16_000
        );
        assert_eq!(&bytes[36..40], b"data");
        assert_eq!(u32::from_le_bytes(bytes[40..44].try_into().unwrap()), 4000);
        assert_eq!(&bytes[44..], [pcm.clone(), pcm].concat());
    }
}

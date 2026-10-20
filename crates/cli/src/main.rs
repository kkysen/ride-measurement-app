mod gnss;
mod import;
mod map;

use std::path::PathBuf;

use anyhow::Context;
use clap::{Parser, Subcommand};
use ride_fusion::speed_colors::SpeedScale;
use ride_session::proto::record::Payload;
use ride_session::proto::{AudioInfo, StreamKind, manual_event};
use ride_session::read::{Session, StreamContents};
use ride_session::{SESSION_INFO_FILE, STREAM_EXTENSION};

#[derive(Parser)]
#[command(about = "Inspect ride sessions")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Per-stream sample counts, rates, gaps, and any damage, for sessions or directories of sessions.
    /// Exits with an error status if any stream is unreadable or damaged.
    Summary {
        #[arg(required = true)]
        sessions: Vec<PathBuf>,
    },
    /// Print one stream as CSV: timestamp_ns, accuracy, values...
    Csv { session: PathBuf, stream: String },
    /// Export an audio stream as a WAV file, bit-exact PCM.
    Wav {
        session: PathBuf,
        output: PathBuf,
        #[arg(long, default_value = "audio")]
        stream: String,
    },
    /// How well raw GNSS solves for velocity: navigation messages, ephemerides, epochs, residuals,
    /// and the solved speed against the phone's own.
    Gnss { session: PathBuf },
    /// Convert a GnssLogger log (Google's GPS Measurement Tools, Smartphone Decimeter Challenge) into a new session,
    /// to try `gnss` on public logs (and `map`, if the log has the phone's fixes with UTC times, which
    /// Smartphone Decimeter Challenge logs don't). Never writes into an existing non-empty directory.
    ImportGnsslogger { log: PathBuf, session: PathBuf },
    /// The orbit files to download for a session, one UTC day per line as `path url` pairs to try in order
    /// (`just orbits` downloads them; the app follows the same plan live).
    OrbitPlan { session: PathBuf },
    /// Write the ride's GPS path, colored by speed or acceleration, as an HTML map (default `<session>/map.html`).
    Map {
        session: PathBuf,
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// The speed color scale; by default, chosen from the ride's top fused speed.
        #[arg(long, value_parser = parse_scale)]
        scale: Option<SpeedScale>,
        /// Color by acceleration instead of speed at first; the page can switch either way.
        #[arg(long)]
        accel: bool,
    },
}

fn main() -> anyhow::Result<()> {
    match Cli::parse().command {
        Command::Summary { sessions } => summaries(&sessions),
        Command::Csv { session, stream } => csv(&Session::open(session)?, &stream),
        Command::Wav {
            session,
            output,
            stream,
        } => wav(&Session::open(session)?, &stream, &output),
        Command::Gnss { session } => gnss::gnss(&Session::open(session)?),
        Command::ImportGnsslogger { log, session } => import::import(&log, &session),
        Command::OrbitPlan { session } => gnss::orbit_plan(&Session::open(session)?),
        Command::Map {
            session,
            output,
            scale,
            accel,
        } => {
            let output = output.unwrap_or_else(|| session.join("map.html"));
            map::map(&Session::open(session)?, &output, scale, accel)
        }
    }
}

fn parse_scale(id: &str) -> Result<SpeedScale, String> {
    SpeedScale::from_id(id).ok_or_else(|| {
        let ids: Vec<_> = SpeedScale::ALL.iter().map(|s| s.id()).collect();
        format!("expected one of {}", ids.join(", "))
    })
}

/// Problems found while summarizing; errors fail the command, warnings don't.
#[derive(Default)]
struct Problems {
    errors: Vec<String>,
    warnings: Vec<String>,
}

/// Sessions are directories with a `session.pb`; any other directory is searched one level down.
fn find_sessions(paths: &[PathBuf]) -> anyhow::Result<Vec<PathBuf>> {
    let mut sessions = Vec::new();
    for path in paths {
        if path.join(SESSION_INFO_FILE).exists() {
            sessions.push(path.clone());
            continue;
        }
        let mut children: Vec<_> = std::fs::read_dir(path)
            .with_context(|| format!("reading {}", path.display()))?
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.is_dir())
            .collect();
        children.sort();
        if children.is_empty() {
            anyhow::bail!(
                "{} is not a session and contains no sessions",
                path.display()
            );
        }
        sessions.extend(children);
    }
    Ok(sessions)
}

fn summaries(paths: &[PathBuf]) -> anyhow::Result<()> {
    let mut problems = Problems::default();
    for (i, dir) in find_sessions(paths)?.iter().enumerate() {
        if i > 0 {
            println!();
        }
        match Session::open(dir) {
            Ok(session) => summary(&session, &mut problems),
            Err(e) => problems.errors.push(format!("{}: {e:#}", dir.display())),
        }
    }
    println!();
    for w in &problems.warnings {
        println!("warning: {w}");
    }
    for e in &problems.errors {
        println!("ERROR: {e}");
    }
    if !problems.errors.is_empty() {
        anyhow::bail!("{} errors", problems.errors.len());
    }
    if problems.warnings.is_empty() {
        println!("all streams clean");
    }
    Ok(())
}

/// Prints damage and truncation for one stream and records them as problems.
fn report_health<C>(contents: &StreamContents<C>, label: &str, problems: &mut Problems) {
    if contents.truncated {
        println!("  truncated tail (expected if recording was killed)");
        problems.warnings.push(format!(
            "{label}: truncated tail; complete chunks before it were kept"
        ));
    }
    for d in &contents.damage {
        println!(
            "  DAMAGE at byte {} ({} bytes): {}",
            d.offset, d.len, d.reason
        );
        problems.errors.push(format!(
            "{label}: damaged bytes {}..{}: {}",
            d.offset,
            d.offset + d.len,
            d.reason
        ));
    }
}

fn summary(session: &Session, problems: &mut Problems) {
    let info = &session.info;
    println!("{}", session.dir.display());
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
        let label = format!("{}/{}", info.session_id, stream.name);
        let result = match stream.kind() {
            StreamKind::Records => record_summary(session, &stream.name, &label, problems),
            StreamKind::Audio => audio_summary(
                session,
                &stream.name,
                stream.audio.as_ref(),
                &label,
                problems,
            ),
            StreamKind::Sensor => sensor_summary(session, &stream.name, &label, problems),
        };
        if let Err(e) = result {
            println!("{}: ERROR {e:#}", stream.name);
            problems.errors.push(format!("{label}: {e:#}"));
        }
    }
    // Stream files on disk that `session.pb` doesn't list, e.g. from a newer app version.
    let listed: std::collections::HashSet<_> = info
        .streams
        .iter()
        .map(|s| format!("{}.{STREAM_EXTENSION}", s.name))
        .collect();
    if let Ok(entries) = std::fs::read_dir(&session.dir) {
        let mut unlisted: Vec<_> = entries
            .filter_map(|e| e.ok()?.file_name().into_string().ok())
            .filter(|n| n.ends_with(&format!(".{STREAM_EXTENSION}")) && !listed.contains(n))
            .collect();
        unlisted.sort();
        for name in unlisted {
            problems.warnings.push(format!(
                "{}: {name} isn't listed in {SESSION_INFO_FILE}",
                info.session_id
            ));
        }
    }
}

fn sensor_summary(
    session: &Session,
    name: &str,
    label: &str,
    problems: &mut Problems,
) -> anyhow::Result<()> {
    let contents = session.read_stream(name)?;
    let stream = name;
    let timestamps: Vec<i64> = contents
        .chunks
        .iter()
        .flat_map(|c| c.timestamps_ns())
        .collect();
    println!(
        "{stream}: {} chunks, {} samples",
        contents.chunks.len(),
        timestamps.len()
    );
    report_health(&contents, label, problems);
    if timestamps.len() < 2 {
        return Ok(());
    }
    let mut dts: Vec<i64> = timestamps.windows(2).map(|w| w[1] - w[0]).collect();
    let non_monotonic = dts.iter().filter(|&&dt| dt <= 0).count();
    let duration_s = (timestamps[timestamps.len() - 1] - timestamps[0]) as f64 * 1e-9;
    dts.sort_unstable();
    let median = dts[dts.len() / 2];
    let max = dts[dts.len() - 1];
    let gaps = dts.iter().filter(|&&dt| dt > 2 * median).count();
    let rate = if duration_s > 0.0 {
        format!("{:.1} Hz", (timestamps.len() - 1) as f64 / duration_s)
    } else {
        "? Hz".into()
    };
    println!(
        "  {duration_s:.1} s, mean {rate}, median interval {:.3} ms, max {:.3} ms, {gaps} gaps (> 2x median), {non_monotonic} non-increasing",
        median as f64 * 1e-6,
        max as f64 * 1e-6,
    );
    if non_monotonic > 0 {
        problems.warnings.push(format!(
            "{label}: {non_monotonic} non-increasing timestamps"
        ));
    }
    Ok(())
}

fn record_summary(
    session: &Session,
    name: &str,
    label: &str,
    problems: &mut Problems,
) -> anyhow::Result<()> {
    let contents = session.read_records(name)?;
    let records: Vec<_> = contents.chunks.iter().flat_map(|c| &c.records).collect();
    println!(
        "{name}: {} chunks, {} records",
        contents.chunks.len(),
        records.len()
    );
    report_health(&contents, label, problems);
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
        if let Some(Payload::RecorderError(e)) = &r.payload {
            let first_line = e.message.lines().next().unwrap_or_default();
            problems.warnings.push(format!(
                "{label}: at {:.1} s, the app reported {} failing ({} times so far): {first_line}",
                (r.received_elapsed_realtime_ns - start) as f64 * 1e-9,
                e.source,
                e.count
            ));
        }
    }
    for (kind, n) in counts {
        println!("  {n} {kind}");
    }
    Ok(())
}

fn audio_summary(
    session: &Session,
    name: &str,
    info: Option<&AudioInfo>,
    label: &str,
    problems: &mut Problems,
) -> anyhow::Result<()> {
    let contents = session.read_audio(name)?;
    let rate = info.map_or(0, |i| i.sample_rate_hz);
    let frames: usize = contents.chunks.iter().map(|c| c.pcm_s16le.len() / 2).sum();
    println!("{name}: {} chunks, {frames} frames", contents.chunks.len());
    report_health(&contents, label, problems);
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
    if !contents.is_clean() {
        eprintln!(
            "warning: {stream}: truncated: {}, {} damaged ranges skipped",
            contents.truncated,
            contents.damage.len()
        );
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
        Some(Payload::GnssCapabilities(_)) => "gnss_capabilities",
        Some(Payload::GnssAntennaInfos(_)) => "gnss_antenna_infos",
        Some(Payload::Thermal(_)) => "thermal",
        Some(Payload::BleScan(_)) => "ble_scan",
        Some(Payload::RecorderError(_)) => "recorder_error",
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
    if !contents.is_clean() {
        eprintln!(
            "warning: {stream}: truncated: {}, {} damaged ranges skipped",
            contents.truncated,
            contents.damage.len()
        );
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

// Scratch (PLAN.md items 25 and 42): audio checks and codec benchmark helpers, streaming so a ride needn't fit in memory.
// Usage:
//   audiobench check <session>                      chunk formats, Health reports, levels, channel likeness, float bit grids
//   audiobench export <session> <out.wav> [--int24] [--start S] [--len S]
//                                                   float WAV (or 24-bit PCM, only if every sample is k/2^23) and the SHA-256 of
//                                                   the samples as stored (f32 little-endian bytes)
//   audiobench wavhash <file.wav>                   SHA-256 of a WAV's samples as f32 little-endian bytes
//                                                   (24-bit or 32-bit integer PCM as k/2^23 or k/2^31; 16-bit as k/2^15)
//   audiobench rebuild <session> <decoded.wav> <out dir> [--start S]
//                                                   a copy of the session with its audio samples replaced by the WAV's
//                                                   (for lossy side measurements only), other streams copied
use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufReader, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::Path;

use anyhow::{Context, bail};
use prost::Message;
use ride_session::proto::record::Payload;
use ride_session::read::Session;
use ride_session::{AudioChunk, RecordChunk};
use sha2::{Digest, Sha256};

fn audio_layout(s: &Session) -> anyhow::Result<(usize, u32)> {
    let a = s
        .info
        .streams
        .iter()
        .find(|x| x.name == "audio")
        .and_then(|x| x.audio.clone())
        .context("no audio stream")?;
    Ok((a.channels.max(1) as usize, a.sample_rate_hz as u32))
}

fn chunks(s: &Session) -> anyhow::Result<impl Iterator<Item = anyhow::Result<AudioChunk>>> {
    Ok(s.stream_chunks::<AudioChunk>("audio")?.map(|c| {
        let mut c = c?;
        let had_flac = !c.flac.is_empty();
        if !ride_session::audio_flac::restore(&mut c) {
            bail!("FLAC chunk doesn't decode");
        }
        if had_flac {
            // Mark it so `check` can count it: the field is otherwise empty once restored.
            c.written_elapsed_realtime_ns = -c.written_elapsed_realtime_ns - 1;
        }
        Ok(c)
    }))
}

fn floats(c: &AudioChunk) -> Vec<f32> {
    match c.pcm() {
        ride_session::Pcm::F32(b) => b
            .as_chunks::<4>()
            .0
            .iter()
            .map(|&b| f32::from_le_bytes(b))
            .collect(),
        ride_session::Pcm::S16(b) => b
            .as_chunks::<2>()
            .0
            .iter()
            .map(|&b| f32::from(i16::from_le_bytes(b)) / 32768.0)
            .collect(),
    }
}

fn db(x: f64) -> f64 {
    10.0 * x.max(1e-30).log10()
}

fn median(v: &mut [f64]) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn check(path: &Path) -> anyhow::Result<()> {
    let s = Session::open(path)?;
    let (ch, rate) = audio_layout(&s)?;
    let a = s.info.streams.iter().find(|x| x.name == "audio").unwrap();
    let ai = a.audio.as_ref().unwrap();
    println!(
        "{}: {} Hz, {} channels, source {}, index mask {:#x}, encoding {}, input '{}', refused {:?}",
        path.display(),
        rate,
        ch,
        ai.audio_source_name,
        ai.channel_index_mask,
        ai.encoding,
        ai.input_device,
        ai.refused_formats
    );
    let stored = std::fs::metadata(s.stream_path("audio"))?.len();
    // Health reports.
    if s.stream_path("events").exists() {
        let mut errors: BTreeMap<String, (i64, String)> = BTreeMap::new();
        for c in s.stream_chunks::<RecordChunk>("events")? {
            for r in c?.records {
                if let Some(Payload::RecorderError(e)) = r.payload {
                    let first = e.message.lines().next().unwrap_or("").to_string();
                    let entry = errors.entry(e.source.clone()).or_insert((0, first.clone()));
                    entry.0 = entry.0.max(e.count);
                    if e.source.contains("audio") {
                        println!("  health {}: {} (count {})", e.source, first, e.count);
                    }
                }
            }
        }
        println!(
            "  health sources: {:?}",
            errors
                .iter()
                .map(|(k, v)| (k.as_str(), v.0))
                .collect::<Vec<_>>()
        );
    }
    let (mut n_flac, mut n_f32, mut n_s16, mut n_f32_grid) = (0, 0, 0, 0);
    let mut frames: u64 = 0;
    let mut discont = 0;
    let mut expected = 0i64;
    // Bit grids: the exponent of each nonzero float's lowest set significand bit (its grid step is 2^e).
    let mut step_hist: BTreeMap<i32, u64> = BTreeMap::new();
    let mut zeros = 0u64;
    let mut neg_zeros = 0u64;
    let mut nonfinite = 0u64;
    let mut over1 = 0u64;
    let mut at1 = vec![0u64; ch];
    let mut mantissa_or = 0u32;
    let mut peak = vec![0f64; ch];
    // 10 s windows.
    let win = rate as usize * 10;
    let mut buf: Vec<f32> = Vec::with_capacity(win * ch);
    let mut rms: Vec<Vec<f64>> = vec![Vec::new(); ch];
    let mut corr0 = Vec::new();
    let mut corr_best = Vec::new();
    let mut lag_best = Vec::new();
    let mut level_diff = Vec::new();
    let mut copies = 0;
    let mut silent = vec![0; ch];
    let mut windows = 0;
    let mut window = |buf: &[f32]| {
        let n = buf.len() / ch;
        windows += 1;
        let mut e = vec![0f64; ch];
        for f in buf.chunks_exact(ch) {
            for (c, &v) in f.iter().enumerate() {
                e[c] += f64::from(v) * f64::from(v);
            }
        }
        for c in 0..ch {
            if e[c] == 0.0 {
                silent[c] += 1;
            }
            rms[c].push(db(e[c] / n as f64));
        }
        if ch >= 2 {
            if buf.chunks_exact(ch).all(|f| f[0] == f[1]) {
                copies += 1;
            }
            level_diff.push(db(e[1] / e[0].max(1e-30)));
            // Correlation over the first second, lags -48..=48 (channel 1 later is positive).
            let m = (rate as usize).min(n);
            let x: Vec<f64> = buf
                .chunks_exact(ch)
                .take(m)
                .map(|f| f64::from(f[0]))
                .collect();
            let y: Vec<f64> = buf
                .chunks_exact(ch)
                .take(m)
                .map(|f| f64::from(f[1]))
                .collect();
            let mx = x.iter().sum::<f64>() / m as f64;
            let my = y.iter().sum::<f64>() / m as f64;
            let mut best = (f64::MIN, 0i32);
            let mut c0 = 0.0;
            for lag in -48i32..=48 {
                let (mut sxy, mut sxx, mut syy) = (0.0, 0.0, 0.0);
                for i in 48..m - 48 {
                    let a = x[i] - mx;
                    let b = y[(i as i32 + lag) as usize] - my;
                    sxy += a * b;
                    sxx += a * a;
                    syy += b * b;
                }
                let r = sxy / (sxx * syy).sqrt().max(1e-30);
                if lag == 0 {
                    c0 = r;
                }
                if r > best.0 {
                    best = (r, lag);
                }
            }
            corr0.push(c0);
            corr_best.push(best.0);
            lag_best.push(f64::from(best.1));
        }
    };
    for c in chunks(&s)? {
        let c = c?;
        if c.written_elapsed_realtime_ns < 0 {
            n_flac += 1;
        }
        let v = floats(&c);
        match c.pcm() {
            ride_session::Pcm::F32(_) => {
                n_f32 += 1;
                if v.iter().all(|&x| {
                    let k = x * 8_388_608.0;
                    k.fract() == 0.0
                        && (-8_388_608.0..8_388_608.0).contains(&k)
                        && !(x == 0.0 && x.is_sign_negative())
                }) {
                    n_f32_grid += 1;
                }
                for (i, &x) in v.iter().enumerate() {
                    let bits = x.to_bits();
                    if !x.is_finite() {
                        nonfinite += 1;
                        continue;
                    }
                    if x == 0.0 {
                        zeros += 1;
                        if bits >> 31 == 1 {
                            neg_zeros += 1;
                        }
                        continue;
                    }
                    let e = ((bits >> 23) & 0xff) as i32;
                    let m = bits & 0x7f_ffff;
                    mantissa_or |= m;
                    let (sig, exp) = if e == 0 {
                        (m, -126 - 23)
                    } else {
                        (m | 0x80_0000, e - 127 - 23)
                    };
                    let step = exp + sig.trailing_zeros() as i32;
                    *step_hist.entry(step).or_default() += 1;
                    if x.abs() > 1.0 {
                        over1 += 1;
                    }
                    // Full scale either way: -1.0, or (2^23 - 1)/2^23 as +1.0 isn't on the 24-bit grid.
                    if x.abs() >= 8_388_607.0 / 8_388_608.0 {
                        at1[i % ch] += 1;
                    }
                }
            }
            ride_session::Pcm::S16(_) => n_s16 += 1,
        }
        for (i, &x) in v.iter().enumerate() {
            peak[i % ch] = peak[i % ch].max(f64::from(x.abs()));
        }
        let f = (v.len() / ch) as u64;
        if c.first_frame != expected {
            discont += 1;
        }
        expected = c.first_frame + f as i64;
        frames += f;
        for fr in v.chunks_exact(ch) {
            buf.extend_from_slice(fr);
            if buf.len() == win * ch {
                window(&buf);
                buf.clear();
            }
        }
    }
    let hours = frames as f64 / f64::from(rate) / 3600.0;
    println!(
        "  {frames} frames = {:.4} h; chunks: {n_flac} were FLAC, {n_f32} float ({n_f32_grid} all k/2^23), {n_s16} 16-bit; {discont} discontinuities",
        hours
    );
    println!(
        "  stored audio.zst {:.1} MB = {:.1} MB/h ({:.2} bytes a sample)",
        stored as f64 / 1e6,
        stored as f64 / 1e6 / hours,
        stored as f64 / (frames * ch as u64) as f64
    );
    let total: u64 = step_hist.values().sum();
    println!(
        "  floats: {zeros} zeros ({neg_zeros} negative), {nonfinite} non-finite, {over1} over 1.0, at/over full scale per channel {at1:?}, mantissa bits ever set {:#08x}",
        mantissa_or
    );
    println!("  grid step 2^e of nonzero floats (e: share):");
    let mut coarser24 = 0;
    for (e, n) in &step_hist {
        if *e >= -23 {
            coarser24 += n;
        }
        if *n as f64 / total as f64 > 1e-5 {
            println!("    {e}: {:.5}", *n as f64 / total as f64);
        }
    }
    println!(
        "  on the 2^-23 grid or coarser: {:.6} of nonzero floats; finest step 2^{}",
        coarser24 as f64 / total.max(1) as f64,
        step_hist.keys().next().copied().unwrap_or(0)
    );
    for c in 0..ch {
        println!(
            "  channel {c}: median {:.1} dBFS over {} windows, {} silent, peak {:.3}",
            median(&mut rms[c].clone()),
            windows,
            silent[c],
            peak[c]
        );
    }
    if ch >= 2 {
        let mut cb = corr_best.clone();
        cb.sort_by(f64::total_cmp);
        println!(
            "  best correlation per window: min {:.2}, 10th {:.2}, 90th {:.2}",
            cb.first().copied().unwrap_or(f64::NAN),
            cb.get(cb.len() / 10).copied().unwrap_or(f64::NAN),
            cb.get(cb.len() * 9 / 10).copied().unwrap_or(f64::NAN)
        );
        println!(
            "  channel 1 - 0 level median {:.1} dB; exact copies {copies}/{windows}; correlation at lag 0 median {:.2}, best median {:.2} at lag median {:.0} (lag 10th..90th {:.0}..{:.0})",
            median(&mut level_diff),
            median(&mut corr0.clone()),
            median(&mut corr_best.clone()),
            median(&mut lag_best.clone()),
            {
                let mut l = lag_best.clone();
                l.sort_by(f64::total_cmp);
                l.get(l.len() / 10).copied().unwrap_or(f64::NAN)
            },
            {
                let mut l = lag_best.clone();
                l.sort_by(f64::total_cmp);
                l.get(l.len() * 9 / 10).copied().unwrap_or(f64::NAN)
            },
        );
    }
    Ok(())
}

fn arg_f64(args: &[String], name: &str) -> Option<f64> {
    args.iter()
        .position(|a| a == name)
        .map(|i| args[i + 1].parse().unwrap())
}

fn wav_header(
    out: &mut impl Write,
    ch: u16,
    rate: u32,
    int24: bool,
    data_len: u32,
) -> std::io::Result<()> {
    let (fmt, bytes) = if int24 { (1u16, 3u16) } else { (3, 4) };
    out.write_all(b"RIFF")?;
    out.write_all(&(4 + 8 + 16 + 8 + data_len).to_le_bytes())?;
    out.write_all(b"WAVEfmt ")?;
    out.write_all(&16u32.to_le_bytes())?;
    out.write_all(&fmt.to_le_bytes())?;
    out.write_all(&ch.to_le_bytes())?;
    out.write_all(&rate.to_le_bytes())?;
    out.write_all(&(rate * u32::from(ch) * u32::from(bytes)).to_le_bytes())?;
    out.write_all(&(ch * bytes).to_le_bytes())?;
    out.write_all(&(bytes * 8).to_le_bytes())?;
    out.write_all(b"data")?;
    out.write_all(&data_len.to_le_bytes())
}

fn export(path: &Path, out_path: &Path, args: &[String]) -> anyhow::Result<()> {
    let s = Session::open(path)?;
    let (ch, rate) = audio_layout(&s)?;
    let int24 = args.iter().any(|a| a == "--int24");
    let start = (arg_f64(args, "--start").unwrap_or(0.0) * f64::from(rate)) as u64;
    let len = arg_f64(args, "--len").map(|l| (l * f64::from(rate)) as u64);
    let mut out = BufWriter::new(File::create(out_path)?);
    wav_header(&mut out, ch as u16, rate, int24, 0)?;
    let mut hash = Sha256::new();
    let mut frame: u64 = 0;
    let mut written: u64 = 0;
    'outer: for c in chunks(&s)? {
        let c = c?;
        let raw: Vec<u8> = match c.pcm() {
            ride_session::Pcm::F32(b) => b.to_vec(),
            ride_session::Pcm::S16(_) => bail!("16-bit chunk"),
        };
        for f in raw.chunks_exact(4 * ch) {
            if frame >= start {
                if len.is_some_and(|l| written >= l) {
                    break 'outer;
                }
                hash.update(f);
                if int24 {
                    for b in f.as_chunks::<4>().0 {
                        let x = f32::from_le_bytes(*b);
                        let k = x * 8_388_608.0;
                        if k.fract() != 0.0
                            || !(-8_388_608.0..8_388_608.0).contains(&k)
                            || (x == 0.0 && x.is_sign_negative())
                        {
                            bail!("sample {x} at frame {frame} isn't k/2^23");
                        }
                        out.write_all(&(k as i32).to_le_bytes()[..3])?;
                    }
                } else {
                    out.write_all(f)?;
                }
                written += 1;
            }
            frame += 1;
        }
    }
    let data_len = u32::try_from(written * ch as u64 * if int24 { 3 } else { 4 })
        .context("too long for WAV")?;
    let mut file = out.into_inner()?;
    file.seek(SeekFrom::Start(0))?;
    wav_header(&mut file, ch as u16, rate, int24, data_len)?;
    println!(
        "{} frames ({:.1} s) {}",
        written,
        written as f64 / f64::from(rate),
        hex(&hash.finalize())
    );
    Ok(())
}

/// A WAV's format and its samples, streamed as f32 frames to `each` (a block at a time).
fn read_wav(
    path: &Path,
    mut each: impl FnMut(&[f32], &[u8]) -> anyhow::Result<()>,
) -> anyhow::Result<(u16, u32, u16, u16)> {
    let mut r = BufReader::with_capacity(1 << 20, File::open(path)?);
    let mut head = [0u8; 12];
    r.read_exact(&mut head)?;
    if &head[0..4] != b"RIFF" || &head[8..12] != b"WAVE" {
        bail!("not a WAV");
    }
    let (mut fmt, mut ch, mut rate, mut bits) = (0u16, 0u16, 0u32, 0u16);
    loop {
        let mut h = [0u8; 8];
        r.read_exact(&mut h)?;
        let len = u32::from_le_bytes(h[4..8].try_into().unwrap());
        if &h[0..4] == b"fmt " {
            let mut b = vec![0u8; len as usize];
            r.read_exact(&mut b)?;
            fmt = u16::from_le_bytes([b[0], b[1]]);
            ch = u16::from_le_bytes([b[2], b[3]]);
            rate = u32::from_le_bytes(b[4..8].try_into().unwrap());
            bits = u16::from_le_bytes([b[14], b[15]]);
            if fmt == 0xfffe {
                fmt = u16::from_le_bytes([b[24], b[25]]);
            }
            if len % 2 == 1 {
                r.read_exact(&mut [0u8; 1])?;
            }
        } else if &h[0..4] == b"data" {
            let bytes = usize::from(bits / 8);
            let frame = bytes * usize::from(ch);
            // A streamed WAV may give 0 or 0xFFFFFFFF as the length: then read to the end.
            let mut left = if len == 0 || len == u32::MAX {
                u64::MAX
            } else {
                u64::from(len)
            };
            let mut block = vec![0u8; frame * 4096];
            let mut out = Vec::with_capacity(4096 * usize::from(ch));
            loop {
                let want = (block.len() as u64).min(left) as usize;
                let mut got = 0;
                while got < want {
                    let n = r.read(&mut block[got..want])?;
                    if n == 0 {
                        break;
                    }
                    got += n;
                }
                let got = got / frame * frame;
                if got == 0 {
                    break;
                }
                left -= got as u64;
                out.clear();
                for s in block[..got].chunks_exact(bytes) {
                    out.push(match (fmt, bits) {
                        (3, 32) => f32::from_le_bytes(s.try_into().unwrap()),
                        (1, 16) => f32::from(i16::from_le_bytes(s.try_into().unwrap())) / 32768.0,
                        (1, 24) => {
                            (i32::from_le_bytes([0, s[0], s[1], s[2]]) >> 8) as f32 / 8_388_608.0
                        }
                        (1, 32) => {
                            (f64::from(i32::from_le_bytes(s.try_into().unwrap())) / 2_147_483_648.0)
                                as f32
                        }
                        _ => bail!("format {fmt} {bits}-bit unsupported"),
                    });
                }
                each(&out, &block[..got])?;
            }
            return Ok((fmt, rate, ch, bits));
        } else {
            std::io::copy(
                &mut (&mut r).take(u64::from(len + len % 2)),
                &mut std::io::sink(),
            )?;
        }
    }
}

fn wavhash(path: &Path) -> anyhow::Result<()> {
    let mut hash = Sha256::new();
    let mut samples = 0u64;
    let mut ch = 1;
    let (fmt, rate, c, bits) = read_wav(path, |v, _| {
        for x in v {
            hash.update(x.to_le_bytes());
        }
        samples += v.len() as u64;
        Ok(())
    })?;
    ch = ch.max(c);
    println!(
        "{} frames ({:.1} s, format {fmt} {bits}-bit, {ch} ch, {rate} Hz) {}",
        samples / u64::from(ch),
        samples as f64 / f64::from(ch) / f64::from(rate),
        hex(&hash.finalize())
    );
    Ok(())
}

/// Lossy side measurement only: the session with its audio samples taken from a decoded WAV, the decoded audio's frame
/// `i` replacing the session's frame `start + i`; frames before `start` or past the WAV keep the originals.
fn rebuild(path: &Path, wav: &Path, out_dir: &Path, args: &[String]) -> anyhow::Result<()> {
    let s = Session::open(path)?;
    let (ch, rate) = audio_layout(&s)?;
    let start = (arg_f64(args, "--start").unwrap_or(0.0) * f64::from(rate)) as u64;
    std::fs::create_dir(out_dir)?;
    for e in std::fs::read_dir(path)? {
        let e = e?;
        let name = e.file_name();
        if name == "audio.zst" {
            continue;
        }
        if e.file_type()?.is_dir() {
            std::fs::create_dir(out_dir.join(&name))?;
            for f in std::fs::read_dir(e.path())? {
                let f = f?;
                if f.file_type()?.is_file() {
                    std::fs::copy(f.path(), out_dir.join(&name).join(f.file_name()))?;
                }
            }
        } else {
            std::fs::copy(e.path(), out_dir.join(&name))?;
        }
    }
    // The decoded samples, streamed in step with the chunks through a channel from a reader thread.
    let (tx, rx) = std::sync::mpsc::sync_channel::<Vec<f32>>(16);
    let wav = wav.to_path_buf();
    let reader = std::thread::spawn(move || {
        read_wav(&wav, |v, _| {
            tx.send(v.to_vec()).ok();
            Ok(())
        })
    });
    let mut pending: std::collections::VecDeque<f32> = Default::default();
    let mut source_done = false;
    let mut next = |n: usize, pending: &mut std::collections::VecDeque<f32>| -> Vec<f32> {
        while pending.len() < n && !source_done {
            match rx.recv() {
                Ok(v) => pending.extend(v),
                Err(_) => source_done = true,
            }
        }
        let k = n.min(pending.len());
        pending.drain(..k).collect()
    };
    let file = File::create(out_dir.join("audio.zst"))?;
    let mut enc = zstd::stream::write::Encoder::new(BufWriter::new(file), 3)?;
    let mut frame: u64 = 0;
    let mut replaced: u64 = 0;
    for c in chunks(&s)? {
        let mut c = c?;
        if c.written_elapsed_realtime_ns < 0 {
            c.written_elapsed_realtime_ns = -(c.written_elapsed_realtime_ns + 1);
        }
        let mut v = floats(&c);
        let nf = v.len() / ch;
        let skip = start.saturating_sub(frame).min(nf as u64) as usize;
        let take = nf - skip;
        let new = next(take * ch, &mut pending);
        v[skip * ch..skip * ch + new.len()].copy_from_slice(&new);
        replaced += (new.len() / ch) as u64;
        c.pcm_s16le.clear();
        c.flac.clear();
        c.pcm_f32le = v.iter().flat_map(|x| x.to_le_bytes()).collect();
        let bytes = c.encode_to_vec();
        let mut d = Vec::new();
        prost::encoding::encode_varint(bytes.len() as u64, &mut d);
        enc.write_all(&d)?;
        enc.write_all(&bytes)?;
        frame += nf as u64;
    }
    enc.finish()?.flush()?;
    drop(rx);
    reader.join().unwrap().ok();
    println!("{replaced} of {frame} frames replaced");
    Ok(())
}

/// Today's storage of a WAV's samples: their f32 bytes as one zstd frame (as post-ride recompression writes a stream),
/// written to `out`, timed, then decoded back and hashed.
fn zstd_bench(wav: &Path, level: i32, out: &Path) -> anyhow::Result<()> {
    let t = std::time::Instant::now();
    let mut enc = zstd::stream::write::Encoder::new(BufWriter::new(File::create(out)?), level)?;
    enc.include_checksum(true)?;
    let mut samples = 0u64;
    let mut rate = 0;
    let mut ch = 1;
    let (_, r, c, _) = read_wav(wav, |v, _| {
        for x in v {
            enc.write_all(&x.to_le_bytes())?;
        }
        samples += v.len() as u64;
        Ok(())
    })?;
    rate = rate.max(r);
    ch = ch.max(c);
    enc.finish()?.flush()?;
    let enc_s = t.elapsed().as_secs_f64();
    let t = std::time::Instant::now();
    let mut dec = zstd::stream::read::Decoder::new(File::open(out)?)?;
    let mut hash = Sha256::new();
    let mut buf = vec![0u8; 1 << 20];
    loop {
        let n = dec.read(&mut buf)?;
        if n == 0 {
            break;
        }
        hash.update(&buf[..n]);
    }
    let dec_s = t.elapsed().as_secs_f64();
    let secs = samples as f64 / f64::from(ch) / f64::from(rate);
    let size = std::fs::metadata(out)?.len();
    println!(
        "zstd -{level}: {size} bytes, {:.1} MB/h, encode {enc_s:.1} s ({:.1}x realtime), decode {dec_s:.1} s ({:.0}x), {}",
        size as f64 / 1e6 / (secs / 3600.0),
        secs / enc_s,
        secs / dec_s,
        hex(&hash.finalize())
    );
    Ok(())
}

/// A `zstd` file of f32 samples back to a float WAV (2 channels, 48 kHz, as the rides record).
fn zstd_dec(zst: &Path, out_path: &Path) -> anyhow::Result<()> {
    let mut dec = zstd::stream::read::Decoder::new(File::open(zst)?)?;
    let mut out = BufWriter::new(File::create(out_path)?);
    wav_header(&mut out, 2, 48_000, false, 0)?;
    let n = std::io::copy(&mut dec, &mut out)?;
    let mut file = out.into_inner()?;
    file.seek(SeekFrom::Start(0))?;
    wav_header(&mut file, 2, 48_000, false, u32::try_from(n)?)?;
    Ok(())
}

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("check") => {
            for p in &args[2..] {
                check(Path::new(p))?;
            }
            Ok(())
        }
        Some("export") => export(Path::new(&args[2]), Path::new(&args[3]), &args[4..]),
        Some("wavhash") => wavhash(Path::new(&args[2])),
        Some("zstd-dec") => zstd_dec(Path::new(&args[2]), Path::new(&args[3])),
        Some("zstd") => zstd_bench(Path::new(&args[2]), args[3].parse()?, Path::new(&args[4])),
        Some("rebuild") => rebuild(
            Path::new(&args[2]),
            Path::new(&args[3]),
            Path::new(&args[4]),
            &args[5..],
        ),
        _ => bail!("usage: audiobench check|export|wavhash|rebuild ..."),
    }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

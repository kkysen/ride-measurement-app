// Scratch (PLAN.md item 42): bit-exact lossless encodings of each non-audio stream vs the current post-ride format.
// Usage: compbench <session dir>... ; prints one line per (stream, candidate), and checks every round trip.
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::Instant;

use prost::Message;
use ride_session::proto::record::Payload;
use ride_session::{RecordChunk, SensorChunk};
use zstd::zstd_safe::CParameter;

const SENSORS: &[&str] = &[
    "accel_uncal",
    "gyro_uncal",
    "mag_uncal",
    "pressure",
    "light",
];
const RECORDS: &[&str] = &["radio", "gnss", "location", "transit"];

// ---------- generic compressors ----------

#[derive(Clone, Copy)]
enum Comp {
    Zstd(i32, bool),
    Xz,
}

impl Comp {
    fn name(self) -> String {
        match self {
            Comp::Zstd(l, false) => format!("zstd-{l}"),
            Comp::Zstd(l, true) => format!("zstd-{l}-long"),
            Comp::Xz => "xz-6e".into(),
        }
    }
    fn compress(self, data: &[u8]) -> Vec<u8> {
        match self {
            Comp::Zstd(level, long) => {
                let mut c = zstd::bulk::Compressor::new(level).unwrap();
                if long {
                    c.set_parameter(CParameter::EnableLongDistanceMatching(true))
                        .unwrap();
                    c.set_parameter(CParameter::WindowLog(27)).unwrap();
                }
                c.compress(data).unwrap()
            }
            Comp::Xz => pipe(&["-6e", "-c", "-T1"], data),
        }
    }
    fn decompress(self, data: &[u8], len: usize) -> Vec<u8> {
        match self {
            Comp::Zstd(..) => {
                let _ = len;
                let mut d = zstd::stream::Decoder::new(data).unwrap();
                d.window_log_max(31).unwrap();
                let mut out = Vec::new();
                d.read_to_end(&mut out).unwrap();
                out
            }
            Comp::Xz => pipe(&["-d", "-c", "-T1"], data),
        }
    }
}

fn pipe(args: &[&str], data: &[u8]) -> Vec<u8> {
    let mut child = Command::new("xz")
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let mut stdin = child.stdin.take().unwrap();
    let input = data.to_vec();
    let writer = std::thread::spawn(move || stdin.write_all(&input).unwrap());
    let mut out = Vec::new();
    child.stdout.take().unwrap().read_to_end(&mut out).unwrap();
    writer.join().unwrap();
    assert!(child.wait().unwrap().success());
    out
}

const COMPS: &[Comp] = &[
    Comp::Zstd(19, false),
    Comp::Zstd(22, false),
    Comp::Zstd(22, true),
    Comp::Xz,
];

// ---------- varints ----------

fn put_varint(out: &mut Vec<u8>, mut v: u64) {
    while v >= 0x80 {
        out.push(v as u8 | 0x80);
        v >>= 7;
    }
    out.push(v as u8);
}
fn get_varint(b: &[u8], pos: &mut usize) -> u64 {
    let mut v = 0u64;
    let mut shift = 0;
    loop {
        let byte = b[*pos];
        *pos += 1;
        v |= u64::from(byte & 0x7f) << shift;
        if byte & 0x80 == 0 {
            return v;
        }
        shift += 7;
    }
}
fn zz(v: i64) -> u64 {
    ((v << 1) ^ (v >> 63)) as u64
}
fn unzz(v: u64) -> i64 {
    ((v >> 1) as i64) ^ -((v & 1) as i64)
}

/// The decompressed stream (length-delimited messages) split into message bodies.
fn split_delimited(raw: &[u8]) -> Vec<&[u8]> {
    let mut pos = 0;
    let mut out = Vec::new();
    while pos < raw.len() {
        let len = get_varint(raw, &mut pos) as usize;
        out.push(&raw[pos..pos + len]);
        pos += len;
    }
    out
}
fn join_delimited(msgs: impl IntoIterator<Item = Vec<u8>>) -> Vec<u8> {
    let mut out = Vec::new();
    for m in msgs {
        put_varint(&mut out, m.len() as u64);
        out.extend_from_slice(&m);
    }
    out
}

// ---------- columnar sensor encoding ----------

#[derive(Clone, Copy, PartialEq)]
enum Values {
    SubLe,
    Rank,
}

fn columnar_encode(chunks: &[SensorChunk], mode: Values) -> Vec<u8> {
    let vps = chunks.first().map_or(1, |c| c.values_per_sample) as usize;
    let mut head = Vec::new();
    put_varint(&mut head, chunks.len() as u64);
    put_varint(&mut head, vps as u64);
    let mut ts = Vec::new();
    let mut acc = Vec::new();
    let mut vals: Vec<u32> = Vec::new();
    let (mut prev_t, mut prev_d, mut prev_w, mut prev_a) = (0i64, 0i64, 0i64, 0i64);
    for c in chunks {
        assert_eq!(c.values_per_sample as usize, vps);
        put_varint(&mut head, c.timestamp_deltas_ns.len() as u64);
        put_varint(&mut head, c.value_bits.len() as u64);
        put_varint(&mut head, c.accuracy.len() as u64);
        put_varint(
            &mut head,
            zz(c.written_elapsed_realtime_ns.wrapping_sub(prev_w)),
        );
        prev_w = c.written_elapsed_realtime_ns;
        let mut t = 0i64;
        for (i, &d) in c.timestamp_deltas_ns.iter().enumerate() {
            t = if i == 0 { d } else { t.wrapping_add(d) };
            let delta = t.wrapping_sub(prev_t);
            put_varint(&mut ts, zz(delta.wrapping_sub(prev_d)));
            prev_d = delta;
            prev_t = t;
        }
        for &a in &c.accuracy {
            put_varint(&mut acc, zz(i64::from(a) - prev_a));
            prev_a = i64::from(a);
        }
        vals.extend_from_slice(&c.value_bits);
    }
    let n = vals.len();
    let samples = n / vps.max(1);
    let mut vb = Vec::new();
    for ch in 0..vps {
        let col: Vec<u32> = (0..samples).map(|s| vals[s * vps + ch]).collect();
        match mode {
            Values::SubLe => {
                let mut p = 0u32;
                for &v in &col {
                    vb.extend_from_slice(&v.wrapping_sub(p).to_le_bytes());
                    p = v;
                }
            }
            Values::Rank => {
                let mut tab = col.clone();
                tab.sort_by(|a, b| f32::from_bits(*a).total_cmp(&f32::from_bits(*b)));
                tab.dedup();
                put_varint(&mut vb, tab.len() as u64);
                let mut p = 0u32;
                for &t in &tab {
                    put_varint(&mut vb, u64::from(t.wrapping_sub(p)));
                    p = t;
                }
                let mut pr = 0i64;
                for &v in &col {
                    let r = tab
                        .binary_search_by(|t| f32::from_bits(*t).total_cmp(&f32::from_bits(v)))
                        .unwrap() as i64;
                    put_varint(&mut vb, zz(r - pr));
                    pr = r;
                }
            }
        }
    }
    let mut out = Vec::with_capacity(head.len() + ts.len() + acc.len() + vb.len() + 30);
    for sec in [&head, &ts, &acc] {
        put_varint(&mut out, sec.len() as u64);
    }
    put_varint(&mut out, n as u64);
    out.extend_from_slice(&head);
    out.extend_from_slice(&ts);
    out.extend_from_slice(&acc);
    out.extend_from_slice(&vb);
    out
}

fn columnar_decode(b: &[u8], mode: Values) -> Vec<SensorChunk> {
    let mut pos = 0;
    let hl = get_varint(b, &mut pos) as usize;
    let tl = get_varint(b, &mut pos) as usize;
    let al = get_varint(b, &mut pos) as usize;
    let n = get_varint(b, &mut pos) as usize;
    let head = &b[pos..pos + hl];
    let ts = &b[pos + hl..pos + hl + tl];
    let acc = &b[pos + hl + tl..pos + hl + tl + al];
    let planes = &b[pos + hl + tl + al..];
    let mut hp = 0;
    let nchunks = get_varint(head, &mut hp) as usize;
    let vps = get_varint(head, &mut hp) as usize;
    let samples = n / vps.max(1);
    let mut vals = vec![0u32; n];
    let mut pp = 0;
    for ch in 0..vps {
        match mode {
            Values::SubLe => {
                let mut p = 0u32;
                for s in 0..samples {
                    let d = u32::from_le_bytes(planes[pp..pp + 4].try_into().unwrap());
                    pp += 4;
                    p = p.wrapping_add(d);
                    vals[s * vps + ch] = p;
                }
            }
            Values::Rank => {
                let tl = get_varint(planes, &mut pp) as usize;
                let mut tab = Vec::with_capacity(tl);
                let mut p = 0u32;
                for _ in 0..tl {
                    p = p.wrapping_add(get_varint(planes, &mut pp) as u32);
                    tab.push(p);
                }
                let mut r = 0i64;
                for s in 0..samples {
                    r += unzz(get_varint(planes, &mut pp));
                    vals[s * vps + ch] = tab[r as usize];
                }
            }
        }
    }
    let (mut tp, mut ap, mut vp) = (0, 0, 0);
    let (mut prev_t, mut prev_d, mut prev_w, mut prev_a) = (0i64, 0i64, 0i64, 0i64);
    let mut chunks = Vec::with_capacity(nchunks);
    for _ in 0..nchunks {
        let nt = get_varint(head, &mut hp) as usize;
        let nv = get_varint(head, &mut hp) as usize;
        let na = get_varint(head, &mut hp) as usize;
        let w = prev_w.wrapping_add(unzz(get_varint(head, &mut hp)));
        prev_w = w;
        let mut deltas = Vec::with_capacity(nt);
        for i in 0..nt {
            let d = prev_d.wrapping_add(unzz(get_varint(ts, &mut tp)));
            let t = prev_t.wrapping_add(d);
            deltas.push(if i == 0 { t } else { d });
            prev_d = d;
            prev_t = t;
        }
        let mut accuracy = Vec::with_capacity(na);
        for _ in 0..na {
            let a = prev_a + unzz(get_varint(acc, &mut ap));
            accuracy.push(a as i32);
            prev_a = a;
        }
        chunks.push(SensorChunk {
            timestamp_deltas_ns: deltas,
            values_per_sample: vps as u32,
            value_bits: vals[vp..vp + nv].to_vec(),
            accuracy,
            written_elapsed_realtime_ns: w,
        });
        vp += nv;
    }
    chunks
}

// ---------- transit: diffs and dictionaries ----------

fn gtfs_mut(
    c: &mut RecordChunk,
) -> impl Iterator<Item = &mut ride_session::proto::GtfsRealtimeFeed> {
    c.records.iter_mut().filter_map(|r| match &mut r.payload {
        Some(Payload::GtfsRealtime(g)) => Some(g),
        _ => None,
    })
}

/// Each poll's body replaced by a `zstd` patch against the previous body from the same URL (raw-content dictionary).
fn transit_diff_encode(raw: &[u8]) -> Vec<u8> {
    let mut prev: std::collections::HashMap<String, Vec<u8>> = Default::default();
    let mut msgs = Vec::new();
    for m in split_delimited(raw) {
        let mut c = RecordChunk::decode(m).unwrap();
        for g in gtfs_mut(&mut c) {
            let body = std::mem::take(&mut g.body);
            g.body = match prev.get(&g.url) {
                Some(p) => zstd::bulk::Compressor::with_dictionary(19, p)
                    .unwrap()
                    .compress(&body)
                    .unwrap(),
                None => body.clone(),
            };
            prev.insert(g.url.clone(), body);
        }
        msgs.push(c.encode_to_vec());
    }
    join_delimited(msgs)
}
fn transit_diff_decode(raw: &[u8]) -> Vec<u8> {
    let mut prev: std::collections::HashMap<String, Vec<u8>> = Default::default();
    let mut msgs = Vec::new();
    for m in split_delimited(raw) {
        let mut c = RecordChunk::decode(m).unwrap();
        for g in gtfs_mut(&mut c) {
            if let Some(p) = prev.get(&g.url) {
                g.body = zstd::bulk::Decompressor::with_dictionary(p)
                    .unwrap()
                    .decompress(&g.body, 64 << 20)
                    .unwrap();
            }
            prev.insert(g.url.clone(), g.body.clone());
        }
        msgs.push(c.encode_to_vec());
    }
    join_delimited(msgs)
}
/// Each poll's body compressed alone with a dictionary trained on another ride's polls.
fn transit_dict_encode(raw: &[u8], dict: &[u8]) -> Vec<u8> {
    let mut comp = zstd::bulk::Compressor::with_dictionary(19, dict).unwrap();
    let mut msgs = Vec::new();
    for m in split_delimited(raw) {
        let mut c = RecordChunk::decode(m).unwrap();
        for g in gtfs_mut(&mut c) {
            g.body = comp.compress(&g.body).unwrap();
        }
        msgs.push(c.encode_to_vec());
    }
    join_delimited(msgs)
}
fn transit_dict_decode(raw: &[u8], dict: &[u8]) -> Vec<u8> {
    let mut d = zstd::bulk::Decompressor::with_dictionary(dict).unwrap();
    let mut msgs = Vec::new();
    for m in split_delimited(raw) {
        let mut c = RecordChunk::decode(m).unwrap();
        for g in gtfs_mut(&mut c) {
            g.body = d.decompress(&g.body, 64 << 20).unwrap();
        }
        msgs.push(c.encode_to_vec());
    }
    join_delimited(msgs)
}
fn transit_bodies(raw: &[u8], cap: usize) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let mut total = 0;
    for m in split_delimited(raw) {
        let mut c = RecordChunk::decode(m).unwrap();
        for g in gtfs_mut(&mut c) {
            if total < cap {
                total += g.body.len();
                out.push(std::mem::take(&mut g.body));
            }
        }
    }
    out
}

// ---------- driver ----------

fn read_raw(path: &Path) -> Vec<u8> {
    let f = std::fs::File::open(path).unwrap();
    let mut d = zstd::stream::Decoder::new(f).unwrap();
    d.window_log_max(31).unwrap();
    let mut out = Vec::new();
    d.read_to_end(&mut out).unwrap();
    out
}

fn report(
    session: &str,
    stream: &str,
    cand: &str,
    raw: usize,
    size: usize,
    hours: f64,
    enc_s: f64,
    dec_s: f64,
    now: usize,
) {
    let mb = raw as f64 / 1e6;
    println!(
        "{session}\t{stream}\t{cand}\t{size}\t{:.3}x_vs_now\t{:.3}MB/h\tenc {:.1} MB/s\tdec {:.1} MB/s",
        now as f64 / size as f64,
        size as f64 / 1e6 / hours,
        mb / enc_s,
        mb / dec_s,
    );
}

/// Runs a candidate: encode, decode, check bit-exact against `raw`, report.
fn run(
    session: &str,
    stream: &str,
    cand: &str,
    raw: &[u8],
    hours: f64,
    now: usize,
    enc: impl Fn() -> Vec<u8>,
    dec: impl Fn(&[u8]) -> Vec<u8>,
) {
    let t = Instant::now();
    let e = enc();
    let enc_s = t.elapsed().as_secs_f64();
    let t = Instant::now();
    let d = dec(&e);
    let dec_s = t.elapsed().as_secs_f64();
    assert!(d == raw, "{session} {stream} {cand}: round trip differs");
    report(
        session,
        stream,
        cand,
        raw.len(),
        e.len(),
        hours,
        enc_s,
        dec_s,
        now,
    );
}

fn diag(path: &Path) {
    let raw = read_raw(path);
    let chunks: Vec<SensorChunk> = split_delimited(&raw)
        .into_iter()
        .map(|m| SensorChunk::decode(m).unwrap())
        .collect();
    drop(raw);
    let vps = chunks[0].values_per_sample as usize;
    let z = |b: &[u8]| zstd::bulk::compress(b, 19).unwrap().len();
    let mut ts = Vec::new();
    for c in &chunks {
        let mut t = 0i64;
        for (i, &d) in c.timestamp_deltas_ns.iter().enumerate() {
            t = if i == 0 { d } else { t + d };
            ts.push(t);
        }
    }
    let (mut d1, mut d2, mut p, mut pd) = (Vec::new(), Vec::new(), 0i64, 0i64);
    for &t in &ts {
        put_varint(&mut d1, zz(t - p));
        put_varint(&mut d2, zz(t - p - pd));
        pd = t - p;
        p = t;
    }
    println!(
        "vps {vps} samples {} chunks {} ts-delta {} ts-dod {}",
        ts.len(),
        chunks.len(),
        z(&d1),
        z(&d2)
    );
    println!(
        "first deltas {:?}",
        &ts.windows(2)
            .map(|w| w[1] - w[0])
            .take(12)
            .collect::<Vec<_>>()
    );
    let vals: Vec<u32> = chunks
        .iter()
        .flat_map(|c| c.value_bits.iter().copied())
        .collect();
    let n = vals.len() / vps;
    for ch in 0..vps {
        let col: Vec<u32> = (0..n).map(|s| vals[s * vps + ch]).collect();
        let mut line = format!("ch{ch} first {:?}", f32::from_bits(col[0]));
        for (name, f) in [("raw", 0), ("xor", 1), ("sub", 2)] {
            let x: Vec<u32> = (0..n)
                .map(|i| {
                    let q = if i > 0 { col[i - 1] } else { 0 };
                    match f {
                        0 => col[i],
                        1 => col[i] ^ q,
                        _ => col[i].wrapping_sub(q),
                    }
                })
                .collect();
            let le: Vec<u8> = x.iter().flat_map(|v| v.to_le_bytes()).collect();
            let mut sh = Vec::new();
            for b in 0..4 {
                sh.extend(x.iter().map(|v| (v >> (8 * b)) as u8));
            }
            let planes: Vec<usize> = (0..4).map(|b| z(&sh[b * n..(b + 1) * n])).collect();
            line += &format!(
                " | {name} le {} shuf {} planes {:?}",
                z(&le),
                z(&sh),
                planes
            );
        }
        println!("{line}");
        let mut u: Vec<f32> = col
            .iter()
            .map(|&b| f32::from_bits(b))
            .filter(|v| v.is_finite())
            .collect();
        u.sort_by(|a, b| a.partial_cmp(b).unwrap());
        u.dedup();
        let q = u
            .windows(2)
            .map(|w| w[1] - w[0])
            .fold(f32::INFINITY, f32::min);
        let ok32 = col
            .iter()
            .filter(|&&b| {
                let v = f32::from_bits(b);
                ((v / q).round() * q).to_bits() == b
            })
            .count();
        let ok64 = col
            .iter()
            .filter(|&&b| {
                let v = f32::from_bits(b) as f64;
                (((v / q as f64).round() * q as f64) as f32).to_bits() == b
            })
            .count();
        let mut tab: Vec<u32> = col.clone();
        tab.sort_by(|a, b| f32::from_bits(*a).total_cmp(&f32::from_bits(*b)));
        tab.dedup();
        let mut rk = Vec::new();
        let mut rk16 = Vec::new();
        let mut pr = 0i64;
        for &b in &col {
            let r = tab
                .binary_search_by(|t| f32::from_bits(*t).total_cmp(&f32::from_bits(b)))
                .unwrap() as i64;
            put_varint(&mut rk, zz(r - pr));
            rk16.extend_from_slice(&((r - pr) as i16).to_le_bytes());
            pr = r;
        }
        let mut tb = Vec::new();
        let mut pt = 0u32;
        for &t in &tab {
            put_varint(&mut tb, u64::from(t.wrapping_sub(pt)));
            pt = t;
        }
        println!(
            "  rank-delta varint {} i16 {} table {}",
            z(&rk),
            z(&rk16),
            z(&tb)
        );
        println!(
            "  unique {} min step {q:e} grid-exact f32 {ok32}/{n} f64 {ok64}/{n}",
            u.len()
        );
    }
}

fn main() {
    if let Ok(p) = std::env::var("DIAG") {
        diag(Path::new(&p));
        return;
    }
    let dirs: Vec<PathBuf> = std::env::args().skip(1).map(PathBuf::from).collect();
    // Hours of each ride, from the barometer's first and last timestamps.
    let hours: Vec<f64> = dirs
        .iter()
        .map(|d| {
            let raw = read_raw(&d.join("pressure.zst"));
            let mut ts = Vec::new();
            for m in split_delimited(&raw) {
                let c = SensorChunk::decode(m).unwrap();
                let mut t = 0;
                for (i, &x) in c.timestamp_deltas_ns.iter().enumerate() {
                    t = if i == 0 { x } else { t + x };
                    ts.push(t);
                }
            }
            (ts.iter().max().unwrap() - ts.iter().min().unwrap()) as f64 / 3.6e12
        })
        .collect();
    // Transit dictionaries: each ride's is trained on the previous ride's polls (the last's for the first), so out of sample.
    let dicts: Vec<Vec<u8>> = (0..dirs.len())
        .map(|i| {
            let other = &dirs[(i + dirs.len() - 1) % dirs.len()];
            let bodies = transit_bodies(&read_raw(&other.join("transit.zst")), 32 << 20);
            zstd::dict::from_samples(&bodies, 256 << 10).unwrap()
        })
        .collect();
    for (di, dir) in dirs.iter().enumerate() {
        let session = dir.file_name().unwrap().to_string_lossy().to_string();
        let h = hours[di];
        println!("# {session}: {:.3} h", h);
        for &stream in SENSORS.iter().chain(RECORDS) {
            let path = dir.join(format!("{stream}.zst"));
            if !path.exists() {
                continue;
            }
            let now = std::fs::metadata(&path).unwrap().len() as usize;
            let raw = read_raw(&path);
            println!(
                "{session}\t{stream}\tnow\t{now}\traw {}\t{:.3}MB/h",
                raw.len(),
                now as f64 / 1e6 / h
            );
            for &c in COMPS {
                run(
                    &session,
                    stream,
                    &format!("proto+{}", c.name()),
                    &raw,
                    h,
                    now,
                    || c.compress(&raw),
                    |e| c.decompress(e, raw.len()),
                );
            }
            if SENSORS.contains(&stream) {
                // Prost re-encoding must reproduce the stored bytes for the columnar check to be meaningful.
                let chunks: Vec<SensorChunk> = split_delimited(&raw)
                    .into_iter()
                    .map(|m| SensorChunk::decode(m).unwrap())
                    .collect();
                assert!(join_delimited(chunks.iter().map(|c| c.encode_to_vec())) == raw);
                drop(chunks);
                for (mode, mname) in [(Values::SubLe, "sub"), (Values::Rank, "rank")] {
                    for &c in COMPS {
                        run(
                            &session,
                            stream,
                            &format!("col-{mname}+{}", c.name()),
                            &raw,
                            h,
                            now,
                            || {
                                let chunks: Vec<SensorChunk> = split_delimited(&raw)
                                    .into_iter()
                                    .map(|m| SensorChunk::decode(m).unwrap())
                                    .collect();
                                c.compress(&columnar_encode(&chunks, mode))
                            },
                            |e| {
                                let col = c.decompress(e, 512 << 20);
                                join_delimited(
                                    columnar_decode(&col, mode)
                                        .iter()
                                        .map(|c| c.encode_to_vec()),
                                )
                            },
                        );
                    }
                }
            }
            if stream == "transit" {
                for &c in &[Comp::Zstd(19, false), Comp::Zstd(22, true), Comp::Xz] {
                    run(
                        &session,
                        stream,
                        &format!("diff+{}", c.name()),
                        &raw,
                        h,
                        now,
                        || c.compress(&transit_diff_encode(&raw)),
                        |e| transit_diff_decode(&c.decompress(e, 1 << 30)),
                    );
                    let dict = &dicts[di];
                    run(
                        &session,
                        stream,
                        &format!("dict+{} (+{} B dict)", c.name(), dict.len()),
                        &raw,
                        h,
                        now,
                        || c.compress(&transit_dict_encode(&raw, dict)),
                        |e| transit_dict_decode(&c.decompress(e, 1 << 30), dict),
                    );
                }
            }
        }
    }
}

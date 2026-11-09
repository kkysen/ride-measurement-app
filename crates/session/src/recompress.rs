//! Post-ride recompression: rewrites each stream of a finished session as one zstd frame at a high level.
//!
//! The decoded content (the sequence of length-delimited chunks) is unchanged, so readers don't care.
//! A stream is replaced only after its new file is synced and decodes to the same SHA-256 as the original;
//! any failure, including an original that doesn't decode cleanly (e.g. a crash-truncated tail), keeps the original.
//! Audio streams also have each chunk's PCM moved into FLAC (`crate::audio_flac`); their content is compared
//! with every chunk converted back, which gives the original chunks byte for byte.
//! Sensor streams are written in columns (`crate::sensor_columns`) where those give every chunk back byte for byte,
//! else as plain zstd; their content is compared after decoding the columns.

use std::fs::{self, File};
use std::io::{self, BufReader, Read, Write};
use std::path::Path;

use anyhow::Context;

use crate::delimited::{MAX_MESSAGE_BYTES, read_message};
use crate::proto::StreamKind;
use crate::{STREAM_EXTENSION, audio_flac};
use sha2::{Digest, Sha256};

pub const LEVEL: i32 = 19;
/// Written into a session once every stream was handled; uploads wait for it.
pub const MARKER: &str = ".recompressed";
/// The whole compressed original is read into memory to check its frames.
const MAX_STREAM_BYTES: u64 = 512 << 20;
const TMP_SUFFIX: &str = ".recompress-tmp";

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    Replaced { before: u64, after: u64 },
    KeptOriginal { reason: String },
}

impl std::fmt::Display for Outcome {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Outcome::Replaced { before, after } => write!(f, "replaced, {before} -> {after} bytes"),
            Outcome::KeptOriginal { reason } => write!(f, "kept original: {reason}"),
        }
    }
}

fn kept(reason: impl std::fmt::Display) -> Outcome {
    Outcome::KeptOriginal {
        reason: reason.to_string(),
    }
}

/// An audio stream's layout, for its FLAC.
#[derive(Clone, Copy, Debug)]
pub struct Audio {
    pub channels: usize,
    pub sample_rate_hz: usize,
}

/// The session's audio streams by file name, from its `session.pb` (none if it can't be read).
/// The session's audio streams by file name, from its `session.pb` (none if it can't be read).
#[doc(hidden)]
pub fn audio_streams_for_test(dir: &Path) -> std::collections::BTreeMap<String, Audio> {
    audio_streams(dir)
}

/// The session's sensor streams' file names, from its `session.pb` (none if it can't be read).
fn sensor_streams(dir: &Path) -> std::collections::BTreeSet<String> {
    let Ok(session) = crate::read::Session::open(dir) else {
        return Default::default();
    };
    session
        .info
        .streams
        .iter()
        .filter(|s| s.kind == StreamKind::Sensor as i32)
        .map(|s| format!("{}.{STREAM_EXTENSION}", s.name))
        .collect()
}

fn audio_streams(dir: &Path) -> std::collections::BTreeMap<String, Audio> {
    let Ok(session) = crate::read::Session::open(dir) else {
        return Default::default();
    };
    session
        .info
        .streams
        .iter()
        .filter(|s| s.kind == StreamKind::Audio as i32)
        .filter_map(|s| {
            let a = s.audio.as_ref()?;
            Some((
                format!("{}.{STREAM_EXTENSION}", s.name),
                Audio {
                    channels: usize::try_from(a.channels).ok()?,
                    sample_rate_hz: usize::try_from(a.sample_rate_hz).ok()?,
                },
            ))
        })
        .collect()
}

/// Recompresses every stream of the session in `dir`, in name order; writes [`MARKER`] once all were handled.
pub fn recompress_session(dir: &Path, level: i32) -> anyhow::Result<Vec<(String, Outcome)>> {
    let audio = audio_streams(dir);
    let sensors = sensor_streams(dir);
    let mut names: Vec<String> = fs::read_dir(dir)
        .with_context(|| format!("reading {}", dir.display()))?
        .filter_map(|e| e.ok())
        .filter(|e| e.file_type().is_ok_and(|t| t.is_file()))
        .filter_map(|e| e.file_name().into_string().ok())
        .filter(|n| n.ends_with(&format!(".{STREAM_EXTENSION}")))
        .collect();
    names.sort();
    let results: Vec<(String, Outcome)> = names
        .into_iter()
        .map(|name| {
            let path = dir.join(&name);
            let outcome = match audio.get(&name) {
                Some(&a) => recompress_audio_stream(&path, level, a),
                None if sensors.contains(&name) => recompress_sensor_stream(&path, level),
                None => recompress_stream(&path, level),
            };
            (name, outcome)
        })
        .collect();
    let mut marker = File::create(dir.join(MARKER))?;
    for (name, outcome) in &results {
        writeln!(marker, "{name}: {outcome}")?;
    }
    marker.sync_all()?;
    sync_dir(dir)?;
    Ok(results)
}

/// How a stream is rewritten.
#[derive(Clone, Copy, Debug)]
pub enum Encoding {
    /// The same content as one zstd frame.
    Plain,
    /// Each chunk's PCM as FLAC.
    Audio(Audio),
    /// `SensorChunk`s in columns, or plain where columns can't give them back exactly.
    Sensor,
}

pub fn recompress_stream(path: &Path, level: i32) -> Outcome {
    recompress_stream_checked(path, level, Encoding::Plain, |_| {})
}

pub fn recompress_audio_stream(path: &Path, level: i32, audio: Audio) -> Outcome {
    recompress_stream_checked(path, level, Encoding::Audio(audio), |_| {})
}

pub fn recompress_sensor_stream(path: &Path, level: i32) -> Outcome {
    recompress_stream_checked(path, level, Encoding::Sensor, |_| {})
}

/// `corrupt_for_test` can damage the new file before verification, to check the original survives.
#[doc(hidden)]
pub fn recompress_stream_checked(
    path: &Path,
    level: i32,
    encoding: Encoding,
    corrupt_for_test: impl FnOnce(&Path),
) -> Outcome {
    let mut tmp_name = path.as_os_str().to_owned();
    tmp_name.push(TMP_SUFFIX);
    let tmp = Path::new(&tmp_name);
    let _ = fs::remove_file(tmp);
    let outcome = recompress_via(path, tmp, level, encoding, corrupt_for_test);
    let _ = fs::remove_file(tmp);
    outcome
}

fn recompress_via(
    path: &Path,
    tmp: &Path,
    level: i32,
    encoding: Encoding,
    corrupt_for_test: impl FnOnce(&Path),
) -> Outcome {
    let original = match fs::metadata(path) {
        Ok(m) if m.len() > MAX_STREAM_BYTES => return kept("too large to check in memory"),
        Ok(_) => match fs::read(path) {
            Ok(bytes) => bytes,
            Err(e) => return kept(format!("reading the original failed: {e}")),
        },
        Err(e) => return kept(format!("reading the original failed: {e}")),
    };
    // A plain zstd reader would take the columns for the content, so they're never recompressed again.
    if crate::sensor_columns::is_columnar(&original) {
        return kept("already in sensor columns");
    }
    if let Err(e) = check_complete_frames(&original) {
        return kept(format!("original doesn't decode cleanly: {e}"));
    }
    // One decoding pass over the original checks it, recompresses it, and hashes it.
    let written = match encoding {
        Encoding::Audio(a) => write_audio_recompressed(&original, tmp, level, a),
        Encoding::Sensor => write_sensor_recompressed(&original, tmp, level),
        Encoding::Plain => write_recompressed(&original, tmp, level),
    };
    let original_digest = match written {
        Ok(digest) => digest,
        Err(e) if e.kind() == io::ErrorKind::InvalidData => {
            return kept(format!("original doesn't decode cleanly: {e}"));
        }
        Err(e) => return kept(format!("writing the new file failed: {e}")),
    };
    corrupt_for_test(tmp);
    let digest = match encoding {
        Encoding::Audio(_) => restored_digest(tmp),
        Encoding::Sensor => columns_digest(tmp),
        Encoding::Plain => decoded_digest(tmp),
    };
    match digest {
        Ok(digest) if digest == original_digest => {}
        Ok(_) => return kept("decoded content differs"),
        Err(e) => return kept(format!("new file doesn't decode: {e}")),
    }
    let before = original.len() as u64;
    let after = match fs::metadata(tmp) {
        Ok(m) => m.len(),
        Err(e) => return kept(format!("reading the new file failed: {e}")),
    };
    if after >= before {
        return kept(format!("not smaller ({after} >= {before} bytes)"));
    }
    // `rename` replaces the original atomically: either the old or the verified new file exists, never neither.
    if let Err(e) = fs::rename(tmp, path) {
        return kept(format!("rename failed: {e}"));
    }
    if let Some(dir) = path.parent() {
        let _ = sync_dir(dir);
    }
    Outcome::Replaced { before, after }
}

/// Errors unless `bytes` is entirely complete zstd frames; the stream decoder alone accepts a cut-off last frame.
fn check_complete_frames(bytes: &[u8]) -> io::Result<()> {
    let mut offset = 0;
    while offset < bytes.len() {
        match zstd::zstd_safe::find_frame_compressed_size(&bytes[offset..]) {
            Ok(size) if size > 0 && offset + size <= bytes.len() => offset += size,
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("incomplete zstd frame at byte {offset} of {}", bytes.len()),
                ));
            }
        }
    }
    Ok(())
}

/// Decodes the original (checking frame checksums and that it's a whole number of length-delimited messages),
/// writes it recompressed to `tmp` and syncs it, and returns the SHA-256 of the decoded content.
fn write_recompressed(original: &[u8], tmp: &Path, level: i32) -> io::Result<[u8; 32]> {
    let file = File::create_new(tmp)?;
    let mut encoder = zstd::stream::write::Encoder::new(file, level)?;
    encoder.include_checksum(true)?;
    let mut decoder = zstd::stream::read::Decoder::new(original)?;
    let mut sha = Sha256::new();
    let mut messages = MessageBoundaries::default();
    let mut buf = vec![0u8; 1 << 16];
    loop {
        let n = match decoder.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => n,
            // A bad checksum or corrupt frame in the original.
            Err(e) => return Err(io::Error::new(io::ErrorKind::InvalidData, e)),
        };
        sha.update(&buf[..n]);
        messages.feed(&buf[..n])?;
        encoder.write_all(&buf[..n])?;
    }
    messages.finish()?;
    encoder.finish()?.sync_all()?;
    Ok(sha.finalize().into())
}

/// Like [`write_recompressed`] for an audio stream, chunk by chunk, with each chunk's PCM as FLAC where that's
/// lossless; returns the SHA-256 of the content with every chunk as PCM (as [`restored_digest`] computes it).
fn write_audio_recompressed(
    original: &[u8],
    tmp: &Path,
    level: i32,
    audio: Audio,
) -> io::Result<[u8; 32]> {
    let file = File::create_new(tmp)?;
    let mut encoder = zstd::stream::write::Encoder::new(file, level)?;
    encoder.include_checksum(true)?;
    let mut decoder = BufReader::new(zstd::stream::read::Decoder::new(original)?);
    let mut sha = Sha256::new();
    // A bad checksum, corrupt frame, or cut-off message in the original.
    let invalid = |e: io::Error| io::Error::new(io::ErrorKind::InvalidData, e);
    while let Some(message) = read_message(&mut decoder).map_err(invalid)? {
        hash_message(
            &mut sha,
            audio_flac::to_pcm(&message).as_deref().unwrap_or(&message),
        );
        let out = audio_flac::to_flac(&message, audio.channels, audio.sample_rate_hz);
        write_message(&mut encoder, out.as_deref().unwrap_or(&message))?;
    }
    encoder.finish()?.sync_all()?;
    Ok(sha.finalize().into())
}

/// Like [`write_recompressed`] for a sensor stream, in columns where they give every chunk back byte for byte
/// (checked again on the written file), else plain; returns the SHA-256 of the decoded content.
fn write_sensor_recompressed(original: &[u8], tmp: &Path, level: i32) -> io::Result<[u8; 32]> {
    let mut decoder = BufReader::new(zstd::stream::read::Decoder::new(original)?);
    let mut sha = Sha256::new();
    let mut bodies = Vec::new();
    // A bad checksum, corrupt frame, or cut-off message in the original.
    let invalid = |e: io::Error| io::Error::new(io::ErrorKind::InvalidData, e);
    while let Some(message) = read_message(&mut decoder).map_err(invalid)? {
        hash_message(&mut sha, &message);
        bodies.push(message);
    }
    match crate::sensor_columns::encode_file(&bodies, level) {
        Ok(bytes) => {
            let mut file = File::create_new(tmp)?;
            file.write_all(&bytes)?;
            file.sync_all()?;
            Ok(sha.finalize().into())
        }
        Err(_) => write_recompressed(original, tmp, level),
    }
}

/// SHA-256 of what a sensor stream file decodes to, in columns or plain.
fn columns_digest(path: &Path) -> io::Result<[u8; 32]> {
    let bytes = fs::read(path)?;
    if !crate::sensor_columns::is_columnar(&bytes) {
        return decoded_digest(path);
    }
    Ok(Sha256::digest(crate::sensor_columns::decode_file(&bytes)?).into())
}

/// SHA-256 of an audio stream's content with every FLAC chunk converted back to PCM.
fn restored_digest(path: &Path) -> io::Result<[u8; 32]> {
    let mut decoder = BufReader::new(zstd::stream::read::Decoder::new(BufReader::new(
        File::open(path)?,
    ))?);
    let mut sha = Sha256::new();
    while let Some(message) = read_message(&mut decoder)? {
        hash_message(
            &mut sha,
            audio_flac::to_pcm(&message).as_deref().unwrap_or(&message),
        );
    }
    Ok(sha.finalize().into())
}

fn delimiter(len: usize) -> Vec<u8> {
    let mut d = Vec::with_capacity(10);
    prost::encoding::encode_varint(len as u64, &mut d);
    d
}

fn hash_message(sha: &mut Sha256, body: &[u8]) {
    sha.update(delimiter(body.len()));
    sha.update(body);
}

fn write_message(w: &mut impl Write, body: &[u8]) -> io::Result<()> {
    w.write_all(&delimiter(body.len()))?;
    w.write_all(body)
}

/// SHA-256 of what a file decodes to.
fn decoded_digest(path: &Path) -> io::Result<[u8; 32]> {
    let mut decoder = zstd::stream::read::Decoder::new(BufReader::new(File::open(path)?))?;
    let mut sha = Sha256::new();
    let mut buf = vec![0u8; 1 << 16];
    loop {
        match decoder.read(&mut buf)? {
            0 => return Ok(sha.finalize().into()),
            n => sha.update(&buf[..n]),
        }
    }
}

/// Checks a byte stream fed in pieces is a whole number of length-delimited messages,
/// as a crash mid-chunk would leave a cut-off one.
#[derive(Default)]
struct MessageBoundaries {
    /// Bytes of the current message body still to skip.
    remaining: u64,
    /// The length varint being read, and how many of its bytes so far.
    varint: u64,
    varint_bytes: u32,
}

impl MessageBoundaries {
    fn feed(&mut self, mut bytes: &[u8]) -> io::Result<()> {
        while !bytes.is_empty() {
            if self.remaining > 0 {
                let skip = self.remaining.min(bytes.len() as u64);
                self.remaining -= skip;
                bytes = &bytes[skip as usize..];
                continue;
            }
            let byte = bytes[0];
            bytes = &bytes[1..];
            if self.varint_bytes >= 10 {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "varint too long",
                ));
            }
            self.varint |= u64::from(byte & 0x7f) << (7 * self.varint_bytes);
            self.varint_bytes += 1;
            if byte & 0x80 == 0 {
                if self.varint > MAX_MESSAGE_BYTES {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        format!("message length {} exceeds {MAX_MESSAGE_BYTES}", self.varint),
                    ));
                }
                self.remaining = self.varint;
                self.varint = 0;
                self.varint_bytes = 0;
            }
        }
        Ok(())
    }

    fn finish(&self) -> io::Result<()> {
        if self.remaining > 0 || self.varint_bytes > 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "ends inside a message",
            ));
        }
        Ok(())
    }
}

/// Makes a rename or new file in `dir` durable.
fn sync_dir(dir: &Path) -> io::Result<()> {
    File::open(dir)?.sync_all()
}

//! Post-ride recompression: rewrites each stream of a finished session as one zstd frame at a high level.
//!
//! The decoded content (the sequence of length-delimited chunks) is unchanged, so readers don't care.
//! A stream is replaced only after its new file is synced and decodes to the same SHA-256 as the original;
//! any failure, including an original that doesn't decode cleanly (e.g. a crash-truncated tail), keeps the original.
//! Audio streams also have each chunk's PCM moved into FLAC (`crate::audio_flac`); their content is compared
//! with every chunk converted back, which gives the original chunks byte for byte.
//! Sensor streams are written in columns (`crate::sensor_columns`) where those give every chunk back byte for byte,
//! else as plain zstd; their content is compared after decoding the columns.
//!
//! Everything streams, chunk by chunk, so a stream of any size is handled in bounded memory:
//! the original is read twice (its frames checked, then decoded, hashed and rewritten),
//! and the new file once (decoded and hashed).
//! The one exception is sensor columns, which are a whole-stream encoding (one value table per axis)
//! and are decoded whole by readers too: a sensor stream decoding to over [`SENSOR_COLUMNS_MAX_BYTES`]
//! is written as plain zstd instead.
//! Each outcome names the content's SHA-256 (as [`content_sha256`] computes it), written into [`MARKER`],
//! so later copies of a stream can be checked without the original.

use std::fs::{self, File};
use std::io::{self, BufRead, BufReader, Read, Seek, Write};
use std::path::Path;

use anyhow::Context;

use crate::delimited::{MAX_MESSAGE_BYTES, read_message};
use crate::proto::StreamKind;
use crate::{STREAM_EXTENSION, audio_flac};
use sha2::{Digest, Sha256};

pub const LEVEL: i32 = 19;
/// Written into a session once every stream was handled; uploads wait for it.
pub const MARKER: &str = ".recompressed";
/// The most decoded content a sensor stream may have to be written in columns, which are encoded
/// and decoded in memory; past it, the stream is written as plain zstd.
/// About 80 min of `accel_uncal` at 480 Hz (about 50 MB/h).
/// Measured on the PC: about 92 MB for the level-19 encoder plus 1.8 times the content,
/// so 154 MB at 33 MB of content, 204 MB at 65 MB, and about 210 MB at this limit.
pub const SENSOR_COLUMNS_MAX_BYTES: u64 = 64 << 20;
const TMP_SUFFIX: &str = ".recompress-tmp";

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// `sha256` is of the decoded content, the same for the original and the new file.
    Replaced {
        before: u64,
        after: u64,
        sha256: [u8; 32],
    },
    /// `sha256` is of the original's decoded content, when it was read through.
    KeptOriginal {
        reason: String,
        sha256: Option<[u8; 32]>,
    },
}

fn hex(digest: &[u8; 32]) -> String {
    digest.iter().map(|b| format!("{b:02x}")).collect()
}

impl std::fmt::Display for Outcome {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Outcome::Replaced {
                before,
                after,
                sha256,
            } => write!(
                f,
                "replaced, {before} -> {after} bytes, content SHA-256 {}",
                hex(sha256)
            ),
            Outcome::KeptOriginal { reason, sha256 } => {
                write!(f, "kept original: {reason}")?;
                match sha256 {
                    Some(d) => write!(f, ", content SHA-256 {}", hex(d)),
                    None => Ok(()),
                }
            }
        }
    }
}

fn kept(reason: impl std::fmt::Display) -> Outcome {
    Outcome::KeptOriginal {
        reason: reason.to_string(),
        sha256: None,
    }
}

fn kept_hashed(reason: impl std::fmt::Display, sha256: [u8; 32]) -> Outcome {
    Outcome::KeptOriginal {
        reason: reason.to_string(),
        sha256: Some(sha256),
    }
}

/// An audio stream's layout, for its FLAC.
#[derive(Clone, Copy, Debug)]
pub struct Audio {
    pub channels: usize,
    pub sample_rate_hz: usize,
}

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

/// Like [`recompress_sensor_stream`] with another limit than [`SENSOR_COLUMNS_MAX_BYTES`] for columns.
#[doc(hidden)]
pub fn recompress_sensor_stream_with_max(path: &Path, level: i32, columns_max: u64) -> Outcome {
    recompress_in_tmp(path, level, Encoding::Sensor, columns_max, |_| {})
}

/// `corrupt_for_test` can damage the new file before verification, to check the original survives.
#[doc(hidden)]
pub fn recompress_stream_checked(
    path: &Path,
    level: i32,
    encoding: Encoding,
    corrupt_for_test: impl FnOnce(&Path),
) -> Outcome {
    recompress_in_tmp(
        path,
        level,
        encoding,
        SENSOR_COLUMNS_MAX_BYTES,
        corrupt_for_test,
    )
}

fn recompress_in_tmp(
    path: &Path,
    level: i32,
    encoding: Encoding,
    columns_max: u64,
    corrupt_for_test: impl FnOnce(&Path),
) -> Outcome {
    let mut tmp_name = path.as_os_str().to_owned();
    tmp_name.push(TMP_SUFFIX);
    let tmp = Path::new(&tmp_name);
    let _ = fs::remove_file(tmp);
    let outcome = recompress_via(path, tmp, level, encoding, columns_max, corrupt_for_test);
    let _ = fs::remove_file(tmp);
    outcome
}

fn recompress_via(
    path: &Path,
    tmp: &Path,
    level: i32,
    encoding: Encoding,
    columns_max: u64,
    corrupt_for_test: impl FnOnce(&Path),
) -> Outcome {
    let before = match fs::metadata(path) {
        Ok(m) => m.len(),
        Err(e) => return kept(format!("reading the original failed: {e}")),
    };
    // A plain zstd reader would take the columns for the content, so they're never recompressed again.
    match starts_columnar(path) {
        Ok(true) => {
            return match columns_digest(path) {
                Ok(digest) => kept_hashed("already in sensor columns", digest),
                Err(e) => kept(format!(
                    "already in sensor columns, which don't decode: {e}"
                )),
            };
        }
        Ok(false) => {}
        Err(e) => return kept(format!("reading the original failed: {e}")),
    }
    match check_complete_frames(path) {
        Ok(()) => {}
        Err(e) if e.kind() == io::ErrorKind::InvalidData => {
            return kept(format!("original doesn't decode cleanly: {e}"));
        }
        Err(e) => return kept(format!("reading the original failed: {e}")),
    }
    // One decoding pass over the original checks it, recompresses it, and hashes it.
    let written = match encoding {
        Encoding::Audio(a) => write_audio_recompressed(path, tmp, level, a),
        Encoding::Sensor => write_sensor_recompressed(path, tmp, level, columns_max),
        Encoding::Plain => write_recompressed(path, tmp, level),
    };
    let original_digest = match written {
        Ok(digest) => digest,
        Err(e) if e.kind() == io::ErrorKind::InvalidData => {
            return kept(format!("original doesn't decode cleanly: {e}"));
        }
        Err(e) => return kept(format!("writing the new file failed: {e}")),
    };
    corrupt_for_test(tmp);
    match content_sha256(tmp, encoding) {
        Ok(digest) if digest == original_digest => {}
        Ok(_) => return kept_hashed("decoded content differs", original_digest),
        Err(e) => return kept_hashed(format!("new file doesn't decode: {e}"), original_digest),
    }
    let after = match fs::metadata(tmp) {
        Ok(m) => m.len(),
        Err(e) => {
            return kept_hashed(format!("reading the new file failed: {e}"), original_digest);
        }
    };
    if after >= before {
        return kept_hashed(
            format!("not smaller ({after} >= {before} bytes)"),
            original_digest,
        );
    }
    // `rename` replaces the original atomically: either the old or the verified new file exists, never neither.
    if let Err(e) = fs::rename(tmp, path) {
        return kept_hashed(format!("rename failed: {e}"), original_digest);
    }
    if let Some(dir) = path.parent() {
        let _ = sync_dir(dir);
    }
    Outcome::Replaced {
        before,
        after,
        sha256: original_digest,
    }
}

/// SHA-256 of a stream file's decoded content, the sequence of length-delimited chunks as recorded:
/// audio chunks with FLAC converted back to PCM, sensor columns decoded. It's the same for a stream
/// before and after recompression, and is what [`Outcome`] names, so a copy can be checked against it.
pub fn content_sha256(path: &Path, encoding: Encoding) -> io::Result<[u8; 32]> {
    match encoding {
        Encoding::Audio(_) => restored_digest(path),
        Encoding::Sensor => columns_digest(path),
        Encoding::Plain => decoded_digest(path),
    }
}

/// Whether the file starts like a sensor columns file, reading only its header.
fn starts_columnar(path: &Path) -> io::Result<bool> {
    let mut start = Vec::new();
    File::open(path)?
        .take(crate::sensor_columns::header_len() as u64)
        .read_to_end(&mut start)?;
    Ok(crate::sensor_columns::is_columnar(&start))
}

/// Errors (as `InvalidData`) unless the file is entirely complete zstd frames;
/// the stream decoder alone may accept a cut-off last frame.
/// It reads only the frame and block headers, seeking past the blocks, so memory stays bounded.
fn check_complete_frames(path: &Path) -> io::Result<()> {
    let file = File::open(path)?;
    let len = file.metadata()?.len();
    check_complete_frames_in(BufReader::with_capacity(1 << 16, file), len)
}

fn check_complete_frames_in<R: Read + Seek>(reader: BufReader<R>, len: u64) -> io::Result<()> {
    let mut walk = FrameWalk {
        reader,
        pos: 0,
        len,
    };
    while walk.pos < len {
        let start = walk.pos;
        if !walk.frame()? {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("incomplete zstd frame at byte {start} of {len}"),
            ));
        }
    }
    Ok(())
}

/// Steps through zstd frames as `ZSTD_findFrameCompressedSize` does, without holding a frame in memory.
struct FrameWalk<R> {
    reader: BufReader<R>,
    pos: u64,
    len: u64,
}

impl<R: Read + Seek> FrameWalk<R> {
    /// The next `N` bytes, or `None` if the file ends first.
    fn bytes<const N: usize>(&mut self) -> io::Result<Option<[u8; N]>> {
        if self.len - self.pos < N as u64 {
            return Ok(None);
        }
        let mut b = [0u8; N];
        self.reader.read_exact(&mut b)?;
        self.pos += N as u64;
        Ok(Some(b))
    }

    /// Skips `n` bytes; false if the file ends first.
    fn skip(&mut self, n: u64) -> io::Result<bool> {
        if self.len - self.pos < n {
            return Ok(false);
        }
        // Within the buffer this just moves in it; past it, one seek.
        let buffered = self.reader.buffer().len() as u64;
        if n <= buffered {
            self.reader.consume(n as usize);
        } else {
            self.reader.seek_relative(n as i64)?;
        }
        self.pos += n;
        Ok(true)
    }

    /// Steps over one whole frame; false if what's next isn't one (cut off, or not a zstd frame).
    fn frame(&mut self) -> io::Result<bool> {
        let Some(magic) = self.bytes::<4>()? else {
            return Ok(false);
        };
        let magic = u32::from_le_bytes(magic);
        if magic & 0xffff_fff0 == 0x184d_2a50 {
            // Skippable: its size, then that many bytes.
            let Some(size) = self.bytes::<4>()? else {
                return Ok(false);
            };
            return self.skip(u32::from_le_bytes(size).into());
        }
        if magic != 0xfd2f_b528 {
            return Ok(false);
        }
        let Some([descriptor]) = self.bytes::<1>()? else {
            return Ok(false);
        };
        // The reserved bit must be clear.
        if descriptor & 0x08 != 0 {
            return Ok(false);
        }
        let single_segment = descriptor & 0x20 != 0;
        if !single_segment {
            let Some([window]) = self.bytes::<1>()? else {
                return Ok(false);
            };
            // zstd refuses a window over 2^31 bytes (`ZSTD_WINDOWLOG_MAX` on 64-bit).
            if 10 + u32::from(window >> 3) > 31 {
                return Ok(false);
            }
        }
        let dictionary_id = [0, 1, 2, 4][usize::from(descriptor & 0x03)];
        let content_size = match descriptor >> 6 {
            0 => u64::from(single_segment),
            1 => 2,
            2 => 4,
            _ => 8,
        };
        if !self.skip(dictionary_id + content_size)? {
            return Ok(false);
        }
        loop {
            let Some(h) = self.bytes::<3>()? else {
                return Ok(false);
            };
            let header = u32::from(h[0]) | u32::from(h[1]) << 8 | u32::from(h[2]) << 16;
            let size = u64::from(header >> 3);
            let stored = match (header >> 1) & 0x03 {
                // Raw, and compressed: `size` bytes.
                0 | 2 => size,
                // RLE: one byte, repeated `size` times.
                1 => 1,
                _ => return Ok(false),
            };
            if !self.skip(stored)? {
                return Ok(false);
            }
            if header & 1 != 0 {
                break;
            }
        }
        let checksum = if descriptor & 0x04 != 0 { 4 } else { 0 };
        self.skip(checksum)
    }
}

/// The original's decoded content, as a reader; any read error from it is a damaged original.
fn decode_original(path: &Path) -> io::Result<impl Read> {
    zstd::stream::read::Decoder::new(File::open(path)?)
}

/// A bad checksum, corrupt frame, or cut-off message in the original.
fn invalid(e: io::Error) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, e)
}

/// Decodes the original (checking frame checksums and that it's a whole number of length-delimited messages),
/// writes it recompressed to `tmp` and syncs it, and returns the SHA-256 of the decoded content.
fn write_recompressed(original: &Path, tmp: &Path, level: i32) -> io::Result<[u8; 32]> {
    let mut decoder = decode_original(original)?;
    let file = File::create_new(tmp)?;
    let mut encoder = zstd::stream::write::Encoder::new(file, level)?;
    encoder.include_checksum(true)?;
    let mut sha = Sha256::new();
    let mut messages = MessageBoundaries::default();
    let mut buf = vec![0u8; 1 << 16];
    loop {
        let n = match decoder.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => n,
            Err(e) => return Err(invalid(e)),
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
    original: &Path,
    tmp: &Path,
    level: i32,
    audio: Audio,
) -> io::Result<[u8; 32]> {
    let mut decoder = BufReader::new(decode_original(original)?);
    let file = File::create_new(tmp)?;
    let mut encoder = zstd::stream::write::Encoder::new(file, level)?;
    encoder.include_checksum(true)?;
    let mut sha = Sha256::new();
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
/// (checked again on the written file) and its content is at most `columns_max` bytes, else plain;
/// returns the SHA-256 of the decoded content.
fn write_sensor_recompressed(
    original: &Path,
    tmp: &Path,
    level: i32,
    columns_max: u64,
) -> io::Result<[u8; 32]> {
    let mut decoder = BufReader::new(decode_original(original)?);
    let mut sha = Sha256::new();
    // Decoded as they're read, so the bodies aren't held too.
    let mut chunks = Vec::new();
    let mut content = 0u64;
    while let Some(message) = read_message(&mut decoder).map_err(invalid)? {
        content += (delimiter(message.len()).len() + message.len()) as u64;
        let chunk = match crate::sensor_columns::column_chunk(&message) {
            Ok(chunk) if content <= columns_max => chunk,
            // Too long a stream for columns in memory, or a chunk they can't give back:
            // plain, streaming, from the start.
            _ => {
                drop(chunks);
                return write_recompressed(original, tmp, level);
            }
        };
        hash_message(&mut sha, &message);
        chunks.push(chunk);
    }
    let encoded = crate::sensor_columns::encode_chunks_file(&chunks, level);
    drop(chunks);
    match encoded {
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
    if !starts_columnar(path)? {
        return decoded_digest(path);
    }
    let bytes = fs::read(path)?;
    let mut sha = Sha256::new();
    crate::sensor_columns::decode_file_with(&bytes, |body| hash_message(&mut sha, body))?;
    Ok(sha.finalize().into())
}

/// SHA-256 of an audio stream's content with every FLAC chunk converted back to PCM.
fn restored_digest(path: &Path) -> io::Result<[u8; 32]> {
    let mut decoder = BufReader::new(zstd::stream::read::Decoder::new(File::open(path)?)?);
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
    let mut decoder = zstd::stream::read::Decoder::new(File::open(path)?)?;
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

#[cfg(test)]
mod tests {
    use super::*;

    /// The slice-based check this module used before streaming, as the reference.
    fn complete_frames_in_memory(bytes: &[u8]) -> bool {
        let mut offset = 0;
        while offset < bytes.len() {
            match zstd::zstd_safe::find_frame_compressed_size(&bytes[offset..]) {
                Ok(size) if size > 0 && offset + size <= bytes.len() => offset += size,
                _ => return false,
            }
        }
        true
    }

    fn complete_frames_streamed(bytes: &[u8]) -> bool {
        let reader = BufReader::with_capacity(16, io::Cursor::new(bytes));
        match check_complete_frames_in(reader, bytes.len() as u64) {
            Ok(()) => true,
            Err(e) if e.kind() == io::ErrorKind::InvalidData => false,
            Err(e) => panic!("{e}"),
        }
    }

    /// Frames of every kind: one-shot (single segment, content size), streamed with a checksum,
    /// raw blocks (noise), RLE blocks (a long run), several blocks, an empty frame, and a skippable frame.
    fn fixture() -> Vec<u8> {
        let mut state = 0x2545_f491_4f6c_dd1du64;
        let noise: Vec<u8> = (0..300_000)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                state as u8
            })
            .collect();
        let mut out = zstd::bulk::compress(b"a few bytes, a few bytes, a few bytes", 3).unwrap();
        out.extend(zstd::bulk::compress(&noise[..1000], 19).unwrap());
        let mut streamed = zstd::stream::write::Encoder::new(Vec::new(), 3).unwrap();
        streamed.include_checksum(true).unwrap();
        streamed.write_all(&noise).unwrap();
        streamed.write_all(&[7u8; 200_000]).unwrap();
        streamed.write_all(b"the end").unwrap();
        out.extend(streamed.finish().unwrap());
        out.extend(0x184d_2a53u32.to_le_bytes());
        out.extend(5u32.to_le_bytes());
        out.extend(b"skip!");
        out.extend(zstd::bulk::compress(&[], 3).unwrap());
        let mut rle = zstd::stream::write::Encoder::new(Vec::new(), 1).unwrap();
        rle.write_all(&[0u8; 50_000]).unwrap();
        out.extend(rle.finish().unwrap());
        out
    }

    #[test]
    fn streamed_frame_check_agrees_with_zstd_on_every_cut() {
        let bytes = fixture();
        assert!(complete_frames_streamed(&bytes));
        let mut rejected = 0;
        for cut in 0..bytes.len() {
            let expected = complete_frames_in_memory(&bytes[..cut]);
            assert_eq!(
                complete_frames_streamed(&bytes[..cut]),
                expected,
                "cut at {cut}"
            );
            rejected += usize::from(!expected);
        }
        assert!(rejected > bytes.len() / 2);
    }

    #[test]
    fn streamed_frame_check_agrees_with_zstd_on_damage() {
        let bytes = fixture();
        let mut damaged = bytes.clone();
        for i in 0..bytes.len() {
            for flip in [0x01, 0x02, 0x08, 0x20, 0x40, 0x80, 0xff] {
                damaged[i] = bytes[i] ^ flip;
                assert_eq!(
                    complete_frames_streamed(&damaged),
                    complete_frames_in_memory(&damaged),
                    "byte {i} ^ {flip:#x}"
                );
            }
            damaged[i] = bytes[i];
        }
    }
}

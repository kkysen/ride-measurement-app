//! Post-ride recompression: rewrites each stream of a finished session as one zstd frame at a high level.
//!
//! The decoded content (the sequence of length-delimited chunks) is unchanged, so readers don't care.
//! A stream is replaced only after its new file is synced and decodes to the same SHA-256 as the original;
//! any failure, including an original that doesn't decode cleanly (e.g. a crash-truncated tail), keeps the original.

use std::fs::{self, File};
use std::io::{self, BufReader, Read, Write};
use std::path::Path;

use anyhow::Context;

use crate::STREAM_EXTENSION;
use crate::delimited::MAX_MESSAGE_BYTES;
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

/// Recompresses every stream of the session in `dir`, in name order; writes [`MARKER`] once all were handled.
pub fn recompress_session(dir: &Path, level: i32) -> anyhow::Result<Vec<(String, Outcome)>> {
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
            let outcome = recompress_stream(&dir.join(&name), level);
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

pub fn recompress_stream(path: &Path, level: i32) -> Outcome {
    recompress_stream_checked(path, level, |_| {})
}

/// `corrupt_for_test` can damage the new file before verification, to check the original survives.
#[doc(hidden)]
pub fn recompress_stream_checked(
    path: &Path,
    level: i32,
    corrupt_for_test: impl FnOnce(&Path),
) -> Outcome {
    let mut tmp_name = path.as_os_str().to_owned();
    tmp_name.push(TMP_SUFFIX);
    let tmp = Path::new(&tmp_name);
    let _ = fs::remove_file(tmp);
    let outcome = recompress_via(path, tmp, level, corrupt_for_test);
    let _ = fs::remove_file(tmp);
    outcome
}

fn recompress_via(
    path: &Path,
    tmp: &Path,
    level: i32,
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
    if let Err(e) = check_complete_frames(&original) {
        return kept(format!("original doesn't decode cleanly: {e}"));
    }
    // One decoding pass over the original checks it, recompresses it, and hashes it.
    let original_digest = match write_recompressed(&original, tmp, level) {
        Ok(digest) => digest,
        Err(e) if e.kind() == io::ErrorKind::InvalidData => {
            return kept(format!("original doesn't decode cleanly: {e}"));
        }
        Err(e) => return kept(format!("writing the new file failed: {e}")),
    };
    corrupt_for_test(tmp);
    match decoded_digest(tmp) {
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

//! Post-ride recompression: rewrites each stream of a finished session as one zstd frame at a high level.
//!
//! The decoded content (the sequence of length-delimited chunks) is unchanged, so readers don't care.
//! A stream is replaced only after its new file is synced and decodes back to exactly the same bytes;
//! any failure, including an original that doesn't decode cleanly (e.g. a crash-truncated tail), keeps the original.

use std::fs::{self, File};
use std::io::{self, BufReader, Read, Write};
use std::path::Path;

use anyhow::Context;

use crate::STREAM_EXTENSION;
use crate::delimited::read_message;

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
    if let Err(e) = check_clean(&original) {
        return kept(format!("original doesn't decode cleanly: {e}"));
    }
    if let Err(e) = write_recompressed(&original, tmp, level) {
        return kept(format!("writing the new file failed: {e}"));
    }
    corrupt_for_test(tmp);
    match same_decoded(&original, tmp) {
        Ok(true) => {}
        Ok(false) => return kept("decoded content differs"),
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

/// Errors unless `bytes` is entirely complete zstd frames (the stream decoder alone accepts a cut-off last frame)
/// that decode, checksums and all, to a whole number of length-delimited messages.
fn check_clean(bytes: &[u8]) -> io::Result<()> {
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
    let mut decoder = zstd::stream::read::Decoder::new(bytes)?;
    while read_message(&mut decoder)?.is_some() {}
    Ok(())
}

fn write_recompressed(original: &[u8], tmp: &Path, level: i32) -> io::Result<()> {
    let file = File::create_new(tmp)?;
    let mut encoder = zstd::stream::write::Encoder::new(file, level)?;
    encoder.include_checksum(true)?;
    io::copy(
        &mut zstd::stream::read::Decoder::new(original)?,
        &mut encoder,
    )?;
    let file = encoder.finish()?;
    file.sync_all()
}

/// Whether the new file decodes to exactly what the original does, compared as both stream.
fn same_decoded(original: &[u8], tmp: &Path) -> io::Result<bool> {
    let mut a = zstd::stream::read::Decoder::new(original)?;
    let mut b = zstd::stream::read::Decoder::new(BufReader::new(File::open(tmp)?))?;
    let mut buf_a = vec![0u8; 1 << 16];
    let mut buf_b = vec![0u8; 1 << 16];
    loop {
        let n = read_full(&mut a, &mut buf_a)?;
        let m = read_full(&mut b, &mut buf_b)?;
        if n != m || buf_a[..n] != buf_b[..m] {
            return Ok(false);
        }
        if n == 0 {
            return Ok(true);
        }
    }
}

/// Fills `buf` unless the input ends first; returns how much it read.
fn read_full(r: &mut impl Read, buf: &mut [u8]) -> io::Result<usize> {
    let mut n = 0;
    while n < buf.len() {
        match r.read(&mut buf[n..])? {
            0 => break,
            k => n += k,
        }
    }
    Ok(n)
}

/// Makes a rename or new file in `dir` durable.
fn sync_dir(dir: &Path) -> io::Result<()> {
    File::open(dir)?.sync_all()
}

use std::fs::{self, File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};

use anyhow::Context;
use prost::Message;

use crate::{SESSION_INFO_FILE, STREAM_EXTENSION, SensorChunk, SessionInfo};

/// Level used while recording; cheap, since post-ride recompression squeezes further.
pub const LIVE_ZSTD_LEVEL: i32 = 3;

pub fn write_session_info(dir: &Path, info: &SessionInfo) -> anyhow::Result<()> {
    fs::create_dir_all(dir)?;
    let path = dir.join(SESSION_INFO_FILE);
    let mut file = File::create(path)?;
    file.write_all(&info.encode_length_delimited_to_vec())?;
    file.sync_all()?;
    Ok(())
}

/// Frames that couldn't be written are kept and retried, up to this much; past it, new chunks are refused
/// (with an error) rather than letting memory run out and losing every stream.
pub const MAX_PENDING_BYTES: usize = 256 << 20;

/// Appends chunks to `<dir>/<name>.zst`, one independent zstd frame per chunk, synced after each.
///
/// A frame that fails to write (e.g. the disk is momentarily full) is kept and retried, in order,
/// on the next append, so a transient error loses nothing.
/// If a failed write left part of a frame in the file, readers skip it and resync at the retried frame.
pub struct StreamWriter {
    path: PathBuf,
    file: File,
    pending: std::collections::VecDeque<Vec<u8>>,
    pending_bytes: usize,
}

impl StreamWriter {
    pub fn create(dir: &Path, name: &str) -> anyhow::Result<Self> {
        let path = dir.join(format!("{name}.{STREAM_EXTENSION}"));
        let file = OpenOptions::new()
            .create_new(true)
            .append(true)
            .open(&path)?;
        Ok(Self {
            path,
            file,
            pending: Default::default(),
            pending_bytes: 0,
        })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn append(&mut self, chunk: &impl Message) -> anyhow::Result<()> {
        self.append_encoded(&chunk.encode_to_vec())
    }

    /// Appends a chunk already encoded as a protobuf message (not length-delimited),
    /// as when the app builds record chunks from Android's objects.
    pub fn append_encoded(&mut self, chunk: &[u8]) -> anyhow::Result<()> {
        let mut delimited = Vec::with_capacity(chunk.len() + 10);
        prost::encoding::encode_varint(chunk.len() as u64, &mut delimited);
        delimited.extend_from_slice(chunk);
        // Frame checksums let readers tell a corrupted frame from valid data.
        let mut encoder = zstd::stream::Encoder::new(Vec::new(), LIVE_ZSTD_LEVEL)?;
        encoder.include_checksum(true)?;
        encoder.write_all(&delimited)?;
        let frame = encoder.finish()?;
        if self.pending_bytes + frame.len() > MAX_PENDING_BYTES {
            self.write_pending()?;
            anyhow::bail!(
                "{}: {} bytes still unwritten; dropping a {}-byte chunk",
                self.path.display(),
                self.pending_bytes,
                frame.len()
            );
        }
        self.pending_bytes += frame.len();
        self.pending.push_back(frame);
        self.write_pending()
    }

    /// Chunks waiting to be written after a failed write.
    pub fn pending_chunks(&self) -> usize {
        self.pending.len()
    }

    /// Writes and syncs pending frames in order, stopping at the first failure (which keeps it pending).
    pub fn write_pending(&mut self) -> anyhow::Result<()> {
        while let Some(frame) = self.pending.front() {
            self.file
                .write_all(frame)
                .and_then(|()| self.file.sync_data())
                .with_context(|| format!("writing {}", self.path.display()))?;
            self.pending_bytes -= frame.len();
            self.pending.pop_front();
        }
        Ok(())
    }
}

/// Accumulates sensor samples and builds delta-encoded chunks with exact `f32` bits.
/// Every value is kept: a chunk's `values_per_sample` is the first sample's value count,
/// and a sample of a different size needs a new chunk (see [`ChunkBuilder::fits`]).
#[derive(Default)]
pub struct ChunkBuilder {
    timestamps_ns: Vec<i64>,
    value_bits: Vec<u32>,
    accuracy: Vec<i32>,
    values_per_sample: u32,
}

impl ChunkBuilder {
    pub fn len(&self) -> usize {
        self.timestamps_ns.len()
    }

    pub fn is_empty(&self) -> bool {
        self.timestamps_ns.is_empty()
    }

    /// Whether a sample with `value_count` values can go in this chunk.
    pub fn fits(&self, value_count: usize) -> bool {
        self.is_empty() || value_count == self.values_per_sample as usize
    }

    /// `value_bits` are the IEEE 754 bits of each value, so nothing on the way can alter them (e.g. NaN payloads).
    pub fn add(
        &mut self,
        timestamp_ns: i64,
        value_bits: &[u32],
        accuracy: i32,
    ) -> anyhow::Result<()> {
        anyhow::ensure!(
            self.fits(value_bits.len()),
            "sample has {} values, chunk has {}",
            value_bits.len(),
            self.values_per_sample
        );
        if self.is_empty() {
            self.values_per_sample = u32::try_from(value_bits.len())?;
        }
        self.timestamps_ns.push(timestamp_ns);
        self.value_bits.extend_from_slice(value_bits);
        self.accuracy.push(accuracy);
        Ok(())
    }

    /// The chunk so far, emptying the builder.
    pub fn build(&mut self, written_elapsed_realtime_ns: i64) -> SensorChunk {
        let mut prev = 0;
        let timestamp_deltas_ns = self
            .timestamps_ns
            .drain(..)
            .map(|t| {
                let d = t - prev;
                prev = t;
                d
            })
            .collect();
        SensorChunk {
            timestamp_deltas_ns,
            values_per_sample: self.values_per_sample,
            value_bits: std::mem::take(&mut self.value_bits),
            accuracy: std::mem::take(&mut self.accuracy),
            written_elapsed_realtime_ns,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::read::read_stream_file;

    #[test]
    fn failed_writes_are_kept_and_retried_in_order() {
        let dir = tempfile::tempdir().unwrap();
        let mut w = StreamWriter::create(dir.path(), "s").unwrap();
        let chunk = |i: i64| SensorChunk::from_samples(&[i], 1, &[i as f32], &[0], i);
        w.append(&chunk(1)).unwrap();
        // The disk "fills up": writes fail, and the chunks wait.
        let real = std::mem::replace(
            &mut w.file,
            OpenOptions::new().write(true).open("/dev/full").unwrap(),
        );
        assert!(w.append(&chunk(2)).is_err());
        assert!(w.append(&chunk(3)).is_err());
        assert_eq!(w.pending_chunks(), 2);
        // Space again: the next append writes everything, in order.
        w.file = real;
        w.append(&chunk(4)).unwrap();
        assert_eq!(w.pending_chunks(), 0);
        let contents = read_stream_file::<SensorChunk>(w.path()).unwrap();
        assert!(contents.is_clean());
        let written: Vec<i64> = contents
            .chunks
            .iter()
            .map(|c| c.written_elapsed_realtime_ns)
            .collect();
        assert_eq!(written, [1, 2, 3, 4]);
    }
}

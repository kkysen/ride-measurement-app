//! Reading and writing ride sessions, as described in `proto/ride/v1/session.proto`.

pub mod audio_flac;
pub mod clock;
mod delimited;
pub mod marks;
pub mod read;
pub mod recompress;
pub mod sensor_columns;
pub mod track;
pub mod transit;
pub mod write;

/// GTFS-realtime as the MTA's subway feeds use it (`proto/gtfs_realtime.proto`), for the `transit` stream.
pub mod transit_realtime {
    include!(concat!(env!("OUT_DIR"), "/transit_realtime.rs"));
}

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/ride.v1.rs"));
}

pub use proto::{AudioChunk, RecordChunk, SensorChunk, SessionInfo};

pub const FORMAT_VERSION: u32 = 1;
pub const SESSION_INFO_FILE: &str = "session.pb";
pub const STREAM_EXTENSION: &str = "zst";

/// One audio chunk's samples as recorded: which field holds them decides the format.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Pcm<'a> {
    /// 16-bit little-endian integers.
    S16(&'a [u8]),
    /// 32-bit little-endian IEEE floats, nominally in [-1, 1].
    F32(&'a [u8]),
}

impl<'a> Pcm<'a> {
    pub fn bytes(self) -> &'a [u8] {
        match self {
            Self::S16(b) | Self::F32(b) => b,
        }
    }

    pub fn bytes_per_sample(self) -> usize {
        match self {
            Self::S16(_) => 2,
            Self::F32(_) => 4,
        }
    }

    /// Whole frames of `channels` samples each.
    pub fn frames(self, channels: usize) -> usize {
        self.bytes().len() / (self.bytes_per_sample() * channels.max(1))
    }
}

impl AudioChunk {
    /// The chunk's PCM, float if it has any, else 16-bit (empty if moved into `flac`).
    pub fn pcm(&self) -> Pcm<'_> {
        if self.pcm_f32le.is_empty() {
            Pcm::S16(&self.pcm_s16le)
        } else {
            Pcm::F32(&self.pcm_f32le)
        }
    }
}

impl SensorChunk {
    /// Absolute timestamps, undoing the delta encoding.
    pub fn timestamps_ns(&self) -> Vec<i64> {
        self.timestamp_deltas_ns
            .iter()
            .scan(0i64, |t, &d| {
                *t += d;
                Some(*t)
            })
            .collect()
    }

    /// Values as `f32`, bit-exact, `values_per_sample` per sample.
    pub fn values(&self) -> impl Iterator<Item = f32> + '_ {
        self.value_bits.iter().map(|&b| f32::from_bits(b))
    }

    pub fn sample_count(&self) -> usize {
        self.timestamp_deltas_ns.len()
    }

    /// Builds a chunk from absolute timestamps and values, applying the delta encoding.
    pub fn from_samples(
        timestamps_ns: &[i64],
        values_per_sample: u32,
        values: &[f32],
        accuracy: &[i32],
        written_elapsed_realtime_ns: i64,
    ) -> Self {
        let bits = values.iter().map(|v| v.to_bits()).collect();
        Self::from_sample_bits(
            timestamps_ns,
            values_per_sample,
            bits,
            accuracy,
            written_elapsed_realtime_ns,
        )
        .unwrap()
    }

    /// Like [`Self::from_samples`], with each value's IEEE 754 bits, as the app passes them so nothing on the way
    /// can alter one (e.g. a NaN payload). Errors unless there are `values_per_sample` values and one accuracy
    /// per timestamp.
    pub fn from_sample_bits(
        timestamps_ns: &[i64],
        values_per_sample: u32,
        value_bits: Vec<u32>,
        accuracy: &[i32],
        written_elapsed_realtime_ns: i64,
    ) -> anyhow::Result<Self> {
        anyhow::ensure!(
            value_bits.len() == timestamps_ns.len() * values_per_sample as usize
                && accuracy.len() == timestamps_ns.len(),
            "{} timestamps, {} values at {values_per_sample} per sample, {} accuracies",
            timestamps_ns.len(),
            value_bits.len(),
            accuracy.len()
        );
        let mut prev = 0;
        let timestamp_deltas_ns = timestamps_ns
            .iter()
            .map(|&t| {
                let d = t - prev;
                prev = t;
                d
            })
            .collect();
        Ok(Self {
            timestamp_deltas_ns,
            values_per_sample,
            value_bits,
            accuracy: accuracy.to_vec(),
            written_elapsed_realtime_ns,
        })
    }
}

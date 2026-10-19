//! Reading and writing ride sessions, as described in `proto/ride/v1/session.proto`.

mod delimited;
pub mod read;
pub mod write;

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/ride.v1.rs"));
}

pub use proto::{SensorChunk, SessionInfo};

pub const FORMAT_VERSION: u32 = 1;
pub const SESSION_INFO_FILE: &str = "session.pb";
pub const STREAM_EXTENSION: &str = "zst";

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
        assert_eq!(
            values.len(),
            timestamps_ns.len() * values_per_sample as usize
        );
        assert_eq!(accuracy.len(), timestamps_ns.len());
        let mut prev = 0;
        let timestamp_deltas_ns = timestamps_ns
            .iter()
            .map(|&t| {
                let d = t - prev;
                prev = t;
                d
            })
            .collect();
        Self {
            timestamp_deltas_ns,
            values_per_sample,
            value_bits: values.iter().map(|v| v.to_bits()).collect(),
            accuracy: accuracy.to_vec(),
            written_elapsed_realtime_ns,
        }
    }
}

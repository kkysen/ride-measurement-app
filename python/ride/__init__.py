"""Read ride sessions into numpy arrays, bit-exactly, via the Rust reader."""

from dataclasses import dataclass

import numpy as np

from ride import _native


@dataclass
class Stream:
    timestamps_ns: np.ndarray  # int64, `SensorEvent.timestamp`
    values: np.ndarray  # float32, shape (samples, values_per_sample)
    accuracy: np.ndarray  # int32
    truncated: bool
    damaged_ranges: int  # byte ranges skipped as corrupt; their samples are missing


def stream_names(session_dir: str) -> list[str]:
    return _native.stream_names(str(session_dir))


def read_stream(session_dir: str, stream: str) -> Stream:
    raw = _native.read_stream(str(session_dir), stream)
    values = np.frombuffer(raw["values"], dtype="<f4")
    return Stream(
        timestamps_ns=np.frombuffer(raw["timestamps_ns"], dtype="<i8"),
        values=values.reshape(-1, max(raw["values_per_sample"], 1)),
        accuracy=np.frombuffer(raw["accuracy"], dtype="<i4"),
        truncated=raw["truncated"],
        damaged_ranges=raw["damaged_ranges"],
    )

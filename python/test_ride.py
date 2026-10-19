import os

import numpy as np
import pytest

import ride

FIXTURE = os.environ.get("RIDE_KOTLIN_FIXTURE")


@pytest.mark.skipif(not FIXTURE, reason="RIDE_KOTLIN_FIXTURE not set")
@pytest.mark.parametrize("suffix", ["", "-recompressed"])
def test_kotlin_fixture_bits(suffix):
    fixture = FIXTURE + suffix
    s = ride.read_stream(fixture, "accel")
    assert ride.stream_names(fixture) == ["accel"]
    assert not s.truncated
    assert s.values.shape == (300, 3)
    tricky = [0x00000000, 0x80000000, 0x00800000, 0x00000001, 0x7F7FFFFF,
              0x7F800000, 0xFF800000, 0x7FC01234, 0x411CE80A, 0xB4A5B4B2]
    expected = np.array([tricky[i % 10] ^ i for i in range(900)], dtype=np.uint32)
    assert np.array_equal(s.values.reshape(-1).view(np.uint32), expected)
    samples = np.arange(300)
    assert np.array_equal(s.timestamps_ns, 1_000_000_000 + samples * 2_500_000 + samples % 3)

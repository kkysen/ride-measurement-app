"""Barometric height (relative, m) every `step` s with the manual events, and how noisy it is at rest."""

import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import events, imu, ride, session_dir  # noqa: E402

name = sys.argv[1]
step = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
_, _, _, t0 = imu(name, 1.0)
p = ride.read_stream(session_dir(name), "pressure")
t = (p.timestamps_ns - t0) * 1e-9
hpa = p.values[:, 0].astype(float)
# Hypsometric: about 8.3 m per hPa near sea level.
h = 44330.0 * (1 - (hpa / hpa[0]) ** (1 / 5.255))
ev = events(name, t0)
# Noise: rms of 1 s means about their 10 s trend.
k = int(round(1 / np.median(np.diff(t))))
h1 = np.convolve(h, np.ones(k) / k, mode="same")
h10 = np.convolve(h, np.ones(10 * k) / (10 * k), mode="same")
print(f"{len(t)} samples at {k} Hz; 1 s vs 10 s residual rms {np.std((h1 - h10)[10 * k : -10 * k]):.2f} m")
e = 0
for t_s in np.arange(0, t[-1], step):
    i = np.searchsorted(t, t_s)
    marks = []
    while e < len(ev) and ev[e][0] < t_s + step:
        marks.append(f"{ev[e][1].lower()} {ev[e][2]}".strip())
        e += 1
    print(f"{t_s:6.0f} s  {h1[i]:7.1f} m  {'; '.join(marks)}")

"""Per simulated outage (the last `outage` s of each `period`): live speed error at +10, +20, +29 s, vs our own Doppler."""

import csv
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS, fused  # noqa: E402

t0, run = int(sys.argv[1]), sys.argv[2]
period, outage, t_end = 90.0, 30.0, 1360.0
with open(ANALYSIS / run / "own.csv") as f:
    own = np.array([[(int(r["t_ns"]) - t0) * 1e-9, float(r["speed"]), float(r["sigma"])] for r in csv.DictReader(f)])
x = fused(run, t0)
ok = (own[:, 2] < 0.5) & (own[:, 0] < t_end)
ot, ov = own[ok, 0], own[ok, 1]
e = (np.interp(ot, x["t"], x["forward"]) - ov) * 3.6
for k in range(int(t_end // period)):
    s = k * period + period - outage
    vals = []
    for d in [10, 20, outage - 1]:
        i = np.argmin(abs(ot - (s + d)))
        vals.append(e[i] if abs(ot[i] - (s + d)) < 1.5 else np.nan)
    i0, i1 = np.argmin(abs(ot - s)), np.argmin(abs(ot - (s + outage - 1)))
    print(f"{s:5.0f} s: {vals[0]:+6.1f} {vals[1]:+6.1f} {vals[2]:+6.1f}   true {ov[i0] * 3.6:5.1f} -> {ov[i1] * 3.6:5.1f} km/h")

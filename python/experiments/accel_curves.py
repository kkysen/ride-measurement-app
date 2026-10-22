"""Acceleration envelopes per car model above ground, for a limit underground: on rides with GNSS on a moving train,
our own Doppler speed (sigma under 0.5 m/s, fixes 1 s apart), its acceleration over 2 s, binned by speed;
per bin the 99th and 1st percentiles, i.e. the most the train motored and braked at that speed.

Usage: `accel_curves.py model:ride[,ride...] ...`, e.g. `accel_curves.py R188:154951,184134`;
needs `own.csv` in `target/analysis/<ride>`."""

import csv
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS  # noqa: E402

BIN_MPS = 2.0


def accels(ride):
    with open(ANALYSIS / ride / "own.csv") as f:
        own = np.array([[int(r["t_ns"]) * 1e-9, float(r["speed"]), float(r["sigma"])] for r in csv.DictReader(f)])
    t, v, s = own.T
    out = []
    for i in range(1, len(t) - 1):
        if max(s[i - 1 : i + 2]) < 0.5 and abs(t[i + 1] - t[i - 1] - 2) < 0.2:
            out.append((v[i], (v[i + 1] - v[i - 1]) / (t[i + 1] - t[i - 1])))
    return np.array(out)


for arg in sys.argv[1:]:
    model, rides = arg.split(":")
    va = np.concatenate([accels(r) for r in rides.split(",")])
    print(f"{model}: {len(va)} samples; speed bin (mph), n, motoring p99 / max, braking p1 / min (m/s^2)")
    for lo in np.arange(0, 22, BIN_MPS):
        a = va[(va[:, 0] >= lo) & (va[:, 0] < lo + BIN_MPS), 1]
        if len(a) < 10:
            continue
        print(f"  {lo * 2.237:4.1f}-{(lo + BIN_MPS) * 2.237:4.1f}  {len(a):4d}  "
              f"{np.percentile(a, 99):+.2f} / {a.max():+.2f}   {np.percentile(a, 1):+.2f} / {a.min():+.2f}")

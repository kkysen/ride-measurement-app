"""Live (forward) speed against our own raw-Doppler GNSS speed where it's confident (the phone's own speed
holds at 0 for seconds as a train departs), on the elevated 7 part of a run exported by
`crates/cli/examples/export.rs` into target/analysis/<run>: everywhere, and inside simulated outages."""

import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import csv  # noqa: E402

from common import ANALYSIS, fused  # noqa: E402


def score(run, t0, t_end=1360.0, outage=30.0, period=90.0, t_first=0.0):
    import os
    outage = float(os.environ.get("SCORE_OUTAGE_S", outage))
    period = float(os.environ.get("SCORE_PERIOD_S", period))
    with open(ANALYSIS / run / "own.csv") as file:
        own = np.array([[(int(r["t_ns"]) - t0) * 1e-9, float(r["speed"]), float(r["sigma"])] for r in csv.DictReader(file)])
    x = fused(run, t0)
    ok = (own[:, 2] < 0.5) & (own[:, 0] < t_end)
    ft, fv = own[ok, 0], own[ok, 1]
    fwd = np.interp(ft, x["t"], x["forward"])
    err = (fwd - fv) * 3.6
    phase = (ft - t_first) % period
    inside = phase >= period - outage
    ends = inside & (phase >= period - 2)
    rms = lambda e: np.sqrt(np.mean(e**2)) if len(e) else float("nan")
    return rms(err[~inside]), rms(err[inside]), rms(err[ends]), np.percentile(np.abs(err[inside]), 95) if inside.any() else np.nan


if __name__ == "__main__":
    t0 = int(sys.argv[1])
    for run in sys.argv[2:]:
        a, b, c, d = score(run, t0)
        print(f"{run:45s} with GNSS {a:5.2f}  outage rms {b:5.2f}, at outage end {c:5.2f}, outage 95% {d:5.2f} km/h")

"""The rider's elevator at home (5 flights, about 13 m) against the stairs, at the ends of rides: the steepest
25 s height change in the first or last 6 minutes, from the barometer (about 8.3 m per hPa), and how much the
accelerometer shakes meanwhile. Both are about 12 m in 25 s; the stairs shake 4.9 to 6.4 m/s^2 (std, from steps),
the elevator 0.4 to 0.7, so a 10 m change under 1.5 m/s^2 is the elevator, and 7 m (stairs are slower) over 3 the stairs."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ride  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
CASES = [("20261021-182135", "start"), ("20261021-135509", "start"), ("20261021-215922", "end"),
         ("20261020-214427", "end"), ("20261021-155919", "end")]


def steepest(session, where, span_s=360.0, window_s=25):
    p = ride.read_stream(session, "pressure")
    t, v = p.timestamps_ns, p.values[:, 0].astype(float)
    t0 = t[0] if where == "start" else t[-1] - span_s * 1e9
    m = (t >= t0) & (t <= t0 + span_s * 1e9)
    tt, vv = t[m], v[m]
    sec = np.arange(tt[0], tt[-1], 1e9)
    med = np.array([np.median(vv[(tt >= a) & (tt < a + 2e9)]) for a in sec])
    alt = -8.3 * (med - med[0])
    d = alt[window_s:] - alt[:-window_s]
    i = int(np.argmax(np.abs(d)))
    return sec[i], sec[i] + window_s * 1e9, d[i]


def shake(session, a, b):
    acc = ride.read_stream(session, "accel_uncal")
    x = acc.values[:, :3].astype(float) - acc.values[:, 3:6]
    m = (acc.timestamps_ns >= a) & (acc.timestamps_ns < b)
    return np.linalg.norm(x[m], axis=1).std()


for name, where in CASES:
    s = ROOT / "data/sessions" / name
    a, b, dz = steepest(s, where)
    sd = shake(s, a, b)
    kind = "elevator" if abs(dz) >= 10 and sd < 1.5 else "stairs" if abs(dz) >= 7 and sd > 3 else "?"
    print(f"{name} {where}: {dz:+.0f} m in 25 s, accel std {sd:.2f} m/s^2: {kind}")

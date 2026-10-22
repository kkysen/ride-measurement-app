"""The rider's elevator at home (5 flights, about 13 m) against the stairs, at the ends of rides: the steepest
25 s height change in the first or last 6 minutes, from the barometer (about 8.3 m per hPa), and how much the
accelerometer shakes meanwhile. Both are about 12 m in 25 s; the stairs shake 4.9 to 6.4 m/s^2 (std, from steps),
the elevator 0.4 to 0.7, so a 10 m change under 1.5 m/s^2 is the elevator, and 7 m (stairs are slower) over 3 the stairs.

`moves`: anywhere in a ride, every 10 m change in 25 s, with the step rhythm (`sensor_trips.walking`) during it,
in the minute before, and in the minute after. Stairs have steps during (and shake 2 to 5 m/s^2).
Riding up or down has none during but walking just before: the home elevator at four rides' ends went 10 to 11 m
at 0.5 m/s, with 5 to 14 walking windows before and 1 to 3 after. Without walking before, the change was on a
train or standing (grades, tunnel pressure). An escalator is also about 0.5 m/s and walked onto, so this can't
tell it from an elevator; the elevator's start and stop jolts might. At home, the fix before it says which."""

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


def moves(session, walks):
    """[(start ns, height change m, peak m/s over 10 s, kind)] for every 10 m change in 25 s;
    kind "stairs", "riding" (elevator or escalator, walked onto), or "vehicle" (no walking around it)."""
    p = ride.read_stream(session, "pressure")
    t, v = p.timestamps_ns, p.values[:, 0].astype(float)
    sec = np.arange(t[0], t[-1], 1e9)
    idx = np.searchsorted(t, sec)
    med = np.array([np.median(v[a:b]) if b > a else np.nan for a, b in zip(idx, np.append(idx[1:], len(t)))])
    alt = -8.3 * (med - med[0])
    out, last = [], -np.inf
    for i in range(len(sec) - 25):
        dz = alt[i + 25] - alt[i]
        if abs(dz) < 10 or sec[i] - last < 60e9:
            continue
        seg = alt[i : i + 26]
        peak = max(abs(seg[k + 10] - seg[k]) for k in range(16)) / 10
        during = sum(1 for w in walks if sec[i] <= w < sec[i] + 21e9)
        before = sum(1 for w in walks if sec[i] - 60e9 <= w < sec[i])
        kind = "stairs" if during >= 2 else "riding" if before >= 3 else "vehicle"
        out.append((int(sec[i]), dz, peak, kind))
        last = sec[i]
    return out


if __name__ == "__main__":
    for name, where in CASES:
        s = ROOT / "data/sessions" / name
        a, b, dz = steepest(s, where)
        sd = shake(s, a, b)
        kind = "elevator" if abs(dz) >= 10 and sd < 1.5 else "stairs" if abs(dz) >= 7 and sd > 3 else "?"
        print(f"{name} {where}: {dz:+.0f} m in 25 s, accel std {sd:.2f} m/s^2: {kind}")

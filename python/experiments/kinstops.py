"""Stops from motion alone: a sustained braking pulse, then a jerk (the lurch) within a few seconds of its end,
then quiet. Sign-free, as underground nothing gives the axis's sign: along the dominant horizontal direction
of the last 2 minutes, with gravity following the accelerometer over 20 s.

Truth: on `154951`, GNSS stops (our own Doppler under 1 km/h after over 10 km/h);
on `151449`, a stop 3 s before each doors-open press while riding (doors open after boarding a waiting train
don't count). Prints each rule's hits, misses, and false stops per hour of riding.
"""

import csv
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS, events, imu  # noqa: E402

RATE = 50.0


def along(name):
    t, acc, gyr, t0 = imu(name, RATE)
    dt = 1 / RATE
    k = 1 - np.exp(-dt / 20)
    g = acc[0].copy()
    grav = np.empty_like(acc)
    for i in range(len(t)):
        g -= np.cross(gyr[i], g) * dt
        g += k * (acc[i] - g)
        grav[i] = g
    u = grav / np.linalg.norm(grav, axis=1, keepdims=True)
    lin = acc - grav
    h = lin - np.sum(lin * u, axis=1, keepdims=True) * u
    # 1 s means, then the dominant direction over a trailing 120 s, its sign kept continuous.
    n = int(RATE)
    m = len(h) // n
    h1 = h[: m * n].reshape(m, n, 3).mean(1)
    a = np.zeros(m)
    axis = np.array([1.0, 0, 0])
    for i in range(m):
        w = h1[max(0, i - 120) : i + 1]
        w = w - w.mean(0)
        c = w.T @ w
        x = axis
        for _ in range(5):
            y = c @ x
            if np.linalg.norm(y) < 1e-9:
                break
            x = y / np.linalg.norm(y)
        if x @ axis < 0:
            x = -x
        axis = x
        a[i] = h1[i] @ axis
    # Jerk at 0.2 s along the same axis.
    n5 = int(RATE / 5)
    m5 = len(h) // n5
    h5 = h[: m5 * n5].reshape(m5, n5, 3).mean(1)
    mag5 = np.linalg.norm(h5, axis=1)
    jerk = np.r_[0, np.abs(np.diff(mag5)) * 5]
    jerk1 = jerk[: m * 5].reshape(m, 5).max(1)
    return np.arange(m) + 0.5, a, jerk1, t0


def truth(name, t0):
    """Stops 3 s before each doors-open press while riding (after boarding, or from the start if the session began
    on the train; before alighting), skipping a waiting train boarded just after; and the riding spans."""
    ev = events(name, t0)
    stops = []
    riding = not any(k == "BOARDED" for _, k, _ in ev[:3])
    spans = [[0.0, None]] if riding else []
    for i, (t, k, _) in enumerate(ev):
        if k == "BOARDED" and not riding:
            riding = True
            spans.append([t, None])
        if k == "ALIGHTED" and riding:
            riding = False
            spans[-1][1] = t
        boards_next = any(k2 == "BOARDED" and 0 <= t2 - t < 20 for t2, k2, _ in ev[i + 1 :])
        if k == "DOORS_OPEN" and riding and not boards_next:
            stops.append(t - 3)
    if spans and spans[-1][1] is None:
        spans[-1][1] = ev[-1][0] + 15
    return stops, spans


def riding(x, spans):
    return any(a <= x <= b for a, b in spans)


def vibration(name, t):
    """Accelerometer 4-10 Hz plus audio 20-100 Hz log power, per second at `t` (from `vibration.py`)."""
    rows = np.load(ANALYSIS / name / "vibration.npy")
    return np.interp(t, rows[:, 0] + 0.5, rows[:, 4] + rows[:, 9])


def detect(t, a, jerk, pre_min, step_min, flat, jerk_min):
    """A stop at second i: the 6 s before brake steadily (one sign, mean magnitude over `pre_min`), then the
    acceleration steps the other way by over `step_min` (the lurch: braking ends at once) and stays flat (standard
    deviation under `flat`) for 5 s, with a 0.2 s jerk over `jerk_min` around the step. Sign-free, as the
    leftover offset after the step is braking that leaked into gravity. One per 20 s."""
    out = []
    for i in range(8, len(a) - 6):
        pre = a[i - 6 : i]
        post = a[i + 1 : i + 6]
        m = pre.mean()
        if abs(m) < pre_min or np.any(np.sign(pre) != np.sign(m)):
            continue
        step = post.mean() - m
        if np.sign(step) == np.sign(m) or abs(step) < step_min or np.abs(np.diff(post)).max() > flat:
            continue
        if jerk[i - 1 : i + 2].max() < jerk_min:
            continue
        if out and t[i] - out[-1] < 20:
            continue
        out.append(t[i])
    return out


def score(found, stops, riding, tol=8):
    hits = sum(any(abs(f - s) <= tol for f in found) for s in stops)
    false = sum(not any(abs(f - s) <= tol for s in stops) for f in found if riding[0] <= f <= riding[1])
    hours = (riding[1] - riding[0]) / 3600
    return hits, len(stops), false / hours, false


if __name__ == "__main__":
    data = {}
    for name in ["154951", "151449"]:
        t, a, jerk, t0 = along(name)
        data[name] = (t, a, jerk, truth(name, t0))
        print(name, "stops at", [round(s) for s in data[name][3][0]])
    best = []
    for pre_min in [0.2, 0.3, 0.4]:
        for step_min in [0.3, 0.4, 0.5, 0.7]:
            for flat in [0.1, 0.15, 0.25, 0.4]:
                for jerk_min in [0, 2, 4]:
                    p = (pre_min, step_min, flat, jerk_min)
                    res = []
                    for name, (t, a, jerk, (stops, riding)) in data.items():
                        res.append(score(detect(t, a, jerk, *p), stops, riding))
                    best.append((res, p))
    best.sort(key=lambda r: -sum(h - 0.3 * f for (h, n, fph, f) in r[0]))
    for res, p in best[:12]:
        print(f"pre {p[0]} step {p[1]} flat {p[2]} jerk {p[3]}:  " + "   ".join(f"{h}/{n} found, {f} false ({fph:.0f}/h)" for h, n, fph, f in res))

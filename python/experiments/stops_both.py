"""Stopped/moving across both rides: the elevated 7 (`154951`, labeled by GNSS) and the subway (`151449`,
labeled by door presses: doors open to closed is stopped; from 15 s after doors closed to 5 s before the next
"at station" is moving). Trains on one ride and tests on the other, and on both with leave-one-ride-out."""

import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import stopclass as sc  # noqa: E402
from common import ANALYSIS, events, imu  # noqa: E402

COLS = [3, 4, 5, 9, 10]


def load(name):
    rows = np.load(ANALYSIS / name / "vibration.npy")
    t = rows[:, 0]
    if name == "154951":
        sp, ph = rows[:, 1], rows[:, 2]
        stopped = (ph == 0) & (sp < 5)
        moving = sp > 15
    else:
        _, _, _, t0 = imu(name, 1.0)
        ev = events(name, t0)
        stopped = np.zeros(len(t), bool)
        moving = np.zeros(len(t), bool)
        opens = [te for te, k, _ in ev if k == "DOORS_OPEN"]
        for te, k, _ in ev:
            if k == "DOORS_CLOSED":
                prior = [o for o in opens if o < te]
                if prior and te - prior[-1] < 120:
                    stopped |= (t >= prior[-1]) & (t < te)
                nxt = [a for a, kk, _ in ev if a > te + 20 and kk in ("AT_STATION", "DOORS_OPEN")]
                if nxt:
                    moving |= (t >= te + 15) & (t < nxt[0] - 5)
    return t, sc.smoothed(rows[:, COLS], 5, True), stopped, moving


def report(label, p, stopped, moving):
    m = stopped | moving
    y = moving[m].astype(float)
    print(f"{label:34s} AUC {sc.auc(p[m], y):.3f}  stopped seconds called moving {np.mean(p[stopped] > 0.5):.2f}"
          f"  moving seconds called stopped {np.mean(p[moving] <= 0.5):.2f}  ({stopped.sum()} s / {moving.sum()} s)")


data = {n: load(n) for n in ["154951", "151449"]}
for n, (t, X, s, m) in data.items():
    print(f"{n}: {s.sum()} s stopped, {m.sum()} s moving labeled")
for tr, te in [("154951", "151449"), ("151449", "154951")]:
    _, X, s, m = data[tr]
    lab = s | m
    model = sc.fit(X[lab], m[lab].astype(float))
    _, Xt, st, mt = data[te]
    report(f"train {tr}, test {te}", model(Xt), st, mt)
# Per-ride normalization: subtract each ride's median of each band, so a loud tunnel and a quiet viaduct line up.
print("with each band's per-ride median removed:")
norm = {n: (t, X - np.median(X, 0), s, m) for n, (t, X, s, m) in data.items()}
for tr, te in [("154951", "151449"), ("151449", "154951")]:
    _, X, s, m = norm[tr]
    lab = s | m
    model = sc.fit(X[lab], m[lab].astype(float))
    _, Xt, st, mt = norm[te]
    report(f"train {tr}, test {te}", model(Xt), st, mt)

print("within 151449, 4 time blocks, per-ride median removed; and pooled (both rides) blocks:")
for names in [["151449"], ["154951", "151449"]]:
    for cols, label in [(slice(None), "accel 1-25 + audio 20-300"), ([0, 1, 2], "accel only"), ([3, 4], "audio only")]:
        Xs = np.concatenate([norm[n][1][:, cols] for n in names])
        S = np.concatenate([norm[n][2] for n in names])
        M = np.concatenate([norm[n][3] for n in names])
        p = np.full(len(S), np.nan)
        for f in np.array_split(np.arange(len(S)), 4 * len(names)):
            tr = np.setdiff1d(np.arange(len(S)), f)
            tr = tr[(S | M)[tr]]
            p[f] = sc.fit(Xs[tr], M[tr].astype(float))(Xs[f])
        report(f"{'+'.join(names)} {label}", p, S, M)

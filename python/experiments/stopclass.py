"""Stopped vs moving from band powers (from vibration.py), by logistic regression, cross-validated in 5 time blocks.

Features are smoothed over `smooth` s (a centered median, or trailing for live); prints AUC and the accuracy
at the threshold that's best on the training folds.
"""

import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS  # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "154951"
rows = np.load(ANALYSIS / name / "vibration.npy")
speed, phone = rows[:, 1], rows[:, 2]
stopped = (phone == 0) & (speed < 5)
moving = speed > 15
labeled = stopped | moving
y = moving.astype(float)
NAMES = ["a1-4", "a4-10", "a10-25", "a25-60", "a60-120", "a120-240", "s20-100", "s100-300", "s300-1k", "s1k-3k", "s3k-8k"]


def smoothed(x, n, trailing):
    out = np.empty_like(x)
    for i in range(len(x)):
        lo, hi = (i - n + 1, i + 1) if trailing else (i - n // 2, i + n // 2 + 1)
        out[i] = np.median(x[max(lo, 0) : hi], axis=0)
    return out


def fit(X, y, iters=3000, lr=0.1, l2=1e-3):
    mu, sd = X.mean(0), X.std(0) + 1e-9
    Z = np.c_[(X - mu) / sd, np.ones(len(X))]
    w = np.zeros(Z.shape[1])
    for _ in range(iters):
        p = 1 / (1 + np.exp(-Z @ w))
        w -= lr * (Z.T @ (p - y) / len(y) + l2 * np.r_[w[:-1], 0])
    return lambda X: 1 / (1 + np.exp(-np.c_[(X - mu) / sd, np.ones(len(X))] @ w))


def auc(s, y):
    pos, neg = s[y == 1], s[y == 0]
    return (np.sum(pos[:, None] > neg[None, :]) + 0.5 * np.sum(pos[:, None] == neg[None, :])) / (len(pos) * len(neg))


def evaluate(cols, n, trailing):
    X = smoothed(rows[:, [3 + c for c in cols]], n, trailing)
    folds = np.array_split(np.arange(len(rows)), 5)
    scores = np.full(len(rows), np.nan)
    for f in folds:
        train = np.setdiff1d(np.arange(len(rows)), f)
        train = train[labeled[train]]
        model = fit(X[train], y[train])
        scores[f] = model(X[f])
    m = labeled
    acc = np.mean((scores[m] > 0.5) == (y[m] == 1))
    # Missed stops (stopped windows called moving) and false stops (moving called stopped).
    missed = np.mean(scores[stopped] > 0.5)
    false = np.mean(scores[moving] <= 0.5)
    return auc(scores[m], y[m]), acc, missed, false


SETS = {
    "audio 20-100": [6],
    "audio 20-300": [6, 7],
    "accel 1-25": [0, 1, 2],
    "accel 1-25 + audio 20-300": [0, 1, 2, 6, 7],
    "all": list(range(11)),
}
for trailing in [False, True]:
    for n in [1, 3, 5, 9]:
        for label, cols in SETS.items():
            a, acc, missed, false = evaluate(cols, n, trailing)
            print(f"{'trailing' if trailing else 'centered'} {n} s  {label:28s} AUC {a:.3f}  accuracy {acc:.3f}  "
                  f"stops missed {missed:.3f}  false stops {false:.3f}")
        print()


def events(mask):
    """(start, end) indices of runs of True."""
    d = np.diff(np.r_[0, mask.astype(int), 0])
    return list(zip(np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]))


print("Per stop, trailing 5 s, accelerometer 1-25 Hz + audio 20-300 Hz, cross-validated:")
X = smoothed(rows[:, [3, 4, 5, 9, 10]], 5, True)
folds = np.array_split(np.arange(len(rows)), 5)
scores = np.full(len(rows), np.nan)
for f in folds:
    train = np.setdiff1d(np.arange(len(rows)), f)
    train = train[labeled[train]]
    scores[f] = fit(X[train], y[train])(X[f])
t = rows[:, 0]
fast = speed > 20
for thr in [0.5, 0.7, 0.8, 0.9]:
    called = scores < 1 - thr  # stopped when P(moving) < 1 - thr... i.e. confident of stopping
    called = scores < thr
    stops = events(stopped)
    found, delays = 0, []
    for a, b in stops:
        if b - a < 5:
            continue
        hit = np.nonzero(called[a:b])[0]
        if len(hit):
            found += 1
            delays.append(hit[0])
    false_runs = [r for r in events(called & fast)]
    long_false = [r for r in false_runs if r[1] - r[0] >= 3]
    n = sum(1 for a, b in stops if b - a >= 5)
    print(f"  stopped when P(moving) < {thr}: {found}/{n} stops found, median {np.median(delays) if delays else float('nan'):.0f} s in;"
          f" false stops above 20 km/h: {len(false_runs)} runs ({len(long_false)} of 3 s or more) in {fast.sum()} s")

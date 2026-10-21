"""Traction current in the magnetometer as a measure of acceleration: on a ride with GNSS, how well the magnetic
field (phone frame, 1 s means, its 60 s mean removed, so the Earth's field and the car's steel drop out) predicts
the along-track acceleration from our own Doppler, by linear regression, cross-validated in 5 time blocks.
Regenerative braking reverses the current, so the field's signed components are used, not its magnitude."""

import csv
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS, imu, ride, session_dir  # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "154951"
t_from, t_to = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (40.0, 1330.0)
_, _, _, t0 = imu(name, 1.0)
m = ride.read_stream(session_dir(name), "mag_uncal")
mt = (m.timestamps_ns - t0) * 1e-9
mv = m.values[:, :3].astype(float)
sec = np.arange(int(t_from), int(t_to))
B = np.array([mv[(mt >= s) & (mt < s + 1)].mean(0) if np.any((mt >= s) & (mt < s + 1)) else [np.nan] * 3 for s in sec])
slow = np.array([np.nanmean(B[max(0, i - 30) : i + 30], 0) for i in range(len(B))])
X = B - slow
with open(ANALYSIS / name / "own.csv") as f:
    own = np.array([[(int(r["t_ns"]) - t0) * 1e-9, float(r["speed"]), float(r["sigma"])] for r in csv.DictReader(f)])
own = own[own[:, 2] < 0.5]
v = np.interp(sec + 0.5, own[:, 0], own[:, 1])
gap = np.interp(sec + 0.5, own[:, 0], np.r_[0, np.diff(own[:, 0])])
a = np.gradient(v)
ok = np.all(np.isfinite(X), 1) & (gap < 2)
lags = [0, 1, 2]
for extra, label in [([], "field"), (["abs"], "field and |field|")]:
    for lag in lags:
        Xl = np.roll(X, lag, 0)
        F = np.c_[Xl, np.abs(Xl).sum(1)] if extra else Xl
        pred = np.full(len(a), np.nan)
        idx = np.nonzero(ok)[0]
        for fold in np.array_split(idx, 5):
            tr = np.setdiff1d(idx, fold)
            A = np.c_[F[tr], np.ones(len(tr))]
            w, *_ = np.linalg.lstsq(A, a[tr], rcond=None)
            pred[fold] = np.c_[F[fold], np.ones(len(fold))] @ w
        e = pred[ok] - a[ok]
        r2 = 1 - np.var(e) / np.var(a[ok])
        moving = ok & (v > 3)
        r2m = 1 - np.var(pred[moving] - a[moving]) / np.var(a[moving])
        print(f"{label:18s} lag {lag} s: cross-validated R^2 {r2:.2f} (moving only {r2m:.2f}), rms error {np.sqrt(np.mean(e**2)):.2f} m/s^2 "
              f"vs acceleration rms {a[ok].std():.2f} m/s^2, {ok.sum()} s")

# Sign-free: the size of the fast field against the size of the acceleration (power or regenerative braking both
# draw current), and per acceleration band.
mag = np.linalg.norm(X, axis=1)
k = ok & (v > 1)
print(f"corr(|fast field|, |a|) {np.corrcoef(mag[k], np.abs(a[k]))[0, 1]:.2f}; corr with a {np.corrcoef(mag[k], a[k])[0, 1]:.2f}")
for lo, hi in [(-2, -0.5), (-0.5, -0.15), (-0.15, 0.15), (0.15, 0.5), (0.5, 2)]:
    s = k & (a >= lo) & (a < hi)
    if s.any():
        print(f"  a in [{lo:+.2f}, {hi:+.2f}) m/s^2: {s.sum():4d} s, median |fast field| {np.median(mag[s]):5.1f} uT")

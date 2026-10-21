"""Travel axis by spread (centered covariance, as in Rust) vs. by correlation with GNSS speed changes, scored by
along-track acceleration against our own Doppler's, per 1 s, on the elevated 7 (`154951`, run sweep-nooutage)."""

import csv
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS, imu  # noqa: E402

name, run, t_end = "154951", "sweep-nooutage", 1360.0
tau_axis = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
rate = 50.0
dt = 1 / rate
t, acc, gyr, t0 = imu(name, rate)
with open(ANALYSIS / run / "own.csv") as f:
    own = np.array([[(int(r["t_ns"]) - t0) * 1e-9, float(r["speed"]), float(r["sigma"])] for r in csv.DictReader(f)])

# Gravity as in Rust while GNSS is recent: 30 s, without the linear-acceleration subtraction (small effect).
k = 1 - np.exp(-dt / 30)
g = acc[0].copy()
h = np.empty_like(acc)
for i in range(len(t)):
    g -= np.cross(gyr[i], g) * dt
    g += k * (acc[i] - g)
    u = g / np.linalg.norm(g)
    lin = acc[i] - g
    h[i] = lin - (lin @ u) * u

# GNSS acceleration between consecutive confident epochs, physically possible ones only (<= 1.5 m/s^2).
ot, ov, os_ = own.T
pairs = [(i, i + 1) for i in range(len(ot) - 1)
         if os_[i] < 0.5 and os_[i + 1] < 0.5 and 0.5 < ot[i + 1] - ot[i] < 1.5 and ot[i + 1] < t_end]
ref = {}
for i, j in pairs:
    a = (ov[j] - ov[i]) / (ot[j] - ot[i])
    if abs(a) <= 1.5:
        ref[int(round(ot[j] * rate))] = (int(round(ot[i] * rate)), a)


def run_axis(mode):
    kx = 1 - np.exp(-dt / tau_axis)
    mean = np.zeros(3)
    cov = np.zeros((3, 3))
    cross = np.zeros(3)
    axis = np.array([1.0, 0, 0])
    hsum = np.zeros(3)
    along = np.empty(len(t))
    for i in range(len(t)):
        w = gyr[i]
        W = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        for v in (mean, cross, axis, hsum):
            v -= np.cross(w, v) * dt
        cov = cov - (W @ cov - cov @ W) * dt
        hsum += h[i] * dt
        if mode == "spread":
            mean += kx * (h[i] - mean)
            d = h[i] - mean
            cov += kx * (np.outer(d, d) - cov)
            x = axis
            for _ in range(3):
                x = cov @ x
                x /= np.linalg.norm(x)
            axis = x if x @ axis >= 0 else -x
        elif i in ref:
            i0, a = ref[i]
            hm = hsum / ((i - i0) * dt)
            # Weight per GNSS pair: exponential over tau_axis in time.
            kc = 1 - np.exp(-(i - i0) * dt / tau_axis)
            cross += kc * (hm * a - cross)
            if np.linalg.norm(cross) > 1e-3:
                axis = cross / np.linalg.norm(cross)
        if i in ref or mode == "spread":
            pass
        if i + 1 < len(t) and (i + 1) in ref:
            hsum[:] = 0
        along[i] = h[i] @ axis
    return along


for mode in ["spread", "gnss"]:
    along = run_axis(mode)
    errs = []
    for j, (i0, a) in ref.items():
        if j * dt > 60:
            errs.append(along[i0:j].mean() - a)
    e = np.array(errs)
    # Sign: if the spread axis is backward, it's still "right" up to sign, as the Rust filter learns its sign.
    flipped = np.array(errs) + 2 * np.array([a for j, (i0, a) in ref.items() if j * dt > 60])
    print(f"{mode:7s} axis tau {tau_axis:4.0f} s: along vs GNSS rms {np.sqrt(np.mean(e**2)):.3f} m/s^2 "
          f"(sign-flipped {np.sqrt(np.mean(flipped**2)):.3f}), n={len(e)}")
    # Per 2 minutes, to see sign flips over time.
    print("   per 120 s:", " ".join(f"{np.sqrt(np.mean(e[(np.array(list(ref))[[j * dt > 60 for j in ref]] * dt // 120) == m]**2)):.2f}"
                              for m in range(0, int(t_end // 120) + 1)))

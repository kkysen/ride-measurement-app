"""The estimator's travel axis (uncentered horizontal-acceleration covariance, 30 s, principal axis) against
the axis that best explains GNSS acceleration, per minute; and its along-axis acceleration against GNSS's."""

import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import tilt  # noqa: E402  (runs the gravity comparison too, and shares its data)

t, acc, gyr, rate, gt, ga = tilt.t, tilt.acc, tilt.gyr, tilt.rate, tilt.gt, tilt.ga
dt = 1 / rate
GRAVITY_TAU = float(sys.argv[3]) if len(sys.argv) > 3 else 600.0
CENTERED = len(sys.argv) > 4 and sys.argv[4] == "centered"
h = tilt.horizontal(tilt.gravity(GRAVITY_TAU))
k = 1 - np.exp(-dt / 30)
mean = np.zeros(3)
cov = np.zeros((3, 3))
axis = np.array([1.0, 0, 0])
axes = np.empty_like(h)
for i in range(len(t)):
    w = gyr[i]
    W = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    cov = cov - (W @ cov - cov @ W) * dt
    axis = axis - np.cross(w, axis) * dt
    mean = mean - np.cross(w, mean) * dt
    mean += k * (h[i] - mean)
    d = h[i] - mean if CENTERED else h[i]
    cov += k * (np.outer(d, d) - cov)
    x = axis
    for _ in range(3):
        x = cov @ x
        x /= np.linalg.norm(x)
    if x @ axis < 0:
        x = -x
    axis = x
    axes[i] = axis
along = np.sum(h * axes, axis=1)

idx = np.searchsorted(t, gt)
hw = np.array([h[max(i - 50, 0) : i + 50].mean(axis=0) for i in idx])
aw = np.array([along[max(i - 50, 0) : i + 50].mean() for i in idx])
print(f"gravity tau {GRAVITY_TAU} s, {'centered' if CENTERED else 'uncentered'} axis")
diffs = []
for w0 in np.arange(0, tilt.t_end, 60):
    m = (gt >= w0) & (gt < w0 + 60)
    if m.sum() < 20:
        continue
    c = np.corrcoef(aw[m], ga[m])[0, 1]
    s = np.sign(c) or 1
    e = s * aw[m] - ga[m]
    diffs.append(e)
    print(f"{w0:5.0f} s  corr {c:+.2f}  mean error {np.mean(e):+.2f}, rms {np.sqrt(np.mean(e**2)):.2f} m/s^2")
e = np.concatenate(diffs)
print(f"all: mean error {np.mean(e):+.2f}, rms {np.sqrt(np.mean(e**2)):.2f} m/s^2")

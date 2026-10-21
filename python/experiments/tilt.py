"""How well each gravity estimate leaves along-track acceleration matching GNSS, on the elevated 7 (`154951`).

For each method, horizontal acceleration (gravity removed) is fit per 60 s window to the GNSS acceleration
with the best axis in the phone's frame; the residual is the error from gravity leaking in, not from the axis.
"""

import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import fixes, imu  # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "154951"
t_end = float(sys.argv[2]) if len(sys.argv) > 2 else 1300.0
rate = 50.0
t, acc, gyr, t0 = imu(name, rate)
f = fixes(name, t0)

# GNSS acceleration at each fix: central difference of the phone's speed, where it's confident.
ft, fv, fa = f["t"], f["speed"], f["speed_acc"]
good = (fa[:-2] < 1.0) & (fa[2:] < 1.0) & (np.diff(ft, 2) < 2.5) & (ft[1:-1] < t_end)
gt = ft[1:-1][good]
ga = ((fv[2:] - fv[:-2]) / (ft[2:] - ft[:-2]))[good]
print(f"{len(gt)} GNSS accelerations, rms {np.sqrt(np.mean(ga**2)):.2f} m/s^2, max |a| {np.max(np.abs(ga)):.2f}")


def gravity(tau, linear=None):
    """Gyro-propagated gravity, pulled toward the accelerometer (minus `linear` if given) with time constant tau."""
    dt = 1 / rate
    k = 1 - np.exp(-dt / tau)
    g = acc[0].copy()
    out = np.empty_like(acc)
    for i in range(len(t)):
        g -= np.cross(gyr[i], g) * dt
        target = acc[i] if linear is None else acc[i] - linear[i]
        g += k * (target - g)
        out[i] = g
    return out


def horizontal(gr):
    u = gr / np.linalg.norm(gr, axis=1, keepdims=True)
    lin = acc - gr
    return lin - np.sum(lin * u, axis=1, keepdims=True) * u


def evaluate(h, label):
    # Average to 1 s around each GNSS acceleration (its central difference spans 2 s).
    idx = np.searchsorted(t, gt)
    hw = np.array([h[max(i - 50, 0) : i + 50].mean(axis=0) for i in idx])
    res = []
    for w0 in np.arange(0, t_end, 60):
        m = (gt >= w0) & (gt < w0 + 60)
        if m.sum() < 20:
            continue
        x, *_ = np.linalg.lstsq(hw[m], ga[m], rcond=None)
        res.append(ga[m] - hw[m] @ x)
    r = np.concatenate(res)
    print(f"{label:40s} residual rms {np.sqrt(np.mean(r**2)):.3f} m/s^2, 95% {np.percentile(np.abs(r), 95):.3f}")


if __name__ == "__main__":
    evaluate(acc - acc.mean(axis=0), "no gravity tracking (fixed mean)")
for tau in ([] if __name__ != "__main__" else [600, 60, 20, 10, 5, 2, 1]):
    evaluate(horizontal(gravity(tau)), f"gyro + accel, tau {tau} s")

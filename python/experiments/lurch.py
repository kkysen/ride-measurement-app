"""Along-track acceleration around each GNSS-confirmed stop, to see the lurch as a train comes to rest.

Gravity follows the accelerometer over 20 s; the axis is the horizontal direction that best explains
GNSS acceleration in the minute before each stop. Stop time: the first fix at or under 0.3 m/s after 3 m/s.
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
dt = 1 / rate
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

ft, fv = f["t"], f["speed"]
stops = []
moving = False
for i in range(len(ft)):
    if ft[i] > t_end:
        break
    if fv[i] > 3:
        moving = True
    elif moving and fv[i] <= 0.3:
        stops.append(ft[i])
        moving = False

ga_t = ft[1:-1]
ga = (fv[2:] - fv[:-2]) / (ft[2:] - ft[:-2])
offsets = np.arange(-8, 4.01, 0.5)
print("stop    " + " ".join(f"{o:+5.1f}" for o in offsets) + "   (along, m/s^2, 0.5 s means; 0 = GNSS stop fix)")
rows = []
for s in stops:
    m = (ga_t > s - 60) & (ga_t < s - 2)
    if m.sum() < 20:
        continue
    idx = np.searchsorted(t, ga_t[m])
    hw = np.array([h[max(i - 50, 0) : i + 50].mean(axis=0) for i in idx])
    ax, *_ = np.linalg.lstsq(hw, ga[m], rcond=1e-3)
    along = h @ ax
    row = []
    for o in offsets:
        j = np.searchsorted(t, s + o)
        row.append(along[j - 12 : j + 13].mean())
    # Jerk: the largest 0.2 s change in a 0.2 s mean, within -6..+2 s.
    w = 10
    sm = np.convolve(along, np.ones(w) / w, mode="same")
    a, b = np.searchsorted(t, [s - 6, s + 2])
    jerk = np.diff(sm[a:b:w]) / (w * dt)
    j_at = t[a:b:w][np.argmax(jerk) + 1] - s
    rows.append(row)
    print(f"{s:6.0f}  " + " ".join(f"{x:+5.2f}" for x in row) + f"   max jerk {jerk.max():+.1f} m/s^3 at {j_at:+.1f} s")
print("mean    " + " ".join(f"{x:+5.2f}" for x in np.mean(rows, axis=0)))

# False alarms: the same jerk measure (on |h|, no axis needed) while moving steadily vs. near stops.
mag = np.convolve(np.linalg.norm(h, axis=1), np.ones(10) / 10, mode="same")
hj = np.abs(np.diff(mag[::10])) / 0.2
tj = t[::10][1:]
speed_at = np.interp(tj, ft, fv)
near = np.zeros(len(tj), bool)
for s in stops:
    near |= (tj > s - 3) & (tj < s + 1)
cruise = (speed_at > 8) & (tj < t_end)
print(f"|h| jerk, 0.2 s steps: near stops 99th pct {np.percentile(hj[near], 99):.1f}, max {hj[near].max():.1f}; "
      f"cruising >29 km/h 99th pct {np.percentile(hj[cruise], 99):.1f}, 99.9th {np.percentile(hj[cruise], 99.9):.1f}, max {hj[cruise].max():.1f} m/s^3")
for thr in [3, 5, 8]:
    ev = tj[(hj > thr) & cruise]
    print(f"  > {thr} m/s^3 while cruising: {len(ev)} steps over {cruise.sum() * 0.2:.0f} s")

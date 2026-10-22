"""Each car beacon's forward direction in 3D and its gain, from GNSS's acceleration, and whether they hold across rides.

The live estimator (`crates/fusion/src/live_speed.rs`, `CarAxis::forward`) takes one horizontal axis and its sign,
and the cars read 0.6 to 0.75 of GNSS's acceleration on it. Here each car's readings (de-meaned over 60 s, which
removes gravity and the mount's tilt) are regressed on GNSS's along-track acceleration (from the phone's own speed,
where its accuracy is good) per second: `car = g * a * u + noise`, with `u` a unit vector, `g` the gain.
`u` and `g` come from the least-squares fit of the three axes on `a`; the single-axis fit is the old model.
Rides with car beacons and GNSS: `154951` (the 7, elevated), `184134` (the 7 and Queens Boulevard), `151449`.

Needs `target/analysis/<ride>/{ble,fixes}.csv`."""

import csv
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ANALYSIS = ROOT / "target/analysis"
PREFIX = "e2c56db5dffb48d2b060d0f5a710"
COUNT = 9.80665 / 64
RIDES = ["154951", "184134", "151449"]


def signed(b):
    return b - 256 if b >= 128 else b


def gnss_accel(ride):
    t, v = [], []
    with open(ANALYSIS / ride / "fixes.csv") as f:
        for r in csv.DictReader(f):
            if r["speed"] and r["speed_acc"] and float(r["speed_acc"]) < 1.0:
                t.append(int(r["t_ns"]) / 1e9)
                v.append(float(r["speed"]))
    t, v = np.array(t), np.array(v)
    grid = np.arange(t[0], t[-1], 1.0)
    vg = np.interp(grid, t, v)
    near = np.array([np.min(np.abs(t - g)) for g in grid]) < 1.5
    a = np.gradient(vg)
    return grid, a, near


def cars(ride):
    out = defaultdict(list)
    with open(ANALYSIS / ride / "ble.csv") as f:
        for r in csv.DictReader(f):
            i = r["data"].find(PREFIX)
            if i < 0:
                continue
            b = bytes.fromhex(r["data"][i + 32 : i + 42])
            out[r["data"][i + 28 : i + 32]].append((int(r["t_ns"]) / 1e9, signed(b[0]), signed(b[1]), signed(b[2])))
    return {k: np.array(v) for k, v in out.items() if len(v) >= 200}


def fit(ride):
    grid, a, near = gnss_accel(ride)
    res = {}
    for car, x in cars(ride).items():
        # Per second: the car's mean reading, de-meaned over 60 s.
        sec = np.floor(x[:, 0]).astype(int)
        keys, idx = np.unique(sec, return_inverse=True)
        m = np.stack([np.bincount(idx, x[:, k + 1]) / np.bincount(idx) for k in range(3)], axis=1) * COUNT
        level = np.stack([np.convolve(m[:, k], np.ones(61) / 61, mode="same") for k in range(3)], axis=1)
        y = m - level
        gi = np.searchsorted(grid, keys)
        ok = (gi > 0) & (gi < len(grid)) & (np.abs(grid[np.clip(gi, 0, len(grid) - 1)] - keys) < 0.6)
        ok &= near[np.clip(gi, 0, len(grid) - 1)]
        ok[:30] = ok[-30:] = False
        if ok.sum() < 60:
            continue
        aa, yy = a[gi[ok]], y[ok]
        w = yy.T @ aa / (aa @ aa)  # car = w * a, per axis
        g = np.linalg.norm(w)
        u = w / g
        # Read back: the car's estimate of a is (u . y) / g.
        est3 = yy @ u / g
        k = int(np.argmax(np.abs(w)))
        est1 = yy[:, k] / w[k]
        raw1 = yy[:, k] * np.sign(w[k])
        rms = lambda e: float(np.sqrt(np.mean(e**2)))  # noqa: E731
        res[car] = dict(n=int(ok.sum()), u=u, g=g, axis=k, gain1=abs(w[k]),
                        corr3=float(np.corrcoef(yy @ u, aa)[0, 1]), corr1=float(np.corrcoef(raw1, aa)[0, 1]),
                        err3=rms(est3 - aa), err1=rms(est1 - aa), err_raw=rms(raw1 - aa), a_rms=rms(aa))
    return res


def main(rides):
    seen = defaultdict(list)
    for ride in rides:
        print(f"== {ride}")
        for car, r in sorted(fit(ride).items(), key=lambda kv: -kv[1]["n"]):
            seen[car].append((ride, r))
            print(f"  {car}: {r['n']:4d} s, u ({r['u'][0]:+.2f}, {r['u'][1]:+.2f}, {r['u'][2]:+.2f}), gain {r['g']:.2f}"
                  f" (axis {'xyz'[r['axis']]} alone {r['gain1']:.2f}); corr 3D {r['corr3']:.2f} vs 1 axis {r['corr1']:.2f};"
                  f" error 3D+gain {r['err3']:.2f}, 1 axis+gain {r['err1']:.2f}, 1 axis raw {r['err_raw']:.2f}"
                  f" (signal {r['a_rms']:.2f} m/s^2)")
    print("== cars on more than one ride")
    for car, rs in seen.items():
        if len(rs) > 1:
            print(f"  {car}: " + "; ".join(f"{ride} u ({r['u'][0]:+.2f}, {r['u'][1]:+.2f}, {r['u'][2]:+.2f}) gain {r['g']:.2f}"
                                           for ride, r in rs))


if __name__ == "__main__":
    main(sys.argv[1:] or RIDES)

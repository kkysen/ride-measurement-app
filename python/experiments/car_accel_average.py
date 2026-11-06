"""Several cars' beacon accelerometers at once (`train_beacons.py`): each car's forward axis and sign from its fit
to GNSS acceleration (the phone's own speed, differentiated, where fixes are 1 s apart), then per car and
averaged over the cars heard in the same second: noise (the residual's rms) against the signal, and whether two cars
agree with each other better than either agrees with GNSS (the "noise" is the train's own motion) or not
(it's the beacons' sensors or their 1/64 g steps). Needs `ble.csv` and `fixes.csv` in `data/work/analysis/<ride>`."""

import csv
import itertools
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS  # noqa: E402

PREFIX = "e2c56db5dffb48d2b060d0f5a710"
G = 9.81 / 64
MIN_S = 60


def signed(b):
    return b - 256 if b >= 128 else b


def cars(ride):
    out = {}
    with open(ANALYSIS / ride / "ble.csv") as f:
        for r in csv.DictReader(f):
            i = r["data"].find(PREFIX)
            if i < 0:
                continue
            cid = r["data"][i + 28 : i + 32]
            b = bytes.fromhex(r["data"][i + 32 : i + 42])
            out.setdefault(cid, []).append((int(r["t_ns"]) / 1e9, signed(b[0]) * G, signed(b[1]) * G, signed(b[2]) * G))
    return {k: np.array(v) for k, v in out.items() if len(v) >= 30 and k != "96e0"}


def gnss_accel(ride):
    rows = list(csv.DictReader(open(ANALYSIS / ride / "fixes.csv")))
    t = np.array([int(r["t_ns"]) / 1e9 for r in rows])
    v = np.array([float(r["speed"]) if r["speed"] else np.nan for r in rows])
    ok = np.isfinite(v)
    t, v = t[ok], v[ok]
    grid = np.arange(np.ceil(t[0]), t[-1], 1.0)
    near = np.abs(t[np.clip(np.searchsorted(t, grid), 0, len(t) - 1)] - grid) < 1.5
    vv = np.interp(grid, t, v)
    a = np.gradient(vv)
    a[~near] = np.nan
    return grid, a


def per_second(c, grid):
    """Each second's mean of the horizontal axes (x, y); NaN when not heard."""
    out = np.full((len(grid), 2), np.nan)
    idx = np.searchsorted(grid, c[:, 0] - 0.5)
    for k in range(len(grid)):
        m = idx == k
        if m.any():
            out[k] = c[m, 1:3].mean(0)
    return out


def main(rides):
    for ride in rides:
        grid, a = gnss_accel(ride)
        fwd = {}
        for cid, c in cars(ride).items():
            xy = per_second(c, grid)
            ok = np.isfinite(a) & np.isfinite(xy[:, 0])
            if ok.sum() < MIN_S:
                continue
            xy0 = xy - np.nanmean(xy, 0)
            # The forward axis: least squares of GNSS acceleration on the two horizontal axes (sign included).
            w, *_ = np.linalg.lstsq(xy0[ok], a[ok], rcond=None)
            f = xy0 @ (w / np.linalg.norm(w))
            corr = np.corrcoef(f[ok], a[ok])[0, 1]
            if abs(corr) < 0.4:
                continue
            fwd[cid] = f * np.sign(corr)
        if not fwd:
            print(f"{ride}: no car with enough GNSS")
            continue
        print(f"== {ride}: {len(fwd)} cars, GNSS acceleration std {np.nanstd(a):.2f} m/s^2")
        for cid, f in fwd.items():
            ok = np.isfinite(a) & np.isfinite(f)
            k = np.polyfit(f[ok], a[ok], 1)
            res = a[ok] - np.polyval(k, f[ok])
            print(f"  {cid}: {ok.sum():4d} s, corr {np.corrcoef(f[ok], a[ok])[0, 1]:.2f}, gain {k[0]:.2f}, residual {res.std():.2f}")
        stack = np.array(list(fwd.values()))
        n = np.isfinite(stack).sum(0)
        avg = np.where(n > 0, np.nanmean(np.where(np.isfinite(stack), stack, np.nan), 0), np.nan)
        for need in (1, 2, 3):
            ok = np.isfinite(a) & (n >= need)
            if ok.sum() < MIN_S:
                continue
            k = np.polyfit(avg[ok], a[ok], 1)
            res = a[ok] - np.polyval(k, avg[ok])
            print(f"  average of {need}+ cars: {ok.sum():4d} s, corr {np.corrcoef(avg[ok], a[ok])[0, 1]:.2f}, residual {res.std():.2f}")
        for (c1, f1), (c2, f2) in itertools.combinations(fwd.items(), 2):
            ok = np.isfinite(f1) & np.isfinite(f2) & np.isfinite(a)
            if ok.sum() < MIN_S:
                continue
            print(f"  {c1} with {c2}: {ok.sum():4d} s, each other {np.corrcoef(f1[ok], f2[ok])[0, 1]:.2f},"
                  f" with GNSS {np.corrcoef(f1[ok], a[ok])[0, 1]:.2f} and {np.corrcoef(f2[ok], a[ok])[0, 1]:.2f},"
                  f" difference {np.std(f1[ok] - f2[ok]):.2f}")


if __name__ == "__main__":
    main(sys.argv[1:] or ["184134", "154951", "151449", "155919"])

"""More signals for telling the subway from the street, per 10 s window, scored like `subway.py` against the
rider's marks (which only score): the train's BLE beacon, position against the subway map (last fix near an
entrance, fix on a track), pressure below the last fix and tunnel waves, the magnetometer (traction current),
cell, hunting (the bogies' lateral sway), and sound (door chimes, OMNY beeps). Then a logistic regression on them,
fitted on all rides but one and scored on that one, for each ride in turn.

Needs `data/gtfs/` (the MTA's GTFS `stops.txt`, `shapes.txt`, and `entrances.csv` from data.ny.gov),
the CSVs from `crates/cli/examples/export.rs` and `radio.rs`, and WAVs from `ride wav` in `data/work/audio/`."""

import csv
import sys
import wave
from pathlib import Path

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ride  # noqa: E402
from subway import ANALYSIS, RIDES, WINDOW_S, features as wifi_features, rows, windows  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
GTFS = ROOT / "data/gtfs"
SESSIONS = {r: ROOT / "data/sessions" / (("20261021-" if r == "135509" else "20261020-") + r) for r in RIDES}
BEACON = "e2c56db5dffb48d2b060d0f5a710"
GOOD_FIX_M = 50.0


def local_m(lat, lon, lat0=40.75, lon0=-73.95):
    """Meters east and north of a point in the city (an equirectangular projection is plenty here)."""
    return (np.asarray(lon) - lon0) * 111_320 * np.cos(np.radians(lat0)), (np.asarray(lat) - lat0) * 110_540


class Grid:
    """Nearest-point distance by a 100 m grid."""

    def __init__(self, x, y, cell=100.0):
        self.cell, self.cells = cell, {}
        for i, (a, b) in enumerate(zip(x, y)):
            self.cells.setdefault((int(a // cell), int(b // cell)), []).append((a, b))

    def dist(self, a, b, reach=5):
        ca, cb, best = int(a // self.cell), int(b // self.cell), np.inf
        for da in range(-reach, reach + 1):
            for db in range(-reach, reach + 1):
                for x, y in self.cells.get((ca + da, cb + db), ()):
                    best = min(best, np.hypot(x - a, y - b))
        return best


def subway_map():
    with open(GTFS / "entrances.csv") as f:
        e = list(csv.DictReader(f))
    ex, ey = local_m([float(r["Entrance Latitude"]) for r in e], [float(r["Entrance Longitude"]) for r in e])
    with open(GTFS / "shapes.txt") as f:
        s = list(csv.DictReader(f))
    sx, sy = local_m([float(r["shape_pt_lat"]) for r in s], [float(r["shape_pt_lon"]) for r in s])
    # Shapes are sparse on straight track: fill in points every 20 m so the grid sees the line between them.
    fx, fy = [], []
    for i in range(len(s) - 1):
        if s[i]["shape_id"] != s[i + 1]["shape_id"]:
            continue
        n = max(1, int(np.hypot(sx[i + 1] - sx[i], sy[i + 1] - sy[i]) // 20))
        for k in range(n):
            fx.append(sx[i] + (sx[i + 1] - sx[i]) * k / n)
            fy.append(sy[i] + (sy[i + 1] - sy[i]) * k / n)
    return Grid(ex, ey), Grid(fx, fy)


def per_window(starts, t_ns, values, fn):
    """`fn` of each window's values (times in ns), NaN where a window has none."""
    w = int(WINDOW_S * 1e9)
    i = (np.asarray(t_ns) - starts[0]) // w
    out = np.full(len(starts), np.nan)
    order = np.argsort(i, kind="stable")
    i, values = i[order], np.asarray(values)[order]
    bounds = np.searchsorted(i, np.arange(len(starts) + 1))
    for k in range(len(starts)):
        a, b = bounds[k], bounds[k + 1]
        if b > a:
            out[k] = fn(values[a:b])
    return out


def features(name, entrances, tracks):
    starts, labels = windows(name)
    f, _ = wifi_features(name)
    n, w = len(starts), int(WINDOW_S * 1e9)
    mids = starts + w // 2
    d = SESSIONS[name]

    # The train's beacon: sightings in the window, and whether any in the last minute.
    bt = np.array([int(r["t_ns"]) for r in rows(name, "ble.csv") if BEACON in r["data"].lower()], dtype=np.int64)
    f["beacon_n"] = np.array([((bt >= s) & (bt < s + w)).sum() for s in starts], dtype=float)
    f["beacon_recent"] = np.array([((bt >= m - 60e9) & (bt <= m)).any() for m in mids], dtype=float)

    # Position: the last good fix's distance to an entrance and its age; the window's own fix's distance to a track.
    fx = [(int(r["t_ns"]), float(r["lat"]), float(r["lon"])) for r in rows(name, "fixes.csv") if r["h_acc"] and float(r["h_acc"]) < GOOD_FIX_M]
    ft = np.array([t for t, _, _ in fx], dtype=np.int64)
    fxy = [local_m(lat, lon) for _, lat, lon in fx]
    f["fix_age_s"], f["last_fix_entrance_m"], f["fix_track_m"] = np.full(n, np.nan), np.full(n, np.nan), np.full(n, np.nan)
    for k, m in enumerate(mids):
        j = np.searchsorted(ft, m, side="right") - 1
        if j < 0:
            continue
        f["fix_age_s"][k] = (m - ft[j]) * 1e-9
        f["last_fix_entrance_m"][k] = min(entrances.dist(*fxy[j]), 2000.0)
        if m - ft[j] < w:
            f["fix_track_m"][k] = min(tracks.dist(*fxy[j]), 2000.0)

    # Pressure: below the level at the last good fix (hPa, positive is deeper), and the spread of what's left
    # after a 60 s moving average (tunnel pressure waves).
    p = ride.read_stream(d, "pressure")
    pt, pv = p.timestamps_ns, p.values[:, 0].astype(float)
    at_fix = np.interp(ft, pt, pv) if len(ft) else np.array([])
    f["below_fix_hpa"] = np.full(n, np.nan)
    for k, m in enumerate(mids):
        j = np.searchsorted(ft, m, side="right") - 1
        if j >= 0:
            f["below_fix_hpa"][k] = np.interp(m, pt, pv) - at_fix[j]
    rate = 1e9 / np.median(np.diff(pt))
    k60 = max(1, int(60 * rate))
    smooth = np.convolve(pv, np.ones(k60) / k60, mode="same")
    f["pressure_wave_hpa"] = per_window(starts, pt, pv - smooth, np.std)

    # Magnetometer: the field's strength (calibrated: raw minus Android's hard-iron estimate), its spread in the window.
    mg = ride.read_stream(d, "mag_uncal")
    b = np.linalg.norm(mg.values[:, :3].astype(float) - mg.values[:, 3:6], axis=1)
    f["mag_std_ut"] = per_window(starts, mg.timestamps_ns, b, np.std)

    # Cell: how many cells were heard in the window.
    ct = np.array([int(r["t_ns"]) for r in rows(name, "cell.csv")], dtype=np.int64)
    f["cells_n"] = per_window(starts, ct, np.ones(len(ct)), np.sum)

    # Hunting: horizontal acceleration at 50 Hz, gravity (its 2 s mean) projected out; the sharpest spectral peak
    # in 0.5 to 4 Hz against the band's median, over the two horizontal axes.
    a = ride.read_stream(d, "accel_uncal")
    av = a.values[:, :3].astype(float) - a.values[:, 3:6]
    t50 = np.arange(a.timestamps_ns[0], a.timestamps_ns[-1], int(2e7))
    acc = np.stack([np.interp(t50, a.timestamps_ns, av[:, i]) for i in range(3)], axis=1)
    g = np.stack([np.convolve(acc[:, i], np.ones(100) / 100, mode="same") for i in range(3)], axis=1)
    up = g / np.linalg.norm(g, axis=1, keepdims=True)
    lin = acc - g
    lin_h = lin - (lin * up).sum(1, keepdims=True) * up
    # Two horizontal axes, turning with the phone: any vector not along up, made perpendicular, and up x it.
    ref = np.where(np.abs(up[:, :1]) < 0.9, [[1.0, 0, 0]], [[0, 1.0, 0]])
    e1 = ref - (ref * up).sum(1, keepdims=True) * up
    e1 /= np.linalg.norm(e1, axis=1, keepdims=True)
    e2 = np.cross(up, e1)
    h = np.stack([(lin_h * e1).sum(1), (lin_h * e2).sum(1)], axis=1)
    freqs = np.fft.rfftfreq(500, 1 / 50)
    band = (freqs >= 0.5) & (freqs <= 4)

    def hunting(idx):
        seg = h[idx]
        if len(seg) < 500:
            return np.nan
        spec = sum(np.abs(np.fft.rfft((seg[:500, i] - seg[:500, i].mean()) * np.hanning(500))) ** 2 for i in range(2))
        s = spec[band]
        return s.max() / max(np.median(s), 1e-12)

    w50 = ((t50 - starts[0]) // w).astype(np.int64)
    bounds = np.searchsorted(w50, np.arange(n + 1))
    f["hunting_peak"] = np.array([hunting(np.arange(bounds[k], bounds[k + 1])) for k in range(n)])

    # Sound: per 1 s frame, the strongest narrow peak against its band's median, in the door chimes' band
    # (400 to 700 Hz: 441/555 and 516/645 Hz) and the OMNY beep's (2800 to 3050 Hz); the window's maximum.
    f["chime_snr"], f["omny_snr"] = np.full(n, np.nan), np.full(n, np.nan)
    ts = np.loadtxt(ANALYSIS / name / "audio_ts.csv", delimiter=",", skiprows=1, dtype=np.int64)
    with wave.open(str(ROOT / "data/work/audio" / f"{name}.wav")) as wf:
        sr = wf.getframerate()
        pcm = np.frombuffer(wf.readframes(wf.getnframes()), dtype="<i2").astype(float)
    frame_t = np.interp(np.arange(0, len(pcm) - sr, sr), ts[:, 0], ts[:, 1])
    af = np.fft.rfftfreq(sr, 1 / sr)
    chime_b, omny_b = (af >= 400) & (af <= 700), (af >= 2800) & (af <= 3050)
    snr = np.zeros((len(frame_t), 2))
    win = np.hanning(sr)
    for i in range(len(frame_t)):
        spec = np.abs(np.fft.rfft(pcm[i * sr : (i + 1) * sr] * win)) ** 2
        for j, bnd in enumerate((chime_b, omny_b)):
            s = spec[bnd]
            snr[i, j] = s.max() / max(np.median(s), 1e-12)
    f["chime_snr"] = per_window(starts, frame_t, snr[:, 0], np.max)
    f["omny_snr"] = per_window(starts, frame_t, snr[:, 1], np.max)
    return f, labels


def auc(x, y):
    pos, neg = x[y == 1], x[y == 0]
    pos, neg = pos[~np.isnan(pos)], neg[~np.isnan(neg)]
    if not len(pos) or not len(neg):
        return np.nan
    return (pos[:, None] > neg[None, :]).mean() + (pos[:, None] == neg[None, :]).mean() / 2


def design(f, names, stats=None):
    """Features as columns, log-scaled where they span decades, NaNs as the training median plus a flag."""
    cols = []
    for k in names:
        x = f[k].astype(float)
        if k in LOG:
            x = np.log1p(np.maximum(x, 0))
        cols.append(x)
    x = np.stack(cols, axis=1)
    if stats is None:
        med = np.nanmedian(x, axis=0)
        filled = np.where(np.isnan(x), med, x)
        stats = (med, filled.mean(0), filled.std(0) + 1e-9)
    med, mu, sd = stats
    missing = np.isnan(x).astype(float)
    x = (np.where(np.isnan(x), med, x) - mu) / sd
    return np.concatenate([x, missing], axis=1), stats


def fit(x, y, l2=1.0, steps=3000, lr=0.1):
    w, b = np.zeros(x.shape[1]), 0.0
    for _ in range(steps):
        p = 1 / (1 + np.exp(-(x @ w + b)))
        w -= lr * (x.T @ (p - y) / len(y) + l2 * w / len(y))
        b -= lr * (p - y).mean()
    return w, b


LOG = {"wifi_n", "ble_n", "beacon_n", "fix_age_s", "last_fix_entrance_m", "fix_track_m", "mag_std_ut", "cells_n",
       "hunting_peak", "chime_snr", "omny_snr", "pressure_wave_hpa"}


def main():
    entrances, tracks = subway_map()
    per = {}
    for r in RIDES:
        f, labels = features(r, entrances, tracks)
        per[r] = (f, np.array([np.nan if l is None else l for l in labels]))
        print(f"{r}: features done", file=sys.stderr)
    names = list(per[RIDES[0]][0])
    print("AUC on-train vs off (0.5 = no signal; far from 0.5 either way is signal):")
    print(f"{'feature':20s}" + "".join(f"{r:>9s}" for r in RIDES) + "      all")
    for k in names:
        vals = [auc(per[r][0][k], per[r][1]) for r in RIDES]
        allx = np.concatenate([per[r][0][k] for r in RIDES])
        ally = np.concatenate([per[r][1] for r in RIDES])
        print(f"{k:20s}" + "".join(f"{v:9.2f}" for v in vals) + f"{auc(allx, ally):9.2f}")
    # Leave one ride out: fit on the others, score the held-out ride.
    print("\nlogistic regression, each ride scored by a model fitted on the others:")
    for held in RIDES:
        train = [r for r in RIDES if r != held]
        xs, ys = [], []
        for r in train:
            f, y = per[r]
            xs.append({k: f[k][y >= 0] for k in names})
            ys.append(y[y >= 0])
        ftrain = {k: np.concatenate([x[k] for x in xs]) for k in names}
        ytrain = np.concatenate(ys)
        xtr, stats = design(ftrain, names)
        w, b = fit(xtr, ytrain)
        f, y = per[held]
        xte, _ = design(f, names, stats)
        p = 1 / (1 + np.exp(-(xte @ w + b)))
        lab = y >= 0
        on = p[lab & (y == 1)] > 0.5
        off = p[lab & (y == 0)] > 0.5
        print(f"  {held}: AUC {auc(p, y):.3f}; in the subway on {on.mean() * 100:5.1f}% of {len(on)} train windows, "
              f"{off.mean() * 100:5.1f}% of {len(off)} off-train windows")
    xa, stats = design({k: np.concatenate([per[r][0][k][per[r][1] >= 0] for r in RIDES]) for k in names}, names)
    w, _ = fit(xa, np.concatenate([per[r][1][per[r][1] >= 0] for r in RIDES]))
    print("\nweights on all rides (standardized; positive means subway):")
    for k, v in sorted(zip(names + [k + "_missing" for k in names], w), key=lambda kv: -abs(kv[1]))[:20]:
        print(f"  {v:+6.2f}  {k}")


if __name__ == "__main__":
    main()

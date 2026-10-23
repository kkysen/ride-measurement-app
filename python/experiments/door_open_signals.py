"""Which sensors mark the doors opening, and can any of them time it better than the stop + 3.2 s?

Method: every signal on a 10 Hz grid; its step d(t) = mean over (t, t + 1 s] minus mean over (t - 1 s, t]
(for vectors, the norm of the per-axis steps), as a z score against the ride's median absolute step
inside stop windows (the stop record's `stopped_ns` to `doors_closed_ns`, from `stops2-<ride>/stops.csv`).
Signals: audio band levels (dB: broadband 50 Hz to 8 kHz, 20-100 Hz, 100-500 Hz, 500 Hz-2 kHz, 2-8 kHz),
barometric pressure (Pa), magnetometer (uncalibrated vector, uT), and accelerometer and gyro activity
(log rms about a 1 s mean, the car rocking as people move).
Each "doors open" tap is paired to the stop whose window holds it (from 3 s before the stop).
Event-locked: the median z at each second from -10 to +10 s about the tap.
Detector, per signal: the time of the largest signed step (sign from training) between stop + 0.5 s and
the closing - 3 s, shifted by the training rides' median lag to the tap; if that step's z is under a bar
tuned on the training rides, fall back to stop + 3.2 s. Leave-one-ride-out: every ride is held out once,
tuned on the other seven, so all scores are on held-out rides. Taps only score.

Result (2026-10-23), 8 rides, 36 taps paired with a stop record that has a closing:
- Audio: a small rise right at the tap (median z +1 to +2 at -1..+1 s, 100-500 Hz and 2-8 kHz most),
  too weak against the other steps in the window: no band beats the baseline (16 to 22 within 1 s).
- Pressure: a small drop at 0..+1 s (median z -1.7), the clearest non-audio mark,
  but the detector on it hits only 14 within 1 s.
- Magnetometer: no step at the tap (flat z about +0.6 to +1.0 throughout); 20 within 1 s.
- Accelerometer and gyro activity: a one-sample spike at 0 s (z +2), most likely the rider's own tap
  on the phone, so not usable; 20 to 22 within 1 s, mostly fallbacks.
Baseline stop + 3.2 s: 22 within 1 s, 30 within 2 s, median error 0.8 s.
No single signal beats it on held-out rides: the tuned bars mostly fall back to the baseline.
Keep stop + 3.2 s; a fused score (audio + pressure) would need more taps to tune.
"""

import csv
import sys
import wave
from pathlib import Path

import numpy as np
from scipy import signal

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS, ride  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
RIDES = ["135509", "151449", "154951", "155919", "182135", "184134", "214427", "215922"]
RATE = 10.0
W = int(RATE)  # 1 s step halves
BANDS = {"audio 50-8k": (50, 8000), "audio 20-100": (20, 100), "audio 100-500": (100, 500),
         "audio 500-2k": (500, 2000), "audio 2k-8k": (2000, 8000)}
BASELINE_S = 3.2


def audio(name, grid):
    ts = np.loadtxt(ANALYSIS / name / "audio_ts.csv", delimiter=",", skiprows=1, dtype=np.int64)
    with wave.open(str(ROOT / "target/audio" / f"{name}.wav")) as w:
        sr = w.getframerate()
        pcm = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float32)
    hop = int(sr / RATE)
    f, t, z = signal.stft(pcm, sr, nperseg=2 * hop, noverlap=hop, boundary=None, padded=False)
    p = np.abs(z) ** 2
    tn = np.interp(t * sr, ts[:, 0], ts[:, 1])
    return {k: np.interp(grid, tn, 10 * np.log10(p[(f >= a) & (f < b)].sum(0) + 1e-9)) for k, (a, b) in BANDS.items()}


def stream(name, kind):
    s = ride.read_stream(next((ROOT / "data/sessions").glob(f"*-{name}")), kind)
    return s.timestamps_ns, s.values.astype(float)


def activity(t, v, grid):
    """log rms of the vector about its 1 s moving mean, on the grid."""
    rate = 1e9 / np.median(np.diff(t))
    k = max(int(rate), 1)
    ker = np.ones(k) / k
    dev = v[:, :3] - np.column_stack([np.convolve(v[:, j], ker, mode="same") for j in range(3)])
    e = np.convolve((dev ** 2).sum(1), ker, mode="same")
    return np.log(np.interp(grid, t, e) + 1e-12)


def signals(name, grid):
    out = {k: v[:, None] for k, v in audio(name, grid).items()}
    t, v = stream(name, "pressure")
    out["pressure"] = np.interp(grid, t, v[:, 0] * 100)[:, None]
    t, v = stream(name, "mag_uncal")
    out["mag"] = np.column_stack([np.interp(grid, t, v[:, j]) for j in range(3)])
    t, v = stream(name, "accel_uncal")
    out["accel activity"] = activity(t, v, grid)[:, None]
    t, v = stream(name, "gyro_uncal")
    out["gyro activity"] = activity(t, v, grid)[:, None]
    return out


def step(x):
    """mean over the next W samples minus over the last W; signed for scalars, the norm for vectors."""
    c = np.vstack([np.zeros((1, x.shape[1])), np.cumsum(x, 0)])
    d = np.full(x.shape, np.nan)
    i = np.arange(W, len(x) - W)
    d[i] = (c[i + W] - c[i]) / W - (c[i] - c[i - W]) / W
    return d[:, 0] if x.shape[1] == 1 else np.linalg.norm(d, axis=1)


def load(name):
    with open(ANALYSIS / f"stops2-{name}" / "stops.csv") as f:
        stops = [(int(r["stopped_ns"]), int(float(r["doors_closed_ns"]))) for r in csv.DictReader(f) if r["doors_closed_ns"]]
    with open(ANALYSIS / name / "events.csv") as f:
        taps = [int(r["t_ns"]) for r in csv.DictReader(f) if r["kind"] == "KIND_DOORS_OPEN"]
    t0 = min(s for s, _ in stops) - 30e9
    t1 = max(c for _, c in stops) + 30e9
    grid = np.arange(t0, t1, 1e9 / RATE)
    z = {}
    inwin = np.zeros(len(grid), bool)
    for s, c in stops:
        inwin |= (grid >= s) & (grid <= c)
    for k, x in signals(name, grid).items():
        d = step(x)
        z[k] = d / (np.nanmedian(np.abs(d[inwin])) * 1.4826 + 1e-12)
    cases = []
    for s, c in stops:
        mine = [a for a in taps if s - 3e9 <= a <= c]
        if mine:
            cases.append((s, c, mine[0]))
    return grid, z, cases


def pick(grid, zk, s, c, sign):
    i0, i1 = np.searchsorted(grid, [s + 0.5e9, c - 3e9])
    if i1 <= i0:
        return None, -np.inf
    seg = np.nan_to_num(sign * zk[i0:i1], nan=-np.inf)
    j = int(np.argmax(seg))
    return grid[i0 + j], seg[j]


def main():
    data = {r: load(r) for r in RIDES}
    names = list(next(iter(data.values()))[1])
    offs = np.arange(-10, 11)
    print(f"{sum(len(d[2]) for d in data.values())} tapped openings in {len(RIDES)} rides\n")
    print("median step z about the tap, each second from -10 to +10 s")
    print(f"{'':16s}" + "".join(f"{o:6d}" for o in offs))
    for k in names:
        rows = []
        for grid, z, cases in data.values():
            for _, _, a in cases:
                rows.append(np.interp(a + offs * 1e9, grid, np.nan_to_num(z[k])))
        print(f"{k:16s}" + "".join(f"{v:+6.1f}" for v in np.median(rows, 0)))

    def score(preds, truth):
        e = np.abs(np.array(preds) - np.array(truth)) / 1e9
        return int((e <= 1).sum()), int((e <= 2).sum()), np.median(e)

    base = [(s + BASELINE_S * 1e9, a) for d in data.values() for s, _, a in d[2]]
    h1, h2, me = score(*zip(*base))
    print(f"\nbaseline stop + {BASELINE_S} s: {h1} within 1 s, {h2} within 2 s, median error {me:.1f} s")
    print("\nleave-one-ride-out detector, all held-out: within 1 s, within 2 s, median error, fallbacks")
    for k in names:
        preds, truth, fb = [], [], 0
        for held in RIDES:
            train = [r for r in RIDES if r != held]
            # Sign: whichever way the step at the tap leans in training.
            at = [np.interp(a, data[r][0], np.nan_to_num(data[r][1][k])) for r in train for *_, a in data[r][2]]
            sign = 1.0 if np.median(at) >= 0 else -1.0
            tr = [(pick(data[r][0], data[r][1][k], s, c, sign), a, s) for r in train for s, c, a in data[r][2]]
            lags = [a - p for (p, _), a, _ in tr if p is not None]
            lag = np.median(lags) if lags else 0.0
            best = (-1, np.inf)
            for bar in [-np.inf, 2, 3, 4, 5, 6, 8, 10, 15]:
                ok = sum(abs((p + lag if p is not None and v >= bar else s + BASELINE_S * 1e9) - a) <= 1e9 for (p, v), a, s in tr)
                if ok > best[0]:
                    best = (ok, bar)
            bar = best[1]
            grid, z, cases = data[held]
            for s, c, a in cases:
                p, v = pick(grid, z[k], s, c, sign)
                if p is None or v < bar:
                    p, fb = s + BASELINE_S * 1e9 - lag, fb + 1
                preds.append(p + lag)
                truth.append(a)
        h1, h2, me = score(preds, truth)
        print(f"  {k:16s} {h1:3d} {h2:3d} {me:5.1f} s {fb:3d}")


if __name__ == "__main__":
    main()

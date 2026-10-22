"""Traction sound against GNSS on rides with it (the elevated 7, R188): per 0.5 s frame, the whitened log spectrum
(each frequency's median over the ride removed), our own Doppler speed, and its acceleration (2 s slope).

1. Pitch following speed: for each ratio k (Hz per m/s), the mean whitened level at f = k * v, over frames at 3 m/s and up,
   separately while motoring (over +0.3 m/s^2), braking (under -0.3) and coasting. A peak in k is a line whose pitch
   follows speed (motor frequency), a GNSS-free speed.
2. Motoring or braking from the sound: logistic regression on 40 log-spaced band levels, 5 time-block cross-validation,
   AUC for motoring vs not and braking vs not.

On `154951` and `184134` (2026-10-22, 7227 frames with GNSS): above k 20, the most prominent line is k 32.5 Hz per m/s
with harmonics near 65 and 98, strongest braking (+0.28 over neighboring k), then motoring (+0.20), weak coasting (+0.10):
a pitch locked to speed (gear mesh, or a harmonic of the motor's electrical frequency). Below k 20 it's broadband rumble.
It's too weak to read speed from: picking the best speed per frame is off by 7 to 13 m/s (median); searching within
4 m/s of a prior 3 m/s off ends 1.0 to 4.5 m/s off, not reliably nearer. Band levels tell motoring from the rest at
AUC 0.57 and braking at 0.62: not usable either. Next: longer FFTs (finer bins at low k) and other car types.

Usage: `vvvf_gnss.py [ride ...]`; needs `audio.wav`, `audio_ts.csv`, `own.csv` in `target/analysis/<ride>`."""

import csv
import sys
import wave

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS  # noqa: E402

N = 4096
HOP_S = 0.5


def frames(ride):
    with open(ANALYSIS / ride / "audio_ts.csv") as f:
        ats = np.array([[int(r["frame"]), int(r["t_ns"])] for r in csv.DictReader(f)], float)
    with wave.open(str(ANALYSIS / ride / "audio.wav")) as w:
        rate = w.getframerate()
        pcm = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(float)
    slope, icept = np.polyfit(ats[:, 1], ats[:, 0], 1)
    with open(ANALYSIS / ride / "own.csv") as f:
        own = np.array([[int(r["t_ns"]), float(r["speed"]), float(r["sigma"])] for r in csv.DictReader(f)], float)
    own = own[own[:, 2] < 0.5]
    win = np.hanning(N)
    ts, specs = [], []
    for t in np.arange(own[0, 0], own[-1, 0], HOP_S * 1e9):
        a = int(slope * t + icept) - N // 2
        if a < 0 or a + N > len(pcm):
            continue
        ts.append(t)
        specs.append(np.log10(np.abs(np.fft.rfft(pcm[a : a + N] * win)) ** 2 + 1e-9))
    ts = np.array(ts)
    S = np.array(specs)
    S -= np.median(S, axis=0)
    # Only frames with GNSS within 1.5 s.
    near = np.abs(own[np.clip(np.searchsorted(own[:, 0], ts), 0, len(own) - 1), 0] - ts) < 1.5e9
    v = np.interp(ts, own[:, 0], own[:, 1])
    acc = (np.interp(ts + 1e9, own[:, 0], own[:, 1]) - np.interp(ts - 1e9, own[:, 0], own[:, 1])) / 2
    return np.fft.rfftfreq(N, 1 / rate), S[near], v[near], acc[near]


def auc(score, label):
    order = np.argsort(score)
    r = np.empty(len(score))
    r[order] = np.arange(1, len(score) + 1)
    p = label.sum()
    n = len(label) - p
    return (r[label].sum() - p * (p + 1) / 2) / (p * n) if p and n else np.nan


def logistic(X, y, iters=300, lam=1.0):
    w = np.zeros(X.shape[1])
    for _ in range(iters):
        p = 1 / (1 + np.exp(-X @ w))
        g = X.T @ (p - y) + lam * w
        H = (X * (p * (1 - p))[:, None]).T @ X + lam * np.eye(X.shape[1])
        w -= np.linalg.solve(H, g)
    return w


def main(rides):
    fs, Ss, vs, accs = [], [], [], []
    for ride in rides:
        f, S, v, a = frames(ride)
        fs, Ss, vs, accs = f, Ss + [S], vs + [v], accs + [a]
        print(f"{ride}: {len(v)} frames with GNSS")
    S, v, acc = np.vstack(Ss), np.concatenate(vs), np.concatenate(accs)
    f = fs
    moving = v >= 3
    print("\n1. level at f = k * v (whitened, log10), by k; top ks per phase")
    ks = np.arange(5, 400, 1.0)
    for name, m in (("motoring", moving & (acc > 0.3)), ("braking", moving & (acc < -0.3)), ("coasting", moving & (np.abs(acc) < 0.1))):
        idx = np.searchsorted(f, ks[:, None] * v[m][None, :])
        ok = idx < len(f)
        lv = np.where(ok, S[m][np.arange(m.sum())[None, :], np.clip(idx, 0, len(f) - 1)], np.nan)
        mean = np.nanmean(lv, axis=1)
        top = np.argsort(mean)[::-1][:4]
        print(f"  {name:9s} n {m.sum():4d}: " + ", ".join(f"k {ks[i]:.0f} ({mean[i]:+.2f})" for i in top)
              + f"; median over k {np.nanmedian(mean):+.2f}")
    print("\n2. motoring / braking from 40 band levels, 5-block cross-validated AUC")
    edges = np.geomspace(80, f[-1] * 0.95, 41)
    bands = np.array([S[:, (f >= a) & (f < b)].mean(axis=1) for a, b in zip(edges[:-1], edges[1:])]).T
    X = np.hstack([bands, np.ones((len(bands), 1))])
    blocks = np.array_split(np.arange(len(X)), 5)
    for name, y in (("motoring", acc > 0.3), ("braking", acc < -0.3)):
        score = np.empty(len(X))
        for b in blocks:
            tr = np.setdiff1d(np.arange(len(X)), b)
            w = logistic(X[tr], y[tr].astype(float))
            score[b] = X[b] @ w
        print(f"  {name}: AUC {auc(score, y):.2f} ({y.sum()} of {len(y)} frames)")


if __name__ == "__main__":
    main(sys.argv[1:] or ["154951", "184134"])

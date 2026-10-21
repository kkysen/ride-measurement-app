"""Train starts and stops in the IMU and magnetometer, against the rider's precise "train stopped/started" taps
(2026-10-21, `20261021-135509`, 110 St (C) to Lexington Av/53 St (E)), which only score.

First, the signals lined up on the taps (median and quartiles across taps, 15 s each side at 10 Hz):
horizontal acceleration (gravity removed), its jerk, and the magnetic field's deviation from its local level.
Then simple detectors, and how far their times fall from the taps."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ride  # noqa: E402
from subway import ANALYSIS, rows  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
NAME = "135509"
SESSION = ROOT / "data/sessions/20261021-135509"
RATE = 10.0
# Taps known to be wrong: the "train stopped" 0.9 s before a "train started" at 14:17:29.
MISTAPS_NS = {"TRAIN_STOPPED": [1087.5]}  # seconds from the first tap


def taps():
    ev = rows(NAME, "events.csv")
    t_first = int(ev[0]["t_ns"])
    out = {"TRAIN_STOPPED": [], "TRAIN_STARTED": []}
    for e in ev:
        k = e["kind"].removeprefix("KIND_")
        if k in out:
            rel = round((int(e["t_ns"]) - t_first) * 1e-9, 1)
            if rel not in MISTAPS_NS.get(k, []):
                out[k].append(int(e["t_ns"]))
    return out


def resample(stream, t):
    s = ride.read_stream(SESSION, stream)
    v = s.values[:, :3].astype(float) - s.values[:, 3:6]
    return np.stack([np.interp(t, s.timestamps_ns, v[:, i]) for i in range(3)], axis=1)


def signals():
    a = ride.read_stream(SESSION, "accel_uncal")
    t = np.arange(a.timestamps_ns[0], a.timestamps_ns[-1], int(1e9 / RATE))
    # Average the 480 Hz samples into each 0.1 s bin rather than sample them, so vibration doesn't alias.
    av = a.values[:, :3].astype(float) - a.values[:, 3:6]
    i = np.searchsorted(t, a.timestamps_ns)
    acc = np.stack([np.bincount(i, av[:, k], len(t) + 1)[: len(t)] for k in range(3)], 1)
    acc /= np.maximum(np.bincount(i, None, len(t) + 1)[: len(t)], 1)[:, None]
    # Gravity: a 20 s moving average (the train's own accelerations last seconds; the phone is held fairly still).
    k = int(20 * RATE)
    g = np.stack([np.convolve(acc[:, j], np.ones(k) / k, mode="same") for j in range(3)], 1)
    up = g / np.linalg.norm(g, axis=1, keepdims=True)
    lin = acc - g
    horiz = lin - (lin * up).sum(1, keepdims=True) * up
    # Smoothed over 1 s: the train's surge, not its vibration.
    k1 = int(RATE)
    hs = np.stack([np.convolve(horiz[:, j], np.ones(k1) / k1, mode="same") for j in range(3)], 1)
    h = np.linalg.norm(hs, axis=1)
    jerk = np.linalg.norm(np.gradient(hs, axis=0), axis=1) * RATE
    b = np.linalg.norm(resample("mag_uncal", t), axis=1)
    kb = int(30 * RATE)
    b_dev = np.abs(b - np.convolve(b, np.ones(kb) / kb, mode="same"))
    return t, {"horiz_accel": h, "jerk": jerk, "mag_dev": b_dev}


def aligned(t, x, at, span=15.0):
    offs = np.arange(-span, span + 1e-9, 1 / RATE)
    return offs, np.array([np.interp(a + offs * 1e9, t, x) for a in at])


def main():
    t, sig = signals()
    tp = taps()
    for kind, at in tp.items():
        print(f"\n{kind}: {len(at)} taps; median [quartiles] across taps, every 1 s from the tap")
        for name, x in sig.items():
            offs, m = aligned(t, x, at)
            q = np.percentile(m, [25, 50, 75], axis=0)
            line = " ".join(f"{q[1][j]:5.2f}" for j in range(0, len(offs), int(RATE)))
            print(f"  {name:12s} {line}")
        print(f"  {'s from tap':12s} " + " ".join(f"{o:5.0f}" for o in offs[:: int(RATE)]))



def detect(t, h, quiet=0.35, moving=0.6, hold_s=4.0, move_s=1.5):
    """A two-state machine with hysteresis: moving once h stays over `moving` for `move_s`,
    stopped once it stays under `quiet` for `hold_s`. Each switch is dated to the start of the run that made it.
    Times in ns."""
    starts, stops = [], []
    is_moving = False
    run_start, run = None, 0
    need = int((hold_s if is_moving else move_s) * RATE)
    for i in range(len(h)):
        cond = h[i] < quiet if is_moving else h[i] > moving
        if cond:
            if run == 0:
                run_start = i
            run += 1
        else:
            run = 0
        need = int((hold_s if is_moving else move_s) * RATE)
        if run >= need:
            (stops if is_moving else starts).append(t[run_start])
            is_moving = not is_moving
            run = 0
    return np.array(starts), np.array(stops)


def offsets(found, at, reach_s=15.0):
    """Each tap's nearest detection, in s (positive: the detection is after the tap); NaN if none within reach."""
    out = []
    for a in at:
        if not len(found):
            out.append(np.nan)
            continue
        d = (found - a) * 1e-9
        j = np.argmin(np.abs(d))
        out.append(d[j] if abs(d[j]) <= reach_s else np.nan)
    return np.array(out)


def fused_transitions():
    """The replayed speed estimator's forward pass: moving to stopped and back, at 1 m/s."""
    r = rows(NAME, "fused.csv")
    t = np.array([int(x["t_ns"]) for x in r])
    v = np.array([float(x["forward"]) for x in r])
    moving = v > 1.0
    ch = np.nonzero(moving[1:] != moving[:-1])[0] + 1
    return t[ch][moving[ch]], t[ch][~moving[ch]]


def score():
    t, sig = signals()
    tp = taps()
    starts, stops = detect(t, sig["horiz_accel"])
    fs, fp = fused_transitions()
    for label, found, at in [
        ("IMU starts", starts, tp["TRAIN_STARTED"]),
        ("IMU stops", stops, tp["TRAIN_STOPPED"]),
        ("speed estimator starts", fs, tp["TRAIN_STARTED"]),
        ("speed estimator stops", fp, tp["TRAIN_STOPPED"]),
    ]:
        o = offsets(found, at)
        ok = o[~np.isnan(o)]
        print(f"{label:24s}: {len(ok)} of {len(at)} taps matched within 15 s; offsets (s) {np.round(o, 1).tolist()}"
              + (f"; median {np.median(ok):+.1f} s, spread (IQR) {np.subtract(*np.percentile(ok, [75, 25])):.1f} s" if len(ok) else ""))
    print(f"(IMU found {len(starts)} starts and {len(stops)} stops over the whole ride)")


if __name__ == "__main__":
    score() if sys.argv[1:] == ["score"] else main()

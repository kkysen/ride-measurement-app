"""The rider's own OMNY tap: a loud beep (`chime_omny.py`, 2.8 to 3.05 kHz, 0.15 to 0.5 s), since their phone is
right at the reader. (First tried: stairs near the beep, since the fare gates sit between the street and the platform;
but not every station has stairs to its gates, so loudness is the rule.) Stairs: a height change of at least 2.5 m (barometer, about
8.3 m per hPa) over 20 s with steps in the accelerometer (|a| std over 2 m/s^2). A beep counts when stairs come within
`NEAR_S` before or after it. Scored on the known taps (Google Wallet, `chime_omny.OMNY_TAPS`), which only score.

On seven phone-mic rides: 38 beeps, 21 with stairs within 45 s (18 within 30 s), keeping all 4 known taps.
Loudness does better, as the rider suggested (their phone is right at the reader): 4 known taps at 38 to 47 dB,
every other beep on those rides at 33 dB or less.
The rest of the paired beeps may be the rider's own untracked taps (only four tap times are known) or others' at
the same gates; more known tap times would tell."""

import datetime
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import chime_omny as C  # noqa: E402
import ride  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
SESSIONS = {"184134": "20261020-184134", "214427": "20261020-214427", "135509": "20261021-135509",
            "154951": "20261020-154951", "155919": "20261021-155919", "182135": "20261021-182135",
            "215922": "20261021-215922"}
NEAR_S = 45.0
# The rider's own phone is centimeters from the reader: their known taps peaked at 38 to 47 dB in the beep's band,
# every other beep on those rides at 33 dB or less.
LOUD_DB = 36.0


def loudness(name, times):
    """Each beep's peak level (dB, Welch over 0.3 s) in 2.8 to 3.05 kHz."""
    from scipy import signal

    sr, pcm, ts = C.load(name)
    out = []
    for x in times:
        f0 = int(np.interp(x, ts[:, 1], ts[:, 0]))
        f, p = signal.welch(pcm[f0 : f0 + int(0.3 * sr)], sr, nperseg=1024)
        out.append(10 * np.log10(p[(f >= 2800) & (f <= 3050)].max() + 1e-9))
    return np.array(out)


def beeps(name):
    """`chime_omny.omny`'s detection, as times (ns)."""
    sr, pcm, ts = C.load(name)
    f, t, prom = C.prominence(sr, pcm, 2700, 3150)
    tn = C.frame_times_ns(t, sr, ts)
    band = (f >= 2800) & (f <= 3050)
    strong = (prom[band] > 20).any(0)
    tone = strong.copy()
    on = np.nonzero(strong)[0]
    for a, b in zip(on[:-1], on[1:]):
        if 1 < b - a <= 3:
            tone[a:b] = True
    return np.array(C.runs(tone, tn, 0.15, 0.5))


def stairs(session):
    """Start times (ns) of 20 s windows (every 5 s) with a 2.5 m height change and steps."""
    p = ride.read_stream(session, "pressure")
    t, v = p.timestamps_ns, p.values[:, 0].astype(float)
    a = ride.read_stream(session, "accel_uncal")
    at = a.timestamps_ns
    x = np.linalg.norm(a.values[:, :3].astype(float) - a.values[:, 3:6], axis=1)
    out = []
    for s in np.arange(t[0], t[-1] - 20e9, 5e9):
        m0 = (t >= s) & (t < s + 2e9)
        m1 = (t >= s + 18e9) & (t < s + 20e9)
        if not m0.any() or not m1.any():
            continue
        dz = -8.3 * (np.median(v[m1]) - np.median(v[m0]))
        ma = (at >= s) & (at < s + 20e9)
        if abs(dz) >= 2.5 and ma.any() and x[ma].std() > 2.0:
            out.append(s)
    return np.array(out)


def known(name):
    """Known tap times (ns), from `chime_omny.OMNY_TAPS`' local clock times."""
    import csv

    with open(C.ANALYSIS / name / "fixes.csv") as f:
        r = next(csv.DictReader(f))
    off = int(r["time_ms"]) * 1_000_000 - int(r["t_ns"])
    day = datetime.datetime.fromtimestamp(int(r["time_ms"]) / 1000).date()
    out = []
    for s in C.OMNY_TAPS.get(name, []):
        hms = s.split(" ")[0]
        fmt = "%H:%M:%S.%f" if "." in hms else "%H:%M:%S"
        tt = datetime.datetime.combine(day, datetime.datetime.strptime(hms, fmt).time())
        out.append(int(tt.timestamp() * 1e9) - off)
    return np.array(out)


def main():
    tot = np.zeros(6, int)
    for name, sess in SESSIONS.items():
        try:
            b = beeps(name)
        except FileNotFoundError:
            continue
        st = stairs(ROOT / "data/sessions" / sess)
        paired = np.array([x for x in b if len(st) and np.min(np.abs(st + 10e9 - x)) < NEAR_S * 1e9])
        loud = b[loudness(name, b) >= LOUD_DB] if len(b) else b
        k = known(name)
        hit_all = sum(1 for x in k if len(b) and np.min(np.abs(b - x)) < 2e9)
        hit_paired = sum(1 for x in k if len(paired) and np.min(np.abs(paired - x)) < 2e9)
        hit_loud = sum(1 for x in k if len(loud) and np.min(np.abs(loud - x)) < 2e9)
        tot += [len(b), len(paired), hit_all, hit_paired, len(loud), hit_loud]
        print(f"{name}: {len(b)} beeps, {len(paired)} with stairs within {NEAR_S:.0f} s, {len(loud)} loud; "
              f"known taps {len(k)}: found {hit_all} among beeps, {hit_paired} paired, {hit_loud} loud")
    print(f"all: {tot[0]} beeps, {tot[1]} paired, {tot[4]} loud; known taps found {tot[2]}, {tot[3]}, {tot[5]}")


if __name__ == "__main__":
    main()

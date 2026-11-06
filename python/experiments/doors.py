"""Doors opening and closing in the sound: the HVAC and platform noise get louder while the doors are open.
Band levels (dB) at 10 Hz, lined up on the rider's "doors open" and "doors closed" taps (which only score),
median across taps and rides, on the rides recorded with the phone's own microphone."""

import csv
import sys
import wave
from pathlib import Path

import numpy as np
from scipy import signal

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from subway import ANALYSIS  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
RIDES = ["154951", "184134", "214427", "135509"]  # 151449 used a headphone mic
RATE = 10.0
BANDS = {"low 50-200 Hz": (50, 200), "mid 200-1000 Hz": (200, 1000), "high 1-4 kHz": (1000, 4000), "top 4-8 kHz": (4000, 8000)}


def levels(name):
    ts = np.loadtxt(ANALYSIS / name / "audio_ts.csv", delimiter=",", skiprows=1, dtype=np.int64)
    with wave.open(str(ROOT / "data/work/audio" / f"{name}.wav")) as w:
        sr = w.getframerate()
        pcm = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float32)
    hop = int(sr / RATE)
    f, t, z = signal.stft(pcm, sr, nperseg=2 * hop, noverlap=hop, boundary=None, padded=False)
    p = np.abs(z) ** 2
    out = {k: 10 * np.log10(p[(f >= a) & (f < b)].sum(0) + 1e-9) for k, (a, b) in BANDS.items()}
    return np.interp(t * sr, ts[:, 0], ts[:, 1]), out


def taps(name, kind):
    with open(ANALYSIS / name / "events.csv") as f:
        return [int(r["t_ns"]) for r in csv.DictReader(f) if r["kind"] == f"KIND_{kind}"]


def main():
    offs = np.arange(-10, 10.01, 1 / RATE)
    for kind in ["DOORS_OPEN", "DOORS_CLOSED"]:
        rows = {k: [] for k in BANDS}
        n = 0
        for name in RIDES:
            t, lv = levels(name)
            for a in taps(name, kind):
                n += 1
                for k, x in lv.items():
                    seg = np.interp(a + offs * 1e9, t, x)
                    # Relative to the level 5 to 10 s before the tap.
                    rows[k].append(seg - np.median(seg[offs < -5]))
        print(f"\n{kind}: {n} taps; median dB change from 5-10 s before, every 1 s from the tap")
        for k, r in rows.items():
            m = np.median(np.array(r), axis=0)
            print(f"  {k:16s} " + " ".join(f"{m[j]:+5.1f}" for j in range(0, len(offs), int(RATE))))
        print(f"  {'s from tap':16s} " + " ".join(f"{o:5.0f}" for o in offs[:: int(RATE)]))


def steps(t, x, up_db, down_db, span_s=2.0):
    """Times the level over the next `span_s` differs from the last `span_s` by over `up_db` (up) or `down_db`
    (down, positive), at each step's sharpest point, at most one per 5 s each way."""
    n = int(span_s * RATE)
    c = np.concatenate([[0], np.cumsum(x)])
    d = np.full(len(x), np.nan)
    i = np.arange(n, len(x) - n)
    d[i] = (c[i + n] - c[i]) / n - (c[i] - c[i - n]) / n
    out = {"up": [], "down": []}
    for way, sign, thr in [("up", 1, up_db), ("down", -1, down_db)]:
        cand = np.nonzero(sign * np.nan_to_num(d) > thr)[0]
        last = -np.inf
        for j in cand:
            # The sharpest point of this run.
            if t[j] - last < 5e9:
                continue
            k = j
            while k + 1 < len(d) and sign * d[k + 1] > thr:
                k += 1
            best = j + int(np.argmax(sign * d[j : k + 1]))
            out[way].append(t[best])
            last = t[best]
    return np.array(out["up"]), np.array(out["down"])


def score():
    band = "high 1-4 kHz"
    for up_db, down_db in [(3, 2), (4, 2.5), (5, 3)]:
        tot = {"open": [0, 0], "closed": [0, 0]}
        false_n, hours, offs_o, offs_c = 0, 0.0, [], []
        for name in RIDES:
            t, lv = levels(name)
            ups, downs = steps(t, lv[band], up_db, down_db)
            hours += (t[-1] - t[0]) * 1e-9 / 3600
            opens, closes = taps(name, "DOORS_OPEN"), taps(name, "DOORS_CLOSED")
            for found, at, key, offs in [(ups, opens, "open", offs_o), (downs, closes, "closed", offs_c)]:
                for a in at:
                    tot[key][1] += 1
                    if len(found):
                        d = (found - a) * 1e-9
                        j = np.argmin(np.abs(d))
                        if abs(d[j]) <= 3:
                            tot[key][0] += 1
                            offs.append(d[j])
            door = np.array(opens + closes)
            false_n += sum(1 for f in np.concatenate([ups, downs]) if not len(door) or np.min(np.abs(door - f)) > 30e9)
        print(f"up > {up_db} dB, down > {down_db} dB: opens {tot['open'][0]}/{tot['open'][1]} (median {np.median(offs_o):+.1f} s), "
              f"closes {tot['closed'][0]}/{tot['closed'][1]} (median {np.median(offs_c):+.1f} s); "
              f"{false_n} steps over 30 s from any door tap ({false_n / hours:.0f}/h)")



CHIME_HZ = [(436, 456), (545, 565), (512, 526), (630, 650)]  # the 441/555 and 516/645 Hz families


def chime_times(name, prominent_db=15.0, min_s=0.25):
    """Starts of tones at the known chime pitches lasting `min_s`, from `chime_omny`'s spectrogram."""
    import chime_omny as C

    sr, pcm, ts = C.load(name)
    f, t, prom = C.prominence(sr, pcm, 400, 700)
    tn = C.frame_times_ns(t, sr, ts)
    sel = np.zeros(len(f), bool)
    for a, b in CHIME_HZ:
        sel |= (f >= a) & (f <= b)
    tone = (prom[sel] > prominent_db).any(0)
    return np.array(C.runs(tone, tn, min_s))


def sequences(up_db=3.0, down_db=2.0):
    """Door cycles: a step up, then a step down 8 to 90 s later with a chime in the 8 s before it."""
    band = "high 1-4 kHz"
    got = {"open": [0, 0], "closed": [0, 0]}
    false_n, hours = 0, 0.0
    for name in RIDES:
        t, lv = levels(name)
        ups, downs = steps(t, lv[band], up_db, down_db)
        ch = chime_times(name)
        hours += (t[-1] - t[0]) * 1e-9 / 3600
        cycles = []
        for u in ups:
            later = downs[(downs > u + 8e9) & (downs < u + 90e9)]
            for d in later:
                if ((ch > d - 8e9) & (ch < d)).any():
                    cycles.append((u, d))
                    break
        opens, closes = np.array(taps(name, "DOORS_OPEN")), np.array(taps(name, "DOORS_CLOSED"))
        for key, at, k in [("open", opens, 0), ("closed", closes, 1)]:
            for a in at:
                got[key][1] += 1
                if any(abs(c[k] - a) <= 3e9 for c in cycles):
                    got[key][0] += 1
        door = np.concatenate([opens, closes])
        false_n += sum(1 for c in cycles if not len(door) or min(np.min(np.abs(door - c[0])), np.min(np.abs(door - c[1]))) > 30e9)
        print(f"  {name}: {len(cycles)} door cycles found; {len(opens)} open and {len(closes)} closed taps")
    print(f"door cycles (up > {up_db} dB, down > {down_db} dB, chime before the close): "
          f"opens {got['open'][0]}/{got['open'][1]}, closes {got['closed'][0]}/{got['closed'][1]}; "
          f"{false_n} cycles over 30 s from any door tap ({false_n / hours:.1f}/h)")


if __name__ == "__main__":
    {"score": score, "cycles": sequences}.get(sys.argv[1] if sys.argv[1:] else "", main)()

"""The doors' own sound, not just the platform noise they let in (`doors.py`) or the chime before closing:
sudden broadband onsets (positive spectral flux, 300 Hz to 8 kHz, at 20 Hz, scaled by its median and spread),
the strongest in the 4 s before to 1 s after each "doors open" and "doors closed" tap, against random times.

On five phone-mic rides: openings have an onset median 11.0 against 4.5 at random, 0.4 to 1.3 s before the tap
(the rider's reaction), but only 19% stand above random's 95th percentile; closings 6.3 against 4.3,
0.2 to 0.7 s before, 12% above. So the doors are heard, consistently timed, but too faintly to find alone:
they can time a door cycle found from the level steps and the chime."""

import csv
import sys
import wave
from pathlib import Path

import numpy as np
from scipy import signal

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
RIDES = ["135509", "182135", "215922", "184134", "214427"]
RATE = 20.0


def flux(name):
    ts = np.loadtxt(ANALYSIS / name / "audio_ts.csv", delimiter=",", skiprows=1, dtype=np.int64)
    with wave.open(str(ROOT / "data/work/audio" / f"{name}.wav")) as w:
        sr = w.getframerate()
        pcm = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float32)
    hop = int(sr / RATE)
    f, t, z = signal.stft(pcm, sr, nperseg=2 * hop, noverlap=hop, boundary=None, padded=False)
    lp = np.log10(np.abs(z) ** 2 + 1e-6)
    fl = np.maximum(np.diff(lp[f > 300], axis=1), 0).mean(0)
    z = (fl - np.median(fl)) / (np.percentile(fl, 75) - np.percentile(fl, 25))
    return np.interp(t[1:] * sr, ts[:, 0], ts[:, 1]), z


def main():
    offs = np.arange(-6, 6.01, 1 / RATE)
    win = (offs > -4) & (offs < 1)
    rng = np.random.default_rng(0)
    for kind in ["DOORS_OPEN", "DOORS_CLOSED"]:
        peaks, at, rnd = [], [], []
        for name in RIDES:
            tn, z = flux(name)
            with open(ANALYSIS / name / "events.csv") as f:
                taps = [int(r["t_ns"]) for r in csv.DictReader(f) if r["kind"] == f"KIND_{kind}"]
            for a in taps:
                seg = np.interp(a + offs * 1e9, tn, z)[win]
                peaks.append(seg.max())
                at.append(offs[win][np.argmax(seg)])
            for a in rng.uniform(tn[0], tn[-1], 5 * len(taps)):
                rnd.append(np.interp(a + offs * 1e9, tn, z)[win].max())
        thr = np.percentile(rnd, 95)
        print(f"{kind}: {len(peaks)} taps; strongest onset median {np.median(peaks):.1f}, random {np.median(rnd):.1f}; "
              f"{np.mean(np.array(peaks) > thr):.0%} over random's 95th percentile; "
              f"at {np.median(at):+.1f} s from the tap (quartiles {np.percentile(at, 25):+.1f}, {np.percentile(at, 75):+.1f})")


if __name__ == "__main__":
    main()

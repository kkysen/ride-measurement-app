"""The doors' own sound as a template: the spectrogram (64 bands, 100 Hz to 8 kHz, at 50 Hz) around each
"doors open" or "doors closed" tap, aligned on its strongest onset in the 2 s before the tap (the rider's
reaction), averaged per kind across rides, then matched (correlation of the band-normalized spectrogram)
over each ride. Scored leave-one-ride-out: the template from the other rides finds this ride's door taps
(a match within 2 s before) against matches elsewhere, at a threshold set for 10 false matches an hour.
`within`: each ride's own template, leaving out the tap being found.

Across rides, 25 of 55 openings and 17 of 52 closings, almost none on the 1/2/3 (`182135`, `215922`).
With each ride's own template, 40 of 55 openings and 24 of 52 closings: each car type has its own door sound,
as it has its own chime pitches. Closings come out worse partly from the taps: the doors sometimes close,
reopen, and close again, which the rider may tap as two closings."""

import csv
import sys
import wave
from pathlib import Path

import numpy as np
from scipy import signal

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
RIDES = ["135509", "182135", "215922", "184134", "214427", "154951", "155919"]
RATE = 50.0
SPAN = (-0.3, 1.2)  # s around the onset the template covers


def spectrogram(name):
    ts = np.loadtxt(ANALYSIS / name / "audio_ts.csv", delimiter=",", skiprows=1, dtype=np.int64)
    with wave.open(str(ROOT / "target/audio" / f"{name}.wav")) as w:
        sr = w.getframerate()
        pcm = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float32)
    hop = int(sr / RATE)
    f, t, z = signal.stft(pcm, sr, nperseg=4 * hop, noverlap=3 * hop, boundary=None, padded=False)
    p = np.abs(z) ** 2
    edges = np.geomspace(100, 8000, 65)
    bands = np.stack([p[(f >= a) & (f < b)].sum(0) for a, b in zip(edges, edges[1:])])
    db = 10 * np.log10(bands + 1e-6)
    # Each band relative to its level over the last 10 s, so the train's steady noise drops out.
    k = int(10 * RATE)
    c = np.cumsum(np.pad(db, ((0, 0), (k, 0)), mode="edge"), axis=1)
    db = db - (c[:, k:] - c[:, :-k]) / k
    tn = np.interp(t * sr, ts[:, 0], ts[:, 1]).astype(np.int64)
    return tn, db


def taps(name, kind):
    with open(ANALYSIS / name / "events.csv") as f:
        return [int(r["t_ns"]) for r in csv.DictReader(f) if r["kind"] == f"KIND_{kind}"]


def onset(tn, db, a):
    """Index of the strongest broadband rise in the 2 s before `a`."""
    i0, i1 = np.searchsorted(tn, [a - 2e9, a])
    flux = np.maximum(np.diff(db[:, i0 : i1 + 1], axis=1), 0).sum(0)
    return i0 + 1 + int(np.argmax(flux)) if len(flux) else None


def match(db, tpl):
    """Correlation of each window with the template (both centered and scaled)."""
    n = tpl.shape[1]
    t = (tpl - tpl.mean()) / tpl.std()
    out = np.full(db.shape[1], -1.0)
    for i in range(db.shape[1] - n):
        w = db[:, i : i + n]
        s = w.std()
        if s > 0:
            out[i] = ((w - w.mean()) / s * t).mean()
    return out


def main():
    a0, a1 = int(SPAN[0] * RATE), int(SPAN[1] * RATE)
    data = {r: spectrogram(r) for r in RIDES}
    for kind in ["DOORS_OPEN", "DOORS_CLOSED"]:
        cuts = {r: [] for r in RIDES}
        for r, (tn, db) in data.items():
            for a in taps(r, kind):
                i = onset(tn, db, a)
                if i is not None and i + a0 >= 0 and i + a1 <= db.shape[1]:
                    cuts[r].append(db[:, i + a0 : i + a1])
        hits = total = false = 0
        hours = 0.0
        for r, (tn, db) in data.items():
            others = [c for o in RIDES if o != r for c in cuts[o]]
            if not others or not taps(r, kind):
                continue
            tpl = np.mean(others, axis=0)
            m = match(db, tpl)
            # Peaks at least 3 s apart.
            peaks, _ = signal.find_peaks(m, distance=int(3 * RATE))
            pt, pv = tn[peaks] - int(a0 / RATE * 1e9), m[peaks]
            door = np.array(taps(r, "DOORS_OPEN") + taps(r, "DOORS_CLOSED"))
            away = np.array([np.min(np.abs(door - t)) > 30e9 for t in pt])
            h = (tn[-1] - tn[0]) * 1e-9 / 3600
            thr = np.sort(pv[away])[::-1][min(int(10 * h), away.sum() - 1)] if away.any() else 0
            found = sum(1 for a in taps(r, kind) if ((pt > a - 2.5e9) & (pt < a + 0.5e9) & (pv > thr)).any())
            hits += found
            total += len(taps(r, kind))
            false += int((pv[away] > thr).sum())
            hours += h
            print(f"  {kind} {r}: {found} of {len(taps(r, kind))} at {(pv[away] > thr).sum() / h:.0f} false an hour")
        print(f"{kind}: {hits} of {total} found, {false / hours:.0f} false an hour (left-out ride each time)")


def within():
    """The same with each ride's own template, leaving out the tap being found: is one car type's door consistent?"""
    a0, a1 = int(SPAN[0] * RATE), int(SPAN[1] * RATE)
    for kind in ["DOORS_OPEN", "DOORS_CLOSED"]:
        hits = total = 0
        for r in RIDES:
            tn, db = spectrogram(r)
            at = taps(r, kind)
            cuts = []
            for a in at:
                i = onset(tn, db, a)
                ok = i is not None and i + a0 >= 0 and i + a1 <= db.shape[1]
                cuts.append(db[:, i + a0 : i + a1] if ok else None)
            h = (tn[-1] - tn[0]) * 1e-9 / 3600
            door = np.array(taps(r, "DOORS_OPEN") + taps(r, "DOORS_CLOSED"))
            found = 0
            for k, a in enumerate(at):
                rest = [c for j, c in enumerate(cuts) if j != k and c is not None]
                if len(rest) < 2:
                    continue
                m = match(db, np.mean(rest, axis=0))
                peaks, _ = signal.find_peaks(m, distance=int(3 * RATE))
                pt, pv = tn[peaks] - int(a0 / RATE * 1e9), m[peaks]
                away = np.array([np.min(np.abs(door - t)) > 30e9 for t in pt])
                thr = np.sort(pv[away])[::-1][min(int(10 * h), away.sum() - 1)]
                found += bool(((pt > a - 2.5e9) & (pt < a + 0.5e9) & (pv > thr)).any())
            hits += found
            total += len(at)
            print(f"  {kind} {r}: {found} of {len(at)} with its own ride's template, at 10 false an hour")
        print(f"{kind}: {hits} of {total}")


if __name__ == "__main__":
    {"within": within}.get(sys.argv[1] if sys.argv[1:] else "", main)()

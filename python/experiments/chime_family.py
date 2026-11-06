"""Which chime family each door chime is, scored against the line the rider's nearest "at station" tag names
(R62 family on the 1 and 3; NTT on the 2, 7, E, F, M, R; other lines not scored).

The Rust detector (`crates/fusion/src/doors.rs`) took the family of the first prominent bin, lowest pitch first,
in the tone's last window: a tone with any prominent bin in 512 to 526 Hz while its 441 Hz faded came out R62.
Here every chime-like tone (any of the four bands 15 dB over its +-60 Hz neighborhood for 0.25 s) is scored
by summing each family's best prominence over the tone's windows, and by the share of windows each family leads.

Needs `data/work/audio/<ride>.wav` and `data/work/analysis/stops2-<ride>/{audio_ts,events}.csv`."""

import csv
import re
import sys
import wave
from pathlib import Path

import numpy as np
from scipy import signal

ROOT = Path(__file__).resolve().parents[2]
ANALYSIS = ROOT / "data/work/analysis"
RIDES = ["151449", "154951", "184134", "214427", "135509", "155919", "182135", "215922"]
NPERSEG, HOP = 2048, 512
PROMINENT_DB = 15.0
MIN_S = 0.25
BANDS = [((436, 456), "Ntt"), ((545, 565), "Ntt"), ((512, 526), "R62"), ((630, 650), "R62")]
FAMILY = {"1": "R62", "3": "R62", "2": "Ntt", "7": "Ntt", "E": "Ntt", "F": "Ntt", "M": "Ntt", "R": "Ntt"}


def load(ride):
    d = ANALYSIS / f"stops2-{ride}"
    ts = np.loadtxt(d / "audio_ts.csv", delimiter=",", skiprows=1, dtype=np.int64)
    with wave.open(str(ROOT / "data/work/audio" / f"{ride}.wav")) as w:
        sr = w.getframerate()
        pcm = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float32)
    tags = []
    with open(d / "events.csv") as f:
        for r in csv.DictReader(f):
            m = re.search(r"\(([0-9A-Z])\)", r["text"])
            if r["kind"].endswith("AT_STATION") and m:
                tags.append((int(r["t_ns"]), m.group(1)))
    return sr, pcm, ts, tags


def band_prominence(sr, pcm):
    """Per band, per window: the best bin's dB over its neighborhood median."""
    f, t, z = signal.stft(pcm, sr, nperseg=NPERSEG, noverlap=NPERSEG - HOP, boundary=None, padded=False)
    db = 10 * np.log10(np.abs(z) ** 2 + 1e-9)
    k = int(60 / (f[1] - f[0]))
    out = []
    for (lo, hi), _ in BANDS:
        sel = np.nonzero((f >= lo) & (f <= hi))[0]
        prom = np.stack([db[i] - np.median(db[max(0, i - k) : i + k + 1], axis=0) for i in sel])
        out.append(prom.max(axis=0))
    return t, np.array(out)


def tones(t, prom, sr):
    on = (prom > PROMINENT_DB).any(axis=0)
    min_n = int(np.ceil(MIN_S * sr / HOP))
    i, n = 0, len(on)
    while i < n:
        if not on[i]:
            i += 1
            continue
        j = i
        while j < n and on[j]:
            j += 1
        if j - i >= min_n:
            yield i, j
        i = j


def main():
    totals = {}
    for ride in RIDES:
        sr, pcm, ts, tags = load(ride)
        t, prom = band_prominence(sr, pcm)
        tns = np.interp(t * sr, ts[:, 0], ts[:, 1])
        rows = {"first": [0, 0], "sum": [0, 0], "lead": [0, 0], "two": [0, 0]}
        for i, j in tones(t, prom, sr):
            prior = [line for tt, line in tags if 0 <= tns[i] - tt < 600e9]
            truth = FAMILY.get(prior[-1]) if prior else None
            if truth is None:
                continue
            p = prom[:, i:j]
            fam = np.array([b[1] for b in BANDS])
            # The old rule: the last window's first prominent band, lowest pitch first.
            order = np.argsort([b[0][0] for b in BANDS])
            last = p[:, -1]
            first = next((fam[k] for k in order if last[k] > PROMINENT_DB), None)
            if first is None:
                first = next((fam[k] for k in order if p[k, -2 if p.shape[1] > 1 else -1] > PROMINENT_DB), "Ntt")
            ntt = p[fam == "Ntt"].clip(min=0).sum()
            r62 = p[fam == "R62"].clip(min=0).sum()
            by_sum = "Ntt" if ntt >= r62 else "R62"
            lead = (p[fam == "Ntt"].max(axis=0) > p[fam == "R62"].max(axis=0)).mean()
            by_lead = "Ntt" if lead >= 0.5 else "R62"
            # Both tones of a family: each family's two bands prominent somewhere in the tone.
            both = {f: all((p[k] > PROMINENT_DB).any() for k in range(4) if fam[k] == f) for f in ("Ntt", "R62")}
            by_two = by_sum if both["Ntt"] == both["R62"] else ("Ntt" if both["Ntt"] else "R62")
            for key, guess in [("first", first), ("sum", by_sum), ("lead", by_lead), ("two", by_two)]:
                rows[key][0] += guess == truth
                rows[key][1] += 1
        print(ride, {k: f"{a}/{b}" for k, (a, b) in rows.items()})
        for k, (a, b) in rows.items():
            totals.setdefault(k, [0, 0])
            totals[k][0] += a
            totals[k][1] += b
    print("all", {k: f"{a}/{b}" for k, (a, b) in totals.items()})


if __name__ == "__main__":
    main()

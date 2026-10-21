"""Inverter (VVVF) sound against speed: per 0.5 s, the strongest tonal lines between 100 and 4000 Hz (prominence
over the local median spectrum), printed against our own Doppler speed, to see lines whose pitch follows speed
(motor frequency is proportional to speed, plus slip) during accelerations."""

import csv
import sys
import wave

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS, imu  # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "184134"
t_from, t_to = float(sys.argv[2]), float(sys.argv[3])
_, _, _, t0 = imu(name, 1.0)
with open(ANALYSIS / name / "audio_ts.csv") as f:
    ats = np.array([[int(r["frame"]), (int(r["t_ns"]) - t0) * 1e-9] for r in csv.DictReader(f)])
with wave.open(str(ANALYSIS / name / "audio.wav")) as w:
    rate = w.getframerate()
    pcm = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(float)
slope, icept = np.polyfit(ats[:, 1], ats[:, 0], 1)
with open(ANALYSIS / name / "own.csv") as f:
    own = np.array([[(int(r["t_ns"]) - t0) * 1e-9, float(r["speed"]) * 3.6, float(r["sigma"])] for r in csv.DictReader(f)])
own = own[own[:, 2] < 0.5]
N = 8192
freqs = np.fft.rfftfreq(N, 1 / rate)
band = (freqs > 100) & (freqs < 4000)
for t in np.arange(t_from, t_to, 0.5):
    a = int(slope * t + icept)
    x = pcm[a : a + N]
    if len(x) < N:
        break
    s = 20 * np.log10(np.abs(np.fft.rfft(x * np.hanning(N))) + 1e-9)
    k = 60
    med = np.array([np.median(s[max(0, i - k) : i + k]) for i in range(0, len(s), 4)])
    med = np.repeat(med, 4)[: len(s)]
    p = np.where(band, s - med, -99)
    top = []
    for i in np.argsort(p)[::-1]:
        if p[i] < 12 or len(top) >= 4:
            break
        if all(abs(freqs[i] - f) > 30 for f, _ in top):
            top.append((freqs[i], p[i]))
    v = np.interp(t + 0.25, own[:, 0], own[:, 1])
    print(f"{t:7.1f} {v:5.1f} km/h  " + "  ".join(f"{f:6.0f}/{q:2.0f}" for f, q in sorted(top)))

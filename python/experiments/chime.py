"""Door chimes: tonal peaks (narrow spectral lines well above their neighborhood) in the 20 s before each
"doors closed" press, as a spectrogram summary per second, to see whether a repeatable chime shows up."""

import csv
import sys
import wave

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS, events, imu  # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "154951"
_, _, _, t0 = imu(name, 1.0)
with open(ANALYSIS / name / "audio_ts.csv") as f:
    ats = np.array([[int(r["frame"]), (int(r["t_ns"]) - t0) * 1e-9] for r in csv.DictReader(f)])
with wave.open(str(ANALYSIS / name / "audio.wav")) as w:
    rate = w.getframerate()
    pcm = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(float)
slope, icept = np.polyfit(ats[:, 1], ats[:, 0], 1)
frame = lambda t: int(slope * t + icept)
N = 4096
freqs = np.fft.rfftfreq(N, 1 / rate)
band = (freqs > 300) & (freqs < 4000)


def tonality(t):
    """Per 0.25 s from t: the strongest line's frequency and its prominence (dB over the local median)."""
    out = []
    for k in range(4):
        a = frame(t + k * 0.25)
        x = pcm[a : a + N]
        if len(x) < N:
            return []
        s = 20 * np.log10(np.abs(np.fft.rfft(x * np.hanning(N))) + 1e-9)
        med = np.array([np.median(s[max(0, i - 40) : i + 40]) for i in range(0, len(s))])
        p = np.where(band, s - med, -99)
        i = int(np.argmax(p))
        out.append((freqs[i], p[i]))
    return max(out, key=lambda z: z[1])


for te, k, _ in events(name, t0):
    if k != "DOORS_CLOSED":
        continue
    row = []
    for dt in range(-20, 3):
        r = tonality(te + dt)
        row.append(f"{r[0]:4.0f}/{r[1]:2.0f}" if r else "   -   ")
    print(f"{te:6.0f}  " + " ".join(row))
print("columns: seconds -20..+2 around each doors-closed press; each: strongest tone Hz / prominence dB")

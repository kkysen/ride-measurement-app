"""How well accelerometer and audio band powers separate stopped from moving, per 1 s window, on the elevated 7.

Truth: stopped where the phone's speed is 0 and our own Doppler under 5 km/h; moving where our own is over 15 km/h.
Prints each band's AUC (1 = perfectly louder when moving, 0 = perfectly quieter, 0.5 = no use) and its
correlation with speed while moving.
"""

import csv
import sys
import wave

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS, fixes, ride, session_dir  # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "154951"
t_from, t_to = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (40.0, 1360.0)
a = ride.read_stream(session_dir(name), "accel_uncal")
t0 = a.timestamps_ns[0]
at = (a.timestamps_ns - t0) * 1e-9
av = a.values[:, :3].astype(float)

with open(ANALYSIS / name / "own.csv") as f:
    own = np.array([[(int(r["t_ns"]) - t0) * 1e-9, float(r["speed"])] for r in csv.DictReader(f)]).reshape(-1, 2)
fx = fixes(name, t0)

with open(ANALYSIS / name / "audio_ts.csv") as f:
    ats = np.array([[int(r["frame"]), (int(r["t_ns"]) - t0) * 1e-9] for r in csv.DictReader(f)])
with wave.open(str(ANALYSIS / name / "audio.wav")) as w:
    rate = w.getframerate()
    pcm = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(float)
# Frame -> time: a straight fit, as the recorded rate is steady (measured 15999.33 Hz).
fit = np.polyfit(ats[:, 0], ats[:, 1], 1)
audio_t0, audio_dt = fit[1], fit[0]

ACC_BANDS = [(1, 4), (4, 10), (10, 25), (25, 60), (60, 120), (120, 240)]
AUD_BANDS = [(20, 100), (100, 300), (300, 1000), (1000, 3000), (3000, 8000)]


def band_powers(x, fs, bands):
    x = x - x.mean(axis=0)
    spec = np.abs(np.fft.rfft(x * np.hanning(len(x))[:, None] if x.ndim > 1 else x * np.hanning(len(x)), axis=0)) ** 2
    if spec.ndim > 1:
        spec = spec.sum(axis=1)
    freqs = np.fft.rfftfreq(len(x), 1 / fs)
    return [np.log10(spec[(freqs >= lo) & (freqs < hi)].sum() + 1e-12) for lo, hi in bands]


rows = []
for w0 in np.arange(t_from, t_to - 1, 1.0):
    i, j = np.searchsorted(at, [w0, w0 + 1])
    if j - i < 400:
        continue
    k0 = int((w0 - audio_t0) / audio_dt)
    k1 = k0 + rate
    if k0 < 0 or k1 > len(pcm):
        continue
    oi = np.argmin(abs(own[:, 0] - (w0 + 0.5))) if len(own) else 0
    fi = np.argmin(abs(fx["t"] - (w0 + 0.5)))
    # Without GNSS (underground), keep the window unlabeled, for applying a model trained elsewhere.
    o = own[oi, 1] * 3.6 if len(own) and abs(own[oi, 0] - (w0 + 0.5)) <= 1 else np.nan
    p = fx["speed"][fi] * 3.6 if abs(fx["t"][fi] - (w0 + 0.5)) <= 1 else np.nan
    rows.append([w0, o, p]
                + band_powers(av[i:j], 480, ACC_BANDS) + band_powers(pcm[k0:k1], rate, AUD_BANDS))
rows = np.array(rows)
speed, phone = rows[:, 1], rows[:, 2]
stopped = (phone == 0) & (speed < 5)
moving = speed > 15
print(f"{stopped.sum()} stopped and {moving.sum()} moving windows of {len(rows)}")


def auc(x):
    pos, neg = x[moving], x[stopped]
    return (np.sum(pos[:, None] > neg[None, :]) + 0.5 * np.sum(pos[:, None] == neg[None, :])) / (len(pos) * len(neg))


names = [f"accel {lo}-{hi} Hz" for lo, hi in ACC_BANDS] + [f"audio {lo}-{hi} Hz" for lo, hi in AUD_BANDS]
for c, n in enumerate(names):
    x = rows[:, 3 + c]
    r = np.corrcoef(x[moving], speed[moving])[0, 1]
    print(f"{n:22s} AUC {auc(x):.3f}   stopped {np.median(x[stopped]):6.2f}  moving {np.median(x[moving]):6.2f}  "
          f"(log power)   corr with speed while moving {r:+.2f}")
np.save(ANALYSIS / name / "vibration.npy", rows)

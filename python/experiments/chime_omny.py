"""Door chimes and OMNY beeps as events, at 32 ms resolution, scored against the rider's marks
(which only score): a chime should come in the 8 s before each "doors closed" press,
and an OMNY beep at each turnstile (Google Wallet taps: three on 2026-10-20, one on 2026-10-21 at 110 St,
placed by the pressure: between the stairs from the street and the stairs to the platform).

Chime pitches differ by car type (441/555 Hz on the 7, 2, F, R, M; 645/516 Hz on the 1), so each ride's are found
from the data: the 8 Hz bins whose tone is prominent before "doors closed" presses far more often than elsewhere.
Leave-one-press-out isn't done, so the chime's recall is optimistic; its false rate isn't."""

import csv
import sys
import wave
from pathlib import Path

import numpy as np
from scipy import signal

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from subway import ANALYSIS, RIDES  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
NPERSEG, HOP = 2048, 512
PROMINENT_DB = 15.0
# The Google Wallet taps of 2026-10-20 (UTC milliseconds from their local times, EDT).
OMNY_TAPS = {"184134": ["18:56:03", "19:16:02"], "214427": ["21:45:06.8"], "135509": ["13:57:06 (110 St)"]}


def load(name):
    ts = np.loadtxt(ANALYSIS / name / "audio_ts.csv", delimiter=",", skiprows=1, dtype=np.int64)
    with wave.open(str(ROOT / "target/audio" / f"{name}.wav")) as w:
        sr = w.getframerate()
        pcm = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float32)
    return sr, pcm, ts


def prominence(sr, pcm, lo, hi):
    """Spectrogram in [lo, hi] Hz as dB over each bin's +-60 Hz neighborhood median (a pure tone stands out)."""
    f, t, z = signal.stft(pcm, sr, nperseg=NPERSEG, noverlap=NPERSEG - HOP, boundary=None, padded=False)
    p = np.abs(z) ** 2 + 1e-9
    db = 10 * np.log10(p)
    df = f[1] - f[0]
    k = int(60 / df)
    sel = np.nonzero((f >= lo) & (f <= hi))[0]
    med = np.stack([np.median(db[max(0, i - k) : i + k + 1], axis=0) for i in sel])
    return f[sel], t, db[sel] - med


def frame_times_ns(t, sr, ts):
    """Audio frame time (s into the WAV) to elapsed realtime ns."""
    return np.interp(t * sr, ts[:, 0], ts[:, 1])


def events(name):
    with open(ANALYSIS / name / "events.csv") as f:
        return [(int(r["t_ns"]), r["kind"].removeprefix("KIND_")) for r in csv.DictReader(f)]


def runs(mask, t, min_s, max_s=np.inf):
    """Start times of runs of True lasting min_s to max_s."""
    out, start = [], None
    for i, m in enumerate(np.append(mask, False)):
        if m and start is None:
            start = i
        elif not m and start is not None:
            d = (i - start) * HOP
            if min_s <= d / 16000 <= max_s:
                out.append(t[start])
            start = None
    return out


def chimes(name):
    sr, pcm, ts = load(name)
    f, t, prom = prominence(sr, pcm, 300, 1000)
    tn = frame_times_ns(t, sr, ts)
    closed = np.array([t for t, k in events(name) if k == "DOORS_CLOSED"])
    if not len(closed):
        print(f"{name}: no doors-closed presses")
        return
    before = np.zeros(len(tn), bool)
    for c in closed:
        before |= (tn >= c - 8e9) & (tn <= c)
    hot = prom > PROMINENT_DB
    # Per bin: how much more often it's prominent before a press than elsewhere.
    rate_before = hot[:, before].mean(1)
    rate_else = hot[:, ~before].mean(1) + 1e-4
    lift = rate_before / rate_else
    pitches = [f[i] for i in np.argsort(-lift)[:6] if lift[i] > 3 and rate_before[i] > 0.02]
    if not pitches:
        print(f"{name}: no chime pitch stands out")
        return
    idx = [int(np.argmin(np.abs(f - p))) for p in pitches]
    tone = hot[idx].any(0)
    found = np.array(runs(tone, tn, 0.25))
    # Merge detections closer than 3 s (a chime is a few notes).
    merged = [x for i, x in enumerate(found) if i == 0 or x - found[i - 1] > 3e9]
    merged = np.array(merged)
    hit = [((merged >= c - 8e9) & (merged <= c + 1e9)).any() for c in closed]
    door_any = np.array([t for t, k in events(name) if k.startswith("DOORS")])
    far = [m for m in merged if not (np.abs(door_any - m) < 15e9).any()]
    hours = (tn[-1] - tn[0]) * 1e-9 / 3600
    print(f"{name}: chime pitches {', '.join(f'{p:.0f}' for p in sorted(pitches))} Hz; "
          f"{sum(hit)} of {len(closed)} doors-closed presses have one in the 8 s before; "
          f"{len(far)} detections away from doors ({len(far) / hours:.1f}/h)")


def omny(name):
    sr, pcm, ts = load(name)
    f, t, prom = prominence(sr, pcm, 2700, 3150)
    tn = frame_times_ns(t, sr, ts)
    band = (f >= 2800) & (f <= 3050)
    # Above 20 dB, bridging gaps of up to two 32 ms frames: one frame below broke
    # the 2026-10-21 tap at 110 St into pieces too short to count. (A lower floor instead joined background tones on.)
    strong = (prom[band] > 20).any(0)
    tone = strong.copy()
    on = np.nonzero(strong)[0]
    for a, b in zip(on[:-1], on[1:]):
        if 1 < b - a <= 3:
            tone[a:b] = True
    found = runs(tone, tn, 0.15, 0.5)
    # Clock times from a fix's UTC milliseconds.
    with open(ANALYSIS / name / "fixes.csv") as fh:
        r = next(csv.DictReader(fh))
    offset = int(r["time_ms"]) * 1_000_000 - int(r["t_ns"])
    import datetime

    clock = lambda ns: datetime.datetime.fromtimestamp((ns + offset) / 1e9).strftime("%H:%M:%S.%f")[:-4]
    print(f"{name}: {len(found)} OMNY-like beeps at {', '.join(clock(x) for x in found[:12])}"
          + (" ..." if len(found) > 12 else "")
          + (f"; known taps at {', '.join(OMNY_TAPS[name])}" if name in OMNY_TAPS else ""))


if __name__ == "__main__":
    if sys.argv[1:] != ["omny"]:
        for r in RIDES:
            chimes(r)
    for r in RIDES:
        omny(r)

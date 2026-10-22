"""Elevator or escalator, by the shape of the climb and what comes before and after it, for every "riding" height change
(`elevator.moves`): an elevator car speeds up, cruises, and slows down, and the rider waits for it and for its doors;
an escalator climbs steadily from the moment it's stepped onto to the moment it's stepped off, walking on both sides.
Per change: the vertical speed's rise and fall times (10% to 90% of its cruise, from the barometer at 2 Hz smoothed
over 2 s), the cruise speed, and the gaps from the last walking window before to the start, and from the end to the
first walking window after (`sensor_trips.walking`, 4 s windows).

Result (2026-10-22, 10 changes): no separation yet, and too few labels to score one.
The home elevator going up at rides' ends (Oct 20 10:03 pm, Oct 21 4:35 pm and 10:49 pm, Oct 22 3:34 am):
0.19 to 0.47 m/s, unsteadiness (4 s speeds' spread over their mean) 0.11 to 1.12, walking ending -0.5 to 15 s
before and starting 15 to 38 s after (the walking detector needs several windows, so "after" runs late).
The PABT escalators to Gate 301 (`002914`, about 12:43 am) aren't among the riding changes at all,
and the other changes (1:18 am and 2:46 am on `002914`, Oct 20 4:42 pm, Oct 21 2:17 pm, 3:19 pm, 4:05 pm)
are unlabeled. A 0.5 s barometer median is too noisy for an elevator's speed-up and slow-down over 10 m.
Next: the rider's labels for escalator rides (times), and a finer step-onset detector than 4 s windows."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import ride  # noqa: E402
from elevator import moves  # noqa: E402
from elevator_jolts import altitude, ends  # noqa: E402
from sensor_trips import walking  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
WINDOW_S = 4.0


def shape(sec, alt, a, b):
    """The vertical speed over the climb from `a` to `b` (heights at 10% and 90% of the change): its mean (m/s),
    and its unsteadiness, the spread of the 4 s mean speeds within the climb over their mean."""
    m = (sec >= a) & (sec <= b)
    s, z = sec[m], alt[m]
    if len(s) < 10:
        return np.nan, np.nan
    mean = abs(z[-1] - z[0]) / ((s[-1] - s[0]) * 1e-9)
    k = 8  # 4 s at 2 Hz
    seg = np.array([abs(z[i + k] - z[i]) / ((s[i + k] - s[i]) * 1e-9) for i in range(0, len(z) - k, 2)])
    return mean, seg.std() / mean if len(seg) else np.nan


def clock(session, t):
    """`t` as a New York clock time, from the ride's exported fixes (`target/analysis/<ride>/fixes.csv`)."""
    import csv
    from datetime import datetime
    from zoneinfo import ZoneInfo

    fixes = ROOT / "target/analysis" / session.name.split("-")[1] / "fixes.csv"
    if not fixes.exists():
        return "?"
    r = next(csv.DictReader(open(fixes)))
    off = int(r["time_ms"]) * 1_000_000 - int(r["t_ns"])
    return datetime.fromtimestamp((t + off) / 1e9, ZoneInfo("America/New_York")).strftime("%b %-d %-I:%M:%S %p").lower()


def gaps(walks, a, b):
    before = [w for w in walks if w + WINDOW_S * 1e9 <= a + 2e9]
    after = [w for w in walks if w >= b - 2e9]
    gb = (a - (before[-1] + WINDOW_S * 1e9)) * 1e-9 if before else np.nan
    ga = (after[0] - b) * 1e-9 if after else np.nan
    return gb, ga


if __name__ == "__main__":
    for s in sorted((ROOT / "data/sessions").iterdir()):
        try:
            walks = walking(s)
            mv = [m for m in moves(s, walks) if m[3] == "riding"]
        except Exception as e:  # noqa: BLE001
            print(f"{s.name}: {e}")
            continue
        if not mv:
            continue
        sec, alt = altitude(s)
        for t0, dz, _, _ in mv:
            a, b = ends(sec, alt, t0, dz)
            mean, unsteady = shape(sec, alt, a, b)
            gb, ga = gaps(walks, a, b)
            print(f"{s.name} {clock(s, t0)} {dz:+5.1f} m in {(b - a) * 1e-9:4.0f} s, {mean:4.2f} m/s, "
                  f"unsteady {unsteady:4.2f}, walk ends {gb:5.1f} s before, starts {ga:5.1f} s after")

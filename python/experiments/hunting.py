"""Hunting, the bogies' side-to-side sway, told from walking: in the gyro-tracked world frame
(`startstop.world_horizontal_50hz`), per 10 s window, the horizontal and vertical acceleration power in 0.5 to 4 Hz,
and the sharpest horizontal spectral peak against the band's median. Walking bounces the rider up and down at the
step rate; a train sways them sideways. Classes, which only score: walking (`sensor_trips.walking`), aboard and
moving (between the rider's marks, away from heard door cycles), aboard and standing (inside a door cycle).

On six rides: horizontal/vertical power is 0.38 walking (median, 580 windows) and 1.37 on a moving train (218),
AUC 0.86, so sideways sway against vertical bounce tells a train from walking, which the old per-window hunting peak
(`subway_signals.py`) got backwards. It doesn't tell moving from standing aboard (AUC 0.60 for the peak's
sharpness; standing at a station sways relatively more, 3.12, from people moving about), so it confirms
"on a train, not walking" but not motion."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import sensor_trips as ST  # noqa: E402
import startstop as ss  # noqa: E402
import stops as S  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
WINDOW = 500  # 10 s at 50 Hz
RIDES = {"135509": "20261021-135509", "182135": "20261021-182135", "215922": "20261021-215922",
         "214427": "20261020-214427", "184134": "20261020-184134", "155919": "20261021-155919"}


def features(t, a):
    f = np.fft.rfftfreq(WINDOW, 1 / 50)
    band = (f >= 0.5) & (f <= 4)
    win = np.hanning(WINDOW)
    out = []
    for i in range(0, len(t) - WINDOW, WINDOW):
        seg = a[i : i + WINDOW]
        spec = [np.abs(np.fft.rfft((seg[:, k] - seg[:, k].mean()) * win)) ** 2 for k in range(3)]
        h = (spec[0] + spec[1])[band]
        v = spec[2][band]
        out.append((t[i], h.sum() / max(v.sum(), 1e-12), h.max() / max(np.median(h), 1e-12), f[band][np.argmax(h)]))
    return np.array(out)


def auc(pos, neg):
    pos, neg = np.asarray(pos), np.asarray(neg)
    if not len(pos) or not len(neg):
        return np.nan
    return (np.mean(pos[:, None] > neg[None, :]) + 0.5 * np.mean(pos[:, None] == neg[None, :]))


def main():
    allc = {"walking": [], "moving": [], "standing": []}
    for name, sess in RIDES.items():
        s = ROOT / "data/sessions" / sess
        t, a = ss.world_horizontal_50hz(session=s)
        fe = features(t, a)
        walks = ST.walking(s)
        cyc = np.loadtxt(ROOT / f"data/work/doors_eval/track/{name}/door_cycles.csv", delimiter=",", skiprows=1,
                         dtype=np.int64).reshape(-1, 2)
        on = S.aboard(name, fe[:, 0])
        walking = np.array([((walks >= x - 4e9) & (walks < x + 10e9)).any() for x in fe[:, 0]])
        standing = np.array([((cyc[:, 1] - 25e9 <= x) & (x + 10e9 <= cyc[:, 1])).any() for x in fe[:, 0]])
        near_door = np.array([(np.abs(cyc[:, 1] - x) < 60e9).any() for x in fe[:, 0]])
        allc["walking"].append(fe[walking & ~on])
        allc["moving"].append(fe[on & ~walking & ~near_door])
        allc["standing"].append(fe[on & standing])
    c = {k: np.vstack([x for x in v if len(x)]) for k, v in allc.items()}
    for k, v in c.items():
        print(f"{k:9s} {len(v):4d} windows: horizontal/vertical median {np.median(v[:, 1]):.2f}, "
              f"peak/median {np.median(v[:, 2]):.1f}, peak Hz median {np.median(v[:, 3]):.1f}")
    print(f"AUC horizontal/vertical, moving train vs walking: {auc(c['moving'][:, 1], c['walking'][:, 1]):.2f}")
    print(f"AUC peak sharpness, moving vs standing aboard: {auc(c['moving'][:, 2], c['standing'][:, 2]):.2f}")
    print(f"AUC horizontal/vertical, moving vs standing aboard: {auc(c['moving'][:, 1], c['standing'][:, 1]):.2f}")


if __name__ == "__main__":
    main()

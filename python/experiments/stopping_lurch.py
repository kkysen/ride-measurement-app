"""The lurch that stops the train: of the braking lurches around a stop (`stops.lurches`), the one after which
the train stands, the known zero speed. Hard braking without stopping also lurches, and the rider moving to get
off right after a stop jolts too, so neither "the last lurch before the doors close" nor "any lurch" works.

In the gyro-tracked world frame, each lurch gets:
- `after`: the mean acceleration along its braking direction over the 1 to 5 s after it. Stopped, the car is
  still or settles back (10th to 90th percentile at tapped stops -0.35 to +0.07 m/s^2); a lurch in braking that
  goes on still averages +0.04 to +0.82;
- `braked`: the share of the 20 s before spent braking along that direction (median 0.83 at stops, 0.62 not);
- `size`: the braking just before it (median 0.73 m/s^2 at stops, 0.48 not).
A lurch is a candidate if `after` is under `AFTER_MAX`; its score is `braked * size - 2 * max(after, 0)`,
and it stops the train if the score is at least `MIN_SCORE` and the best within `SEP_S` either side.
(Requiring a door cycle or start surge after it lost more true stops than false ones: the door finder misses many.)

Truth, which only scores: the "train stopped" taps of the rides with precise ones (`stops.PRECISE`), which come
about 0.8 s after the lurch; "doors open" taps for the delay to the doors.

Result (2026-10-22), 4 rides, 28 stop taps: 20 within 1 s, 21 within 2 s (any lurch at all: 23 and 24, among 746).
On all 7 rides, 93 found, 51 of them aboard (by the taps); of those GNSS can check (confident within 2 s),
2 at speed and 1 stopped. Doors open taps follow the stopping lurch by a median 2.9 s.

Openings anchored on it (`--openings`): the ride's learned opening sound (`door_lurch_openings.learned`) searched
1 to 10 s after each stopping lurch; see `main_openings`.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import doors as D  # noqa: E402
import startstop as ss  # noqa: E402
from stops import PRECISE, RIDES, aboard, gnss_speed, lurches, session  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
CACHE = ROOT / "data/work/cache/world_h"
RATE = 50.0
DIR_S = 3.0
AFTER_S = (1.0, 5.0)
BRAKED_S = 20.0
AFTER_MAX = 0.1
MIN_SCORE = 0.2
SEP_S = 30.0


def world_h(name):
    """`startstop.world_horizontal_50hz`, cached under `data/work/cache`."""
    CACHE.mkdir(parents=True, exist_ok=True)
    f = CACHE / f"{name}.npz"
    if f.exists():
        z = np.load(f)
        return z["t"], z["h"]
    t, h3 = ss.world_horizontal_50hz(session=session(name))
    h = np.column_stack([h3[:, 0], h3[:, 1], np.zeros(len(h3))])
    np.savez(f, t=t, h=h)
    return t, h


def mean1s(h):
    n = int(RATE)
    c = np.cumsum(np.vstack([np.zeros(3), h]), axis=0)
    m = np.zeros_like(h)
    m[n:] = (c[n + 1 :] - c[1 : len(h) - n + 1]) / n
    return m


def features(t, h, m, at):
    """(after, braked, size) for the lurch at `at`, or None near the ride's end."""
    i = np.searchsorted(t, at)
    u = h[max(0, i - int(DIR_S * RATE)) : i].mean(0)
    size = np.linalg.norm(u)
    u /= size + 1e-12
    a, b = i + int(AFTER_S[0] * RATE), i + int(AFTER_S[1] * RATE)
    if b > len(m):
        return None
    after = (m[a:b] @ u).mean()
    braked = ((h[max(0, i - int(BRAKED_S * RATE)) : i] @ u) > 0.2).mean()
    return after, braked, size


def score(f):
    if f is None or f[0] >= AFTER_MAX:
        return -np.inf
    after, braked, size = f
    return braked * size - 2 * max(after, 0.0)


def stopping(t, h):
    """(times ns, scores) of the stopping lurches."""
    m = mean1s(h)
    lu = lurches(t, h)
    sc = np.array([score(features(t, h, m, x)) for x in lu])
    keep = [k for k in range(len(lu)) if sc[k] >= MIN_SCORE
            and not ((np.abs(lu - lu[k]) <= SEP_S * 1e9) & (sc > sc[k])).any()]
    return lu[keep], sc[keep]


def main():
    tot = np.zeros(7, int)
    offs = []
    for name in RIDES:
        t, h = world_h(name)
        sl, _ = stopping(t, h)
        on = aboard(name, sl) if len(sl) else np.array([], bool)
        v = gnss_speed(name, sl[on]) if on.any() else np.array([])
        opens = np.array(D.taps(name, "DOORS_OPEN"))
        for o in opens:
            before = sl[(sl < o) & (sl > o - 30e9)]
            if len(before):
                offs.append((o - before[-1]) / 1e9)
        row = [0, 0, 0, len(sl), int(on.sum()), int(np.sum(v > 3)), int(np.sum(v < 0.5))]
        if name in PRECISE:
            at = np.array(D.taps(name, "TRAIN_STOPPED"))
            near = [np.min(np.abs(sl - a)) if len(sl) else np.inf for a in at]
            row[:3] = [len(at), sum(d < 1e9 for d in near), sum(d < 2e9 for d in near)]
        tot += row
        print(f"{name}: {row[3]} stopping lurches ({row[4]} aboard; GNSS: {row[5]} at speed, {row[6]} stopped)"
              + (f"; {row[0]} stop taps, {row[1]} within 1 s, {row[2]} within 2 s" if name in PRECISE else ""))
    print(f"all: {tot[0]} stop taps, {tot[1]} within 1 s, {tot[2]} within 2 s; {tot[3]} found, {tot[4]} aboard, "
          f"GNSS {tot[5]} at speed, {tot[6]} stopped")
    o = np.array(offs)
    print(f"doors open taps after the stopping lurch: {len(o)}, median {np.median(o):.1f} s, "
          f"10-90% {np.percentile(o, 10):.1f} to {np.percentile(o, 90):.1f} s")


def main_openings():
    """Door openings searched for after each stopping lurch whose door cycle closes within 120 s, scored
    against the "doors open" taps within 3 s like `door_lurch_openings`."""
    import door_lurch_openings as L
    import door_self_template as S

    bars = [-1.0, 0.1, 0.2, 0.3]
    hits = {b: 0 for b in bars}
    extra = {b: 0 for b in bars}
    n_taps = 0
    for name in S.RIDES:
        cycles = np.loadtxt(ROOT / "data/work/doors" / f"{name}.csv", delimiter=",", skiprows=1,
                            dtype=np.int64).reshape(-1, 2)
        tn, db, tpl, picks = L.learned(name, cycles)
        t, h = world_h(name) if name in RIDES else (None, None)
        if t is None:
            t3, h3 = ss.world_horizontal_50hz(session=next((ROOT / "data/sessions").glob(f"*-{name}")))
            t, h = t3, np.column_stack([h3[:, 0], h3[:, 1], np.zeros(len(h3))])
        sl, _ = stopping(t, h)
        opens = np.array(D.taps(name, "DOORS_OPEN"))
        n_taps += len(opens)
        found = []
        for (_, close), pick in zip(cycles, picks):
            before = sl[(sl > close - 120e9) & (sl < close - 8e9)]
            f = L.best_after(tn, db, tpl, before[-1], close) if len(before) else None
            found.append((f, pick))
        for bar in bars:
            chosen = np.array([f[0] if f is not None and f[1] >= bar else p for f, p in found
                               if (f is not None and f[1] >= bar) or p is not None])
            hits[bar] += sum(1 for a in opens if len(chosen) and np.min(np.abs(chosen - a)) < 3e9)
            extra[bar] += sum(1 for c in chosen if not len(opens) or np.min(np.abs(opens - c)) >= 3e9)
    for bar in bars:
        print(f"anchored on the stopping lurch, bar {bar}: {hits[bar]} of {n_taps} taps within 3 s, "
              f"{extra[bar]} picks untapped")


if __name__ == "__main__":
    main_openings() if "--openings" in sys.argv else main()

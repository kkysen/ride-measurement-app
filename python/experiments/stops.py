"""One stop detector from the train itself: the final lurch (a collapse of sustained braking, in the gyro-tracked
frame), confirmed by a door cycle in the sound (a step up, then a chime and a step down) or by the start surge
that follows. No stillness is required: the rider often moves to get off right after a stop.

Truth, which only scores: the precise "train stopped" taps of `20261021-135509`; on the other phone-mic rides,
"doors open" taps (the stop comes a few seconds before), which miss stops the rider didn't tap.
Prints each rule's hits, misses, and detections away from any truth, per ride and per hour."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import doors  # noqa: E402
import startstop as ss  # noqa: E402
from subway import on_train_intervals, rows  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
RIDES = {
    "135509": "20261021-135509",
    "154951": "20261020-154951",
    "184134": "20261020-184134",
    "214427": "20261020-214427",
}
RATE = 50.0


def session(name):
    local = ROOT / "data/sessions" / RIDES[name]
    return local if local.exists() else Path("/mnt/c/Users/kkyse/OneDrive/Khyber/Documents/RideLogger/sessions") / RIDES[name]


def lurches(t, h, min_braking=0.5):
    """Collapses of sustained braking (`startstop.surge_ends`) after braking for `min_braking` of the 10 s before."""
    out = []
    for ti, _, _ in ss.surge_ends(t, h, sustained=0.3, collapse=0.5):
        i = np.searchsorted(t, ti)
        u = h[i - 150 : i].mean(0)
        u /= np.linalg.norm(u)
        if ((h[max(0, i - 500) : i - 25] @ u) > 0.2).mean() >= min_braking:
            out.append(ti)
    return np.array(out)


def surges(t, h, steady=0.06, steady_s=4.0, strong=0.5, strong_s=3.0):
    """Start surges: horizontal acceleration (1 s mean) stepping by over `strong` from a steady level and holding
    along one direction for `strong_s`. The level before is steady (spread under `steady`) but not zero:
    a tilt of a degree or two leaks 0.2 to 0.4 m/s^2 of gravity in, which the step is measured from."""
    n = int(RATE)
    c = np.cumsum(np.vstack([np.zeros(3), h]), axis=0)
    m = np.zeros_like(h)
    m[n:] = (c[n + 1 :] - c[1 : len(h) - n + 1]) / n
    nq, ns = int(steady_s * RATE), int(strong_s * RATE)
    out, last = [], -np.inf
    for i in range(nq + n, len(h) - ns, 5):
        if t[i] - last < 20e9:
            continue
        before = m[i - nq - n : i - n]
        base = before.mean(0)
        step = m[i] - base
        size = np.linalg.norm(step)
        if size < strong or np.linalg.norm(before - base, axis=1).max() > 2.5 * steady or before.std(0).max() > steady:
            continue
        if ((m[i : i + ns] - base) @ (step / size)).min() > strong:
            out.append(t[i])
            last = t[i]
    return np.array(out)


def door_cycles(name, up_db=3.0, down_db=2.0, max_s=60.0):
    """`doors.sequences`' cycles, at most `max_s` long: (open, close) times in ns.
    Most tapped dwells are 13 to 41 s; the cycles found at speed on `154951` were 75 to 80 s."""
    t, lv = doors.levels(name)
    ups, downs = doors.steps(t, lv["high 1-4 kHz"], up_db, down_db)
    ch = doors.chime_times(name)
    out = []
    for u in ups:
        for d in downs[(downs > u + 8e9) & (downs < u + max_s * 1e9)]:
            if ((ch > d - 8e9) & (ch < d)).any():
                out.append((u, d))
                break
    return np.array(out).reshape(-1, 2)


def truth(name):
    """(times ns, window before, window after) a detection may fall in, in s."""
    if name == "135509":
        return np.array(ss.taps()["TRAIN_STOPPED"]), 3.0, 1.0
    return np.array(doors.taps(name, "DOORS_OPEN")), 25.0, 1.0


def score(found, at, before, after):
    hit = sum(1 for a in at if ((found > a - before * 1e9) & (found < a + after * 1e9)).any())
    away = sum(1 for f in found if not ((at > f - after * 1e9 - 30e9) & (at < f + before * 1e9 + 30e9)).any())
    return hit, away


def gnss_speed(name, at):
    """Our own Doppler speed (m/s) at each time where it's confident (sigma under 0.5 m/s within 2 s), else NaN."""
    own = np.array([[int(r["t_ns"]), float(r["speed"]), float(r["sigma"])] for r in rows(name, "own.csv")])
    own = own[own[:, 2] < 0.5]
    out = np.full(len(at), np.nan)
    for k, a in enumerate(at):
        j = np.searchsorted(own[:, 0], a)
        near = [i for i in (j - 1, j) if 0 <= i < len(own) and abs(own[i, 0] - a) < 2e9]
        if near:
            out[k] = own[min(near, key=lambda i: abs(own[i, 0] - a)), 1]
    return out


def aboard(name, x):
    """Which times fall on a train, from the Boarded and Alighted marks (scoring only); all of them if none."""
    iv = on_train_intervals(name)
    if not iv:
        return np.ones(len(x), bool)
    return np.array([any(a <= v <= b for a, b in iv) for v in x], bool).reshape(-1)


def main():
    rules = ["lurch", "doors", "start", "lurch + doors", "lurch + start", "lurch + (doors or start)",
             "(lurch + (doors or start)) or doors"]
    total = {r: np.zeros(5, int) for r in rules}
    for name in RIDES:
        t, h3 = ss.world_horizontal_50hz(session=session(name))
        h = np.column_stack([h3[:, 0], h3[:, 1], np.zeros(len(h3))])
        lu, st, cy = lurches(t, h), surges(t, h), door_cycles(name)
        opens = cy[:, 0] if len(cy) else np.array([])

        def near(x, ref, lo, hi):
            return np.array([((ref > v + lo * 1e9) & (ref < v + hi * 1e9)).any() for v in x], bool).reshape(-1)

        by_doors, by_start = near(lu, opens, 0, 25), near(lu, st, 5, 180)
        # A door cycle with no lurch before it: the stop, a few seconds before the doors open.
        lone = opens[~near(opens, lu, -25, 0)] - 3e9
        found = {
            "lurch": lu,
            "doors": opens - 3e9,
            # A start's stop: somewhere before it; dated 20 s before for scoring.
            "start": st - 20e9,
            "lurch + doors": lu[by_doors],
            "lurch + start": lu[by_start],
            "lurch + (doors or start)": lu[by_doors | by_start],
            "(lurch + (doors or start)) or doors": np.sort(np.concatenate([lu[by_doors | by_start], lone])),
        }
        at, before, after = truth(name)
        print(f"{name}: {len(at)} truth stops; {len(lu)} lurches, {len(st)} start surges, {len(cy)} door cycles")
        print(f"  {'':36s} hit  of | away aboard: at speed, stopped, GNSS unsure | away off the train")
        for r in rules:
            f = found[r]
            win = (25.0, 1.0) if r in ("doors", "start") else (before, after)
            hit, _ = score(f, at, *win)
            away = np.array([not ((at > v - win[1] * 1e9 - 30e9) & (at < v + win[0] * 1e9 + 30e9)).any() for v in f], bool)
            on = aboard(name, f)
            v = gnss_speed(name, f[away & on])
            row = np.array([hit, len(at), np.sum(v > 3), np.sum(v < 0.5), np.sum(~(v > 3) & ~(v < 0.5))])
            off = int(np.sum(away & ~on))
            total[r] += row
            print(f"  {r:36s} {hit:3d} {len(at):3d} | {row[2]:4d} {row[3]:4d} {row[4]:4d} | {off:4d}")
    print("\nall rides, aboard")
    for r, (hit, n, fast, stopped, unsure) in total.items():
        print(f"  {r:36s} {hit:3d} of {n:3d}; away: {fast:3d} at speed, {stopped:3d} stopped, {unsure:3d} unsure")


if __name__ == "__main__":
    main()

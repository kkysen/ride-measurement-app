"""Door openings anchored on the train's stop: the doors only open once it has stopped, a few seconds after the final
brake lurch (`stops.lurches`), so search 1 to 10 s after each lurch whose door cycle closes within 60 s for the ride's
own learned opening sound (`door_self_template`), at a looser bar than a search over the whole cycle needs.
Cycles with no lurch keep the learned pick among the 1 to 4 kHz steps up.
Scores against the rider's "doors open" taps (which only score), within 3 s, against `door_self_template`'s learned
picks; and counts picks with no tap within 3 s (some stops weren't tapped, so not all are wrong).

Result (2026-10-22), 9 rides, 104 door cycles (87 with a lurch in the 60 s before the closing), 60 "doors open" taps:
learned picks alone 30 hits; anchored, at a correlation bar of 0.3, 32 (bars of 0.2 and 0.4: 30, 31; no bar: 26).
The rider's taps do follow the lurch steadily (46 taps, all with a lurch in the 60 s before: 1.0 to 5.6 s after,
10th to 90th percentile, median 3.3), but "the cycle's last lurch plus 3.3 s" hits only 24: the last lurch
before a closing is often not the one that stopped the train. Not clearly better, so not ported to `doors.rs`.
Next: find the stopping lurch itself (the first after sustained braking that's followed by no motion),
which would fix the window.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import door_self_template as S  # noqa: E402
import doors as D  # noqa: E402
import door_template as T  # noqa: E402
import startstop as ss  # noqa: E402
from stops import lurches  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
AFTER_S = (1.0, 10.0)
BARS = [-1.0, 0.1, 0.2, 0.3, 0.4]


def session(name):
    return next((ROOT / "data/sessions").glob(f"*-{name}"))


def learned(name, cycles):
    """The ride's learned opening template and each cycle's learned pick (or None)."""
    t, lv = D.levels(name)
    tn, db = T.spectrogram(name)
    cands = [[(u, c) for u in S.candidates(t, lv, close) if (c := S.cut(tn, db, u)) is not None] for _, close in cycles]
    tpl = np.mean([c for cs in cands for _, c in cs], axis=0)
    for _ in range(5):
        picks = [max(cs, key=lambda uc: S.corr(uc[1], tpl)) for cs in cands if cs]
        tpl = np.mean([c for _, c in picks], axis=0)
    picks = [max(cs, key=lambda uc: S.corr(uc[1], tpl))[0] if cs else None for cs in cands]
    return tn, db, tpl, picks


def best_after(tn, db, tpl, lurch, close):
    """The best template match (time of the onset, correlation) from 1 to 10 s after the lurch, before the closing."""
    a0 = int(T.SPAN[0] * T.RATE)
    lo, hi = lurch + AFTER_S[0] * 1e9, min(lurch + AFTER_S[1] * 1e9, close - 8e9)
    i0, i1 = np.searchsorted(tn, [lo, hi])
    i0 += a0
    if i0 < 0 or i1 <= i0 or i1 + tpl.shape[1] > db.shape[1]:
        return None
    seg = db[:, i0 : i1 + tpl.shape[1]]
    m = T.match(seg, tpl)[: i1 - i0]
    k = int(np.argmax(m))
    return tn[i0 + k - a0], m[k]


def main():
    hits = {b: 0 for b in BARS}
    extra = {b: 0 for b in BARS}
    base_hits = base_extra = n_taps = n_cycles = n_anchored = 0
    for name in S.RIDES:
        cycles = np.loadtxt(ROOT / "target/doors" / f"{name}.csv", delimiter=",", skiprows=1, dtype=np.int64).reshape(-1, 2)
        tn, db, tpl, picks = learned(name, cycles)
        t, h3 = ss.world_horizontal_50hz(session=session(name))
        lu = lurches(t, np.column_stack([h3[:, 0], h3[:, 1], np.zeros(len(h3))]))
        opens = np.array(D.taps(name, "DOORS_OPEN"))
        n_taps += len(opens)
        n_cycles += len(cycles)

        def near(x):
            return len(opens) > 0 and np.min(np.abs(opens - x)) < 3e9

        base = [p for p in picks if p is not None]
        base_hits += sum(1 for a in opens if base and np.min(np.abs(np.array(base) - a)) < 3e9)
        base_extra += sum(1 for p in base if not near(p))
        anchored = []
        for (_, close), pick in zip(cycles, picks):
            before = lu[(lu > close - 60e9) & (lu < close - 8e9)]
            found = best_after(tn, db, tpl, before[-1], close) if len(before) else None
            anchored.append((found, pick))
        n_anchored += sum(1 for f, _ in anchored if f is not None)
        row = []
        for bar in BARS:
            chosen = np.array([f[0] if f is not None and f[1] >= bar else p for f, p in anchored
                               if (f is not None and f[1] >= bar) or p is not None])
            h = sum(1 for a in opens if len(chosen) and np.min(np.abs(chosen - a)) < 3e9)
            e = sum(1 for c in chosen if not near(c))
            hits[bar] += h
            extra[bar] += e
            row.append(f"bar {bar}: {h} hits, {e} untapped")
        print(f"{name}: {len(cycles)} cycles ({sum(1 for f, _ in anchored if f is not None)} after a lurch), "
              f"{len(opens)} open taps; " + "; ".join(row))
    print(f"all: {n_taps} open taps, {n_cycles} cycles, {n_anchored} after a lurch; "
          f"learned picks alone {base_hits} hits, {base_extra} untapped")
    for bar in BARS:
        print(f"  anchored, bar {bar}: {hits[bar]} hits, {extra[bar]} untapped")


if __name__ == "__main__":
    main()

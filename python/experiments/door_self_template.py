"""Openings from the sound alone, learned per ride: each Rust door cycle (`crates/fusion/examples/door_cycles.rs`,
in `target/doors/`) has a closing that's sure and an opening that isn't, since several steps up can come before one
closing. The doors' own opening sound repeats within a ride (`door_template.py`), so: average the onset spectrograms
at every candidate step up, pick each cycle's best match, re-average from the picks, and repeat.
Scores the picked openings against the rider's "doors open" taps (which only score), against the earliest and latest
candidates.

Result on all 9 subway rides with door cycles (2026-10-22), 60 "doors open" taps, within 3 s:
learned 30, any candidate 33, latest 17, earliest 7. Given a candidate at the opening, the learned sound picks it
30 times in 33; but the 1 to 4 kHz step up misses 27 of the 60 openings altogether. Not ported to `doors.rs`:
the closing alone sets the stop, and an opening at speed would zero the speed wrongly. Next: a better step-up finder.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import doors as D  # noqa: E402
import door_template as T  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
RIDES = ["135509", "154951", "184134", "214427", "155919", "182135", "215922", "151449", "151825"]


def candidates(t, lv, close):
    ups, _ = D.steps(t, lv["high 1-4 kHz"], 3.0, 2.0)
    return ups[(ups > close - 60e9) & (ups < close - 8e9)]


def cut(tn, db, at):
    """The spectrogram around the strongest onset within 1 s of `at`."""
    a0, a1 = int(T.SPAN[0] * T.RATE), int(T.SPAN[1] * T.RATE)
    i0, i1 = np.searchsorted(tn, [at - 1e9, at + 1e9])
    if i0 + a0 < 1 or i1 + a1 > db.shape[1]:
        return None
    flux = np.maximum(np.diff(db[:, i0 - 1 : i1], axis=1), 0).sum(0)
    i = i0 + int(np.argmax(flux))
    return db[:, i + a0 : i + a1]


def corr(a, b):
    a = (a - a.mean()) / a.std()
    b = (b - b.mean()) / b.std()
    return (a * b).mean()


def main():
    tot = {"earliest": 0, "latest": 0, "learned": 0, "any": 0}
    n_taps = 0
    for name in RIDES:
        cycles = np.loadtxt(ROOT / "target/doors" / f"{name}.csv", delimiter=",", skiprows=1, dtype=np.int64).reshape(-1, 2)
        t, lv = D.levels(name)
        tn, db = T.spectrogram(name)
        cands = []
        for _, close in cycles:
            cs = [(u, c) for u in candidates(t, lv, close) if (c := cut(tn, db, u)) is not None]
            cands.append(cs)
        tpl = np.mean([c for cs in cands for _, c in cs], axis=0)
        for _ in range(5):
            picks = [max(cs, key=lambda uc: corr(uc[1], tpl)) for cs in cands if cs]
            tpl = np.mean([c for _, c in picks], axis=0)
        picked = np.array([max(cs, key=lambda uc: corr(uc[1], tpl))[0] for cs in cands if cs])
        early = np.array([cs[0][0] for cs in cands if cs])
        late = np.array([cs[-1][0] for cs in cands if cs])
        opens = np.array(D.taps(name, "DOORS_OPEN"))
        n_taps += len(opens)

        def hits(x):
            return sum(1 for a in opens if len(x) and np.min(np.abs(x - a)) < 3e9)

        row = {"earliest": hits(early), "latest": hits(late), "learned": hits(picked),
               "any": hits(np.array([u for cs in cands for u, _ in cs]))}
        for k, v in row.items():
            tot[k] += v
        print(f"{name}: {len(cycles)} cycles, {len(opens)} open taps; within 3 s: " + ", ".join(f"{k} {v}" for k, v in row.items()))
    print(f"all: {n_taps} open taps; " + ", ".join(f"{k} {v}" for k, v in tot.items()))


if __name__ == "__main__":
    main()

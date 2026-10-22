"""Which car the rider is in, and where that puts them along the train and the platform, from the car beacons
(`train_beacons.py`: one iBeacon per car, the UUID's last two bytes its ID). Ready for the rider's "Car number" taps
(`KIND_CAR_NUMBER`, labels only): none exist yet, so the number column stays empty until they do.

- Consist: the beacons heard steadily (`STEADY_S` or more) between boarding and alighting; the rider's car is the
  one with the strongest median signal. Range is only 2 to 3 cars, so the far end of a long train is never heard.
- Ahead or behind: signal strength alone can't say which side a car is on. Evidence, in order of strength:
  1. The train pulling in past a rider standing still: the front cars pass first, so a car whose signal peaks
     before the rider's own car's (and fades) is ahead of it. This is the only one that works without taps.
  2. Walking along the platform before boarding: cars peak in walking order; the phone's heading against the
     train's after it departs says which way that is (needs the gyro-tracked heading, not done here).
  3. Each beacon's accelerometer sign while braking gives the car's facing, which flips at a set's ends.
  4. Car numbers, once tagged: a set's cars are numbered consecutively, so numbers fix the order within a set.
  5. Platform Wi-Fi radios near the ends of the platform.
- Offset: with the car's index from the front, its length (`CAR_LENGTH_M` by model), and the train stopping with
  its front at the platform's stop marker, the rider is (index + 0.5) car lengths behind the marker.

Tried on `184134` (the 7, then the M/R) and `135509` (the C at 110 St, 1:59 pm): every other car's peak near boarding
came while the rider was walking, so only an order comes out, never ahead or behind (evidence 1 needs the rider
standing as the train pulls in). On `135509`, `b3e6` peaked before the rider's own `9ba5` and faded while `9ba5`
rose to -35 dBm, as if it passed first, but steps were within 5 s. Next: evidence 2 (headings), and car number taps."""

import csv
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS, LOCAL  # noqa: E402
from sensor_trips import clock, walking  # noqa: E402

PREFIX = "e2c56db5dffb48d2b060d0f5a710"
# Apple's AirLocate default, some other beacon.
OTHER = {"96e0"}
STEADY_S = 60.0
STILL_S = 5.0
WALK_WINDOW_S = 4.0
# Car body lengths (m): A Division 51 ft, B Division 60 ft (75 ft for R46/R68).
CAR_LENGTH_M = {"R62": 15.6, "R62A": 15.6, "R142": 15.7, "R142A": 15.7, "R188": 15.7,
                "R143": 18.4, "R160": 18.4, "R179": 18.4, "R211": 18.4, "R46": 22.9, "R68": 22.9}


def sightings(ride):
    """{beacon ID: array of (t ns, dBm)}; 127 dBm is "unknown" and dropped."""
    out = defaultdict(list)
    with open(ANALYSIS / ride / "ble.csv") as f:
        for r in csv.DictReader(f):
            i = r["data"].find(PREFIX)
            rssi = int(r["rssi"])
            if i < 0 or rssi == 127:
                continue
            cid = r["data"][i + len(PREFIX) : i + len(PREFIX) + 4]
            if cid not in OTHER:
                out[cid].append((int(r["t_ns"]), rssi))
    return {k: np.array(v, float) for k, v in out.items()}


def events(ride, kind):
    with open(ANALYSIS / ride / "events.csv") as f:
        return [(int(r["t_ns"]), r["text"]) for r in csv.DictReader(f) if r["kind"] == f"KIND_{kind}"]


def consist(s, a, b):
    """Beacons steady between a and b (ns), strongest first: [(ID, median dBm, first heard ns)]."""
    out = []
    for cid, v in s.items():
        m = (v[:, 0] >= a) & (v[:, 0] <= b)
        if m.sum() >= 5 and (v[m, 0].max() - v[m, 0].min()) * 1e-9 >= STEADY_S:
            out.append((cid, float(np.median(v[m, 1])), float(v[m, 0].min())))
    return sorted(out, key=lambda c: -c[1])


def peak_time(v, a, b):
    m = (v[:, 0] >= a) & (v[:, 0] <= b)
    return float(v[m][np.argmax(v[m, 1]), 0]) if m.any() else None


def ahead_behind(s, cars, board_ns, walks):
    """Evidence 1 or 2 around boarding: each other car's peak in the 90 s before boarding (and 20 s after),
    against the rider's car's. A peak with no steps within `STILL_S` is the train passing a rider standing still
    (pull-in: earlier is ahead); otherwise it's the rider walking past (an order only, until headings say which way).
    Returns {ID: (verdict, kind)}."""
    a, b = board_ns - 90e9, board_ns + 20e9
    mine = cars[0][0]
    t_mine = peak_time(s[mine], a, b)
    out = {}
    for cid, _, _ in cars[1:]:
        t = peak_time(s[cid], a, b)
        if t is None or t_mine is None:
            continue
        before = t < t_mine
        still = not any(abs(w + WALK_WINDOW_S * 0.5e9 - t) <= STILL_S * 1e9 for w in walks)
        if still:
            out[cid] = ("ahead" if before else "behind", "pull-in")
        else:
            out[cid] = ("earlier" if before else "later", "walking")
    return out


def tags(ride, s):
    """Car number taps mapped to the strongest beacon in the 30 s around each: [(clock ns, number, ID)]."""
    out = []
    for t, text in events(ride, "CAR_NUMBER"):
        best = max(s.items(), key=lambda kv: np.median(kv[1][(abs(kv[1][:, 0] - t) < 15e9), 1])
                   if (abs(kv[1][:, 0] - t) < 15e9).any() else -999, default=None)
        out.append((t, text.strip('"'), best[0] if best else None))
    return out


def offset_m(index_from_front, model):
    """The rider's distance behind the train's front, so behind the stop marker when the train is stopped."""
    return (index_from_front + 0.5) * CAR_LENGTH_M[model]


def main(rides):
    for ride in rides:
        s = sightings(ride)
        clk = clock(ride)
        session = next(LOCAL.glob(f"*-{ride}"))
        walks = walking(session)
        boards = [t for t, _ in events(ride, "BOARDED")]
        alights = [t for t, _ in events(ride, "ALIGHTED")]
        print(f"== {ride}")
        for board in boards:
            end = min([t for t in alights if t > board], default=board + 3600e9)
            cars = consist(s, board - 90e9, end)
            if not cars:
                continue
            side = ahead_behind(s, cars, board, walks)
            print(f"  boarded {clk(board)}: rider's car {cars[0][0]} ({cars[0][1]:.0f} dBm)")
            for cid, med, first in cars[1:]:
                verdict, kind = side.get(cid, ("?", ""))
                print(f"    {cid} {med:.0f} dBm, first {clk(first)}: {verdict} ({kind})")
        for t, number, cid in tags(ride, s):
            print(f"  car number {number} at {clk(t)}: beacon {cid}")


if __name__ == "__main__":
    main(sys.argv[1:] or ["184134", "135509"])

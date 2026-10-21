"""Which signals tell the subway from the street: per 10 s window of each ride, features from the radio and GNSS,
against the rider's marks (on a train from "Boarded" to "Alighted"). The marks only score; nothing here uses them
as an input. `python subway.py ssids` lists the networks seen on and off trains instead."""

import collections
import csv
import re
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS  # noqa: E402

RIDES = ["151449", "154951", "184134", "214427", "135509"]
WINDOW_S = 10.0
# Marks known to be wrong, by ride and seconds from the first mark: what they should have been.
CORRECTIONS = {"135509": {1067.3: "BOARDED"}}
# Rides whose first train was boarded before the recording started (no "Boarded" mark).
BOARDED_AT_START = {"154951"}


def rows(ride, f):
    with open(ANALYSIS / ride / f) as fh:
        return list(csv.DictReader(fh))


def on_train_intervals(ride):
    """(start, end) in ns from "Boarded" to the next "Alighted"; unmatched marks are left out."""
    ev = rows(ride, "events.csv")
    t_first = int(ev[0]["t_ns"])
    fixes = CORRECTIONS.get(ride, {})
    marks = [(0, "BOARDED")] if ride in BOARDED_AT_START else []
    for e in ev:
        kind = e["kind"].removeprefix("KIND_")
        rel = round((int(e["t_ns"]) - t_first) * 1e-9, 1)
        kind = fixes.get(rel, kind)
        if kind in ("BOARDED", "ALIGHTED"):
            marks.append((int(e["t_ns"]), kind))
    out, start = [], None
    for t, kind in marks:
        if kind == "BOARDED":
            start = t
        elif start is not None:
            out.append((start, t))
            start = None
    return out


def windows(ride):
    """Window start times (ns) over the whole recording, and each one's label: 1 on a train, 0 off, None unknown.
    Off means outside every train interval and over 60 s from one, so walking to and from platforms is unlabeled."""
    times = [int(r["t_ns"]) for f in ("wifi.csv", "cell.csv", "fixes.csv") for r in rows(ride, f)]
    t0, t1 = min(times), max(times)
    starts = np.arange(t0, t1, WINDOW_S * 1e9).astype(np.int64)
    trains = on_train_intervals(ride)
    labels = []
    for s in starts:
        mid = s + WINDOW_S * 5e8
        if any(a <= mid <= b for a, b in trains):
            labels.append(1)
        elif all(mid < a - 60e9 or mid > b + 60e9 for a, b in trains):
            labels.append(0)
        else:
            labels.append(None)
    return starts, labels


def ssids():
    on, off = collections.Counter(), collections.Counter()
    for ride in RIDES:
        starts, labels = windows(ride)
        seen = collections.defaultdict(set)
        for r in rows(ride, "wifi.csv"):
            i = (int(r["t_ns"]) - starts[0]) // int(WINDOW_S * 1e9)
            if 0 <= i < len(starts):
                seen[i].add(r["ssid"].strip())
        for i, label in enumerate(labels):
            for s in seen.get(i, ()):
                (on if label == 1 else off if label == 0 else collections.Counter())[s] += 1
        print(f"{ride}: {sum(l == 1 for l in labels)} windows on trains, {sum(l == 0 for l in labels)} off")
    print("\nnetworks by on-train windows (on / off):")
    for s, n in on.most_common(25):
        print(f"  {n:5d} / {off[s]:5d}  {s!r}")


# The subway's Wi-Fi (Boldyn, formerly Transit Wireless) and the MTA's own; the street's (LinkNYC, Spectrum).
# `BoldynPasspoint` (not `PasspointBoldyn`) is outdoors: up to -31 dBm on the street, never seen on a train.
TRANSIT = re.compile(r"^(boldynwifi|passpointboldyn|ofmta.*|tw_survey.*|nycta|findsystem)$", re.I)
STREET = re.compile(r"linknyc|spectrum", re.I)


def features(ride):
    starts, labels = windows(ride)
    n = len(starts)
    w = int(WINDOW_S * 1e9)
    idx = lambda t: (int(t) - starts[0]) // w
    f = {k: np.zeros(n) for k in ["wifi_n", "transit_wifi", "street_wifi", "ble_n", "fix", "cell_dbm", "no_service"]}
    f["cell_dbm"][:] = np.nan
    ble = collections.defaultdict(set)
    for r in rows(ride, "wifi.csv"):
        i = idx(r["t_ns"])
        if 0 <= i < n:
            f["wifi_n"][i] += 1
            if TRANSIT.search(r["ssid"].strip()):
                f["transit_wifi"][i] = 1
            if STREET.search(r["ssid"]):
                f["street_wifi"][i] = 1
    for r in rows(ride, "ble.csv"):
        i = idx(r["t_ns"])
        if 0 <= i < n:
            ble[i].add(r["address"])
    for i, a in ble.items():
        f["ble_n"][i] = len(a)
    for r in rows(ride, "fixes.csv"):
        i = idx(r["t_ns"])
        if 0 <= i < n and r["h_acc"] and float(r["h_acc"]) < 50:
            f["fix"][i] = 1
    for r in rows(ride, "cell.csv"):
        i = idx(r["t_ns"])
        if 0 <= i < n and r["registered"] == "true" and r["dbm"]:
            f["cell_dbm"][i] = np.nanmax([f["cell_dbm"][i], float(r["dbm"])])
    for r in rows(ride, "service.csv"):
        i = idx(r["t_ns"])
        if 0 <= i < n and r["state"] != "0":
            f["no_service"][i] = 1
    return f, labels


def auc(x, y):
    """How often an on-train window scores above an off one (ties count half)."""
    pos, neg = x[y == 1], x[y == 0]
    pos, neg = pos[~np.isnan(pos)], neg[~np.isnan(neg)]
    if not len(pos) or not len(neg):
        return np.nan
    gt = (pos[:, None] > neg[None, :]).mean()
    eq = (pos[:, None] == neg[None, :]).mean()
    return gt + eq / 2


def main():
    if sys.argv[1:] == ["ssids"]:
        return ssids()
    per = {}
    for ride in RIDES:
        f, labels = features(ride)
        y = np.array([np.nan if l is None else l for l in labels])
        per[ride] = (f, y)
    names = list(next(iter(per.values()))[0])
    print("AUC on-train vs off (0.5 = no signal, 1 or 0 = perfect), per ride and all together:")
    print(f"{'feature':14s}" + "".join(f"{r:>9s}" for r in RIDES) + "      all")
    for k in names:
        vals = [auc(per[r][0][k], per[r][1]) for r in RIDES]
        allx = np.concatenate([per[r][0][k] for r in RIDES])
        ally = np.concatenate([per[r][1] for r in RIDES])
        print(f"{k:14s}" + "".join(f"{v:9.2f}" for v in vals) + f"{auc(allx, ally):9.2f}")
    print("\nmeans on / off:")
    for k in names:
        allx = np.concatenate([per[r][0][k] for r in RIDES])
        ally = np.concatenate([per[r][1] for r in RIDES])
        print(f"  {k:14s} {np.nanmean(allx[ally == 1]):8.2f} / {np.nanmean(allx[ally == 0]):8.2f}")




def system_rule():
    """In the subway system: subway Wi-Fi seen in the last 60 s, or no GNSS fix with few networks around.
    Scored on train windows (should be in) and on off-train windows with a good fix (street: should be out)."""
    for ride in RIDES:
        f, labels = features(ride)
        y = np.array([np.nan if l is None else l for l in labels])
        recent = np.convolve(f["transit_wifi"], np.ones(6), mode="full")[: len(y)] > 0
        rule = recent | ((f["fix"] == 0) & (f["wifi_n"] < 20))
        train = y == 1
        street = (y == 0) & (f["fix"] == 1)
        print(
            f"{ride}: in system on {rule[train].mean() * 100:5.1f}% of {train.sum()} train windows, "
            f"{rule[street].mean() * 100:5.1f}% of {street.sum()} street windows"
        )


main() if sys.argv[1:] != ["rule"] else system_rule()

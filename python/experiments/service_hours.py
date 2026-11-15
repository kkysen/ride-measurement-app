"""When each line runs, from the MTA's static GTFS (`data/mta/gtfs`): for each line and day of the week,
the hours (in the agency's zone, `agency.txt`'s `agency_timezone`) any of its trips is at a stop. A trip past midnight (`25:10:00`) counts on the next day.
Only the regular `Weekday`, `Saturday` and `Sunday` services, not the dated holiday ones.
Lines as the app names them: express variants (`6X`) are their line, `GS`/`FS`/`H` the S, `SI` the SIR.

Writes `crates/fusion/data/service_hours.csv`: line, then one 24-bit mask of hours per day, Monday first;
and `crates/fusion/data/service_zone.txt`: the zone those hours are in, an IANA id."""

import csv
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GTFS = ROOT / "data/mta/gtfs"
OUT = ROOT / "crates/fusion/data/service_hours.csv"
ZONE_OUT = ROOT / "crates/fusion/data/service_zone.txt"
DAYS = {"Weekday": [0, 1, 2, 3, 4], "Saturday": [5], "Sunday": [6]}


def line(route: str) -> str:
    if route in ("GS", "FS", "H"):
        return "S"
    if route == "SI":
        return "SIR"
    return route[:-1] if route.endswith("X") and len(route) > 1 else route


def main() -> None:
    # GTFS requires one zone for all of a feed's agencies, and gives every schedule time in it.
    with open(GTFS / "agency.txt") as f:
        zones = {r["agency_timezone"] for r in csv.DictReader(f)}
    if len(zones) != 1:
        raise SystemExit(f"expected one agency_timezone, got {sorted(zones)}")
    trips = {}
    with open(GTFS / "trips.txt") as f:
        for r in csv.DictReader(f):
            if r["service_id"] in DAYS:
                trips[r["trip_id"]] = (line(r["route_id"]), DAYS[r["service_id"]])
    masks: dict[str, list[int]] = defaultdict(lambda: [0] * 7)
    with open(GTFS / "stop_times.txt") as f:
        for r in csv.DictReader(f):
            trip = trips.get(r["trip_id"])
            if trip is None:
                continue
            name, days = trip
            hour = int(r["arrival_time"].split(":")[0])
            for d in days:
                masks[name][(d + hour // 24) % 7] |= 1 << (hour % 24)
    with open(OUT, "w") as f:
        f.write("line,mon,tue,wed,thu,fri,sat,sun\n")
        for name in sorted(masks):
            f.write(",".join([name, *(str(m) for m in masks[name])]) + "\n")
    ZONE_OUT.write_text(zones.pop() + "\n")


if __name__ == "__main__":
    main()

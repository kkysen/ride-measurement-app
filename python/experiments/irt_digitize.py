"""Reads the IRT Capacity Study's event recorder charts (Figures F.2-1 to F.2-28,
`data/downloads/speed-capacity-docs-20261108/charts/f-NNN.png`): the speed scale from the gridlines (0 to 50 mph),
the stations where the simulated (black) line touches 0, and per column the highest event recorder (blue, orange)
speed. Prints, per chart, each stretch between stations' highest recorded speed."""
import sys
import numpy as np
from PIL import Image

def load(path):
    return np.asarray(Image.open(path).convert("RGB")).astype(int)

def masks(a):
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    blue = (b > 150) & (b - r > 60) & (b - g > 20)
    orange = (r > 200) & (g > 90) & (g < 200) & (b < 120) & (r - b > 100)
    black = (r < 60) & (g < 60) & (b < 60)
    gray = (abs(r - g) < 6) & (abs(g - b) < 6) & (r > 200) & (r < 235)
    return blue, orange, black, gray

def gridlines(gray):
    # Rows mostly gray across the plot: the 0, 5, ..., 50 mph lines.
    w = gray.shape[1]
    rows = [y for y in range(gray.shape[0]) if gray[y].sum() > 0.5 * w]
    # Merge neighbors.
    out = []
    for y in rows:
        if out and y - out[-1][-1] <= 2:
            out[-1].append(y)
        else:
            out.append([y])
    return [sum(g) / len(g) for g in out]

def top_speed(mask, y50, y0):
    """Per column, the speed (mph) of the highest pixel of `mask` inside the plot, or NaN."""
    px = (y0 - y50) / 50.0
    sub = mask[int(y50) - 3 : int(y0) + 2]
    out = np.full(mask.shape[1], np.nan)
    for x in range(mask.shape[1]):
        ys = np.nonzero(sub[:, x])[0]
        if len(ys):
            out[x] = max(0.0, (y0 - (ys[0] + int(y50) - 3)) / px)
    return out

def stations(black, y0, x0, x1):
    """Each station's x: where the simulated (black) line comes down to 0, its near-vertical strokes reaching
    just above the 0 mph line (columns 15 px apart at least are separate stations)."""
    y = round(y0)
    near = black[y - 8 : y - 1, :].any(axis=0)
    xs = [x for x in range(x0, x1) if near[x]]
    groups = []
    for x in xs:
        if groups and x - groups[-1][-1] <= 15:
            groups[-1].append(x)
        else:
            groups.append([x])
    return [sum(gr) / len(gr) for gr in groups]

def read(path):
    """The stations' x and each stretch's (recorded, simulated) peak mph. Two chart styles: two event recorder runs
    (blue, orange) with the simulation in black; or several cars' runs, each its own color, with the simulations
    in yellow and a thick light blue (the updated one, where the stations are found)."""
    a = load(path)
    r, g_, b = a[..., 0], a[..., 1], a[..., 2]
    blue, orange, black, gray = masks(a)
    cyan = (b > 200) & (g_ > 150) & (r < 100)
    yellow = (r > 220) & (g_ > 150) & (b < 90)
    g = gridlines(gray)
    lines = [y for y in g if 20 < y < a.shape[0] - 20]
    y50, y0 = lines[0], lines[-1]
    rows = [round(y) for y in lines]
    span = np.nonzero(gray[rows].any(axis=0))[0]
    x0, x1 = span.min(), span.max()
    inside = np.zeros_like(black)
    inside[int(y50) - 3 : int(y0) + 2, x0 : x1 + 1] = True
    spread = a.max(axis=2) - a.min(axis=2)
    colored = (spread > 30) & ~yellow & inside
    # The simulation: black where there's a black line in the plot, else the thick light blue.
    if (black & inside).sum() > 2000:
        sim_mask = black & inside
        recorded = colored & ~black
    else:
        sim_mask = cyan & inside
        recorded = colored & ~cyan
    sim = top_speed(sim_mask, y50, y0)
    rec = top_speed(recorded, y50, y0)
    st = stations(sim_mask, y0, x0, x1 + 1)
    return st, rec, sim


def peak(speeds, a, b):
    """The highest speed (mph) between two stations' columns, or None."""
    seg = speeds[int(a) + 3 : int(b) - 2]
    return round(float(np.nanmax(seg)), 1) if np.isfinite(seg).any() else None


def main(path):
    st, rec, sim = read(path)
    print(path.split("/")[-1], len(st), "stations")
    print("  recorded/simulated peaks:", [(peak(rec, a, b), peak(sim, a, b)) for a, b in zip(st, st[1:])])

for p in sys.argv[1:]:
    main(p)

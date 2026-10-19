"""Plot every value of a stream against time: `python -m ride.plot <session dir> <stream> [output.png]`."""

import sys

import matplotlib

from ride import read_stream


def main() -> None:
    session, stream, *out = sys.argv[1:]
    if out:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    s = read_stream(session, stream)
    t = (s.timestamps_ns - s.timestamps_ns[0]) * 1e-9
    fig, ax = plt.subplots(figsize=(12, 4))
    for i in range(s.values.shape[1]):
        ax.plot(t, s.values[:, i], lw=0.5, label=f"value[{i}]")
    ax.set_xlabel("time (s)")
    ax.set_title(f"{stream}{' (truncated)' if s.truncated else ''}")
    ax.legend()
    if out:
        fig.savefig(out[0], dpi=150)
    else:
        plt.show()


if __name__ == "__main__":
    main()

"""Gallery of trackdrive circuits for docs/sim.md: cones, centerline and staging position.

    ./build/release/bench/sim/apex_export_tracks build/tracks
    uv run --project tools python tools/sim/plot_tracks.py build/tracks docs/img/sim-tracks.png
"""

import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

CONE_COLORS = {
    "blue": "#1f5fd1",
    "yellow": "#e3b505",
    "orange": "#f07c13",
    "big_orange": "#c94f00",
}
ORDER = ["technical", "flowing", "mixed"] + [f"generated-{seed}" for seed in range(1, 7)]


def length(centerline):
    points = centerline["points"]
    total = 0.0
    for a, b in zip(points, points[1:] + points[:1]):
        total += ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
    return total


def draw(ax, track):
    rows = track["centerline"]["points"]
    xs = [r[0] for r in rows] + [rows[0][0]]
    ys = [r[1] for r in rows] + [rows[0][1]]
    ax.plot(xs, ys, color="#9a9a9a", linewidth=0.6, linestyle="--")
    for color, hex_color in CONE_COLORS.items():
        cones = [c["position"] for c in track["cones"] if c["color"] == color]
        if cones:
            size = 9 if color == "big_orange" else 3
            ax.scatter([p[0] for p in cones], [p[1] for p in cones], s=size, color=hex_color)
    start = track["start_pose"]
    ax.annotate(
        "",
        xy=(start["x"] + 8.0 * math.cos(start["yaw"]), start["y"] + 8.0 * math.sin(start["yaw"])),
        xytext=(start["x"], start["y"]),
        arrowprops={"arrowstyle": "->", "color": "black", "linewidth": 1.0},
    )
    ax.set_title(f"{track['name']} ({length(track['centerline']):.0f} m)", fontsize=9)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: plot_tracks.py <track directory> <output.png>")
    directory, output = Path(sys.argv[1]), Path(sys.argv[2])
    tracks = [json.loads((directory / f"{name}.json").read_text()) for name in ORDER]
    fig, axes = plt.subplots(3, 3, figsize=(10, 10))
    for ax, track in zip(axes.flat, tracks):
        draw(ax, track)
    fig.suptitle("Reference layouts (top row) and generated tracks, seeds 1-6", fontsize=11)
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=110)


if __name__ == "__main__":
    main()

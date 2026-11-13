"""Figures for docs/planning.md, from the examples the planning evaluations export.

    ./build/release/bench/planning/apex_planning_raceline_eval \
        --out results/planning/raceline.json --example results/planning/raceline_example.json
    ./build/release/bench/planning/apex_planning_exploration_eval \
        --out results/planning/exploration.json \
        --example results/planning/exploration_example.json
    uv run --project tools python tools/planning/plot_planning.py results/planning docs/img

writes docs/img/planning-raceline.png (track model and raceline of generated-1, colored by
speed) and docs/img/planning-exploration.png (one exploration plan in a hairpin of a worst-case
map). To look at any plan of the evaluation:

    ./build/release/bench/planning/apex_planning_exploration_eval --plan 9 50 nominal plan.json
    uv run --project tools python tools/planning/plot_planning.py --plan plan.json plan.png
"""

import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

CONE_COLORS = {
    "blue": "#1f5fd1",
    "yellow": "#e3b505",
    "orange": "#f07c13",
    "big_orange": "#c94f00",
    "unknown": "#8a8a8a",
}


def closed(points):
    return [p[0] for p in points] + [points[0][0]], [p[1] for p in points] + [points[0][1]]


def speed_line(ax, rows, vmin, vmax, closed_line):
    xs = [r[0] for r in rows] + ([rows[0][0]] if closed_line else [])
    ys = [r[1] for r in rows] + ([rows[0][1]] if closed_line else [])
    speeds = [r[2] for r in rows] + ([rows[0][2]] if closed_line else [])
    segments = [[(xs[i], ys[i]), (xs[i + 1], ys[i + 1])] for i in range(len(xs) - 1)]
    lines = LineCollection(segments, cmap="viridis", linewidths=2.2)
    lines.set_array([0.5 * (speeds[i] + speeds[i + 1]) for i in range(len(xs) - 1)])
    lines.set_clim(vmin, vmax)
    ax.add_collection(lines)
    return lines


def tidy(ax):
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)


def plot_raceline(example, out):
    fig, (left, right) = plt.subplots(1, 2, figsize=(12, 5.6))
    speeds = [r[2] for r in example["raceline"] + example["centerline"]]
    vmin, vmax = min(speeds), max(speeds)
    panels = (
        (left, "centerline", f"Centerline: {example['centerline_lap_s']:.2f} s"),
        (right, "raceline", f"Raceline: {example['raceline_lap_s']:.2f} s"),
    )
    for ax, key, title in panels:
        for line in ("true_left", "true_right"):
            ax.plot(*closed(example[line]), color="#c8c8c8", linewidth=0.8)
        for line, color in (("model_left", CONE_COLORS["blue"]), ("model_right", "#d9a400")):
            xs, ys = closed(example[line])
            ax.plot(xs, ys, color=color, linewidth=0.5)
            ax.scatter(xs, ys, s=2, color=color)
        lines = speed_line(ax, example[key], vmin, vmax, True)
        ax.set_title(title, fontsize=10)
        tidy(ax)
    bar = fig.colorbar(lines, ax=[left, right], shrink=0.75, pad=0.02)
    bar.set_label("planned speed (m/s)")
    fig.suptitle(
        f"{example['track']}: track model (cone lines) from an imperfect map, "
        "centerline vs minimum-curvature raceline",
        fontsize=10,
    )
    fig.savefig(out, dpi=150, bbox_inches="tight")


def plot_exploration(example, out):
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    for line in ("left", "right"):
        rows = example[line]
        ax.plot([p[0] for p in rows], [p[1] for p in rows], color="#c8c8c8", linewidth=0.8)
    for x, y, color in example["cones"]:
        ax.scatter([x], [y], s=16, color=CONE_COLORS.get(color, "#8a8a8a"), zorder=3)
    path = example["path"]
    if path:
        lines = speed_line(ax, path, 0.0, max(r[2] for r in path), False)
        bar = fig.colorbar(lines, ax=ax, shrink=0.7, pad=0.02)
        bar.set_label("planned speed (m/s)")
    car_x, car_y, yaw = example["car"]
    ax.annotate(
        "",
        xy=(car_x + 3.0 * math.cos(yaw), car_y + 3.0 * math.sin(yaw)),
        xytext=(car_x, car_y),
        arrowprops={"arrowstyle": "->", "color": "black", "linewidth": 1.2},
    )
    ax.set_xlim(car_x - 22, car_x + 22)
    ax.set_ylim(car_y - 22, car_y + 22)
    ax.set_title(
        "Exploration plan from a worst-case map (gray: true cone lines)", fontsize=10
    )
    tidy(ax)
    fig.savefig(out, dpi=150, bbox_inches="tight")


def main():
    if len(sys.argv) == 4 and sys.argv[1] == "--plan":
        plot_exploration(json.loads(Path(sys.argv[2]).read_text()), sys.argv[3])
        return
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    results, images = Path(sys.argv[1]), Path(sys.argv[2])
    images.mkdir(parents=True, exist_ok=True)
    plot_raceline(
        json.loads((results / "raceline_example.json").read_text()),
        images / "planning-raceline.png",
    )
    plot_exploration(
        json.loads((results / "exploration_example.json").read_text()),
        images / "planning-exploration.png",
    )


if __name__ == "__main__":
    main()

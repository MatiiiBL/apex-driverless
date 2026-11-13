"""Draw a run recorded by `apex replay`: the track, the map the car built, its path.

    uv run --project tools python tools/plot_replay.py replay.json --out docs/img/evaluation-replay.png
    uv run --project tools python tools/plot_replay.py replay.json --gif docs/img/replay.gif

The static figure shows the true cones, the landmarks of the map as it was frozen after lap 1,
and the true path of the whole run colored by speed. The animation replays lap 1 and the first
racing lap: the car, the map growing as it is built, and the reference it tracks.
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"
# Physical cone colors (FS rules): they name the cones, not data series.
CONE_COLORS = {
    "blue": "#2a78d6",
    "yellow": "#eda100",
    "orange": "#eb6834",
    "big_orange": "#eb6834",
    "unknown": "#8e8d88",
}
# One-hue sequential ramp (light to dark) for speed.
SPEED_MAP = LinearSegmentedColormap.from_list(
    "speed", ["#b7d3f6", "#6da7ec", "#2a78d6", "#1c5cab", "#0d366b"]
)


def style_axes(ax):
    ax.set_facecolor(SURFACE)
    ax.set_aspect("equal")
    ax.tick_params(colors=TEXT_SECONDARY, labelsize=8)
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.grid(color=GRID, linewidth=0.6)
    ax.set_xlabel("x (m)", color=TEXT_SECONDARY, fontsize=9)
    ax.set_ylabel("y (m)", color=TEXT_SECONDARY, fontsize=9)


def cone_arrays(cones):
    xy = np.array([[c[0], c[1]] for c in cones]) if cones else np.zeros((0, 2))
    colors = [CONE_COLORS.get(c[2], CONE_COLORS["unknown"]) for c in cones]
    return xy, colors


def frozen_map(replay):
    """The map as it was when the mission stopped building it, else the last one."""
    maps = replay["maps"]
    if not maps:
        return []
    return maps[-1]["cones"]


def figure_size(points, width=8.0):
    """A figure as wide as `width` inches whose plot keeps the track's aspect ratio."""
    span = points.max(axis=0) - points.min(axis=0)
    return width, min(max(width * span[1] / span[0] + 1.6, 3.5), 9.0)


def static_figure(replay, out):
    frames = replay["frames"]
    xy = np.array([[f["x"], f["y"]] for f in frames])
    speed = np.array([f["speed"] for f in frames])
    true_xy, true_colors = cone_arrays(replay["track"]["cones"])
    fig, ax = plt.subplots(figsize=figure_size(true_xy), facecolor=SURFACE)
    style_axes(ax)

    ax.scatter(true_xy[:, 0], true_xy[:, 1], s=16, c=true_colors, linewidths=0, zorder=2)
    map_xy, map_colors = cone_arrays(frozen_map(replay))
    if len(map_xy):
        ax.scatter(map_xy[:, 0], map_xy[:, 1], s=46, facecolors="none", edgecolors=map_colors,
                   linewidths=1.0, zorder=3)

    segments = np.stack([xy[:-1], xy[1:]], axis=1)
    path = LineCollection(segments, cmap=SPEED_MAP, linewidths=2.0, zorder=4)
    path.set_array(0.5 * (speed[:-1] + speed[1:]))
    ax.add_collection(path)
    bar = fig.colorbar(path, ax=ax, shrink=0.7, pad=0.02)
    bar.set_label("true speed (m/s)", color=TEXT_SECONDARY, fontsize=9)
    bar.ax.tick_params(colors=TEXT_SECONDARY, labelsize=8)
    bar.outline.set_visible(False)

    name = replay["track"].get("name") or "track"
    ax.set_title(f"{name}: the map after lap 1 and the path of the whole run", color=TEXT,
                 fontsize=11, loc="left")
    handles = [
        Line2D([], [], marker="o", linestyle="", markersize=4, markeredgewidth=0,
               markerfacecolor=TEXT_SECONDARY, label="true cones (filled, in their colors)"),
        Line2D([], [], marker="o", linestyle="", markersize=7, markerfacecolor="none",
               markeredgecolor=TEXT_SECONDARY, label="mapped landmarks (rings)"),
    ]
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=2,
              fontsize=8, frameon=False, labelcolor=TEXT_SECONDARY)
    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def first_racing_lap_end(replay):
    """Time at which the animation stops: the end of the first racing lap, or the run's end."""
    states = [(f["t"], f["state"]) for f in replay["frames"]]
    racing = [t for t, s in states if s == "racing"]
    if not racing:
        return states[-1][0]
    result = replay.get("result", {})
    laps = result.get("laps", [])
    if len(laps) >= 2:
        return racing[0] + laps[1] + 1.0
    return states[-1][0]


def animation(replay, out, fps, speedup, dpi):
    frames = [f for f in replay["frames"] if f["t"] <= first_racing_lap_end(replay)]
    step = max(1, round(speedup / (fps * 0.05)))  # recorded frames are 50 ms apart
    frames = frames[::step]
    maps = replay["maps"]
    references = replay["references"]

    true_xy, _ = cone_arrays(replay["track"]["cones"])
    fig, ax = plt.subplots(figsize=figure_size(true_xy, 6.4), facecolor=SURFACE)
    style_axes(ax)
    ax.scatter(true_xy[:, 0], true_xy[:, 1], s=10, c="#c3c2b7", linewidths=0, zorder=1)
    pad = 5.0
    ax.set_xlim(true_xy[:, 0].min() - pad, true_xy[:, 0].max() + pad)
    ax.set_ylim(true_xy[:, 1].min() - pad, true_xy[:, 1].max() + pad)
    landmarks = ax.scatter([], [], s=18, zorder=2)
    (reference_line,) = ax.plot([], [], color="#1baf7a", linewidth=2.0, zorder=3)
    (trail,) = ax.plot([], [], color="#2a78d6", linewidth=1.2, alpha=0.6, zorder=3)
    (car,) = ax.plot([], [], marker="o", markersize=7, color="#0b0b0b", zorder=4)
    handles = [
        Line2D([], [], marker="o", linestyle="", markersize=4, markeredgewidth=0,
               markerfacecolor="#c3c2b7", label="true cones"),
        Line2D([], [], marker="o", linestyle="", markersize=4, markeredgewidth=0,
               markerfacecolor=TEXT_SECONDARY, label="the car's map, in cone colors"),
        Line2D([], [], color="#1baf7a", linewidth=2.0, label="path it tracks"),
        Line2D([], [], color="#2a78d6", linewidth=1.2, label="path it drove"),
    ]
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=4,
              fontsize=7, frameon=False, labelcolor=TEXT_SECONDARY)
    fig.tight_layout()
    label = ax.text(0.02, 0.97, "", transform=ax.transAxes, va="top", color=TEXT, fontsize=9)
    trail_xy = np.array([[f["x"], f["y"]] for f in frames])

    def latest(items, t):
        chosen = None
        for item in items:
            if item["t"] > t:
                break
            chosen = item
        return chosen

    def draw(i):
        f = frames[i]
        snapshot = latest(maps, f["t"])
        if snapshot and snapshot["cones"]:
            xy, colors = cone_arrays(snapshot["cones"])
            landmarks.set_offsets(xy)
            landmarks.set_color(colors)
        reference = latest(references, f["t"])
        if reference:
            pts = np.array(reference["points"])
            reference_line.set_data(pts[:, 0], pts[:, 1])
        trail.set_data(trail_xy[: i + 1, 0], trail_xy[: i + 1, 1])
        car.set_data([f["x"]], [f["y"]])
        label.set_text(f"t = {f['t']:5.1f} s   {f['state'].replace('_', ' ')}   "
                       f"{f['speed']:4.1f} m/s")
        return landmarks, reference_line, trail, car, label

    anim = FuncAnimation(fig, draw, frames=len(frames), blit=False)
    anim.save(out, writer=PillowWriter(fps=fps), dpi=dpi)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("replay", type=Path, help="a replay.json from `apex replay`")
    parser.add_argument("--out", type=Path, help="static PNG to write")
    parser.add_argument("--gif", type=Path, help="animated GIF to write")
    parser.add_argument("--fps", type=int, default=10, help="GIF frames per second")
    parser.add_argument("--speedup", type=float, default=1.0,
                        help="seconds of simulated time per second of animation")
    parser.add_argument("--dpi", type=int, default=80, help="GIF resolution")
    args = parser.parse_args()
    if not args.out and not args.gif:
        parser.error("give --out, --gif or both")
    replay = json.loads(args.replay.read_text())
    if args.out:
        static_figure(replay, args.out)
    if args.gif:
        animation(replay, args.gif, args.fps, args.speedup, args.dpi)


if __name__ == "__main__":
    main()

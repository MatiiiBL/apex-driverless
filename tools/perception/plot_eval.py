"""Plots the perception evaluation against range: recall, position error and NEES.

Reads results/perception/eval.json (written by apex_perception_eval) and writes
docs/img/perception-range.png. From the repository root:

    uv run --project tools python tools/perception/plot_eval.py
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"
SERIES = ["#2a78d6", "#eb6834"]  # categorical slots 1 and 2, in that order


def bin_centers(bins):
    centers = []
    for b in bins:
        low, _, high = b["range_m"].partition("-")
        centers.append(0.5 * (float(low) + float(high)) if high else float(low.rstrip("+")))
    return centers


def style(ax, title, ylabel):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=TEXT, fontsize=11, loc="left")
    ax.set_xlabel("Range from the LiDAR (m)", color=TEXT_SECONDARY, fontsize=9)
    ax.set_ylabel(ylabel, color=TEXT_SECONDARY, fontsize=9)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(colors=TEXT_SECONDARY, labelsize=8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.set_xlim(0, 40)


def line(ax, x, y, slot, label):
    ax.plot(x, y, color=SERIES[slot], linewidth=2, marker="o", markersize=4, label=label)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--eval", default="results/perception/eval.json", type=Path)
    parser.add_argument("--out", default="docs/img/perception-range.png", type=Path)
    args = parser.parse_args()

    result = json.loads(args.eval.read_text())
    # Bins up to 40 m, the clustering range; nothing is detected beyond it.
    bins = [b for b in result["by_range_2m"] if b["in_view"] > 0][:20]
    x = bin_centers(bins)
    detected = [b for b in bins if b["detected"] > 0]
    xd = bin_centers(detected)

    fig, axes = plt.subplots(1, 3, figsize=(12, 3.6), facecolor=SURFACE)
    recall, error, nees = axes

    line(recall, x, [b["recall"] for b in bins], 0, "of cones in the field of view")
    line(recall, x, [b["recall_of_hit"] for b in bins], 1, "of cones with a return")
    recall.set_ylim(0, 1.05)
    style(recall, "Detection recall", "Recall")
    recall.legend(frameon=False, fontsize=8, labelcolor=TEXT, loc="lower left")

    line(error, xd, [100 * b["rms_radial_m"] for b in detected], 0, "radial")
    line(error, xd, [100 * b["rms_tangential_m"] for b in detected], 1, "tangential")
    error.set_ylim(bottom=0)
    style(error, "Center error, RMS", "cm")
    error.legend(frameon=False, fontsize=8, labelcolor=TEXT, loc="upper left")

    nees.axhline(2.0, color=TEXT_SECONDARY, linewidth=1, linestyle="--")
    line(nees, xd, [b["nees"] for b in detected], 0, "NEES")
    top = max(5.0, max(b["nees"] for b in detected) * 1.1)
    nees.set_ylim(0, top)
    nees.text(39.5, 0.94 * top, "dashed: expected 2", color=TEXT_SECONDARY, fontsize=8, ha="right")
    style(nees, "Mean NEES (2 dof)", "NEES")

    scans = result["scans"]
    tracks = len(result["tracks"])
    fig.suptitle(
        f"Perception against range: {scans} scans along {tracks} generated tracks",
        color=TEXT,
        fontsize=12,
        x=0.01,
        ha="left",
    )
    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150, facecolor=SURFACE)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()

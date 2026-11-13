"""Figures for docs/control.md from the control benchmarks' results.

    uv run --project tools python tools/control/plot.py [results/control] [docs/img]

Reads qp_bench.json and the trace_<controller>.csv files written by the benchmarks
(bench/control/) and writes control-qp-time.png and control-tracking.png.
"""

import csv
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
MUTED = "#52514e"
GRID = "#e4e3df"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]  # categorical slots 1-3
CONTROLLERS = [("pure_pursuit", "Pure pursuit"), ("stanley", "Stanley"), ("mpc", "MPC")]


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.xaxis.label.set_color(TEXT)
    ax.yaxis.label.set_color(TEXT)


def plot_qp(results: Path, out: Path):
    data = json.loads((results / "qp_bench.json").read_text())
    horizons = [h["horizon"] for h in data["horizons"]]
    series = [
        ("ours_warm", "Riccati IPM, warm", SERIES[0]),
        ("osqp_warm", "OSQP, warm (setup + solve)", SERIES[1]),
    ]
    fig, ax = plt.subplots(figsize=(6.4, 3.8), facecolor=SURFACE)
    style(ax)
    for key, label, color in series:
        p50 = [h[key]["p50_us"] / 1000.0 for h in data["horizons"]]
        p99 = [h[key]["p99_us"] / 1000.0 for h in data["horizons"]]
        ax.plot(horizons, p50, color=color, linewidth=2, marker="o", markersize=7,
                markeredgecolor=SURFACE, markeredgewidth=2, label=f"{label}, p50")
        ax.plot(horizons, p99, color=color, linewidth=2, linestyle=(0, (4, 3)), marker="o",
                markersize=7, markerfacecolor=SURFACE, markeredgewidth=2, label=f"{label}, p99")
    ax.set_yscale("log")
    ax.set_xlabel("horizon N (stages of 50 ms)")
    ax.set_ylabel("time per QP (ms)")
    ax.set_xticks(horizons)
    ax.set_title("MPC QPs at tolerance 1e-6: solve time per QP", color=TEXT, fontsize=11,
                 loc="left")
    legend = ax.legend(frameon=False, fontsize=8, labelcolor=TEXT, handlelength=4.0)
    legend.set_title(None)
    fig.tight_layout()
    fig.savefig(out / "control-qp-time.png", dpi=150, facecolor=SURFACE)
    plt.close(fig)


def read_trace(path: Path):
    with path.open() as f:
        rows = list(csv.DictReader(f))
    return [float(r["time"]) for r in rows], [float(r["lateral"]) for r in rows]


def plot_tracking(results: Path, out: Path):
    traces = []
    for key, label in CONTROLLERS:
        path = results / f"trace_{key}.csv"
        if path.exists():
            traces.append((label, *read_trace(path)))
    fig, axes = plt.subplots(len(traces), 1, figsize=(6.4, 1.6 * len(traces) + 0.8),
                             sharex=True, sharey=True, facecolor=SURFACE)
    for ax, (label, time, lateral), color in zip(axes, traces, SERIES):
        style(ax)
        ax.plot(time, lateral, color=color, linewidth=1.5)
        ax.axhline(0.0, color=MUTED, linewidth=0.8)
        ax.text(0.01, 0.92, label, transform=ax.transAxes, color=TEXT, fontsize=9,
                va="top")
        ax.set_ylabel("lateral (m)", fontsize=9)
    axes[-1].set_xlabel("time (s), two laps of the 'mixed' layout from a standing start")
    axes[0].set_title("Lateral error from the centerline at 85% of the grip limit", color=TEXT,
                      fontsize=11, loc="left")
    fig.tight_layout()
    fig.savefig(out / "control-tracking.png", dpi=150, facecolor=SURFACE)
    plt.close(fig)


def main():
    results = Path(sys.argv[1] if len(sys.argv) > 1 else "results/control")
    out = Path(sys.argv[2] if len(sys.argv) > 2 else "docs/img")
    out.mkdir(parents=True, exist_ok=True)
    plot_qp(results, out)
    plot_tracking(results, out)


if __name__ == "__main__":
    main()

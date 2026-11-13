"""Figures of the evaluation (docs/evaluation.md) from the sweeps in results/runs/.

    uv run --project tools python tools/plot_evaluation.py --results results/runs --out docs/img

evaluation-laps.png       mean speed per lap of the baseline, the stack against the oracles
evaluation-ablations.png  racing speed and completion for each ablation's variants
evaluation-noise.png      map and localization error against the sensor-noise scale

Speeds (track length over lap time) make laps of tracks of different lengths comparable. A
figure whose sweep has not been run is skipped.
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]  # categorical slots 1-3, in order

ABLATIONS = [
    ("controllers", "Controller"),
    ("oracles", "Oracles"),
    ("noise", "Sensor noise scale"),
    ("latency", "Latency"),
    ("friction", "Track grip (planner unchanged)"),
]


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.tick_params(colors=TEXT_SECONDARY, labelsize=8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.grid(axis="y", color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def load_runs(results, experiment):
    path = results / experiment / "runs.json"
    return json.loads(path.read_text()) if path.exists() else None


def variants_in_order(runs):
    order = []
    for run in runs:
        if run["variant"] not in order:
            order.append(run["variant"])
    return order


def lap_speeds(run):
    result = run["result"]
    length = result["track"]["length"]
    return [length / t for t in result["laps"] if t > 0]


def plot_laps(runs, out):
    fig, ax = plt.subplots(figsize=(7, 4), facecolor=SURFACE)
    style(ax)
    for color, variant in zip(SERIES, variants_in_order(runs)):
        per_lap = {}
        for run in runs:
            if run["variant"] != variant or not run["result"]["completed"]:
                continue
            for lap, speed in enumerate(lap_speeds(run), start=1):
                per_lap.setdefault(lap, []).append(speed)
        if not per_lap:
            continue
        laps = sorted(per_lap)
        median = [np.median(per_lap[k]) for k in laps]
        lo = [np.percentile(per_lap[k], 25) for k in laps]
        hi = [np.percentile(per_lap[k], 75) for k in laps]
        ax.fill_between(laps, lo, hi, color=color, alpha=0.15, linewidth=0)
        ax.plot(laps, median, color=color, linewidth=2, marker="o", markersize=5, label=variant)
    ax.set_xlabel("lap", color=TEXT_SECONDARY, fontsize=9)
    ax.set_ylabel("mean speed (m/s)", color=TEXT_SECONDARY, fontsize=9)
    ax.set_xticks(range(1, 11))
    ax.axvspan(0.5, 1.5, color=GRID, alpha=0.5, linewidth=0)
    ax.text(1.0, ax.get_ylim()[0], "exploring", ha="center", va="bottom", fontsize=8,
            color=TEXT_SECONDARY)
    ax.set_title("Lap 1 explores the unknown track; laps 2-10 race the raceline\n"
                 "median over completed runs, band: 25th-75th percentile",
                 color=TEXT, fontsize=10, loc="left")
    ax.legend(fontsize=8, frameon=False, labelcolor=TEXT_SECONDARY, loc="lower right")
    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def racing_speed(run):
    speeds = lap_speeds(run)[1:]
    return float(np.mean(speeds)) if speeds else None


def plot_ablations(results, out):
    present = [(name, title, load_runs(results, name)) for name, title in ABLATIONS]
    present = [p for p in present if p[2]]
    if not present:
        return False
    fig, axes = plt.subplots(1, len(present), figsize=(3.2 * len(present), 3.6),
                             facecolor=SURFACE, sharey=True, squeeze=False)
    for ax, (_, title, runs) in zip(axes[0], present):
        style(ax)
        names = variants_in_order(runs)
        means, labels = [], []
        for variant in names:
            mine = [r for r in runs if r["variant"] == variant]
            done = [r for r in mine if r["result"]["completed"]]
            speeds = [s for s in (racing_speed(r) for r in done) if s is not None]
            means.append(np.mean(speeds) if speeds else 0.0)
            labels.append(f"{len(done)}/{len(mine)}")
        x = np.arange(len(names))
        bars = ax.bar(x, means, width=0.6, color=SERIES[0])
        for bar, label in zip(bars, labels):
            ax.annotate(label, (bar.get_x() + bar.get_width() / 2, bar.get_height()),
                        xytext=(0, 3), textcoords="offset points", ha="center", fontsize=7,
                        color=TEXT_SECONDARY)
        ax.set_xticks(x)
        ax.set_xticklabels(names, rotation=35, ha="right", fontsize=7)
        ax.set_title(title, color=TEXT, fontsize=9, loc="left")
    axes[0][0].set_ylabel("mean racing-lap speed (m/s)", color=TEXT_SECONDARY, fontsize=9)
    fig.suptitle("Ablations: racing speed over completed runs (label: runs completed)",
                 color=TEXT, fontsize=10, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    return True


def plot_noise(results, out):
    path = results / "noise" / "summary.json"
    if not path.exists():
        return False
    summaries = json.loads(path.read_text())
    scales, series = [], {"map, after lap 1": [], "localization, racing": []}
    for s in summaries:
        scales.append(float(s["variant"].split("-", 1)[1]))
        series["map, after lap 1"].append(s["map_rms"])
        series["localization, racing"].append(s["localization_racing"])
    fig, ax = plt.subplots(figsize=(6, 3.8), facecolor=SURFACE)
    style(ax)
    for color, (label, values) in zip(SERIES, series.items()):
        xs = [x for x, v in zip(scales, values) if v is not None]
        ys = [100.0 * v for v in values if v is not None]
        ax.plot(xs, ys, color=color, linewidth=2, marker="o", markersize=5, label=label)
    ax.set_xlabel("sensor noise scale (1 = the simulator's defaults)", color=TEXT_SECONDARY,
                  fontsize=9)
    ax.set_ylabel("RMS error (cm)", color=TEXT_SECONDARY, fontsize=9)
    ax.set_ylim(bottom=0.0)
    ax.set_title("Map and localization error against sensor noise\nmean over the runs of each "
                 "noise level", color=TEXT, fontsize=10, loc="left")
    ax.legend(fontsize=8, frameon=False, labelcolor=TEXT_SECONDARY, loc="best")
    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--results", type=Path, default=Path("results/runs"))
    parser.add_argument("--out", type=Path, default=Path("docs/img"))
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    written = []
    baseline = load_runs(args.results, "baseline")
    if baseline:
        plot_laps(baseline, args.out / "evaluation-laps.png")
        written.append("evaluation-laps.png")
    if plot_ablations(args.results, args.out / "evaluation-ablations.png"):
        written.append("evaluation-ablations.png")
    if plot_noise(args.results, args.out / "evaluation-noise.png"):
        written.append("evaluation-noise.png")
    if not written:
        raise SystemExit(f"no sweeps under {args.results}")
    print("wrote " + ", ".join(written) + f" in {args.out}")


if __name__ == "__main__":
    main()

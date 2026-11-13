"""Cone update and IMU prediction cost against the landmark count.

Reads results/estimation/latency.json (written by apex_estimation_bench) and writes
docs/img/estimation-latency.png:

    uv run --project tools python tools/estimation/plot_latency.py
"""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
BUDGET_UPDATE_MS = 5.0  # docs/design.md, section 6: one cone update at 200 landmarks
BUDGET_PREDICT_US = 50.0  # one IMU prediction


def main() -> None:
    latency = json.loads((ROOT / "results/estimation/latency.json").read_text())
    bins = latency["bins"]
    centers = [0.5 * (b["landmarks_from"] + b["landmarks_to"]) for b in bins]
    update_p50 = [b["update"]["p50_us"] / 1000.0 for b in bins]
    update_p99 = [b["update"]["p99_us"] / 1000.0 for b in bins]
    predict_p99 = [b["predict"]["p99_us"] for b in bins]

    fig, (left, right) = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    left.plot(centers, update_p50, "o-", label="p50")
    left.plot(centers, update_p99, "s-", label="p99")
    left.axhline(BUDGET_UPDATE_MS, color="grey", linestyle="--", label="budget at 200 landmarks")
    left.set_xlabel("landmarks in the map")
    left.set_ylabel("one cone update (ms)")
    left.set_title("Association + EKF update per scan")
    left.legend()
    left.grid(alpha=0.3)

    right.plot(centers, predict_p99, "s-", color="tab:green", label="p99")
    right.axhline(BUDGET_PREDICT_US, color="grey", linestyle="--", label="budget")
    right.set_xlabel("landmarks in the map")
    right.set_ylabel("one IMU prediction (us)")
    right.set_title("Velocity filter + odometry per IMU sample")
    right.legend()
    right.grid(alpha=0.3)

    out = ROOT / "docs/img/estimation-latency.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=120)
    print(f"wrote {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()

"""Gather the evaluation sweeps' summaries into results/runs/README.md.

    uv run --project tools python tools/summarize_evaluation.py --results results/runs

Each sweep directory written by `apex sweep` holds a summary.md; this puts them in one file, in
the order of docs/evaluation.md, with the commit they were produced at.
"""

import argparse
import subprocess
from pathlib import Path

EXPERIMENTS = ["baseline", "controllers", "oracles", "noise", "latency", "friction"]


def commit() -> str:
    def git(*args):
        return subprocess.run(["git", *args], capture_output=True, text=True, check=True).stdout

    head = git("rev-parse", "--short", "HEAD").strip()
    dirty = git("status", "--porcelain", "--untracked-files=no").strip()
    return head + (" (with uncommitted changes)" if dirty else "")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--results", type=Path, default=Path("results/runs"))
    args = parser.parse_args()

    parts = [
        "# Evaluation results\n",
        f"Produced by `tools/run_evaluation.sh` at commit {commit()}. The methodology, the "
        "definitions of every column and the analysis are in "
        "[docs/evaluation.md](../../docs/evaluation.md); each sweep's config is in "
        "[configs/](configs/) and every run's full result in `<sweep>/runs.json`.\n",
    ]
    found = 0
    for experiment in EXPERIMENTS:
        summary = args.results / experiment / "summary.md"
        if summary.exists():
            parts.append(summary.read_text().strip() + "\n")
            found += 1
    if found == 0:
        raise SystemExit(f"no sweep summaries under {args.results}")
    (args.results / "README.md").write_text("\n".join(parts))


if __name__ == "__main__":
    main()

#!/usr/bin/env bash
# Runs the evaluation of docs/evaluation.md: every sweep in results/runs/configs/ (or the ones
# named on the command line), then the combined summary and the figures.
#
#   make PRESET=release build
#   tools/run_evaluation.sh                     # all six sweeps
#   tools/run_evaluation.sh baseline noise      # some of them
#
# THREADS (default 3) runs at a time; each run is single-threaded and its result does not
# depend on it. APEX picks the binary (default: the release build).
set -euo pipefail

root="$(git rev-parse --show-toplevel)"
cd "$root"
apex="${APEX:-build/release/apps/apex}"
threads="${THREADS:-3}"
experiments=("$@")
if [ ${#experiments[@]} -eq 0 ]; then
    experiments=(baseline controllers oracles noise latency friction)
fi

for experiment in "${experiments[@]}"; do
    "$apex" sweep --config "results/runs/configs/${experiment}.json" \
        --out "results/runs/${experiment}" --threads "$threads" --quiet > /dev/null
done
uv run --project tools python tools/summarize_evaluation.py --results results/runs
uv run --project tools python tools/plot_evaluation.py --results results/runs --out docs/img

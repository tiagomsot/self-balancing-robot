"""Plot matched BO/DE score convergence against evaluations and wall time."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="Output path without extension")
    args = parser.parse_args()
    data = json.loads((args.folder/"comparison.json").read_text())
    methods = {"bayesian": ("Bayesian", "#1769aa"), "differential_evolution": ("Differential evolution", "#c46a00")}
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.9), layout="constrained")
    time_start = max(r["trace"][0]["elapsed_s"] for r in data["runs"])
    time_end = max(r["search_elapsed_s"] for r in data["runs"])
    seconds = np.linspace(time_start, time_end, 250)
    for method, (label, color) in methods.items():
        subset = [r for r in data["runs"] if r["method"] == method]
        counts = np.array([row["evaluations"] for row in subset[0]["trace"]])
        costs = np.array([[row["best_objective"] for row in r["trace"]] for r in subset])
        axes[0].plot(counts, np.median(costs, axis=0), color=color, marker="o", markersize=3.5, label=label)
        axes[0].fill_between(counts, costs.min(axis=0), costs.max(axis=0), color=color, alpha=.12)
        traces = []
        for run in subset:
            stamps = np.array([row["elapsed_s"] for row in run["trace"]])
            values = np.array([row["best_objective"] for row in run["trace"]])
            indices = np.clip(np.searchsorted(stamps, seconds, side="right")-1, 0, len(values)-1)
            traces.append(values[indices])
        values = np.array(traces)
        axes[1].step(seconds, np.median(values, axis=0), where="post", color=color, label=label)
        axes[1].fill_between(seconds, values.min(axis=0), values.max(axis=0), color=color, alpha=.12, step="post")
    for axis in axes:
        axis.axhline(data["active_reference"]["objective"], color="#777777", linestyle="--", linewidth=1,
                     label="Previously active gains")
        axis.set_ylabel("Best feasible training score (lower is better)")
        axis.grid(alpha=.2)
        axis.legend(fontsize=8)
    axes[0].set(xlabel="Scored parameter settings per method", title="Equal evaluation budget")
    axes[1].set(xlabel="Search time (seconds)", title="Observed score versus wall time")
    fig.suptitle(f"Robot controller tuning: {len(data['seeds'])} paired seeds, {data['trials_per_method_per_seed']} trials per method", fontsize=13)
    fig.supxlabel("Lines: median; shading: full seed range. CUDA physics / shared C++ control. Warm-up and holdout excluded.", fontsize=9)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for extension in ("png", "svg"):
        fig.savefig(args.output.with_suffix("."+extension), dpi=170)
    plt.close(fig)
    print(args.output.with_suffix(".png"))


if __name__ == "__main__":
    main()

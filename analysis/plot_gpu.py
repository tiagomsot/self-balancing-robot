"""Plot preserved GPU benchmark timings; no ROS or GPU dependency required."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    reports = sorted([json.loads((run/"summary.json").read_text()) for run in args.runs], key=lambda r: r["worlds"])
    if not all(r["passed"] for r in reports):
        parser.error("A run failed validation; inspect it before publishing a benchmark")
    figure, axes = plt.subplots(1, 2, figsize=(11, 4.8), layout="constrained")
    worlds = [r["worlds"] for r in reports]
    rates = [r["aggregate_realtime_factor"] for r in reports]
    labels = [str(w) for w in worlds]
    axes[0].bar(labels, rates, color="#267bb4")
    for i, rate in enumerate(rates):
        axes[0].text(i, rate*1.04, f"{rate:,.0f}", ha="center")
    reference = reports[-1]["cpu_reference_aggregate_realtime_factor"]
    axes[0].axhline(reference, linestyle="--", color="#bd6527", label="Sequential CPU reference (4 worlds)")
    axes[0].set(title="More worlds use the GPU more effectively", xlabel="Parallel GPU robot worlds",
                ylabel="Simulated robot seconds / wall second", ylim=(0, max(rates)*1.18))
    axes[0].legend(loc="upper left", fontsize=8)
    seconds = [r["batch_wall_s"] for r in reports]
    axes[1].bar(labels, seconds, color="#30866b")
    for i, report in enumerate(reports):
        axes[1].text(i, seconds[i]+.025, f"{seconds[i]:.2f} s", ha="center")
    axes[1].set(title=f"Each world runs {reports[-1]['duration_s']:g} simulated seconds",
                xlabel="Parallel GPU robot worlds", ylabel="GPU batch wall time (s)",
                ylim=(0, max(seconds)*1.25))
    for axis in axes:
        axis.grid(axis="y", alpha=.2)
        axis.set_axisbelow(True)
    figure.suptitle("Two-wheel balancing robot | RTX 4060 Laptop GPU", fontsize=15)
    figure.supxlabel("Timing excludes setup/compilation and file writes. CPU reference is a sequential Python workflow, not a parallel CPU benchmark.", fontsize=8)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=160)
    plt.close(figure)
    args.output.with_suffix(".json").write_text(json.dumps(dict(
        source_runs=[str(path) for path in args.runs], reports=reports,
        scope="MuJoCo Warp balancing-only batch. Not Gazebo throughput or a multithreaded CPU benchmark."), indent=2)+"\n")
    print(args.output.resolve())


if __name__ == "__main__":
    main()

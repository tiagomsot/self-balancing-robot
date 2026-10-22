"""Make the portfolio comparison from preserved search and Gazebo recordings."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("search", type=Path)
    parser.add_argument("validation", type=Path)
    parser.add_argument("--output", type=Path, default=Path("docs/images/controller-tuning.png"))
    parser.add_argument("--selection", type=Path, help="Torque-cap/local refinement result directory")
    parser.add_argument("--long-validation", type=Path, help="Additional long Gazebo run directory")
    args = parser.parse_args()
    search = json.loads((args.search / "search.json").read_text())
    validation = json.loads((args.validation / "validation.json").read_text())
    selection = (json.loads((args.selection / "selection.json").read_text())
                 if args.selection else None)
    with (args.search / "progress.csv").open() as stream:
        progress = list(csv.DictReader(stream))
    plt.rcParams.update({"font.size": 11, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), layout="constrained")
    fig.suptitle("Automatic controller tuning | Gazebo comparison", fontsize=17)
    styles = (("baseline", "Original gains", "#b35328"),
              ("candidate", "Selected gains", "#007c91"))
    for profile, label, color in styles:
        for case, ax in (("nominal_5deg", axes[0, 0]), ("deadband_lag_5deg", axes[0, 1])):
            data = np.genfromtxt(args.validation / profile / case / "telemetry.csv",
                                 delimiter=",", names=True)
            ax.plot(data["time_s"], np.rad2deg(data["pitch_rad"]), label=label, color=color, lw=1.6)
            if case == "deadband_lag_5deg":
                axes[1, 0].plot(data["time_s"], 1000*data["x_m"], label=label, color=color, lw=1.6)
    for ax, title in ((axes[0, 0], "Nominal motor / driver, +5° start"),
                      (axes[0, 1], "Assumed 8% deadband + 2 ms motor response lag")):
        ax.axhspan(-1, 1, color="#007c91", alpha=0.07, label="±1° band")
        ax.set(title=title, xlabel="Simulation time (s)", ylabel="Body pitch (°)", xlim=(0, 10))
        ax.legend(fontsize=9)
    axes[1, 0].axhline(0, color="black", lw=0.6)
    axes[1, 0].set(title="Wheel odometry in the deadband / lag case",
                   xlabel="Simulation time (s)", ylabel="Position from start (mm)", xlim=(0, 10))
    axes[1, 0].legend(fontsize=9)
    axes[1, 1].plot([int(r["generation"]) for r in progress], [float(r["cost"]) for r in progress],
                    color="#007c91", lw=1.8)
    axes[1, 1].set(title="Gain-search score before torque-cap refinement",
                   xlabel="Differential-evolution generation", ylabel="Weighted scenario score", yscale="log")
    for ax in axes.flat:
        ax.grid(alpha=0.18)
    fig.supxlabel("Reference-component simulation • Ideal orientation / wheel feedback • Hardware electrical compliance unverified",
                  fontsize=10)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)
    # Keep the selected evidence in Git even though full CSV/log folders are ignored.
    saved_runs = []
    for record in validation["runs"]:
        folder = args.validation / record["profile"] / record["scenario"]
        raw = json.loads((folder / "summary.json").read_text())
        saved_runs.append({**record, "folder": f"{record['profile']}/{record['scenario']}",
                           "recorded_duration_s": raw.get("final_time_s"),
                           "controller_faulted": raw.get("controller_faulted"),
                           "actuator": raw.get("actuator", {}),
                           "source_sha256": raw.get("source_sha256", {}),
                           "parameters_sha256": hashlib.sha256((folder / "parameters.yaml").read_bytes()).hexdigest()})
    evidence = dict(
        candidate_gains=selection["parameters"] if selection else search["candidate_gains"],
        baseline_gains=search["baseline_gains"],
        seed=search["seed"], bounds=search["bounds"], generations=search["generations"],
        candidate_evaluations=search["candidate_evaluations"],
        baseline_objective=search["baseline_objective"], candidate_objective=search["candidate_objective"],
        training_candidate=selection["training_candidate"] if selection else search["training_candidate"],
        holdout_candidate=selection["holdout_candidate"] if selection else search["holdout_candidate"],
        search_source_sha256=search["source_sha256"],
        selection_source_sha256=selection["source_sha256"] if selection else None,
        gazebo_runs=saved_runs,
        long_gazebo_runs=([{**r, "folder": f"long/{r['profile']}/{r['scenario']}"}
                          for r in json.loads((args.long_validation / "validation.json").read_text())["runs"]]
                         if args.long_validation else []),
        hardware_rating_compliance_established=False,
        note="Selected simulation results; finite scenarios and a bounded search do not prove global optimality or hardware robustness.")
    args.output.with_suffix(".json").write_text(json.dumps(evidence, indent=2) + "\n")
    print(args.output)


if __name__ == "__main__":
    main()

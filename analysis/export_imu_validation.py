"""Export a passing IMU-only search and repeated Gazebo checks for the portfolio.

Full local outputs stay untouched. Publish candidate history, all case summaries,
and unfiltered controller time series from the first independent repetition.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
CASES = ("nominal_5deg", "nominal_minus5deg", "nominal_10deg", "nominal_minus10deg")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("search", type=Path)
    parser.add_argument("--gazebo", type=Path, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--figure", type=Path, required=True)
    args = parser.parse_args()
    search = json.loads((args.search / "search.json").read_text())
    candidate = yaml.safe_load((args.search / "candidate.yaml").read_text())
    p = candidate["balance_controller"]["ros__parameters"]
    if (search.get("feedback_mode") != "imu_only" or p["wheel_feedback_enabled"]
            or p["position_gain"] or p["velocity_gain"]
            or not search["selected"]["all_training_cases_pass"]
            or not search["candidate_holdout"]["passed"]):
        raise ValueError("A passing IMU-only search is required")
    active_path = ROOT / "ros2_ws/src/balancing_robot_sim/config/balance.yaml"
    if yaml.safe_load(active_path.read_text()) != candidate:
        raise ValueError("Candidate must be the currently active, validated profile")
    records, summaries, samples = [], [], {}
    for repeat, folder in enumerate(args.gazebo, 1):
        runs = json.loads((folder / "validation.json").read_text())["runs"]
        for name in CASES:
            matches = [r for r in runs if r["profile"] == "candidate" and r["scenario"] == name]
            if len(matches) != 1:
                raise ValueError(f"Missing or duplicate case {name} in {folder}")
            row = matches[0]
            case_dir = folder / "candidate" / name
            summary = json.loads((case_dir / "summary.json").read_text())
            checked = yaml.safe_load((case_dir / "parameters.yaml").read_text())
            if (not row.get("response_passed") or not row.get("electrical_prediction_passed")
                    or row.get("returncode") != 0 or not summary.get("passed")
                    or summary.get("wheel_feedback_used") is not False
                    or summary.get("feedback_mode") != "imu_only"
                    or summary.get("requested_duration_s", 0) < 12 or checked != candidate):
                raise ValueError(f"Failed or different controller: repetition {repeat}/{name}")
            records.append(dict(repetition=repeat, **{k: v for k, v in row.items() if k != "folder"}))
            summaries.append(dict(repetition=repeat, scenario=name, **summary))
            if repeat == 1:
                samples[name] = case_dir / "telemetry.csv"
    args.output.mkdir(parents=True, exist_ok=False)
    for filename in ("search.json", "trials.json", "candidate.yaml", "current.yaml", "promotion.json"):
        source = args.search / filename
        if source.exists():
            shutil.copyfile(source, args.output / filename)
    for name, source in samples.items():
        shutil.copyfile(source, args.output / f"{name}.csv")
    report = dict(
        feedback_mode="imu_only", wheel_feedback_used=False, hardware_validated=False,
        gains={name: p[name] for name in ("kp", "kd", "ki")},
        active_config_sha256=hashlib.sha256(active_path.read_bytes()).hexdigest(),
        method=search["method"], seed=search["seed"], trials=search["trials"],
        training_simulations=search["trials"] * len(search["training_cases"]),
        training_cases=search["training_cases"], additional_cases=search["holdout_cases"],
        baseline_objective=search["baseline"]["objective"],
        selected_objective=search["selected"]["objective"],
        objective_improvement_percent=100 * search["improvement_fraction"],
        candidate_training_passed=True, candidate_additional_passed=True,
        gazebo_repetitions=len(args.gazebo), gazebo_passed=len(records),
        gazebo_total=len(records), gazebo_cases=records, gazebo_summaries=summaries,
        limits="Nominal calibrated sensors and averaged actuator model. Repeats vary launch timing, "
               "not measured noise, mass, bias or friction. Extra initial states were used in earlier "
               "design iterations; they are not a fresh statistical holdout. No position/speed hold, "
               "real motor calibration, hardware validation or global optimum is established.")
    (args.output / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    trials = json.loads((args.search / "trials.json").read_text())
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
    colors = ("#0072B2", "#56B4E9", "#D55E00", "#CC79A7")
    degree = 180 / math.pi
    for name, color in zip(CASES, colors):
        data = np.genfromtxt(samples[name], delimiter=",", names=True)
        label = name.replace("nominal_", "").replace("minus", "-")
        axes[0, 0].plot(data["time_s"], degree * data["pitch_rad"], color=color, label=label)
        axes[0, 0].plot(data["time_s"], degree * data["estimated_pitch_rad"], color=color,
                        linestyle="--", linewidth=.8, alpha=.7)
        axes[0, 1].plot(data["time_s"], degree * data["pitch_rad"], color=color)
        axes[1, 1].plot(data["time_s"], data["x_m"], color=color, label=label)
    axes[0, 0].axhspan(-1, 1, color="#009E73", alpha=.12)
    axes[0, 0].set(title="Gazebo recovery: solid truth, dashed IMU estimate", ylabel="Pitch (degrees)", xlim=(0, 3))
    axes[0, 0].legend(ncols=2)
    axes[0, 1].axhspan(-1, 1, color="#009E73", alpha=.12)
    axes[0, 1].set(title="Settled pitch, final three seconds", ylabel="Pitch (degrees)", xlim=(9, 12), ylim=(-1, 1))
    incumbent = np.minimum.accumulate([r["objective"] if r["all_training_cases_pass"] else np.inf for r in trials])
    axes[1, 0].plot(np.arange(1, len(trials) + 1), incumbent, color="#0072B2")
    axes[1, 0].axhline(search["baseline"]["objective"], color="gray", linestyle="--", label="Previous gains")
    axes[1, 0].set(title="Bayesian search: best feasible training score", xlabel="Candidates evaluated", ylabel="Objective (lower is better)")
    axes[1, 0].legend()
    axes[1, 1].set(title="Drift remains measurable without wheel feedback", ylabel="Axle displacement (m)")
    for ax in (axes[0, 0], axes[0, 1], axes[1, 1]):
        ax.set_xlabel("Simulation time (s)")
    for ax in axes.flat:
        ax.grid(alpha=.2)
    fig.suptitle(f"IMU-only PID | {len(records)}/{len(records)} Gazebo checks pass | simulation evidence", fontsize=14)
    args.figure.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.figure, dpi=150)
    plt.close(fig)
    print(json.dumps({k: report[k] for k in ("gains", "trials", "objective_improvement_percent", "gazebo_passed", "gazebo_total")}, indent=2))


if __name__ == "__main__":
    main()

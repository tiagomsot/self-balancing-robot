"""Validate both method finalists and activate the best eligible training score."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from analysis.apply_optimized_gains import (ACTIVE, apply_run, check_validation, digest,
                                           validate_in_gazebo)


def choose_method(results, incumbent_score):
    eligible = [name for name, row in results.items() if row.get("gpu_passed") and
                row.get("gazebo_balance_passed") and row.get("movement_passed") is not False and
                math.isfinite(row["training_score"]) and row["training_score"] <= incumbent_score+1e-12]
    return min(eligible, key=lambda name:results[name]["training_score"]) if eligible else None


def finalize(folder, root=ROOT, check_shuttle=False, reuse_validation=False):
    folder, root = Path(folder).resolve(), Path(root).resolve()
    comparison = json.loads((folder/"comparison.json").read_text())
    results = {}
    for method in ("bayesian", "differential_evolution"):
        target = folder/f"finalist-{method}"
        search = json.loads((target/"search.json").read_text())
        candidate = yaml.safe_load((target/"candidate.yaml").read_text())
        row = dict(seed=search["seed"], training_score=search["selected"]["objective"],
                   holdout_score=search["candidate_holdout"]["objective"],
                   gpu_passed=bool(search["candidate_ready_for_gazebo"]),
                   gazebo_balance_passed=False, movement_passed=None)
        results[method] = row
        if row["gpu_passed"]:
            print(f"Validating {method} finalist from seed {row['seed']}...", flush=True)
            try:
                if not reuse_validation:
                    validate_in_gazebo(target, root)
                records = json.loads((target/"gazebo-validation/validation.json").read_text())["runs"]
                row["balance_cases"] = records
                row["gazebo_response_cases_passed"] = sum(r.get("response_passed") is True for r in records)
                row["gazebo_electrical_cases_passed"] = sum(r.get("electrical_prediction_passed") is True for r in records)
                check_validation(target, root, candidate)
                row["gazebo_balance_passed"] = True
                if check_shuttle:
                    destination = target/"shuttle-validation"
                    print(f"Checking default movement routine for {method}...", flush=True)
                    if not reuse_validation or not (destination/"summary.json").exists():
                        command = ["bash", "-c", 'set -e; source /opt/ros/jazzy/setup.bash; '
                                   'source "$1/ros2_ws/install/setup.bash"; cd "$1"; shift; '
                                   'exec /usr/bin/python3 "$@"', "comparison-shuttle", str(root),
                                   str(root/"analysis/check_shuttle.py"), "--cycles", "1",
                                   "--params-file", str(target/"candidate.yaml"), "--output", str(destination)]
                        with (target/"shuttle-validation.log").open("w") as log:
                            process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
                    motion = json.loads((destination/"summary.json").read_text())
                    row["movement_passed"] = bool(motion["passed"])
                    row["movement_completion_sim_s"] = motion.get("final_status", {}).get("time_s")
                    row["leg_distances_m"] = motion.get("ground_truth_leg_distances_m")
                    row["turn_errors_deg"] = motion.get("ground_truth_turn_errors_deg")
            except (RuntimeError, OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
                row["validation_error"] = str(error)
                if check_shuttle and row["gazebo_balance_passed"]:
                    row["movement_passed"] = False
        (folder/"finalist-validation.json").write_text(json.dumps(results, indent=2)+"\n")
        print(json.dumps(dict(method=method, **{key:row[key] for key in
              ("training_score", "gpu_passed", "gazebo_balance_passed", "movement_passed")})), flush=True)
    selected = choose_method(results, comparison["active_reference"]["objective"])
    promotion = dict(status="kept_previous_gains", active_config_modified=False)
    if selected is not None:
        promotion = apply_run(folder/f"finalist-{selected}", root, reuse_validation=True)
    report = dict(finalists=results, selected_method=selected, promotion=promotion,
                  selection_rule="Lowest feasible training score, passing independent checks, no worse than active reference. Holdout/Gazebo scores are not ranking criteria.",
                  active_config_sha256=digest(root/ACTIVE), check_shuttle=check_shuttle,
                  finalizer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), hardware_validated=False)
    (folder/"finalization.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(dict(selected_method=selected, **promotion), indent=2), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("--check-shuttle", action="store_true", help="Also check the default two-leg routine")
    parser.add_argument("--reuse-validation", action="store_true")
    args = parser.parse_args()
    finalize(args.folder, check_shuttle=args.check_shuttle, reuse_validation=args.reuse_validation)


if __name__ == "__main__":
    main()

"""Repeat fixed finalist measurements in every three-candidate batch ordering."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
import warp as wp

from bayes_search import Evaluator, TRAINING
from parallel_sim import ROOT, configuration, load_model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    folder = args.folder.resolve()
    output = folder/"finalist-recheck.json"
    if output.exists():
        parser.error("Recheck already exists; preserve it and use a separate comparison folder")
    comparison = json.loads((folder/"comparison.json").read_text())
    names = ("bayesian", "differential_evolution", "previously_active")
    gains = [json.loads((folder/f"finalist-{m}/search.json").read_text())["selected"]["gains"] for m in names[:2]]
    gains.append(comparison["active_reference"]["gains"])
    # Use the exact pre-search settings, even if finalist validation has since
    # updated the active profile. This does not change or retune any gains.
    profile = folder/"sources/ros2_ws/src/balancing_robot_sim/config/balance.yaml"
    _, balance, _, parameters = configuration(profile)
    wp.init()
    device = wp.get_device("cuda:0")
    rows = []
    with wp.ScopedDevice(device):
        evaluator = Evaluator(load_model(), TRAINING, comparison["batch_size"], np.array(gains[2]), parameters, device, balance)
        for order in itertools.permutations(range(3)):
            cost, passed, cases, elapsed = evaluator.evaluate(np.array([gains[i] for i in order]), comparison["training_seconds"])
            rows.append(dict(order=[names[i] for i in order], physics_wall_s=elapsed,
                             results={names[i]:dict(objective=float(c), passed=bool(ok), cases=row)
                                      for i,c,ok,row in zip(order,cost,passed,cases)}))
    statistics = {}
    for name in names:
        costs = np.array([row["results"][name]["objective"] for row in rows])
        statistics[name] = dict(median=float(np.median(costs)), mean=float(np.mean(costs)),
                               std=float(np.std(costs)), minimum=float(costs.min()), maximum=float(costs.max()),
                               all_passed=all(row["results"][name]["passed"] for row in rows))
    report = dict(purpose="Audit numerical/ordering variability of fixed finalists; does not select or retune gains.",
                  protocol="All six permutations of the three profiles; original batch size, cases and duration; final-batch padding as in search.",
                  names=names, gains=gains, statistics=statistics, repetitions=rows,
                  recheck_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  active_config_modified=False)
    output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(statistics, indent=2), flush=True)


if __name__ == "__main__":
    main()

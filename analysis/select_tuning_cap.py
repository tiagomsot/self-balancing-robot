"""Refine the final population against a discrete, lower torque-cap sweep.

The request cap is a controller setting, not a physical current regulator.
The original 0.060 N m upper bound is never increased.
"""

import argparse
import hashlib
from itertools import product
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analysis.tuning_model import (BASELINE, GAIN_NAMES, PARAMETER_NAMES, ROOT, rows_for,
                                   simulate, timing_holdout_scenarios,
                                   timing_stress_scenarios, training_scenarios)
from analysis.tune_controller import write_profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("search", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-cap", type=float, default=0.060)
    parser.add_argument("--refine-neighborhood", action="store_true",
                        help="Explore higher damping with a lower cap, then rescore a shortlist")
    args = parser.parse_args()
    report = json.loads((args.search / "search.json").read_text())
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / "selection.json").exists():
        parser.error("Choose a fresh output directory")
    population = sorted(report["refined_population"], key=lambda r: r["cost"])
    # Retain a spread of finalists, not 40 almost identical rows.
    finalists = []
    for member in population:
        g = np.array([member["gains"][name] for name in GAIN_NAMES])
        if not any(np.allclose(g, old, rtol=0.002, atol=1e-5) for old in finalists):
            finalists.append(g)
        if len(finalists) == 12:
            break
    caps = tuple(cap for cap in (0.035, 0.040, 0.045, 0.050, 0.055, 0.060) if cap <= args.max_cap)
    if not caps:
        parser.error("--max-cap must be at least 0.035 N m")
    candidates = np.array([np.r_[g, cap] for g in finalists for cap in caps])
    cases = training_scenarios() + timing_stress_scenarios()
    coarse_records = []
    if args.refine_neighborhood:
        grid = list(product((0.55, 0.70, 0.85, 1.0), (0.08, 0.10, 0.12),
                            (0.08, 0.15), (0.15, 0.25, 0.35),
                            tuple(cap for cap in (0.040, 0.045) if cap <= args.max_cap)))
        if grid:
            candidates = np.concatenate((candidates, np.array(grid)))
        print(f"Screening {len(candidates)} local gain/cap combinations", flush=True)
        coarse = simulate(candidates, cases, seconds=15, physics_substeps=2, tail_seconds=5)
        coarse_cost = coarse["cost"].mean(axis=1) + 0.75*coarse["cost"].max(axis=1)
        coarse_records = [dict(parameters=dict(zip(PARAMETER_NAMES, g.tolist())), cost=float(cost))
                          for g, cost in zip(candidates, coarse_cost)]
        candidates = candidates[np.argsort(coarse_cost)[:24]]
    print(f"Screening {len(candidates)} finalist/cap combinations over 20 s with 1 ms physics", flush=True)
    metrics = simulate(candidates, cases, seconds=20, physics_substeps=5, tail_seconds=10)
    costs = metrics["cost"].mean(axis=1) + 0.75*metrics["cost"].max(axis=1)
    index = int(np.argmin(costs))
    selected = candidates[index]
    heldout = timing_holdout_scenarios()
    independent = simulate(selected, heldout, seconds=20, physics_substeps=5, tail_seconds=5)
    records = [dict(parameters=dict(zip(PARAMETER_NAMES, g.tolist())), cost=float(cost),
                    response_passes=int(np.sum(metrics["response_passed"][i])),
                    electrical_prediction_passes=int(np.sum(metrics["electrical_prediction_passed"][i])))
               for i, (g, cost) in enumerate(zip(candidates, costs))]
    source_paths = [ROOT / "analysis/select_tuning_cap.py", ROOT / "analysis/tuning_model.py",
                    ROOT / "analysis/tune_controller.py"]
    hashes = {}
    for path in source_paths:
        relative = path.relative_to(ROOT)
        data = path.read_bytes()
        hashes[str(relative)] = hashlib.sha256(data).hexdigest()
        snapshot = args.output / "sources" / relative
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        snapshot.write_bytes(data)
    result = dict(
        parameters=dict(zip(PARAMETER_NAMES, selected.tolist())),
        baseline_parameters=dict(zip(PARAMETER_NAMES, np.r_[BASELINE, .060].tolist())),
        candidates=records, selected_cost=float(costs[index]),
        coarse_candidates=coarse_records, max_cap_nm=args.max_cap,
        training_candidate=rows_for(metrics, cases, index),
        holdout_candidate=rows_for(independent, heldout),
        source_sha256=hashes, parent_search=str(args.search),
        parent_search_sha256=hashlib.sha256((args.search / "search.json").read_bytes()).hexdigest(),
        hardware_rating_compliance_established=False,
        note="Bounded gain search followed by a discrete cap sweep; require actual Gazebo validation.")
    (args.output / "selection.json").write_text(json.dumps(result, indent=2) + "\n")
    write_profile(args.output / "candidate.yaml", selected)
    print(json.dumps({"parameters": result["parameters"], "selected_cost": result["selected_cost"]}), flush=True)


if __name__ == "__main__":
    main()

"""Reproducible, bounded gain search; shortlisted gains still need Gazebo checks."""

import argparse
import csv
from dataclasses import asdict
import hashlib
import json
import platform
import sys
import time
from pathlib import Path

import numpy as np
import scipy
from scipy.optimize import differential_evolution

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analysis.tuning_model import (BASELINE, BOUNDS, GAIN_NAMES, PARAMETER_NAMES, ROOT, holdout_scenarios,
                                   objective, rows_for, simulate, training_scenarios,
                                   timing_holdout_scenarios, timing_stress_scenarios)


def write_profile(path, gains):
    names = PARAMETER_NAMES if len(gains) == 5 else GAIN_NAMES
    path.write_text("# Automated screening candidate; see docs/controller-tuning.md.\n"
                    "balance_controller:\n  ros__parameters:\n" + "".join(
                        f"    {key}: {float(value)!r}\n"
                        for key, value in zip(names, gains)), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/controller-search"))
    parser.add_argument("--seed", type=int, default=20261021)
    parser.add_argument("--generations", type=int, default=60)
    parser.add_argument("--population", type=int, default=48)
    parser.add_argument("--timing-stress", action="store_true",
                        help="Include known 5/10 ms timing sensitivities and use new holdout cases")
    parser.add_argument("--robust-window", action="store_true",
                        help="Score 15 s runs over the last 5 s, then rank the population with 1 ms physics")
    parser.add_argument("--kd-max", type=float, default=BOUNDS[1][1])
    args = parser.parse_args()
    if args.generations < 1 or args.population < 8:
        parser.error("Need >=1 generations and >=8 population members")
    if not np.isfinite(args.kd_max) or args.kd_max <= BOUNDS[1][0]:
        parser.error("--kd-max must be finite and exceed the lower bound")
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / "search.json").exists():
        parser.error("Output already contains a search; select a new directory")
    training, heldout = training_scenarios(), holdout_scenarios()
    if args.timing_stress:
        training += timing_stress_scenarios()
        heldout = timing_holdout_scenarios()
    source_paths = [ROOT / path for path in (
        "analysis/tune_controller.py", "analysis/tuning_model.py", "simulation/model.py",
        "ros2_ws/src/balancing_robot_sim/scripts/motor_model.py",
        "ros2_ws/src/balancing_robot_sim/config/balance.yaml")]
    source_hashes = {}
    for source in source_paths:
        relative = source.relative_to(ROOT)
        data = source.read_bytes()
        source_hashes[str(relative)] = hashlib.sha256(data).hexdigest()
        snapshot = args.output / "sources" / relative
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        snapshot.write_bytes(data)
    rng = np.random.default_rng(args.seed)
    bounds = np.array(BOUNDS)
    bounds[1, 1] = args.kd_max
    population = rng.uniform(bounds[:, 0], bounds[:, 1], (args.population, 4))
    population[0] = BASELINE
    started = time.monotonic()
    history = []
    options = dict(seconds=15 if args.robust_window else 10,
                   tail_seconds=5 if args.robust_window else 1,
                   physics_substeps=2 if args.robust_window else 1)
    evaluated = {}

    def loss(values):
        batch = np.atleast_2d(values.T)
        costs = objective(batch, training, **options)
        evaluated.update((tuple(g), float(cost)) for g, cost in zip(batch, costs))
        return costs

    def progress(values, convergence):
        cost = evaluated[tuple(values)]
        record = dict(generation=len(history) + 1, cost=cost,
                      elapsed_s=time.monotonic() - started,
                      **dict(zip(GAIN_NAMES, values.tolist())))
        history.append(record)
        with (args.output / "progress.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(record))
            writer.writeheader()
            writer.writerows(history)
        write_profile(args.output / "candidate.yaml", values)
        print(json.dumps(record), flush=True)

    print(f"Searching {len(training)} scenarios; seed={args.seed}; bounds={bounds.tolist()}", flush=True)
    result = differential_evolution(
        loss, bounds, maxiter=args.generations, init=population, seed=args.seed,
        vectorized=True, updating="deferred", workers=1, polish=False,
        tol=0.005, mutation=(0.5, 1.0), recombination=0.8, callback=progress)
    selected = result.x
    population_refined = None
    if args.robust_window:
        print("Ranking the final population with 1 ms physics over 20 seconds", flush=True)
        population_refined = simulate(result.population, training, seconds=20,
                                      physics_substeps=5, tail_seconds=10)
        refined_cost = population_refined["cost"].mean(axis=1) + 0.75*population_refined["cost"].max(axis=1)
        selected = result.population[np.argmin(refined_cost)]
    candidates = np.stack((BASELINE, selected))
    training_metrics = simulate(candidates, training, **options)
    heldout_metrics = simulate(candidates, heldout, seconds=20 if args.robust_window else 15,
                               physics_substeps=5 if args.robust_window else 1,
                               tail_seconds=5 if args.robust_window else 1)
    # Halve the mechanical RK4 step without changing sampled control/PWM timing.
    refined = simulate(selected, training, seconds=20 if args.robust_window else 10,
                       physics_substeps=5, tail_seconds=10 if args.robust_window else 1)
    report = dict(
        method="SciPy differential_evolution; bounded stochastic screening, no global optimality claim",
        seed=args.seed, bounds=dict(zip(GAIN_NAMES, bounds.tolist())), generations=len(history),
        timing_stress=args.timing_stress,
        robust_window=args.robust_window,
        candidate_evaluations=(len(history)+1)*args.population,
        elapsed_s=time.monotonic()-started,
        python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__,
        control_step_s=0.005, training_duration_s=options["seconds"],
        scoring_tail_seconds=options["tail_seconds"],
        search_physics_step_s=0.005/options["physics_substeps"],
        refined_duration_s=20 if args.robust_window else 10,
        refined_scoring_tail_seconds=10 if args.robust_window else 1,
        holdout_duration_s=20 if args.robust_window else 15,
        fixed_command_cap_nm=0.060, fixed_target_tilt_cap_deg=5,
        objective="mean scenario cost + 0.75 * worst scenario cost; score_metrics() defines weights",
        baseline_gains=dict(zip(GAIN_NAMES, BASELINE.tolist())),
        candidate_gains=dict(zip(GAIN_NAMES, selected.tolist())),
        baseline_objective=float(objective(BASELINE, training, **options)[0]),
        candidate_objective=float(objective(selected, training, **options)[0]),
        optimizer_winner_objective=float(result.fun),
        training_scenarios=[asdict(c) for c in training],
        holdout_scenarios=[asdict(c) for c in heldout],
        training_baseline=rows_for(training_metrics, training, 0),
        training_candidate=rows_for(training_metrics, training, 1),
        holdout_baseline=rows_for(heldout_metrics, heldout, 0),
        holdout_candidate=rows_for(heldout_metrics, heldout, 1),
        refined_physics_candidate=rows_for(refined, training),
        refined_population=([dict(gains=dict(zip(GAIN_NAMES, g.tolist())),
                                  cost=float(cost),
                                  response_passed=bool(np.all(population_refined["response_passed"][i])))
                             for i, (g, cost) in enumerate(zip(result.population, refined_cost))]
                            if population_refined is not None else []),
        source_sha256=source_hashes,
        hardware_rating_compliance_established=False,
        scope="Planar, symmetric rolling surrogate. Contact, transport jitter, sensors and actual hardware require separate validation.",
    )
    (args.output / "search.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    write_profile(args.output / "candidate.yaml", selected)
    print(json.dumps({key: report[key] for key in (
        "baseline_objective", "candidate_objective", "candidate_gains", "elapsed_s")}), flush=True)


if __name__ == "__main__":
    main()

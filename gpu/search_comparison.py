"""Matched-budget optimizer loops, independent of CUDA and robot physics."""
import copy
import time

import numpy as np
from scipy.optimize import differential_evolution
from scipy.stats import qmc


def initial_design(seed, starting_point, population=16):
    if population < 8 or population & (population - 1):
        raise ValueError("Initial population must be a power of two, at least eight")
    point = np.asarray(starting_point, dtype=float)
    if point.ndim != 1 or not np.isfinite(point).all() or np.any((point < 0) | (point > 1)):
        raise ValueError("Starting point must be a finite unit-cube vector")
    points = qmc.Sobol(len(point), scramble=True, seed=seed).random_base2(population.bit_length()-1)
    points[0] = point
    return points


def select_candidate(records):
    feasible = [record for record in records if record["all_training_cases_pass"]]
    return min(feasible or records, key=lambda row: row["objective"])


def run_search(method, initial, initial_records, initial_trace, initial_seconds,
               evaluate, propose, seed, trials=64, batch_size=8, progress=None):
    """evaluate(unit_points) returns cost, passed flags, case records, gains.

    Both methods receive the SAME initial observations. Their measured common
    initialization cost is charged to each. Every new candidate is evaluated,
    including duplicates. Selection never uses held-out cases.
    """
    initial = np.asarray(initial, dtype=float)
    population, dimensions = initial.shape
    if method not in ("bayesian", "differential_evolution"):
        raise ValueError("Unknown optimizer")
    if trials < population * 2 or trials % population or not 1 <= batch_size <= population:
        raise ValueError("Budget must include whole DE generations after initialization")
    if len(initial_records) != population or len({r["trial"] for r in initial_records}) != population:
        raise ValueError("Initial observations do not match the population")
    records, trace = copy.deepcopy(initial_records), copy.deepcopy(initial_trace)
    observed = initial.tolist()
    costs = [row["objective"] for row in records]
    rng, gp_records = np.random.default_rng(seed), []
    began = time.perf_counter()

    def measure(points, stage):
        points = np.asarray(points, dtype=float)
        if len(records) + len(points) > trials:
            raise RuntimeError("Optimizer exceeded its evaluation budget")
        values = []
        for start in range(0, len(points), batch_size):
            chunk = points[start:start+batch_size]
            scores, passed, cases, gains = evaluate(chunk)
            if len(scores) != len(chunk) or not np.isfinite(scores).all():
                raise RuntimeError("Evaluator returned invalid scores")
            for point, gain, score, feasible, row in zip(chunk, gains, scores, passed, cases):
                records.append(dict(trial=len(records), stage=stage, gains=np.asarray(gain).tolist(),
                                    objective=float(score), all_training_cases_pass=bool(feasible), cases=row))
                observed.append(point.tolist())
                costs.append(float(score))
                values.append(float(score))
            trace.append(dict(evaluations=len(records), elapsed_s=initial_seconds+time.perf_counter()-began,
                              best_objective=float(select_candidate(records)["objective"])))
            if progress is not None:
                progress(records, trace, gp_records)
        return np.array(values)

    if method == "bayesian":
        while len(records) < trials:
            points = propose(np.array(observed), np.array(costs), min(batch_size, trials-len(records)), rng, gp_records)
            measure(points, "bayesian_ei")
        optimizer_info = dict(kernel="Matern-5/2", acquisition="analytic EI with batch distance exclusion")
    else:
        first = True

        def loss(columns):
            nonlocal first
            points = np.asarray(columns).T
            if first:
                first = False
                if points.shape != initial.shape or not np.allclose(points, initial, rtol=0, atol=1e-14):
                    raise RuntimeError("DE did not use the shared initial population")
                return np.array([row["objective"] for row in initial_records])
            return measure(points, "de_generation")

        result = differential_evolution(loss, [(0, 1)]*dimensions, init=initial,
            maxiter=trials//population-1, strategy="best1bin", mutation=(.5, 1), recombination=.7,
            # Generator-valued seed also supports Ubuntu's older SciPy; it
            # preserves the same generator used by newer rng= interfaces.
            seed=rng, updating="deferred", vectorized=True, polish=False, tol=0, atol=0)
        optimizer_info = dict(strategy="best1bin", mutation=[.5, 1], recombination=.7,
                              generations=int(result.nit), population=population,
                              polish=False, updating="deferred", scipy_message=str(result.message))
    if len(records) != trials:
        raise RuntimeError(f"Expected {trials} evaluations, recorded {len(records)}")
    return dict(method=method, seed=seed, trials=trials, records=records, trace=trace,
                selected=select_candidate(records), feasible_trials=sum(r["all_training_cases_pass"] for r in records),
                search_elapsed_s=initial_seconds+time.perf_counter()-began,
                surrogate_fits=gp_records, optimizer_settings=optimizer_info)

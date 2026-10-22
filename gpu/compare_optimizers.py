"""Paired BO/DE experiment using the SAME C++ controller and CUDA evaluator."""
import argparse
from dataclasses import asdict
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import sys
import time

import numpy as np
import warp as wp
import yaml

from bayes_search import Evaluator, TRAINING, HELDOUT, propose, save_profile
from optimization_core import BOUNDS, NAMES, full_gains
from parallel_sim import ROOT, GEOMETRY_PATH, MODEL_PATH, GAIN_NAMES, configuration, load_model
from search_comparison import initial_design, run_search, select_candidate
from shared_core import LIBRARY_PATH


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2)+"\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=64)
    parser.add_argument("--seeds", type=int, nargs="+", default=[7, 42, 123, 20261021, 20261022])
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--seconds", type=float, default=8)
    parser.add_argument("--search-only", action="store_true", help="Skip Gazebo checks and updating the normal menu")
    parser.add_argument("--check-shuttle", action="store_true", help="Also validate the default two-leg movement routine")
    parser.add_argument("--start-gains", type=float, nargs=3, default=[1, .1, 0],
                        help="Shared cold-start incumbent; defaults to the pre-tuning gains")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.check_shuttle:
        parser.error("The IMU-only robot cannot execute the measured-distance shuttle; omit --check-shuttle")
    if not 32 <= args.trials <= 512 or args.trials % 16 or not 1 <= args.batch_size <= 16:
        parser.error("Use 32..512 trials divisible by 16 and batch size 1..16")
    if len(set(args.seeds)) != len(args.seeds) or any(s < 0 for s in args.seeds):
        parser.error("Seeds must be distinct nonnegative integers")
    if not np.isfinite(args.seconds) or not 5 <= args.seconds <= 20 or abs(args.seconds/.005-round(args.seconds/.005)) > 1e-6:
        parser.error("Use 5..20 seconds divisible by 0.005")
    start = np.array(args.start_gains)
    if not np.isfinite(start).all() or not np.all((start >= BOUNDS[:, 0]) & (start <= BOUNDS[:, 1])):
        parser.error("Starting gains must lie within the common bounds")
    if args.output.exists():
        parser.error("Use a fresh output folder")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True)
    config_path, balance, _, parameters = configuration()
    config = yaml.safe_load(config_path.read_text())
    active = np.array([balance[k] for k in GAIN_NAMES])
    if balance.get("wheel_feedback_enabled", False) or np.any(active[2:4]):
        raise ValueError("Comparison requires IMU-only control and zero wheel gains")
    if not 0 < active[4] <= .040000001:
        raise ValueError("Comparison preserves a torque cap no greater than 0.040 N m")
    normalized_start = (start-BOUNDS[:, 0])/(BOUNDS[:, 1]-BOUNDS[:, 0])
    wp.init()
    if not wp.is_cuda_available():
        raise RuntimeError("The matched comparison requires CUDA physics")
    device = wp.get_device("cuda:0")
    files = [Path(__file__), ROOT/"gpu/search_comparison.py", ROOT/"gpu/bayes_search.py",
             ROOT/"cpp/targets/mujoco/warp_host.cpp",ROOT/"cpp/targets/host/robot_c_api.cpp",
             ROOT/"gpu/optimization_core.py", ROOT/"gpu/shared_sim.py", ROOT/"gpu/parallel_sim.py",
             ROOT/"gpu/robot_model.py", ROOT/"gpu/requirements.txt", ROOT/"compare_optimizers.sh",
             ROOT/"analysis/apply_optimized_gains.py", ROOT/"analysis/finalize_optimizer_comparison.py",
             LIBRARY_PATH, GEOMETRY_PATH, MODEL_PATH, config_path,
             ROOT/"ros2_ws/src/balancing_robot_sim/scripts/shared_core.py",
             ROOT/"ros2_ws/src/balancing_robot_sim/scripts/motor_model.py"]
    files += sorted((ROOT/"cpp/src").rglob("*.cpp"))+sorted((ROOT/"cpp/src").rglob("*.h"))
    hashes = {}
    for source in files:
        name = source.relative_to(ROOT).as_posix()
        target = args.output/"sources"/name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
        hashes[name] = hashlib.sha256(source.read_bytes()).hexdigest()
    manifest = dict(protocol="Paired equal-evaluation-budget BO vs SciPy DE; shared initial observations",
        seeds=args.seeds, trials_per_method_per_seed=args.trials, population=16, batch_size=args.batch_size,
        training_seconds=args.seconds, holdout_seconds=max(12, args.seconds),
        starting_gains=dict(zip(NAMES, args.start_gains)), active_gains=active.tolist(),
        bounds=dict(zip(NAMES, BOUNDS.tolist())), fixed_torque_cap_nm=float(active[4]),
        training_cases=[asdict(c) for c in TRAINING], holdout_cases=[asdict(c) for c in HELDOUT],
        controller="shared C++ on CPU", physics_device=str(device), gpu_name=device.name,
        source_sha256=hashes, versions={name: version(name) for name in
          ("mujoco", "mujoco-warp", "warp-lang", "numpy", "scipy", "scikit-learn")},
        timing="Same measured initial evaluation time charged to both methods; warm-up and holdout excluded from search time. Includes CPU proposals, transfers, resets, scoring and physics.",
        selection="Lowest feasible TRAINING score per run and per method. Holdout/Gazebo only accept or reject finalists; their scores do not select gains.",
        order="Alternating BO-first and DE-first across seeds; one GPU worker, sequential methods.",
        scope=f"{len(args.seeds)}-seed nominal simulation experiment; not proof of global or hardware superiority.",
        feedback_mode="imu_only", optimized_gains=list(NAMES),
        active_config_modified=False, hardware_validated=False)
    write_json(args.output/"run.json", manifest) # Protocol fixed before evaluation.
    save_profile(args.output/"current.yaml", config, active)
    runs = []
    overall_start = time.perf_counter()
    with wp.ScopedDevice(device):
        warmup_start = time.perf_counter()
        evaluator = Evaluator(load_model(), TRAINING, args.batch_size, active, parameters, device, balance)
        evaluator.evaluate(np.tile(active, (args.batch_size, 1)), args.seconds)
        warmup_s = time.perf_counter()-warmup_start

        def evaluate(points):
            gains = full_gains(BOUNDS[:, 0]+np.asarray(points)*(BOUNDS[:, 1]-BOUNDS[:, 0]), active[4])
            cost, passed, rows, _ = evaluator.evaluate(gains, args.seconds)
            return cost, passed, rows, gains

        # Measure today's active profile separately; it is not injected into
        # either cold-start optimizer and cannot bias their common population.
        active_cost, active_pass, active_rows, _ = evaluator.evaluate(active[None], args.seconds)
        active_record = dict(gains=active.tolist(), objective=float(active_cost[0]),
                             all_training_cases_pass=bool(active_pass[0]), cases=active_rows[0])
        for index, seed in enumerate(args.seeds):
            initial = initial_design(seed, normalized_start)
            initial_records, initial_trace = [], []
            initial_began = time.perf_counter()
            for offset in range(0, len(initial), args.batch_size):
                costs, passed, rows, gains = evaluate(initial[offset:offset+args.batch_size])
                for gain, cost, feasible, cases in zip(gains, costs, passed, rows):
                    initial_records.append(dict(trial=len(initial_records), stage="shared_initial_sobol",
                        gains=gain.tolist(), objective=float(cost), all_training_cases_pass=bool(feasible), cases=cases))
                initial_trace.append(dict(evaluations=len(initial_records), elapsed_s=time.perf_counter()-initial_began,
                                          best_objective=select_candidate(initial_records)["objective"]))
            initial_seconds = time.perf_counter()-initial_began
            write_json(args.output/f"initial-{seed}.json", dict(points=initial.tolist(), records=initial_records,
                       trace=initial_trace, elapsed_s=initial_seconds))
            order = ["bayesian", "differential_evolution"]
            if index % 2: order.reverse()
            for method in order:
                folder = args.output/f"{method}-{seed}"
                folder.mkdir()

                def progress(records, trace, gp_records):
                    write_json(folder/"trials.json", records)
                    print(json.dumps(dict(method=method, seed=seed, completed=len(records),
                         trials=args.trials, best_objective=select_candidate(records)["objective"],
                         elapsed_s=trace[-1]["elapsed_s"])), flush=True)

                result = run_search(method, initial, initial_records, initial_trace, initial_seconds,
                    evaluate, propose, seed, args.trials, args.batch_size, progress)
                result["folder"] = folder.name
                write_json(folder/"search.json", result)
                runs.append(result)
                write_json(args.output/"runs.json", runs)
        holdout = Evaluator(load_model(), HELDOUT, min(args.batch_size, len(runs)+1), active, parameters, device, balance)
        all_gains = np.array([active.tolist()]+[r["selected"]["gains"] for r in runs])
        heldout_records = []
        for offset in range(0, len(all_gains), holdout.size):
            scores, passed, cases, _ = holdout.evaluate(all_gains[offset:offset+holdout.size], max(12, args.seconds))
            heldout_records += [dict(objective=float(score), passed=bool(ok), cases=row)
                                for score, ok, row in zip(scores, passed, cases)]
    for run, held in zip(runs, heldout_records[1:]):
        run["holdout"] = held
        write_json(args.output/run["folder"]/"search.json", run)
    finalists = {}
    for method in ("bayesian", "differential_evolution"):
        candidates = [r for r in runs if r["method"] == method]
        eligible = [r for r in candidates if r["selected"]["all_training_cases_pass"]]
        winner = min(eligible or candidates, key=lambda r:r["selected"]["objective"])
        finalists[method] = winner
        target = args.output/f"finalist-{method}"
        target.mkdir()
        save_profile(target/"current.yaml", config, active)
        save_profile(target/"candidate.yaml", config, winner["selected"]["gains"], label=method)
        profile_report = dict(**manifest, method=method, seed=winner["seed"], trials=args.trials,
            baseline=active_record, selected=winner["selected"], candidate_holdout=winner["holdout"],
            current_holdout=heldout_records[0],
            candidate_ready_for_gazebo=bool(winner["selected"]["all_training_cases_pass"] and winner["holdout"]["passed"]))
        write_json(target/"search.json", profile_report)
    stats = {}
    for method in finalists:
        subset = [r for r in runs if r["method"] == method]
        scores = [r["selected"]["objective"] for r in subset]
        times = [r["search_elapsed_s"] for r in subset]
        stats[method] = dict(median_training_score=float(np.median(scores)), best_training_score=min(scores),
            worst_training_score=max(scores), median_search_s=float(np.median(times)),
            training_feasible_runs=sum(r["selected"]["all_training_cases_pass"] for r in subset),
            holdout_passed_runs=sum(r["holdout"]["passed"] for r in subset),
            best_seed=finalists[method]["seed"])
    report = dict(**manifest, warmup_s=warmup_s, elapsed_s=time.perf_counter()-overall_start,
                  active_reference=active_record, active_holdout=heldout_records[0],
                  statistics=stats, runs=runs,
                  best_training_method=min(stats, key=lambda m:stats[m]["best_training_score"]),
                  best_median_training_method=min(stats, key=lambda m:stats[m]["median_training_score"]))
    write_json(args.output/"comparison.json", report)
    print(json.dumps(dict(output=str(args.output), statistics=stats,
        best_training_method=report["best_training_method"], active_config_modified=False), indent=2), flush=True)
    if not args.search_only:
        sys.path.insert(0, str(ROOT))
        from analysis.finalize_optimizer_comparison import finalize
        finalize(args.output, check_shuttle=args.check_shuttle)


if __name__ == "__main__":
    main()

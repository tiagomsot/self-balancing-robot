"""Bayesian gain tuning: CPU Gaussian process, batched CUDA robot physics.

The three IMU pitch PID gains are searched; torque, geometry and actuator assumptions
remain fixed. The default workflow checks the winner in Gazebo and updates the
active gains after validation. See docs/gpu-optimization.md for objective/scope.
"""

import argparse
from dataclasses import asdict, dataclass
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import time
import warnings

import numpy as np
from scipy.stats import qmc
from sklearn.exceptions import ConvergenceWarning
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import ConstantKernel, Matern
from threadpoolctl import threadpool_limits
import warp as wp
import yaml

from optimization_core import BOUNDS, NAMES, full_gains, search_values, aggregate, expected_improvement, score_worlds
from parallel_sim import (ROOT, GEOMETRY_PATH, MODEL_PATH, GAIN_NAMES, GpuBatch, body_pitch,
                          configuration, load_model)
from shared_sim import SharedGpuBatch
from shared_core import LIBRARY_PATH


@dataclass(frozen=True)
class Case:
    name: str
    tilt_deg: float = 0
    speed_m_s: float = 0
    pitch_rate_deg_s: float = 0


TRAINING = [Case("lean_+5", 5), Case("lean_-5", -5),
            Case("lean_+10", 10), Case("lean_-10", -10),
            Case("moving_+0.08", speed_m_s=.08), Case("moving_-0.08", speed_m_s=-.08),
            Case("rotating_+10", pitch_rate_deg_s=10), Case("rotating_-10", pitch_rate_deg_s=-10)]
HELDOUT = [Case("lean_+7", 7), Case("lean_-7", -7),
           Case("moving_+0.12", speed_m_s=.12), Case("moving_-0.12", speed_m_s=-.12),
           Case("rotating_+15", pitch_rate_deg_s=15), Case("rotating_-15", pitch_rate_deg_s=-15)]


@wp.kernel
def extra_metrics(q: wp.array2d(dtype=float), v: wp.array2d(dtype=float),
                  initial: wp.array(dtype=float), extra: wp.array2d(dtype=float),
                  radius: float, tail: int):
    i = wp.tid()
    tilt = wp.acos(wp.clamp(1.0-2.0*(q[i, 4]*q[i, 4]+q[i, 5]*q[i, 5]), -1.0, 1.0))
    extra[i, 0] = wp.max(extra[i, 0], tilt)
    if tail != 0:
        pitch = body_pitch(q, i)
        distance = radius*((q[i, 7]+q[i, 8])*0.5+pitch-initial[i])
        speed = radius*((v[i, 6]+v[i, 7])*0.5+v[i, 4])
        extra[i, 1] += pitch*pitch*.005
        extra[i, 2] += v[i, 4]*v[i, 4]*.005
        extra[i, 3] += distance*distance*.005
        extra[i, 4] += speed*speed*.005


class Evaluator:
    def __init__(self, model, cases, batch_size, baseline, parameters, device, settings):
        self.cases, self.size, self.radius = cases, batch_size, float(parameters[0])
        self.tilts = np.radians(np.tile([c.tilt_deg for c in cases], batch_size))
        self.batch = SharedGpuBatch(model, np.tile(baseline, (len(self.tilts), 1)), self.tilts, parameters, device, settings)
        self.extra = wp.zeros((len(self.tilts), 5), dtype=float)
        wp.launch(extra_metrics, dim=len(self.tilts), inputs=[self.batch.data.qpos, self.batch.data.qvel,
                  self.batch.initial, self.extra, self.radius, 0])
        wp.synchronize()
        self.graphs = []
        for tail in (0, 1):
            with wp.ScopedCapture() as capture:
                self.batch.physics_step(tail)
                wp.launch(extra_metrics, dim=len(self.tilts), inputs=[self.batch.data.qpos, self.batch.data.qvel,
                          self.batch.initial, self.extra, self.radius, tail])
            self.graphs.append(capture.graph)
        self.base_velocity = np.zeros((len(self.tilts), model.nv), dtype=np.float32)
        for i, case in enumerate(cases*batch_size):
            rate = math.radians(case.pitch_rate_deg_s)
            self.base_velocity[i, 0], self.base_velocity[i, 4] = case.speed_m_s, rate
            self.base_velocity[i, 6:8] = case.speed_m_s/self.radius-rate

    def evaluate(self, gains, seconds):
        count = len(gains)
        if not 1 <= count <= self.size:
            raise ValueError("Candidate batch has an invalid size")
        # Pad only the last batch; repeated padding worlds never enter reports.
        padded = np.concatenate((gains, np.tile(gains[-1], (self.size-count, 1))))
        self.batch.gains_host = np.repeat(padded, len(self.cases), axis=0)
        self.batch.gains.assign(self.batch.gains_host.astype(np.float32))
        self.batch.reset(self.tilts)
        self.batch.data.qvel.assign(self.base_velocity)
        self.batch.refresh_sensors()
        self.extra.zero_()
        wp.synchronize()
        began = time.perf_counter()
        for step in range(round(seconds/.005)):
            tail = int((step+1)*.005 > seconds-1+1e-8)
            self.batch.update_control()
            wp.capture_launch(self.graphs[tail])
        wp.synchronize()
        elapsed = time.perf_counter()-began
        if np.any(self.batch.data.overflow.numpy()):
            raise RuntimeError("GPU contact/constraint buffer overflow; results cannot be scored")
        stats, diagnostic, extra = self.batch.stats.numpy(), self.batch.diagnostic.numpy(), self.extra.numpy()
        finite = (np.isfinite(self.batch.data.qpos.numpy()).all(axis=1)
                  & np.isfinite(self.batch.data.qvel.numpy()).all(axis=1)
                  & np.isfinite(stats).all(axis=1) & np.isfinite(extra).all(axis=1))
        costs, passed = score_worlds(stats, diagnostic, extra, finite, seconds)
        rows = []
        for i in range(count):
            row = []
            for j, case in enumerate(self.cases):
                k = i*len(self.cases)+j
                row.append(dict(case=case.name, cost=float(costs[k]), passed=bool(passed[k]),
                    max_body_tilt_deg=math.degrees(float(extra[k, 0])),
                    tail_max_pitch_deg=math.degrees(float(stats[k, 4])),
                    tail_max_rate_deg_s=math.degrees(float(stats[k, 5])),
                    final_distance_m=float(stats[k, 6]), final_speed_m_s=float(stats[k, 7]),
                    max_average_motor_current_a=float(diagnostic[k, 0]),
                    max_on_current_estimate_a=float(diagnostic[k, 1]), faulted=bool(diagnostic[k, 3])))
            rows.append(row)
        costs = costs.reshape(self.size, len(self.cases))[:count]
        passed = passed.reshape(self.size, len(self.cases))[:count]
        return aggregate(costs), passed.all(axis=1), rows, elapsed


def propose(observed, costs, size, rng, gp_records):
    """EI over a Sobol/local pool, with distance exclusion for batch diversity.

    This is batched single-point EI, not joint qEI or constrained BO.
    Feasibility enters the fixed penalized objective rather than a second GP.
    """
    dimensions = len(NAMES)
    kernel = ConstantKernel(1, (.01, 100))*Matern(np.full(dimensions, .3), (.03, 3), nu=2.5)
    gp = GaussianProcessRegressor(kernel=kernel, alpha=1e-6, normalize_y=True,
                                  n_restarts_optimizer=1, random_state=int(rng.integers(2**31)))
    with warnings.catch_warnings(record=True) as caught, threadpool_limits(limits=1):
        warnings.simplefilter("always", ConvergenceWarning)
        target = np.log1p(costs)
        gp.fit(observed, target)
        pool = qmc.Sobol(dimensions, scramble=True, seed=int(rng.integers(2**31))).random_base2(12)
        local = np.clip(observed[int(np.argmin(costs))]+rng.normal(0, .10, (1024, dimensions)), 0, 1)
        pool = np.concatenate((pool, local))
        mean, std = gp.predict(pool, return_std=True)
    utility = expected_improvement(mean, std, float(target.min()))
    available = np.min(np.linalg.norm(pool[:, None]-observed[None], axis=-1), axis=1) > 1e-4
    chosen = []
    for _ in range(size):
        index = int(np.argmax(np.where(available, utility, -np.inf)))
        if not available[index]:
            raise RuntimeError("Acquisition candidate pool was exhausted")
        chosen.append(pool[index])
        available &= np.linalg.norm(pool-pool[index], axis=1) > .04
    gp_records.append(dict(kernel=str(gp.kernel_), warnings=[str(item.message) for item in caught],
                           max_expected_improvement=float(utility.max())))
    return np.array(chosen)


def save_profile(path, base_config, gains, label="GPU Bayesian search"):
    config = json.loads(json.dumps(base_config))
    config["balance_controller"]["ros__parameters"].update(dict(zip(GAIN_NAMES, map(float, gains))))
    path.write_text(f"# {label} profile; validate in Gazebo before use.\n"+yaml.safe_dump(config, sort_keys=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=64)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--seconds", type=float, default=8)
    parser.add_argument("--seed", type=int, default=20261021)
    parser.add_argument("--search-only", action="store_true",
                        help="Save a candidate without Gazebo validation or updating active gains")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 16 <= args.trials <= 512 or not 1 <= args.batch_size <= 32:
        parser.error("Need 16..512 gain trials and batch size 1..32")
    if not math.isfinite(args.seconds) or not 5 <= args.seconds <= 20 or abs(args.seconds/.005-round(args.seconds/.005)) > 1e-6:
        parser.error("Need 5..20 seconds divisible by 0.005")
    if args.output.exists():
        parser.error("Choose a fresh output folder")
    path, balance, _, parameters = configuration()
    base_config = yaml.safe_load(path.read_text())
    baseline = np.array([balance[k] for k in GAIN_NAMES])
    if balance.get("wheel_feedback_enabled", False) or np.any(baseline[2:4]):
        raise ValueError("Optimization must match the IMU-only robot; wheel gains must be zero")
    if not np.all((search_values(baseline) >= BOUNDS[:, 0]) & (search_values(baseline) <= BOUNDS[:, 1])):
        raise ValueError("The current gains are outside the declared search bounds")
    if baseline[4] > .040000001:
        raise ValueError("This workflow keeps the total torque request cap at or below 0.040 N m")
    wp.init()
    if not wp.is_cuda_available():
        raise RuntimeError("GPU optimization needs a CUDA device")
    device = wp.get_device("cuda:0")
    args.output.mkdir(parents=True)
    files = [Path(__file__), Path(__file__).with_name("optimization_core.py"),
             Path(__file__).with_name("parallel_sim.py"), Path(__file__).with_name("robot_model.py"),
             Path(__file__).with_name("requirements.txt"), ROOT/"optimize.sh",
             Path(__file__).with_name("shared_sim.py"), LIBRARY_PATH,
             ROOT/"analysis/apply_optimized_gains.py",
             ROOT/"ros2_ws/src/balancing_robot_sim/scripts/shared_core.py",
             MODEL_PATH, GEOMETRY_PATH, path, ROOT/"ros2_ws/src/balancing_robot_sim/scripts/motor_model.py"]
    files += [p for p in (ROOT/"cpp").rglob("*")
              if p.suffix in (".h", ".cpp") and "build" not in p.relative_to(ROOT/"cpp").parts]
    hashes = {}
    for source in files:
        relative = source.relative_to(ROOT)
        target = args.output/"sources"/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
        hashes[relative.as_posix()] = hashlib.sha256(source.read_bytes()).hexdigest()
    manifest = dict(method="Gaussian-process Bayesian optimization; Matern-5/2, analytic EI, diverse candidate batches",
        seed=args.seed, trials=args.trials, batch_size=args.batch_size, seconds=args.seconds,
        bounds=dict(zip(NAMES, BOUNDS.tolist())), fixed_torque_cap_nm=float(baseline[4]),
        training_cases=[asdict(case) for case in TRAINING], holdout_cases=[asdict(case) for case in HELDOUT],
        objective="mean case cost + 0.75*worst case cost; see optimization_core.score_worlds",
        gpu_name=device.name, surrogate_device="CPU", controller="shared C++ on CPU", physics_device=str(device), source_sha256=hashes,
        versions={name: version(name) for name in ("mujoco", "mujoco-warp", "warp-lang", "numpy", "scipy", "scikit-learn")},
        scope="IMU-only, shared complementary filter, MPU quantization and 5 ms DLPF-delay approximation; no wheel feedback. Fixed nominal model, calibrated sensors; noise, bias and full chip frequency response remain unmeasured.",
        feedback_mode="imu_only", optimized_gains=list(NAMES),
        hardware_validated=False, active_config_modified=False,
        automatic_update_requested=not args.search_only)
    (args.output/"run.json").write_text(json.dumps(manifest, indent=2)+"\n")
    save_profile(args.output/"current.yaml", base_config, baseline)
    rng = np.random.default_rng(args.seed)
    initial = qmc.Sobol(len(NAMES), scramble=True, seed=args.seed).random_base2(4)
    initial[0] = (search_values(baseline)-BOUNDS[:, 0])/(BOUNDS[:, 1]-BOUNDS[:, 0])
    records, gp_records = [], []
    observed, values = [], []
    began = time.perf_counter()
    with wp.ScopedDevice(device):
        evaluator = Evaluator(load_model(), TRAINING, args.batch_size, baseline, parameters, device, balance)
        while len(records) < args.trials:
            count = min(args.batch_size, args.trials-len(records), 16-len(records) if len(records) < 16 else args.batch_size)
            if len(records) < 16:
                normalized = initial[len(records):len(records)+count]
                stage = "initial_sobol"
            else:
                normalized = propose(np.array(observed), np.array(values), count, rng, gp_records)
                stage = "bayesian_ei"
            gains = full_gains(BOUNDS[:, 0]+normalized*(BOUNDS[:, 1]-BOUNDS[:, 0]), baseline[4])
            costs, passed, rows, elapsed = evaluator.evaluate(gains, args.seconds)
            for x, gain, cost, feasible, cases in zip(normalized, gains, costs, passed, rows):
                observed.append(x.tolist())
                values.append(float(cost))
                records.append(dict(trial=len(records), stage=stage, gains= gain.tolist(),
                                    objective=float(cost), all_training_cases_pass=bool(feasible), cases=cases))
            (args.output/"trials.json").write_text(json.dumps(records, indent=2)+"\n")
            print(json.dumps(dict(completed=len(records), trials=args.trials, stage=stage,
                                  best_objective=min(values), batch_physics_wall_s=elapsed)), flush=True)
        eligible = [record for record in records if record["all_training_cases_pass"]]
        selected = min(eligible or records, key=lambda row: row["objective"])
        selected_gains = np.array(selected["gains"])
        save_profile(args.output/"candidate.yaml", base_config, selected_gains)
        # The holdout is evaluated only after selection and never used to refit.
        heldout = Evaluator(load_model(), HELDOUT, 2, baseline, parameters, device, balance)
        hold_cost, hold_pass, hold_rows, _ = heldout.evaluate(np.stack((baseline, selected_gains)), max(12, args.seconds))
    report = dict(**manifest, elapsed_s=time.perf_counter()-began,
        baseline=records[0], selected=selected, feasible_trials=len(eligible),
        improvement_fraction=(records[0]["objective"]-selected["objective"])/max(1e-12, records[0]["objective"]),
        current_holdout=dict(objective=float(hold_cost[0]), passed=bool(hold_pass[0]), cases=hold_rows[0]),
        candidate_holdout=dict(objective=float(hold_cost[1]), passed=bool(hold_pass[1]), cases=hold_rows[1]),
        surrogate_fits=gp_records, gazebo_validation_passed=None,
        candidate_ready_for_gazebo=bool(selected["all_training_cases_pass"] and hold_pass[1]),
        globally_optimal=False)
    (args.output/"search.json").write_text(json.dumps(report, indent=2)+"\n")
    summary = dict(output=str(args.output.resolve()), selected_trial=selected["trial"],
        current_objective=records[0]["objective"], candidate_objective=selected["objective"],
        improvement_percent=100*report["improvement_fraction"],
        candidate_ready_for_gazebo=report["candidate_ready_for_gazebo"],
        active_config_modified=False)
    if not args.search_only:
        # Import only after CUDA scoring. The helper invokes the system ROS
        # Python for Gazebo checks; this venv need not contain rclpy.
        import sys
        import subprocess
        sys.path.insert(0, str(ROOT))
        from analysis.apply_optimized_gains import apply_run, UpdateRejected
        print("GPU search complete. Checking and updating the normal simulation settings...", flush=True)
        try:
            result = apply_run(args.output)
        except (UpdateRejected, OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
            summary.update(update_status="kept_previous_gains", update_error=str(error))
            (args.output/"promotion.json").write_text(json.dumps(summary, indent=2)+"\n")
            print(json.dumps(summary, indent=2), flush=True)
            raise SystemExit(1)
        summary.update(update_status=result["status"],
                       active_config_modified=result["active_config_modified"],
                       active_config=result["active_config"])
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()

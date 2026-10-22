"""GPU contact-physics prototype for balancing-controller screening.

The GPU runs MuJoCo Warp physics; the shared C++ IMU estimator, PID and motor
adapter run on CPU. MPU quantization and a 5 ms delay approximation are included.
Historical Warp control kernels remain as source, outside normal hardware mode.
Candidate gains must be independently validated in Gazebo.
"""

import argparse
import csv
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import sys
import time

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/balancing_robot_sim/scripts"))
from motor_model import MotorDriverModel, MotorParameters
from robot_model import GEOMETRY_PATH, MODEL_PATH, load_model

GAIN_NAMES = ("kp", "kd", "position_gain", "velocity_gain", "total_torque_limit_nm", "ki")


@wp.func
def body_pitch(q: wp.array2d(dtype=float), i: int):
    return wp.asin(wp.clamp(2.0*(q[i, 3]*q[i, 5]-q[i, 6]*q[i, 4]), -1.0, 1.0))


@wp.kernel
def balance_and_motors(q: wp.array2d(dtype=float), v: wp.array2d(dtype=float),
                       ctrl: wp.array2d(dtype=float), gains: wp.array2d(dtype=float),
                       initial: wp.array(dtype=float), p: wp.array(dtype=float),
                       diagnostic: wp.array2d(dtype=float)):
    i = wp.tid()
    pitch = body_pitch(q, i)
    rate = v[i, 4]
    distance = p[0]*((q[i, 7]+q[i, 8])*0.5+pitch-initial[i])
    speed = p[0]*((v[i, 6]+v[i, 7])*0.5+rate)
    target = wp.clamp(-gains[i, 2]*distance-gains[i, 3]*speed, -p[1], p[1])
    requested = wp.clamp(gains[i, 0]*(pitch-target)+gains[i, 1]*rate, -gains[i, 4], gains[i, 4])
    if wp.abs(pitch) > p[2] or diagnostic[i, 3] > 0.5:
        diagnostic[i, 3] = 1.0
        requested = 0.0
    # Nominal fixed-bus coast PWM, same calibration and 8-bit round-to-even
    # quantization as motor_model.py. p contains derived electrical constants.
    duty = wp.rint(wp.clamp(requested*0.5/p[3], -1.0, 1.0)*p[4])/p[4]
    if wp.abs(duty) < p[5]:
        duty = 0.0
    for motor in range(2):
        shaft_speed = v[i, 6+motor]
        current = float(0.0)
        on_current = float(0.0)
        if duty != 0.0:
            on_current = (wp.sign(duty)*wp.max(0.0, p[6]-p[7])-p[8]*shaft_speed)/p[9]
            current = wp.abs(duty)*on_current
        ctrl[i, motor] = p[10]*current-p[11]*shaft_speed
        diagnostic[i, 0] = wp.max(diagnostic[i, 0], wp.abs(current))
        diagnostic[i, 1] = wp.max(diagnostic[i, 1], wp.abs(on_current))
    diagnostic[i, 2] = wp.max(diagnostic[i, 2], wp.abs(requested))


@wp.kernel
def record_metrics(q: wp.array2d(dtype=float), v: wp.array2d(dtype=float),
                   initial: wp.array(dtype=float), stats: wp.array2d(dtype=float),
                   radius: float, dt: float, tail: int):
    i = wp.tid()
    pitch = body_pitch(q, i)
    distance = radius*((q[i, 7]+q[i, 8])*0.5+pitch-initial[i])
    speed = radius*((v[i, 6]+v[i, 7])*0.5+v[i, 4])
    stats[i, 0] = wp.max(stats[i, 0], wp.abs(pitch))
    stats[i, 1] += pitch*pitch*dt
    stats[i, 2] += distance*distance*dt
    stats[i, 3] += speed*speed*dt
    if tail != 0:
        stats[i, 4] = wp.max(stats[i, 4], wp.abs(pitch))
        stats[i, 5] = wp.max(stats[i, 5], wp.abs(v[i, 4]))
    stats[i, 6] = distance
    stats[i, 7] = speed


def configuration(params_file=None):
    path = ROOT / "ros2_ws/src/balancing_robot_sim/config/balance.yaml"
    if params_file is not None:
        path = Path(params_file).resolve()
    config = yaml.safe_load(path.read_text())
    balance = config["balance_controller"]["ros__parameters"]
    if balance.get("wheel_feedback_enabled", False) or balance["position_gain"] or balance["velocity_gain"]:
        raise ValueError("This hardware has no encoders; use zero wheel gains and IMU-only feedback")
    balance["wheel_radius_m"] = json.loads(GEOMETRY_PATH.read_text())["wheel_radius_m"]
    fields = set(MotorParameters.__dataclass_fields__)
    motor = MotorParameters(**{k: v for k, v in config["motor_driver"]["ros__parameters"].items() if k in fields})
    if motor.battery_resistance_ohm or motor.current_time_constant_s or motor.pwm_off_mode != "coast":
        raise ValueError("GPU prototype currently supports fixed-bus, quasi-static coast PWM only")
    if config["motor_driver"]["ros__parameters"]["mode"] != "motor_driver":
        raise ValueError("GPU prototype expects the motor_driver actuator configuration")
    parameters = np.array([
        balance["wheel_radius_m"], math.radians(balance["target_tilt_limit_deg"]),
        math.radians(balance["fall_cutoff_deg"]), motor.command_full_pwm_torque_nm,
        motor.pwm_levels, motor.command_deadband, motor.battery_voltage_v,
        motor.driver_drop_intercept_v, motor.back_emf_v_s_rad,
        motor.resistance_ohm+motor.driver_resistance_ohm,
        motor.torque_constant_nm_a, motor.viscous_drag_nm_s_rad,
    ], dtype=np.float32)
    return path, balance, motor, parameters


def initial_state(model, count, tilts):
    q = np.tile(model.qpos0, (count, 1))
    q[:, 3], q[:, 5] = np.cos(tilts/2), np.sin(tilts/2)
    return q.astype(np.float32), np.zeros((count, model.nv), dtype=np.float32)


def cpu_command(v, readings, driver, estimator, controller):
    pitch,rate,tilt,yaw,valid=estimator.update(readings)
    result=controller.update(pitch,rate,0,0,yaw,tilt,0 if valid else 1,0,False,0,0,.005,False)
    sample=driver.step(result[3:5],v[6:8],.005)
    return np.array([sample.left.torque_nm,sample.right.torque_nm]), bool(result[6])


class GpuBatch:
    def __init__(self, model, gains, tilts, parameters, device):
        self.cpu_model, self.count = model, len(gains)
        self.parameters_host = parameters
        with wp.ScopedDevice(device):
            self.model = mjw.put_model(model)
            self.data = mjw.make_data(model, nworld=self.count, nconmax=16, njmax=64)
            self.gains = wp.array(gains.astype(np.float32))
            self.initial = wp.array(tilts.astype(np.float32))
            self.parameters = wp.array(parameters)
            self.stats = wp.zeros((self.count, 8), dtype=float)
            self.diagnostic = wp.zeros((self.count, 4), dtype=float)
            self.reset(tilts)
            # Compile kernels before capture; the compilation/warm-up time is
            # excluded from steady-state throughput but reported separately.
            self.control_step(0)
            wp.synchronize()
            self.reset(tilts)
            with wp.ScopedCapture() as capture:
                self.control_step(0)
            self.graph = capture.graph
            with wp.ScopedCapture() as capture:
                self.control_step(1)
            self.tail_graph = capture.graph
            self.reset(tilts)

    def reset(self, tilts):
        mjw.reset_data(self.model, self.data)
        q, v = initial_state(self.cpu_model, self.count, tilts)
        self.data.qpos.assign(q)
        self.data.qvel.assign(v)
        self.stats.zero_()
        self.diagnostic.zero_()

    def control_step(self, tail):
        wp.launch(balance_and_motors, dim=self.count, inputs=[
            self.data.qpos, self.data.qvel, self.data.ctrl, self.gains,
            self.initial, self.parameters, self.diagnostic])
        for _ in range(5):
            mjw.step(self.model, self.data)
            wp.launch(record_metrics, dim=self.count, inputs=[
                self.data.qpos, self.data.qvel, self.initial, self.stats,
                float(self.parameters_host[0]), 0.001, tail])

    def run(self, seconds):
        steps = round(seconds/0.005)
        wp.synchronize()
        start = time.perf_counter()
        for step in range(steps):
            wp.capture_launch(self.tail_graph if (step+1)*0.005 > seconds-1 else self.graph)
        wp.synchronize()
        elapsed = time.perf_counter()-start
        return elapsed, self.data.qpos.numpy(), self.data.qvel.numpy(), self.stats.numpy(), self.diagnostic.numpy()


def cpu_run(model, gains, tilts, balance, motor, seconds):
    """Independent CPU MuJoCo + existing Python motor model for parity/timing."""
    results = []
    start = time.perf_counter()
    for gain, tilt in zip(gains, tilts):
        data = mujoco.MjData(model)
        q, _ = initial_state(model, 1, np.array([tilt]))
        data.qpos[:] = q[0]
        from shared_core import Controller, ImuEstimator
        settings=dict(balance,**dict(zip(GAIN_NAMES,gain)))
        driver,estimator,controller=MotorDriverModel(motor),ImuEstimator(),Controller(settings)
        estimator.update([-math.sin(tilt),0,math.cos(tilt),0,0,0])
        delayed=np.array([-math.sin(tilt),0,math.cos(tilt),0,0,0])
        mujoco.mj_forward(model,data)
        for _ in range(round(seconds/0.005)):
            readings=data.sensordata.copy();readings[:3]/=9.80665
            data.ctrl[:], fault=cpu_command(data.qvel,delayed,driver,estimator,controller)
            delayed=readings
            for _ in range(5):
                mujoco.mj_step(model, data)
        results.append((data.qpos.copy(), data.qvel.copy()))
    return time.perf_counter()-start, np.array([r[0] for r in results]), np.array([r[1] for r in results])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worlds", type=int, default=256)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--seed", type=int, default=20261021)
    parser.add_argument("--legacy-controller", action="store_true", help="Reproduce the historical separate Warp controller")
    parser.add_argument("--output", type=Path, default=ROOT/"outputs/gpu-first-check")
    args = parser.parse_args()
    if args.legacy_controller:
        parser.error("The legacy controller used ideal orientation/wheel feedback and does not match this robot")
    if not 1 <= args.worlds <= 4096 or not 2 <= args.seconds <= 30 or abs(args.seconds/.005-round(args.seconds/.005)) > 1e-6:
        parser.error("Need 1..4096 worlds and 2..30 seconds divisible by 0.005")
    if args.output.exists():
        parser.error("Choose a new output directory to preserve results")
    wp.init()
    if not wp.is_cuda_available():
        raise RuntimeError("No CUDA GPU detected: this benchmark will not silently use CPU physics")
    device = wp.get_device("cuda:0")
    path, balance, motor, parameters = configuration()
    from shared_sim import SharedGpuBatch
    from shared_core import LIBRARY_PATH
    model = load_model()
    if (model.nq, model.nv, model.nu) != (9, 8, 2):
        raise ValueError("Unexpected robot joint ordering")
    baseline = np.array([balance[k] for k in GAIN_NAMES])
    rng = np.random.default_rng(args.seed)
    gains = np.tile(baseline, (args.worlds, 1))
    # A reproducible screening batch, not Bayesian optimization or a global
    # optimizer. Keep the first +/-5-degree cases at the existing preset.
    gains[2:, :4] *= rng.uniform(.75, 1.25, size=(max(0, args.worlds-2), 4))
    tilts = np.radians(np.resize([5., -5., 10., -10.], args.worlds))
    args.output.mkdir(parents=True)
    snapshot = args.output/"sources"
    snapshot.mkdir()
    sources = [Path(__file__), MODEL_PATH, Path(__file__).with_name("robot_model.py"), GEOMETRY_PATH, path,
               ROOT/"ros2_ws/src/balancing_robot_sim/scripts/motor_model.py"]
    if not args.legacy_controller:
        sources += [LIBRARY_PATH, Path(__file__).with_name("shared_sim.py"),
                    ROOT/"ros2_ws/src/balancing_robot_sim/scripts/shared_core.py"]
        sources += [p for p in (ROOT/"cpp").rglob("*") if p.suffix in (".h", ".cpp")
                    and "build" not in p.relative_to(ROOT/"cpp").parts]
    hashes = {}
    for source in sources:
        content = source.read_bytes()
        relative = source.relative_to(ROOT).as_posix()
        target = snapshot/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(content)
        hashes[relative] = hashlib.sha256(content).hexdigest()
    package_versions = {name: version(name) for name in ("mujoco", "mujoco-warp", "warp-lang", "numpy")}
    (args.output/"run.json").write_text(json.dumps(dict(
        worlds=args.worlds, seconds=args.seconds, seed=args.seed, gpu_name=device.name,
        versions=package_versions, source_sha256=hashes), indent=2)+"\n")
    np.savez(args.output/"inputs.npz", gains=gains, tilts_rad=tilts, motor_parameters=parameters)
    print(f"Using {device.name}: {args.worlds} separate robot worlds", flush=True)
    compiled = time.perf_counter()
    with wp.ScopedDevice(device):
        batch = (GpuBatch(model, gains, tilts, parameters, device) if args.legacy_controller else
                 SharedGpuBatch(model, gains, tilts, parameters, device, balance))
        warmup = time.perf_counter()-compiled
        # Independently compare the ported GPU actuator against the original
        # Python MotorDriverModel over varied wheel speeds and control errors.
        probe_q, probe_v = initial_state(model, args.worlds, tilts)
        probe_q[:, 7:9] = rng.uniform(-2, 2, (args.worlds, 2))
        probe_v[:, 4] = rng.uniform(-.2, .2, args.worlds)
        probe_v[:, 6:8] = rng.uniform(-8, 8, (args.worlds, 2))
        batch.reset(tilts)
        batch.data.qpos.assign(probe_q)
        batch.data.qvel.assign(probe_v)
        batch.refresh_sensors()
        probe_readings=batch.delayed_readings.copy()
        if args.legacy_controller:
            wp.launch(balance_and_motors, dim=args.worlds, inputs=[
                batch.data.qpos, batch.data.qvel, batch.data.ctrl, batch.gains,
                batch.initial, batch.parameters, batch.diagnostic])
        else:
            batch.update_control()
        gpu_torques = batch.data.ctrl.numpy()
        from shared_core import Controller, ImuEstimator
        cpu_torques=[]
        for v,raw,gain,tilt in zip(probe_v,probe_readings,gains,tilts):
            estimator=ImuEstimator();estimator.update([-math.sin(tilt),0,math.cos(tilt),0,0,0])
            controller=Controller(dict(balance,**dict(zip(GAIN_NAMES,gain))))
            cpu_torques.append(cpu_command(v,raw,MotorDriverModel(motor),estimator,controller)[0])
        cpu_torques=np.array(cpu_torques)
        motor_error = float(np.max(abs(gpu_torques-cpu_torques)))
        motor_parity = motor_error < 1e-6
        print(f"Independent motor/controller parity: torque error={motor_error:.3g}, passed={motor_parity}", flush=True)
        batch.reset(tilts)
        # Verify a short CPU/GPU run independently before interpreting scores.
        _, gpu_q, gpu_v, _, _ = batch.run(.1)
        cpu_elapsed, cpu_q, cpu_v = cpu_run(model, gains[:4], tilts[:4], balance, motor, .1)
        q_error = float(np.max(abs(gpu_q[:len(cpu_q)]-cpu_q)))
        v_error = float(np.max(abs(gpu_v[:len(cpu_v)]-cpu_v)))
        parity = q_error < .005 and v_error < .1
        print(f"CPU/GPU 0.1 s parity: q error={q_error:.3g}, v error={v_error:.3g}, passed={parity}", flush=True)
        batch.reset(tilts)
        elapsed, q, v, stats, diagnostic = batch.run(args.seconds)
    reference_count = min(4, args.worlds)
    cpu_full_elapsed, cpu_full_q, cpu_full_v = cpu_run(
        model, gains[:reference_count], tilts[:reference_count], balance, motor, args.seconds)
    print(f"GPU batch: {elapsed:.3f} s; CPU reference ({reference_count} worlds): {cpu_full_elapsed:.3f} s", flush=True)
    finite = np.isfinite(q).all(axis=1) & np.isfinite(v).all(axis=1) & np.isfinite(stats).all(axis=1)
    feasible = (finite & (diagnostic[:, 3] == 0) & (stats[:, 0] < math.radians(20))
                & (stats[:, 4] < math.radians(1)) & (stats[:, 5] < math.radians(5)))
    score = stats[:, 1]/args.seconds/math.radians(5)**2
    score = np.where(finite, score+1000*(~feasible), 1e9)
    headers = ["world", "tilt_deg", *GAIN_NAMES, "score", "response_pass", "max_pitch_deg",
               "tail_max_pitch_deg", "tail_max_rate_deg_s", "final_distance_m", "final_speed_m_s",
               "max_average_current_a", "max_on_current_estimate_a", "faulted"]
    with (args.output/"candidates.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(headers)
        for i in range(args.worlds):
            writer.writerow([i, math.degrees(tilts[i]), *gains[i], float(score[i]), bool(feasible[i]),
                             math.degrees(stats[i, 0]), math.degrees(stats[i, 4]), math.degrees(stats[i, 5]),
                             stats[i, 6], stats[i, 7], diagnostic[i, 0], diagnostic[i, 1], bool(diagnostic[i, 3])])
    overflow = batch.data.overflow.numpy() if hasattr(batch.data, "overflow") else None
    report = dict(
        device=str(device), gpu_name=device.name, physics_on_gpu=True, worlds=args.worlds,
        controller="legacy Warp GPU kernel" if args.legacy_controller else "shared C++ on CPU",
        controller_on_gpu=args.legacy_controller,
        duration_s=args.seconds, seed=args.seed, warmup_and_compile_s=warmup,
        batch_wall_s=elapsed, physics_steps_per_s=args.worlds*round(args.seconds/.001)/elapsed,
        aggregate_realtime_factor=args.worlds*args.seconds/elapsed,
        cpu_short_check_wall_s=cpu_elapsed, cpu_gpu_parity_passed=parity,
        cpu_reference_worlds=reference_count, cpu_reference_wall_s=cpu_full_elapsed,
        cpu_reference_aggregate_realtime_factor=reference_count*args.seconds/cpu_full_elapsed,
        measured_throughput_ratio=(args.worlds/elapsed)/(reference_count/cpu_full_elapsed),
        cpu_reference_final_qpos_max_error=float(np.max(abs(q[:reference_count]-cpu_full_q))),
        cpu_reference_final_qvel_max_error=float(np.max(abs(v[:reference_count]-cpu_full_v))),
        cpu_gpu_max_qpos_error=q_error, cpu_gpu_max_qvel_error=v_error,
        motor_controller_parity_passed=motor_parity, motor_controller_max_torque_error_nm=motor_error,
        no_buffer_overflow=bool(np.all(overflow == 0)) if overflow is not None else None,
        all_states_finite=bool(finite.all()), response_pass_count=int(feasible.sum()),
        existing_preset_response_pass=feasible[:min(2, args.worlds)].tolist(),
        best_world=int(np.argmin(score)), source_sha256=hashes,
        versions=package_versions,
        scope="Nominal fixed-bus balancing only; randomized gains are individual cases, not robustly optimized presets.",
        gazebocomparison_established=False, hardware_validated=False,
        pulse_current_compliance_established=False,
    )
    report["passed"] = bool(parity and motor_parity and finite.all() and report["no_buffer_overflow"] is not False
                            and all(report["existing_preset_response_pass"]))
    (args.output/"summary.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2), flush=True)
    if not report["passed"]:
        raise RuntimeError("GPU validation failed; inspect preserved results before using this backend")


if __name__ == "__main__":
    main()

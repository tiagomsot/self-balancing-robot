"""Record and verify a finite shuttle demo using ROS and independent Gazebo poses."""

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, Float64MultiArray, String
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/balancing_robot_sim/scripts"))
from motion_core import attitude, wrap


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--distance", type=float, default=0.5)
    parser.add_argument("--speed", type=float, default=0.25)
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--output", type=Path, default=Path("outputs/shuttle-check"))
    parser.add_argument("--params-file", type=Path, help="Full ROS YAML profile to validate")
    args = parser.parse_args()
    library = Path(get_package_prefix("balancing_robot_sim")) / "lib/balancing_robot_sim"
    executed = {name: library / name for name in
                ("balance_controller", "shuttle_demo", "motion_core.py", "motor_driver", "motor_model.py")}
    for name, installed in executed.items():
        source_name = name if name.endswith(".py") else name+".py"
        source = ROOT / "ros2_ws/src/balancing_robot_sim/scripts" / source_name
        if installed.read_bytes() != source.read_bytes():
            raise RuntimeError(f"Installed {name} is stale. Run colcon build before recording.")
    if args.cycles < 1 or not 0 < args.distance <= 2 or not 0 < args.speed <= 0.25:
        parser.error("Need positive cycles, distance <=2 m and speed <=0.25 m/s")
    if args.output.exists():
        parser.error("Use a new output directory to preserve recordings")
    args.output.mkdir(parents=True)
    os.environ["GZ_PARTITION"] = f"shuttle_check_{os.getpid()}"
    os.environ["ROS_DOMAIN_ID"] = "94"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    rclpy.init()
    node = rclpy.create_node("shuttle_response_check")
    rows, motors, poses, statuses = [], [], [], []
    fields, motor_fields = [], []
    names = set()
    faulted = [False]
    latched = QoSProfile(depth=5000, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def telemetry(message):
        fields[:] = message.layout.dim[0].label.split(",")
        rows.append(dict(zip(fields, message.data)))

    def motor_callback(message):
        motor_fields[:] = message.layout.dim[0].label.split(",")
        motors.append(dict(zip(motor_fields, message.data)))

    def status_callback(message):
        report = json.loads(message.data)
        statuses.append(report)
        print(f"{report['time_s']:.2f}s {report['phase']} legs={report['legs_completed']}", flush=True)

    def read_poses(stream):
        # The SceneBroadcaster Pose_V -> TF conversion discards entity names.
        # Read its native JSON through gz topic so model identity is retained.
        decoder = json.JSONDecoder()
        buffer = ""
        for line in stream:
            buffer += line
            while buffer.strip():
                buffer = buffer.lstrip()
                try:
                    message, end = decoder.raw_decode(buffer)
                except json.JSONDecodeError:
                    break
                buffer = buffer[end:]
                stamp = message.get("header", {}).get("stamp", {})
                seconds = int(stamp.get("sec", 0))+int(stamp.get("nsec", 0))*1e-9
                for pose in message.get("pose", []):
                    name = pose.get("name", "")
                    names.add(name)
                    if name != "balancing_robot":
                        continue
                    point, q = pose.get("position", {}), pose.get("orientation", {})
                    roll, pitch, heading = attitude(q.get("x", 0), q.get("y", 0),
                                                     q.get("z", 0), q.get("w", 1))
                    poses.append(dict(time_s=seconds, x_m=point.get("x", 0), y_m=point.get("y", 0),
                                      yaw_rad=heading, roll_rad=roll, pitch_rad=pitch))

    node.create_subscription(Float64MultiArray, "/shuttle/telemetry", telemetry, latched)
    node.create_subscription(String, "/shuttle/status", status_callback, QoSProfile(
        depth=64, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Float64MultiArray, "/motors/telemetry", motor_callback, latched)
    node.create_subscription(Bool, "/balance/fault", lambda m: faulted.__setitem__(0, m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    sources = [ROOT / path for path in (
        "ros2_ws/src/balancing_robot_sim/scripts/balance_controller.py",
        "ros2_ws/src/balancing_robot_sim/scripts/shuttle_demo.py",
        "ros2_ws/src/balancing_robot_sim/scripts/motion_core.py",
        "ros2_ws/src/balancing_robot_sim/scripts/motor_driver.py",
        "ros2_ws/src/balancing_robot_sim/scripts/motor_model.py",
        "ros2_ws/src/balancing_robot_sim/config/balance.yaml",
        "ros2_ws/src/balancing_robot_sim/config/robot_geometry.json",
        "ros2_ws/src/balancing_robot_sim/launch/passive.launch.py",
        "ros2_ws/src/balancing_robot_sim/launch/balance.launch.py",
        "ros2_ws/src/balancing_robot_sim/urdf/robot.urdf.xacro",
        "ros2_ws/src/balancing_robot_sim/worlds/balance.sdf",
        "analysis/check_shuttle.py")]
    sources += list(executed.values())
    sources += [library / "librobot_core.so", library / "shared_core.py",
                ROOT / "ros2_ws/src/balancing_robot_sim/scripts/shared_core.py"]
    sources += [p for p in (ROOT / "cpp/src").rglob("*") if p.suffix in (".h", ".cpp")]
    profile = args.params_file.resolve() if args.params_file else None
    profile_hash = None
    if profile:
        content = profile.read_bytes()
        (args.output / "input-parameters.yaml").write_bytes(content)
        profile_hash = hashlib.sha256(content).hexdigest()
    hashes = {}
    for source in sources:
        relative = source.relative_to(ROOT)
        destination = args.output / "sources" / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(source.read_bytes())
        hashes[str(relative)] = hashlib.sha256(source.read_bytes()).hexdigest()
    (args.output / "run.json").write_text(json.dumps(dict(
        distance_m=args.distance, speed_m_s=args.speed, cycles=args.cycles,
        params_file=str(profile) if profile else None, params_sha256=profile_hash,
        source_sha256=hashes, hardware_validated=False), indent=2)+"\n")
    processes = []
    truth_thread = None
    error = None
    try:
        with (args.output / "launch.log").open("w") as launch_log, (args.output / "truth-bridge.log").open("w") as truth_log:
            processes.append(subprocess.Popen([
                "ros2", "launch", "balancing_robot_sim", "balance.launch.py", "gui:=false",
                "shuttle:=true", f"distance_m:={args.distance}", f"speed_m_s:={args.speed}",
                f"cycles:={args.cycles}",
                *([f"params_file:={profile}"] if profile else []),
            ], stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True))
            deadline = time.monotonic()+max(160, args.cycles*(2*args.distance/args.speed+65)*4)
            while True:
                rclpy.spin_once(node, timeout_sec=0.02)
                if statuses and truth_thread is None:
                    truth_process = subprocess.Popen([
                        "gz", "topic", "-e", "-t", "/world/balance/pose/info", "--json-output",
                    ], stdout=subprocess.PIPE, stderr=truth_log, text=True, start_new_session=True)
                    processes.append(truth_process)
                    truth_thread = threading.Thread(target=read_poses, args=(truth_process.stdout,), daemon=True)
                    truth_thread.start()
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError("A launch process exited; inspect saved logs")
                if faulted[0] or (statuses and statuses[-1]["phase"] == "FAULT"):
                    raise RuntimeError("Controller or routine faulted; inspect saved logs")
                if statuses and statuses[-1]["phase"] == "DONE" and rows and rows[-1]["time_s"] >= statuses[-1]["time_s"]+1:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError("Shuttle timed out; inspect saved logs")
    except Exception as exception:
        error = str(exception)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        if truth_thread is not None:
            truth_thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
    for name, data, headers in (("telemetry", rows, fields), ("motors", motors, motor_fields),
                                ("ground-truth", poses, list(poses[0]) if poses else [])):
        if data:
            with (args.output / f"{name}.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=headers)
                writer.writeheader()
                writer.writerows(data)
    (args.output / "statuses.json").write_text(json.dumps(statuses, indent=2)+"\n")
    final = statuses[-1] if statuses else {}
    events = final.get("events", [])
    drives = [e for e in events if e["phase"] == "DRIVE"]
    stops = [e for e in events if e["phase"] == "STOP"]
    leg_ends = [e for e in events if e["phase"] in ("TURN", "DONE")]
    turns = [e for e in events if e["phase"] == "TURN"]
    independent_legs, turn_errors = [], []
    if poses:
        stamps = np.array([p["time_s"] for p in poses])
        def nearest(stamp):
            return poses[int(np.argmin(abs(stamps-stamp)))]
        for start, stop in zip(drives, leg_ends):
            a, b = nearest(start["time_s"]), nearest(stop["time_s"])
            distance = (b["x_m"]-a["x_m"])*math.cos(a["yaw_rad"]) + (b["y_m"]-a["y_m"])*math.sin(a["yaw_rad"])
            independent_legs.append(distance)
        for turn, drive in zip(turns, drives[1:]):
            a, b = nearest(turn["time_s"]), nearest(drive["time_s"])
            turn_errors.append(math.degrees(abs(abs(wrap(b["yaw_rad"]-a["yaw_rad"]))-math.pi)))
    def total_tilt(pose):
        cosine = math.cos(pose["roll_rad"])*math.cos(pose["pitch_rad"])
        return math.degrees(math.acos(max(-1, min(1, cosine))))

    checks = dict(
        no_fault=not faulted[0] and error is None,
        completed=final.get("phase") == "DONE" and final.get("legs_completed") == 2*args.cycles,
        stopped_at_each_end=len(stops) == 2*args.cycles,
        turns_before_each_next_leg=len(turns) == 2*args.cycles-1,
        independent_pose_available=bool(poses),
        ground_truth_distances=(len(independent_legs) == 2*args.cycles and
                                all(abs(d-args.distance) < 0.05 for d in independent_legs)),
        ground_truth_half_turns=(len(turn_errors) == 2*args.cycles-1 and all(e < 5 for e in turn_errors)),
        stays_upright=max((total_tilt(p) for p in poses), default=math.inf) < 20,
        respects_wheel_request_caps=bool(motors) and max(max(abs(m["requested_left_nm"]), abs(m["requested_right_nm"])) for m in motors) <= 0.02000001,
    )
    report = dict(passed=all(checks.values()), checks=checks, error=error,
                  final_status=final, ground_truth_leg_distances_m=independent_legs,
                  ground_truth_turn_errors_deg=turn_errors, observed_pose_names=sorted(names),
                  maximum_average_motor_current_a=max((max(abs(m["current_left_a"]), abs(m["current_right_a"])) for m in motors), default=None),
                  pulse_rating_uncertain=any(m["pulse_rating_uncertain"] for m in motors),
                  hardware_rating_compliance_established=False, source_sha256=hashes)
    (args.output / "summary.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2), flush=True)
    if not report["passed"]:
        raise RuntimeError("Shuttle checks failed; recordings and failed checks are preserved")


if __name__ == "__main__":
    main()

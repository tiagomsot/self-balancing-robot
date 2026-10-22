"""Run the preset ROS/Gazebo controller headlessly and save its actual response."""

import argparse
import csv
import hashlib
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray


FIELDS = ["time_s", "pitch_rad", "rate_rad_s", "x_m", "v_m_s", "target_rad",
          "requested_nm", "commanded_nm", "estimated_pitch_rad", "estimated_tilt_rad"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tilt", type=float, default=5.0)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--output", type=Path, default=Path("outputs/gazebo-balance-check"))
    parser.add_argument("--params-file", type=Path)
    parser.add_argument("--mode", choices=["ideal", "motor_driver"])
    args = parser.parse_args()
    if not math.isfinite(args.tilt) or abs(args.tilt) > 10:
        parser.error("check tilt must be finite and within +/-10 degrees")
    if not math.isfinite(args.seconds) or args.seconds < 8:
        parser.error("check duration must be finite and at least 8 simulated seconds")
    os.environ["GZ_PARTITION"] = f"balance_validation_{os.getpid()}"
    os.environ["ROS_DOMAIN_ID"] = "93"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    args.output.mkdir(parents=True, exist_ok=True)
    share = Path(get_package_share_directory("balancing_robot_sim"))
    parameters = yaml.safe_load((share / "config/balance.yaml").read_text())
    if args.params_file:
        overrides = yaml.safe_load(args.params_file.read_text())
        for node_name, node_parameters in overrides.items():
            parameters.setdefault(node_name, {}).setdefault("ros__parameters", {}).update(
                node_parameters.get("ros__parameters", {}))
    if args.mode:
        parameters["motor_driver"]["ros__parameters"]["mode"] = args.mode
    snapshot = (args.output / "parameters.yaml").resolve()
    snapshot.write_text(yaml.safe_dump(parameters, sort_keys=False))
    library = Path(get_package_prefix("balancing_robot_sim")) / "lib/balancing_robot_sim"
    sources = {
        "controller": library / "balance_controller", "motor_driver": library / "motor_driver",
        "motor_model": library / "motor_model.py", "robot": share / "urdf/robot.urdf.xacro",
        "motion_core": library / "motion_core.py",
        "world": share / "worlds/balance.sdf",
        "balance_launch": share / "launch/balance.launch.py",
        "passive_launch": share / "launch/passive.launch.py",
        "geometry": share / "config/robot_geometry.json",
        "native_library": library / "librobot_core.so",
        "native_binding": library / "shared_core.py",
    }
    project = Path(__file__).resolve().parents[1]
    for source in (project / "cpp/src").rglob("*"):
        if source.suffix in (".h", ".cpp"):
            sources[source.relative_to(project).as_posix()] = source
    source_hashes = {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in sources.items()}
    for name, source in sources.items():
        if name.startswith("cpp/"):
            copied = args.output / "sources" / name
            copied.parent.mkdir(parents=True, exist_ok=True)
            copied.write_bytes(source.read_bytes())
    (args.output / "robot.urdf.xacro").write_bytes(sources["robot"].read_bytes())
    (args.output / "robot_geometry.json").write_bytes(sources["geometry"].read_bytes())
    (args.output / "world.sdf").write_bytes(sources["world"].read_bytes())
    rclpy.init()
    node = rclpy.create_node("balance_response_check")
    rows = []
    motors = []
    motor_fields = []
    imu_rows = []

    def record(message):
        if len(message.data) != len(FIELDS) or not all(math.isfinite(x) for x in message.data):
            raise RuntimeError("Invalid controller telemetry")
        rows.append(dict(zip(FIELDS, message.data)))

    def record_motor(message):
        fields = message.layout.dim[0].label.split(",")
        if len(fields) != len(message.data) or not all(math.isfinite(x) for x in message.data):
            raise RuntimeError("Invalid motor telemetry")
        motor_fields[:] = fields
        motors.append(dict(zip(fields, message.data)))

    def record_imu(message):
        q = message.orientation
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2*(q.x*q.x + q.y*q.y)))))
        imu_rows.append({"time_s": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                         "tilt_from_upright_deg": tilt})

    node.create_subscription(
        Float64MultiArray, "/balance/telemetry", record,
        QoSProfile(depth=4000, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(
        Float64MultiArray, "/motors/telemetry", record_motor,
        QoSProfile(depth=4000, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Imu, "/imu/data", record_imu, qos_profile_sensor_data)
    process = None
    controller_faulted = False
    try:
        with (args.output / "launch.log").open("w") as log:
            process = subprocess.Popen([
                "ros2", "launch", "balancing_robot_sim", "balance.launch.py", "gui:=false",
                f"initial_tilt_deg:={args.tilt}",
                f"params_file:={snapshot}",
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + max(40, args.seconds * 4)
            while not rows or not motors or min(rows[-1]["time_s"], motors[-1]["time_s"]) < args.seconds:
                rclpy.spin_once(node, timeout_sec=0.05)
                if process.poll() is not None:
                    raise RuntimeError(f"Launch exited early; see {args.output / 'launch.log'}")
                if time.monotonic() > deadline:
                    raise RuntimeError(f"Simulation timed out; see {args.output / 'launch.log'}")
                if "torque disabled" in (args.output / "launch.log").read_text():
                    controller_faulted = True
                    # Preserve failed responses too, including IMU motion after cutoff.
                    drain_until = time.monotonic() + 0.3
                    while time.monotonic() < drain_until:
                        rclpy.spin_once(node, timeout_sec=0.02)
                    break
        if not rows or not motors:
            (args.output / "summary.json").write_text(json.dumps({
                "passed": False, "controller_faulted": controller_faulted,
                "error": "No complete controller/actuator telemetry", "source_sha256": source_hashes,
            }, indent=2) + "\n")
            raise RuntimeError("No complete telemetry; see saved launch log")
        completed_duration = min(rows[-1]["time_s"], motors[-1]["time_s"]) >= args.seconds
        # A recorder can catch up late. Score the requested time window, not
        # the extra simulation time that elapsed while DDS queues drained.
        rows = [row for row in rows if row["time_s"] <= args.seconds + 1e-8]
        motors = [row for row in motors if row["time_s"] <= args.seconds + 1e-8]
        imu_rows = [row for row in imu_rows if row["time_s"] <= args.seconds + 1e-8]
        initial = math.degrees(rows[0]["pitch_rad"])
        tail = [row for row in rows if row["time_s"] >= args.seconds - 1]
        if not tail:
            tail = rows[-1:]
        report = {
            "initial_pitch_deg": initial,
            "maximum_abs_pitch_deg": max(abs(math.degrees(r["pitch_rad"])) for r in rows),
            "last_second_max_abs_pitch_deg": max(abs(math.degrees(r["pitch_rad"])) for r in tail),
            "last_second_max_abs_rate_deg_s": max(abs(math.degrees(r["rate_rad_s"])) for r in tail),
            "final_position_m": rows[-1]["x_m"],
            "final_velocity_m_s": rows[-1]["v_m_s"],
            "maximum_abs_commanded_total_torque_nm": max(abs(r["commanded_nm"]) for r in rows),
            "maximum_abs_delivered_total_torque_nm": max(abs(r["applied_left_nm"] + r["applied_right_nm"]) for r in motors),
            "final_time_s": rows[-1]["time_s"],
            "requested_duration_s": args.seconds,
            "received_samples": len(rows),
            "controller_faulted": controller_faulted,
            "maximum_observed_body_tilt_deg": max((r["tilt_from_upright_deg"] for r in imu_rows), default=0),
            "motor_mode": parameters["motor_driver"]["ros__parameters"]["mode"],
            "parameters_file": "parameters.yaml",
            "source_sha256": source_hashes,
        }
        motor_enabled = report["motor_mode"] == "motor_driver"
        if motor_enabled:
            report["actuator"] = {
                "maximum_average_motor_current_a": max(max(abs(r["current_left_a"]), abs(r["current_right_a"])) for r in motors),
                "maximum_on_current_estimate_a": max(max(abs(r["on_left_est_a"]), abs(r["on_right_est_a"])) for r in motors),
                "maximum_on_time_estimate_s": max(max(r.get("on_time_left_est_s", 0), r.get("on_time_right_est_s", 0)) for r in motors),
                "minimum_bus_voltage_v": min(r["bus_voltage_v"] for r in motors),
                "maximum_driver_loss_w": max(r["driver_loss_w"] for r in motors),
                "maximum_steady_junction_estimate_c": max(r["steady_junction_est_c"] for r in motors),
                "steady_thermal_proxy_exceeds_rating": any(
                    r["steady_junction_est_c"] > parameters["motor_driver"]["ros__parameters"]["driver_max_junction_c"] for r in motors),
                "average_current_rating_exceeded": any(r["average_current_exceeded"] for r in motors),
                "pulse_rating_uncertain": any(r["pulse_rating_uncertain"] for r in motors),
                "peak_current_estimate_exceeded": any(r["peak_current_est_exceeded"] for r in motors),
                "supply_out_of_range": any(r["supply_out_of_range"] for r in motors),
                "pwm_saturated": any(r["pwm_saturated"] for r in motors),
                "rms_current_proxy_exceeds_continuous_rating": any(
                    max(r["rms_left_est_a"], r["rms_right_est_a"]) > parameters["motor_driver"]["ros__parameters"]["driver_current_rating_a"] for r in motors),
            }
        checks = {
            "completed_duration": completed_duration,
            "no_controller_fault": not controller_faulted,
            "initial_condition": abs(initial - args.tilt) < 0.5,
            "stays_upright": report["maximum_abs_pitch_deg"] < 20,
            "settles_pitch": report["last_second_max_abs_pitch_deg"] < 1,
            "settles_rate": report["last_second_max_abs_rate_deg_s"] < 5,
            "returns_near_start": abs(report["final_position_m"]) < 0.1,
            "slows_down": abs(report["final_velocity_m_s"]) < 0.05,
            "respects_command_limit": report["maximum_abs_commanded_total_torque_nm"] <= parameters["balance_controller"]["ros__parameters"]["total_torque_limit_nm"] + 1e-8,
        }
        # Drift remains visible in the report, without asserting an unobservable
        # position/speed hold for the encoder-free hardware.
        report["feedback_mode"] = "imu_only"
        report["wheel_feedback_used"] = False
        report["drift_diagnostics"] = {key: checks.pop(key) for key in ("returns_near_start", "slows_down")}
        report.update(checks=checks, passed=all(checks.values()))
        report["hardware_rating_compliance_established"] = False
        report["scope"] = "IMU-only response: accelerometer/gyro plus the shared Nano filter; wheel state and true attitude are evaluation-only. Nominal calibrated sensors; bias/noise/internal DLPF and real actuator calibration remain unmeasured."
        (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
        with (args.output / "telemetry.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        with (args.output / "motors.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=motor_fields)
            writer.writeheader()
            writer.writerows(motors)
        if imu_rows:
            with (args.output / "imu.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(imu_rows[0]))
                writer.writeheader()
                writer.writerows(imu_rows)
        print(json.dumps(report, indent=2))
        if not report["passed"]:
            raise RuntimeError("Balance response checks failed")
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

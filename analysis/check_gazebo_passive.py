"""Check the paused passive launch using real bridged ROS measurements.

Run in sourced Ubuntu/ROS with the same GZ_PARTITION and ROS_DOMAIN_ID as the
launch. This advances the paused world by two simulated seconds. No motor
commands are sent. See docs/ros-first-model.md for the complete procedure.
"""

import argparse
import csv
import json
import math
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState


def message_time(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/gazebo-passive-check"))
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("passive_model_check")
    samples = []
    joints = {}
    sim_time = [0.0]

    def imu_callback(message):
        q = message.orientation
        norm_squared = q.x**2 + q.y**2 + q.z**2 + q.w**2
        if not math.isfinite(norm_squared) or abs(norm_squared - 1) > 1e-3:
            raise RuntimeError("IMU orientation is not a finite unit quaternion")
        # Angle between the body's +z axis and world +z; stays valid past 90 deg.
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2*(q.x*q.x + q.y*q.y)))))
        samples.append({
            "time_s": message_time(message.header.stamp),
            "tilt_from_upright_deg": tilt,
            "pitch_rate_rad_s": message.angular_velocity.y,
            "frame_id": message.header.frame_id,
        })

    def joint_callback(message):
        joints.update(zip(message.name, message.position))

    def clock_callback(message):
        sim_time[0] = message_time(message.clock)

    subscriptions = [
        node.create_subscription(Imu, "/imu/data", imu_callback, qos_profile_sensor_data),
        node.create_subscription(JointState, "/joint_states", joint_callback, qos_profile_sensor_data),
        node.create_subscription(Clock, "/clock", clock_callback, qos_profile_sensor_data),
    ]

    def wait_until(condition, description, timeout=15):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() >= deadline:
                raise RuntimeError(f"Timed out waiting for {description}; check launch and environment")
            rclpy.spin_once(node, timeout_sec=0.05)

    def step(iterations):
        command = [
            "gz", "service", "-s", "/world/balance/control",
            "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
            "--timeout", "5000", "--req", f"pause: true multi_step: {iterations}",
        ]
        response = subprocess.run(command, capture_output=True, text=True, timeout=10, check=True)
        if "data: true" not in response.stdout:
            raise RuntimeError(f"Gazebo rejected step request: {response.stdout} {response.stderr}")

    try:
        wait_until(lambda: node.count_publishers("/imu/data") > 0, "IMU bridge discovery")
        # The launch must already report successful robot creation and be paused.
        step(20)
        wait_until(lambda: samples and sim_time[0] >= 0.019 and len(joints) == 2,
                   "initial IMU, joint states, and simulation clock")
        initial_tilt = samples[0]["tilt_from_upright_deg"]
        if not 3.0 <= initial_tilt <= 8.0:
            raise RuntimeError(f"Expected a fresh 5-degree passive launch; initial tilt was {initial_tilt:.3f}")
        step(1980)
        wait_until(lambda: samples[-1]["time_s"] >= 1.995 - 1e-8 and sim_time[0] >= 2 - 1e-8,
                   "two simulated seconds")
        max_tilt = max(sample["tilt_from_upright_deg"] for sample in samples)
        if max_tilt < 45:
            raise RuntimeError(f"Unpowered robot did not fall; maximum observed tilt {max_tilt:.3f}")
        if set(joints) != {"left_wheel_joint", "right_wheel_joint"}:
            raise RuntimeError(f"Unexpected wheel joints: {list(joints)}")
        if not all(math.isfinite(value) for value in joints.values()):
            raise RuntimeError("Nonfinite wheel position")
        if any(not math.isfinite(s["pitch_rate_rad_s"]) for s in samples):
            raise RuntimeError("Nonfinite pitch rate")
        report = {
            "passed": True,
            "initial_tilt_deg": initial_tilt,
            "maximum_tilt_from_upright_deg": max_tilt,
            "final_tilt_from_upright_deg": samples[-1]["tilt_from_upright_deg"],
            "final_imu_time_s": samples[-1]["time_s"],
            "final_clock_time_s": sim_time[0],
            "received_imu_samples": len(samples),
            "imu_frame_ids": sorted({s["frame_id"] for s in samples}),
            "wheel_positions_rad": joints,
            "scope": "Unpowered fall and ROS sensor delivery; not a balancing or hardware test.",
        }
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
        with (args.output / "imu.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
            writer.writeheader()
            writer.writerows(samples)
        print(json.dumps(report, indent=2))
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
